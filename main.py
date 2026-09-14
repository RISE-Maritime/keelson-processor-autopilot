#!/usr/bin/env python3

import sys
import time
import json
import math
import logging
import pathlib
import argparse
import threading
from contextlib import ExitStack
from typing import Tuple, List, Optional

import skarv
import skarv.middlewares
import zenoh
import keelson
import geojson
import numpy as np
import nvector as nv
from simple_pid import PID
from keelson.payloads.Primitives_pb2 import TimestampedFloat
from keelson.payloads.foxglove.LocationFix_pb2 import LocationFix
from keelson.interfaces.VehicleControl_pb2 import ControlAxisMapping
from keelson.scaffolding.liveliness import declare_liveliness

from google.protobuf.message import DecodeError

logger = logging.getLogger("keelson-processor-autopilot")

WGS84 = nv.FrameE(name="WGS84")

Trajectory = List[nv.GeoPath]


def angular_difference(angle1: float, angle2: float) -> float:
    diff = angle2 - angle1

    while diff < -np.pi:
        diff += 2 * np.pi

    while diff > np.pi:
        diff -= 2 * np.pi

    return diff


def angular_average(angles: List[float]) -> float:
    """
    Calculates average of multiple angles using sin-cosine. Input and output in radians
    """
    angles = np.array(angles)

    sin = np.sum(np.sin(angles))
    cos = np.sum(np.cos(angles))

    return np.arctan2(sin, cos)


def dead_reckon(
    start: nv.GeoPoint,
    sog: float,
    cog: float,
    heading: float,
    rot: float,
    duration: int,
) -> Tuple[nv.GeoPoint, float]:

    northing, easting = 0, 0
    cog_relative_bow = angular_difference(cog, heading)
    u = sog * math.cos(cog_relative_bow)
    v = sog * -math.sin(cog_relative_bow)

    # Whole one-second steps; the duration may arrive as a float from the CLI
    for _ in range(int(duration)):

        # Deltas in northing and easting
        delta_northing = u * math.cos(heading) - v * math.sin(heading)
        delta_easting = u * math.sin(heading) + v * math.cos(heading)

        # Take the step
        dt = 1
        heading += rot * dt
        northing += delta_northing * dt
        easting += delta_easting * dt

    azimuth = math.atan2(easting, northing)
    distance = math.sqrt(northing**2 + easting**2)

    end, _ = start.displace(distance, azimuth)

    return end, heading


def find_segment_of_interest(trajectory: Trajectory, point: nv.GeoPoint) -> nv.GeoPath:

    segments = []

    # Filter segments which are out of "scope"
    for segment in trajectory:
        if segment.on_path(segment.closest_point_on_great_circle(point)):
            segments.append(segment)

    # If we found no segments in scope, we check if we are "between" segments
    if not segments:
        for first, second in zip(trajectory[:-1], trajectory[1:]):
            if (
                first.closest_point_on_path(point) == first.point_b
                and second.closest_point_on_path(point) == second.point_a
            ):
                segments.append(second)

    # Still no luck? Include first and/or last segment if we are "outside" the trajectory
    if not segments:
        first = trajectory[0]
        last = trajectory[-1]

        if first.closest_point_on_path(point) == first.point_a:
            segments.append(first)

        if last.closest_point_on_path(point) == last.point_b:
            segments.append(last)

    # We are out of options...
    if not segments:
        raise ValueError("Couldnt find any relevant segments...")

    # Calculate cross-track errors for all relevant segments
    xtes = [segment.cross_track_distance(point) for segment in segments]

    # Use the one with absolute minimum
    idx = np.argmin(np.abs(xtes))

    return segments[idx]


def bearing_of_segment(segment: nv.GeoPath) -> float:
    _, az_a, az_b = segment.point_a.distance_and_azimuth(segment.point_b)
    return angular_average([az_a, az_b])


def from_keelson_to_skarv(sample: zenoh.Sample):
    try:
        # keelson 0.6 parses a str; zenoh hands the callback a KeyExpr
        subject = keelson.get_subject_from_pubsub_key(str(sample.key_expr))
        # Every keelson payload travels in an Envelope; decoding the envelope
        # itself as the payload type yields a message of zeroes
        _, _, payload = keelson.uncover(sample.payload.to_bytes())
        message = keelson.decode_protobuf_payload_from_type_name(
            payload, keelson.get_subject_schema(subject)
        )
    except KeyError:
        logger.exception("Subject is not well-known in %s", sample.key_expr)
        return
    except DecodeError:
        logger.exception("Failed to decode payload on key %s", sample.key_expr)
        return

    skarv.put(subject, message)


def from_geojson_linestring_to_trajectory(line: geojson.LineString) -> Trajectory:
    trajectory = []
    for pt1, pt2 in zip(line["coordinates"][:-1], line["coordinates"][1:]):
        trajectory.append(
            nv.GeoPath(
                nv.GeoPoint(*pt1[::-1], degrees=True),
                nv.GeoPoint(*pt2[::-1], degrees=True),
            )
        )

    return trajectory


def parse_output_key(key: Optional[str]) -> Optional[dict]:
    """The realm, entity, subject and source of a concrete pubsub key.

    None for anything that is not one — a wildcard, or a free-form key like
    `test/test` — which is still a valid place to publish, just not one that
    can be announced by liveliness or wired to a vessel's helm.
    """
    if not key or "*" in key or "$" in key:
        return None
    try:
        parsed = keelson.parse_pubsub_key(key)
    except Exception:  # noqa: BLE001 — any unparseable key is simply not a pubsub key
        return None
    if not parsed or not parsed.get("subject") or not parsed.get("entity_id"):
        return None
    return parsed


def build_control_mapping(
    steering: dict, throttle: Optional[dict], max_axis_age_s: float
) -> ControlAxisMapping:
    """A vehicle_control/v1 mapping from this autopilot's own output keys."""
    mapping = ControlAxisMapping()
    mapping.max_axis_age_s = max_axis_age_s
    for axis, parsed in (("steering", steering), ("throttle", throttle)):
        if parsed is None:
            continue
        mapping.axes[axis].entity_id = parsed["entity_id"]
        mapping.axes[axis].subject = parsed["subject"]
        mapping.axes[axis].source_id = parsed["source_id"]
    return mapping


def mapping_is_ours(installed: ControlAxisMapping, wanted: ControlAxisMapping) -> bool:
    """Whether every axis we want is already wired to our subject and source."""
    for axis, want in wanted.axes.items():
        if axis not in installed.axes:
            return False
        have = installed.axes[axis]
        if (have.subject, have.source_id) != (want.subject, want.source_id):
            return False
    return True


def keep_control_mapping(
    session: zenoh.Session,
    steering: dict,
    wanted: ControlAxisMapping,
    stop: threading.Event,
    period_s: float = 5.0,
) -> None:
    """Hold the vessel's helm mapping for as long as nobody else has it.

    Polls get_control_mapping and installs ours only when the vessel reports
    no mapping at all — after boot, or after a scenario load cleared it. A
    mapping held by someone else is left alone and logged once: an operator
    who has taken the helm is not overridden by a background process.
    """

    def rpc_key(procedure: str) -> str:
        return keelson.construct_rpc_key(
            steering["base_path"],
            steering["entity_id"],
            "vehicle_control",
            "v1",
            procedure,
            "*",
        )

    held_by_other = False
    while not stop.is_set():
        try:
            installed = None
            for reply in session.get(rpc_key("get_control_mapping"), timeout=2.0):
                if isinstance(reply.result, zenoh.Sample):
                    installed = ControlAxisMapping()
                    installed.ParseFromString(reply.result.payload.to_bytes())

            if installed is None:
                logger.info(
                    "No vehicle_control/v1 responder for %s yet", steering["entity_id"]
                )
            elif mapping_is_ours(installed, wanted):
                held_by_other = False
            elif len(installed.axes) == 0:
                replies = list(
                    session.get(
                        rpc_key("set_control_mapping"),
                        payload=wanted.SerializeToString(),
                        timeout=2.0,
                    )
                )
                if any(isinstance(r.result, zenoh.Sample) for r in replies):
                    logger.info(
                        "Installed control mapping on %s", steering["entity_id"]
                    )
                else:
                    logger.warning(
                        "set_control_mapping refused on %s: %s",
                        steering["entity_id"],
                        [r.result.payload.to_bytes() for r in replies],
                    )
            elif not held_by_other:
                held_by_other = True
                logger.warning(
                    "%s is mapped to someone else; not taking the helm",
                    steering["entity_id"],
                )
        except Exception:  # noqa: BLE001 — the keeper must outlive a bad reply
            logger.exception("Checking the control mapping failed")

        stop.wait(period_s)


# PID controller object
xte_pid = PID(setpoint=0.0, output_limits=[-50, 50], sample_time=None)
hdg_pid = PID(setpoint=0.0, output_limits=[-50, 50], sample_time=None)


def calculate_control_values(
    trajectory: Trajectory,
    location_fix: LocationFix,
    sog: TimestampedFloat,
    cog: TimestampedFloat,
    heading: TimestampedFloat,
    rot: TimestampedFloat,
    dead_reckon_duration: float,
) -> Tuple[float, float]:

    predicted_pos, predicted_hdg = dead_reckon(
        # As GeoPoint
        nv.GeoPoint(location_fix.latitude, location_fix.longitude, degrees=True),
        sog.value * 0.5144,  # To m/s
        # To radians
        math.radians(cog.value),
        # To radians
        math.radians(heading.value),
        # yaw_rate_degps is already per second; to radians/s
        math.radians(rot.value),
        dead_reckon_duration,
    )

    logger.debug("Predicted position: %s", predicted_pos.latlon_deg)
    logger.debug("Predicted heading: %s", predicted_hdg)

    segment = find_segment_of_interest(trajectory, predicted_pos)
    logger.debug("Found segment: %s", (pt.latlon_deg for pt in segment.geo_points()))

    # Calculate errors
    xte = segment.cross_track_distance(predicted_pos)
    wanted_heading = bearing_of_segment(segment)
    hdg_error = math.degrees(angular_difference(wanted_heading, predicted_hdg))
    logger.debug("Errors:")
    logger.debug("  xte: %s", xte)
    logger.debug("  hdg_error: %s", hdg_error)

    # Feed into PIDs
    xte_pid_output = xte_pid(xte)
    hdg_pid_output = hdg_pid(hdg_error)
    logger.debug("PID outputs:")
    logger.debug("  xte_pid: %s", xte_pid_output)
    logger.debug("  hdg_pid: %s", hdg_pid_output)

    return xte_pid_output, hdg_pid_output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog="keelson-processor-autopilot",
        description="A generic autopilot for keelson",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    parser.add_argument("--log-level", type=int, default=logging.INFO)

    parser.add_argument(
        "--mode",
        "-m",
        dest="mode",
        choices=["peer", "client"],
        type=str,
        help="The zenoh session mode.",
    )

    parser.add_argument(
        "--connect",
        action="append",
        type=str,
        help="Endpoints to connect to, in case multicast is not working. ex. tcp/localhost:7447",
    )

    # Subscribe keys
    parser.add_argument(
        "--location_fix-key",
        type=str,
        required=True,
        help="Key expression to subscribe to with subject 'location_fix'",
    )

    parser.add_argument(
        "--heading-key",
        type=str,
        required=True,
        help="Key expression to subscribe to with subjects either 'heading_true_north_deg' or 'heading_magnetic_deg'",
    )

    parser.add_argument(
        "--cog-key",
        type=str,
        required=True,
        help="Key expression to subscribe to with subject 'course_over_ground_deg'",
    )

    parser.add_argument(
        "--sog-key",
        type=str,
        required=True,
        help="Key expression to subscribe to with subject 'speed_over_ground_knots'",
    )

    parser.add_argument(
        "--rot-key",
        type=str,
        required=True,
        help="Key expression to subscribe to with subject 'yaw_rate_degps'",
    )

    # Output key
    parser.add_argument(
        "--output-key",
        type=str,
        required=True,
        help="Key on which to output wanted rudder angle in percent, the payload will be a TimestampedFloat. "
        "A full pubsub key (<realm>/@v0/<entity>/pubsub/<subject>/<source>) also declares liveliness for it",
    )

    ### Throttle ###
    parser.add_argument(
        "--throttle-pct",
        type=float,
        required=False,
        default=None,
        help="Constant throttle in percent to publish alongside every rudder order. The autopilot steers only; "
        "a vessel whose dead-man watches both axes needs a throttle order too",
    )

    parser.add_argument(
        "--throttle-output-key",
        type=str,
        required=False,
        default=None,
        help="Key on which to output --throttle-pct, the payload will be a TimestampedFloat",
    )

    ### Taking the helm ###
    parser.add_argument(
        "--install-control-mapping",
        action="store_true",
        help="Map the output keys onto the steering (and throttle) axes of the entity named in --output-key "
        "over vehicle_control/v1, and re-install the mapping whenever the vessel reports it gone",
    )

    parser.add_argument(
        "--max-axis-age-s",
        type=float,
        required=False,
        default=1.0,
        help="Staleness limit requested in the control mapping",
    )

    ### Track to follow ###
    parser.add_argument(
        "--geojson-track",
        type=pathlib.Path,
        required=True,
        help="The path at where to load a GeoJson containing a single LineString geometry that is to be used for tracking.",
    )

    ### PIDs configuration ###
    parser.add_argument(
        "--position-kp",
        type=float,
        required=False,
        default=1.0,
        help="Proportional coefficient for position error PID, relates cross-track error in meter with rudder angle in percent",
    )

    parser.add_argument(
        "--position-ki",
        type=float,
        required=False,
        default=0.01,
        help="Integrating coefficient for position error PID, relates cross-track error in meter with rudder angle in percent",
    )

    parser.add_argument(
        "--heading-kp",
        type=float,
        required=False,
        default=2.0,
        help="Proportional coefficient for heading error PID, relates error in heading (towards track bearing) with rudder angle in percent",
    )

    parser.add_argument(
        "--heading-ki",
        type=float,
        required=False,
        default=0.02,
        help="Integrating coefficient for heading error PID, relates error in heading (towards track bearing) with rudder angle in percent",
    )

    parser.add_argument(
        "--dead-reckon-duration",
        type=int,
        required=False,
        default=30,
        help="Duration to be used for predicting the future position and heading using dead reckoning",
    )

    # Parse arguments and start doing our thing
    args = parser.parse_args()

    steering_key = parse_output_key(args.output_key)
    throttle_key = parse_output_key(args.throttle_output_key)

    if (args.throttle_pct is None) != (args.throttle_output_key is None):
        parser.error("--throttle-pct and --throttle-output-key go together")
    if args.install_control_mapping and steering_key is None:
        parser.error(
            "--install-control-mapping needs --output-key to be a full pubsub key"
        )
    if (
        args.install_control_mapping
        and args.throttle_output_key
        and throttle_key is None
    ):
        parser.error(
            "--install-control-mapping needs --throttle-output-key to be a full pubsub key"
        )

    # Setup logger
    logging.basicConfig(
        format="%(asctime)s %(levelname)s %(name)s %(message)s", level=args.log_level
    )
    logging.captureWarnings(True)

    # Load geojson track to follow
    with args.geojson_track.open() as fp:
        trajectory: Trajectory = from_geojson_linestring_to_trajectory(geojson.load(fp))

    # Configure PIDs
    xte_pid.Kp = args.position_kp
    xte_pid.Ki = args.position_ki
    hdg_pid.Kp = args.heading_kp
    hdg_pid.Ki = args.heading_ki

    # Put together zenoh session configuration
    conf = zenoh.Config()

    if args.mode is not None:
        conf.insert_json5("mode", json.dumps(args.mode))
    if args.connect is not None:
        conf.insert_json5("connect/endpoints", json.dumps(args.connect))

    # Construct session
    logger.info("Opening Zenoh session...")
    with zenoh.open(conf) as session, ExitStack() as stack:

        # Announce what this process publishes, grouped per entity and source,
        # so a consumer can require it before it relies on it.
        announced = {}
        for parsed in (steering_key, throttle_key):
            if parsed is None:
                continue
            group = (parsed["base_path"], parsed["entity_id"], parsed["source_id"])
            announced.setdefault(group, []).append(parsed["subject"])
        for (base_path, entity_id, source_id), subjects in announced.items():
            stack.enter_context(
                declare_liveliness(
                    session, base_path, entity_id, source_id, pubsub_subjects=subjects
                )
            )
            logger.info(
                "Declared liveliness for %s/%s: %s", entity_id, source_id, subjects
            )

        # Throttle the PID update frequency to 5Hz
        skarv.register_middleware(
            "location_fix", skarv.middlewares.throttle(at_most_every=0.2)
        )

        # Update the PIDs when we get new LocationFix messages
        @skarv.subscribe("location_fix")
        def _(sample: skarv.Sample):

            location_fix: LocationFix = sample.value

            # Fetch other necessary data from storage
            if not (res := skarv.get("heading_$*")):
                logger.info("Found nothing in storage for 'heading_$*'")
                return

            heading: TimestampedFloat = res[0].value

            if not (res := skarv.get("course_over_ground_deg")):
                logger.info("Found nothing in storage for 'course_over_ground_deg'")
                return

            cog: TimestampedFloat = res[0].value

            if not (res := skarv.get("speed_over_ground_knots")):
                logger.info("Found nothing in storage for 'speed_over_ground_knots'")
                return

            sog: TimestampedFloat = res[0].value

            if not (res := skarv.get("yaw_rate_degps")):
                logger.info("Found nothing in storage for 'yaw_rate_degps'")
                return

            rot: TimestampedFloat = res[0].value

            control_values = calculate_control_values(
                trajectory,
                location_fix,
                sog,
                cog,
                heading,
                rot,
                args.dead_reckon_duration,
            )

            # Output to skarv
            skarv.put("control_value", sum(control_values))

        # Zenoh publishers
        publisher = session.declare_publisher(args.output_key)
        throttle_publisher = (
            session.declare_publisher(args.throttle_output_key)
            if args.throttle_output_key
            else None
        )

        def put_float(pub, value: float):
            message = TimestampedFloat()
            message.timestamp.FromNanoseconds(time.time_ns())
            message.value = value
            pub.put(keelson.enclose(message.SerializeToString()))

        # Skarv to keelson. The throttle goes out with every rudder order, so
        # both axes stay fresh together and fall silent together.
        @skarv.subscribe("control_value")
        def _(sample: skarv.Sample):
            put_float(publisher, sample.value)
            if throttle_publisher is not None:
                put_float(throttle_publisher, args.throttle_pct)

        stop = threading.Event()
        if args.install_control_mapping:
            wanted = build_control_mapping(
                steering_key, throttle_key, args.max_axis_age_s
            )
            threading.Thread(
                target=keep_control_mapping,
                args=(session, steering_key, wanted, stop),
                daemon=True,
            ).start()

        # Subscribe to data from zenoh network
        session.declare_subscriber(args.location_fix_key, from_keelson_to_skarv)
        session.declare_subscriber(args.heading_key, from_keelson_to_skarv)
        session.declare_subscriber(args.cog_key, from_keelson_to_skarv)
        session.declare_subscriber(args.sog_key, from_keelson_to_skarv)
        session.declare_subscriber(args.rot_key, from_keelson_to_skarv)

        # Data-driven, idling...
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            logger.info("Closing down on user request!")
        finally:
            stop.set()
            sys.exit(0)
