#!/usr/bin/env python3

import sys
import time
import json
import math
import logging
import pathlib
import argparse
from typing import Tuple, List

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

    for _ in range(duration):

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
        subject = keelson.get_subject_from_pubsub_key(sample.key_expr)
        message = keelson.decode_protobuf_payload_from_type_name(
            sample.payload.to_bytes(), keelson.get_subject_schema(subject)
        )
    except KeyError:
        logger.exception("Subject is not well-known in %s", sample.key_expr)
        return
    except DecodeError:
        logger.exception("Failed to decode payload on key %s", sample.key_expr)

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
        # To radians/s
        math.radians(rot.value) / 60,
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
        help="Key expression to subscribe to with subject 'speed_over_ground_kn'",
    )

    parser.add_argument(
        "--rot-key",
        type=str,
        required=True,
        help="Key expression to subscribe to with subject 'rate_of_turn_degpm'",
    )

    # Output key
    parser.add_argument(
        "--output-key",
        type=str,
        required=True,
        help="Key expression on which to output wanted rudder angle in percent, the payload will be a TimestampedFloat'",
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
        type=float,
        required=False,
        default=30,
        help="Duration to be used for predicting the future position and heading using dead reckoning",
    )

    # Parse arguments and start doing our thing
    args = parser.parse_args()

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
    with zenoh.open(conf) as session:

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

            if not (res := skarv.get("speed_over_ground_kn")):
                logger.info("Found nothing in storage for 'speed_over_ground_kn'")
                return

            sog: TimestampedFloat = res[0].value

            if not (res := skarv.get("rate_of_turn_degpm")):
                logger.info("Found nothing in storage for 'rate_of_turn_degpm'")
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

        # Zenoh publisher
        publisher = session.declare_publisher(args.output_key)

        # Skarv to keelson
        @skarv.subscribe("control_value")
        def _(sample: skarv.Sample):

            message = TimestampedFloat()
            message.timestamp.FromNanoseconds(time.time_ns())
            message.value = sample.value

            envelope = keelson.enclose(message.SerializeToString())

            publisher.put(envelope)

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
            sys.exit(0)
