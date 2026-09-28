import math

import geojson
import keelson
import nvector as nv
import pytest

import main
from main import (
    LocationFix,
    TimestampedFloat,
    calculate_control_values,
    from_geojson_linestring_to_trajectory,
    heading_setpoint_deg,
    publish_heading_setpoint,
)


def segment_from(lat_a, lon_a, lat_b, lon_b) -> nv.GeoPath:
    return nv.GeoPath(
        nv.GeoPoint(lat_a, lon_a, degrees=True),
        nv.GeoPoint(lat_b, lon_b, degrees=True),
    )


@pytest.mark.parametrize(
    "segment, expected",
    [
        (segment_from(57.0, 11.0, 57.1, 11.0), 0.0),  # north
        (segment_from(0.0, 11.0, 0.0, 11.1), 90.0),  # east, on the equator
        (segment_from(57.1, 11.0, 57.0, 11.0), 180.0),  # south
        (segment_from(0.0, 11.1, 0.0, 11.0), 270.0),  # west: not -90
    ],
)
def test_setpoint_is_the_segment_bearing_on_0_360(segment, expected):
    setpoint = heading_setpoint_deg(segment)
    assert 0.0 <= setpoint < 360.0
    # 0 and 360 are the same heading
    diff = (setpoint - expected + 180.0) % 360.0 - 180.0
    assert diff == pytest.approx(0.0, abs=1e-6)


def test_setpoint_just_west_of_north_is_not_negative():
    setpoint = heading_setpoint_deg(segment_from(57.0, 11.0, 57.1, 10.999))
    assert 359.0 < setpoint < 360.0


def test_setpoint_is_the_heading_pid_reference_not_biased_by_xte():
    main.xte_pid.Kp, main.xte_pid.Ki = 1.0, 0.0
    main.hdg_pid.Kp, main.hdg_pid.Ki = 2.0, 0.0

    line = geojson.LineString([(11.0, 57.0), (11.0, 57.5)])  # due north
    trajectory = from_geojson_linestring_to_trajectory(line)
    zero = TimestampedFloat(value=0.0)

    # Well off the track to the east, heading north: a large XTE correction
    fix = LocationFix(latitude=57.2, longitude=11.01)
    xte_out, hdg_out, setpoint = calculate_control_values(
        trajectory, fix, TimestampedFloat(value=5.0), zero, zero, zero, 30
    )

    assert xte_out != 0.0
    assert hdg_out == pytest.approx(0.0, abs=1e-3)
    assert heading_setpoint_deg(trajectory[0]) == setpoint
    diff = (setpoint + 180.0) % 360.0 - 180.0
    assert diff == pytest.approx(0.0, abs=1e-6)


class RecordingPublisher:
    def __init__(self):
        self.payloads = []

    def put(self, payload):
        self.payloads.append(payload)


def test_nothing_published_without_a_setpoint_key():
    assert publish_heading_setpoint(None, 123.0) is False


def test_setpoint_published_as_timestamped_float():
    publisher = RecordingPublisher()
    assert publish_heading_setpoint(publisher, 123.5) is True
    assert len(publisher.payloads) == 1

    _, _, payload = keelson.uncover(publisher.payloads[0])
    message = TimestampedFloat.FromString(payload)
    assert message.value == pytest.approx(123.5)
    assert message.timestamp.ToNanoseconds() > 0
    assert not math.isnan(message.value)
