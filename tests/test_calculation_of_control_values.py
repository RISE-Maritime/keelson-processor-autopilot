import geojson
import nvector as nv

import main
from main import (
    from_geojson_linestring_to_trajectory,
    calculate_control_values,
    LocationFix,
    TimestampedFloat,
)

import pytest

main.xte_pid.Kp = 1.0
main.xte_pid.Ki = 0.0
main.hdg_pid.Kp = 2.0
main.hdg_pid.Ki = 0.0


def test_left_side_diverging():

    line = """
{
        "coordinates": [
          [
            11.629417763269316,
            56.884853103967515
          ],
          [
            11.688533445143975,
            57.252419758399384
          ],
          [
            11.3670919249505,
            57.59854235532205
          ],
          [
            10.838745518195765,
            57.76052508976764
          ]
        ],
        "type": "LineString"
      }"""

    line = geojson.loads(line)

    trajectory = from_geojson_linestring_to_trajectory(line)

    location_fix = LocationFix(longitude=11.679237888955555, latitude=57.1988983900857)

    sog = TimestampedFloat(value=10.0)
    cog = TimestampedFloat(value=0.0)
    heading = TimestampedFloat(value=0.0)
    rot = TimestampedFloat(value=0.0)

    xte_pid_output, hdg_pid_output = calculate_control_values(
        trajectory, location_fix, sog, cog, heading, rot, 30
    )

    assert xte_pid_output > 0
    assert hdg_pid_output > 0


def test_left_side_converging():
    line = """
{
        "coordinates": [
          [
            11.629417763269316,
            56.884853103967515
          ],
          [
            11.688533445143975,
            57.252419758399384
          ],
          [
            11.3670919249505,
            57.59854235532205
          ],
          [
            10.838745518195765,
            57.76052508976764
          ]
        ],
        "type": "LineString"
      }"""

    line = geojson.loads(line)

    trajectory = from_geojson_linestring_to_trajectory(line)

    location_fix = LocationFix(longitude=11.679237888955555, latitude=57.1988983900857)

    sog = TimestampedFloat(value=10.0)
    cog = TimestampedFloat(value=15.0)
    heading = TimestampedFloat(value=15.0)
    rot = TimestampedFloat(value=0.0)

    xte_pid_output, hdg_pid_output = calculate_control_values(
        trajectory, location_fix, sog, cog, heading, rot, 30
    )

    assert xte_pid_output > 0
    assert hdg_pid_output < 0


def test_crossing_from_left_to_right():
    line = """
{
        "coordinates": [
          [
            11.629417763269316,
            56.884853103967515
          ],
          [
            11.688533445143975,
            57.252419758399384
          ],
          [
            11.3670919249505,
            57.59854235532205
          ],
          [
            10.838745518195765,
            57.76052508976764
          ]
        ],
        "type": "LineString"
      }"""

    line = geojson.loads(line)

    trajectory = from_geojson_linestring_to_trajectory(line)

    location_fix = LocationFix(longitude=11.679237888955555, latitude=57.1988983900857)

    sog = TimestampedFloat(value=10.0)
    cog = TimestampedFloat(value=70.0)
    heading = TimestampedFloat(value=70.0)
    rot = TimestampedFloat(value=0.0)

    xte_pid_output, hdg_pid_output = calculate_control_values(
        trajectory, location_fix, sog, cog, heading, rot, 30
    )

    assert xte_pid_output < 0
    assert hdg_pid_output < 0
