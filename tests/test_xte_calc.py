import geojson
import nvector as nv

from main import from_geojson_linestring_to_trajectory, find_segment_of_interest

import pytest


def test_finding_correct_segment():

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

    # Left side of first segment
    point = """
{
        "coordinates": [
          11.588775731980462,
          57.10823769082734
        ],
        "type": "Point"
      }"""

    point = geojson.loads(point)

    point = nv.GeoPoint(*point["coordinates"][::-1], degrees=True)

    segment = find_segment_of_interest(trajectory, point)

    assert segment is trajectory[0]

    # Right side of second segment
    point = """
{
        "coordinates": [
          11.669641089155078,
          57.48579221903424
        ],
        "type": "Point"
      }"""

    point = geojson.loads(point)

    point = nv.GeoPoint(*point["coordinates"][::-1], degrees=True)

    segment = find_segment_of_interest(trajectory, point)

    assert segment is trajectory[1]

    # Before first waypoint
    point = """
{
        "coordinates": [
          11.887839871734855,
          56.749919863467596
        ],
        "type": "Point"
      }"""

    point = geojson.loads(point)

    point = nv.GeoPoint(*point["coordinates"][::-1], degrees=True)

    segment = find_segment_of_interest(trajectory, point)

    assert segment is trajectory[0]

    # In between second and third segment
    point = """
{
        "coordinates": [
          11.371292565541069,
          57.600520584848255
        ],
        "type": "Point"
      }"""

    point = geojson.loads(point)

    point = nv.GeoPoint(*point["coordinates"][::-1], degrees=True)

    segment = find_segment_of_interest(trajectory, point)

    assert segment is trajectory[2]
