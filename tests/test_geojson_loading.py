import geojson

from main import from_geojson_linestring_to_trajectory

import pytest


def test_loading_of_geojson():

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

    assert len(trajectory) == 3
    assert trajectory[0].point_a.latitude_deg == pytest.approx(
        line["coordinates"][0][1]
    )
