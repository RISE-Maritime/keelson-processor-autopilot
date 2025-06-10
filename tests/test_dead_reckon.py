import nvector as nv

from main import dead_reckon

import pytest


def test_straight_ahead():
    start = nv.GeoPoint(0, 0)

    pred, _ = dead_reckon(start, 10, 0, 0, 0, 10)

    assert start.distance_and_azimuth(pred)[0] == pytest.approx(10 * 10)


def test_rotation():
    start = nv.GeoPoint(0, 0)

    _, pred = dead_reckon(start, 0, 0, 0, 10, 10)

    assert pred == 10 * 10
