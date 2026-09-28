import nvector as nv

from main import dead_reckon


def test_a_float_duration_reckons_like_the_whole_seconds_it_names():
    """--dead-reckon-duration arrives from the command line; 10.0 must work."""
    start = nv.GeoPoint(57.686, 11.82, degrees=True)

    from_float, heading_float = dead_reckon(start, 3.0, 0.0, 0.0, 0.01, 10.0)
    from_int, heading_int = dead_reckon(start, 3.0, 0.0, 0.0, 0.01, 10)

    assert from_float.latlon_deg == from_int.latlon_deg
    assert heading_float == heading_int
