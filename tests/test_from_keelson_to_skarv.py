import keelson
import skarv
import zenoh

from main import TimestampedFloat, from_keelson_to_skarv


class _Payload:
    def __init__(self, data: bytes):
        self._data = data

    def to_bytes(self) -> bytes:
        return self._data


class _Sample:
    """What a zenoh subscriber hands the callback: a KeyExpr, not a str."""

    def __init__(self, key: str, data: bytes):
        self.key_expr = zenoh.KeyExpr(key)
        self.payload = _Payload(data)


def _enclosed_float(value: float) -> bytes:
    message = TimestampedFloat()
    message.value = value
    return keelson.enclose(message.SerializeToString())


def test_a_sample_with_a_key_expr_lands_under_its_subject():
    key = "rise/@v0/slipway-alpha/pubsub/yaw_rate_degps/sim"
    from_keelson_to_skarv(_Sample(key, _enclosed_float(1.5)))

    stored = skarv.get("yaw_rate_degps")
    assert stored and stored[0].value.value == 1.5


def test_speed_is_stored_under_the_keelson_0_6_name():
    key = "rise/@v0/slipway-alpha/pubsub/speed_over_ground_knots/sim"
    from_keelson_to_skarv(_Sample(key, _enclosed_float(6.0)))

    stored = skarv.get("speed_over_ground_knots")
    assert stored and stored[0].value.value == 6.0
