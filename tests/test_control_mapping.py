from main import (
    ControlAxisMapping,
    build_control_mapping,
    mapping_is_ours,
    parse_output_key,
)


STEERING = "rise/@v0/slipway-alpha/pubsub/wheel_position_pct/autopilot"
THROTTLE = "rise/@v0/slipway-alpha/pubsub/autopilot_throttle_pct/autopilot"


def test_parses_a_concrete_pubsub_key():
    parsed = parse_output_key(STEERING)
    assert parsed["entity_id"] == "slipway-alpha"
    assert parsed["subject"] == "wheel_position_pct"
    assert parsed["source_id"] == "autopilot"


def test_free_form_and_wildcard_keys_are_not_pubsub_keys():
    assert parse_output_key("test/test") is None
    assert parse_output_key("rise/@v0/**/wheel_position_pct/**") is None
    assert parse_output_key(None) is None


def test_mapping_wires_both_axes_to_our_keys():
    mapping = build_control_mapping(
        parse_output_key(STEERING), parse_output_key(THROTTLE), 1.0
    )
    assert mapping.max_axis_age_s == 1.0
    assert mapping.axes["steering"].subject == "wheel_position_pct"
    assert mapping.axes["throttle"].subject == "autopilot_throttle_pct"
    assert mapping.axes["steering"].source_id == "autopilot"


def test_steering_only_mapping_has_no_throttle_axis():
    mapping = build_control_mapping(parse_output_key(STEERING), None, 1.0)
    assert list(mapping.axes.keys()) == ["steering"]


def test_an_empty_mapping_is_not_ours():
    wanted = build_control_mapping(parse_output_key(STEERING), None, 1.0)
    assert not mapping_is_ours(ControlAxisMapping(), wanted)


def test_someone_elses_mapping_is_not_ours():
    wanted = build_control_mapping(parse_output_key(STEERING), None, 1.0)
    other = ControlAxisMapping()
    other.axes["steering"].subject = "joystick_x_pct"
    other.axes["steering"].source_id = "hc"
    assert not mapping_is_ours(other, wanted)


def test_our_installed_mapping_is_ours():
    wanted = build_control_mapping(
        parse_output_key(STEERING), parse_output_key(THROTTLE), 1.0
    )
    installed = ControlAxisMapping()
    installed.CopyFrom(wanted)
    assert mapping_is_ours(installed, wanted)
