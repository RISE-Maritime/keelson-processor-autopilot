# keelson-processor-autopilot

## What?
An autopilot adhearing to be run in a keelson network.

Currently supports the following modes of operation:
* Track following

## Why? 
Good question. Next?

## How?
Utilizing two PID regulators (one for cross track error and one for heading error) for a predicted vessel state in a future time instant (using dead reckoning).

## Inputs and outputs

Built against keelson `0.6.0rc15`. It subscribes to five subjects:

| Argument | Subject | Unit |
| --- | --- | --- |
| `--location_fix-key` | `location_fix` | |
| `--heading-key` | `heading_true_north_deg` or `heading_magnetic_deg` | deg |
| `--cog-key` | `course_over_ground_deg` | deg |
| `--sog-key` | `speed_over_ground_knots` | kn |
| `--rot-key` | `yaw_rate_degps` | deg/s |

keelson renamed the last two from `speed_over_ground_kn` and `rate_of_turn_degpm`; v0.1.0 still read the old names and never produced an order.

It publishes the wanted rudder angle in percent on `--output-key`, and, with `--throttle-pct`, a constant throttle in percent on `--throttle-output-key` alongside every rudder order.

With `--heading-setpoint-key` it also publishes, with every rudder order, keelson#316's `heading_setpoint_deg`: the heading it is steering toward, in degrees on [0, 360), as a `TimestampedFloat`. That is the bearing of the track segment the heading PID references (the segment relevant to the dead-reckoned position). The cross-track PID is summed into the rudder order, not into this reference, so an XTE correction moves the rudder and never the setpoint. Nothing is published without the argument, before the first fix, or when no segment applies. keelson `0.6.0rc15` does not yet know the subject, so it logs a "NOT well-known" warning when declaring liveliness for it; the key is published as given.

When an output key is a full pubsub key (`<realm>/@v0/<entity>/pubsub/<subject>/<source>`), the source and subject liveliness tokens are declared for it, so a consumer can require the autopilot before relying on it.

## Taking the helm

With `--install-control-mapping` the autopilot wires its outputs onto the `steering` (and `throttle`) axes of the entity in `--output-key` over `vehicle_control/v1`. It checks every 5 s and installs the mapping only when the vessel reports none, so it comes back after a vessel restart but never overrides a mapping someone else holds.

## Usage

Supplied as a docker image, which accepts the following arguments:
```bash
usage: keelson-processor-autopilot [-h] [--log-level LOG_LEVEL] [--mode {peer,client}] [--connect CONNECT] --location_fix-key
                                   LOCATION_FIX_KEY --heading-key HEADING_KEY --cog-key COG_KEY --sog-key SOG_KEY --rot-key
                                   ROT_KEY --output-key OUTPUT_KEY [--throttle-pct THROTTLE_PCT]
                                   [--throttle-output-key THROTTLE_OUTPUT_KEY]
                                   [--heading-setpoint-key HEADING_SETPOINT_KEY] [--install-control-mapping]
                                   [--max-axis-age-s MAX_AXIS_AGE_S] --geojson-track GEOJSON_TRACK
                                   [--position-kp POSITION_KP] [--position-ki POSITION_KI] [--heading-kp HEADING_KP]
                                   [--heading-ki HEADING_KI] [--dead-reckon-duration DEAD_RECKON_DURATION]
```

See `python main.py --help` for the description of each argument, and `examples/example.sh` for a local run.
