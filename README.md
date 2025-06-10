# keelson-processor-autopilot

## What?
An autopilot adhearing to be run in a keelson network.

Currently supports the following modes of operation:
* Track following

## Why? 
Good question. Next?

## How?
Utilizing two PID regulators (one for cross track error and one for heading error) for a predicted vessel state in a future time instant (using dead reckoning).

## Usage

Supplied as a docker image, which accepts the following arguments:
```bash
usage: keelson-processor-autopilot [-h] [--log-level LOG_LEVEL] [--mode {peer,client}] [--connect CONNECT] --location_fix-key
                                   LOCATION_FIX_KEY --heading-key HEADING_KEY --cog-key COG_KEY --sog-key SOG_KEY --rot-key
                                   ROT_KEY --output-key OUTPUT_KEY --geojson-track GEOJSON_TRACK [--position-kp POSITION_KP]
                                   [--position-ki POSITION_KI] [--heading-kp HEADING_KP] [--heading-ki HEADING_KI]
                                   [--dead-reckon-duration DEAD_RECKON_DURATION]

A generic autopilot for keelson

options:
  -h, --help            show this help message and exit
  --log-level LOG_LEVEL
  --mode {peer,client}, -m {peer,client}
                        The zenoh session mode. (default: None)
  --connect CONNECT     Endpoints to connect to, in case multicast is not working. ex. tcp/localhost:7447 (default: None)
  --location_fix-key LOCATION_FIX_KEY
                        Key expression to subscribe to with subject 'location_fix' (default: None)
  --heading-key HEADING_KEY
                        Key expression to subscribe to with subjects either 'heading_true_north_deg' or 'heading_magnetic_deg'
                        (default: None)
  --cog-key COG_KEY     Key expression to subscribe to with subject 'course_over_ground_deg' (default: None)
  --sog-key SOG_KEY     Key expression to subscribe to with subject 'speed_over_ground_kn' (default: None)
  --rot-key ROT_KEY     Key expression to subscribe to with subject 'rate_of_turn_degpm' (default: None)
  --output-key OUTPUT_KEY
                        Key expression on which to output wanted rudder angle in percent, the payload will be a TimestampedFloat'
                        (default: None)
  --geojson-track GEOJSON_TRACK
                        The path at where to load a GeoJson containing a single LineString geometry that is to be used for
                        tracking. (default: None)
  --position-kp POSITION_KP
                        Proportional coefficient for position error PID, relates cross-track error in meter with rudder angle in
                        percent (default: 1.0)
  --position-ki POSITION_KI
                        Integrating coefficient for position error PID, relates cross-track error in meter with rudder angle in
                        percent (default: 0.01)
  --heading-kp HEADING_KP
                        Proportional coefficient for heading error PID, relates error in heading (towards track bearing) with
                        rudder angle in percent (default: 2.0)
  --heading-ki HEADING_KI
                        Integrating coefficient for heading error PID, relates error in heading (towards track bearing) with
                        rudder angle in percent (default: 0.02)
  --dead-reckon-duration DEAD_RECKON_DURATION
                        Duration to be used for predicting the future position and heading using dead reckoning (default: 30)
```