python ../main.py \
    --log-level=0 \
    --location_fix-key="rise/@v0/**/location_fix/**" \
    --heading-key="rise/@v0/**/heading_$*/**" \
    --cog-key="rise/@v0/**/course_over_ground_deg/**" \
    --sog-key="rise/@v0/**/speed_over_ground_kn/**" \
    --rot-key="rise/@v0/**/rate_of_turn_degpm/**" \
    --output-key="test/test" \
    --geojson-track=./track.geojson \
 