# Toy public-transport scenario (SUMO)

This directory contains a tiny SUMO network plus a minimal PT schedule (a "subway" line `1` and a bus line `B42`) that matches the explicit person-chain JSON examples.

Quick run (headless):

`python3 tools/person_chain_experiment.py --plan examples/toy_pt/plan_subway1_busB42.json --net examples/toy_pt/toy.net.xml --routes examples/toy_pt/toy_pt_routes.rou.xml --additional examples/toy_pt/toy_pt_stops.add.xml --line-map examples/toy_pt/line_map_strip_prefix.json --output-dir /tmp/mg_exp_toy --copy-inputs --run`

Outputs land in `/tmp/mg_exp_toy/` (notably `personinfo.xml`, `tripinfo.xml`, `summary.xml`).

