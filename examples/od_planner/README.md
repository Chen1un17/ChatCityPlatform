# OD → Explicit Public-Transport Chains

This example shows how to convert an OD JSON file (edge→edge) into explicit person chains and run a SUMO simulation.

Toy scenario (fast):

`python3 tools/od_pt_pipeline.py --od examples/od_planner/od_toy.json --net examples/toy_pt/toy.net.xml --routes examples/toy_pt/toy_pt_routes.rou.xml --additional examples/toy_pt/toy_pt_stops.add.xml --output-dir examples/od_planner/exp_toy --run --copy-inputs`

Planning-only + validation (no simulation):

1) plan to `explicit_chains.json` + `persons.rou.xml` using TraCI intermodal routing

`python3 tools/od_to_explicit_chains.py --scenario toy --od examples/od_planner/od_toy.json --output-dir examples/od_planner/exp_toy_plan`

2) validate that ride legs can be served by the PT timetable (data-level validation)

`python3 tools/explicit_chains_validate.py --chains examples/od_planner/exp_toy_plan/explicit_chains.json --additional examples/toy_pt/toy_pt_stops.add.xml --routes examples/toy_pt/toy_pt_routes.rou.xml`

Or use the one-command wrapper:

`python3 tools/od_plan_and_validate.py --scenario toy --od examples/od_planner/od_toy.json --output-dir examples/od_planner/exp_toy_plan_validate`

Outputs:
- `examples/od_planner/exp_toy/explicit_chains.json`
- `examples/od_planner/exp_toy/persons.rou.xml`
- `examples/od_planner/exp_toy/personinfo.xml`
