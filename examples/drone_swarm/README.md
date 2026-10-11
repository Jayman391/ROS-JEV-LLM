# Drone swarm: search & rescue on jev_ai (Part B)

The reference application of the framework. N Gazebo X3 quadcopters search an area for survivors,
inspect detections, report to a mission coordinator, and come home. Every decision is a jev_ai
decision: Jev answers, Claude augments, and deterministic C++ owns flight and safety.

Specs: [SYSTEM_MODEL.md](../../docs/examples/drone_swarm/SYSTEM_MODEL.md) (world, vehicle,
sensors, objectives) and [DECISION_MODEL.md](../../docs/examples/drone_swarm/DECISION_MODEL.md)
(catalogs, encoders, masks).

## Run it

In the dev container (`docker compose up -d` at the repo root, then `docker compose exec ros2 bash`):

```bash
colcon build && source install/setup.bash
ros2 launch drone_swarm swarm.launch.py gui:=true      # keyless mocks; mission auto-starts
#   Gazebo:                 http://localhost:6080/vnc.html  (run `gui-fullscreen` to maximise)
#   decisions + swarm:      http://localhost:8080
ros2 launch drone_swarm swarm.launch.py gui:=true jev_backend:=typesafe llm_backend:=anthropic
ros2 launch drone_swarm swarm.launch.py num_drones:=6 seed:=3 t_max_s:=900
```

`auto_start:=false` waits for a goal instead:
`ros2 action send_goal --feedback /coordinator/mission swarm_interfaces/action/Mission "{t_max_s: 600}"`.

Watch it from the command line: `ros2 topic echo /coordinator/phase`, `/coordinator/tracks`,
`/swarm/status`, `ros2 jev status`. The episode's score is logged by `mission_scorer` and written to
`/tmp/drone_swarm/episode_<run_id>.json`; decisions go to `/tmp/drone_swarm/decisions.jsonl`
(`ros2 jev report /tmp/drone_swarm/decisions.jsonl -o report.html`).

Microservices (one container per node):

```bash
docker build -f deploy/docker/Dockerfile -t jev-ai:humble .
docker build -f deploy/docker/Dockerfile.swarm -t jev-ai-swarm:humble .
docker compose -f deploy/compose/swarm.yaml up -d      # Gazebo :6081/vnc.html, dashboard :8080
kubectl apply -k deploy/k8s/overlays/swarm             # see the overlay for 6 drones
```

Tuning (FRAMEWORK §12): labels come from the scorer, so after a run

```bash
ros2 jev calibrate /tmp/drone_swarm/decisions.jsonl --labels /tmp/drone_swarm/decisions-labels.jsonl
ros2 jev search --runner drone_swarm.episode:SwarmEpisode --runner-arg t_max_s=300 \
    --space policy=P0,P1,P2,P3 --min-seeds 1 --max-seeds 4       # P0..P4: drone_swarm/episode.py
```

## What runs

| Node | Package | Role |
|---|---|---|
| Gazebo Fortress + `ros_gz_bridge` | `drone_swarm` `sim.launch.py` | the world generated for (N, seed): AO, sectors, obstacles, NFZs, targets, N X3s |
| `environment` | drone_swarm | hidden ground truth: targets (animals wander), wind, visibility, faults |
| `sensor_sim` ×N | swarm_control (C++) | battery (energy model), proximity scan, detector (+ local tracks), link |
| `flight_controller` ×N | swarm_control (C++) | `task` action: TAKEOFF … LAND; safety H1–H8; DroneStatus |
| `drone_agent` ×N | drone_swarm | jev_ai engine, set `step` (W1–W7) → tasks, reports |
| `comms_sim` | swarm_control (C++) | the radio: drops and delays reports/directives by link quality |
| `formation_planner` | swarm_control (C++) | Hungarian: sector allocation, formation slots |
| `mission_coordinator` | drone_swarm | jev_ai engine (C1–C8), mission FSM, Sonnet plan, track fusion, dispatch, relay |
| `mission_scorer` | drone_swarm | J (SYSTEM_MODEL §6), Labels for calibration, swarm status on the dashboard |
| `jev_server`, `llm_server` (Haiku), `/coordinator/llm_server` (Sonnet), `decision_logger`, `dashboard` | jev_ai | the framework |

**Mission:** LAUNCH (staggered takeoff) → TRANSIT (the C2 formation assembles over the base, then
translates into the AO) → SEARCH (Hungarian allocation with C7 fit; lawnmower surveys; detections →
W1 inspections and coordinator tracks → C3 verdicts → dispatch) → CONFIRM (survey stops, open
tracks are inspected) → RECOVER (regroup in formation over the base, staggered landing) → DONE.
Code owns the guards: `advance` is masked until the phase's work is done and required once its
time budget is spent; models choose inside them.

## Simplifications (ponytail)

- Obstacles are vertical cylinders; animals move in the ground truth only (their boxes stay put).
- No IMU, GPS, altimeter or magnetometer plugins: no decision reads them (ground-truth odometry, §3).
- Wind affects the energy model and the agents' state, not the Gazebo physics.
- Detections, link and battery are simulated from ground truth (no GPU camera).
- Task actions bypass `comms_sim`; the coordinator won't dispatch to a drone with q < 0.2.
- W2/W3 are always asked (no conditional questions in jev_ai); ignored when there is no detection.
- Pads are 8 m apart (2·d_sep), not 3 m, so parked neighbours never trip the separation reflex.
- Humble + Fortress only (the generated world uses the `ignition-gazebo` plugin names).
