# ROS-JEV: design and roadmap

## Goal

The goal is **a core Jev + LLM decision framework for ROS 2 (`rosjev`)**, with a drone swarm as its reference example.
- **System 1:** TypeSafe Jev, as ROS services, gives calibrated typed decisions.
- **System 2:** Claude LLMs, as ROS actions, augment those decisions.
- **The framework** handles fusion, safety masking, sampling, commitment, hierarchy, logging and offline optimisation.

The framework is built first, through that lens, and verified on its own. The drone swarm is built **at the very end**, purely as an application of it.

| Doc | Contents |
|---|---|
| [RESEARCH.md](RESEARCH.md) | prior art, Jev and Anthropic APIs, Gazebo and k8s findings |
| [FRAMEWORK.md](FRAMEWORK.md) | **the core spec:** concepts, interfaces, catalog, runtime, fusion, LLM contract, hierarchy, safety, optimisation toolkit, deployment |
| [examples/drone_swarm/SYSTEM_MODEL.md](examples/drone_swarm/SYSTEM_MODEL.md) | example world: mission, environment, vehicle, sensors, action spaces, objectives, constraints, ROS graph |
| [examples/drone_swarm/DECISION_MODEL.md](examples/drone_swarm/DECISION_MODEL.md) | the example's rosjev configuration: encoders, question catalog, masks, triggers, allocator |

## Principles

1. **The framework knows nothing about drones.**
   - Every domain concept enters through the catalog (YAML) or the three agent hooks: `encode_state`, `mask`, `on_decision`.
   - Anything the drone example needs that is generic gets added to the framework first.
2. **System 1 decides, System 2 always augments.** Confidence-weighted log-linear pooling, masks, shared-seed temperature sampling, act-then-revise (FRAMEWORK §5–6).
3. **Deterministic code owns numbers and safety.** Models choose among declared classes; masks are absolute.
4. **One ROS node = one container = one service.** One image; Fast DDS Discovery Server; both compose and k8s, from the first runnable phase onward.
5. **Each phase is standalone and verifiable.**
   - Each phase has its own verification command.
   - Mock backends mean nothing needs API keys until a phase explicitly verifies live inference.
6. **Python and C++ are equal citizens.**
   - The core logic is mirrored in both languages and pinned by shared golden vectors.
   - The SDK clients live in Python nodes, which C++ reaches over ROS interfaces.

## Repo layout

```
Dockerfile                    stages: base (ROS + Gazebo + noVNC) · rosjev (built workspace)
docker-compose.yml            dev shell (ros2) + profiles per phase
deploy/compose/rosjev.yml     framework base (x-rosjev-node, discovery, decision_logger)
k8s/base, k8s/components/     kustomize: discovery, logger, secrets · jev, llm, agent templates
src/rosjev_interfaces/        Question, Distribution, JevDecide, Reason, Report, Directive, DecisionRecord, Label
src/rosjev_core/              pure-Python decision engine + golden vectors
src/rosjev_cpp/               C++ engine (parity) + clients + rosjev::Agent
src/rosjev_jev/               Jev service node + backends
src/rosjev_llm/               LLM action node + backends
src/rosjev_agent/             Python Agent runtime
src/rosjev_tools/             decision_logger, rosjev-calibrate, rosjev-replay, rosjev-search
src/rosjev_examples/          toy_patrol (Python + C++ agents, supervisor), no Gazebo
examples/drone_swarm/         ← Part B only: swarm_interfaces, swarm_sim, swarm_control, swarm_ai, k8s overlay
docs/
```

## Roadmap

### Part A: the rosjev framework

| # | Feature | Verify | Pairing point |
|---|---|---|---|
| A1 | `rosjev_interfaces` + workspace + `rosjev` image stage | `ros2 interface show rosjev_interfaces/srv/JevDecide` in the built image | review the message fields |
| A2 | **`rosjev_core`**: catalog loader and validator, fusion, shared-seed inverse-CDF sampler, masks, triggers, token bucket, commitment machine, prompt/schema builder, parameter clamping. Golden vectors | `pytest src/rosjev_core` (includes `fusion_01` = FRAMEWORK §6.2) | **you write the β(c) schedule and the keep/revise rule** |
| A3 | **`rosjev_jev`** node: backends `mock`, `typesafe`, `openrouter`, `replay` + compose service + k8s component | `ros2 service call /jev/decide …` (mock); the same call live with `TYPESAFE_API_KEY` | the mock backend's realism |
| A4 | **`rosjev_llm`** node: backends `mock`, `anthropic` (streaming, json_schema, effort, prompt caching), `replay` + compose + k8s | `ros2 action send_goal --feedback /llm/reason …` (mock); live with `ANTHROPIC_API_KEY` | augmentation system-prompt template |
| A5 | **`rosjev_agent`** Python runtime + `decision_logger` + `toy_patrol` single Python agent | `docker compose --profile toy up`: records stream to JSONL; stop the llm container → records show β = 0 fallback; restart → augmentation resumes | toy catalog design |
| A6 | **`rosjev_cpp`**: C++ engine (golden-vector parity), Jev/LLM clients, `rosjev::Agent`; C++ toy agent | `colcon test --packages-select rosjev_cpp` (gtest parity); the C++ agent runs in the toy profile | C++ API ergonomics |
| A7 | **Hierarchy**: Report/Directive, supervisor agents, per-item sets, batched augmentation, directive-epoch shared seeds; toy supervisor + 3 workers | the supervisor issues a directive; the workers' records share `u`; per-item requests are visible in the logs | supervisor question design |
| A8 | **Optimisation toolkit**: `rosjev-calibrate`, `rosjev-replay`, `rosjev-search` + toy `EpisodeRunner` + toy `Label`s | calibrate on toy runs → a policy patch; replay with a permuted class order → a bias report; search over 2 configs × 3 seeds → a J table | which metrics to report |
| A9 | **k8s deployment kit**: kustomize base + components; toy app on Docker Desktop k8s; scale workers | `kubectl apply -k k8s/overlays/toy`; `kubectl scale statefulset toy-worker --replicas 5` → 5 agents in the records | resource limits |

### Part B: drone swarm reference example (very end)

| # | Feature | Verify |
|---|---|---|
| B1 | Sim core: world generator (AO, base, obstacles, NFZ, N X3 drones with IMU/altimeter/GPS), bridge, sim pod | `ros2 topic echo /uav1/odom` from another container; `cmd_vel` → climb in noVNC |
| B2 | `environment`, `sensor_sim`, `comms_sim` (C++/Py): seeded targets, battery, proximity, detector, link | fly over a target → `/uav1/detections` shows its class scores |
| B3 | `flight_controller` (C++): Task primitives + safety H1–H8 | SURVEY goal → lawnmower pattern; GOTO into an NFZ → deflected |
| B4 | `formation_planner` + allocator (C++): Hungarian, 5 formations, relay point | gtest; `Formation` goal → V formation in Gazebo |
| B5 | `drone_agent` on rosjev: encoders, catalog W1–W7, masks | uav1 alone finds and inspects a target; its records show fusion |
| B6 | `mission_coordinator` on rosjev: Sonnet plan, FSM, C1–C8, track registry | `Mission` goal runs end to end with 4 drones |
| B7 | `mission_scorer` + `EpisodeRunner`; P0–P4 ablations via `rosjev-search` | J table over seeds |
| B8 | k8s overlay + scale | `kubectl scale … --replicas 6` → 6 drones fly |

## Decisions log

| Date | Decision |
|---|---|
| 2026-10-10 | Mission scenario: search and rescue (example) |
| 2026-10-10 | Augmentation timing: act-then-revise |
| 2026-10-10 | Fusion: log-linear pool + temperature |
| 2026-10-10 | LLM cadence: event-triggered + rate cap |
| 2026-10-10 | Coordinator Sonnet effort: `medium` for both augmentation and planning; drone Haiku stays `low` |
| 2026-10-10 | Class order: status quo first |
| 2026-10-10 | LLM may propose bounded parameters (`llm_params`, clamped) |
| 2026-10-10 | Sampling: one shared seed per group (inverse-CDF on a common u) |
| 2026-10-10 | Relay placement: included in v1 (example) |
| 2026-10-10 | LLM cost weight in J: 10 points per $ (example) |
| 2026-10-10 | **Project goal: framework first; the drone swarm is the final example** |
