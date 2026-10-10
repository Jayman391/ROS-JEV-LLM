# Design: LLM + Jev drone swarm POC (search & rescue)

| Doc | Contents |
|---|---|
| [RESEARCH.md](RESEARCH.md) | prior art, APIs, simulation and k8s findings |
| [SYSTEM_MODEL.md](SYSTEM_MODEL.md) | mission, environment, vehicle, sensors, action spaces, objectives, constraints, parameters |
| [DECISION_MODEL.md](DECISION_MODEL.md) | feature space, Jev question classes, output mapping, Jev+LLM fusion, optimization |
| this file | ROS graph, interfaces, containers, k8s, phased plan |

## Principles

1. **System 1 decides, System 2 augments.**
   - Jev (a **service**) gives a calibrated baseline for every decision.
   - The LLM (an **action**) always contributes a preference that is pooled into Jev's distribution. Its weight grows as Jev's confidence falls.
   - Sampling at temperature τ keeps some nondeterminism even when Jev is confident.
   - Decisions act on Jev immediately and are revised when the LLM answers ("act-then-revise").
   - See [DECISION_MODEL §5](DECISION_MODEL.md#5-fusion-jev-baseline--llm-augmentation).
2. **LLM proposes, controller disposes.**
   - Models choose from a fixed action vocabulary.
   - Deterministic C++ executes the choice, and enforces the hard constraints as reflexes *and* as action masks.
3. **Numbers live in code; meaning lives in the models.** Features are bucketed, interpreted strings, because Jev is weak at arithmetic.
4. **One ROS node = one container = one service.** One image, many commands. Discovery goes through the Fast DDS Discovery Server.
5. **Each phase is standalone and verifiable.**
   - Each phase adds its services to both compose and k8s.
   - The model nodes have mock backends, so everything runs without API keys.
6. **Everything is logged for offline optimisation.** Every decision emits a `DecisionRecord`, and every episode emits a score J.

## ROS graph

```
                     ┌──────────────────────── /coordinator ─────────────────────────┐
 user ─Mission.action─▶ mission_coordinator (py) ── FSM, track registry               │
                     │    ├─ JevDecide.srv ──▶ jev (py)      C1–C7                     │
                     │    ├─ Reason.action ──▶ llm (py)      Sonnet 5.5: plan/augment  │
                     │    └─ Formation.action / Allocate.srv ─▶ formation_planner (C++) │
                     └───────▲ /swarm/reports (via comms_sim) ─── orders ▼ ───────────┘
                             │                                         │
                     ┌───────┴────────────── /uavK (×N) ───────────────▼─────────────┐
                     │ drone_agent (py) ── features → Jev W1–W7 → fuse(LLM) → act     │
                     │    ├─ JevDecide.srv ──▶ jev (py)                                │
                     │    ├─ Reason.action ──▶ llm (py)      Haiku 5.5, effort low    │
                     │    └─ Task.action ────▶ flight_controller (C++) primitives +    │
                     │                           safety layer H1–H8, publishes status  │
                     │ sensor_sim (C++) ── battery, proximity, detections, link        │
                     └──────▲ odom/imu/altimeter/gps ───── cmd_vel ▼───────────────────┘
                     ┌──────┴────────────────────────────────────────────────────────┐
                     │ sim pod: Gazebo Fortress (noVNC :6080) + ros_gz_bridge         │
                     │ environment (py): scenario seed → targets, wind, faults, truth │
                     │ comms_sim (C++)   mission_scorer (py)   decision_logger (py)    │
                     └───────────── discovery (Fast DDS Discovery Server :11811) ────┘
```

**Ground-truth isolation:** only `environment`, `sensor_sim` and `mission_scorer` subscribe to `/env/ground_truth`. Agents see the world only through sensors.

## Interfaces (`swarm_interfaces`)

| Kind | Name | Purpose |
|---|---|---|
| msg | `JevQuestion`, `JevAnswer` | typed Jev question and answer (choice / score / noul) |
| msg | `Detection`, `DetectionArray` | detector output: track id, position estimate, class scores, altitude, sightings |
| msg | `LinkStatus` | link quality, latency, loss |
| msg | `DroneStatus` | mode, pose, velocity, energy margin, current task |
| msg | `DroneReport` | situation (W7), class probabilities (W2), recommendation, escalation, rationale |
| msg | `DroneOrder` | coordinator → drone: assignment / inspection / recall / formation slot |
| msg | `Track`, `TrackArray` | the coordinator's fused track registry |
| msg | `DecisionRecord` | full decision trace for offline optimisation |
| srv | `JevDecide` | state + questions → answers (all Jev nodes) |
| srv | `Allocate` | cost inputs → Hungarian assignment |
| action | `Reason` | task + context + JSON schema → streamed text → structured result (all LLM nodes) |
| action | `Task` | flight primitive (TAKEOFF, GOTO, HOLD, SURVEY, ORBIT, FORMATION_SLOT, RTB, LAND) |
| action | `Formation` | shape, spacing, centre, heading → all drones in their slots |
| action | `Mission` | natural-language objective → plan → executed mission summary |

## Containers (N = 4 → 30 services, one image `ros-jev:swarm`)

| Service | Lang | Count | Notes |
|---|---|---|---|
| discovery | — | 1 | `fastdds discovery -i 0 -p 11811` |
| sim (+ bridge) | — | 1 | Gazebo server + GUI on Xvfb/noVNC; in k8s, Gazebo and the bridge are two containers in one pod |
| environment | Python | 1 | scenario generator (seed), ground truth, moving animals, wind, faults |
| comms_sim | C++ | 1 | relays reports and orders with distance-based loss |
| mission_scorer | Python | 1 | computes J |
| decision_logger | Python | 1 | JSONL on a volume |
| flight_controller | C++ | N | |
| sensor_sim | C++ | N | |
| jev (drone) | Python | N | |
| llm (drone) | Python | N | |
| drone_agent | Python | N | |
| formation_planner | C++ | 1 | coordinator |
| jev (coordinator) | Python | 1 | coordinator |
| llm (coordinator) | Python | 1 | coordinator |
| mission_coordinator | Python | 1 | coordinator |

**Disk:** the host has about 16 GiB free, so one shared image and a slim runtime stage are mandatory.

**Kubernetes mapping:**
- Each per-drone node type is a **StatefulSet** with `replicas: N`. Pod ordinal `k` maps to namespace `uav{k+1}`, so `kubectl scale` resizes the swarm. The world generator reads the same N.
- Per-drone services become **headless** Services.
- API keys are a Secret.
- All manifests are kustomize (`kubectl apply -k k8s/`), on Docker Desktop's cluster with `imagePullPolicy: Never`.

## Repo layout

```
Dockerfile                  stages: base (ROS + Gazebo + noVNC), swarm (built workspace)
docker-compose.yml          ros2 dev shell + profile "swarm" (one service per node)
config/scenario.yaml        parameter registry defaults (SYSTEM_MODEL §8, DECISION_MODEL §8)
src/swarm_interfaces/       msgs, srvs, actions
src/swarm_sim/              world generator, bridge config, environment, scorer (py)
src/swarm_control/          flight_controller, sensor_sim, comms_sim, formation_planner (C++)
src/swarm_ai/               jev, llm, decision core, drone_agent, mission_coordinator, logger (py)
k8s/                        kustomize base + per-phase components
eval/                       episode runner, ablations, calibration notebooks
docs/
```

## Phases

Each phase ships its own code, its compose and k8s entries, and one verification command, and it works without the later phases.

| # | Feature | Verify |
|---|---|---|
| 1 | `swarm_interfaces` + `swarm` image stage + `config/scenario.yaml` | `ros2 interface show swarm_interfaces/srv/JevDecide` |
| 2 | **Sim core:** discovery, world generator (AO, base, obstacles, NFZ visuals, N X3 drones with IMU/altimeter/GPS), bridge | from a separate container: `ros2 topic echo /uav1/odom`; publish `/uav1/cmd_vel` → the drone climbs in noVNC |
| 3 | **Environment + sensors:** `environment` (seeded targets visible in Gazebo, wind, faults), `sensor_sim` (battery, proximity, detector, link), `comms_sim` | fly uav1 over a target with `cmd_vel` → `/uav1/detections` shows its class scores; the battery drains |
| 4 | **`flight_controller`:** Task primitives + safety layer H1–H8 | `ros2 action send_goal /uav1/task "{type: SURVEY …}"` → lawnmower pattern; a GOTO into an NFZ is deflected |
| 5 | **`formation_planner` + allocator:** Hungarian, the 5 formations | unit test, then `ros2 action send_goal /coordinator/formation "{shape: v}"` |
| 6 | **`jev` service:** TypeSafe SDK + mock backend | `ros2 service call /uav1/jev/decide …` returns distributions |
| 7 | **`llm` action:** Anthropic streaming + JSON-schema output + mock backend | `ros2 action send_goal --feedback /uav1/llm/reason …` |
| 8 | **Decision core** (pure Python, no ROS): feature buckets, question catalog, fusion, masks, triggers, rate limiter, act-then-revise | `pytest src/swarm_ai/test` (includes the DECISION_MODEL §5.2 worked example) |
| 9 | **`drone_agent`:** one drone autonomously surveys and inspects | uav1 alone finds and inspects a target; its `DecisionRecord`s show fusion |
| 10 | **`mission_coordinator`:** Sonnet plan, FSM, track registry, C1–C7 | `ros2 action send_goal /coordinator/mission "{objective: …}"` runs end to end |
| 11 | **Scoring + evaluation:** `mission_scorer`, `decision_logger`, episode runner, P0–P4 ablations | `eval/run.py --seeds 2 --policies P1,P4` prints a J table |
| 12 | **k8s scale & hardening:** resource limits, scale test | `kubectl scale statefulset … --replicas 6` → 6 drones fly |
