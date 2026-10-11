# Drone swarm example: system model (sensor–robot–environment)

This is the physical and mission model for the **reference application** of the jev_ai framework. Every number here is a **parameter**: section 8 collects them, with defaults and ranges.

- [DECISION_MODEL.md](DECISION_MODEL.md) is the swarm's jev_ai catalog, encoders and masks.
- [FRAMEWORK.md](../../FRAMEWORK.md) is the framework itself.
- [DESIGN.md](../../DESIGN.md) is the roadmap. This example is built last.

Status: **implemented** (B1–B8, `examples/drone_swarm/`). Where the build deliberately differs
(cylinder obstacles, no IMU/GPS/altimeter plugins, wind outside the physics, 8 m pads), see the
[README's simplifications](../../../examples/drone_swarm/README.md#simplifications-ponytail).
VERIFY items: the X3 twist is body-frame (the controller holds yaw at 0 and rotates by yaw); no
WindEffects (wind drives the energy model and the agents' state).

---

## 1. Mission: search & rescue survey

A swarm of N quadcopters (default N = 4) launches from a base pad. It must:

1. Search an area of operations (AO) for ground targets.
2. Classify each target. Only survivors matter; vehicles, debris and animals are distractors.
3. Confirm survivors through low-altitude inspection.
4. Report confirmed survivor positions to the coordinator.
5. Recover every drone to base before the mission clock or its battery runs out.

**Mission phases** (the coordinator owns the transitions):

```
LAUNCH ─▶ TRANSIT ─▶ SEARCH ─▶ CONFIRM ─▶ RECOVER ─▶ DONE
   └──────────┴─────────┴──────────┴──────▶ ABORT (recall all)
```

| Phase | Entered when | Swarm behaviour |
|---|---|---|
| LAUNCH | mission accepted | staggered takeoff to the transit altitude |
| TRANSIT | all airborne | fly in formation from base to the AO edge |
| SEARCH | formation at the AO | each drone surveys its assigned sector (lawnmower pattern); detections trigger local inspections |
| CONFIRM | coverage ≥ target **or** time ≥ 70% of T_max | survey stops; drones are dispatched to unresolved tracks |
| RECOVER | all tracks resolved **or** time ≥ 85% of T_max | return in formation, then staggered landing |
| ABORT | coordinator decision or a safety trigger | recall all drones immediately |

SEARCH and CONFIRM can overlap: a drone that finishes its sector early can be sent to inspect while others still survey.

## 2. Environment

**World frame:** ENU, origin at the base pad, metres. Ground is flat at z = 0.

| Element | Model | Rendered in Gazebo |
|---|---|---|
| AO | rectangle x ∈ [20, 320], y ∈ [−150, 150] (300 × 300 m) | ground texture/outline |
| Sectors | the AO split into N equal vertical strips (generated per N) | thin coloured lines |
| Coverage grid | 10 × 10 m cells (900 cells); a cell is *covered* once it is seen at survey quality (§4.6) | not rendered |
| Base | pad at (0, 0), N landing spots 3 m apart | pad model |
| Obstacles | 6 static structures: boxes or cylinders, 10–40 m tall, fixed per world | grey boxes/cylinders |
| No-fly zones (NFZ) | 2 vertical cylinders, radius 15–25 m, unlimited height | translucent red cylinders (visual only, no collision) |
| Targets | 10 ground objects with a hidden true class, randomised per episode | coloured primitives: survivor = orange, vehicle = blue, debris = brown, animal = green |
| Wind | constant vector (speed W, direction θ_w) + Gaussian gusts σ_g | Gazebo WindEffects system (VERIFY phase 2; if missing, wind affects only the energy and disturbance model) |
| Visibility | scalar v_vis ∈ [0.5, 1] (haze); scales detection probability | none |
| Comms | base station at the origin, range R_c; link quality decays with distance (§4.8) | none |
| Mission clock | T_max = 10 min of sim time | none |

**Target mix (per episode):**

| Class | Count | Notes |
|---|---|---|
| survivor | 3–5 | positive class; 1–2 sit close to a distractor |
| vehicle | 1–2 | |
| debris | 2–3 | |
| animal | 1–2 | moves slowly (0.3 m/s random walk) |
| *spurious* | — | not a physical object; the detector's false positives (§4.6) |

**Episode randomisation** (for evaluation, [FRAMEWORK §12](../../FRAMEWORK.md#12-observability-and-optimisation-toolkit-jev_ai_tools)). An episode seed sets:
- target positions and the class mix
- wind (W ∈ [0, 8] m/s, any direction)
- visibility
- 0–2 injected faults (§5.4)

The obstacle and NFZ layout is fixed per world file.

## 3. Vehicle

The vehicle is the Gazebo X3 quadcopter, driven by `MulticopterVelocityControl`.

| Property | Value | Notes |
|---|---|---|
| Command interface | twist (vx, vy, vz, yaw_rate) on `/<ns>/gazebo/command/twist` | VERIFY phase 2: body frame or world frame; the controller rotates by yaw if needed |
| Max horizontal speed | 5 m/s | survey 4 m/s, transit 5 m/s, inspection 2 m/s |
| Max climb / descent | 2 / 1.5 m/s | |
| Max acceleration | 2 m/s² | enforced by the controller's rate limiter |
| Max yaw rate | 1 rad/s | |
| Altitude band | 5–60 m AGL | survey 30 m, transit 40 m, inspection 10 m |
| State estimate | ground-truth odometry, 50 Hz | ponytail: perfect state; add an EKF over IMU+GPS only if GPS faults need to matter for control |

### 3.1 Energy model (simulated battery)

Gazebo's X3 has no battery, so `battery_sim` (C++) integrates power from odometry:

```
P = P_hover + k_h·v_h² + k_c·max(v_z, 0) + k_w·|v_air|²       [W]
v_air = v_ground − wind
E(t)  = E_0 − ∫ P dt                                           [Wh]
```

| Parameter | Default | Gives |
|---|---|---|
| E_0 | 40 Wh | about 13 min of hover, or about 11 min while surveying. Surveying alone takes about 4 min of the 10-minute mission, so both energy and time constrain it |
| P_hover | 180 W | |
| k_h | 2.0 W/(m/s)² | +50 W at 5 m/s |
| k_c | 25 W/(m/s) | |
| k_w | 1.5 W/(m/s)² | headwind cost |

**Energy needed to get home** (computed deterministically, never by a model):
`E_rtb = (d_home / v_transit)·P(v_transit, headwind) + E_land`, plus a safety reserve **R = 10% E_0**.
The **energy margin** feature is `m_E = (E − E_rtb − R) / E_0`.

## 4. Sensors (ROS topics, per drone `/uavK`)

All sensors are simulated. "gz" means a Gazebo system plugin bridged to ROS; "sim" means a C++ node that computes the reading from ground truth plus a noise model, for sensors that would need a GPU or that Gazebo doesn't have.

| # | Sensor | Source | Topic | Type | Rate | Noise / model |
|---|---|---|---|---|---|---|
| 4.1 | Odometry | gz OdometryPublisher | `/uavK/odom` | nav_msgs/Odometry | 50 Hz | ground truth |
| 4.2 | IMU | gz Imu | `/uavK/imu` | sensor_msgs/Imu | 100 Hz | SDF Gaussian noise: gyro σ 0.009 rad/s, accel σ 0.02 m/s² |
| 4.3 | Altimeter | gz Altimeter | `/uavK/altimeter` | ros_gz_interfaces/Altimeter | 20 Hz | σ 0.1 m |
| 4.4 | GPS | gz NavSat | `/uavK/gps` | sensor_msgs/NavSatFix | 5 Hz | σ 1.5 m horizontal, 3 m vertical; ×5 under a GPS fault |
| 4.5 | Proximity | sim | `/uavK/proximity` | sensor_msgs/LaserScan | 10 Hz | 16 horizontal beams, 20 m range, σ 0.1 m; ray-cast against known obstacle geometry (no GPU lidar) |
| 4.6 | Target detector | sim | `/uavK/detections` | swarm_interfaces/DetectionArray | 2 Hz | see below |
| 4.7 | Battery | sim | `/uavK/battery` | sensor_msgs/BatteryState | 2 Hz | §3.1; ×3 drain under a battery fault |
| 4.8 | Link | sim | `/uavK/link` | swarm_interfaces/LinkStatus | 1 Hz | see below |

The magnetometer and barometer are omitted because no decision uses them. They would be one SDF plugin each, added when a heading-fault scenario needs them.

**4.6 Target detector model (downward camera + classifier):**

- **Footprint:** a disc of radius `r = h·tan(FOV/2)`, with FOV = 60°. At 30 m that is r ≈ 17 m; at 10 m, r ≈ 6 m.
- **Detection probability** for a target inside the footprint:
  `p_d = p_0 · exp(−(h − h_opt)²/(2σ_h²)) · exp(−v/v_blur) · v_vis`
  Defaults: p_0 = 0.9, h_opt = 15 m, σ_h = 15 m, v_blur = 8 m/s.
- **Class scores:** a noisy probability vector over {survivor, vehicle, debris, animal, nothing}. It is drawn from a Dirichlet distribution centred on a row of an altitude-dependent confusion matrix:
  - **survey altitude (30 m):** about 60% of the mass on the true class, and survivor↔animal and debris↔nothing are confused.
  - **inspection altitude (10 m):** about 90% of the mass on the true class.
- **Position estimate:** true position plus Gaussian noise, σ = 0.05·h + GPS error.
- **False positives:** Poisson with rate λ_fp = 0.02 per covered cell per pass. Their class scores are drawn from the "nothing" row, which is deliberately ambiguous.
- **Data association:** the detector tags detections that are within 5 m of an existing local track; the coordinator fuses tracks across drones.
- **Survey quality**, which is what counts as coverage: h ≤ 40 m and v ≤ 5 m/s.

**4.8 Link model:**
- `q = 1 / (1 + exp((d − R_c)/s))`, with R_c = 250 m and s = 20 m (the far AO corner is ≈ 355 m out, so far sectors lose link). Here d is the distance to the base, or to the nearest relay drone if that link is better.
- `comms_sim` relays drone→coordinator reports and coordinator→drone orders, dropping each message with probability 1 − q and adding latency 50 ms + U(0, 200) ms.
- ponytail: ROS actions (FlyTo, Formation) bypass `comms_sim`. Instead the coordinator refuses to dispatch to a drone with q < 0.2. Real packet-level loss on actions would need a DDS router.

## 5. Action spaces

There are three layers. Only the **decision** layers (5.2, 5.3) are chosen by Jev+LLM. The **primitive** layer (5.1) and the **safety** layer (5.4) are deterministic C++.

### 5.1 Flight primitives (C++ `flight_controller`, the `/uavK/task` action)

| Primitive | Params | Completes when |
|---|---|---|
| `TAKEOFF` | altitude | within 0.5 m of altitude |
| `GOTO` | point, speed | within tolerance |
| `HOLD` | duration (0 = until replaced) | duration elapsed |
| `SURVEY` | sector polygon, altitude, lane spacing (default 2r·0.8 overlap), speed | lawnmower finished |
| `ORBIT` | centre, radius, altitude, laps | laps done |
| `FORMATION_SLOT` | slot pose relative to the leader point (from the coordinator) | replaced |
| `RTB` | — | over the assigned pad |
| `LAND` | — | on the ground, motors off |

Primitives are preemptible: a new goal cancels the current one. Exceptions are `LAND` below 2 m and any active safety reflex.

### 5.2 Worker decision space A_w (per drone, chosen by the agent)

| Action | Means | Maps to primitive(s) | Reversible? | Masked (unavailable) when |
|---|---|---|---|---|
| `continue` | keep the current assigned task | (no change) | yes | forced RTB is active; there is no task |
| `inspect` | divert to the best unresolved local detection | GOTO → ORBIT at 10 m, r = 6 m, 1 lap | yes until descent below 15 m (the **commit point**) | no unresolved detection; m_E < 0.1 |
| `hold` | hover in place and wait for the coordinator | HOLD(0) | yes | m_E < 0.05 |
| `return_to_base` | abandon the task and go home | RTB → LAND | yes (cancellable in flight) | — |
| `request_support` | keep the task and ask the coordinator for help (a second inspector, or a sector handoff) | (no motion change) + priority report | yes | a request is already pending for less than 20 s |
| `land_now` | emergency landing in place | LAND | **no** | not in critical failure (§5.4); over an obstacle |

`inspect` and `land_now` are **irreversible-class** decisions. Under act-then-revise they wait for the LLM's augmentation (up to T_wait) before they commit; see [FRAMEWORK §5.3](../../FRAMEWORK.md#53-commitment-act-then-revise).

The parameters of a chosen action (which detection, orbit radius, inspection altitude) are computed deterministically. The LLM may *propose* parameter adjustments within bounds (inspection altitude 8–15 m, orbit radius 4–10 m, laps 1–2), and the controller clamps them.

### 5.3 Coordinator decision space A_c

| Decision | Options | Executes as |
|---|---|---|
| **Phase transition** (each cycle) | `stay`, `advance`, `recover_all`, `abort` | state-machine transition; triggers the phase's swarm behaviour |
| **Transit formation** (on entering TRANSIT/RECOVER) | `column`, `line`, `v`, `diamond`, `circle` | Formation action; Hungarian slot assignment |
| **Sector allocation** (on entering SEARCH and after a drone is lost) | assignment matrix | Hungarian on cost C(d, s) ([DECISION_MODEL §7](DECISION_MODEL.md#7-domain-optimization-coordinator-allocator-c)) |
| **Track verdict** (per fused detection track) | `confirm_survivor`, `confirm_non_survivor`, `request_inspection`, `dismiss`, `await_data` | updates the track registry; `request_inspection` → dispatch the cheapest feasible drone |
| **Report response** (per incoming worker report) | `acknowledge`, `dispatch_support`, `reassign_sector`, `recall_drone`, `replan` | an order to the drone(s); `replan` → a Sonnet mission replan |
| **Relay placement** (C8, in v1) | `none`, or `relay(drone, point)` | GOTO + HOLD on the base→sector line, where both hops have q ≥ 0.7 |

## 6. Objectives

The **mission score J** is computed per episode by `mission_scorer`, the only node that sees ground truth:

```
J = 100 · ( w_R·Recall_surv + w_P·Precision_conf + w_C·Coverage
          + w_T·(1 − t_done/T_max) + w_E·(1 − E_used/(N·E_0)) )
    − Penalties − w_$·LLM_cost_usd
```

| Term | Definition | Default weight |
|---|---|---|
| Recall_surv | confirmed survivors within 5 m of a true survivor ÷ true survivors | w_R = 0.40 |
| Precision_conf | correct confirmations ÷ all confirmations (any class) | w_P = 0.20 |
| Coverage | covered AO cells ÷ total cells | w_C = 0.20 |
| Time | t_done = time all drones are landed (T_max if not) | w_T = 0.10 |
| Energy | total energy used | w_E = 0.10 |
| LLM cost | $ spent on the Anthropic + TypeSafe APIs | w_$ = 10 points per $ |

**Penalties** (counted, but the safety layer should keep them near zero):

| Event | Penalty |
|---|---|
| collision with an obstacle or the ground | −100 per drone |
| battery depleted in flight | −50 per drone |
| drone not recovered by T_max | −30 per drone |
| NFZ incursion | −20 per second |
| separation breach (< d_sep) | −5 per event |

Secondary metrics are logged but not scored:
- time to first survivor confirmation
- decisions per minute
- LLM calls per minute
- the fraction of decisions where the LLM changed the argmax
- calibration error per Jev question

## 7. Constraints

### 7.1 Hard constraints

Hard constraints are enforced in C++ by the `flight_controller` safety layer *and* as **action masks** in the decision layer. No model can override them.

| ID | Constraint | Enforcement (reflex) |
|---|---|---|
| H1 | Geofence: AO + base corridor, altitude 5–60 m | velocity clamp at the boundary |
| H2 | NFZ: never inside any NFZ cylinder + 5 m buffer | repulsive field + path planner detour |
| H3 | Obstacle clearance ≥ 3 m | repulsive field from the proximity scan; stop if < 1.5 m |
| H4 | Drone separation ≥ d_sep = 4 m | pairwise repulsive field (neighbour positions from `/uavK/status`) |
| H5 | Speed / acceleration / climb limits (§3) | saturation |
| H6 | Energy reserve: if m_E ≤ 0, **forced RTB** | overrides the task; masks every action except `return_to_base` and `land_now` |
| H7 | Lost link: q < 0.1 for 10 s → hold; for 30 s → RTB | timer in the agent, mirrored in the controller |
| H8 | Critical failure (battery fault with m_E < −0.05, or a motor fault) → `land_now` is unmasked, and forced if no other action is feasible | |

### 7.2 Soft constraints

Soft constraints live in the objective, or are penalised in the allocation cost:
- coverage overlap between sectors
- dwell time over already-covered cells
- inspections of tracks another drone is already inspecting
- leaving the assigned sector without an order

### 7.3 Faults (injected by episode seed)

| Fault | Effect |
|---|---|
| `battery_fault` | battery drain ×3 from a random time |
| `gps_degraded` | GPS noise ×5 and detector position σ ×3 |
| `motor_degraded` | max speed 2 m/s and climb 0.5 m/s |
| `comms_blackout` | link quality 0 inside a random 40 m disc in the AO |

## 8. Parameter registry

Every parameter is a ROS 2 parameter or an environment variable. They are grouped by owner node and are loaded from `config/scenario.yaml` (phase 2+).

| Group | Parameter | Default | Range |
|---|---|---|---|
| swarm | `num_drones` | 4 | 2–6 |
| env | `ao_bounds` | [20, 320] × [−150, 150] | |
| env | `num_targets` | 10 | 6–14 |
| env | `wind_speed` / `wind_dir` | random | 0–8 m/s / 0–2π |
| env | `visibility` | random | 0.5–1.0 |
| env | `t_max_s` | 600 | |
| env | `seed` | 0 | |
| vehicle | `v_max_h`, `v_max_up`, `v_max_down`, `a_max`, `yaw_rate_max` | 5, 2, 1.5, 2, 1 | |
| vehicle | `alt_survey`, `alt_transit`, `alt_inspect` | 30, 40, 10 m | 20–40, 30–50, 8–15 |
| energy | `E_0`, `P_hover`, `k_h`, `k_c`, `k_w`, `reserve` | 40 Wh, 180 W, 2, 25, 1.5, 0.10 | |
| sensors | `det_fov`, `det_p0`, `det_h_opt`, `det_sigma_h`, `det_v_blur`, `det_lambda_fp` | 60°, 0.9, 15, 15, 8, 0.02 | |
| comms | `R_c`, `link_slope`, `latency_ms` | 250 m, 20 m, 50 + U(0, 200) | |
| safety | `d_sep`, `obstacle_clear`, `nfz_buffer`, `lost_link_hold_s`, `lost_link_rtb_s` | 4, 3, 5 m, 10, 30 s | |
| control | `controller_hz`, `agent_hz`, `coordinator_hz` | 50, 1, 0.5 | |
| decision | see [DECISION_MODEL §8](DECISION_MODEL.md#8-example-parameters-overrides-of-framework-14) and [FRAMEWORK §14](../../FRAMEWORK.md) | | |
| scoring | `w_R`, `w_P`, `w_C`, `w_T`, `w_E`, `w_$` | 0.4, 0.2, 0.2, 0.1, 0.1, 10/$ | |

## 9. ROS graph and containers (built on jev_ai)

```
                     ┌──────────────────────── /coordinator ─────────────────────────┐
 user ─Mission.action─▶ mission_coordinator (jev_ai Agent, supervisor) ── FSM, tracks │
                     │    ├─ JevDecide.srv ──▶ jev (jev_ai_jev)                      │
                     │    ├─ Reason.action ──▶ llm (jev_ai_llm, Sonnet 5.5 medium)   │
                     │    └─ Formation.action / Allocate.srv ─▶ formation_planner (C++)
                     └───────▲ jev_ai Reports (via comms_sim) ── Directives ▼ ────────┘
                     ┌───────┴────────────── /uavK (×N) ───────────────▼─────────────┐
                     │ drone_agent (jev_ai Agent) ── step set W1–W7                   │
                     │    ├─ JevDecide.srv ──▶ jev (jev_ai_jev)                       │
                     │    ├─ Reason.action ──▶ llm (jev_ai_llm, Haiku 5.5 low)        │
                     │    └─ Task.action ────▶ flight_controller (C++) + safety H1–H8  │
                     │ sensor_sim (C++) ── battery, proximity, detections, link        │
                     └──────▲ odom/imu/altimeter/gps ───── cmd_vel ▼───────────────────┘
                     ┌──────┴────────────────────────────────────────────────────────┐
                     │ sim pod: Gazebo Fortress (noVNC :6080) + ros_gz_bridge         │
                     │ environment (py)  comms_sim (C++)  mission_scorer (py → Label) │
                     └────── jev_ai base: discovery · decision_logger ────────────────┘
```

**Ground-truth isolation:** only `environment`, `sensor_sim` and `mission_scorer` subscribe to `/env/ground_truth`.

**Example-specific interfaces** (`swarm_interfaces`): `Detection`, `DetectionArray`, `LinkStatus`, `DroneStatus`, `Track`, `TrackArray`, `Allocate.srv`, `Task.action`, `Formation.action`, `Mission.action`. All decision traffic uses `jev_ai_msgs`.

**Containers at N = 4:** 30 services from one image.

| Scope | Services |
|---|---|
| Base | discovery, decision_logger |
| Sim and world | sim + bridge, environment, comms_sim, mission_scorer |
| Per drone (×N) | flight_controller, sensor_sim, jev, llm, drone_agent |
| Coordinator | formation_planner, jev, llm, mission_coordinator |

**Kubernetes:** per-drone node types are StatefulSets whose pod ordinal gives the namespace (`uav{k+1}`).
