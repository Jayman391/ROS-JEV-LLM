# Drone swarm example: decision model

This is the drone swarm's use of **jev_ai**. It covers the **state encoders**, the **question catalog**, how each question's output maps to actions, the triggers, the masks, and the domain-specific optimization.

The mechanics (fusion, sampling, commitment, the LLM contract, logging and the tuning tools) are framework features specified in [FRAMEWORK.md](../../FRAMEWORK.md). This doc only configures them. The physical model is in [SYSTEM_MODEL.md](SYSTEM_MODEL.md).

## 1. Agents

| Agent | Count | Runtime | Jev | LLM | Question sets |
|---|---|---|---|---|---|
| `drone_agent` | N (`/uavK`) | Python node + `jev_ai.DecisionEngine` + `jev_ai.hierarchy` (worker) | `jev-1.13.0` | `claude-haiku-5-5`, effort `low` | `step` (1 Hz) |
| `mission_coordinator` | 1 | Python node + `jev_ai.DecisionEngine` + `jev_ai.hierarchy` (supervisor) | `jev-1.13.0` | `claude-sonnet-5-5`, effort `medium` (augmentation **and** mission planning) | `global` (0.5 Hz), `track` (per item), `report` (per item), `fit` (on allocation) |

**Coupling** (FRAMEWORK §6.3): every agent is in group `swarm`, which shares Gumbel noise.

| Question kind | ρ | Epoch |
|---|---|---|
| Answers a coordinator directive | 1.0 | `(coordinator, directive_id)` |
| Autonomous decision (W1) | 0.5 | 5 s node-clock bucket |
| Coordinator decisions (single agent) | — | coupling is moot |

Drones in similar situations mostly choose alike, a drone in a genuinely different situation diverges, and half the autonomous draws stay independent. That limits swarm-wide correlated errors.

## 2. State encoders

All values are computed in code and passed as bucket + rounded value + unit (FRAMEWORK §1, "State").

### 2.1 Drone `step` state (`state_keys`, in order)

| Key | Sources | Derivation | Encoding |
|---|---|---|---|
| `mission` | coordinator phase directive | phase + clock | `"SEARCH, 4:10 of 10:00 elapsed"` |
| `task` | `/uavK/task` feedback | primitive + progress + ETA | `"survey sector B: 62% done, ~95 s left"` · `"inspect T7: en route, 40 m"` · `"relay at (160, 40): holding"` · `"none"` |
| `energy` | `/uavK/battery`, odom, wind | m_E = (E − E_rtb − R)/E_0 | `ample` ≥ 0.30 · `adequate` 0.15–0.30 · `tight` 0.05–0.15 · `critical` < 0.05, plus `"(38% left, 31% needed to finish and return)"` |
| `home` | odom, wind | distance, ETA, along-track wind | `"180 m, ~40 s, headwind 4 m/s"` |
| `link` | `/uavK/link` | q + time in bucket + relay hop | `good` ≥ 0.7 · `degraded` 0.3–0.7 · `poor` 0.1–0.3 · `lost` < 0.1, plus `"for 12 s via uav3"` |
| `hazards` | proximity scan, peer status, NFZ map | active reflexes + nearest distances | `"none"` · `"obstacle 6 m ahead, closing 2 m/s; uav3 5 m"` |
| `detection` | `/uavK/detections` local tracks | best unresolved track by expected survivor value | `"T7, 140 m E: 3 sightings from 30 m; scores survivor 0.46, animal 0.31, debris 0.12; not inspected; no other drone assigned"` · `"none"` |
| `other_detections` | local tracks | count | `"2 more unresolved"` · `"none"` |
| `health` | drain residual, GPS σ, speed cap | fault detectors (residual > 3σ for 5 s) | `"nominal"` · `"battery draining 3x faster than expected"` · `"gps degraded"` · `"motor degraded"` |
| `coordinator` | directives (via `comms_sim`) | last directive + age + pending request | `"last order 12 s ago: survey sector B; support request pending 8 s"` |
| `wind` | env broadcast | speed + direction | `calm` < 2 · `moderate` 2–6 · `strong` > 6 m/s, plus `"from W"` |

### 2.2 Coordinator states

- **`global`:**
  - `mission`
  - `coverage`, e.g. `"64%; A 90%, B 62%, C 41% (uav3 slow), D 70%"`
  - `swarm`: one line per drone with task, energy and link
  - `tracks`, e.g. `"3 confirmed survivors, 1 confirmed other, 4 unresolved (2 likely survivor)"`
  - `time_budget`, e.g. `"recover must start by 8:30 (energy-limited by uav3)"`
  - `comms`, e.g. `"sector D far half: link poor; no relay"`
  - `risk_events`
- **`track`** (one per unresolved track): fused sightings, the best view altitude, the class beliefs gathered from reports, the position spread, the nearest confirmed survivor, and the cost of an inspection.
- **`report`** (one per incoming report): the report's kind, its beliefs and payload, the sender's `swarm` line, and its sector's coverage.
- **`fit`:** one line per (drone, sector) pair, e.g. `"uav2→C: 120 m away, energy after sector: tight, sector 41% covered"`.

## 3. Question catalog

The default or status-quo class is listed **first** (decided), which points Jev's first-option bias toward "no change". `ros2 jev replay` measures that bias. Sampling itself is order-free (Gumbel coupling), and status-quo inertia comes from κ.

### 3.1 Drone `step` (augmented; tick 1 Hz)

| ID | Type / act | Name | Instructions | Classes in order (criteria) | Reversibility |
|---|---|---|---|---|---|
| W1 | choice / sample, τ 0.5 | `next_action` | "Choose this search-and-rescue drone's next action." | `continue` (keep doing the assigned task; nothing requires a change) · `inspect` (fly lower to look at the unresolved detection; worthwhile if it may be a survivor and energy allows) · `hold` (hover and wait; the situation is unclear or orders are awaited) · `return_to_base` (go home now; energy, health or link make continuing unsafe or pointless) · `request_support` (keep working but ask the coordinator for help or a handoff) · `land_now` (emergency landing in place; only for a critical failure) | `inspect`: until_commit (descent below 15 m) · `land_now`: false · others: true |
| W2 | choice / none (beliefs) | `detection_class` | "What is the detected object most likely to be? Sightings from higher altitude are less reliable; consistent repeated sightings are more reliable." | `survivor` (a person needing rescue) · `animal` · `vehicle` · `debris` (rubble, wreckage or other inanimate clutter) · `nothing` (a false alarm; no real object) | — |
| W3 | score / expectation | `detection_priority` | "How important is it to resolve this detection soon?" | `ignore` · `low` · `moderate` · `high` · `urgent` | — |
| W4 | score / expectation | `energy_risk` | "How risky is the drone's energy situation for finishing its task and getting home, considering wind and distance?" | `comfortable` · `adequate` · `tight` · `critical` | advisory: never acted on (H6 is deterministic) |
| W5 | noul / threshold 0.5 | `report_now` | "Should the drone send a report to the coordinator now?" | true: something changed that the coordinator should know · false: nothing new | — |
| W6 | noul / threshold 0.5 | `unusual` | "Is this situation unusual, or poorly covered by routine behaviour?" | true: novel, conflicting or surprising signals · false: routine | → trigger `app:unusual` |
| W7 | choice / argmax | `situation` | "Which best summarises the drone's situation?" | `nominal` · `candidate_found` · `energy_concern` · `hazard` · `link_concern` · `fault` · `task_complete` | → `Report.kind` |

W2 and W3 are dropped from the set when `detection = none`.

**LLM params** (clamped):

| Param | Range | Default |
|---|---|---|
| `inspect_altitude_m` | [8, 15] | 10 |
| `orbit_radius_m` | [4, 10] | 6 |
| `orbit_laps` | [1, 2] | 1 |
| `hold_s` | [0, 60] | 0 |

### 3.2 Coordinator (augmented)

| ID | Set | Type / act | Name | Classes in order |
|---|---|---|---|---|
| C1 | global | choice / sample, τ 0.3 | `phase_transition` | `stay` · `advance` · `recover_all` · `abort` (irreversible) |
| C2 | global | choice / sample, τ 0.5 | `transit_formation` | `column` · `line` · `v` · `diamond` · `circle` |
| C5 | global | score / expectation | `mission_risk` | `low` · `moderate` · `elevated` · `severe` |
| C6 | global | noul / threshold 0.6 | `replan_needed` | true · false |
| C8 | global | noul / threshold 0.5 | `relay_needed` | true: a sector's link is poor enough that reports are being lost · false |
| C3 | track | choice / sample, τ 0.3 | `track_verdict` | `await_data` · `confirm_survivor` (irreversible) · `confirm_non_survivor` (irreversible) · `request_inspection` · `dismiss` |
| C4 | report | choice / sample, τ 0.5 | `report_response` | `acknowledge` · `dispatch_support` · `reassign_sector` · `recall_drone` · `replan` |
| C7 | fit | score / expectation | `fit_{d}_{s}` (N² questions) | `poor` · `fair` · `good` · `excellent` |

**LLM params:**

| Param | Range |
|---|---|
| `survey_altitude_m` | [20, 40] |
| `formation_spacing_m` | [4, 12] |
| `relay_point` | inside the AO, on the base→sector line |

**Mission planning** is app code using the coordinator LLM node.
- On `Mission.action`, Sonnet (effort `medium`) returns a structured plan:
  - the sector partition
  - the survey altitude
  - a transit formation preference
  - phase time budgets
  - priorities
- The plan is validated against the hard constraints. On failure it is re-asked once, then a default plan is used.
- The plan is re-run on C4 `replan` or when C6 crosses its threshold.

## 4. Output → execution

| Q | Consumed by | Execution |
|---|---|---|
| W1 | drone agent `on_decision` | `/uavK/task` goal (SYSTEM_MODEL §5.2) with the clamped params |
| W2 | report `beliefs` | coordinator track fusion |
| W3 | agent (which detection to inspect); coordinator (dispatch order) | utility |
| W4 | LLM context, records | — |
| W5, W7 | agent | `Report(kind = W7, beliefs = [W2, W3], subject = track)` |
| W6 | agent | trigger `app:unusual` (high priority) |
| C1 | FSM | phase change |
| C2 | FSM | Formation action |
| C3 | track registry | `request_inspection` → dispatch (§6) |
| C4 | coordinator | directives; `replan` → Sonnet plan |
| C5 | coordinator | u ≥ 0.75 → trigger `app:risk` |
| C6 | coordinator | replan |
| C7 | allocator | cost term (§6) |
| C8 | coordinator | choose the relay drone and point (§6) → `relay` directive |

## 5. Masks (hard constraints → forbidden classes)

| Condition | Forbidden |
|---|---|
| forced RTB (H6: m_E ≤ 0) | everything in W1 except `return_to_base`, `land_now` |
| no unresolved detection | `inspect` |
| m_E < 0.1 | `inspect` |
| m_E < 0.05 | `hold` |
| support request pending < 20 s | `request_support` |
| not in critical failure (H8), or over an obstacle | `land_now` |
| link lost ≥ 10 s (H7) | everything in W1 except `hold`, `return_to_base`, `land_now` |
| C3, track never seen from ≤ 15 m | `confirm_*` |
| C1, any drone airborne with link lost | `advance` while in LAUNCH or TRANSIT |

Masks are computed by `mask()` from the same deterministic quantities the C++ safety layer uses, so the reflexes and the masks always agree.

## 6. Triggers

| Framework trigger | Drone mapping | Coordinator mapping |
|---|---|---|
| `argmax_change` | W1 | C1, C3, C4 |
| `low_confidence` (c < 0.5) | W1 | C1, C3 |
| `irreversible_pending` | `land_now`, `inspect` past its commit point | `abort`, `confirm_*` |
| `periodic` | 20 s | 30 s |
| `directive` / `report` | directive received | report received |
| `app:*` | `unusual` (W6), `new_detection` | `risk` (C5), `new_track` |

Rate caps: 6 calls/min per drone and 4 calls/min for the coordinator.

## 7. Domain optimization (coordinator allocator, C++)

**Sector allocation** uses the Hungarian algorithm in O(N³). It runs on entering SEARCH, after a drone is lost, and on `reassign_sector`:

```
C(d, s) = w_dist·dist(d, entry(s))/D_max + w_fit·(1 − u_fit(d, s)) + w_sw·[s ≠ current(d)]
        + BIG·[E_need(d, s) + E_rtb > E(d) − R]
```

**Formation slots:** the Hungarian algorithm on squared distances.

**Inspection dispatch:** `d* = argmin over feasible d of [t_travel(d, track) + w_opp·remaining_coverage_value(d)]`, excluding drones whose own W3 priority is above 0.75.

**Relay** (when C8 fires):
- The point is on the segment from the base to the worst-linked sector's centroid, where both hops have q ≥ 0.7.
- The drone is the one with the lowest opportunity cost that is energy-feasible for the relay duration.

**Tuning** uses the framework tools:
- **Labels:** `mission_scorer` publishes `Label`s for W2 and C3 (true class per track subject), which `ros2 jev calibrate` fits.
- **Episode search:** `EpisodeRunner` = one seeded mission; J from SYSTEM_MODEL §6; run with `ros2 jev search`.

## 8. Example parameters (overrides of FRAMEWORK §14)

| Parameter | Drone | Coordinator |
|---|---|---|
| LLM model / effort | `claude-haiku-5-5` / `low` | `claude-sonnet-5-5` / `medium` |
| `k_max` | 6/min | 4/min |
| `T_refresh` | 20 s | 30 s |
| LLM timeout | 6 s | 15 s |
| `group` / `epoch_s` | `swarm` / 5 s | `swarm` / 5 s |
| `coupling` ρ | 1.0 directive-driven, 0.5 autonomous | — |
| `kappa` (status-quo inertia) | 0.7 | 0.7 |
| irreversible classes | `land_now`, `inspect` after its commit point (FRAMEWORK §6.4); H8 emergencies use `require()` | `abort`, `confirm_*` |
| `w_dist`, `w_fit`, `w_sw`, `w_opp` | — | 1.0, 1.0, 0.3, 0.5 |
