# ROS-JEV: design and roadmap

## Goal

The goal is **a core Jev + LLM decision framework for ROS 2, `jev_ai`**.
- **System 1:** Jev, as ROS services, gives calibrated typed decisions.
- **System 2:** LLMs, as ROS actions, always augment those decisions with bounded influence.
- **Integration:** users add API keys and attach the modules to their *existing* nodes, behaviour trees or C++ code.

The framework is built and verified first. The drone swarm is built **at the very end**, purely as an application of it.

| Doc | Contents |
|---|---|
| [RESEARCH.md](RESEARCH.md) | prior art, APIs, Gazebo and k8s findings |
| [FRAMEWORK.md](FRAMEWORK.md) | **the core spec (v2)** |
| [REVIEW.md](REVIEW.md) | adversarial and best-practice review: 33 adopted resolutions plus decisions D1–D4 |
| [examples/drone_swarm/SYSTEM_MODEL.md](examples/drone_swarm/SYSTEM_MODEL.md) | the example world: mission, environment, vehicle, sensors, action spaces, objectives, constraints |
| [examples/drone_swarm/DECISION_MODEL.md](examples/drone_swarm/DECISION_MODEL.md) | the example's jev_ai configuration |

## Principles

1. **Domain-agnostic core.** Domain enters only through the catalog and the app callbacks (`encode_state`, `mask`, `require`, `on_decision`).
2. **Composition, not inheritance.** `DecisionEngine` attaches to any node. Remote clients (C++, behaviour trees, CLI) reach the same engine through `decide_server`.
3. **One engine.** A single Python implementation. Parity is by protocol (D2), pinned by golden vectors and launch tests.
4. **Deterministic code owns numbers and safety.**
   - Masks and `require` are absolute.
   - Irreversible actions are never sampled.
   - The LLM's influence is bounded in nats.
5. **Async everywhere.** Every call has a deadline, and generation and hash checks stop stale results from being applied.
6. **Extensible by plugins.** Backends are named and selected by parameter (pluginlib and entry points). Parameters come from one validated schema.
7. **Safe drop-in.**
   - Keys come only from env vars or a key file.
   - A live backend without a key fails loudly.
   - Budgets and a circuit breaker run on the servers.
   - Diagnostics report health and spend.
8. **Everything is logged** for calibration and exact per-decision off-policy evaluation.
9. **Standalone, verifiable phases.** Mock and replay backends mean CI needs no keys or network.

## Repo layout

```
src/jev_ai_msgs/          interfaces
src/jev_ai/               jev_ai.core (pure) · jev_ai.ros (DecisionEngine, Hierarchy, run) · jev_ai.viz (dashboard)
src/jev_ai_servers/       jev_server · llm_server · decide_server · decision_logger · dashboard · built-in backends
src/jev_ai_cpp/           DecisionClient, JevClient, ReasonClient (async), component example
src/jev_ai_bt/            BehaviorTree.CPP v4 plugins on the jev_ai_cpp clients
src/ros2jev/              ros2 jev status | ask | report | calibrate | replay | ope | search
src/jev_ai_bringup/       bringup.launch.py, params.yaml, catalog template
src/jev_ai_examples/      minimal · toy_patrol (py, C++, BT, supervisor) · turtlesim
deploy/                   one Dockerfile (ROS_DISTRO arg), compose, kustomize (optional)
examples/drone_swarm/     Part B only
docs/
```

## Roadmap

### Part A: the `jev_ai` framework

| # | Feature | Verify | Pairing point |
|---|---|---|---|
| A1 | `jev_ai_msgs` + workspace + dev image stage | `ros2 interface show jev_ai_msgs/action/Decide` | interface review |
| A2 | `jev_ai.core`: catalog and validator, fusion (§6.2), Gumbel coupled selection (§6.3), irreversible rule (§6.4), commitment machine, triggers, rate limiter, prompt and schema builder; **golden vectors** | `pytest src/jev_ai` (pure, no ROS) | **you write the revise/keep rule and the β schedule** |
| A3 | `jev_server` (lifecycle, declared params, diagnostics, budgets) + backends `mock`, `replay`, `typesafe`, `llama_ros` | launch_test with mock; `ros2 service call jev/decide …`; live with `TYPESAFE_API_KEY` | mock realism |
| A4 | `llm_server` + backends `mock`, `replay`, `anthropic`, `openai_compat` | launch_test with mock; `ros2 action send_goal llm/reason …`; live with `ANTHROPIC_API_KEY` | augmentation prompt |
| A5 | `DecisionEngine` (in-process) + `decision_logger` + `jev_ai_bringup` + `ros2 jev status / ask` + `toy_patrol` Python agent + **live dashboard** (`dashboard` node, http://localhost:8080) | `ros2 launch jev_ai_examples toy_patrol.launch.py` → records stream and render live; kill `llm_server` → δ ≡ 0 fallback; restart → augmentation resumes | toy catalog |
| A5.5 | **Metrics**: `jev_ai.core.metrics` over DecisionRecords (phase mix, revision/fallback rates, LLM agreement and influence, confidence, switch rate, propensities, latency, cost, coupling, calibration with labels) shown in the dashboard's Metrics section and as a static report (`ros2 jev report`) | `ros2 jev report /tmp/jev_ai/toy_patrol.jsonl -o report.html`; numbers checked against hand-computed toy cases | which metrics matter |
| A6 | `decide_server` + `jev_ai_cpp` clients + C++ toy agent (component) | launch_test: the C++ agent's decisions are identical to the Python engine's on golden states | C++ API ergonomics |
| A7 | `jev_ai.ros.hierarchy` (`jev_ai.Hierarchy`): reports and directives, supervisor pattern, directive-epoch coupling; toy supervisor + 3 workers | directive → workers' records share the noise key; expired and duplicate directives are dropped | supervisor questions |
| A8 | `jev_ai_bt`: `JevAsk`, `Decide`, `Reason` BT.CPP v4 nodes + BT toy | a BT toy runs a decision loop using only XML | BT port design |
| A9 | **turtlesim example** (Python engine, C++ client and BT variants) | visible: turtles patrol, avoid walls and react to injected "alerts" via jev_ai in noVNC | scenario |
| A10 | `ros2 jev calibrate / replay / ope / search` | on toy runs: calibration patch, bias report, an OPE estimate checked against a re-run, a search table | metrics |
| A11 | `deploy/`: per-distro images, compose, kustomize; toy and turtlesim on Docker Desktop k8s | `kubectl apply -k deploy/k8s/overlays/toy`; scale workers | resources and budgets |

### Part B: drone swarm reference example (very end)

| # | Feature | Verify |
|---|---|---|
| B1 | Sim core: world generator, N X3 drones with sensors, bridge, sim pod | `/uav1/odom` from another container; `cmd_vel` → climb in noVNC |
| B2 | `environment`, `sensor_sim`, `comms_sim`: targets, battery, proximity, detector, link | fly over a target → detections with class scores |
| B3 | `flight_controller` (C++): primitives + safety H1–H8 (H8 → `require`) | SURVEY → lawnmower pattern; NFZ deflection |
| B4 | `formation_planner` + allocator (C++), relay point | gtest; V formation in Gazebo |
| B5 | `drone_agent` on `DecisionEngine` (catalog W1–W7) | one drone finds and inspects a target |
| B6 | `mission_coordinator` (catalog C1–C8, Sonnet plan) | `Mission` goal end to end with 4 drones |
| B7 | `mission_scorer` + `EpisodeRunner` + Labels; P0–P4 via ope/search | J table |
| B8 | k8s overlay + scale | 6 drones fly |

## Decisions log

| Date | Decision |
|---|---|
| 2026-10-10 | Example scenario: search & rescue drone swarm |
| 2026-10-10 | Act-then-revise |
| 2026-10-10 | Log-linear pooling + temperature |
| 2026-10-10 | Event-triggered LLM calls with rate caps |
| 2026-10-10 | Coordinator Sonnet effort `medium`; drone Haiku `low` |
| 2026-10-10 | Status quo first in the catalog; LLM may propose bounded params; relay in v1; cost weight 10/$ |
| 2026-10-10 | **Framework first; the drone swarm is the final example** |
| 2026-10-10 | turtlesim example added; full C++ capability parity |
| 2026-10-10 | Review: 33 resolutions adopted ([REVIEW.md](REVIEW.md)) |
| 2026-10-10 | D1: prefix `jev_ai_*` (REP-144) |
| 2026-10-10 | D2: one engine; C++ and BT via `decide_server` |
| 2026-10-10 | D3: coupling ρ per question (1.0 for directive responses, 0.5 autonomous) |
| 2026-10-10 | D4: `llama_ros` Jev backend (local/offline) |
| 2026-10-10 | A1: `Decision` phases are string constants |
| 2026-10-10 | A2 defaults: β(c) per spec; `revise_or_keep` adopts the augmented pick when it differs |
| 2026-10-10 | A2: a revision inherits its decision event's dwell permission (dwell is checked at events only) |
| 2026-10-10 | A2 (paired): `require()` beats `mask()` on the same class (flagged `require_overrides_mask`) |
| 2026-10-10 | A2 (paired): when the LLM vetoes a pending irreversible class, re-pick among reversible classes with the augmented logits (REVISED/KEPT); FALLBACK only on timeout or missing weights |
| 2026-10-10 | A2 (paired): default γ = 2 (entropy confidence is harsh; LLM weight concentrates on uncertain Jev) |
| 2026-10-10 | A2 security: irreversible rule must also hold on Jev-only logits |
| 2026-10-10 | A2: a Jev error on report-type acts (threshold/expectation/none) yields no output; noul fallback defaults to `false` |
| 2026-10-10 | A2: the validator rejects irreversible classes with `augment: false` or with non-selecting acts |
| 2026-10-10 | A3–A5 shared contract: one lifecycle `BackendServer` base (keys, budget, breaker, deadline, retries, diagnostics, record cassette) for both servers; backends are entry points with pure interfaces in `jev_ai.backends`; replay backends read cassettes recorded by the servers (`record_path`) |
| 2026-10-10 | A3–A5: plain `declare_parameter` instead of generate_parameter_library (not in the Humble image); `decision_logger` writes JSONL only (`ros2 bag record /decisions` covers rosbag2) |
| 2026-10-10 | A4 (paired): Claude pricing per Mtok in/out — Haiku 5.5 $0.10/$0.50, Sonnet 5.5 $2/$10 (Opus 5.5 $4/$20) |
| 2026-10-10 | A4 (paired): `openai_compat` needs no key for local endpoints: loopback, private IPs, and private hostnames (compose/k8s services, `*.local`) |
| 2026-10-10 | A4 (paired): a model refusal or a `max_tokens` truncation gets its own error (not "invalid JSON"), and is not retried |
| 2026-10-10 | A5 (paired): an `until_commit` class past `commit_point()` is **held** (path `committed`) until `completed()`; masks, `require`, and a *different* irreversible class winning §6.4 still override it |
| 2026-10-10 | A5.5 added: metrics over DecisionRecords + dashboard Metrics tab (now a section, 2026-10-11) + `ros2 jev report` |
| 2026-10-10 | Security review: server params are read-only after launch (the ROS param services are open to any peer); a goal's `max_tokens` cannot exceed the server's; dashboard binds 127.0.0.1 by default (compose: 0.0.0.0 behind a loopback port) and checks the Host header; record strings never reach `style=` unsanitized |
| 2026-10-10 | A6: `decide_server` is a plain node (no backend or key to configure), one non-autonomous engine per goal `agent`. A goal with triggers or a directive is a decision event; a goal with neither is a **tick** (Jev asked, decide only if due, else the standing decision is the result), so C++ cadence matches `tick_hz`. A newer goal for the same (agent, set) preempts the older one (aborted). Remote `completed()` / `commit_point()` are the triggers `completed:<q>` / `commit_point:<q>` |
| 2026-10-10 | A6: `jev_ai_cpp` is one header (`jev_ai/clients.hpp`): `DecisionClient` (request / start(hz) / trigger), `JevClient`, `ReasonClient` (deadline cancels the goal). No `visibility_control.h` (Linux only); the component example is `jev_ai_cpp::ToyAgent` (`toy_agent`) |
| 2026-10-10 | A7: `jev_ai.ros.Hierarchy` is one class (report / direct / on_report / on_directive). With an engine, an accepted directive is a decision event in the directive's epoch (§6.3). Empty `issuer_allowlist` = any issuer (prototype); the toy workers allow only `/supervisor` |
| 2026-10-10 | A7 fix: **coupling needs a shared `run_id`** (noise = H(run_id, group, …)); toy launches mint one per launch and pass it to every agent and `decide_server`. Before this, toy agents in separate processes never shared noise |
| 2026-10-10 | A7 fix: toy_patrol uses `all_phases` so `report_now` acts on KEPT (the usual final phase); `on_decision` re-executes only when `Decision.act` |
| 2026-10-10 | A7: toy supervisor = its own engine (`order`: carry_on \| converge) over worker reports; only a *final* converge becomes a directive. C++ toy agents send `orders: none` (no directives in C++ yet) |
| 2026-10-10 | A8: `jev_ai_bt` nodes sit on the `jev_ai_cpp` async clients (BehaviorTree.ROS2 isn't released for Humble; nothing vendored). ROS node from blackboard `"node"` (the BT.ROS2 convention). `Decide` succeeds on the first decision (act-then-revise; `wait_final="true"` waits for the final phase); string ports parse `k=v;k=v` and `q=a,b;q2=c` |
| 2026-10-10 | A8 fix: an idle tick returns the **latest** decision (any phase), not the last final, so a tick during pending augmentation can't flip a BT back to the previous decision. C++ results are always delivered, flagged `standing` when unchanged (BT nodes must complete; apps skip re-acting) |
| 2026-10-10 | A9 scenario (chosen, not paired): 3 turtles in one turtlesim (Python engine, C++ DecisionClient, BT), question `action` = forward \| turn_left \| turn_right \| go_to_alert; walls avoided by **masking** `forward` within 1.2 m (plus an app hard stop at 0.4 m); alerts on `/alerts` (Point, z=1 = cleared) from `turtle_arena` or `ros2 topic pub`; turtle `k_max` 20/min |
| 2026-10-10 | A9 shutdown hygiene: `run()` shuts the executor down before destroying the node; `AsyncLoop.stop()` cancels in-flight calls and refuses new ones; C++ toy mains ignore the second SIGINT. **Known (cosmetic, Ctrl-C only):** `llm_server` can still exceed launch's 5 s grace and get SIGTERM, and C++ nodes can exit -2 (launch delivers SIGINT twice; rclcpp drops its handler after the first) |
| 2026-10-10 | Testing note: containers on the default Docker bridge share DDS discovery; run ROS tests one container at a time (or set distinct `ROS_DOMAIN_ID`s) |
| 2026-10-10 | (user) **No spending cap** while this is a prototype: `max_usd_per_hour` removed; spend is still reported (diagnostics `usd_last_hour`, per-decision `cost_usd`, dashboard). Production hardening (SROS2, auth, caps) deferred to A11 |
| 2026-10-10 | A10 calibrate: conditional logit of exactly the §6.2 rule (log p_J offset, w1·ℓ·d + w2·(1−c)^γ·ℓ·d + b_q), Newton + small ridge; γ picked by the NLL of the **clamped** patch (γ and β_min trade off along a flat ridge: unclamped selection chose infeasible β_min < 0). B on a log grid. b_q is now wired into fusion and the catalog takes per-question `beta_min, beta_max, gamma, B, eta, bias` |
| 2026-10-10 | A10 ope: every logged decision is re-derived from its own logits, noise, τ, κ and inferred `current` (the agent's previous selection); non-reproducing decisions (dwell, commit hold, forced...) are held. Estimates: `direct` (exact on logged states when labels give every class's reward), SNIPS, DR, ESS; cluster bootstrap over (run, group, epoch), the noise-sharing unit |
| 2026-10-10 | A10 replay: Jev only, backend in-process (offline tool: no server budgets); position bias measured by **reversing** class order; the toy mock rules now answer by class name so reordering is meaningful. search: successive halving + common random numbers through an `EpisodeRunner`; `toy_episode` simulates the toy with the real `Commitment` (the engine's decision-event rule is now `commitment.decision_due`, shared). The toy world moved to `toy_world.ToyWorld` and publishes its oracle as Labels |
| 2026-10-10 | A11: one runtime Dockerfile (`ROS_DISTRO` arg) instead of per-distro files; pip for the SDKs instead of a custom rosdep source. Compose: apps `include:` the framework file with an env file. k8s: base + overlays, agents as StatefulSets (pod `patrol-N` → node `patrol_N`, seed N), keys from an optional Secret. No Discovery Server (multicast works on single-node clusters). Production hardening (SROS2, auth, caps) still not built: prototype |
| 2026-10-10 | A10 verified on toy runs (5 robots × 5 min, mock backends, oracle labels): calibrate lowers next_action NLL 0.285 → 0.243 (β hits the §14 bounds: the mock LLM mostly echoes Jev); replay reproduces logged answers exactly and measures the mock's first_bias=0.1 as 0.020 position bias (0.000 with it off). OPE vs re-runs: base log predicts τ=1.2 → 0.784, calibrated → 0.861; re-runs measured 0.832 [0.775, 0.880] and 0.834 [0.781, 0.887]. All three logs rank calibrated > base > τ=1.2 consistently, but run-to-run state mix moves absolute accuracy ±0.04, as large as the effects: per-decision OPE is exact, whole-run levels need longer runs or `search` |
| 2026-10-10 | A11 verified: `jev-ai:humble` (1.69 GB, vs 5.4 GB dev image) and `jev-ai:jazzy` build; in each, the exact compose/k8s node commands (StatefulSet ordinal → `patrol_N`, layered params files) run the toy fleet with one run_id, labels and a serving dashboard, and the turtlesim overlay's commands run all three turtle flavours. `kubectl apply -k` and `docker compose up -d` themselves were not run here (auto-mode blocks shared-cluster / detached-service changes): the user runs them |
| 2026-10-10 | B1–B2: the world is generated per (N, seed) from `scenario.yaml`, so the targets drawn in Gazebo are the hidden ground truth. X3 with primitive visuals and the Fuel X3's inertia and gains (no network), driven by MulticopterVelocityControl (body-frame twist, yaw held at 0). Battery, proximity, detector, link and faults are simulated in C++ `sensor_sim`; `comms_sim` relays reports and directives with drop 1−q and latency. GUI uses **ogre2**: ogre1 on llvmpipe (no GPU, noVNC) drops whole visuals |
| 2026-10-10 | B3–B4: deterministic C++ owns flight and safety: H1–H5 filter every command, H6–H8 are reflexes (H8 surfaces `critical`, and the agent `require`s `land_now`). Nodes run on sim time with wall timers that skip when the clock hasn't moved. One Hungarian solver serves sector allocation (DECISION_MODEL §7 cost) and formation slots (squared distance); TRANSIT assembles the formation, then translates it rigidly |
| 2026-10-10 | B5–B6: **FORCED (no LLM call) whenever every selecting question is `require`d**: drones on the pad, and per-item sets with nothing in focus. Orders are re-sent every 8 s until the drone's status shows *that* order; formation/goto/relay details carry the point, because a shared `formation` string let a stale status ack the next slot unsent (and the agent's same-goal dedupe drop it). Inspections are dispatched only in SEARCH/CONFIRM; a likely survivor (p ≥ 0.3) may pull a surveying drone; replan never reallocates. Surveyed = the FC's lawnmower progress ≥ 0.97 |
| 2026-10-10 | B7: J per SYSTEM_MODEL §6, computed live by `mission_scorer` (the only node with ground truth), which also publishes Labels (`detection_class`, `track_verdict`) and a dashboard card per drone. `ros2 jev search` policies: P0 Jev only (k_max 0.01), P1 as shipped, P2 τ 1.0, P3 κ 0, P4 calibrated patch |
| 2026-10-10 | B8: compose `deploy/compose/swarm.yaml` (one container per node), k8s overlay `swarm` (sim Deployment, coordinator, `uav` StatefulSet with FC + sensors + agent per pod) |
| 2026-10-10 | B verified (mock): 6 drones seed 1 with GUI: TRANSIT 29 s, all sectors allocated, 3 survivors confirmed by 3:30, no tracebacks. Earlier 4-drone seed-0 full missions: J 59.9 / 38.8 / 41.7 (seed 0 is hard: a blackout over two survivors and a motor fault). **Live** (TypeSafe Jev + Haiku drones + Sonnet coordinator, 4 drones seed 0): every phase to DONE, 4/4 landed, 1,241 decisions, 0 fallbacks, Jev p50 162 ms, Claude p50 2.6 s, **$0.60** (Sonnet $0.52) — but J 2.55, 0/3 survivors, 46% coverage: Sonnet planned the 40 m maximum "to clear obstacles" (p_detect 0.14 vs 0.33 at 30 m), drones ran short of energy and were recalled. Fix: the plan context now states `flight_facts` (avoidance is automatic, confirmation needs ≤ 15 m, a p_detect/swath table by altitude); the drone link reads "good (unchanged for N s)" (Haiku read "good for 1 s" as about to drop) |
| 2026-10-10 | GUI: MinimalScene + GzSceneManager / InteractiveViewControl / CameraTracking (GzScene3D is deprecated in v6). **The floating WorldControl / WorldStats overlays must have a width and height**: unsized, they cover the 3D view and swallow every mouse event (no pan/zoom). Dashboard keeps the reader's scroll position across its 1 s re-render |
| 2026-10-10 | (user, paired) **Swarm rescaled**: 8 drones over 400 × 576 m (8 strips of 72 m = 4 lanes at 20 m), 8 survivors / 24 targets, 14 obstacles + 4 NFZs, `E_0` 60 Wh (+50%), `wind_max` 4 m/s, `t_max_s` 900. **Survey altitude fixed at 20 m** and removed from Sonnet's plan (p_detect 0.52 vs 0.33 at 30 m; live Sonnet had picked 40 m). Worst strip out and back in 4 m/s wind: 38 of 54 usable Wh. Radio range kept at 250 m, so far strips need relays |
| 2026-10-10 | (user, paired) **2× real time**: `real_time_factor` 2 with `physics_step_s` 0.008. At 4 ms, 8 drones reached only 1.6–1.8×: Gazebo publishes `/clock` every step (430 Hz wall) and every rclpy node pays for each message. 8 ms holds 2.0× with unchanged flight (TRANSIT 35 s, 0 separation breaches). `T_wait` is sim time, so the catalogs set it to llm_timeout × 2 + 0.5. **The C++ nodes' timers are sim time** (`rclcpp::create_timer` on the node clock): as wall timers, 2× halved the flight controller's control and status rates per sim second (stale peers: 3 separation breaches, two orbits on paired targets and two converging RTBs) and halved the sensor's detection frames per pass |
| 2026-10-10 | (user, paired) **LLM only on low Jev confidence**: `skip_llm_above_c` is now a per-question override; only questions that set it gate augmentation. A set-wide gate never skipped: yes/no and score questions are always near c = 0 (live `report_now` median 0.04), so min-over-all was always low. Drones gate on `next_action` < 0.7 (live data: ~11% of decisions ask Haiku); the coordinator gates track and report decisions at 0.3, keeps the global set (phase, formation: Jev median c 0.1–0.25) always augmented, and irreversible confirmations always ask |
| 2026-10-11 | **RCA: drones stuck at base (user's live run)**. Live Sonnet chose a 10 m *column*; formation slots trail behind the lead point (12, 0), so 8 drones reached back to x = −58, past the flight controller's H1 fence at x = −20. Four drones pressed against the fence (20 of 22 separation breaches), assembly never completed and TRANSIT ran to its 120 s timeout; meanwhile idle drones near home chose `return_to_base`, and landed drones are never re-tasked. Mock plans always fly a V, whose tail fits inside the fence. Fix: the solved formation is shifted along +x until its tail is at x ≥ 12. Also: a drone already orbiting a track counts as its inspection (no second drone on the same orbit), and the drones' Haiku `max_tokens` is 2048 (8 of 192 live answers were truncated at 1024) |
| 2026-10-11 | (user, paired) **Two-key irreversible rule** replaces the margin rule (§6.4): an irreversible class needs Jev's top unmasked class (which alone holds and asks) **and** the LLM's top unmasked class. Live, no survivor could be confirmed: after an inspection Jev gave `confirm_survivor` 0.37 vs 0.24 and Sonnet 0.70, but the LLM could only lower irreversible classes and its centred log-preference *raised* `await_data`. `m_irrev` removed |
| 2026-10-11 | (user, paired) **Hold for the LLM**: while an augmentation is open, Jev's argmax moving is not a decision event (masks, `require()`, app triggers and directives still are). Live at 2×: of 947 drone LLM requests only 75 answers were used; 391 decisions were superseded after a median 1.3 sim s (Haiku ≈ 7 sim s) and 538 were refused by the per-drone bucket, which the superseding decisions kept draining |
| 2026-10-11 | (user, paired) **Relaunch**: in SEARCH the coordinator re-launches, once, a landed drone that is not recalled, has no fault and margin > 0.3 |
| 2026-10-11 | (user, paired) **Dashboard, mission-first**: one page of native `<details>` sections (state kept across polls and reloads; closed ones aren't rendered): a sticky summary bar (statuses named by the dashboard's `summary` param, plus server health and spend), 3+ statuses with the same keys as one table (the drones; a row opens its decisions), each other app status (coordinator, scorer, comms, mission events, mission tracks) as a section, Decisions with agent / question set / outcome dropdowns, Servers, Metrics |
| 2026-10-11 | (user, paired) **Streamline** ([PLAN-streamline.md](PLAN-streamline.md)): composable dials, back-compatible: policy `preset` (fast / balanced / careful), the LLM dial (`llm.use` per set, `llm` per question), `ask_jev: tick \| change` (Jev only when the state's wording changes), `llm_timeout` sent as the LLM call's deadline (server clamps it to [1 s, its timeout]); `policy=` with a loaded Catalog raises. Docs: README quickstart, GUIDE.md, TUNING.md, `minimal` example, `.env.example`; bringup defaults to mock; uv scaffold removed |
| 2026-10-11 | (user, paired) **Swarm tuned for recall** (live run: 3/8, SEARCH reached at 3:19, battery short, recalls on noise): rules own the routine, the AI judges exceptions. Phases end on work done or on the farthest drone's way home (`advance`, `search_end_frac`, `recover_frac` removed; C1 is stay / recover_all / abort); the 64-question `fit` set and C6 `replan_needed` replaced by the Hungarian cost on distance and energy; W4 `energy_risk` removed; `ask_jev: change` on both agents; recall/reassign masked without a cause (margin < 0.15, a fault, critical); an idle drone with nothing to inspect holds. `E_0` 90 Wh, `v_survey` 6 m/s (`v_max_h` 7), coverage gate 1.25·v_survey. **H7 waits for a SURVEY / ORBIT / GOTO to end** (46% of the AO is past the 250 m link) and tracks are resent when the link returns |
| 2026-10-11 | (user, paired) **Relays and free workers** ([PLAN-relays.md](PLAN-relays.md)): num_drones // 4 dedicated relay drones (white, full drone agent), placed by rule every 5 s at the 1-centre of their band's workers and open tracks, clamped to an uplink q ≥ 0.7 (C8 `relay_needed` removed: it never fired). **No formation**: drones take off to and home on altitude layers (workers 40/44/48/52 m, relays 56 m; RTB flies at the goal's altitude), TRANSIT removed, C2 `transit_formation`, the plan's formation fields, `/swarm/formation` and `FORMATION_SLOT` removed (`formation_planner` → `allocator`). Leash: a worker's own `inspect` needs q ≥ 0.3 at the target via the base or a relay. C4 answers only in SEARCH / CONFIRM (seed 0: a `reassign_sector` hold replaced an RTB for 5 min). Scorer reports `contact` (airborne worker-time at q ≥ 0.3) |
| 2026-10-11 | (user) **Dashboard Comms section**: the dashboard discovers every `Directive`/`Report` topic and matches each message across topics (`viz/traffic.py`), so relayed traffic (the swarm's `comms_sim`) shows per link delivered / dropped / latency, a swimlane timeline and a filterable message log. Generic: no app code, no new message type. Mock run: counts equal `comms_sim`'s own (268 relayed, 2 dropped), latency p50 65–90 ms wall |
| 2026-10-10 | (user) **`max_tokens_per_min` removed** from both servers: six drones exhausted 200k/min and silently fell back to Jev only. Tokens/min stays a reported metric; `max_rps` and the circuit breaker remain |
