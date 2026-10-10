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
src/jev_ai/               jev_ai.core (pure) · jev_ai.ros (DecisionEngine, Agent, hierarchy, backend APIs)
src/jev_ai_servers/       jev_server · llm_server · decide_server · decision_logger · built-in backends
src/jev_ai_cpp/           DecisionClient, JevClient, ReasonClient (async), component example
src/jev_ai_bt/            BehaviorTree.CPP v4 plugins (+ vendored BehaviorTree.ROS2)
src/ros2jev/              ros2 jev status | ask | calibrate | replay | ope | search
src/jev_ai_bringup/       bringup.launch.py, params.yaml, catalog template
src/jev_ai_examples/      toy_patrol (py, C++, BT, supervisor) · turtlesim
deploy/                   Dockerfiles per distro, compose, kustomize (optional)
examples/drone_swarm/     Part B only
docs/
```

## Roadmap

### Part A: the `jev_ai` framework

| # | Feature | Verify | Pairing point |
|---|---|---|---|
| A1 | `jev_ai_msgs` + workspace + dev image stage | `ros2 interface show jev_ai_msgs/action/Decide` | interface review |
| A2 | `jev_ai.core`: catalog and validator, fusion (§6.2), Gumbel coupled selection (§6.3), irreversible rule (§6.4), commitment machine, triggers, rate limiter, prompt and schema builder; **golden vectors** | `pytest src/jev_ai` (pure, no ROS) | **you write the revise/keep rule and the β schedule** |
| A3 | `jev_server` (lifecycle, gen-param-lib, diagnostics, budgets) + backends `mock`, `replay`, `typesafe`, `llama_ros` | launch_test with mock; `ros2 service call jev/decide …`; live with `TYPESAFE_API_KEY` | mock realism |
| A4 | `llm_server` + backends `mock`, `replay`, `anthropic`, `openai_compat` | launch_test with mock; `ros2 action send_goal llm/reason …`; live with `ANTHROPIC_API_KEY` | augmentation prompt |
| A5 | `DecisionEngine` (in-process) + `decision_logger` + `jev_ai_bringup` + `ros2 jev status / ask` + `toy_patrol` Python agent + **live dashboard** (`dashboard` node, http://localhost:8080) | `ros2 launch jev_ai_examples toy_patrol.launch.py` → records stream and render live; kill `llm_server` → δ ≡ 0 fallback; restart → augmentation resumes | toy catalog |
| A5.5 | **Metrics**: `jev_ai.core.metrics` over DecisionRecords (phase mix, revision/fallback rates, LLM agreement and influence, confidence, switch rate, propensities, latency, cost, coupling, calibration with labels) shown in the dashboard's Metrics tab and as a static report (`ros2 jev report`) | `ros2 jev report /tmp/jev_ai/toy_patrol.jsonl -o report.html`; numbers checked against hand-computed toy cases | which metrics matter |
| A6 | `decide_server` + `jev_ai_cpp` clients + C++ toy agent (component) | launch_test: the C++ agent's decisions are identical to the Python engine's on golden states | C++ API ergonomics |
| A7 | `jev_ai.hierarchy`: reports and directives, supervisor pattern, directive-epoch coupling; toy supervisor + 3 workers | directive → workers' records share the noise key; expired and duplicate directives are dropped | supervisor questions |
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
| 2026-10-10 | A5.5 added: metrics over DecisionRecords + dashboard Metrics tab + `ros2 jev report` |
| 2026-10-10 | Security review: server params are read-only after launch (the ROS param services are open to any peer); spend guards fail closed (unknown model → highest listed rate, NaN cost closes the budget, `max_tokens` is a ceiling); dashboard binds 127.0.0.1 by default (compose: 0.0.0.0 behind a loopback port) and checks the Host header; record strings never reach `style=` unsanitized |
