# Framework review: findings and reconciliation (2026-10-10)

**Inputs:**
- An adversarial architecture review of FRAMEWORK.md.
- Three web-research sweeps:
  - ROS 2 library best practices
  - LLM-for-ROS library designs
  - decision-fusion, calibration and masking literature

**Status key:**
- **ADOPT**: a clear best practice that fits the goals; it will be folded into FRAMEWORK.md.
- **DECIDE**: conflicts with an earlier decision, or a genuine trade-off. Pair on it.
- **DEFER**: valid, but later.

**Goals the resolutions are judged against:**
- (G1) maximal extensibility
- (G2) drop-in adoption: add API keys, use the modules
- (G3) the decisions already made

## 1. Landscape facts that change the design

| Finding | Source | Consequence |
|---|---|---|
| **llama_ros shipped `llama_msgs/srv/EvaluateDecisions` on 2026-10-05.** It runs local GGUF decision models ("OpenJEV", Lev, Kev, …) with CHOICE/SCORE/NOUL questions, per-question probabilities and confidence. It is launched with `decision.launch.py` | github.com/mgonzs13/llama_ros (README, commits) | It gives a ready-made **local, offline Jev-style backend**. Keep our service shape close to it, with a `llama_ros` backend adapter (see DECIDE D4). Whether OpenJEV is related to TypeSafe Jev is unverified |
| No ROS library combines a calibrated decision service with LLM augmentation, versioned catalogs, record/replay, *enforced* budgets and allowlists, or BehaviorTree.CPP decision nodes. RAI (the largest framework) has **no Anthropic support** | RAI, ROSA, ros2ai, bob_llm, ros-mcp-server, ros2_lingua surveys | This confirms our niche, and these features are the differentiators |
| Verbalised LLM probabilities are usable **as features to calibrate**, not as probabilities. LLMs also have **position bias** in option lists | Tian 2023; Xiong 2024; Zheng 2024 | Treat q_L as a calibrated feature; randomise class order in LLM prompts |
| Using one shared uniform u with inverse-CDF sampling is order-dependent and can make the whole group fail at once. **Shared per-class Gumbel noise** (Gumbel-max coupling) is order-free and stable under masking, with disagreement ≤ 2δ/(1+δ) | Daliri, Musco & Suresh, arXiv 2408.07978; Kleinberg & Raghavan PNAS 2021 | Replace the sampler (A6 below). It keeps the "one seed" decision with better guarantees |
| With a fixed γ, fitting the fusion is a **convex conditional logit** | Heskes 1998; Satopää 2014 | Calibration is an exact convex fit, not a search |
| Logging the noise and propensities enables **off-policy evaluation** (SNIPS/DR) and exact counterfactual actions | Dudík 2011; Oberst & Sontag 2019 | Evaluate new β, τ and κ settings from logs, without re-running the sim |
| `python3-anthropic-pip` is **not** in rosdep, and `typesafe-sdk` has no key. Jazzy/Noble rejects rosdep pip installs (PEP 668). BehaviorTree.ROS2 is **not released** on the Humble or Jazzy buildfarm. Humble Nav2 uses BT.CPP **v3** | rosdistro grep; Discourse | Packaging plan (A14–A16) |
| REP-144 advises **against "ros" in package names** | REP-144 | See DECIDE D1 |

## 2. Resolutions

### Safety and correctness

| # | Finding | Resolution | Status |
|---|---|---|---|
| A1 | When the LLM times out, the sampled irreversible class a₀ commits. Also T_wait (4 s) is shorter than the LLM timeout (6 s) | On timeout commit the catalog **fallback** (which must be reversible and unmaskable), never a₀. The validator enforces `T_wait ≥ llm_timeout + 0.5 s` | ADOPT |
| A2 | Irreversible classes can be sampled from the distribution's tail | Irreversible classes are **never sampled**. They are chosen only when they are the argmax of π by a margin `m_irrev` (default 0.2) *and* augmentation has arrived. The LLM may only **lower** their probability (one-sided pooling) | ADOPT; the margin part **superseded** 2026-10-11 by the two-key rule (FRAMEWORK §6.4; `m_irrev` removed) |
| A3 | No deterministic way to force an action in an emergency | Add `require()` next to `mask()`: the app can force a class, which **bypasses** both models and is recorded as `FORCED` | ADOPT |
| A4 | A stale or racing LLM result can be applied to a changed state | Tag each request with `(decision_id, generation, state_hash, mask_hash)`. Discard the result if the generation has moved on or the mask changed. **Recompute the mask before every act.** Each question set runs in its own MutuallyExclusive callback group (a strand in C++) | ADOPT |
| A5 | Synchronous service calls deadlock; service calls in Humble have no timeout | Calls are **async only** (`call_async` / `async_send_request`). Each request gets a client-side deadline timer, and an expired deadline counts as a failure. API nodes run the HTTP SDK on a dedicated asyncio thread bridged to ROS futures. Tests use a fake clock | ADOPT |

### Fusion and sampling math

| # | Finding | Resolution | Status |
|---|---|---|---|
| A6 | Shared u with inverse-CDF sampling: herding at the tails, order dependence, lockstep switching at epoch boundaries, directive_id collisions | **Gumbel-max coupling**: a = argmax_k (log π_k/τ + g_k + κ·1[k = current]). The noise is `g_k = Gumbel(H(run_seed, group, question, epoch, class_id))`. A **coupling knob ρ** sets the probability that an agent uses group noise rather than its own. The epoch is `(issuer, directive_id)` for directive responses, else the `decision_id` counter, never wall-clock buckets. The same noise is reused for the Jev-only and fused decisions, so a revision happens with probability ≤ 2δ/(1+δ) | ADOPT (the default ρ is DECIDE D3) |
| A7 | The ε floor still acts as a de facto veto (β·log ε ≈ 10⁻⁴), and ε depends on the number of classes \|A\| | Bound the LLM's influence **in nats**: clip β·(log q̃ − log q̄) to [−B, B], with B = log 20. Normalise q over **unmasked** classes. Floor in log space, independent of \|A\|. Drop the "LLM cannot veto" claim and state the bound instead | ADOPT |
| A8 | The prior is double-counted: both p_J and q_L include base rates | Subtract a per-class log prior (learned biases in calibration; uniform by default) | ADOPT |
| A9 | Confidence means different things across question types (Jev's opaque c vs \|2p−1\| for noul) | The framework computes **c = 1 − H(p_J)/log\|A\|** (normalised entropy) for every type. Jev's own confidence is kept as a separate field (`jev_confidence`) and used as a calibration feature | ADOPT |
| A10 | β_max = 2 is too strong before calibration | Uncalibrated default `beta_max = 1.0`; `ros_jev_ai calibrate` raises it if the data supports it | ADOPT |
| A11 | "Status quo first" stacks on Jev's first-option bias. The LLM also has position bias | Status-quo *preference* becomes an explicit hysteresis/inertia logit bonus **κ** (A6), so it no longer comes from ordering. Catalog order stays status-quo-first for Jev (your decision); `replay` measures the bias and is a release gate. Class order is **randomised in LLM prompts** and mapped back | ADOPT (compatible with your decision) |
| A12 | Dithering on near-ties | Hysteresis via κ, plus dwell time; `argmax_change` triggers need a margin | ADOPT |
| A13 | `expectation` on a bimodal score yields a level neither model believes | Expose the full π, its expectation **and** its median; rename the field `u` → `level_mean` / `level_median` | ADOPT |

### ROS idioms and packaging

| # | Finding | Resolution | Status |
|---|---|---|---|
| A14 | "Subclass Agent" clashes with users' existing nodes (G2) | **Composition first:** `DecisionEngine(node, catalog, encode, mask, on_decision)` attaches to *any* existing `rclpy`/`rclcpp` node or LifecycleNode. `Agent` becomes an optional convenience wrapper | ADOPT |
| A15 | Backend extensibility | Nav2-style **named backend plugins** selected by parameter (`backends: [claude]`, `claude.plugin: …`): pluginlib in C++, `importlib.metadata` entry points in Python (`ros_jev_ai.jev_backends`, `ros_jev_ai.llm_backends`). LLM providers ship as `anthropic` (native, default) and `openai_compat` (OpenAI, Gemini, Ollama, vLLM); no LangChain in the core. Jev backends: `typesafe` (OpenRouter is a `base_url` param, not a backend), `llama_ros`, `mock`, `replay` | ADOPT |
| A16 | Parameters | `generate_parameter_library`: one YAML schema produces typed, validated params for C++ **and** Python, plus generated docs | ADOPT |
| A17 | Server lifecycle | The Jev and LLM servers become **lifecycle nodes** (validate key, catalog and connectivity in `on_configure`). C++ pieces are **components** (`rclcpp_components_register_node`) | ADOPT |
| A18 | Absolute topics `/ros_jev_ai/...` | **Relative names** plus remapping. A documented QoS for every endpoint: DecisionRecord reliable/keep_last(1000); Report best-effort; Directive reliable + `expires` stamp + dedupe on `(issuer, id)` + issuer allowlist | ADOPT |
| A19 | JSON strings inside messages | State becomes `diagnostic_msgs/KeyValue[]` (typed, introspectable, easy from C++). JSON stays only for app `payload`, `llm_params` and LLM `json` output. Add `interface_version`, `catalog_hash`, `policy_hash` and model versions to every record | ADOPT |
| A20 | Behaviour trees | A `ros_jev_ai_bt` package with BT.CPP v4 nodes `JevDecide`, `AugmentedDecide`, `Reason` (BehaviorTree.ROS2 wrappers, vendored or built from source). A Nav2-Humble (BT v3) flavour is DEFER | ADOPT |
| A21 | Drop-in UX | `ros2 launch ros_jev_ai_bringup bringup.launch.py` (Jev + LLM + logger, one params YAML). A `ros2 jev status` CLI (ros2cli extension) checks keys, endpoints, models and budgets. Keys come **only** from env or `*_api_key_file`, never from params. Starting a live backend without a key **fails loudly**; mock only on an explicit `backend:=mock` | ADOPT |
| A22 | Installing dependencies | `requirements.txt` + a custom rosdep source list with `python3-anthropic-pip` and `python3-typesafe-sdk-pip`; a PEP 668 note for Jazzy; per-distro Docker images on `ros:<distro>-ros-base` | ADOPT |
| A23 | Observability | `diagnostic_updater` for backend health (latency, error rate, rate-limit headroom, key present, budget). DecisionRecords go to rosbag2 (Jazzy service introspection) **or** JSONL. Prompts are never logged above DEBUG | ADOPT |
| A24 | Time semantics are undefined | Policy timers (dwell, wait, stale) run on the **node clock** (sim time OK). API deadlines are **wall time**. Records carry both stamps | ADOPT |

### Cost, security, scope

| # | Finding | Resolution | Status |
|---|---|---|---|
| A25 | No global rate limits or budgets; Jev's limit is 80 req/s | **Node-level** limits in the Jev and LLM servers: req/s, tokens/min and a **USD/hour budget**, all from config pricing. A circuit breaker degrades to β = 0. Budget state is published via diagnostics | ADOPT |
| A26 | Prompt injection through state, reports and directives | Untrusted fields are delimited and labelled in prompts. One-sided pooling for irreversible classes (A2). Bounded influence (A7). An issuer allowlist for directives. SROS2 guidance. A **data-egress note** (state leaves the robot) | ADOPT |
| A27 | Streaming text deltas as action feedback wastes bandwidth across N clients | Augmentation goals send coarse feedback only (`started`, `first_token`, `done`); text deltas only for free-form `Reason` goals | ADOPT |
| A28 | Overclaimed determinism ("reproduce a run exactly") | Claim **per-decision replay** (recompute π and a from a record) plus OPE, not whole-run reproduction | ADOPT |
| A29 | Cost when Jev is very confident | An optional cascade knob `skip_llm_above_c` (**default off**, so augmentation always runs, as you decided) | ADOPT (off) |
| A30 | Calibration method | `ros2 jev calibrate`: a convex conditional-logit fit (A-facts) with features log p_J, log q̃, (1−c)^γ·log q̃ and per-class biases; ECE/NLL reports for q_L alone | ADOPT |
| A31 | Offline evaluation | `ros2 jev ope`: SNIPS/DR with cluster bootstrap by group, using the logged noise and π. The episode `search` tool is kept only for τ, κ, dwell, deadline and ρ | ADOPT |
| A32 | The hierarchy (Report/Directive) may belong in apps | Keep it in the framework as an **optional module** (now `jev_ai.ros.hierarchy`, exported as `jev_ai.Hierarchy`). The swarm and toy examples use it; core users can ignore it | ADOPT |
| A33 | Docker, k8s and the Discovery Server are not library concerns | Move them to a `ros_jev_ai_deploy` package plus examples, still shipped and maintained (your compose + k8s requirement) but never required by the library | ADOPT |

## 3. Decisions (resolved 2026-10-10)

| # | Decision |
|---|---|
| D1 | Package prefix becomes **`jev_ai_*`**, per REP-144 |
| D2 | **One engine.** A decider node (`decide_server`, `Decide.action`) gives C++, BT and other clients full parity; Python also runs the engine in-process |
| D3 | Coupling ρ **per question**: 1.0 for directive responses, 0.5 for autonomous decisions |
| D4 | A **`llama_ros` adapter backend** for local/offline Jev-style decisions; our messages stay richer |

The original options are kept below for the record.

| # | Question | Options | Recommendation |
|---|---|---|---|
| D1 | Package prefix: REP-144 discourages "ros" in names | keep `ros_jev_ai_*` · switch to `jev_ai_*` | your call. `ros_jev_ai` is legal; `jev_ai` is idiomatic |
| D2 | How to achieve "full Python parity" in C++ | **(a)** a server-side **decider node** (`Decide.action`) owns fusion, commitment and records; C++, BT and Nav2 users are thin clients with full capability parity and one engine · **(b)** a C++ core with pybind11 bindings (rclpy/moveit_py style) · **(c)** two implementations pinned by golden vectors (current plan) | (a), plus an in-process Python `DecisionEngine` for zero-hop Python users |
| D3 | Default coupling ρ (how strongly a group shares noise) | ρ = 1 everywhere (literal "one seed") · ρ = 1 for directive-coordinated questions, 0.5 otherwise · per-question in the catalog, default 1 | per-question, default 1 for directive responses, 0.5 for autonomous decisions |
| D4 | How to align with llama_ros `EvaluateDecisions` | mirror its message shape and ship a `llama_ros` backend · backend adapter only · ignore | adapter backend in v1; keep our richer messages |
