# ros_jev_ai: a System 1 / System 2 decision framework for ROS 2

**ros_jev_ai** turns any ROS 2 node into a decision-making agent. The agent declares its decisions as **typed questions**, and the framework does the rest:

1. answers each question with a calibrated **Jev** model (System 1, exposed as a ROS **service**)
2. **augments** each answer with an **LLM** (System 2, exposed as a ROS **action**)
3. **fuses** the two answers, **masks** unsafe options, **samples**, and **commits** the decision under an act-then-revise protocol
4. logs every decision so the agent can be **optimised offline**

The framework is domain-agnostic. The drone swarm in [examples/drone_swarm/](examples/drone_swarm/) is the reference application, built last on top of the framework.

Status: **draft for review**. ⚑ marks an open decision.

---

## 1. Concepts

| Concept | Definition |
|---|---|
| **Agent** | A ROS 2 node that makes decisions; a Python or C++ subclass of the runtime base class. It provides *state*, *masks* and *execution*; the framework provides everything else. |
| **Question** | One typed decision, answered with a **categorical distribution** over ordered **classes**. There are three types: **choice** (pick one option), **score** (an ordered set of 2–10 levels) and **noul** (true/false). |
| **Question set** | Questions asked together against the same state, in one Jev request and at most one LLM call. It is the unit of a *decision*. |
| **Catalog** | YAML declaring the agent's question sets: instructions, classes and criteria, how each answer is acted on, reversibility, fallbacks, and the parameters the LLM may propose. Versioned and tunable. |
| **State** | A flat JSON object of short interpreted strings produced by the agent's *encoder*, e.g. `"energy": "tight (38% left, 31% needed)"`. Numbers are computed in code, never by the models. |
| **Mask** | The classes forbidden right now, for safety or feasibility. Deterministic and absolute: a masked class has probability 0. |
| **Fusion policy** | Combines the Jev distribution p_J (confidence c) with the LLM preference q_L: a confidence-weighted log-linear pool, then masking, then temperature sampling. |
| **Trigger** | An event that starts a decision and/or an LLM augmentation call (argmax flip, low confidence, an app event, a periodic refresh, …). |
| **Commitment** | The lifecycle of a decision: `PROVISIONAL` → (`REVISED`) → `COMMITTED`. Reversible classes act provisionally; irreversible ones wait for augmentation. |
| **Report / Directive** | The hierarchy primitives. A worker publishes **Reports** upward; a supervisor sends **Directives** downward. A supervisor is just an agent whose triggers are reports. |
| **Backend** | A pluggable implementation behind the Jev node (`typesafe`, `openrouter`, `mock`, `replay`) or the LLM node (`anthropic`, `mock`, `replay`). |
| **DecisionRecord** | The complete trace of one decision. It is the training and evaluation data for the optimisation toolkit. |

## 2. Architecture

```
 ┌──────────────────────────── Agent node (app subclass) ────────────────────────────┐
 │  encode_state()  mask()  on_decision()            ← the app implements these three │
 │  ─────────────────────────────── runtime (ros_jev_ai_agent) ─────────────────────────── │
 │  scheduler ─ triggers ─▶ DecisionEngine (ros_jev_ai_core) ─▶ commitment state machine  │
 │     │                    fusion · masks · sampler · rate limiter · prompt builder  │
 └─────┼──────────────────────────┼─────────────────────────────┼────────────────────┘
       │ JevDecide.srv            │ Reason.action               │ DecisionRecord
       ▼                          ▼                             ▼
 jev node (ros_jev_ai_jev)       llm node (ros_jev_ai_llm)        decision_logger (ros_jev_ai_tools)
 backends: typesafe ·        backends: anthropic ·        → JSONL → calibrate · replay ·
 openrouter · mock · replay  mock · replay                  search (offline optimisation)
```

| Package | Language | Contents | ROS? |
|---|---|---|---|
| `ros_jev_ai_interfaces` | IDL | msgs, srvs and actions (§3) | yes |
| `ros_jev_ai_core` | Python | pure library: catalog loader and validator, fusion, sampler, masks, triggers, rate limiter, commitment machine, prompt/schema builder, parameter clamping | **no**, so it is unit-testable without ROS |
| `ros_jev_ai_cpp` | C++ | header + library: the same fusion, sampler and commitment logic (parity-tested against shared golden vectors), Jev/LLM client wrappers, `ros_jev_ai::Agent` base | yes |
| `ros_jev_ai_jev` | Python | `jev_node`: the JevDecide service server and its backends | yes |
| `ros_jev_ai_llm` | Python | `llm_node`: the Reason action server and its backends | yes |
| `ros_jev_ai_agent` | Python | `ros_jev_ai_agent.Agent` base node: scheduler, clients, records, the report/directive helpers | yes |
| `ros_jev_ai_tools` | Python | `decision_logger` node; CLIs `ros-jev-ai-calibrate`, `ros-jev-ai-replay`, `ros-jev-ai-search` | partly |
| `ros_jev_ai_examples` | Py + C++ | `toy_patrol`: a pure-Python world with Python and C++ agents and a supervisor; it verifies the framework without Gazebo | yes |

**Why Jev and the LLM are separate nodes instead of libraries inside the agent:**
- **Scaling:** they scale and deploy independently, one container each.
- **Shared secrets:** they hold the API keys, so agents never see them.
- **Language independence:** C++ agents need no HTTP or SDK code.
- **Recording and replay:** they can be swapped for `replay` backends, so a recorded run can be re-run deterministically.

## 3. Interfaces (`ros_jev_ai_interfaces`)

```
# msg/Question.msg
uint8 NOUL=0
uint8 CHOICE=1
uint8 SCORE=2
string name
uint8 type
string instructions
string[] classes        # CHOICE options / SCORE levels (ordered) / NOUL ["true","false"]
string[] criteria       # aligned with classes; "" = no description

# msg/Distribution.msg
string question
uint8 type
string[] classes
float64[] probabilities # aligned with classes, sums to 1
float64 confidence      # Jev confidence (choice/score); noul: |2p-1|

# srv/JevDecide.srv
string state            # JSON object
ros_jev_ai_interfaces/Question[] questions
string model            # "" = node default
---
bool success
string error
string model            # exact version that answered
ros_jev_ai_interfaces/Distribution[] answers
float64 latency_ms
uint32 input_tokens

# action/Reason.action
string system           # role + rules; stable per agent (prompt-cached)
string context          # JSON
string response_schema  # JSON Schema; "" = free text
string model            # "" = node default
string effort           # "" = node default
---
bool success
string error
string text
string json
string model
uint32 input_tokens
uint32 output_tokens
float64 latency_ms
---
string delta            # streamed text since last feedback
uint32 chars_so_far

# msg/Report.msg — worker → supervisor
std_msgs/Header header
string agent
string kind             # app-defined label, e.g. a situation class
string subject          # entity the report is about, e.g. "track:T7"
float64 priority        # 0..1
ros_jev_ai_interfaces/Distribution[] beliefs
string payload          # JSON

# msg/Directive.msg — supervisor → worker(s)
std_msgs/Header header
uint64 directive_id
string issuer
string target           # agent name, or "*" for broadcast
string kind
string payload          # JSON

# msg/DecisionRecord.msg — one per decision phase transition
std_msgs/Header header
string run_id
uint64 run_seed
string agent
string group            # sampling group (§6.3)
string question_set
uint64 decision_id
string phase            # PROVISIONAL | REVISED | COMMITTED | KEPT | FALLBACK
string[] triggers
string subject
string state            # JSON as sent
ros_jev_ai_interfaces/Distribution[] jev
ros_jev_ai_interfaces/Distribution[] llm        # empty if no augmentation
float64 llm_age_s
float64[] beta          # per question
ros_jev_ai_interfaces/Distribution[] fused      # after mask, before temperature
string[] masked         # "question/class"
string[] selected       # per question: class acted on ("" if none)
float64[] tau
float64[] u             # uniform draws used
string llm_params       # JSON after clamping
string llm_rationale
float64 jev_latency_ms
float64 llm_latency_ms
uint32 jev_input_tokens
uint32 llm_input_tokens
uint32 llm_output_tokens
float64 cost_usd

# msg/Label.msg — ground truth for offline calibration (apps/scorers publish)
std_msgs/Header header
string run_id
string subject
string question
string true_class
string source
```

## 4. Catalog (declarative decision spec)

```yaml
# share/<app>/catalog.yaml
version: 3
question_sets:
  next_step:                         # one Jev request (+ ≤1 LLM call) per decision
    state_keys: [task, energy, link, hazards, detection]   # fixed order, validated
    tick_hz: 1.0                     # 0 = event-driven only
    augment: true                    # LLM augmentation for this set
    subject_from: detection          # state key or app callback → DecisionRecord.subject
    questions:
      next_action:
        type: choice
        instructions: "Choose the robot's next action."
        act: sample                  # sample | argmax | expectation | threshold | none
        classes:                     # ORDER MATTERS: status quo first (§6.3)
          - {name: continue, criteria: "Keep doing the assigned task.", reversible: true}
          - {name: inspect,  criteria: "Take a closer look at the detection.", reversible: until_commit}
          - {name: halt_now, criteria: "Emergency stop; critical failure only.", reversible: false}
        fallback: continue           # used if all classes masked or both models fail
        tau: 0.5
      report_now:
        type: noul
        instructions: "Should the robot report to its supervisor now?"
        criteria: {"true": "Something changed the supervisor should know.", "false": "Nothing new."}
        act: threshold
        threshold: 0.5
      urgency:
        type: score
        instructions: "How urgent is resolving the detection?"
        classes: [{name: none}, {name: low}, {name: high}]
        act: expectation             # → u = E[level]/(L−1) ∈ [0,1]
    llm_params:                      # bounded parameters the LLM may propose (clamped)
      inspect_altitude_m: {min: 8, max: 15, default: 10}
    llm:
      system: "You are the onboard reasoning module of {agent}. …"   # templated
      max_age_s: 30
policy: {}                           # per-catalog overrides of §14 defaults
```

The loader validates the catalog at startup and the agent refuses to start if validation fails. The checks:
- question types and class counts, against Jev's limits: choice ≤ 255 classes, score 2–10 levels
- `act` matches the question type
- the fallback exists among the classes
- the `state_keys` the encoder returns match the declared keys exactly
- `llm_params` bounds

## 5. Agent runtime

### 5.1 App API

**Python:**

```python
from ros_jev_ai_agent import Agent, Decision

class MyAgent(Agent):
    catalog = "my_pkg/catalog.yaml"

    def encode_state(self, qset: str) -> dict[str, str]: ...
    def mask(self, qset: str) -> dict[str, set[str]]: ...       # question → forbidden classes
    def on_decision(self, d: Decision) -> None: ...             # d.phase, d.selected, d.values, d.params

# App events: self.trigger("new_detection", qset="next_step", priority="high")
# Hierarchy: self.report(kind, subject, beliefs, payload) / self.on_directive(fn)
```

**C++:**

```cpp
class MyAgent : public ros_jev_ai::Agent {
  ros_jev_ai::State encode_state(const std::string& qset) override;
  ros_jev_ai::Mask  mask(const std::string& qset) override;
  void on_decision(const ros_jev_ai::Decision& d) override;
};
```

The C++ runtime does the same scheduling, fusion and commitment as Python. Shared golden vectors (`ros_jev_ai_core/test/golden/*.json`) are asserted by both pytest and gtest.

### 5.2 Decision loop

For each question set, on a tick or a trigger:

1. **State:** `encode_state()` produces the state, which is checked against `state_keys`.
2. **Jev:** call `JevDecide` and get p_J and c per question. On timeout or error, see §11.
3. **Masks:** call `mask()`. If the set has *no* allowed class, use the fallback.
4. **Fuse:** use the latest LLM preference q_L, decayed by its age (§6.1).
5. **Act:**
   - `sample`: sample per §6.3.
   - `argmax`: take the argmax.
   - `expectation`: compute u.
   - `threshold`: compare p(true) with the threshold.
6. **Commit:** run the commitment machine (§5.3) and call `on_decision()`.
7. **Augment:** if an augmentation trigger fired and the rate limiter allows it, send an async `Reason` goal. When it returns, re-fuse and pass the result back through the commitment machine.
8. **Record:** publish a `DecisionRecord` for every phase transition.

### 5.3 Commitment (act-then-revise)

```
           decision event
                │
        sample a₀ from π₀ (LLM-decayed)
      ┌─────────┴─────────────────────────────┐
  a₀ reversible                         a₀ irreversible / past commit point
  on_decision(PROVISIONAL)              hold current; request augmentation (reserved slot)
      │                                 wait ≤ T_wait
  LLM result → π₁                         ├─ result → sample a₁ from π₁ → COMMITTED
  keep if π₁(a_cur) ≥ θ_keep·max π₁       └─ timeout → commit a₀ (FALLBACK, logged)
  and dwell < T_dwell → KEPT
  else resample → REVISED (on_decision)
```

- **Decision events** are the only times sampling happens:
  - Jev's argmax changes from the current class
  - the current class becomes masked
  - an app trigger fires
  - the app reports the current action as complete
  - an LLM result arrives

  Ticks without an event keep the current class. This prevents dithering when τ > 0.
- **Dwell:** no voluntary switch within T_dwell. Switches forced by a mask are immediate.
- `reversible: until_commit` classes become irreversible when the app calls `self.commit_point(question)`.

## 6. Fusion and sampling

### 6.1 Pooling (identical for all question types)

```
q̃_L(a)  = (q_L(a) + ε) / (1 + |A|·ε)              the LLM cannot veto; only masks can
β(c)    = β_min + (β_max − β_min) · (1 − c)^γ      the LLM's weight grows as Jev's confidence falls
β_eff   = β(c) · exp(−age / T_stale)
log π(a) = log p_J(a) + β_eff·log q̃_L(a) + log m(a) − log Z
```

If there is no q_L, β_eff = 0. Questions the LLM omitted also get β_eff = 0.

### 6.2 Worked example (golden vector `fusion_01`)

Inputs: β_min 0.1, β_max 2.0, γ 1; p_J = {.70, .20, .05, .05}; q_L = {.10, .80, .05, .05}.

| c | β | π | π at τ = 0.5 |
|---|---|---|---|
| 0.95 | 0.195 | .644 / .276 / .040 / .040 | .839 / .154 / .003 / .003 |
| 0.60 | 0.86 | .359 / .613 / .014 / .014 | .255 / .744 / .000 / .000 |

**β sets how far the LLM moves the distribution; τ sets how much of the spread becomes behaviour:**
- τ < 1 sharpens π.
- τ = 1 samples π exactly.
- τ > 1 flattens π.

### 6.3 Sampling: shared seed, inverse CDF

```
π_τ(a) ∝ π(a)^(1/τ)                    over classes in catalog order
u      = H(run_seed, group, question, epoch) / 2^64
a      = first class whose cumulative π_τ ≥ u
epoch  = directive_id    if the decision answers a directive
         floor(t / epoch_s) otherwise
```

- **Group:** all agents in a group (e.g. the whole swarm, set by the `group` parameter) share u for the same question in the same epoch.
  - If their fused distributions are similar, they choose the **same** class (coherent behaviour).
  - If their distributions really differ, they still diverge.
- **`group = <agent name>`** gives independent sampling.
- **Class order matters:** with status quo first, a small u selects the status quo for everyone. Catalog class order is therefore part of the policy.
- **Reproducibility:** the same seed, the same records and `replay` backends reproduce a run exactly.

### 6.4 Non-sampled acts

| act | Output | Determinism |
|---|---|---|
| `expectation` | u = Σ ℓ·π(ℓ)/(L−1) | deterministic given π |
| `threshold` | π(true) ≥ θ | deterministic given π |
| `argmax` | argmax π | deterministic given π |

All three are still LLM-augmented through π.

## 7. LLM augmentation contract

- **Triggers** (configurable per question set):

  | Trigger | Fires when | Priority |
  |---|---|---|
  | `argmax_change` | | normal |
  | `low_confidence` | c < c_low | high |
  | `irreversible_pending` | | reserved |
  | `periodic` | every T_refresh | low |
  | `directive` / `report` received | | normal |
  | `app:<name>` | app-defined (e.g. a "novelty" noul ≥ 0.5) | app-set |

- **Rate limiter:**
  - A token bucket allows k_max calls per minute per agent, with one slot reserved for `irreversible_pending`.
  - Triggers that fire while a call is in flight are coalesced into one follow-up call.
- **Request** (built by `ros_jev_ai_core.prompt`):
  - `system`: the catalog `llm.system` template, the class vocabulary with criteria, the mask semantics and the objective. It is stable per agent, so it is prompt-cached.
  - `context`: JSON with `{state, jev: {q: {class: p}, confidence}, masked, current, recent: last 3 decisions + outcomes}`.
  - `response_schema`: generated from the catalog, so the weights are typed per question and per class.

    ```json
    {"weights": {"<question>": {"<class>": 0..1}}, "params": {...bounded...},
     "rationale": "≤2 sentences", "flags": ["novel_situation"|"option_missing"|"data_suspect"]}
    ```

- **Response handling:**
  - Weights are normalised per question into q_L.
  - `params` are clamped to `llm_params` bounds and exposed as `Decision.params`.
  - `flags` are forwarded in records. `option_missing` is surfaced to the app, which typically sends a Report.

## 8. Jev node (`ros_jev_ai_jev`)

- **Parameters:** `backend`, `model` (pinned, e.g. `jev-1.13.0`), `timeout_s` (1.0), `max_retries` (1), `base_url`. The key comes from `TYPESAFE_API_KEY` or `OPENROUTER_API_KEY`.
- **Backends:**

  | Backend | Behaviour |
  |---|---|
  | `typesafe` | uses the `typesafe-sdk` `TypeSafeClient.system_one`: `Question` → `Choice`/`Score`/`Noul`; answers → `Distribution` (noul p → [p, 1−p], confidence = \|2p−1\|) |
  | `openrouter` | the same SDK with `base_url=https://openrouter.ai/api` |
  | `mock` | deterministic and offline. Hashes (state, question) into a Dirichlet sample, biased toward the first class by `mock_first_bias`; optional rule hooks let examples return plausible answers |
  | `replay` | answers from a JSONL record file, keyed by hash(state, questions); errors on a miss |

- **Concurrency:** one async client, so concurrent service calls are handled with a ReentrantCallbackGroup.
- **Metrics:** latency, tokens and errors, published on `~/stats`.

## 9. LLM node (`ros_jev_ai_llm`)

- **Parameters:** `backend`, `model` (e.g. `claude-haiku-5-5`), `effort` (`medium` is the default everywhere, `low` for high-rate workers), `max_tokens`, `timeout_s`. The key comes from `ANTHROPIC_API_KEY`.
- **Backends:**
  - **`anthropic`:** `client.messages.stream(...)` with:
    - `output_config: {effort, format: {type: json_schema, schema}}` when a `response_schema` is given
    - a `cache_control` breakpoint on `system`

    Each text delta becomes action feedback. The final message becomes the result, with usage and cost.
  - **`mock`:** returns schema-valid output by echoing the Jev distribution from `context`, perturbed by a seeded `mock_disagreement`. It streams in fake chunks.
  - **`replay`:** answers from recorded results.
- **Cancellation:** a cancelled goal (the agent has moved on) aborts the HTTP stream.
- The same node also serves non-augmentation reasoning, such as mission planning: any `Reason` goal with any schema.

## 10. Hierarchy: supervisors and workers

- **Workers** publish `Report`s on `/ros_jev_ai/reports` (or on an app topic), typically when a `report_now` noul fires. Reports carry `beliefs`, the fused distributions, so supervisors receive calibrated uncertainty and not just labels.
- **Supervisors** are ordinary agents. A report or a tick triggers their question sets. They can run:
  - **per-item question sets:** one Jev request per subject (each track or report), with a focused state, issued concurrently
  - **batched augmentation:** one LLM call covering all pending items of a set
- **Directives** go down on `/ros_jev_ai/directives`.
  - Each worker's runtime turns a directive into a trigger.
  - `directive_id` is the sampling epoch, so all workers answering one directive share u (§6.3).
- **Transport:** the framework is agnostic. In apps, reports and directives can pass through a lossy relay, such as the drone example's `comms_sim`.

## 11. Safety model

| Invariant | Mechanism |
|---|---|
| Models never select a masked class | log m(a) = −∞; the mask is applied after fusion and before sampling |
| The LLM cannot veto a class | the ε floor |
| Every decision has a defined outcome | catalog `fallback`, plus the failure table below |
| Irreversible actions get System 2 input or an explicit, logged fallback | the commitment machine (§5.3) |
| No unbounded parameters reach the app | clamping to `llm_params` |

| Failure | Behaviour |
|---|---|
| LLM timeout, error or invalid schema | β_eff = 0 (pure Jev); `llm_error` recorded |
| One Jev failure | keep the current class; retry next tick |
| N_fail (3) consecutive Jev failures | fallback class; `degraded` app callback |
| Both systems down | fallback class held; the app decides any domain-specific failsafe |

## 12. Observability and optimisation toolkit (`ros_jev_ai_tools`)

1. **`decision_logger`** subscribes to `DecisionRecord` and `Label` and writes rotating JSONL files: `runs/<run_id>/decisions.jsonl` and `labels.jsonl`.
2. **`ros-jev-ai-calibrate`** is supervised, with no simulation needed. For questions that have `Label`s, joined on `subject` + `question`:
   - Reports reliability (ECE, Brier) for Jev, the LLM and the fused π.
   - Fits (β_min, β_max, γ, ε), and an optional per-question Jev temperature T_J (p_J ← softmax(log p_J/T_J)), by minimising the NLL of the true class.
   - Emits a `policy:` YAML patch.
3. **`ros-jev-ai-replay`** re-asks Jev on logged states using a variant catalog: different wording, criteria or **class order**. It reports the distribution shift and the labelled accuracy, and measures first-option bias. Cheap: Jev input costs $0.042 per million tokens and output is free.
4. **`ros-jev-ai-search`** is episode-level policy search for unlabelled (action) questions.
   - The app implements `EpisodeRunner.run(params, seed) -> Score(J, terms)`.
   - The tool runs random search with successive halving over a declared space, using common random numbers (paired seeds).
   - Output: J ± CI, component terms and $ per config.
   - Built-in baseline presets:
     - `rules`: app-provided
     - `jev_argmax` (β ≡ 0, τ → 0)
     - `jev_sampled` (β ≡ 0)
     - `llm_only` (uniform p_J, β = 1)
     - `fused` (defaults)

## 13. Deployment

- **Image:** one `ros-jev:ai` image, a multi-stage build. Every node is a container whose command selects the node and whose env selects its parameters.
- **Discovery:** the Fast DDS Discovery Server (`discovery` service). Every container sets `ROS_DISCOVERY_SERVER`; CLI containers also set `ROS_SUPER_CLIENT=TRUE`. The same setup works in compose and k8s.
- **Compose:** `deploy/compose/ros_jev_ai.yml` defines `x-ros-jev-ai-node` (image, env, discovery) plus `discovery` and `decision_logger`. Apps `include:` it and declare their nodes with `<<: *ros-jev-ai-node`.
- **Kubernetes:** a kustomize **base** (`discovery`, `decision-logger`, the `ros-jev-ai-keys` Secret, a ConfigMap of catalogs and policy) plus **components**:
  - `jev`, `llm`, `agent`: StatefulSet templates with `replicas: N`. The pod ordinal becomes the agent name or namespace, so scaling is one command.
  - Apps overlay the components with their node commands.
- **Keys:** only the `jev` and `llm` containers mount the API-key Secret.

## 14. Parameters (framework defaults; the catalog's `policy:` and ROS params override them)

| Parameter | Default | Range | Tuned by |
|---|---|---|---|
| `beta_min`, `beta_max`, `gamma` | 0.1, 2.0, 1.0 | [0, 0.5], [1, 4], [0.5, 3] | calibrate |
| `epsilon` | 0.02 | [0.005, 0.1] | calibrate |
| `tau` (per question) | 0.5; 0.2 for irreversible-heavy | [0.05, 1.5] | search |
| `group`, `epoch_s` | the agent's namespace, 5 s | | app |
| `theta_keep`, `T_dwell`, `T_stale`, `T_wait`, `T_refresh` | 0.5, 5 s, 30 s, 4 s, 20 s | | search |
| `k_max` (LLM calls/min per agent), `c_low` | 6, 0.5 | [1, 12], [0.3, 0.7] | search |
| `jev.model` / `jev.timeout_s` | `jev-1.13.0` / 1.0 | | |
| `llm.model` / `llm.effort` / `llm.timeout_s` | per app / `medium` / 6 | | |
| `N_fail` | 3 | | |

## 15. Non-goals (v1)

- Online learning of the policy. The per-run bandit over τ is a v2 idea.
- Running Jev or the LLM on the device: both are remote APIs behind service/action boundaries, so a local backend can be swapped in later.
- Multi-step LLM tool use inside augmentation: an augmentation is one structured call.
- A general mission planner. Planning is app code that uses `Reason`, as in the drone example.

## 16. Open questions ⚑

1. **Package prefix:** `ros_jev_ai_*`. Keep it, or choose another name?
2. **`toy_patrol`** (a pure-Python world with no Gazebo) as the framework's test bed: is that enough for verifying phases A5–A9, or do you want a turtlesim-based visual example too?
3. **C++ runtime scope:** the full `ros_jev_ai::Agent` parity (current plan), or only fusion, sampling and the clients, so C++ nodes call into Python agents?
