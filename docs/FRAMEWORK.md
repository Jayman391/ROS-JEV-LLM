# jev_ai: a System 1 / System 2 decision framework for ROS 2

**jev_ai** adds calibrated, LLM-augmented decision making to *existing* ROS 2 nodes:

- **System 1:** a fast typed decision model, **Jev** (TypeSafe, or local via llama_ros), runs as a ROS **service**. Each decision is declared as a typed question: choice, score, or yes/no.
- **System 2:** an **LLM** (Claude natively, or any OpenAI-compatible model) runs as a ROS **action**. It always *augments* the Jev answer, with bounded influence.
- **Fusion:** jev_ai fuses the two answers, applies the app's safety masks, samples with coupled noise, and commits the decision under an act-then-revise protocol.
- **Records:** every decision is recorded for calibration and off-policy evaluation.

**Drop-in contract:** set your API keys, launch one bringup file, and attach a `DecisionEngine` to a node you already have. Alternatively, call the services and actions from C++, behaviour trees or the CLI.

| | |
|---|---|
| Target | ROS 2 **Humble** (primary) and Jazzy (CI) |
| Reference apps | `toy_patrol` (no simulator), `turtlesim`, and the drone swarm, built last ([examples/drone_swarm/](examples/drone_swarm/)) |
| Review history | [REVIEW.md](REVIEW.md); A-numbers below refer to it |

---

## 0. Quickstart (the target user experience)

```bash
rosdep install --from-paths src -y && pip install -r src/jev_ai/requirements.txt
export TYPESAFE_API_KEY=...  ANTHROPIC_API_KEY=...        # env only; never ROS params
ros2 launch jev_ai_bringup bringup.launch.py               # jev_server + llm_server (+ decide_server, logger)
ros2 jev status                                            # keys, endpoints, models, budgets
ros2 jev ask --catalog my_catalog.yaml --set next_step --state task="patrolling" battery="low (18%)"
```

**Python, in-process.** Attaches to any `rclpy` node, including your existing one:

```python
from jev_ai import DecisionEngine
self.engine = DecisionEngine(self, "my_catalog.yaml",
                             encode_state=self.encode, mask=self.mask, on_decision=self.act)
```

**C++.** A thin client of `decide_server`:

```cpp
jev_ai::DecisionClient decide(node, "next_step");
decide.on_decision([&](const jev_ai::Decision& d){ act(d.selected("next_action")); });
decide.request(state, mask);            // or decide.start(1.0 /*Hz*/, encode, mask)
```

**Behaviour trees.** BehaviorTree.CPP v4:

```xml
<Decide question_set="next_step" state="{state}" selected="{next_action}"/>
```

Mock mode needs no keys, but **only when you ask for it**: `backend:=mock`. A live backend without a key fails at configure time with a clear error.

## 1. Concepts

| Concept | Definition |
|---|---|
| **Question** | One typed decision, answered as a **categorical distribution** over ordered **classes**. The three types are **choice** (pick one of up to 255 options), **score** (2–10 ordered levels) and **noul** (yes/no). |
| **Question set** | Questions asked together against one state: one Jev request and at most one LLM call. It is the unit of a decision. |
| **Catalog** | YAML that declares question sets: instructions, classes, criteria, how answers are acted on, reversibility, fallbacks, coupling, and the bounded parameters the LLM may propose. It is versioned and hashed. |
| **State** | `KeyValue[]` of short interpreted strings produced by the app's encoder, e.g. `energy = "tight (38% left, 31% needed)"`. Numbers are computed in code. |
| **Mask / require** | App callbacks. `mask` lists classes that are forbidden now; `require` forces a class deterministically, bypassing both models. Both are absolute. |
| **Fusion policy** | Combines the Jev distribution with the LLM preference: confidence-weighted, bounded log-linear pooling plus calibrated biases (§6). |
| **Coupled sampling** | Gumbel-max with noise shared within a **group**. The per-question **coupling ρ** sets how often an agent uses the group noise instead of its own. |
| **Commitment** | `PROVISIONAL` → (`REVISED` \| `KEPT`) → `COMMITTED`, plus `FALLBACK` and `FORCED`. Irreversible classes need augmentation and a margin. |
| **Engine** | `DecisionEngine`: the single implementation of §5–§7. It runs in-process (Python) or hosted by `decide_server` for remote clients (C++, BT, other languages). |
| **Backend** | A named plugin behind a server. Jev: `typesafe`, `llama_ros`, `mock`, `replay`. LLM: `anthropic`, `openai_compat`, `mock`, `replay`. |
| **Report / Directive** | The optional hierarchy module (`jev_ai.hierarchy`): workers report upward and supervisors direct downward. |
| **DecisionRecord** | The full trace of one decision. It includes the noise and propensities that off-policy evaluation needs. |

## 2. Architecture and packages

```
 existing user node ──┬── DecisionEngine (in-process, Python) ─────────────┐
 C++ node / BT / CLI ─┴── Decide.action ──▶ decide_server (hosts engines) ─┤
                                                                           │ async, deadlines
                         JevDecide.srv ──▶ jev_server  [lifecycle]  ◀──────┤ generation ids
                         Reason.action ──▶ llm_server  [lifecycle]  ◀──────┘
                                              │ backends (plugins), budgets, diagnostics
                         DecisionRecord ──▶ decision_logger ──▶ rosbag2 / JSONL ──▶ ros2 jev calibrate | replay | ope | search
```

| Package | Build | Contents |
|---|---|---|
| `jev_ai_msgs` | ament_cmake | all interfaces (§3) |
| `jev_ai` | ament_python | `jev_ai.core`: pure, no rclpy. Catalog and validator, fusion, coupled sampler, commitment machine, triggers, rate limiter, prompt and schema builder, calibration and OPE maths, golden vectors. `jev_ai.ros`: `DecisionEngine`, `Agent` (optional wrapper), `hierarchy`, backend interfaces |
| `jev_ai_servers` | ament_python | lifecycle nodes `jev_server`, `llm_server`, `decide_server`, `decision_logger`; built-in backends; parameters via `generate_parameter_library` |
| `jev_ai_cpp` | ament_cmake | `jev_ai::DecisionClient`, `JevClient`, `ReasonClient` (async, deadlines), a component example, `visibility_control.h` |
| `jev_ai_bt` | ament_cmake | BehaviorTree.CPP v4 plugins: `JevAsk`, `Decide`, `Reason` (BehaviorTree.ROS2 wrappers, vendored) |
| `ros2jev` | ament_python | ros2cli extension: `ros2 jev status \| ask \| calibrate \| replay \| ope \| search` |
| `jev_ai_bringup` | ament_cmake | `bringup.launch.py`, default `params.yaml`, catalog template |
| `jev_ai_examples` | mixed | `toy_patrol` (Python, C++ and BT agents, plus a supervisor), `turtlesim` (visual) |
| `deploy/` (not a ROS package) | — | Dockerfiles per distro, compose, kustomize, Discovery Server config. Optional, never required by the library |

**One engine (D2).** There is exactly **one** decision-engine implementation, in Python.
- Python users run it in-process.
- C++, BT, Nav2 and anything else reach the *same* engine through `decide_server`, which costs about 1–3 ms of extra ROS hop against roughly 100 ms for Jev.
- Full capability parity is therefore guaranteed by protocol, not by duplicated code.
- C++ owns only async clients and a timer-driven convenience loop.

## 3. Interfaces (`jev_ai_msgs`)

All interfaces carry `INTERFACE_VERSION` constants. Names are **relative** (see §10 for QoS).

```
# msg/Question.msg
uint8 NOUL=0
uint8 CHOICE=1
uint8 SCORE=2
string name
uint8 type
string instructions
string[] classes                 # CHOICE options / SCORE levels (ordered) / NOUL ["true","false"]
string[] criteria                # aligned; "" = none

# msg/Distribution.msg
string question
uint8 type
string[] classes
float64[] probabilities          # aligned, sums to 1
float64 confidence               # framework: 1 - H(p)/log|A|  (same meaning for all types)
float64 source_confidence        # what the model itself reported (NaN if none)

# srv/JevDecide.srv
diagnostic_msgs/KeyValue[] state
jev_ai_msgs/Question[] questions
string model                     # "" = server default
---
bool success                     # false only if the whole call failed
string error
string model                     # exact version that answered
jev_ai_msgs/Distribution[] answers
string[] question_errors         # aligned with questions; "" = ok (per-question fault isolation)
float64 latency_ms
uint32 input_tokens

# action/Reason.action
string system                    # stable per caller → prompt-cached
string context                   # free text / JSON for the model
string response_schema           # JSON Schema; "" = free text
string model                     # "" = server default
diagnostic_msgs/KeyValue[] options   # provider options, e.g. effort=low, max_tokens=800
bool stream_text                 # false → coarse feedback only
---
bool success
string error
string text
string json
string model
uint32 input_tokens
uint32 output_tokens
float64 cost_usd
float64 latency_ms
---
string stage                     # queued | started | first_token | streaming | done
string delta                     # only when stream_text

# action/Decide.action  — remote access to a hosted DecisionEngine
string agent                     # engine instance key (e.g. node FQN)
string question_set
diagnostic_msgs/KeyValue[] state
jev_ai_msgs/ClassRef[] mask      # forbidden (question, class)
jev_ai_msgs/ClassRef[] require   # forced (question, class)
jev_ai_msgs/ClassRef[] current   # current class per question (for hysteresis)
string[] triggers
string subject
string directive_issuer          # set when answering a directive (coupling epoch)
uint64 directive_id
---
jev_ai_msgs/Decision decision    # final: COMMITTED | KEPT | FALLBACK | FORCED
---
jev_ai_msgs/Decision decision    # PROVISIONAL, then REVISED/KEPT as augmentation arrives

# msg/ClassRef.msg
string question
string cls

# msg/Decision.msg
string PROVISIONAL=PROVISIONAL   # also REVISED, KEPT, COMMITTED, FALLBACK, FORCED (string constants:
                                 # readable in records/CLI, comparable as Decision::COMMITTED in C++)
uint64 decision_id
uint64 generation
string phase                     # PROVISIONAL | REVISED | KEPT | COMMITTED | FALLBACK | FORCED
jev_ai_msgs/ClassRef[] selected
jev_ai_msgs/Distribution[] fused
diagnostic_msgs/KeyValue[] values    # expectation/threshold outputs, e.g. urgency.level_mean=0.62
string llm_params                # JSON, clamped
string rationale

# msg/QuestionTrace.msg  — everything needed for replay, calibration and OPE
string question
string[] classes
float64[] p_jev
float64 jev_confidence           # framework c
float64[] q_llm                  # empty if none
float64 llm_age_s
float64 beta
bool[] masked
string required                  # "" or forced class
float64[] logits                 # final z (§6), pre-temperature
float64 tau
float64 kappa
float64[] noise                  # Gumbel g per class
bool group_noise                 # coupling draw used group noise
string selected
float64 propensity               # P(selected | logits, τ, κ) — for OPE
bool irreversible_path           # chosen via the deterministic irreversible rule

# msg/DecisionRecord.msg
std_msgs/Header header           # node clock (sim time ok)
builtin_interfaces/Time wall_stamp
string run_id
string agent
string group
string question_set
uint64 decision_id
uint64 generation
string phase
string[] triggers
string subject
diagnostic_msgs/KeyValue[] state
jev_ai_msgs/QuestionTrace[] questions
string llm_params
string llm_rationale
string[] llm_flags
string catalog_hash
string policy_hash
string jev_model
string llm_model
float64 jev_latency_ms
float64 llm_latency_ms
uint32 jev_input_tokens
uint32 llm_input_tokens
uint32 llm_output_tokens
float64 cost_usd

# msg/Report.msg  (hierarchy module)
std_msgs/Header header
string agent
string kind
string subject
float64 priority
jev_ai_msgs/Distribution[] beliefs
string payload                   # JSON, app-defined

# msg/Directive.msg  (hierarchy module)
std_msgs/Header header
string issuer
uint64 directive_id
string target                    # agent or "*"
string kind
builtin_interfaces/Time expires
string payload                   # JSON, app-defined

# msg/Label.msg  (ground truth for calibration)
std_msgs/Header header
string run_id
string subject
string question
string true_class
uint64 decision_id               # 0 = join by subject + time window
string source
```

## 4. Catalog

```yaml
catalog_version: 1
name: patrol
question_sets:
  next_step:
    state_keys: [task, energy, link, hazards, detection]   # exact, ordered, validated
    tick_hz: 1.0                       # 0 = event-driven only
    augment: true
    group: fleet                       # coupling group (default: the agent itself)
    questions:
      next_action:
        type: choice
        instructions: "Choose the robot's next action."
        act: sample                    # sample | argmax | expectation | threshold | none
        classes:                       # status quo first (Jev ordering; bias measured by replay)
          - {name: continue, criteria: "Keep doing the assigned task.", reversible: true}
          - {name: inspect,  criteria: "Look closer at the detection.", reversible: until_commit}
          - {name: halt_now, criteria: "Emergency stop; critical failure only.", reversible: false}
        fallback: continue             # must be reversible; never masked (§11)
        tau: 0.5
        kappa: 0.7                     # hysteresis / status-quo bonus (nats)
        coupling: 0.5                  # ρ (D3): 1.0 default when answering a directive
        argmax_margin: 0.1             # hysteresis on argmax_change triggers
      urgency:
        type: score
        instructions: "How urgent is resolving the detection?"
        classes: [{name: none}, {name: low}, {name: high}]
        act: expectation               # → values: urgency.level_mean, urgency.level_median
      report_now:
        type: noul
        instructions: "Should the robot report now?"
        criteria: {"true": "Something changed worth reporting.", "false": "Nothing new."}
        act: threshold
        threshold: 0.5
    llm_params:
      inspect_altitude_m: {min: 8, max: 15, default: 10}
    llm:
      system: "You are the onboard reasoning module of {agent}. …"
policy: {}                             # overrides of §14
```

**Startup validation** (the engine refuses to start on any failure):
- question types, class counts and the `act`/type combination;
- the fallback is reversible;
- irreversible classes are not the fallback;
- irreversible classes only in `act: sample|argmax` questions of sets with `augment: true`;
- `T_wait ≥ llm_timeout + 0.5 s`;
- the encoder's keys equal `state_keys`;
- `llm_params` bounds are valid;
- coupling ρ ∈ [0, 1].

The catalog and policy hashes go into every record.

## 5. DecisionEngine

### 5.1 API

```python
DecisionEngine(node, catalog,                     # any rclpy Node / LifecycleNode
               encode_state: Callable[[str], dict[str, str]],
               mask:    Callable[[str], dict[str, set[str]]] = None,
               require: Callable[[str], dict[str, str]] = None,
               on_decision: Callable[[Decision], None] = None,
               jev="jev/decide", llm="llm/reason",  # relative names, remappable
               group=None, run_id=None)
engine.trigger(name, qset=None, priority="normal")  # app events
engine.completed(question)                          # current action finished → decision event
engine.commit_point(question)                       # `until_commit` class becomes irreversible
```

- `Agent` is a convenience `Node` subclass that wires one engine. It is optional.
- `decide_server` hosts one engine per `agent` key. For those engines, the encoder, mask and require inputs come from the goal, and decisions go out as action feedback and result.

### 5.2 Decision cycle

For each question set, on a tick or a trigger. Each set runs serially in its own MutuallyExclusive callback group (A4); service and action calls are async only (A5).

1. **State:** `state = encode_state(qset)`, validated against `state_keys`.
2. **Jev:** `JevDecide` (async, deadline `jev_timeout`, generation g) returns p_J per question. Per-question errors keep that question's current class.
3. **Masks:** `mask()` and `require()` are evaluated *fresh*, now and again before every act (A4).
4. **Fuse:** fuse with the cached LLM preference q_L (§6) if its `state_hash` and `mask_hash` match. Otherwise only the age-decayed q_L is used, and only when `reuse_stale_llm` is true (default false).
5. **Select:** select per §6.3 and §6.4, then commit per §5.3, then call `on_decision`.
6. **Augment:** if an augmentation trigger fired, the budget allows it and `c < skip_llm_above_c` (default off: always augment), send an async `Reason` goal tagged `(decision_id, g, state_hash, mask_hash)`.
7. **Revise:** when the LLM result arrives, discard it if g has advanced, or if the masks recomputed now differ from the tagged hash. Otherwise re-fuse with the **same noise** and run the commitment machine.
8. **Record:** publish a `DecisionRecord` at every phase transition.

### 5.3 Commitment

```
 decision event
   │
   ├─ require() forces a class → FORCED (no models consulted)
   │
   ├─ deterministic irreversible rule (§6.4) proposes an irreversible class?
   │      yes → hold current; request augmentation (reserved budget slot); wait ≤ T_wait
   │            ├─ augmented & rule still holds → COMMITTED
   │            └─ otherwise / timeout          → FALLBACK (catalog fallback; logged)
   │
   └─ sample among reversible classes → PROVISIONAL → on_decision
          LLM result → re-fuse with same noise →
             same argmax → KEPT           different → REVISED → on_decision
          (no voluntary switch within T_dwell; mask-forced switches immediate)
```

**Decision events** are the only times sampling happens:
- Jev's argmax changes by more than `argmax_margin`;
- the current class becomes masked;
- an app trigger fires;
- `completed()` is called;
- an LLM result arrives;
- a directive arrives.

### 5.4 Time

- **Policy timers** (`T_dwell`, `T_wait`, `T_stale`, `T_refresh`, ticks, autonomous epochs) run on the **node clock**, so sim time works.
- **API deadlines** run on **wall time**.
- Records carry both clocks.
- The validator warns when `use_sim_time` is set and the sim's real-time factor makes `T_wait` shorter in wall time than `llm_timeout`.

## 6. Fusion and selection

### 6.1 Confidence

For every question type:

`c = 1 − H(p_J) / log|A|`

where H is the entropy of p_J over its |A| classes. The model's own confidence is kept as `source_confidence`, and it can enter calibration as a feature (A9).

### 6.2 Pooling

Let U be the set of unmasked classes.

```
q_U(a)  = q_L(a) / Σ_{b∈U} q_L(b)                         renormalise over unmasked classes
ℓ(a)    = log max(q_U(a), η) − mean_{b∈U} log max(q_U(b), η)  centred LLM log-preference
β(c)    = β_min + (β_max − β_min)(1 − c)^γ
β_eff   = β(c) · exp(−age / T_stale)
δ(a)    = clip(β_eff · ℓ(a), −B, +B)                       LLM influence bounded in nats
δ(a)    = min(δ(a), 0)          if a is irreversible        one-sided: the LLM may only lower irreversible classes
z(a)    = log p_J(a) + δ(a) + b_q(a)                        b_q: calibrated per-class bias (default 0)
z(a)    = −∞                    if a is masked
```

- With no q_L, δ ≡ 0.
- **Bounded influence:** the LLM can move any class by at most B nats in either direction. With B = log 20, that is a 20× odds change. Only masks and `require` are absolute.
- `b_q` corrects the prior that Jev and the LLM both count, plus any residual class bias. It is fitted by `calibrate` (§12).

### 6.3 Coupled selection (Gumbel-max; D3)

```
noise key   k = group                    with prob ρ   (decided by H(run_id, agent, question, epoch) < ρ)
              = agent                    otherwise
epoch       = (directive_issuer, directive_id)   when answering a directive (ρ default 1.0)
            = floor(t_node / epoch_s)            otherwise                  (ρ default 0.5)
g(a)        = −log(−log U(a)),  U(a) = H(run_id, k, question, epoch, a) ∈ (0,1)
s(a)        = (z(a) + κ·1[a = current]) / τ + g(a)       over reversible, unmasked classes
selected    = argmax_a s(a)
propensity  = softmax((z + κ·1[current]) / τ)(selected)
```

Properties:
- **Order-free:** catalog order no longer affects sampling.
- **Stable under masking:** masking a class that would not have won changes nothing.
- **Minimal disagreement:** agents with fused distributions δ apart in total variation disagree with probability ≤ 2δ/(1+δ), which is optimal for coupling without communication.
- **Minimal revisions:** the provisional Jev-only decision and the augmented decision reuse the **same** g, so a revision happens only when augmentation changed the distribution enough.
- **Reproducible:** g is a pure function of logged keys, so every decision can be re-derived exactly.

Status-quo preference is expressed **explicitly** through κ. Catalog order still puts the status quo first, because Jev's first-option bias is measured and gated by `replay`.

### 6.4 Irreversible classes and non-sampled acts

- **Irreversible classes** (`reversible: false`, or `until_commit` after `commit_point`) are **never sampled**. A class a is chosen only if all three hold:
  - a = argmax z;
  - z(a) − max_{b≠a} z(b) ≥ `m_irrev` (default log 2);
  - augmentation for this decision has arrived, or `require()` forced it.

  Otherwise it is excluded from §6.3. Real emergencies use `require()`, which is deterministic and never waits on an API.
- **`argmax`:** argmax of z.
- **`expectation`** (score questions): reports `level_mean` = Σ ℓ·π(ℓ)/(L−1), `level_median`, and the full π. Apps decide which one to use.
- **`threshold`** (noul questions): π(true) ≥ θ.

## 7. LLM augmentation contract

- **Triggers:**

  | Trigger | Priority |
  |---|---|
  | `argmax_change` | normal |
  | `low_confidence` (c < `c_low`) | high |
  | `irreversible_pending` | reserved slot |
  | `periodic` (`T_refresh`) | low |
  | `directive` / `report` | normal |
  | `app:<name>` | app-set |

- **Rate limiting:** a per-engine token bucket (`k_max`/min), with coalescing. Global limits live in the server (§9).
- **Request:**
  - **`system`:** the catalog template, the class vocabulary with criteria, mask semantics and the objective. It is stable, so it is prompt-cached.
  - **`context`:** the state, Jev distributions, masks, current classes and the last 3 decisions with outcomes. Fields that come from sensors or other agents are wrapped in `<untrusted>` delimiters and labelled as data, never instructions (A26).
  - **Class order:** classes are **shuffled per call** with a seeded order and mapped back, so LLM position bias does not stack on Jev's (A11).
  - **`response_schema`:** generated from the catalog:

    ```json
    {"weights": {"<question>": {"<class>": 0..1}}, "params": {...bounded...},
     "rationale": "≤2 sentences", "flags": ["novel_situation"|"option_missing"|"data_suspect"]}
    ```

- **Response handling:**
  - One bounded retry on schema failure; then δ ≡ 0.
  - `params` are clamped to `llm_params`.
  - `option_missing` is surfaced to the app.
- **Feedback:** coarse only (`stream_text = false`) for augmentation (A27).

## 8. Jev server (`jev_server`, lifecycle)

- **Parameters** (generate_parameter_library):
  - `backend` (name)
  - `<backend>.plugin`
  - `<backend>.model` (pinned, e.g. `jev-1.13.0`)
  - `<backend>.base_url` (TypeSafe or OpenRouter)
  - `timeout_s` 1.0
  - `max_retries` 1
  - budgets (§9)
- **Backends**, as Python entry points in the group `jev_ai.jev_backends`:

  | Backend | Behaviour |
  |---|---|
  | `typesafe` | the `typesafe-sdk` async client. OpenRouter is the same backend with `base_url=https://openrouter.ai/api` |
  | `llama_ros` (D4) | adapter to `llama_msgs/srv/EvaluateDecisions`, for local GGUF decision models, offline and with no API key |
  | `mock` | deterministic. Seeded Dirichlet over (state, question), with a configurable first-class bias and optional rule hooks |
  | `replay` | answers from records, keyed by hash(state, questions) |

- **Keys:** read from `TYPESAFE_API_KEY` or a `api_key_file` path. Never from params.
- **Lifecycle:** `on_configure` validates the key and the `/v1/models` endpoint.
- **Threading:** HTTP runs on a dedicated asyncio thread, bridged to ROS futures.
- **Diagnostics:** latency, errors, rate headroom, budget state.

## 9. LLM server (`llm_server`, lifecycle)

- **Backends**, entry points in the group `jev_ai.llm_backends`:

  | Backend | Behaviour |
  |---|---|
  | `anthropic` (default) | native SDK, `messages.stream`. Maps `options.effort` to `output_config.effort`, `response_schema` to `output_config.format`, and puts `cache_control` on `system` |
  | `openai_compat` | OpenAI, Gemini, Ollama, vLLM via `base_url`. Uses `response_format: json_schema` where supported, else JSON mode plus validation |
  | `mock` | schema-valid output by echoing context distributions, with seeded disagreement |
  | `replay` | answers from records |

- **Defaults:** `model` and `effort` are per server instance, e.g. one `llm_server` with Haiku `low` and one with Sonnet `medium`. Clients pick a server by remapping.
- **Budgets (both servers, A25):**
  - `max_rps`, `max_tokens_per_min`, `max_usd_per_hour`, from a pricing table in config.
  - A circuit breaker opens after N errors or budget exhaustion. While it is open the server rejects goals fast, and engines degrade to δ ≡ 0 (Jev only) or keep the current class (Jev).
  - Budget state is published in diagnostics and checked by `ros2 jev status`.
- **Cancellation:** a cancelled goal aborts the HTTP stream.

## 10. Topics, QoS, security

| Endpoint (relative) | Type | QoS |
|---|---|---|
| `jev/decide` | service | default services |
| `llm/reason`, `decide` | actions | default actions |
| `decisions` | DecisionRecord | reliable, keep_last 1000, volatile |
| `reports` | Report | best-effort, keep_last 50 |
| `directives` | Directive | reliable, keep_last 50, volatile; receivers drop expired or duplicate `(issuer, id)` messages, and enforce an `issuer_allowlist` |
| `labels` | Label | reliable, keep_last 1000 |

- **Data egress:** state and questions leave the robot for TypeSafe, OpenRouter or Anthropic. Use `llama_ros` and a local `openai_compat` model for air-gapped or ITAR/GDPR deployments.
- **DDS security:** SROS2 guidance for the directive and report topics.

## 11. Safety invariants

| Invariant | Mechanism |
|---|---|
| A masked class is never selected | z = −∞ after fusion; masks are recomputed before every act |
| Irreversible actions need System 2 input and a margin, or a deterministic `require` | §6.4, §5.3 |
| An LLM timeout never commits an irreversible class | the timeout path is FALLBACK, and the fallback must be reversible |
| The fallback is always executable | validated as reversible; a fallback entry returned by `mask()` is ignored with an error log (apps use `require` for real overrides) |
| The LLM's influence is bounded | \|δ\| ≤ B; one-sided on irreversible classes |
| No unbounded LLM parameters reach the app | `llm_params` clamping |
| Stale or racing results are never applied | generation, state hash and mask hash checks |
| No silent mock | live backends fail at configure time without a key |
| Spend is bounded | server budgets plus the circuit breaker |

| Failure | Behaviour |
|---|---|
| LLM timeout, error, invalid schema or open breaker | δ ≡ 0 (Jev only); recorded |
| One Jev failure | keep the current class |
| N_fail (3) consecutive Jev failures | fallback class; `on_degraded` callback |
| Both systems down | hold the fallback; the app decides any domain failsafe |

## 12. Records, calibration and evaluation (`ros2jev`)

- **`decision_logger`:** subscribes to `decisions` and `labels` and writes **rosbag2** (default) or JSONL. Prompts appear at DEBUG only.
- **`ros2 jev calibrate`:** a convex **conditional-logit** fit per question type, with γ on a grid.
  - Features: log p_J, ℓ, (1−c)^γ·ℓ, source_confidence and per-class biases b_q.
  - It reports ECE and NLL for Jev alone, the LLM alone and the fused result, and emits a `policy:` patch (β_min, β_max, γ, b_q, B).
- **`ros2 jev replay`:** re-asks Jev or the LLM on logged states with a variant catalog (wording, criteria, class order). It reports the shift, the labelled accuracy and **first-option bias**, and this is the gate for the status-quo-first ordering.
- **`ros2 jev ope`:** recomputes z, s and propensities for new (β, γ, τ, κ, ρ, B) **exactly** from the logged inputs and noise.
  - Where the new action equals the logged one, the outcome is known exactly.
  - Otherwise it uses SNIPS or doubly robust estimates with a cluster bootstrap by group, and reports the effective sample size.
- **`ros2 jev search`:** episode-level search (successive halving, common random numbers) through an app `EpisodeRunner`, only for parameters that OPE can't cover (dwell, deadlines, trigger cadence).
- **Replay claim:** *per-decision* re-derivation is exact. Whole-run reproduction is not claimed.

## 13. Deployment (`deploy/`, optional)

- **Images:** per-distro images on `ros:<distro>-ros-base` with pinned SDKs from `requirements.txt`. A custom rosdep source provides `python3-anthropic-pip` and `python3-typesafe-sdk-pip`; there is a PEP 668 note for Jazzy.
- **Compose:** `x-jev-ai-node` anchor, plus `jev_server`, `llm_server`, `decide_server` and `decision_logger`, plus an optional Fast DDS Discovery Server. Apps `include:` it.
- **Kustomize:** a base (servers, logger, Secret, ConfigMap) plus components for app agents (StatefulSets, where the ordinal gives the agent name). Keys come from a k8s Secret as env vars or a mounted file.

## 14. Parameters (defaults; catalog `policy:` and ROS params override)

| Parameter | Default | Range | Tuned by |
|---|---|---|---|
| `beta_min`, `beta_max`, `gamma` | 0.1, **1.0**, 1.0 | [0, 0.5], [0.5, 3], [0.5, 3] | calibrate |
| `B` (LLM bound, nats), `eta` (log floor) | log 20, 1e-3 | [log 2, log 100] | calibrate |
| `tau` | 0.5 | [0.05, 1.5] | ope |
| `kappa` | 0.7 nats | [0, 3] | ope |
| `coupling` ρ | 1.0 (directive), 0.5 (autonomous) | [0, 1] | ope |
| `epoch_s` | 5 s | | app |
| `m_irrev` | log 2 | | ope |
| `argmax_margin` | 0.1 | | search |
| `T_dwell`, `T_stale`, `T_wait`, `T_refresh` | 5, 30, *llm_timeout + 0.5*, 20 s | | search |
| `k_max` (per engine), `c_low`, `skip_llm_above_c` | 6/min, 0.5, off | | search |
| `reuse_stale_llm` | false | | |
| `jev_timeout`, `llm_timeout`, `N_fail` | 1 s, 6 s, 3 | | |
| server `max_rps`, `max_tokens_per_min`, `max_usd_per_hour` | 50, 200k, 5.00 | | ops |

## 15. Non-goals (v1)

- **Online learning** of the policy. OPE plus offline calibration covers tuning.
- **Multi-step LLM tool use** inside augmentation.
- **A general planner:** apps call `Reason` directly for planning, as the drone example does.
- **A Nav2 Humble BT v3 plugin flavour.** Deferred; the v4 BT nodes ship first.
