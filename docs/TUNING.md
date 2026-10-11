# Tuning the System 1 / System 2 balance

Every knob below lives in the catalog, as YAML. None needs code. Old catalogs keep working, because every addition defaults to the previous behaviour.

## 1. The dials

| Dial | Question it answers | Knobs |
|---|---|---|
| **LLM use** | when does System 2 get asked? | a set's `llm.use`, a question's `llm`, `skip_llm_above_c`, `c_low`, `k_max` |
| **LLM weight** | how much does its answer count? | `beta_min`, `beta_max`, `gamma`, `B` (fitted by `ros2 jev calibrate`) |
| **Jev cadence** | when is System 1 asked? | `tick_hz`, `ask_jev`, `T_refresh`, triggers |
| **Stickiness** | how easily does a choice change? | `kappa`, `T_dwell`, `argmax_margin` |
| **Exploration** | how random is the pick? | `act`, `tau` |
| **Coordination** | how alike are a fleet's picks? | `group`, `coupling`, `coupling_directive`, `epoch_s`, a shared `run_id` |
| **Safety** | what can never happen? | `mask()`, `require()`, `reversible`, `fallback` |
| **Deadlines** | how long to wait? | `jev_timeout`, `llm_timeout`, `T_wait`, `N_fail` |

## 2. Goal → what to change

| I want to… | Change |
|---|---|
| **spend less on the LLM** | Put `llm: when_unsure` on the questions that decide actions, and leave out yes/no and score questions, since Jev is rarely confident on those. Lower `k_max`. Or set `llm.use: never` on sets that don't need System 2 |
| **ask the LLM about everything** | `llm.use: always` |
| **make Jev calls cheaper** | `ask_jev: change` (ask only when the state's wording changes). Lower `tick_hz`, or set it to 0 and use triggers. Keep sets small, and don't ask questions you never read |
| **act faster** | `preset: fast`. Lower `T_dwell` and `kappa`. The robot already acts on Jev without waiting; `llm_timeout` only bounds the revision |
| **flip-flop less** | Raise `kappa` (0.7 → 1.0–1.5) and `T_dwell`. `preset: careful` |
| **be more decisive or deterministic** | Lower `tau` (0.5 → 0.2), or `act: argmax` (no noise) |
| **make a fleet agree more** | Same `group` and `run_id` everywhere, and `coupling` → 1.0. Directives already default to 1.0 |
| **make a fleet spread out** | `coupling` → 0, so each agent uses its own noise |
| **protect a dangerous action** | `reversible: false` (Jev and the LLM must both rank it first), plus `require()` for the real emergencies |
| **stop wasting time waiting on the LLM** | Lower `llm_timeout` (it is sent to the server as the call's deadline) |
| **use sim time faster than real time** | `T_wait` runs on sim time, so scale it: `llm_timeout × real_time_factor + 0.5` |

## 3. Presets

`policy: {preset: <name>, ...}`. Any key next to `preset` overrides it.

| Preset | Sets | Use when |
|---|---|---|
| `balanced` (default) | the defaults in §4 | you are starting out |
| `fast` | `tau 0.3, kappa 0.4, T_dwell 2, k_max 3, ask_jev change` | many agents, tight loops, cost matters |
| `careful` | `tau 0.3, kappa 1.0, T_dwell 8, k_max 12, llm_timeout 10` | few, high-stakes decisions |

## 4. Reference

### Question set (`question_sets.<name>`)

| Key | Default | |
|---|---|---|
| `state_keys` | required | the exact keys `encode_state` returns |
| `questions` | required | see below |
| `tick_hz` | 0 | Jev asks per second on the node clock; 0 = triggers only |
| `llm.use` | *(unset: augment, with each question's own gate)* | `never`: Jev only. `when_unsure`: every question without its own `llm` gates at `c_low`. `always`: ignore the gates |
| `llm.system` | `''` | the LLM's system prompt; `{agent}` is substituted |
| `augment` | true | the older spelling of `llm.use: never` (as `false`) |
| `group` | the agent | the coupling group |
| `llm_params` | none | `{name: {min, max, default}}` numbers the LLM may propose; clamped |

### Question (`questions.<name>`)

| Key | Default | |
|---|---|---|
| `type` | required | `choice` \| `score` \| `noul` |
| `instructions` | required | the question, for both models |
| `classes` | required (noul: implicit true/false) | `{name, criteria, reversible}`, status quo first |
| `criteria` | | noul only: `{"true": ..., "false": ...}` |
| `act` | choice/score `sample`, noul `threshold` | `sample` \| `argmax` \| `expectation` (score) \| `threshold` (noul) \| `none` |
| `fallback` | the first class (noul: `false`) | must be reversible |
| `threshold` | 0.5 | noul with `act: threshold` |
| `llm` | *(set's)* | `when_unsure`: the LLM is asked only if this question's confidence < `skip_llm_above_c` (or `c_low`). `always`: this question never blocks the call |
| overrides | policy | `tau`, `kappa`, `coupling`, `coupling_directive`, `argmax_margin`, `skip_llm_above_c`, and the calibrated `beta_min`, `beta_max`, `gamma`, `B`, `eta`, `bias` |

One LLM call answers a whole set. It is skipped only when **every** gating question is confident; a set with no gates always asks. Confidence is `c = 1 − entropy/log(#classes)`: 0 is a uniform answer, 1 is certain.

### Policy (`policy:`)

| Key | Default | Range | What it does |
|---|---|---|---|
| `preset` | `balanced` | | §3 |
| `tau` | 0.5 | 0.05–1.5 | pick temperature; lower = more deterministic |
| `kappa` | 0.7 nats | 0–3 | bonus for keeping the current class |
| `T_dwell` | 5 s | > 0 | no voluntary switch sooner than this |
| `argmax_margin` | 0.1 | 0–1 | how far Jev's top class must move to count as a change |
| `coupling` / `coupling_directive` | 0.5 / 1.0 | 0–1 | share of decisions that use the group noise |
| `epoch_s` | 5 s | > 0 | how long a noise draw lasts for autonomous decisions |
| `beta_min`, `beta_max`, `gamma` | 0.1, 1.0, 2.0 | | LLM weight from Jev's confidence: β(c) = β_min + (β_max − β_min)(1 − c)^γ |
| `B` | log 20 | log 2–log 100 | the most the LLM can move a class, in nats (log 20 = 20× odds) |
| `eta` | 1e-3 | | log floor on LLM weights |
| `T_stale` | 30 s | > 0 | an old LLM answer's weight decays with this |
| `reuse_stale_llm` | false | | fuse an LLM answer from a different state, with decay |
| `c_low` | 0.5 | 0–1 | below it the LLM request is high priority; also the default `when_unsure` gate |
| `skip_llm_above_c` | off | | the gate (see `llm` above) |
| `k_max` | 6 / min | > 0 | LLM requests per minute per engine |
| `ask_jev` | `tick` | tick \| change | `change`: a tick asks Jev only if the encoded state reads differently once its numbers are masked. Triggers and `T_refresh` still ask |
| `T_refresh` | 20 s | > 0 | a periodic decision even when nothing changed |
| `jev_timeout` | 1 s wall | > 0 | the engine gives up on a Jev answer |
| `llm_timeout` | 6 s wall | > 0 | the LLM call's deadline (sent to `llm_server`, capped by its `timeout_s`) |
| `T_wait` | `llm_timeout` + 0.5 | ≥ that | node-clock wait for the LLM before committing |
| `N_fail` | 3 | > 0 | consecutive Jev failures before the fallback |

### Servers

Set servers through `params_file` (`jev_ai_bringup/config/params.yaml`). Parameters are read-only after launch.

| Server | Key | Default | |
|---|---|---|---|
| both | `backend` | launch arg | `jev_backend` / `llm_backend` |
| both | `model` | backend default | e.g. `claude-haiku-5-5`; pin a Jev version when tuning |
| both | `timeout_s` | jev 1.0, llm 6.0 | the deadline cap |
| both | `max_retries` | 1 | within the deadline |
| both | `max_rps` | jev 50, llm 10 | requests per second |
| both | `breaker_errors`, `breaker_cooldown_s` | 5, 30 | the circuit breaker; while open, engines run Jev-only or hold |
| both | `record_path` | `''` | record a cassette for the `replay` backend |
| both | `api_key_file` | `''` | instead of the environment variable |
| `llm_server` | `effort`, `max_tokens` | low, 1024 | `max_tokens` is a ceiling a call may lower |
| `llm_server` | `pricing` | built-in table | USD per Mtok, for reported spend |
| `jev_server` | `usd_per_mtok_input` | 0.042 | |
| `jev_server` | `mock.*` | | `seed`, `alpha`, `first_bias`, `latency_s`, `fail_rate`, `rules` (`module:function` for state-aware answers) |
| `llm_server` | `mock.*` | | `seed`, `disagreement`, `latency_s`, `fail_rate`, `invalid_json_rate` |
| `dashboard` | `port`, `host`, `allowed_hosts`, `max_records`, `summary` | 8080, 127.0.0.1, … | `summary`: comma list of statuses pinned to the top bar |
| `decision_logger` | `path`, `labels_path` | `~/.ros/jev_ai/…` | JSONL |

**Two LLM models:** run a second `llm_server` with another `model` under a namespace, and remap that agent's `llm` to it. The drone swarm gives the coordinator Sonnet this way, and the drones Haiku.

## 5. Measure, then tune

1. **Watch the dashboard.** The Decisions summary says how often the LLM was asked and how often its answer was used. Per question, Metrics shows how often the LLM agreed or flipped Jev's pick, and how often choices switched.
2. **Publish `Label`s** when you learn the truth, for example from a scorer or an operator.
3. **`ros2 jev calibrate decisions.jsonl --labels … --catalog …`** fits `beta_*`, `gamma`, `B` and `bias` per question, and writes a patched catalog.
4. **`ros2 jev ope … --set tau=0.3`** estimates a policy change from the logs alone, using the same noise.
5. **`ros2 jev search`** runs real episodes, for timing knobs that logs can't evaluate (`T_dwell`, `k_max`, `tick_hz`).
