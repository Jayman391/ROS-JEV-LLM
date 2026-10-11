# Building an agent with jev_ai

This guide takes you from an empty node to a fleet. Read [TUNING.md](TUNING.md) for the dials and [FRAMEWORK.md](FRAMEWORK.md) for the exact math.

## 1. The decision cycle

Each decision is one **question set**: a few typed questions asked together about one **state**. The cycle runs on a tick (`tick_hz`) or on an event you trigger:

1. **Encode:** your `encode_state(qset)` returns short strings, one per `state_key`.
2. **System 1:** Jev returns a probability for every class of every question, in about 160 ms.
3. **Hard rules:** your `mask(qset)` lists forbidden classes and `require(qset)` forces answers. Both always win.
4. **Act now:** a class is picked from Jev's probabilities, using seeded noise, temperature `tau` and a bonus for keeping the current choice (`kappa`). `on_decision` gets it as **PROVISIONAL**. The robot does not wait for the LLM.
5. **System 2:** if the set uses the LLM and Jev is unsure (your `llm` dial), the LLM reads the same state plus Jev's answer and returns its own weights.
6. **Revise:** the weights are pooled. The less confident Jev was, the more the LLM counts. The result is re-picked with the *same* noise:
   - same class → **KEPT**
   - different class → **REVISED**, and `on_decision` is called again.

| Phase | Meaning | `on_decision`? |
|---|---|---|
| `PROVISIONAL` | Jev's pick, acted on at once; the LLM was asked | yes |
| `REVISED` | the LLM changed the pick | yes |
| `KEPT` | the LLM agreed | only with `all_phases=True` (`d.act` is False) |
| `COMMITTED` | final; with no LLM asked, it is the first and only phase | yes, or `all_phases` after a revision |
| `FORCED` | `require()` decided, with no model consulted | yes |
| `FALLBACK` | the catalog `fallback`: Jev failed `N_fail` times, or an irreversible choice timed out | yes |

**Irreversible classes** (`reversible: false`) are never picked by chance. They need **two keys**: the class must be Jev's top choice *and* the LLM's top choice. Until the LLM answers, the robot holds its current class. Real emergencies belong in `require()`, which is deterministic.

## 2. Your first agent

`src/jev_ai_examples/jev_ai_examples/minimal.py`, with the catalog `src/jev_ai_bringup/config/catalog_template.yaml`:

```python
class Minimal(Node):
    def __init__(self):
        super().__init__('minimal')
        self.battery = 100.0
        self.engine = DecisionEngine(self, CATALOG, encode_state=self.encode, mask=self.mask,
                                     on_decision=self.act)
        self.create_timer(1.0, self.drain)

    def encode(self, qset):        # words for meaning, numbers for detail
        level = 'ample' if self.battery > 50 else 'low' if self.battery > 20 else 'critical'
        return {'task': 'patrolling the yard', 'battery': f'{level} ({self.battery:.0f}% left)'}

    def mask(self, qset):          # hard rule: never continue on a critical battery
        return {'next_action': {'continue'}} if self.battery <= 20 else {}

    def act(self, d):              # PROVISIONAL at once, REVISED if the LLM changed it
        if d.selected['next_action'] == 'return_home':
            ...
```

Run it with:

```bash
ros2 launch jev_ai_bringup bringup.launch.py     # jev_server, llm_server, logger, dashboard (mocks)
ros2 run jev_ai_examples minimal
```

Watch the decisions at http://localhost:8080. The mock Jev answers at random for a new catalog. For state-aware mock answers, give it `mock.rules: "my_pkg.rules:function"`, as the examples do. To go live, add `jev_backend:=typesafe llm_backend:=anthropic` to the bringup launch.

## 3. The catalog

```yaml
catalog_version: 1
name: my_robot
question_sets:
  next_step:
    state_keys: [task, battery]      # encode_state must return exactly these, as strings
    tick_hz: 1.0                     # 0 = only on triggers
    group: fleet                     # agents in one group share sampling noise (coordination)
    llm:
      use: when_unsure               # never | when_unsure | always   (TUNING.md)
      system: "You are the onboard reasoning module of {agent}."
    questions:
      next_action:
        type: choice                 # choice (2-255 options) | score (2-10 levels) | noul (yes/no)
        instructions: "Choose the robot's next action."
        classes:                     # put the status quo first
          - {name: continue,    criteria: "Keep doing the current task."}
          - {name: return_home, criteria: "Go back to base."}
          - {name: self_destruct, criteria: "Never, unless ...", reversible: false}
        fallback: continue           # reversible; used when Jev is down or a commit times out
    llm_params:                      # bounded numbers the LLM may propose (clamped)
      speed_mps: {min: 1, max: 5, default: 3}
policy: {preset: balanced}           # TUNING.md
```

**How answers are acted on (`act`):**

| `act` | Question types | Result |
|---|---|---|
| `sample` (default for choice and score) | choice, score | a class, picked with noise. Use it for actions, where coordination and exploration matter |
| `argmax` | choice, score | the most likely class, with no noise |
| `expectation` | score | `d.values['<q>.level_mean']` in [0, 1] and `level_median`. Use it for priorities |
| `threshold` | noul | true when p(true) ≥ `threshold` |

**Class `reversible`:**

| Value | Meaning |
|---|---|
| `true` (default) | an ordinary class |
| `false` | the two-key rule applies |
| `until_commit` | reversible until you call `engine.commit_point(question)`, for example once a drone starts its descent |

Validation runs at startup, and the engine refuses a bad catalog with a message that names the field. Start from `catalog_template.yaml`.

## 4. The callbacks and the engine API

```python
DecisionEngine(node, catalog,                    # path, dict or load_catalog() result
               encode_state,                     # qset -> {state_key: str}       required
               mask=None,                        # qset -> {question: {forbidden classes}}
               require=None,                     # qset -> {question: forced class}
               on_decision=None,                 # Decision -> None
               on_degraded=None,                 # qset -> None, when Jev keeps failing
               jev='jev/decide', llm='llm/reason',   # remap to pick servers, e.g. a Sonnet llm_server
               group=None, run_id=None, agent=None,  # share run_id across a fleet (noise coupling)
               policy=None,                      # policy overrides (with a path or dict catalog)
               current=None, all_phases=False)
```

**Writing `encode`:** Jev is weak at arithmetic, so compute in code and say what it means. Write `"tight (38% left, 31% needed to finish)"`, not `"38"`. Put the meaning in words and the detail in numbers. This also makes `ask_jev: change` work (TUNING.md).

**`mask` vs `require`:**
- `mask` removes options; the models choose among the rest.
- `require` decides outright and skips the models.

If masking leaves only a poor option, the leftovers are mostly noise. Use `require` to force the sensible action instead, for example "no order → hold".

**`Decision`** (what `on_decision` receives):

| Field | Meaning |
|---|---|
| `selected` | `{question: class}` |
| `values` | numeric results, e.g. `urgency.level_mean` |
| `phase`, `act`, `final` | where the decision is in the cycle. Act when `d.act`; report when `d.final` |
| `llm_params` | clamped |
| `rationale`, `flags` | from the LLM |
| `fused` | `{question: (classes, probabilities)}`, to send as beliefs |
| `degraded` | Jev is failing and the fallback is held |

**Events:**

| Call | Effect |
|---|---|
| `engine.trigger(name, qset=None, priority='', fresh=False, subject='')` | decide now; `fresh` re-encodes immediately |
| `engine.completed(question)` | the current action finished |
| `engine.commit_point(question)` | `until_commit` classes become irreversible |
| `engine.set_subject(subject)` | what the decision is about (records and labels join on it) |
| `engine.current(qset)` | the current selection |

## 5. Fleets and hierarchies

- **Coordination without messages:** agents with the same `run_id` and `group` share their sampling noise. Given similar information they make similar choices, with no messaging. `coupling` ρ sets how often each agent uses the shared noise. Pass one `run_id` to every agent in a launch.
- **Supervisor and workers:** `Hierarchy` (`from jev_ai import Hierarchy`) sends reports up and directives down:

  ```python
  h = Hierarchy(self, engine=self.engine, qset='next_step', on_directive=self.obey,
                issuer_allowlist=['/supervisor'])               # worker
  h.report('status', subject='person at 12 m', priority=0.7, beliefs=d.fused, payload={...})

  h = Hierarchy(self, agent='/supervisor', on_report=self.on_report)   # supervisor
  h.direct('converge', target='*', payload={'target': [3, 4]}, ttl_s=15)
  ```

  Each accepted directive is a decision event. Workers answering the same directive share their noise (coupling 1.0 by default), so they coordinate.
- The dashboard's **Comms** section shows every report and directive. If you relay them through a lossy link, it also shows delivery and drops.

## 6. C++, behavior trees, and other languages

There is one engine, in Python. Other languages reach it through `decide_server`, which hosts one engine per agent and adds about 1–3 ms per call. Launch bringup with `catalog:=my_catalog.yaml`, then:

```cpp
jev_ai::DecisionClient decide(node, "next_step");                    // jev_ai_cpp
decide.on_decision([&](const jev_ai::Decision& d){ act(d.selected("next_action")); });
decide.request(state, mask);
```

```xml
<Decide question_set="next_step" state="{state}" selected="{next_action}"/>   <!-- jev_ai_bt -->
```

To ask Jev or the LLM directly, without fusion, use `JevClient` / `ReasonClient` (C++) or the `JevAsk` / `Reason` BT nodes. The `turtlesim` example runs the same agent all three ways.

## 7. Running, logging, improving

- **Launch:**
  - `bringup.launch.py` starts `jev_server`, `llm_server`, `decision_logger` and `dashboard`.
  - Arguments: `jev_backend`, `llm_backend`, `params_file`, `dashboard_port`, `catalog` (adds `decide_server`).
  - Server parameters are listed in [TUNING.md](TUNING.md#servers).
- **Keys:** set `TYPESAFE_API_KEY` and `ANTHROPIC_API_KEY` in the environment (the dev container reads `.env`). A live backend without a key fails at startup; it never silently falls back to a mock.
- **Records:** every phase of every decision is a `DecisionRecord` on `/decisions`. The logger writes them as JSONL.
  - `ros2 jev report decisions.jsonl` produces an offline copy of the dashboard.
  - Publish `Label`s when you learn the truth (what happened), so you can calibrate.
- **Improving:** `ros2 jev calibrate` fits how much the LLM should count, per question. `ope` replays logged decisions under a new `tau` / `kappa` / ρ. `search` tunes timing knobs by running episodes. See [TUNING.md §5](TUNING.md#5-measure-then-tune).
