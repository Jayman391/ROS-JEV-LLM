# Assembly line: decision model

What is decided where, by whom, and why. The world is in [SYSTEM_MODEL.md](SYSTEM_MODEL.md).

## 1. Principles

- **Rules for routine work, Jev/LLM for judgement.** Rules handle:
  - routing ops to arms
  - release cadence
  - the coop handshake
  - which op a rework redoes
  - running an op whose pre-op readings are nominal
  - passing a part whose QA readings are all fine

  Jev is asked only when something is off. In a nominal run, most ops and most parts never reach
  a model.
- **Event-driven, never per tick.** Every set has `tick_hz: 0` and is triggered by an event (a
  pallet waits, an arm faults, a reading is off). `T_refresh` is set high, so idle sets stay
  quiet.
- **Act on the final decision.** A drone can change heading when the LLM revises its choice. A
  line can't take back a released pallet, a disposed part or a started weld. So line agents act
  only on a decision's `final` step (`all_phases=True`), and the part waits while the LLM is
  consulted. This is the line's version of *hold for the LLM*. The exception is `disruption`, whose
  responses are routing settings that can be revised freely, so it acts on every phase.
- **One decisive question per set**, gated with `llm: when_unsure`. Every answer is acted on.
- **Per-item status quo.** Per-part sets pass `current=` so each new part starts from the status
  quo, not from the previous part's answer. With no item in hand, `require()` forces the status
  quo (FORCED, no LLM), and actions check the decision's subject is still the item in hand.
- **Never mask without a forced way out.** When masks leave only a poor option, `require()`
  forces the safe one.

## 2. Agents

| agent | catalog | preset | LLM |
|---|---|---|---|
| `line_supervisor` | `config/supervisor.yaml` | balanced | Sonnet (`/line_supervisor/llm_server`) |
| `arm_agent` ×5 | `config/arm.yaml` | fast (`ask_jev: change`) | Haiku (`/llm_server`) |
| `qa_agent` | `config/qa.yaml` | careful | Haiku (`/llm_server`) |

## 3. Supervisor (`line_supervisor`)

### 3.1 `release`: which product next

- **Trigger:** the feeder slot is free, WIP < the WIP cap, and orders remain.
- **Question:** `next_product` ∈ {hold, A, B, C}. `hold` is the status quo: release nothing now.
- **Encodes:** orders (left, in progress, due, behind or ahead), routes (one pass / needs a lap /
  blocked, per product, given arms out of service), line (WIP, full buffers, arms down), time.
- **Mask:** products with nothing left to build.
- **Require:** `hold` when no slot or no orders.
- **Act (final):** the routing rule plans the product's route, then `Release`.

### 3.2 `disruption`: how to ride out a troubled arm

- **Trigger:** an arm reports `fault` (jam or sensor dropout), or a station's buffer stays full
  for `backlog_s`. The subject is the arm.
- **Question:** `response`:

  | class | effect |
  |---|---|
  | `wait_recover` (status quo) | keep routes; work waits for the arm |
  | `reroute` | route new and in-flight work around the arm until it recovers. Upstream alternatives cost a lap |
  | `slow_line` | lower the WIP cap to `wip_slow` until it recovers, so fewer pallets pile up |
  | `pause_station` | order a tool change and take the arm out of service for `pause_s` (directives `tool_change`, `pause`), routing around it. For repeat offenders |

- **Encodes:** the event (what, how long ago, how many this run, estimated clear time), the arm
  (station, wear, parts waiting), alternatives (which ops can move and at what cost; `fasten` has
  none), line and orders.
- **Act (every phase):** the response is a per-arm setting. `wait_recover` undoes a reroute.

### 3.3 `quality_watch`: is an arm drifting?

- **Trigger:** a `qa_result` report blames an op on arm X (its reading is marginal or worse). The
  subject is X.
- **Question:** `escalate` ∈ {monitor, inspect_tool, order_tool_change}.
- **Encodes:** X's QA record (flagged/total over its last parts, by op), wear estimate, ops and
  time since its last tool change, and its queue (a tool change costs `tool_change_s` of
  capacity).
- **Act (final):**
  - `inspect_tool` sends the directive of that name. The arm's next op then goes through
    `op_check` with the note, whatever its readings.
  - `order_tool_change` sends `tool_change`, which the arm must obey at its next idle moment.

### 3.4 Rules

| rule | does |
|---|---|
| Routing | For each op, among capable arms not out of service, prefer stations at or after the pallet's position (no lap), then the least queued work, then the shortest travel. If none is ahead, it laps. `fasten` needs hold + screw arms at one station. An op with no capable arm stays unroutable until an arm returns |
| Hand-off | An arm's `hand_off` report re-routes that op without the arm |
| Unrouted parts | Rework redo ops get routed as soon as `LineState` shows them |

## 4. Arm (`arm_agent`, one per arm)

### 4.1 Routine path (no model)

When an op is pending for the arm, the arm runs it at once if all of these hold:
- grip ok or n/a
- pose ≤ `pose_tol_mm`
- wear estimate < `wear_high`
- no dropout
- no open `inspect_tool` request

### 4.2 `op_check`: an anomalous pre-op reading

- **Trigger:** a pending op fails the routine check. The subject is `part<N>:op<k>`.
- **Question:** `action`:

  | class | effect |
  |---|---|
  | `proceed` (status quo) | run the op as is |
  | `retry` | regrip and re-measure |
  | `tool_change` | change the tool, then re-check |
  | `hand_off` | report it; the supervisor routes the op elsewhere |

- **Encodes:** the operation (kind, op k of n, part, product, regrips so far), grip, alignment,
  tool (wear estimate, ops since change), sensors (nominal, or dropout and for how long), and
  context (supervisor request, queue behind, the alternative arm or none).
- **Mask:**
  - `retry` after `max_regrips` or for a non-gripping op
  - `hand_off` when no other capable arm exists (`fasten`, or the alternative is down)
  - `tool_change` within `min_ops_between_changes`
- **Require:**
  - `proceed` when no op is pending
  - `hand_off` when the grip still reads `slip` after `max_regrips` and an alternative exists
- **Act (final):** `Operate` of that kind, or a `hand_off` report.

### 4.3 `coop`: a late fasten partner (arm_c, arm_d)

- **Trigger:** waiting for the partner exceeds `coop_late_s`.
- **Question:** `coop_action` ∈ {wait, abort_step}.
- **Encodes:** the step, the partner's state (busy, jammed, regripping, grip), how long it has
  waited versus nominal, and the queue behind.
- **Require:** `abort_step` after `coop_max_s`, or when the partner is jammed.
- **Act (final):**
  - `abort_step` cancels the op goal and sends `defer`; the pallet laps and the step is retried.
  - `wait` keeps waiting. The set re-triggers every `coop_late_s` until the partner arrives or
    the `coop_max_s` require fires.

### 4.4 Reports and directives

- **Reports up:**
  - `fault` (jam, sensor dropout)
  - `recovered`
  - `hand_off` (with part and op)
  - `tool_change`
- **Directives down:**
  - `pause` (seconds) and `resume`
  - `inspect_tool`
  - `tool_change`

The arm's `Hierarchy` has no engine attached, because a pause is not a decision. The arm triggers
`op_check` itself when it needs one.

## 5. QA (`qa_agent`)

### 5.1 Routine path

A part whose per-op readings are all `ok` (≥ `ok_min`) is passed by rule, with no model asked.

### 5.2 `inspect`: disposition of a flagged part

- **Trigger:** a part arrives with at least one reading below `ok_min`. The subject is `part<N>`.
- **Question:** `disposition`:

  | class | meaning | reversible |
  |---|---|---|
  | `pass` (status quo) | ship it | yes |
  | `rework` | redo the worst-reading op | yes |
  | `quarantine` | pull it for a human | yes |
  | `scrap` | destroy it | **no: two keys (Jev and LLM must both pick it)** |

- **Encodes:** the part (product, reworks so far of max), per-op readings in words and numbers
  (`ok` / `marginal` / `suspect` / `fail` / `missing`, plus the value), the arms' recent QA
  records, and line pressure (orders behind, rework lane load).
- **Mask:** `rework` once the part has `max_reworks` reworks.
- **Require:** `quarantine` when `rework` is masked and a reading is `missing` (< 0.4). Never
  mask without a way out.
- **Act (final):** `Dispose`, then a `qa_result` report to the supervisor (every part, rule-passed
  ones included, so its per-arm records have a denominator).

## 6. Encoding

Words carry the decision buckets and numbers carry detail:

```
"grip marginal (0.62); offset 4.8 mm, large (tolerance 2 mm); wear high (72% est.)"
```

With `ask_jev: change`, a reading that drifts inside its bucket does not re-ask Jev.

## 7. Mock

`assembly_line.mock_rules:line` answers every question plausibly from the encoded words, putting
real probability on every class. The test suite checks that, over a set of representative states,
**every class of every question is the mock's top choice at least once**. The swarm's worst live
bugs came from classes the mock never chose.

## 8. Tuning

- `ros2 jev search --runner assembly_line.episode:LineEpisode` compares policies:
  - P0: Jev only
  - P1: as shipped
  - P2: exploratory, τ = 1
  - P3: no inertia, κ = 0
- `ros2 jev calibrate` fits the LLM weight for `disposition` from the scorer's labels.
