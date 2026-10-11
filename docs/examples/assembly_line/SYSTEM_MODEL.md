# Assembly line: system model

A multi-arm assembly line on jev_ai. Five arms at four stations build three products as pallets
move station to station; a QA station inspects every finished part. The point of the example is
the **decision shape**: discrete, event-driven, tightly coupled choices (which product next, how to
ride out a faulted arm, whether a marginal grip is good enough, rework or scrap), where most work
is routine and done by rules, and Jev/Claude are asked only for judgement calls.
[DECISION_MODEL.md](DECISION_MODEL.md) covers the questions; this file covers the world.

There is no physics engine. `line_sim` (C++) is a discrete-event simulator: an operation is a
duration plus a quality draw, and every sensor is ground truth plus noise. It owns `/clock`, so
the line runs at `real_time_factor` × wall time.

## 1. Layout

```
feeder ─▶ S1[arm_a] ─▶ S2[arm_b] ─▶ S3[arm_c + arm_d] ─▶ S4[arm_e] ─▶ QA ─▶ out
   ▲                                                         │
   └─────────────────── rework lane ◀────────────────────────┘
```

- Each station has one work slot and an input buffer of `buffer_cap` (2) pallets. A pallet leaves
  a slot only when the next buffer has room, so a slow or jammed station backs the line up.
- A pallet stops only where its next op is routed and skips the other stations
  (`travel_s` per hop).
- After S4, a pallet that still has ops left goes round the rework lane (`rework_lane_s`) and
  re-enters at the feeder ahead of new releases. This **lap** is the only way back upstream, so it
  is how a reroute to an upstream arm or a deferred step is paid for.
- QA (`qa_s`) takes only finished parts. Its outcomes are `pass` (shipped), `rework` (one op
  redone; the part laps), `quarantine` (pulled for a human) and `scrap`.

## 2. Arms, tools, products

| arm | station | tools |
|---|---|---|
| arm_a | S1 | pick, screw, insert |
| arm_b | S2 | weld, screw |
| arm_c | S3 | pick, insert, hold |
| arm_d | S3 | hold, screw, glue |
| arm_e | S4 | glue, weld, pick |

Every solo op has at least two capable arms. `fasten` is **cooperative**: a `hold` arm and a
different `screw` arm at the same station, so only arm_c (holds) + arm_d (screws) can do it. It
has no backup: when either faults, rerouting cannot help.

| op | nominal s | QA sensor (σ) |
|---|---|---|
| pick | 6 | vision (0.08) |
| screw | 8 | torque audit (0.05) |
| weld | 12 | weld current (0.06) |
| glue | 10 | vision (0.08) |
| insert | 7 | vision (0.08) |
| fasten | 10 | torque audit (0.05) |

| product | recipe | route with every arm up |
|---|---|---|
| A | pick, screw, weld, glue | one pass |
| B | pick, insert, fasten, screw, weld | one pass |
| C | pick, weld, glue, insert, fasten, screw | one pass |

The order book (`orders`) is 24 units (10 A, 8 B, 6 C), each with a due time spread over the
shift (`t_max_s`). The episode ends when every order has shipped or at `t_max_s`.

## 3. Hidden truth: quality

Each op yields a true quality

```
q = 1 − 0.6·wear² − 0.35·(1 − grip) − 0.03·max(0, pose_mm − 1) − 0.15·[coop partner grip marginal] + N(0, 0.04)
```

The grip term applies only to gripping ops (pick, insert, fasten). A part is **defective** if any op has
`q < q_min` (0.75). A rework redoes one op with a fresh draw. Truth goes only to the scorer
(`/line/outcomes`, one `PartOutcome` per part that leaves the line).

## 4. Mocked sensors (`ArmStatus`, `QaReading`)

| reading | model | dropout |
|---|---|---|
| grip | true grip g: 0.9 ± 0.05 usually, marginal (≈0.6) 10%, slip (≈0.3) 5%; reading g + N(0, 0.05) | NaN |
| pose offset | \|N(0, 1)\| mm, 6% a 4–7 mm misalignment | NaN |
| tool wear estimate | wear + N(0, 0.08) | NaN |
| joint current | nominal, or spiking while jammed | – |
| QA per-op reading | q + N(0, σ of the op's sensor); < 0.4 reads as a missing feature | – |

Grip and pose are drawn when an op becomes pending for an arm; a regrip redraws them.

## 5. Faults (seeded per episode)

- **Tool wear:** wear grows by the arm's rate per op (one arm wears 2–3× faster). A tool change
  (`tool_change_s`) resets it.
- **Jam:** each op has p = `jam_p0 + jam_p1·wear` of jamming partway. The op fails and the arm is
  down for U(`jam_reset_min`, `jam_reset_max`) s; `ArmStatus` gives a noisy clear estimate.
- **Grip slip:** see §4. During `fasten` a marginal partner grip costs quality.
- **Sensor dropout:** 1–2 arms lose their readings (NaN) for `dropout_s`, once each.

## 6. Nodes, interfaces, data flow

| node | lang | role |
|---|---|---|
| `line_sim` | C++ | the world: conveyor, buffers, faults, quality, sensors, `/clock`, RViz markers; `Operate` action server per arm; `Release`, `Route`, `Dispose` services |
| `line_supervisor` | Py | jev_ai supervisor: releases, the routing rule, disruption response, quality watch |
| `arm_agent` ×5 | Py | jev_ai worker per arm (`/arm_x`): runs its ops; judges anomalous pre-op readings and late coop partners |
| `qa_agent` | Py | jev_ai worker: disposition of each inspected part |
| `line_scorer` | Py | the only node that reads the truth: J, Labels, episode summary |

`line_sim` is one node on purpose: a discrete-event world wants one owner of the state; separate
arm controllers would only add races. Split it if arms ever get real kinematics.

```
line_sim ──/arm_x/status (ArmStatus)──▶ arm_agent ──/arm_x/operate (Operate)──▶ line_sim
line_sim ──/qa/reading (QaReading)────▶ qa_agent ──/line/dispose (Dispose)──▶ line_sim
line_sim ──/line/state (LineState)────▶ line_supervisor ──/line/release, /line/route──▶ line_sim
arm_agent, qa_agent ──/line/reports──▶ line_supervisor ──/line/directives──▶ arm_agent
every engine ──/decisions──▶ decision_logger, dashboard;   status ──/diagnostics──▶ dashboard
line_sim ──/line/outcomes (PartOutcome)──▶ line_scorer ──/labels──▶ ros2 jev calibrate
line_sim ──/line/markers──▶ RViz
```

**`Operate` goal kinds:**

| kind | effect |
|---|---|
| `op` | do the pending op routed to this arm. For `fasten` both arms send one, and the first waits for its partner |
| `regrip` | 3 s; redraws grip and pose |
| `tool_change` | wear → 0 |
| `defer` | leave the pending op undone; the pallet moves on and comes back after a lap |

A cancelled `op` goal releases the coop handshake.

**Routes:**
- A route is one arm per op; `fasten` is written `arm_d+arm_c`, primary then partner.
- An empty entry is **unroutable**: the pallet waits where it is until the supervisor routes it.
- A rework appends a redo op with an empty entry, and the supervisor routes it.

## 7. Time

- `line_sim` advances sim time by `clock_step_s` (0.25 s) every `clock_step_s / real_time_factor`
  wall seconds; every other node uses sim time. Coarse steps keep `/clock` cheap for rclpy (the
  swarm lost speed to a 430 Hz `/clock`).
- The default factor of 4 serves mock and live runs alike. LLM latency is wall time, so the
  catalogs' `T_wait` (sim s) is `llm_timeout × real_time_factor + 0.5`. Change both together.

## 8. Scoring (`line_scorer`)

```
J = 10·good_shipped + 3·good_on_time − 40·escaped_defects − 10·scrapped_good − 3·scrapped_defective
    − 5·quarantined − w_usd·LLM $
```

- An escaped defect is a defective part that QA passed.
- Labels: `disposition` per part (subject `part<N>`). A good part's label is `pass`. A defective
  part's label is `rework` while reworks remain, else `scrap`.
- The summary JSON (`summary_path`) is what `assembly_line.episode` returns to `ros2 jev search`.

## 9. Running

```bash
ros2 launch assembly_line assembly_line.launch.py                      # keyless mocks, headless
ros2 launch assembly_line assembly_line.launch.py rviz:=true           # line view (noVNC :6080)
ros2 launch assembly_line assembly_line.launch.py jev_backend:=typesafe llm_backend:=anthropic
# dashboard: http://localhost:8080
```
