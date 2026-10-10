# Decision model: Jev baseline + LLM augmentation

This document specifies:
- every decision the swarm makes
- the exact input features and Jev question classes behind each decision
- how the LLM augments Jev
- how the whole thing is optimised

The physical world, action spaces, objectives and constraints are in [SYSTEM_MODEL.md](SYSTEM_MODEL.md).

Status: **draft for review**. ⚑ marks an open decision.

---

## 1. Decision architecture

```
             features (deterministic)        System 1                  System 2
sensors ──▶ FeatureExtractor ──state──▶ Jev (JevDecide.srv) ─p_J,c─┐
   │              │                                                  ├─▶ Fusion ─π─▶ sample a ─▶ execute
   │              └──state + p_J ─────▶ LLM (Reason.action) ──q_L───┘     ▲  (masked)
   └──────────────────────────────────────────────────────────── safety masks (C++)
```

- **Jev decides every decision.** It is the baseline, and it is fast enough (~0.1 s) to run every decision tick.
- **The LLM always augments; it is never a gate.** When it has an opinion (`q_L`), that opinion is pooled into Jev's distribution:
  - Its weight **β rises as Jev's confidence falls**, so when Jev is unsure the LLM can overrule it.
  - Its weight never drops to zero, so even confident decisions stay open to the LLM's view.
- **Sampling at temperature τ** gives controlled nondeterminism, even at high confidence:
  - it prevents swarm-wide correlated errors, where every drone makes the same mistake at once;
  - it makes independent drones explore different hypotheses.
- **Act-then-revise:** the Jev-based decision executes at once. The LLM's view arrives 1–3 s later and may revise the decision while it is still reversible.
- **Safety masks are absolute.** An action that breaks a hard constraint (H1–H8) has probability 0, whatever either model says.

| Agent | Jev model | LLM | Decision tick | LLM cadence |
|---|---|---|---|---|
| Drone agent (×N) | `jev-1.13.0` (pinned) | `claude-haiku-5-5`, effort `low` | 1 Hz | event-triggered, ≤ 6/min |
| Coordinator | `jev-1.13.0` (pinned) | `claude-sonnet-5-5`, effort `medium` (augment) / `high` (mission plan) | 0.5 Hz + events | event-triggered, ≤ 4/min |

## 2. Input feature space

Jev is weak at arithmetic, counting and long, noisy state. It is biased toward the first option. Feature extraction therefore follows four rules:

1. **All numbers are computed in code.** Jev and the LLM receive *interpreted* values: a bucket label plus a rounded number with units, e.g. `"tight (38% left, 31% needed)"`.
2. **The state is short:** ≤ 12 fields and ~300 tokens. Every field is a string sentence fragment, and the key order is fixed.
3. **No raw sensor arrays.** Only derived facts are passed.
4. **The same state goes to Jev and the LLM.** The LLM additionally gets Jev's answers (§6).

### 2.1 Drone agent state `s_w`

| Key | Sources | Derivation | Encoding (bucket thresholds) |
|---|---|---|---|
| `mission` | coordinator phase broadcast | phase + clock | `"SEARCH, 4:10 of 10:00 elapsed"` |
| `task` | `/uavK/task` feedback | current primitive + progress + ETA | `"survey sector B: 62% done, ~95 s left"` · `"inspect T7: en route, 40 m"` · `"none"` |
| `energy` | `/uavK/battery`, odom, wind | m_E = (E − E_rtb − R)/E_0; E_task = energy to finish the task | `ample` ≥ 0.30 · `adequate` 0.15–0.30 · `tight` 0.05–0.15 · `critical` < 0.05, plus `"(38% left, 31% needed to finish and return)"` |
| `home` | odom, wind | distance, ETA, wind component along the home vector | `"180 m, ~40 s, headwind 4 m/s"` |
| `link` | `/uavK/link` | quality q + time in the current bucket | `good` ≥ 0.7 · `degraded` 0.3–0.7 · `poor` 0.1–0.3 · `lost` < 0.1, plus `"for 12 s"` |
| `hazards` | `/uavK/proximity`, peers' `/status`, NFZ map | active reflex triggers + nearest distances | `"none"` · `"obstacle 6 m ahead, closing 2 m/s; uav3 5 m"` |
| `detection` | `/uavK/detections` local tracks | best unresolved track by expected survivor value | `"T7, 140 m E: 3 sightings from 30 m; scores survivor 0.46, animal 0.31, debris 0.12; not inspected; no other drone assigned"` · `"none"` |
| `other_detections` | local tracks | count of unresolved tracks besides the best | `"2 more unresolved"` |
| `health` | battery drain rate vs model, GPS σ, speed cap | fault detectors (residual > 3σ for 5 s) | `"nominal"` · `"battery draining 3x faster than expected"` · `"gps degraded"` · `"motor degraded"` |
| `coordinator` | orders via `comms_sim` | last order + age + pending request | `"last order 12 s ago: survey sector B; support request pending 8 s"` |
| `wind` | env broadcast (the drone *knows* the forecast wind) | speed + direction | `calm` < 2 · `moderate` 2–6 · `strong` > 6 m/s, plus `"from W"` |

Example `s_w`:

```json
{
  "mission": "SEARCH, 4:10 of 10:00 elapsed",
  "task": "survey sector B: 62% done, ~95 s left",
  "energy": "tight (38% left, 31% needed to finish and return)",
  "home": "180 m, ~40 s, headwind 4 m/s",
  "link": "degraded (quality 0.45) for 12 s",
  "hazards": "none",
  "detection": "T7, 140 m E: 3 sightings from 30 m; scores survivor 0.46, animal 0.31, debris 0.12; not inspected; no other drone assigned",
  "other_detections": "none",
  "health": "nominal",
  "coordinator": "last order 12 s ago: survey sector B; no pending request",
  "wind": "moderate 5 m/s from W"
}
```

### 2.2 Coordinator states

The coordinator's decisions concern different entities, so it sends **three request types**, each with its own focused state.

**Global state `s_c`** (for C1, C2, C5, C6, C7):

| Key | Encoding example |
|---|---|
| `mission` | `"SEARCH, 4:10 of 10:00; objective: find survivors in AO, recover all drones"` |
| `coverage` | `"64% total; A 90%, B 62%, C 41% (uav3 slow), D 70%"` |
| `swarm` | one line per drone: `"uav1: surveying A, energy adequate, link good"` |
| `tracks` | `"3 confirmed survivors, 1 confirmed other, 4 unresolved (2 likely survivor)"` |
| `time_budget` | `"recover must start by 8:30 (energy-limited by uav3)"`, computed from the worst-case E_rtb |
| `risk_events` | `"uav3 battery fault at 3:40; comms blackout NE corner"` |
| `pairs` (C7 only) | one line per (drone, sector): `"uav2→C: 120 m away, energy after sector: tight, sector 41% covered"` |

**Track state `s_t`** (one request per unresolved track, for C3):
`"T7 at (140, −22): 5 sightings from uav2, uav3; best view from 10 m (inspected): survivor 0.88; survey-altitude views: survivor 0.5 / animal 0.3; position spread 2 m; nearest confirmed survivor 60 m; inspection would cost uav2 ~50 s, energy adequate"`

**Report state `s_r`** (one request per incoming worker report, for C4): the report (situation, recommendation, and LLM reasoning if any) plus the reporting drone's line from `swarm` and its sector's coverage.

## 3. Jev question catalog

Instructions and criteria are the **exact strings** sent to Jev. They are versioned in `swarm_ai/questions.py` and tuned offline (§7.3).

**Option order:** the status-quo or default option is listed **first**, on purpose. Jev's first-option bias then pulls toward inaction rather than toward an arbitrary action. ⚑ This is tested in §7.3d.

### 3.1 Drone agent: one request per tick, all questions against `s_w`

| ID | Type | Name | Instructions | Classes, in order (criteria) | Asked when |
|---|---|---|---|---|---|
| W1 | choice | `next_action` | "Choose this search-and-rescue drone's next action." | `continue` (keep doing the assigned task; nothing requires a change) · `inspect` (fly lower to look at the unresolved detection; worthwhile if it may be a survivor and energy allows) · `hold` (hover and wait; the situation is unclear or orders are awaited) · `return_to_base` (go home now; energy, health or link make continuing unsafe or pointless) · `request_support` (keep working but ask the coordinator for help or a handoff) · `land_now` (emergency landing in place; only for a critical failure) | always |
| W2 | choice | `detection_class` | "What is the detected object most likely to be? Sightings from higher altitude are less reliable; consistent repeated sightings are more reliable." | `survivor` (a person needing rescue) · `animal` · `vehicle` · `debris` (rubble, wreckage or other inanimate clutter) · `nothing` (a false alarm; no real object) | `detection ≠ none` |
| W3 | score | `detection_priority` | "How important is it to resolve this detection soon?" | `ignore` · `low` · `moderate` · `high` · `urgent` | `detection ≠ none` |
| W4 | score | `energy_risk` | "How risky is the drone's energy situation for finishing its task and getting home, considering wind and distance?" | `comfortable` · `adequate` · `tight` · `critical` | always (advisory: logged, and shown to the LLM; never acted on directly, because H6 is deterministic) |
| W5 | noul | `report_now` | "Should the drone send a report to the coordinator now?" | true: something changed that the coordinator should know (a likely survivor, a fault, help needed, a task finished) · false: nothing new worth reporting | always |
| W6 | noul | `unusual` | "Is this situation unusual, or poorly covered by routine behaviour?" | true: novel, conflicting or surprising signals · false: routine | always (LLM trigger) |
| W7 | choice | `situation` | "Which best summarises the drone's situation?" | `nominal` · `candidate_found` · `energy_concern` · `hazard` · `link_concern` · `fault` · `task_complete` | always (becomes `DroneReport.situation`) |

### 3.2 Coordinator

| ID | Type | Name | State | Instructions | Classes, in order |
|---|---|---|---|---|---|
| C1 | choice | `phase_transition` | `s_c` | "The mission is in the phase shown. Should it move on?" | `stay` · `advance` (go to the next phase) · `recover_all` (skip to recovery) · `abort` (recall immediately; safety or mission failure) |
| C2 | choice | `transit_formation` | `s_c` | "Which formation should the swarm fly for this transit leg?" | `column` (narrow; threads between obstacles) · `line` (wide abreast; sweeps on the way) · `v` (good visibility and separation) · `diamond` (compact; easy to hold) · `circle` (symmetric; good for a hold pattern) |
| C3 | choice | `track_verdict` | `s_t` | "What should be done with this detection track?" | `await_data` (wait for more sightings) · `confirm_survivor` · `confirm_non_survivor` (a real object, but not a person) · `request_inspection` (send a drone low for a closer look) · `dismiss` (a false alarm) |
| C4 | choice | `report_response` | `s_r` | "How should the coordinator respond to this drone's report?" | `acknowledge` (no change) · `dispatch_support` (send another drone to help or verify) · `reassign_sector` (move the drone's remaining work to others) · `recall_drone` (send it home) · `replan` (the mission plan no longer fits; replan) |
| C5 | score | `mission_risk` | `s_c` | "How much risk is the mission currently carrying?" | `low` · `moderate` · `elevated` · `severe` |
| C6 | noul | `replan_needed` | `s_c` | "Does the current plan need replanning?" | true: assignments or phase timing no longer fit · false: plan still fits |
| C7 | score ×N² | `fit_{d}_{s}` | `s_c` (+ `pairs`) | "How well suited is drone {d} to search sector {s} now?" | `poor` · `fair` · `good` · `excellent` |

C7 asks N² questions in one request (16 at N = 4). Jev evaluates questions in parallel, so the request costs a single round trip.

## 4. Output decision space

Every Jev answer is treated as a **categorical distribution** over its classes:
- **Choice:** over its options.
- **Score:** over its L levels.
- **Noul:** over {true, false}, with p = `noul`.

**Confidence c:**
- **Choice and Score:** Jev's `confidence` field.
- **Noul:** c = |2p − 1|, because Jev returns no confidence field for Noul answers.

| Question | Fused output | Consumed by | Execution |
|---|---|---|---|
| W1 | sampled action a ∈ A_w | agent | `/uavK/task` goal (SYSTEM_MODEL §5.2) |
| W2 | distribution over classes | report payload; coordinator track fusion | `DroneReport.class_probs` |
| W3 | expected level u = E[ℓ]/(L−1) ∈ [0, 1] | agent: picks *which* detection to inspect; coordinator: dispatch ordering | utility |
| W4 | u ∈ [0, 1] | LLM context, logs | none |
| W5 | p(true) ≥ θ_report (0.5) **or** W7 ≠ previous | agent | publish `DroneReport` |
| W6 | p(true) ≥ θ_unusual (0.5) | agent | LLM trigger event E3 |
| W7 | argmax label | report payload | `DroneReport.situation` |
| C1 | sampled transition | coordinator FSM | phase change → its swarm behaviour |
| C2 | sampled formation | coordinator | Formation action |
| C3 | sampled verdict | track registry | `request_inspection` → dispatch (§7.2) |
| C4 | sampled response | coordinator | orders to drones; `replan` → Sonnet mission replan |
| C5 | u ∈ [0, 1] | coordinator | `u ≥ 0.75` → LLM event; logged |
| C6 | p(true) | coordinator | p ≥ 0.6 → Sonnet replan |
| C7 | u_fit(d, s) ∈ [0, 1] | allocator | Hungarian cost term (§7.2) |

Only **choice** answers drive actions through sampling. Score and Noul answers drive actions through **expected values and thresholds**, which are deterministic given the fused distribution. They are still LLM-augmented by the same pooling rule.

## 5. Fusion: Jev baseline + LLM augmentation

### 5.1 Pooling rule (per question)

For a question with classes A, the inputs are:
- Jev's answer p_J ∈ Δ(A), with confidence c
- the LLM's preference q_L ∈ Δ(A), with age t_L
- the safety mask m ∈ {0, 1}^A

```
q̃_L(a)  = (q_L(a) + ε) / (1 + |A|·ε)                  floor: the LLM can strongly down-weight, not veto
β(c)    = β_min + (β_max − β_min) · (1 − c)^γ          LLM weight grows as Jev confidence falls
β_eff   = β(c) · exp(−t_L / T_stale)                   stale LLM opinions fade back to Jev
log π(a) = log p_J(a) + β_eff · log q̃_L(a) + log m(a) − log Z
a ~ softmax(log π / τ)                                  τ → 0: argmax; τ = 1: sample π
```

- With no `q_L` (no LLM opinion yet, or the call failed), β_eff = 0 and π is the masked p_J.
- If every class is masked, the agent falls back to the safety-layer default action: `return_to_base` for workers, `stay` for the coordinator.
- Because the floor ε stops the LLM from vetoing anything, **vetoes come only from the safety masks**, which are deterministic and auditable.

### 5.2 Worked example (β_min 0.1, β_max 2.0, γ 1, ε ≈ 0)

Jev returns `p_J = {continue .70, inspect .20, hold .05, return_to_base .05}`, and Haiku replies `q_L = {continue .10, inspect .80, hold .05, return_to_base .05}`.

| Jev confidence c | β | Fused π (continue / inspect / hold / rtb) | At τ = 0.5 |
|---|---|---|---|
| 0.95 | 0.195 | .64 / .28 / .04 / .04 | .84 / .15 / … |
| 0.60 | 0.86 | .36 / .61 / .01 / .01 | .26 / .74 / … |

- **High confidence:** Jev keeps the decision, but `inspect` rises from 20% to 28%. Across 4 drones and repeated events, that is real exploration. This is the "high-confidence nondeterminism".
- **Lower confidence:** the LLM flips the decision.
- **τ sharpens or flattens π:**
  - τ < 1 sharpens π toward its argmax. At τ = 0.5 the high-confidence `inspect` share drops back to 15%.
  - τ = 1 samples π exactly. τ > 1 flattens π.
  - So **β sets how much the LLM moves the distribution, and τ sets how much of that spread becomes actual behaviour.** Both are tuned in §7.3.

### 5.3 Act-then-revise protocol

```
tick (1 Hz):  Jev → p_J ─▶ masks ─▶ decision event?  no → keep current action
                                         │ yes
                                         ▼
                          sample a₀ ~ π₀ (uses the last q_L, decayed)
                  a₀ reversible ──────────┴────────── a₀ irreversible-class
                  execute a₀ now                       hold, fire LLM (priority),
                  fire LLM if a trigger fired          wait ≤ T_wait (4 s)
                          │                                   │
             LLM returns q_L → π₁                LLM returns → sample a₁ ~ π₁ → commit
             keep a_cur if π₁(a_cur) ≥ θ_keep·max π₁   timeout → commit a₀ (logged)
             and dwell < T_dwell; else resample and switch
```

- **Decision events**, the only times sampling happens:
  - Jev's argmax differs from the current action
  - the current action becomes masked
  - the current action completes
  - a new detection track appears
  - a coordinator order arrives
  - an LLM result arrives

  Between events the current action is kept. This stops τ > 0 from making the drone dither every tick.
- **Dwell:** no voluntary switch within T_dwell = 5 s of the last switch. Mask-forced switches are immediate.
- **Irreversible-class actions:**
  - `land_now`
  - `inspect` once it is past its commit point
  - coordinator `abort`
  - `confirm_*` verdicts, which are reported as final
- **Reproducibility:** every agent's RNG is seeded with `hash(episode_seed, agent_id)`.

## 6. LLM augmentation contract

### 6.1 Triggers (worker; coordinator analogues in brackets)

| ID | Event | Priority |
|---|---|---|
| E1 | W1's argmax differs from the current action [C1/C3/C4 argmax change] | normal |
| E2 | new detection track, or W2's argmax changes [new or updated track] | normal |
| E3 | W6 `unusual` ≥ 0.5 [C5 ≥ 0.75 or C6 ≥ 0.6] | high |
| E4 | an irreversible-class action is pending | **reserved** (always allowed) |
| E5 | a coordinator order is received [a worker report is received] | normal |
| E6 | periodic refresh every T_refresh = 20 s [30 s] | low |
| E7 | W1 confidence < c_low = 0.5 | high |

**Rate cap:** a token bucket of k_max calls per minute (worker 6, coordinator 4), with 1 slot reserved for E4.
- Coalescing: events that arrive while a call is in flight are merged into one follow-up call.
- Cost bound: at most 4 × 6 = 24 Haiku calls/min and 4 Sonnet calls/min, plus mission planning.

### 6.2 Request

One `Reason.action` goal per event covers **all questions of the triggering Jev request**:
- **`task`:** a fixed system role. Worker: "You are the onboard reasoning module of SAR drone uavK…". Coordinator: "You coordinate a SAR drone swarm…". It also lists the action vocabulary (SYSTEM_MODEL §5), the hard constraints (so the LLM doesn't waste weight on masked actions), and the mission objective.
- **`context`:** JSON containing `state` (s_w / s_c / s_t / s_r), Jev's answers (distributions + confidence), the current mask, the current action, and the last 3 decisions with outcomes (short-term memory).
- **`response_schema`:**

```json
{
  "type": "object", "additionalProperties": false,
  "required": ["weights", "rationale"],
  "properties": {
    "weights":  {"type": "object", "description": "question name -> {class: weight 0..1}",
                 "additionalProperties": {"type": "object", "additionalProperties": {"type": "number"}}},
    "params":   {"type": "object", "description": "optional bounded parameter proposals, e.g. inspect_altitude_m"},
    "rationale": {"type": "string", "description": "<= 2 sentences"},
    "flags":    {"type": "array", "items": {"enum": ["novel_situation", "option_missing", "data_suspect"]}}
  }
}
```

- `weights` is normalised per question into q_L. Questions the LLM omits get β_eff = 0.
- `params` are clamped to the bounds in SYSTEM_MODEL §5.2.
- `option_missing` means the LLM thinks no option fits. It is forwarded to the coordinator as a report, where it is a signal for C4 `replan`.

### 6.3 Coordinator mission planning (Sonnet, beyond augmentation)

On `Mission.action`, Sonnet (effort `high`) turns the natural-language objective into a structured **plan**:
- sector partition (strips or a grid)
- survey altitude within the band
- transit formation preference
- phase time budgets
- priorities, e.g. "inspect likely survivors before finishing coverage"

The plan is validated against the hard constraints and the parameter ranges; on failure it is re-asked once, then a default plan is used. It is then executed by the FSM, which makes each per-step decision with Jev + augmentation. `replan` (C4) or C6 re-invokes planning with the current `s_c`.

### 6.4 Failure behaviour

| Failure | Behaviour |
|---|---|
| LLM timeout (worker 6 s, coordinator 15 s), error, or schema-invalid output | β_eff = 0 (pure Jev); logged |
| Jev timeout (1 s) or error | the agent keeps its current action and retries next tick. After 3 consecutive failures: `hold` (worker) / `stay` (coordinator), and raise a `fault` report |
| Both down for more than 10 s | the worker holds, then follows the lost-link procedure H7 |

## 7. Optimization scheme

The scheme has three nested levels:
- **7.1 Per decision:** what to do now.
- **7.2 Per coordinator cycle:** who does what.
- **7.3 Across episodes:** which parameters and wordings make the swarm score best.

### 7.1 Online, per decision

The pooled policy π (§5.1) is a **tempered product of experts**. If q_L is read as a likelihood, then β is a trust exponent and τ trades exploitation against exploration. The masks project π onto the feasible set. There is no online learning in v1; ⚑ a per-episode bandit over τ is a possible v2.

### 7.2 Online combinatorial: coordinator allocation (C++ `formation_planner` / allocator)

**Sector allocation** (on entering SEARCH, after a drone is lost, and on `reassign_sector`) is a linear assignment problem, solved with the Hungarian algorithm in O(N³):

```
C(d, s) = w_dist · dist(d, entry(s)) / D_max
        + w_fit  · (1 − u_fit(d, s))                       ← C7, fused
        + w_sw   · [s ≠ current_sector(d)]                  ← switching cost
        + BIG    · [E_need(d, s) + E_rtb > E(d) − R]        ← energy infeasible
```

**Formation slots:** Hungarian on the squared distances from the drones to their slot positions. No crossing paths, and minimum total travel.

**Inspection dispatch** (C3 `request_inspection`, C4 `dispatch_support`):

`d* = argmin over feasible d of [ t_travel(d, track) + w_opp · remaining_coverage_value(d) ]`

ties are broken by energy margin. Drones whose W3 priority for their own local track is above 0.75 are excluded.

The weights `w_dist`, `w_fit`, `w_sw` and `w_opp` are decision parameters (§8).

### 7.3 Offline: tuning across episodes

**a. Decision log.**
- Every decision publishes a `DecisionRecord` containing: episode, seed, sim time, agent, question set, state, p_J, c, q_L, t_L, β_eff, τ, mask, π, sampled a, executed a, events, latencies, tokens, and $.
- `decision_logger` writes these to JSONL on a mounted volume.
- `mission_scorer` writes the ground truth (true target classes and positions) and J per episode.

**b. Supervised calibration.** This needs no new simulation runs. Questions with ground-truth labels:
- W2: the true class
- C3: the correct verdict, given the true class and whether the evidence sufficed
- W5/W7: weak labels from coordinator outcomes

For these questions:
- Measure the **reliability (ECE, Brier)** of Jev alone, the LLM alone, and the fused distribution.
- **Fit (β_min, β_max, γ)** to minimise the negative log-likelihood of the true label under π on logged decisions. It is a 3-parameter fit, done with SciPy.
- If Jev's own confidence turns out to be miscalibrated on our states, fit a temperature-scaling correction p_J ← softmax(log p_J / T_J) per question.

**c. Episode-level policy search** for the action questions (W1, C1, C2, C4), which have no direct labels; the objective is E_seed[J].
- **Baselines**, run on the same episode seeds (paired comparison, common random numbers):

  | Policy | Definition |
  |---|---|
  | P0 | rule-based: hand thresholds, no models |
  | P1 | Jev-only, argmax (β ≡ 0, τ → 0) |
  | P2 | Jev-only, sampled (β ≡ 0, τ = default) |
  | P3 | LLM-only (p_J uniform, β = 1) |
  | P4 | fused (defaults) |

- **Search:** random search with successive halving over θ = {β_min, β_max, γ, τ, θ_keep, T_dwell, T_refresh, k_max, c_low, the bucket thresholds of §2, the allocator weights}.
- **Budget:** at RTF ≈ 1 a 10-minute episode takes about 10 min, so 10 seeds per config is about 1.7 h of compute.
- **Report:** J ± 95% CI, its component terms, and $ per episode.

**d. Question wording and option order.** This is cheap because Jev input costs $0.042 per million tokens and output is free.
- Replay logged states through Jev with variant instructions, criteria and permuted option order.
- Measure the distribution shift (first-option bias) and the accuracy on labelled questions.
- Keep a wording only if it improves labelled accuracy without reducing J.

## 8. Decision parameters

| Parameter | Default | Range | Tuned in |
|---|---|---|---|
| `jev_model` | `jev-1.13.0` | pinned | — |
| `worker_llm` / effort | `claude-haiku-5-5` / `low` | | — |
| `coord_llm` / effort | `claude-sonnet-5-5` / `medium` (augment), `high` (plan) | | ⚑ |
| `beta_min`, `beta_max`, `gamma` | 0.1, 2.0, 1.0 | [0, 0.5], [1, 4], [0.5, 3] | 7.3b |
| `epsilon` (LLM floor) | 0.02 | [0.005, 0.1] | 7.3b |
| `tau` | 0.5 (reversible), 0.2 (irreversible-class) | [0.05, 1] | 7.3c |
| `theta_keep` | 0.5 | [0.2, 0.9] | 7.3c |
| `T_dwell`, `T_stale`, `T_wait`, `T_refresh` | 5, 30, 4, 20 s | | 7.3c |
| `k_max` worker / coordinator | 6 / 4 per min | [1, 12] | 7.3c |
| `c_low` | 0.5 | [0.3, 0.7] | 7.3c |
| `theta_report`, `theta_unusual` | 0.5, 0.5 | | 7.3c |
| energy / link / wind bucket thresholds | §2.1 | ±50% | 7.3c, d |
| `w_dist`, `w_fit`, `w_sw`, `w_opp` | 1.0, 1.0, 0.3, 0.5 | | 7.3c |
| timeouts: Jev / worker LLM / coordinator LLM | 1, 6, 15 s | | — |

## 9. Open questions ⚑

1. **Coordinator effort:** Sonnet at `medium` for augmentation is ~2× faster than `high`. Is the drop in quality worth the speed?
2. **First-option bias:** list the status quo first (current plan), or randomise the order per call? Randomising removes the bias but adds variance. Either way, §7.3d measures it.
3. **LLM parameter proposals** (`params`): keep them, or let the LLM touch only the distributions in v1?
4. **Correlated vs independent sampling** across drones: independent (current plan) maximises diversity. A shared seed per event would make swarm behaviour more coherent.
5. **Relay placement** (SYSTEM_MODEL §5.3): include it in v1?
