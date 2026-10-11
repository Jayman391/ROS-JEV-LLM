# Plan: streamline jev_ai, cut Jev load, tune the swarm to 8/8

Status: **approved with decisions** (2026-10-11, user):
- Energy: **battery 60 → 90 Wh and survey speed 4 → 6 m/s**, plus sectors sized to the battery.
- Jev load: **B1 only** (event-driven asks; sets stay whole). B2 `ask_when` and B3 are not built;
  the masked-noise case is handled in the swarm (C3).
- Control: **rules own routine, AI judges** (C1–C6 as below; `fit` replaced by Hungarian).
- Knob API: **dials + presets, back-compatible** (A2).

Order: library first (A), Jev load (B1), swarm (C).

## Evidence (live run `swarm-s0-cfdfe3`, typesafe + Haiku/Sonnet, seed 0)

| | value | target |
|---|---|---|
| survivors found (recall) | **3/8 = 37.5 %** | 8/8 |
| coverage | 37 % | ~100 % |
| false confirmations | 1 (precision 0.86) | 0 |
| separation breaches | 7 | 0 |
| LLM spend | $0.72 | lower |
| Jev | 3,219 records, ~61 items / sim s, p50 160 ms per request | ≥ 75 % fewer items |
| real-time factor | ~1.7× (target 2×) | 2× |

Why recall was 3/8 (coverage was the limit):
1. **3:19 of 15:00 lost before SEARCH.** LAUNCH 1:13, TRANSIT 2:06 for a 72 m hop. `phase_transition`
   (Jev + Sonnet) answered `stay` 52× vs `advance` 8×; drones hovered (hover ≈ flight power).
2. **Battery sized to the mission with no slack.** E_0 60 Wh ≈ 915 s; a sector is ~600 s of survey.
   After the lost 200 s, drones hit "47 % left, 52 % needed" with sectors 8–21 % done → 100 RTBs.
3. **Masked argmax picks noise.** Parked at a formation slot, `continue` is masked (no order) and
   Jev's top class `inspect` (~0.8) is masked too; the argmax of what's left was `request_support`
   (p ≈ 0.1), and the coordinator answered **`recall_drone` 12×** (uav1 lost this way).
4. **Replan churn.** Sonnet replanned ~every minute; "inspection requested, no drone free yet" ×20+.
5. SEARCH ended at 70 % of the clock regardless of coverage.

Why Jev is slow (`engine.py:207-256`, `coordinator.py:76-86`): one request per **set** on **every
tick** whether or not a decision can be due; drones ask all 7 questions at 1 Hz (W4 `energy_risk`
is never read; W2/W3 matter only with a detection); `fit` asks 8 drones × 8 sectors = 64 questions.

## A. Library: clear docs, simple setup, composable S1/S2 dials

**A1 Docs** (newcomer path first, spec last):
- `README.md` rewritten: what jev_ai is (System 1 = Jev, System 2 = LLM, fused), 5-minute
  quickstart (compose → colcon → toy_patrol mock → dashboard), keys via a new `.env.example`.
- `docs/GUIDE.md` (new): the decision cycle in one diagram; a 30-line node; the catalog YAML;
  the 4 callbacks (encode / mask / require / on_decision); act-then-revise; hierarchy.
- `docs/TUNING.md` (new): **goal → dial** table ("ask the LLM less", "act faster", "be more
  decisive", "protect irreversible actions", "coordinate a fleet", "cut Jev cost") + one complete
  knob reference (catalog, policy, servers, engine args) with defaults.
- `FRAMEWORK.md` stays the spec; fix the ~14 stale items found (margin rule, max_tokens_per_min,
  ctor signature, MutuallyExclusive group, `<backend>.plugin`, CLI list, …). DESIGN/REVIEW: mark
  superseded entries. Remove the leftover uv scaffold (`main.py`, `pyproject.toml`, `uv.lock`,
  `.python-version`) and make `bringup.launch.py` default to mock (keyless works first time).

**A2 Composable dials** (old names keep working; each dial = one concern):
- **LLM use** — `llm: never | when_unsure | always` per set or question (`when_unsure` uses
  `skip_llm_above_c`, default 0.5). Replaces "augment:false / k_max≈0 / skip 0" ambiguity; `c_low`
  folds into it.
- **Stickiness** — `kappa`, `T_dwell`, `argmax_margin` documented as one group.
- **Exploration** — `tau`, `act`, coupling `rho`.
- **Safety** — masks, `require()`, `reversible`, two-key rule.
- **Budget/latency** — one timeout per backend: the engine sends its deadline to the server
  (removes the hand-synced pairs); `k_max`, `max_rps`.
- **Presets** — `policy: {preset: fast | balanced | careful}` sets a coherent group; any knob
  overrides it.
- Fixes: `policy=` silently ignored with a Catalog object; masked-argmax noise (below, B3).

## B. Jev load (framework features, generic)

- **B1 Ask only when a decision can be due.** The tick no longer calls Jev by itself; triggers do
  (events, directives, `T_refresh`, and a cheap "state changed" check on the encoded state).
- **B2 Per-question `ask_when`**: `always` (default) | `subject` (only while the app has a subject,
  e.g. a detection) | `trigger:<names>`. The engine sends only the due questions (server already
  accepts subsets).
- **B3 Determined questions skip Jev**: if `require()` fixes it or one class is unmasked → no call.
  If Jev's top class is masked and the rest is under `c_low` → `fallback`, not the noise argmax.
- Expected for the swarm: ~61 → ~10–15 items / sim s (W4 removed, W2/W3 only with a detection,
  next_action only when there's a choice, `fit` replaced, see C4).

## C. Drone swarm: target 8/8 survivors, 0 breaches

- **C1 Phases by rule**: LAUNCH ends when all are airborne; skip the formation when the hop is
  < 150 m (go straight to sectors); SEARCH ends on full coverage or when the time left equals
  RTB time + reserve. Jev + LLM keep the judgement calls (`recover_all`, `abort`).
- **C2 Energy that fits**: sectors sized from the battery model (survey + transit ≤ (1−reserve)·E);
  plus a battery / speed change (decision Q1).
- **C3 No recall without cause**: `recall_drone` masked unless energy is short or a fault exists;
  a drone with no order is held (`require` → `hold`), it does not ask Jev.
- **C4 Allocation deterministic**: Hungarian over a distance/energy cost (C++ `hungarian.hpp`
  exists) replaces the 64-question `fit`; Jev/LLM review it only when the plan changes.
- **C5 Replan only on change** (drone lost, sector done early), ≥ 3 min apart; sector progress kept.
- **C6 Inspections**: the detecting drone inspects inline; queued tracks go to the next drone to
  finish a sector.
- **Acceptance**: mock seeds 0–9 ≥ 7.5/8 average and seed 0 = 8/8; one live seed-0 run 8/8 with
  0 breaches; RTF ≥ 1.95×. Checked with the scorer, results logged in DESIGN.md.
