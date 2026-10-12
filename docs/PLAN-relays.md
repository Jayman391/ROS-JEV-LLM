# Plan: relay drones and free workers (drone swarm)

Status: **approved and implemented** (user, 2026-10-11). Decisions (user, paired, 2026-10-11):

| Topic | Decision |
|---|---|
| Source | Extra dedicated drones: **n/4 relays for n workers** (8 + 2) |
| Placement | **Dynamic**: each relay chases its group, by rule (C8 `relay_needed` removed) |
| Relay brain | Full drone agent (it may inspect what it sees) |
| Autonomy | Workers act on their own, **no formation**; soft leash on their own choices |
| Leash | **q ≥ 0.3** (~267 m hop) |
| Relay move | **1-centre** of its group, clamped to keep its uplink q ≥ 0.7 |
| Transit | Direct flight on **altitude layers**; formations removed |

Why: with R_c 250 m, 46% of the AO is past contact (q < 0.1) and only 33% has a good link. The
existing C8 relay never fired (8 drones, 8 sectors: "relay needed for uav3 but no drone is free").
Survey-through-link-loss (1cc2213) stays: pre-planned work may cross the leash.

## 1. Fleet

- `num_drones` keeps meaning **workers** (one sector each). Relays: `n // 4`, named after the
  workers (`uav9`, `uav10`). `scenario.workers(p)`, `scenario.relays(p)`; `drones(p)` = both.
  Pads, launch (FC + sensor_sim + agent per drone), the world and the scorer use `drones(p)`.
- Groups: sectors are strips along y; relay k serves k-th run of contiguous sectors
  (8 workers: A-D, E-H). n < 4: no relays, behaviour as today.

## 2. Relay rule (coordinator, every 5 s, SEARCH → RECOVER)

```
pts    = airborne workers of the group + unresolved tracks in its sectors   (x, y)
centre = 1-centre(pts)            # Badoiu-Clarkson: start at the centroid, 30 steps toward the
                                  # farthest point with step 1/(i+1); within ~3% of optimal
if |centre| > r_good: centre *= r_good / |centre|      # r_good = R_c - slope*ln(0.7/0.3) ≈ 233 m
order(relay, 'relay', point=[cx, cy, alt_relay]) if it moved > 20 m
```

- `alt_relay` 56 m: above every obstacle (max 45 m) and every transit layer.
- Uplink is the base only. ponytail: no relay-to-relay chains; add when n/4 relays can't reach.
- Relays are never allocated a sector or dispatched (`idle()` / `dispatch` skip them), launch
  with the workers, and land **last** in RECOVER (they carry the others' links home).
- Including unresolved tracks pulls the relay toward pending inspections, so a leashed one
  becomes inspectable as the relay arrives.

## 3. Leash (worker agent mask)

`q_at(point) = max(q(base, point), min(q(relay, point), relay.link_q))` over airborne relays
(peers on `/swarm/status`, so no new topic). The agent masks **`inspect`** when the best track's
`q_at` < `leash_q` (0.3). Coordinator orders (sector surveys, dispatches) are not leashed; H7
already lets them finish out of link. **Risk to watch:** a masked far inspection costs recall
unless the relay arrives; compare recall with and without the leash in the mock runs.

## 4. No formation: direct flight on layers

- LAUNCH: worker k takes off to `alt_transit + 4·(k mod 4)` (40/44/48/52 m; 4 m = d_sep apart),
  relays to `alt_relay`. LAUNCH → SEARCH directly (TRANSIT removed for every hop, as the short
  hop does now); the sector order flies the drone there.
- RECOVER: `land_all` for every hop: workers nearest first, then relays; the RTB goal carries
  the drone's layer (flight_controller: RTB flies at `goal.altitude` when > 0, else
  `alt_transit`).
- Removed: C2 `transit_formation`, the plan's `transit_formation` / `formation_spacing_m`,
  `form` / `at_slots` / `formation` / regroup stage, the `/swarm/formation` service
  (`Formation.srv`, its formation_planner handler, world.hpp slot code and tests). The mission
  plan keeps `priority_sectors` and the rationale.

## 5. Also in this phase

- RECOVER overran in the seed-0 run (11:52 → 16:50; `t_done` 961 s > 900, 1 unrecovered):
  find and fix it.
- Scorer: **contact %** = share of airborne worker-seconds with link q ≥ 0.3 (the relay metric).
- Dashboard: relays marked in the drone list (role column).
- Docs: SYSTEM_MODEL (fleet, layers), DECISION_MODEL (C2/C8 out, leash mask, relay rule),
  README, DESIGN log.

## 6. Acceptance (mock, seed 0, then seeds 0-4)

- contact % ≥ 90 (today ~50); coverage ≥ 90%; recall ≥ 7/8 on seed 0, mean ≥ 7.5/8 over seeds;
- 0 separation breaches, mission done ≤ t_max with all drones landed;
- RTF ≥ 1.9 with 10 X3s; decisions per sim-second within 25% of today's 2,101 / 961 s.

Tests: `workers`/`relays`/groups; 1-centre + clamp; leash mask; RTB layer altitude (gtest);
plan schema without formation fields.
