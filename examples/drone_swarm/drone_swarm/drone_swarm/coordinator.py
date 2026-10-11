"""mission_coordinator (B6): the swarm's supervisor, a jev_ai DecisionEngine + Hierarchy.

Mission phases LAUNCH -> TRANSIT -> SEARCH -> CONFIRM -> RECOVER -> DONE (| ABORT), SYSTEM_MODEL
§1.
Code owns the guards: ``advance`` is masked until the phase's work is done and *required* once its
time budget is spent. The models decide within them (DECISION_MODEL §3.2):

- ``global`` (0.5 Hz): C1 phase_transition, C2 transit_formation, C5 mission_risk (>= 0.75 ->
  app:risk), C6 replan_needed (-> Sonnet replan), C8 relay_needed (-> a relay drone).
- ``track`` (per fused detection track, one at a time): C3 verdict; confirm_* is irreversible and
  needs low-altitude evidence; request_inspection dispatches the cheapest feasible drone.
- ``report`` (per worker report that is not routine): C4 response.
- ``fit`` (generated: one score per drone x sector, Jev only): C7, a term of the Hungarian
  sector-allocation cost (§7) solved by formation_planner (/swarm/allocate).

Mission planning is app code on the coordinator's LLM (Sonnet, effort medium): a structured plan,
validated and clamped; re-asked once, then a default plan. Orders go down as directives through
comms_sim (lossy), so standing orders are re-sent until the drone's status shows them.
Single-threaded: engine callbacks and the FSM never race.
"""
from __future__ import annotations

import json
import math
from collections import deque

import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Vector3
from jev_ai_msgs.action import Reason
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
from rclpy.task import Future
from std_msgs.msg import String
from swarm_interfaces.action import Mission
from swarm_interfaces.msg import DroneStatus, Track, TrackArray
from swarm_interfaces.srv import Allocate, Formation

from drone_swarm import scenario
from drone_swarm.drone_agent import mmss
from jev_ai import Decision, DecisionEngine, Hierarchy
from jev_ai.ros.engine import run

PHASES = ['LAUNCH', 'TRANSIT', 'SEARCH', 'CONFIRM', 'RECOVER', 'DONE']
FIT = ['poor', 'fair', 'good', 'excellent']
W_DIST, W_FIT, W_SW, W_OPP, BIG = 1.0, 1.0, 0.3, 0.5, 1e4
ROUTINE = {'nominal', 'candidate_found', 'task_complete'}
DEFAULT_PLAN = {'survey_altitude_m': 30.0, 'transit_formation': 'v', 'formation_spacing_m': 8.0,
                'search_end_frac': 0.7, 'recover_frac': 0.85, 'priority_sectors': [],
                'rationale': 'default plan'}
PLAN_SCHEMA = {
    'type': 'object', 'additionalProperties': False,
    'required': list(DEFAULT_PLAN),
    'properties': {
        'survey_altitude_m': {'type': 'number',
                              'description': 'survey altitude, 20-40 m; default 30'},
        'transit_formation': {'type': 'string',
                              'enum': ['v', 'column', 'line', 'diamond', 'circle']},
        'formation_spacing_m': {'type': 'number', 'description': 'metres, 8-12; default 8'},
        'search_end_frac': {'type': 'number',
                            'description': 'latest end of SEARCH as a fraction of the mission '
                                           'clock, 0.5-0.8; default 0.7'},
        'recover_frac': {'type': 'number',
                         'description': 'latest start of RECOVER, 0.75-0.9; default 0.85'},
        'priority_sectors': {'type': 'array', 'items': {'type': 'string'}},
        'rationale': {'type': 'string'}}}
EXPECT = {'takeoff': 'launch', 'rtb': 'recall', 'hold': 'ordered'}


def catalog_for(path: str, drones: list[str], sectors: list[str]) -> dict:
    """The coordinator catalog plus the generated C7 ``fit`` set for these drones x sectors."""
    cat = yaml.safe_load(open(path))
    cat['question_sets']['fit'] = {
        'state_keys': ['mission', 'pairs'], 'tick_hz': 0, 'augment': False,
        'questions': {f'fit_{d}_{s}': {
            'type': 'score', 'act': 'expectation', 'classes': [{'name': c} for c in FIT],
            'instructions': f'How well suited is {d} to survey sector {s} now (distance, energy '
                            'after the sector, how much of it is left)?'}
                      for d in drones for s in sectors}}
    return cat


class Coordinator(Node):
    def __init__(self):
        super().__init__('mission_coordinator')
        self.p = scenario.from_node(self)
        self.drones = scenario.drones(self.p)
        self.sectors = scenario.sectors(self.p)
        self.snames = [scenario.sector_name(i) for i in range(len(self.sectors))]
        share = get_package_share_directory('drone_swarm')
        path = self.declare_parameter('catalog', share + '/config/coordinator.yaml').value
        self.auto_start = self.declare_parameter('auto_start', True).value
        self.use_llm_plan = self.declare_parameter('plan_with_llm', True).value
        run_id = self.declare_parameter('run_id', '').value or None

        self.status: dict[str, DroneStatus] = {}
        self.seen: dict[str, float] = {}
        self.wind = (0.0, 0.0)
        self.cov = scenario.Coverage(self.p)
        self.tracks: dict[str, dict] = {}
        self.local: dict[str, str] = {}               # 'uav1:uav1-3' -> coordinator track id
        self.track_q: deque[str] = deque()
        self.track_focus: tuple[str, float] | None = None
        self.report_q: deque[dict] = deque()
        self.report_focus: tuple[dict, float] | None = None
        self.last_report_kind: dict[tuple[str, str], float] = {}
        self.events: deque[tuple[float, str]] = deque(maxlen=20)
        self.phase, self.phase_t, self.stage = 'IDLE', self.now(), ''
        self.t0: float | None = None
        self.t_max = self.p['t_max_s']
        self.plan = dict(DEFAULT_PLAN)
        self.planning = False
        self.last_plan_t = -math.inf
        self.assign: dict[str, int] = {}               # drone -> sector index
        self.orders: dict[str, dict] = {}              # drone -> standing order
        self.recalled: set[str] = set()
        self.relay: str | None = None
        self.u_fit: dict[tuple[str, int], float] = {}
        self.fit_pending: float | None = None
        self.slots: dict[str, tuple[float, float, float]] = {}
        self.rtb_queue: list[str] = []
        self.mission_done: Future | None = None
        self.mission_gh = None
        self.result_summary = ''
        self.objective = ''
        self.was_ready = False
        self.allocating = False
        self.surveyed: set[int] = set()             # sectors whose lawnmower finished
        self.survey_progress: dict[int, float] = {}    # sector -> best FC progress seen

        self.engine = DecisionEngine(self, catalog_for(path, self.drones, self.snames),
                                     encode_state=self.encode, mask=self.mask,
                                     require=self.require, on_decision=self.on_decision,
                                     current=self.current, run_id=run_id, agent='/coordinator',
                                     jev='/jev/decide',
                                     all_phases=True)
        self.hierarchy = Hierarchy(self, agent='/coordinator', on_report=self.on_report)
        self.create_subscription(DroneStatus, '/swarm/status', self._status, 50)
        self.create_subscription(Vector3, '/env/wind', lambda m: setattr(
            self, 'wind', (m.x, m.y)), 10)
        self.tracks_pub = self.create_publisher(TrackArray, 'tracks', 10)
        self.phase_pub = self.create_publisher(String, 'phase', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.allocate_cli = self.create_client(Allocate, '/swarm/allocate')
        self.formation_cli = self.create_client(Formation, '/swarm/formation')
        self.llm = ActionClient(self, Reason, 'llm/reason')
        self.mission_server = ActionServer(self, Mission, 'mission',
                                           execute_callback=self.execute_mission)
        self.create_timer(1.0, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self) -> float:
        return self.now() - self.t0 if self.t0 is not None else 0.0

    def event(self, text: str) -> None:
        self.events.append((self.now(), text))
        self.get_logger().info(text)

    # --- inputs ----------------------------------------------------------------------------------

    def _status(self, m: DroneStatus) -> None:
        # telemetry rides the same radio: nothing gets through a lost link
        if m.link_q < 0.1 and m.drone in self.status:
            return
        self.status[m.drone] = m
        self.seen[m.drone] = self.now()
        if m.task == 'SURVEY' and m.task_detail.startswith('sector '):
            i = self.snames.index(m.task_detail.split(' ', 1)[1])
            self.survey_progress[i] = max(self.survey_progress.get(i, 0.0), m.progress)
        if m.airborne:
            self.cov.see(m.position.x, m.position.y, m.position.z,
                         math.hypot(m.velocity.x, m.velocity.y))

    def on_report(self, msg, payload: dict) -> None:
        d = msg.agent.lstrip('/')
        if d not in self.drones:
            return
        tr = payload.get('track')
        if tr:
            beliefs = {b.question: dict(zip(b.classes, b.probabilities)) for b in msg.beliefs}
            self.fuse(d, tr, beliefs.get('detection_class'))
        if msg.kind not in ROUTINE:
            key = (d, msg.kind)
            if self.now() - self.last_report_kind.get(key, -math.inf) > 30:
                self.last_report_kind[key] = self.now()
                self.report_q.append({'drone': d, 'kind': msg.kind, 'priority': msg.priority,
                                      'payload': payload})
                self.event(f'{d} reports {msg.kind}')

    def fuse(self, drone: str, tr: dict, belief: dict | None) -> None:
        """Associate a drone's local track with a coordinator track (id map, else within 8 m)."""
        key = f'{drone}:{tr["id"]}'
        tid = self.local.get(key)
        if tid is None:
            near = [(math.hypot(t['x'] - tr['x'], t['y'] - tr['y']), t['id'])
                    for t in self.tracks.values()]
            near = [n for n in near if n[0] < 8.0]
            tid = min(near)[1] if near else f'C{len(self.tracks) + 1}'
            self.local[key] = tid
        t = self.tracks.setdefault(tid, {'id': tid, 'x': tr['x'], 'y': tr['y'], 'reporters': {},
                                         'status': 'unresolved', 'verdict': 'await_data',
                                         'assigned': '', 'since': self.now(),
                                         'best_alt': 99.0, 'sightings': 0})
        scores = belief or tr.get('scores', {})
        before = (t['best_alt'], t['sightings'])
        t['reporters'][drone] = {'scores': scores, 'best_alt': tr['best_alt'],
                                 'sightings': tr['sightings'], 'x': tr['x'], 'y': tr['y'],
                                 'local': tr['id']}
        rs = list(t['reporters'].values())
        n = sum(r['sightings'] for r in rs)
        t['x'] = sum(r['x'] * r['sightings'] for r in rs) / n
        t['y'] = sum(r['y'] * r['sightings'] for r in rs) / n
        t['best_alt'], t['sightings'] = min(r['best_alt'] for r in rs), n
        w = [r['sightings'] / max(r['best_alt'], 5.0) for r in rs]
        t['probs'] = {c: sum(wi * r['scores'].get(c, 0.0) for wi, r in zip(w, rs)) / sum(w)
                      for c in scenario.DETECTOR_CLASSES}
        if t['best_alt'] <= 15 and t['verdict'] == 'request_inspection':
            t['verdict'] = 'await_data'                  # inspected: the request is answered
        if t['status'] == 'inspecting' and t['best_alt'] <= 15:
            t['status'] = 'unresolved'                   # inspected: ready for a verdict
            self.event(f'{tid} inspected by {drone} from {t["best_alt"]:.0f} m')
        if t['status'] == 'unresolved' and (t['best_alt'], t['sightings']) != before:
            self.enqueue(tid)

    def enqueue(self, tid: str) -> None:
        if tid not in self.track_q and (not self.track_focus or self.track_focus[0] != tid):
            self.track_q.append(tid)

    # --- mission ---------------------------------------------------------------------------------

    async def execute_mission(self, gh):
        if self.phase not in ('IDLE', 'DONE'):
            gh.abort()
            return Mission.Result(success=False, summary=f'mission already in {self.phase}')
        self.start_mission(gh.request.t_max_s or self.p['t_max_s'], gh.request.objective)
        self.mission_done = Future()
        self.mission_gh = gh
        await self.mission_done
        ok = self.phase == 'DONE'
        (gh.succeed if ok else gh.abort)()
        return Mission.Result(success=ok, summary=self.result_summary,
                              confirmed_survivors=self.confirmed('confirmed_survivor'),
                              coverage=self.cov.fraction())

    def start_mission(self, t_max: float, objective: str = '') -> None:
        self.t0, self.t_max = self.now(), float(t_max)
        self.event(f'mission accepted: {len(self.drones)} drones, T_max {mmss(self.t_max)}')
        self.objective = objective or 'Find and confirm every survivor in the area of ' \
            'operations, cover the area, and recover all drones.'
        if self.use_llm_plan:
            self.request_plan('initial plan')
        self.set_phase('LAUNCH')

    def request_plan(self, why: str) -> None:
        """Sonnet mission plan (DECISION_MODEL §3.2): validated, re-asked once, else default."""
        if self.planning or self.now() - self.last_plan_t < 60 or not self.llm.server_is_ready():
            return
        self.planning, self.last_plan_t = True, self.now()
        x0, x1, y0, y1 = self.p['ao']
        far = [round(scenario.link_q(self.p, math.hypot(x1, (s[1] + s[3]) / 2)), 2)
               for s in self.sectors]
        ctx = json.dumps({
            'objective': self.objective, 'reason': why, 'drones': len(self.drones),
            'area_of_operations': {'x': [x0, x1], 'y': [y0, y1], 'base': [0, 0]},
            'sectors': {n: {'y': [round(s[1]), round(s[3])], 'far_end_link_quality': q}
                        for n, s, q in zip(self.snames, self.sectors, far)},
            'obstacles_m': [{'x': o[0], 'y': o[1], 'height': o[3]} for o in
                            scenario.obstacles(self.p)],
            'no_fly_zones': [{'x': z[0], 'y': z[1], 'radius': z[2]}
                             for z in scenario.nfzs(self.p)],
            'wind_m_s': [round(w, 1) for w in self.wind], 'mission_clock_s': self.t_max,
            'battery_wh': self.p['E_0'], 'swarm_now': self.swarm_line(),
            'coverage_now': round(self.cov.fraction(), 2),
            # live Sonnet maxed the altitude "to clear obstacles": state what flight already
            # handles and what altitude really trades
            'flight_facts': {
                'obstacle_avoidance': 'automatic at any altitude (flight controllers steer '
                                      'around obstacles); altitude need not clear them',
                'confirmation': 'a detection is confirmed only after an inspection from 15 m '
                                'or lower, which costs a drone time and energy',
                'survey_altitude_tradeoff': {
                    f'{h} m': {'p_detect_per_pass': round(scenario.p_detect(
                        self.p, h, self.p['v_survey']), 2),
                               'swath_m': round(scenario.lane_spacing(self.p, h))}
                    for h in (20, 30, 40)}}})
        goal = Reason.Goal(system='You plan search-and-rescue missions for a drone swarm. Return '
                           'only the JSON plan; respect the stated ranges.',
                           context=ctx, response_schema=json.dumps(PLAN_SCHEMA))
        self._ask_plan(goal, attempt=0)

    def _ask_plan(self, goal, attempt: int) -> None:
        def accepted(fut):
            gh = fut.result()
            if not gh.accepted:
                return self._plan_done(None, 'rejected')
            gh.get_result_async().add_done_callback(lambda f: done(f.result().result))

        def done(res):
            plan, err = None, res.error
            if res.success:
                try:
                    plan = self.validate_plan(json.loads(res.json or res.text))
                except (ValueError, KeyError, TypeError) as e:
                    err = f'invalid plan: {e}'
            if plan is None and attempt == 0:
                return self._ask_plan(goal, 1)
            self._plan_done(plan, err, res)
        self.llm.send_goal_async(goal).add_done_callback(accepted)

    def validate_plan(self, raw: dict) -> dict:
        clamp = lambda v, lo, hi: min(hi, max(lo, float(v)))    # noqa: E731
        if raw['transit_formation'] not in ('v', 'column', 'line', 'diamond', 'circle'):
            raise ValueError(f'formation {raw["transit_formation"]!r}')
        return {'survey_altitude_m': clamp(raw['survey_altitude_m'], 20, 40),
                # spacing >= 2 d_sep, so the separation reflex never fights the formation
                'formation_spacing_m': clamp(raw['formation_spacing_m'], 2 * self.p['d_sep'], 12),
                'transit_formation': raw['transit_formation'],
                'search_end_frac': clamp(raw['search_end_frac'], 0.5, 0.8),
                'recover_frac': clamp(raw['recover_frac'], 0.75, 0.9),
                'priority_sectors': [s for s in raw.get('priority_sectors', [])
                                     if s in self.snames],
                'rationale': str(raw.get('rationale', ''))[:300]}

    def _plan_done(self, plan, err, res=None) -> None:
        self.planning = False
        if plan is None:
            self.event(f'plan: LLM failed ({err}); keeping {self.plan["rationale"]}')
            return
        self.plan = plan
        cost = f', ${res.cost_usd:.4f}' if res is not None else ''
        self.event(f'plan ({res.model if res else "?"}{cost}): survey '
                   f'{plan["survey_altitude_m"]:.0f} m, {plan["transit_formation"]} formation at '
                   f'{plan["formation_spacing_m"]:.0f} m,'
                   f' search ends by {plan["search_end_frac"]:.0%}; {plan["rationale"][:120]}')

    # --- FSM -------------------------------------------------------------------------------------

    def set_phase(self, phase: str) -> None:
        if phase == self.phase:
            return
        self.event(f'phase {self.phase} -> {phase} at {mmss(self.elapsed())}')
        self.phase, self.phase_t, self.stage = phase, self.now(), ''
        self.phase_pub.publish(String(data=phase))
        self.hierarchy.direct('phase', payload={'phase': phase, 'elapsed': self.elapsed()},
                              ttl_s=30)
        enter = getattr(self, f'enter_{phase.lower()}', None)
        if enter:
            enter()

    def tick(self) -> None:
        now = self.now()
        if self.phase == 'IDLE':
            if self.auto_start and self.t0 is None and len(self.status) == len(self.drones) and \
                    self.engine.jev.service_is_ready() and now > 5.0:
                self.start_mission(self.p['t_max_s'])
        else:
            getattr(self, f'tick_{self.phase.lower()}', lambda: None)()
            ready = self.ready() or self.forced()
            if ready and not self.was_ready:
                self.engine.trigger('ready', 'global')        # on the transition only
            self.was_ready = ready
            self.resend_orders()
            self.pump()
        self.publish()

    def ready(self) -> bool:
        """The current phase's work is done (C1 advance is allowed)."""
        air = [self.status[d] for d in self.available()]
        if self.phase == 'LAUNCH':
            return len(self.status) == len(self.drones) and all(
                s.airborne and abs(s.position.z - self.p['alt_transit']) < 3
                for s in self.status.values())
        if self.phase == 'TRANSIT':
            return self.stage == 'move' and self.at_slots(4.0)
        if self.phase == 'SEARCH':
            return self.cov.fraction() >= 0.9 or len(self.surveyed) == len(self.sectors)
        if self.phase == 'CONFIRM':
            return not any(t['status'] in ('unresolved', 'inspecting')
                           for t in self.tracks.values())
        if self.phase in ('RECOVER', 'ABORT'):
            return all(not s.airborne for s in self.status.values()) and not air
        return False

    def note_surveyed(self) -> None:
        for i, prog in self.survey_progress.items():
            if prog >= 0.97 and i not in self.surveyed:
                self.surveyed.add(i)
                by = [d for d, a in self.assign.items() if a == i]
                self.event(f'sector {self.snames[i]} surveyed by {", ".join(by)} '
                           f'({self.cov.fraction(self.sectors[i]):.0%} covered)')

    def forced(self) -> bool:
        e = self.elapsed() / self.t_max
        return (self.phase == 'SEARCH' and e >= self.plan['search_end_frac']) or \
            (self.phase == 'CONFIRM' and e >= self.plan['recover_frac']) or \
            (self.phase in ('LAUNCH', 'TRANSIT') and self.now() - self.phase_t > 120)

    def advance(self) -> None:
        if self.phase in ('RECOVER', 'ABORT'):
            return self.finish()
        self.set_phase(PHASES[PHASES.index(self.phase) + 1])

    def finish(self) -> None:
        landed = sum(not s.airborne for s in self.status.values())
        self.result_summary = (f'{self.confirmed("confirmed_survivor")} survivors confirmed, '
                               f'{self.cov.fraction():.0%} covered, {landed}/{len(self.drones)} '
                               f'landed in {mmss(self.elapsed())}')
        prev = self.phase
        self.set_phase('DONE')
        self.event(f'mission {"aborted" if prev == "ABORT" else "done"}: {self.result_summary}')
        if self.mission_done and not self.mission_done.done():
            self.mission_done.set_result(True)

    # phase bodies -------------------------------------------------------------------------------

    def enter_launch(self) -> None:
        for k, d in enumerate(self.drones):            # staggered takeoff, 3 s apart
            self.order(d, 'takeoff', {'altitude': self.p['alt_transit']}, delay=3.0 * k)

    def enter_transit(self) -> None:
        self.stage = 'assemble'
        self.form(self.formation(), (12.0, 0.0), 0.0)

    def tick_transit(self) -> None:
        if self.stage == 'assemble' and self.at_slots(3.0):
            self.stage = 'move'                         # the assembled formation translates
            self.form(self.formation(), (self.p['ao'][0] + 50.0, 0.0), 0.0, keep=True)

    def enter_search(self) -> None:
        self.fit_pending = self.now()
        self.engine.trigger('allocation', 'fit', fresh=True)

    def tick_search(self) -> None:
        if self.fit_pending is not None and self.now() - self.fit_pending > 15:
            self.fit_pending = None                      # no fit answer: allocate without it
            self.allocate()
        self.note_surveyed()
        if self.fit_pending is None:
            self.handoff()
            self.dispatch_idle()

    def enter_confirm(self) -> None:
        for d in self.available():
            if self.orders.get(d, {}).get('kind') == 'survey':
                self.order(d, 'hold', {})
        for t in self.tracks.values():
            if t['status'] == 'unresolved':
                self.enqueue(t['id'])

    def tick_confirm(self) -> None:
        self.dispatch_idle()

    def enter_recover(self) -> None:
        self.stage = 'regroup'
        self.form(self.formation(), (15.0, 0.0), math.pi)

    def tick_recover(self) -> None:
        if self.stage == 'regroup' and (self.at_slots(5.0) or self.now() - self.phase_t > 90):
            self.stage = 'land'
            # staggered landing: nearest first, 8 s apart
            order = sorted(self.available(), key=lambda d: math.hypot(
                self.status[d].position.x, self.status[d].position.y))
            for k, d in enumerate(order):
                self.order(d, 'rtb', {}, delay=8.0 * k)

    def enter_abort(self) -> None:
        for d in self.drones:
            self.order(d, 'rtb', {})

    # --- orders ----------------------------------------------------------------------------------

    def order(self, drone: str, kind: str, payload: dict, delay: float = 0.0) -> None:
        self.orders[drone] = {'kind': kind, 'payload': payload, 'due': self.now() + delay,
                              'sent': None, 'acked': False, 'expect': self.expect(kind, payload)}

    @staticmethod
    def expect(kind: str, payload: dict) -> str:
        if kind == 'survey':
            return f'sector {payload["sector"]}'
        if kind == 'inspect':
            return f'inspect {payload["track"]}'
        if kind in ('goto', 'formation', 'relay'):      # per point: a stale status of the previous
            x, y = payload['point'][:2]                  # slot must not ack the next one
            return f'{kind} ({x:.0f}, {y:.0f})'
        return EXPECT.get(kind, kind)

    def resend_orders(self) -> None:
        """Send due orders; re-send every 8 s until the drone's status shows it (lossy link)."""
        now = self.now()
        for d, o in self.orders.items():
            s = self.status.get(d)
            if s and s.task_detail.startswith(o['expect']):
                o['acked'] = True
            if o['acked'] or now < o['due'] or (o['sent'] and now - o['sent'] < 8.0):
                continue
            if o['sent'] and s and s.link_q < 0.2 and o['kind'] != 'rtb':
                continue                                  # §4.8: no dispatch over a bad link
            o['sent'] = now
            self.hierarchy.direct(o['kind'], target=f'/{d}', payload=o['payload'], ttl_s=30)

    def order_done(self, d: str) -> bool:
        """The drone acknowledged the standing order and its flight controller is idle (finished;
        a drone that diverted on its own, e.g. a W1 inspection, is busy, not done)."""
        o, s = self.orders.get(d), self.status.get(d)
        return bool(o and o['acked'] and s and s.task in ('IDLE', 'LANDED'))

    def available(self) -> list[str]:
        now = self.now()
        return [d for d, s in self.status.items()
                if now - self.seen.get(d, 0) < 15 and s.airborne and d not in self.recalled and
                not s.critical and s.margin > 0.05 and 'forced_rtb' not in s.reflexes]

    def formation(self) -> str:
        return self.engine.current('global').get('transit_formation') or \
            self.plan['transit_formation']

    def form(self, shape: str, centre, heading: float, keep: bool = False) -> None:
        """Formation slots around centre via /swarm/formation; keep = translate the current
        assignment (the formation moves rigidly) instead of re-solving it."""
        drones = [d for d in self.drones if d in self.available()]
        if not drones:
            return
        alt = self.p['alt_transit']
        if keep and all(d in self.slots for d in drones):
            cx = sum(self.slots[d][0] for d in drones) / len(drones)
            cy = sum(self.slots[d][1] for d in drones) / len(drones)
            for d in drones:
                x, y, _ = self.slots[d]
                self.slots[d] = (x - cx + centre[0], y - cy + centre[1], alt)
                self.order(d, 'formation', {'point': list(self.slots[d])})
            return
        if not self.formation_cli.service_is_ready():
            self.get_logger().warn('formation_planner not ready', throttle_duration_sec=5)
            return
        req = Formation.Request(shape=shape, spacing=float(self.plan['formation_spacing_m']),
                                centre=Point(x=float(centre[0]), y=float(centre[1]), z=alt),
                                heading=float(heading),
                                positions=[self.status[d].position for d in drones])

        def done(fut):
            res = fut.result()
            for d, sl in zip(drones, res.slots):
                self.slots[d] = (sl.x, sl.y, sl.z)
                self.order(d, 'formation', {'point': [sl.x, sl.y, sl.z]})
            self.event(f'{res.message} at ({centre[0]:.0f}, {centre[1]:.0f})')
        self.formation_cli.call_async(req).add_done_callback(done)

    def at_slots(self, tol: float) -> bool:
        ds = [d for d in self.available() if d in self.slots]
        return bool(ds) and all(math.hypot(self.status[d].position.x - self.slots[d][0],
                                           self.status[d].position.y - self.slots[d][1]) < tol
                                for d in ds)

    # --- allocation (DECISION_MODEL §7) ----------------------------------------------------------

    def allocate(self, sectors: list[int] | None = None, drones: list[str] | None = None) -> None:
        sectors = [i for i in (sectors if sectors is not None else range(len(self.sectors)))
                   if self.cov.fraction(self.sector_rect(i)) < 0.85]
        drones = drones if drones is not None else [d for d in self.available()
                                                    if d != self.relay]
        if not sectors or not drones or self.allocating or \
                not self.allocate_cli.service_is_ready():
            return
        self.allocating = True
        cost = []
        for d in drones:
            s = self.status[d]
            here = (s.position.x, s.position.y)
            for i in sectors:
                x0, y0, x1, y1 = self.sectors[i]
                dist = math.hypot(x0 - here[0], (y0 + y1) / 2 - here[1])
                need = scenario.survey_wh(self.p, self.sectors[i], here, self.wind) + \
                    scenario.leg_wh(self.p, (x1, (y0 + y1) / 2), (0.0, 0.0), self.wind) + \
                    self.p['E_land_wh']
                have = s.energy_frac * self.p['E_0'] - self.p['reserve'] * self.p['E_0']
                cost.append(W_DIST * dist / 400 + W_FIT * (1 - self.u_fit.get((d, i), 0.5)) +
                            W_SW * (self.assign.get(d, i) != i) + BIG * (need > have))
        req = Allocate.Request(cost=cost, rows=len(drones), cols=len(sectors))

        def done(fut):
            self.allocating = False
            a = fut.result().assignment
            for d, j in zip(drones, a):
                if j < 0:
                    continue
                self.survey(d, sectors[j])
            self.event('allocation: ' + ', '.join(
                f'{d}->{self.snames[sectors[j]]}' for d, j in zip(drones, a) if j >= 0))
        self.allocate_cli.call_async(req).add_done_callback(done)

    def sector_rect(self, i: int):
        return self.sectors[i]

    def idle(self) -> list[str]:
        """Available drones with no live order (finished it or never had one)."""
        return [d for d in self.available() if d != self.relay and (
            d not in self.orders or self.order_done(d) or self.orders[d]['kind'] == 'hold')]

    def handoff(self) -> None:
        """Sectors left unfinished by a drone that is gone or busy go to idle drones."""
        if not self.assign or self.allocating:
            return
        avail, idle = set(self.available()), set(self.idle())
        orphan = []
        for i in range(len(self.sectors)):
            owners = [d for d, a in self.assign.items() if a == i]
            if i in self.surveyed or any(d in avail and d not in idle for d in owners):
                continue                                 # done, or its drone is busy
            back = [d for d in owners if d in idle]
            if back:                                     # e.g. after an inspection: resume
                if self.orders.get(back[0], {}).get('kind') != 'survey' or \
                        self.order_done(back[0]):
                    self.survey(back[0], i)
            elif not any(d in avail for d in owners):
                orphan.append(i)                         # its drone is gone
        free = [d for d in idle if self.assign.get(d) in self.surveyed or d not in self.assign]
        if orphan and free:
            self.event(f'handoff: sectors {[self.snames[i] for i in orphan]} to {free}')
            self.allocate(orphan, free)

    def survey(self, d: str, i: int) -> None:
        x0, y0, x1, y1 = self.sectors[i]
        self.assign[d] = i
        self.order(d, 'survey', {'sector': self.snames[i], 'area': [[x0, y0], [x1, y1]],
                                 'altitude': self.plan['survey_altitude_m']})

    # --- tracks and dispatch ---------------------------------------------------------------------

    def cheapest(self, t: dict, exclude: set[str] = frozenset(),
                 idle_only: bool = False) -> tuple[str, float] | None:
        """d* = argmin travel time + w_opp · remaining coverage value (§7), feasible drones;
        a drone orbiting a detection of its own (its W3 priority at work) is not feasible."""
        best = None
        idle = set(self.idle())
        for d in self.available():
            s = self.status[d]
            if d in exclude or d == self.relay or s.link_q < 0.2 or s.margin < 0.1 or \
                    s.task == 'ORBIT' or (idle_only and d not in idle) or \
                    self.orders.get(d, {}).get('kind') == 'inspect' and not self.order_done(d):
                continue
            travel = math.hypot(t['x'] - s.position.x, t['y'] - s.position.y) / self.p['v_transit']
            busy = self.orders.get(d, {}).get('kind') == 'survey' and not self.order_done(d)
            c = travel + W_OPP * (1 - s.progress) * 120 * busy
            if best is None or c < best[1]:
                best = (d, c)
        return best

    def dispatch(self, t: dict, exclude: set[str] = frozenset()) -> bool:
        """Likely survivors may pull a surveying drone; anything else waits for an idle one."""
        if self.phase not in ('SEARCH', 'CONFIRM'):
            return False                                  # formations and recovery stay intact
        likely = t.get('probs', {}).get('survivor', 0.0) >= 0.3
        pick = self.cheapest(t, exclude, idle_only=not likely)
        if pick is None:
            return False
        d = pick[0]
        t['status'], t['assigned'], t['since'] = 'inspecting', d, self.now()
        self.order(d, 'inspect', {'track': t['id'], 'point': [t['x'], t['y']],
                                  'subject': t['id']})
        self.event(f'dispatch {d} to inspect {t["id"]} ({t["x"]:.0f}, {t["y"]:.0f})')
        return True

    def dispatch_idle(self) -> None:
        """Re-queue stale inspections; dispatch tracks that asked for one, likeliest first."""
        for t in self.tracks.values():
            if t['status'] == 'inspecting' and self.now() - t['since'] > 150:
                t['status'] = 'unresolved'
                self.enqueue(t['id'])
        waiting = sorted((t for t in self.tracks.values() if t['status'] == 'unresolved' and
                          t['verdict'] == 'request_inspection'),
                         key=lambda t: -t.get('probs', {}).get('survivor', 0.0))
        for t in waiting:
            if not self.dispatch(t):
                break

    def resolve(self, t: dict, status: str) -> None:
        t['status'] = status
        self.event(f'{t["id"]}: {status} ({", ".join(t["reporters"])}, best view '
                   f'{t["best_alt"]:.0f} m)')
        by: dict[str, list[str]] = {}
        for d, r in t['reporters'].items():
            by.setdefault(d, []).append(r['local'])
        for d, ids in by.items():
            self.hierarchy.direct('resolved', target=f'/{d}', payload={'tracks': ids}, ttl_s=30)
        if t['assigned'] and self.orders.get(t['assigned'], {}).get('kind') == 'inspect' and \
                self.orders[t['assigned']]['payload'].get('track') == t['id']:
            self.orders.pop(t['assigned'])

    def confirmed(self, status: str) -> int:
        return sum(t['status'] == status for t in self.tracks.values())

    # --- per-item decisions ----------------------------------------------------------------------

    def pump(self) -> None:
        """One track and one report at a time through their question sets."""
        now = self.now()
        if self.track_focus and now - self.track_focus[1] > 40:
            self.track_focus = None                      # no final decision: move on
        if self.track_focus is None:
            while self.track_q:
                tid = self.track_q.popleft()
                if self.tracks[tid]['status'] in ('unresolved', 'inspecting'):
                    self.track_focus = (tid, now)
                    self.engine.trigger('track_update', 'track', subject=tid, fresh=True)
                    break
        if self.report_focus and now - self.report_focus[1] > 40:
            self.report_focus = None
        if self.report_focus is None and self.report_q:
            r = self.report_q.popleft()
            self.report_focus = (r, now)
            self.engine.trigger(f'report:{r["kind"]}', 'report', subject=r['drone'], fresh=True)

    def current(self, qset: str) -> dict[str, str] | None:
        if qset == 'track' and self.track_focus:
            return {'track_verdict': self.tracks[self.track_focus[0]]['verdict']}
        if qset == 'report':
            return {'report_response': 'acknowledge'}
        return None

    # --- jev_ai callbacks ------------------------------------------------------------------------

    def mission_line(self) -> str:
        if self.phase == 'IDLE':
            return 'not started'
        why = self.not_ready()
        return (f'{self.phase}{" (" + self.stage + ")" if self.stage else ""}, '
                f'{mmss(self.elapsed())} of {mmss(self.t_max)}; ' +
                ('ready to advance' if not why else f'not ready: {why}') +
                ('; time budget for this phase is spent' if self.forced() else ''))

    def not_ready(self) -> str:
        if self.ready():
            return ''
        return {'LAUNCH': 'drones still climbing', 'TRANSIT': 'formation not in position',
                'SEARCH': 'sectors still being surveyed', 'CONFIRM': 'tracks unresolved',
                'RECOVER': 'drones still airborne', 'ABORT': 'drones still airborne'}.get(
                    self.phase, 'n/a')

    def swarm_line(self) -> str:
        out = []
        now = self.now()
        for d in self.drones:
            s = self.status.get(d)
            if s is None or now - self.seen.get(d, 0) > 15:
                out.append(f'{d}: lost (no telemetry)')
                continue
            e = 'ample' if s.margin >= 0.3 else 'adequate' if s.margin >= 0.15 else \
                'tight' if s.margin >= 0.05 else 'critical'
            link = 'good' if s.link_q >= 0.7 else 'degraded' if s.link_q >= 0.3 else \
                'poor' if s.link_q >= 0.1 else 'lost'
            task = f'{s.task.lower()} {s.task_detail}'.strip()
            extra = (' recalled' if d in self.recalled else '') + \
                (f' faults {",".join(s.faults)}' if s.faults else '')
            out.append(f'{d}: {task} {s.progress:.0%}, energy {e} ({s.energy_frac:.0%}), '
                       f'link {link}{extra}')
        return '; '.join(out)

    def encode(self, qset: str) -> dict[str, str]:
        mission = self.mission_line()
        if qset == 'global':
            cov = ', '.join(f'{n} {self.cov.fraction(r):.0%}' for n, r in
                            zip(self.snames, self.sectors))
            unres = [t for t in self.tracks.values() if t['status'] in ('unresolved',
                                                                        'inspecting')]
            likely = sum(t.get('probs', {}).get('survivor', 0) > 0.4 for t in unres)
            tracks = (f'{self.confirmed("confirmed_survivor")} confirmed survivors, '
                      f'{self.confirmed("confirmed_non_survivor")} confirmed other, '
                      f'{len(unres)} unresolved ({likely} likely survivor), '
                      f'{self.confirmed("dismissed")} dismissed')
            margins = {d: s.margin for d, s in self.status.items() if s.airborne}
            tb = (f'search ends by {mmss(self.plan["search_end_frac"] * self.t_max)}, recover by '
                  f'{mmss(self.plan["recover_frac"] * self.t_max)}')
            if margins:
                worst = min(margins, key=margins.get)
                tb += f'; energy-limited by {worst} (margin {margins[worst]:.2f})'
            weak = [f'{d} ({self.snames[self.assign[d]] if d in self.assign else "-"}): link '
                    f'{"poor" if s.link_q < 0.3 else "degraded"} ({s.link_q:.2f})'
                    for d, s in self.status.items() if s.airborne and s.link_q < 0.7]
            comms = '; '.join(weak) + (f'; relay {self.relay}' if self.relay else '; no relay') \
                if weak else 'all links good'
            recent = [e for t, e in self.events if self.now() - t < 60 and any(
                k in e for k in ('fault', 'forced', 'lost', 'critical', 'recall', 'reports'))]
            return {'mission': mission, 'coverage': f'{self.cov.fraction():.0%}; {cov}',
                    'swarm': self.swarm_line(), 'tracks': tracks, 'time_budget': tb,
                    'comms': comms, 'risk_events': '; '.join(recent[-4:]) or 'none'}
        if qset == 'track':
            t = self.tracks.get(self.track_focus[0]) if self.track_focus else None
            if t is None:
                return {'mission': mission, 'track': 'none', 'evidence': 'none',
                        'nearby': 'none', 'inspection': 'none'}
            top = sorted(t.get('probs', {}).items(), key=lambda kv: -kv[1])[:3]
            spread = max((math.hypot(r['x'] - t['x'], r['y'] - t['y'])
                          for r in t['reporters'].values()), default=0.0)
            conf = [u for u in self.tracks.values() if u['status'] == 'confirmed_survivor']
            near = min((math.hypot(u['x'] - t['x'], u['y'] - t['y']) for u in conf),
                       default=None)
            pick = self.cheapest(t)
            insp = (f'inspection under way by {t["assigned"]}' if t['status'] == 'inspecting'
                    else f'inspected from {t["best_alt"]:.0f} m' if t['best_alt'] <= 15
                    else 'no inspection yet' + (f'; cheapest {pick[0]} (~{pick[1]:.0f} s)'
                                                if pick else '; no drone available'))
            return {'mission': mission,
                    'track': f'{t["id"]} at ({t["x"]:.0f}, {t["y"]:.0f}); reported by '
                             f'{", ".join(t["reporters"])}',
                    'evidence': f'{t["sightings"]} sightings, seen from {t["best_alt"]:.0f} m at '
                                f'best; scores ' + ', '.join(f'{k} {v:.2f}' for k, v in top) +
                                f'; position spread {spread:.0f} m',
                    'nearby': f'nearest confirmed survivor {near:.0f} m away' if near is not None
                              else 'no confirmed survivor nearby',
                    'inspection': insp}
        if qset == 'report':
            r = self.report_focus[0] if self.report_focus else None
            if r is None:
                return {'mission': mission, 'report': 'none', 'sender': 'none', 'sector': 'none'}
            d = r['drone']
            sline = next((x for x in self.swarm_line().split('; ') if x.startswith(d)), d)
            sector = 'none'
            if d in self.assign:
                i = self.assign[d]
                sector = f'{self.snames[i]}: {self.cov.fraction(self.sectors[i]):.0%} covered'
            pay = {k: v for k, v in r['payload'].items() if k in ('energy', 'margin', 'link',
                                                                  'faults', 'task')}
            return {'mission': mission,
                    'report': f'{r["kind"]} (priority {r["priority"]:.2f}): {json.dumps(pay)}',
                    'sender': sline, 'sector': sector}
        # fit
        pairs = []
        for d in self.drones:
            s = self.status.get(d)
            for i, n in enumerate(self.snames):
                if s is None:
                    pairs.append(f'{d}->{n}: unavailable')
                    continue
                x0, y0, x1, y1 = self.sectors[i]
                dist = math.hypot(x0 - s.position.x, (y0 + y1) / 2 - s.position.y)
                need = scenario.survey_wh(self.p, self.sectors[i], (s.position.x, s.position.y),
                                          self.wind) / self.p['E_0']
                left = s.energy_frac - need
                e = 'ample' if left >= 0.4 else 'adequate' if left >= 0.25 else \
                    'tight' if left >= 0.1 else 'critical'
                pairs.append(f'{d}->{n}: {dist:.0f} m away, energy after sector: {e}, sector '
                             f'{self.cov.fraction(self.sectors[i]):.0%} covered')
        return {'mission': mission, 'pairs': '; '.join(pairs)}

    def mask(self, qset: str) -> dict[str, set[str]]:
        if qset == 'global':
            out = set()
            if self.phase in ('IDLE', 'DONE'):
                return {'phase_transition': {'advance', 'recover_all', 'abort'}}
            if not (self.ready() or self.forced()):
                out.add('advance')
            if self.phase in ('LAUNCH', 'TRANSIT') and any(
                    s.airborne and s.link_lost_s > 0 for s in self.status.values()):
                out.add('advance')
            if self.phase in ('LAUNCH', 'RECOVER', 'ABORT'):
                out.add('recover_all')
            if self.phase == 'ABORT':
                out.add('abort')
            return {'phase_transition': out}
        if qset == 'track':
            t = self.tracks.get(self.track_focus[0]) if self.track_focus else None
            out = set()
            if t is None or t['best_alt'] > 15:
                out |= {'confirm_survivor', 'confirm_non_survivor'}
            if t is None or t['status'] == 'inspecting' or t['best_alt'] <= 15 or \
                    self.phase in ('RECOVER', 'ABORT', 'DONE'):
                out.add('request_inspection')
            return {'track_verdict': out}
        return {}

    def require(self, qset: str) -> dict[str, str]:
        # a per-item set with nothing in focus (a periodic refresh): FORCED status quo, no LLM
        if qset == 'track' and not self.track_focus:
            return {'track_verdict': 'await_data'}
        if qset == 'report' and not self.report_focus:
            return {'report_response': 'acknowledge'}
        if qset == 'global' and self.forced() and self.phase in ('SEARCH', 'CONFIRM', 'LAUNCH',
                                                                 'TRANSIT'):
            return {'phase_transition': 'advance'}
        return {}

    def on_decision(self, d: Decision) -> None:
        if not d.final:
            return
        getattr(self, f'decided_{d.question_set}')(d)

    def decided_global(self, d: Decision) -> None:
        pt = d.selected.get('phase_transition', 'stay')
        if self.phase not in ('IDLE', 'DONE'):
            if pt == 'advance' and (self.ready() or self.forced()):
                self.advance()
            elif pt == 'recover_all' and self.phase in ('TRANSIT', 'SEARCH', 'CONFIRM'):
                self.event(f'recover_all ({d.phase})')
                self.set_phase('RECOVER')
            elif pt == 'abort' and self.phase != 'ABORT':
                self.event(f'abort ({d.phase}): {d.rationale[:100]}')
                self.set_phase('ABORT')
        risk = float(d.values.get('mission_risk', 0.0)) / 3
        if risk >= 0.75:
            self.engine.trigger('risk', 'global', priority='high')
        if d.selected.get('replan_needed') == 'true' and self.phase in ('SEARCH', 'CONFIRM'):
            self.request_plan('replan_needed')       # sectors move only via handoff / C4
        if d.selected.get('relay_needed') == 'true' and self.relay is None and \
                self.phase == 'SEARCH':
            self.place_relay()

    def decided_track(self, d: Decision) -> None:
        if not self.track_focus:
            return
        t = self.tracks[self.track_focus[0]]
        v = d.selected.get('track_verdict', 'await_data')
        t['verdict'] = v
        if v == 'confirm_survivor':
            self.resolve(t, 'confirmed_survivor')
        elif v == 'confirm_non_survivor':
            self.resolve(t, 'confirmed_non_survivor')
        elif v == 'dismiss':
            self.resolve(t, 'dismissed')
        elif v == 'request_inspection' and t['status'] == 'unresolved' and \
                self.phase in ('SEARCH', 'CONFIRM'):            # else dispatch_idle, later
            if not self.dispatch(t):
                self.event(f'{t["id"]}: inspection requested, no drone free yet')
        self.track_focus = None

    def decided_report(self, d: Decision) -> None:
        if not self.report_focus:
            return
        r = self.report_focus[0]
        self.report_focus = None
        resp, drone = d.selected.get('report_response', 'acknowledge'), r['drone']
        if resp != 'acknowledge':
            self.event(f'{drone} {r["kind"]}: {resp}')
        if resp == 'dispatch_support':
            tr = r['payload'].get('track')
            tid = self.local.get(f'{drone}:{tr["id"]}') if tr else None
            if tid and self.tracks[tid]['status'] == 'unresolved':
                self.dispatch(self.tracks[tid], exclude={drone})
        elif resp == 'reassign_sector' and drone in self.assign:
            self.order(drone, 'hold', {})
            self.allocate([self.assign.pop(drone)], [d for d in self.idle() if d != drone])
        elif resp == 'recall_drone':
            self.recalled.add(drone)
            self.order(drone, 'rtb', {})
        elif resp == 'replan':
            self.request_plan(f'{drone} reported {r["kind"]}')

    def decided_fit(self, d: Decision) -> None:
        for i, n in enumerate(self.snames):
            for dr in self.drones:
                v = d.values.get(f'fit_{dr}_{n}')
                if isinstance(v, (int, float)):
                    self.u_fit[(dr, i)] = float(v) / 3
        if self.fit_pending is not None:
            self.fit_pending = None
            self.allocate()

    def place_relay(self) -> None:
        """C8: a drone on the base -> worst-linked sector line where both hops have q >= 0.7."""
        weak = [(s.link_q, d) for d, s in self.status.items() if s.airborne and d in self.assign]
        if not weak:
            return
        q, d_weak = min(weak)
        x0, y0, x1, y1 = self.sectors[self.assign[d_weak]]
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        point = [cx / 2, cy / 2, self.p['alt_transit']]
        cands = [d for d in self.idle() if d != d_weak] or \
            [d for d in self.available() if d != d_weak and d not in self.assign]
        if not cands:
            self.event(f'relay needed for {d_weak} (q {q:.2f}) but no drone is free')
            return
        self.relay = min(cands, key=lambda d: math.hypot(self.status[d].position.x - point[0],
                                                         self.status[d].position.y - point[1]))
        self.order(self.relay, 'relay', {'point': point})
        self.event(f'relay: {self.relay} to ({point[0]:.0f}, {point[1]:.0f}) for {d_weak} '
                   f'(q {q:.2f})')

    # --- outputs ---------------------------------------------------------------------------------

    def publish(self) -> None:
        arr = TrackArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for t in self.tracks.values():
            probs = t.get('probs', {})
            arr.tracks.append(Track(id=t['id'], position=Point(x=float(t['x']), y=float(t['y'])),
                                    classes=list(probs), probabilities=list(probs.values()),
                                    sightings=int(t['sightings']),
                                    best_altitude=float(t['best_alt']), status=t['status'],
                                    assigned=t['assigned'], reporters=list(t['reporters'])))
        self.tracks_pub.publish(arr)
        tracks = list(self.tracks.values())
        kv = [('phase', f'{self.phase} {self.stage}'.strip()),
              ('elapsed', f'{mmss(self.elapsed())} of {mmss(self.t_max)}'),
              ('coverage (estimated)', f'{self.cov.fraction():.0%}'),
              ('tracks', f'{self.confirmed("confirmed_survivor")} survivors confirmed, '
                         f'{self.confirmed("confirmed_non_survivor")} other, '
                         f'{self.confirmed("dismissed")} dismissed, '
                         f'{sum(t["status"] in ("unresolved", "inspecting") for t in tracks)}'
                         ' open'),
              ('allocation', ', '.join(f'{d}->{self.snames[i]}' for d, i in
                                       sorted(self.assign.items())) or '-'),
              ('plan', f'{self.plan["transit_formation"]} formation, survey '
                       f'{self.plan["survey_altitude_m"]:.0f} m ({self.plan["rationale"][:60]})'),
              ('relay', self.relay or '-')]
        kv += [(f'event {i + 1}', f'{mmss(t - (self.t0 or t))} {e}')
               for i, (t, e) in enumerate(list(self.events)[-6:])]
        st = DiagnosticStatus(name='mission_coordinator', hardware_id='swarm',
                              level=DiagnosticStatus.OK, message=self.mission_line(),
                              values=[KeyValue(key=k, value=v) for k, v in kv])
        a = DiagnosticArray(status=[st])
        a.header.stamp = arr.header.stamp
        self.diag_pub.publish(a)
        if getattr(self, 'mission_gh', None) is not None and self.phase != 'DONE':
            self.mission_gh.publish_feedback(Mission.Feedback(
                phase=self.phase, elapsed_s=self.elapsed(), coverage=self.cov.fraction(),
                confirmed=self.confirmed('confirmed_survivor')))


def main(args=None):
    run(Coordinator, args, threads=1)
