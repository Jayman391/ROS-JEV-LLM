"""drone_agent (B5): one per drone (namespace /uavK), a jev_ai DecisionEngine + Hierarchy worker.

The question set ``step`` (W1-W7, ``config/drone.yaml``) runs at 1 Hz with act-then-revise:
- W1 ``next_action`` becomes a ``task`` goal for the C++ flight_controller (SYSTEM_MODEL §5.2):
  ``continue`` executes the coordinator's standing order, ``inspect`` orbits the best unresolved
  local detection (irreversible past its commit point, descent below 15 m), ``hold``,
  ``return_to_base``, ``request_support`` (a priority report, no motion change), ``land_now``.
- W2/W3 are the beliefs and priority sent up with reports; W5/W7 decide when and what to report;
  W6 ``unusual`` re-triggers the engine at high priority; W4 is advisory (H6 is deterministic).
Masks (DECISION_MODEL §5) come from DroneStatus, the same numbers the C++ safety layer uses.
Coordinator orders arrive as jev_ai directives through comms_sim; each is a decision event in the
directive's noise epoch, so drones answering the same order share their Gumbel noise.
"""
from __future__ import annotations

import math

from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from swarm_interfaces.action import Task
from swarm_interfaces.msg import DetectionArray, DroneStatus, LinkStatus
from geometry_msgs.msg import Point, Vector3

from drone_swarm import scenario
from jev_ai import Decision, DecisionEngine, Hierarchy
from jev_ai.ros.engine import run

W1 = ['continue', 'inspect', 'hold', 'return_to_base', 'request_support', 'land_now']
COMMIT_ALT = 15.0          # inspect becomes irreversible below this altitude
SUPPORT_WAIT_S = 20.0
REPORT_GAP_S = 3.0


def compass(dx: float, dy: float) -> str:
    i = round(math.atan2(dy, dx) / (math.pi / 4)) % 8
    return ['E', 'NE', 'N', 'NW', 'W', 'SW', 'S', 'SE'][i]


def mmss(t: float) -> str:
    return f'{int(t // 60)}:{int(t % 60):02d}'


class DroneAgent(Node):
    def __init__(self):
        super().__init__('drone_agent')
        self.p = scenario.from_node(self)
        self.name = self.get_namespace().strip('/') or 'uav1'
        self.home = scenario.pad(self.p, int(self.name[3:]) - 1)
        share = get_package_share_directory('drone_swarm')
        catalog = self.declare_parameter('catalog', share + '/config/drone.yaml').value
        run_id = self.declare_parameter('run_id', '').value or None
        jev = self.declare_parameter('jev_service', '/jev/decide').value
        llm = self.declare_parameter('llm_action', '/llm/reason').value

        self.status: DroneStatus | None = None
        self.link: LinkStatus | None = None
        self.link_bucket, self.link_since = 'good', self.now()
        self.peers: dict[str, DroneStatus] = {}
        self.wind = (0.0, 0.0)
        self.tracks: dict[str, dict] = {}          # local detection tracks
        self.encoded: str = ''                     # track id described by the last encode
        self.order: dict | None = None             # the coordinator's standing order
        self.mission = {'phase': 'not started', 'elapsed': 0.0, 't': self.now()}
        self.support_since: float | None = None
        self.active: tuple[str, str] | None = None   # (primitive, detail) of our goal
        self.goal_seq = 0
        self.inspecting: str | None = None         # local track under a W1 inspection
        self.committed = False
        self.last_report, self.last_situation, self.last_unusual = -math.inf, '', -math.inf
        self.last_logged = ''

        self.create_subscription(DroneStatus, 'status', self._status, 10)
        self.create_subscription(LinkStatus, 'link', self._link, 10)
        self.create_subscription(DetectionArray, 'detections', self._detections, 10)
        self.create_subscription(DroneStatus, '/swarm/status', self._peer, 50)
        self.create_subscription(Vector3, '/env/wind', lambda m: setattr(
            self, 'wind', (m.x, m.y)), 10)
        self.task = ActionClient(self, Task, 'task')
        self.engine = DecisionEngine(self, catalog, encode_state=self.encode, mask=self.mask,
                                     require=self.require, on_decision=self.on_decision,
                                     jev=jev, llm=llm, run_id=run_id, agent=f'/{self.name}',
                                     all_phases=True)
        self.hierarchy = Hierarchy(self, engine=self.engine, qset='step',
                                   on_directive=self.on_directive,
                                   issuer_allowlist=['/coordinator'])
        self.create_timer(0.5, self.watch)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ----------------------------------------------------------------------------------

    def _status(self, m: DroneStatus) -> None:
        self.status = m

    def _peer(self, m: DroneStatus) -> None:
        if m.drone != self.name:
            self.peers[m.drone] = m

    def _link(self, m: LinkStatus) -> None:
        b = 'good' if m.quality >= 0.7 else 'degraded' if m.quality >= 0.3 else \
            'poor' if m.quality >= 0.1 else 'lost'
        if b != self.link_bucket:
            self.link_bucket, self.link_since = b, self.now()
            if b in ('poor', 'lost'):
                self.engine.trigger('link_change', priority='high')
        self.link = m

    def _detections(self, msg: DetectionArray) -> None:
        for d in msg.detections:
            t = self.tracks.get(d.track_id)
            if t is None:
                t = self.tracks[d.track_id] = {'id': d.track_id, 'x': 0.0, 'y': 0.0, 'w': 0.0,
                                               'sightings': [], 'inspected': False,
                                               'resolved': False, 'reported': 0}
            w = 1.0 / (0.05 * d.altitude + 1.5) ** 2          # 1/σ² of the detector (§4.6)
            t['w'] += w
            t['x'] += (d.position.x - t['x']) * w / t['w']
            t['y'] += (d.position.y - t['y']) * w / t['w']
            t['sightings'].append((d.altitude, dict(zip(d.classes, d.scores))))
            if len(t['sightings']) == 1:
                self.engine.trigger('new_detection', subject=d.track_id)

    def on_directive(self, msg, payload: dict) -> None:
        """Orders from /coordinator. The engine then decides (a directive is a decision event)."""
        if msg.kind == 'phase':
            self.mission = {'phase': payload.get('phase', '?'),
                            'elapsed': float(payload.get('elapsed', 0.0)), 't': self.now()}
        elif msg.kind == 'resolved':
            for tid in payload.get('tracks', []):
                if tid in self.tracks:
                    self.tracks[tid]['resolved'] = True
        else:
            self.order = dict(payload, kind=msg.kind, t=self.now(), done=False)
            self.get_logger().info(self.describe_order())

    # --- local tracks ----------------------------------------------------------------------------

    @staticmethod
    def evidence(t: dict) -> tuple[float, dict[str, float]]:
        """(best altitude, mean class scores of the sightings within 5 m of it)."""
        best = min(a for a, _ in t['sightings'])
        near = [s for a, s in t['sightings'] if a <= best + 5]
        keys = near[0].keys()
        return best, {k: sum(s[k] for s in near) / len(near) for k in keys}

    def unresolved(self) -> list[dict]:
        return [t for t in self.tracks.values() if not t['inspected'] and not t['resolved']]

    def best_track(self) -> dict | None:
        """Best unresolved track by expected survivor value, nearer first on ties."""
        s = self.status
        here = (s.position.x, s.position.y) if s else self.home

        def value(t):
            _, sc = self.evidence(t)
            d = math.hypot(t['x'] - here[0], t['y'] - here[1])
            return sc.get('survivor', 0) + 0.5 * sc.get('animal', 0) - d / 1000
        if self.inspecting in self.tracks and not self.tracks[self.inspecting]['inspected']:
            return self.tracks[self.inspecting]
        cands = self.unresolved()
        return max(cands, key=value) if cands else None

    def peer_inspecting(self, t: dict) -> str:
        for name, s in self.peers.items():
            if s.task == 'ORBIT' and f'({t["x"]:.0f},' in s.task_detail:
                return name
        return ''

    # --- jev_ai callbacks ------------------------------------------------------------------------

    def encode(self, qset: str) -> dict[str, str]:
        s, now = self.status, self.now()
        m = self.mission
        elapsed = m['elapsed'] + (now - m['t'] if m['phase'] not in ('not started', 'DONE') else 0)
        mission = f'{m["phase"]}, {mmss(elapsed)} of {mmss(self.p["t_max_s"])} elapsed' \
            if m['phase'] != 'not started' else 'not started; on the pad awaiting orders'
        if s is None:
            return dict.fromkeys(self.engine.cat.question_sets['step'].state_keys, 'unknown')
        # task
        if s.task == 'LANDED':
            task = 'landed'
        elif s.task == 'IDLE':
            task = 'hovering, no task' if not self.order else \
                f'hovering; order done: {self.describe_order()}' if self.order['done'] else \
                f'hovering; order pending: {self.describe_order()}'
        elif s.task == 'ORBIT':
            where = 'orbiting' if s.position.z < COMMIT_ALT + 2 else 'en route'
            task = f'{s.task_detail}: {where}' \
                   f', {s.progress:.0%} done, ~{s.eta_s:.0f} s left'
        else:
            task = f'{s.task.lower()} {s.task_detail}'.strip() + \
                f': {s.progress:.0%} done, ~{s.eta_s:.0f} s left'
        # energy: m_E buckets (DECISION_MODEL §2.1)
        me = s.margin
        bucket = 'ample' if me >= 0.30 else 'adequate' if me >= 0.15 else \
            'tight' if me >= 0.05 else 'critical'
        need = s.energy_needed_frac + s.eta_s * 200.0 / 3600 / self.p['E_0']
        energy = f'{bucket} ({s.energy_frac:.0%} left, {min(need, 1.0):.0%} needed to finish ' \
                 f'and return)'
        # home
        dx, dy = self.home[0] - s.position.x, self.home[1] - s.position.y
        d = math.hypot(dx, dy)
        along = -(self.wind[0] * dx + self.wind[1] * dy) / d if d > 1 else 0.0
        home = f'{d:.0f} m, ~{d / self.p["v_transit"]:.0f} s, ' + \
            (f'headwind {along:.0f} m/s' if along > 0.5 else
             f'tailwind {-along:.0f} m/s' if along < -0.5 else 'crosswind')
        # "unchanged for": live Haiku read "good for 1 s" as "good for only 1 more second"
        link = f'{self.link_bucket} (unchanged for {now - self.link_since:.0f} s)' + \
            (f' via {self.link.via}' if self.link and self.link.via != 'base' else '')
        haz = [r.replace('_', ' ') for r in s.reflexes]
        if s.airborne and s.nearest_obstacle_m < 15:
            haz.append(f'obstacle {s.nearest_obstacle_m:.0f} m')
        if s.airborne and s.nearest_drone_m < 1.5 * self.p['d_sep']:
            haz.append(f'drone {s.nearest_drone_m:.0f} m')
        # detection
        t = self.best_track()
        self.encoded = t['id'] if t else ''
        self.engine.set_subject(self.encoded, 'step')
        detection, others = 'none', 'none'
        if t:
            best, sc = self.evidence(t)
            top = sorted(sc.items(), key=lambda kv: -kv[1])[:3]
            tx, ty = t['x'] - s.position.x, t['y'] - s.position.y
            peer = self.peer_inspecting(t)
            detection = (f'{t["id"]}, {math.hypot(tx, ty):.0f} m {compass(tx, ty)}: '
                         f'{len(t["sightings"])} sightings from {best:.0f} m; scores ' +
                         ', '.join(f'{k} {v:.2f}' for k, v in top) + '; ' +
                         ('inspection under way' if self.inspecting == t['id'] else
                          'not inspected') + '; ' +
                         ('not reported' if t['reported'] == 0 else 'reported') + '; ' +
                         (f'{peer} is inspecting it' if peer else 'no other drone assigned'))
            n = len(self.unresolved()) - 1
            others = f'{n} more unresolved' if n > 0 else 'none'
        faults = {'battery': 'battery draining 3x faster than expected', 'gps': 'gps degraded',
                  'motor': 'motor degraded'}
        health = '; '.join(faults.get(f, f) for f in s.faults) or 'nominal'
        if s.critical:
            health = 'critical: ' + health
        coord = 'no orders yet'
        if self.order:
            coord = f'last order {now - self.order["t"]:.0f} s ago: {self.describe_order()}' + \
                (' (done)' if self.order['done'] else '')
        if self.support_since is not None:
            coord += f'; support request pending {now - self.support_since:.0f} s'
        w = math.hypot(*self.wind)
        wind = ('calm' if w < 2 else 'moderate' if w <= 6 else 'strong') + \
            f' {w:.0f} m/s from {compass(-self.wind[0], -self.wind[1])}'
        return {'mission': mission, 'task': task, 'energy': energy, 'home': home, 'link': link,
                'hazards': '; '.join(haz) or 'none', 'detection': detection,
                'other_detections': others, 'health': health, 'coordinator': coord,
                'wind': wind}

    def describe_order(self) -> str:
        o = self.order or {}
        k = o.get('kind', '')
        if k == 'survey':
            return f'order: survey sector {o.get("sector", "?")}'
        if k == 'inspect':
            return f'order: inspect track {o.get("track", "?")}'
        if k in ('goto', 'formation', 'relay'):
            x, y = o.get('point', [0, 0])[:2]
            return f'order: {k} to ({x:.0f}, {y:.0f})'
        return f'order: {k}'

    def live_order(self) -> bool:
        return bool(self.order) and not self.order['done']

    def mask(self, qset: str) -> dict[str, set[str]]:
        s = self.status
        if s is None or not s.airborne:
            return {'next_action': set(W1) - {'continue'}}       # on the ground: wait for orders
        out = set()
        if s.margin <= 0:                                        # H6 forced RTB
            out |= set(W1) - {'return_to_base', 'land_now'}
        if not self.unresolved() or s.margin < 0.1 or \
                self.mission['phase'] not in ('SEARCH', 'CONFIRM'):
            out.add('inspect')                                   # detours only while searching
        if s.margin < 0.05:
            out.add('hold')
        if self.support_since is not None and self.now() - self.support_since < SUPPORT_WAIT_S:
            out.add('request_support')
        if not s.critical or s.over_obstacle:
            out.add('land_now')
        if s.link_lost_s >= self.p['lost_link_hold_s']:          # H7
            out |= set(W1) - {'hold', 'return_to_base', 'land_now'}
        if not self.live_order() and self.inspecting is None:
            out.add('continue')                                  # no task to continue
        if out >= set(W1):
            out.discard('return_to_base')
        return {'next_action': out}

    def require(self, qset: str) -> dict[str, str]:
        """H8: a critical failure that cannot make it home lands in place."""
        s = self.status
        if s is None or not s.airborne:
            return {'next_action': 'continue'}          # on the pad: FORCED, no LLM spend
        if s.critical and s.energy_frac < s.energy_needed_frac:
            return {'next_action': 'land_now'}
        return {}

    def on_decision(self, d: Decision) -> None:
        a = d.selected.get('next_action')
        if d.act and a:
            self.execute(a, d)
        if d.final:
            self.after_final(d)

    # --- acting ----------------------------------------------------------------------------------

    def execute(self, a: str, d: Decision) -> None:
        if a == 'continue':
            if self.inspecting and self.active and self.active[0] == 'ORBIT':
                return                                           # finish the inspection
            self.inspecting = None
            self.do_order()
        elif a == 'inspect':
            t = self.best_track()
            if t is None:
                return
            if self.inspecting != t['id']:
                self.committed = False
            self.inspecting = t['id']
            pr = d.llm_params
            self.send('ORBIT', f'inspect {t["id"]} ({t["x"]:.0f}, {t["y"]:.0f})',
                      target=(t['x'], t['y']), altitude=pr.get('inspect_altitude_m', 10.0),
                      radius=pr.get('orbit_radius_m', 6.0),
                      laps=int(round(pr.get('orbit_laps', 1))))
        elif a == 'hold':
            self.send('HOLD', 'waiting', duration=float(d.llm_params.get('hold_s', 0.0)))
        elif a == 'return_to_base':
            self.send('RTB', 'home')
        elif a == 'request_support':
            if self.support_since is None or self.now() - self.support_since >= SUPPORT_WAIT_S:
                self.support_since = self.now()
                self.report('request_support', d, priority=0.9)
        elif a == 'land_now':
            self.send('LAND', 'emergency')
        self.log_change(a, d)

    def log_change(self, a: str, d: Decision) -> None:
        if self.last_logged != a:
            self.last_logged = a
            why = f' ({d.rationale[:80]})' if d.rationale else ''
            self.get_logger().info(f'{d.phase}: {a}{why}')

    def do_order(self) -> None:
        o = self.order
        if not o or o['done']:
            return
        k = o['kind']
        if self.status and not self.status.airborne and k != 'takeoff':
            return                                      # landed: only a takeoff order relaunches
        if k == 'takeoff':
            self.send('TAKEOFF', 'launch',
                      altitude=float(o.get('altitude', self.p['alt_transit'])))
        elif k in ('goto', 'formation', 'relay'):
            x, y, z = (list(o.get('point', [0, 0, 0])) + [0])[:3]
            # the point is in the detail: the coordinator's ack must tell two slots apart
            self.send('FORMATION_SLOT' if k == 'formation' else 'GOTO', f'{k} ({x:.0f}, {y:.0f})',
                      target=(x, y),
                      altitude=float(z or self.p['alt_transit']),
                      speed=float(o.get('speed', 0.0)))
        elif k == 'survey':
            (x0, y0), (x1, y1) = o['area']
            self.send('SURVEY', f'sector {o.get("sector", "?")}', area=[(x0, y0), (x1, y1)],
                      altitude=float(o.get('altitude', self.p['alt_survey'])))
        elif k == 'inspect':
            x, y = o['point'][:2]
            self.send('ORBIT', f'inspect {o.get("track", "?")} ({x:.0f}, {y:.0f})', target=(x, y),
                      altitude=self.p['alt_inspect'], radius=6.0, laps=1)
        elif k == 'rtb':
            self.send('RTB', 'recall')
        elif k == 'hold':
            self.send('HOLD', 'ordered')
        elif k == 'land':
            self.send('LAND', 'ordered')

    def send(self, prim: str, detail: str = '', target=(0.0, 0.0), area=(), altitude=0.0,
             speed=0.0, radius=0.0, laps=0, duration=0.0) -> None:
        key = (prim, detail)
        s = self.status
        if self.active == key and s and (s.task == prim or prim == 'RTB' and s.task == 'LAND'):
            return                                               # already doing it
        if not self.task.server_is_ready():
            self.get_logger().warn('flight_controller task server not ready',
                                   throttle_duration_sec=5.0)
            return
        g = Task.Goal(primitive=prim, detail=detail, altitude=float(altitude), speed=float(speed),
                      radius=float(radius), laps=int(laps), duration=float(duration))
        g.target = Point(x=float(target[0]), y=float(target[1]))
        g.area = [Point(x=float(x), y=float(y)) for x, y in area]
        self.active = key
        self.goal_seq += 1
        seq = self.goal_seq
        self.task.send_goal_async(g).add_done_callback(lambda f: self._accepted(f, seq, key))

    def _accepted(self, fut, seq: int, key) -> None:
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn(f'task {key[0]} rejected (safety layer)')
            if seq == self.goal_seq:
                self.active = None
            return
        gh.get_result_async().add_done_callback(lambda f: self._done(f, seq, key))

    def _done(self, fut, seq: int, key) -> None:
        res = fut.result().result
        if seq != self.goal_seq:
            return                                               # superseded by a newer goal
        self.active = None
        prim, detail = key
        if not res.success:
            if 'preempted' not in res.message:
                self.get_logger().warn(f'task {prim} ended: {res.message}')
                self.engine.trigger('task_failed', priority='high')
            return
        if prim == 'ORBIT':
            # every local track the orbit looked at is inspected; report the best-seen one now
            # (the next encode describes an unresolved track, not this one)
            cx, cy = (float(v) for v in detail[detail.rindex('(') + 1:-1].split(','))
            seen = [t for t in self.tracks.values() if t['id'] == self.inspecting or
                    math.hypot(t['x'] - cx, t['y'] - cy) < 10]
            for t in seen:
                t['inspected'] = True
            if seen:
                best = max(seen, key=lambda t: sum(a <= COMMIT_ALT for a, _ in t['sightings']))
                self.report('candidate_found', None, 0.8, track=best)
            self.inspecting = None
            self.committed = False
        if self.order and not self.order['done'] and detail.startswith(
                ('launch', 'goto', 'formation', 'relay', 'sector', 'inspect', 'recall',
                 'ordered')) and (prim != 'ORBIT' or self.order['kind'] == 'inspect'):
            self.order['done'] = True
        self.engine.completed('next_action')

    def watch(self) -> None:
        """Commit point of a W1 inspection (descent below 15 m)."""
        s = self.status
        if self.inspecting and not self.committed and s and s.task == 'ORBIT' and \
                s.position.z < COMMIT_ALT and self.engine.current('step').get(
                    'next_action') == 'inspect':
            self.engine.commit_point('next_action')
            self.committed = True

    # --- reporting -------------------------------------------------------------------------------

    def after_final(self, d: Decision) -> None:
        if self.status is None or not self.status.airborne:
            return                                               # reports are about flight
        if d.selected.get('unusual') == 'true' and self.now() - self.last_unusual > 30:
            self.last_unusual = self.now()
            self.engine.trigger('unusual', priority='high')
        sit = d.selected.get('situation', 'nominal')
        t = self.tracks.get(self.encoded)
        fresh = t is not None and len(t['sightings']) > t['reported'] and (
            t['reported'] == 0 or t['inspected'] or len(t['sightings']) >= 2 * t['reported'])
        if d.selected.get('report_now') == 'true' or sit != self.last_situation or fresh:
            self.report(sit, d, priority=float(d.values.get('detection_priority', 0.0)) / 4
                        if t else 0.3)
        self.last_situation = sit

    def report(self, kind: str, d: Decision | None, priority: float,
               track: dict | None = None) -> None:
        """A Report up (via comms_sim). Beliefs (W2/W3) only when d decided about the track."""
        now = self.now()
        if now - self.last_report < REPORT_GAP_S and kind != 'request_support' and not track:
            return
        self.last_report = now
        s = self.status
        t = track or self.tracks.get(self.encoded)
        payload = {'energy': round(s.energy_frac, 3) if s else None,
                   'margin': round(s.margin, 3) if s else None,
                   'link': round(s.link_q, 2) if s else None,
                   'task': s.task if s else '', 'detail': s.task_detail if s else '',
                   'progress': round(s.progress, 2) if s else 0.0,
                   'pos': [round(s.position.x, 1), round(s.position.y, 1),
                           round(s.position.z, 1)] if s else None,
                   'faults': list(s.faults) if s else []}
        beliefs = {}
        if t:
            best, sc = self.evidence(t)
            payload['track'] = {'id': t['id'], 'x': round(t['x'], 1), 'y': round(t['y'], 1),
                                'sightings': len(t['sightings']), 'best_alt': round(best, 1),
                                'scores': {k: round(v, 3) for k, v in sc.items()},
                                'inspected': t['inspected']}
            if d is not None and t['id'] == self.encoded:
                beliefs = {q: d.fused[q] for q in ('detection_class', 'detection_priority')
                           if q in d.fused}
            t['reported'] = len(t['sightings'])
        self.hierarchy.report(kind, subject=t['id'] if t else '', priority=priority,
                              beliefs=beliefs, payload=payload)


def main(args=None):
    run(DroneAgent, args, threads=1)    # callbacks never race over the local tracks
