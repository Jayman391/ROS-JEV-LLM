"""mission_scorer (B7): the only judge with the ground truth (SYSTEM_MODEL §6).

Scores the episode live:
J = 100·(w_R·Recall + w_P·Precision + w_C·Coverage + w_T·Time + w_E·Energy)
− Penalties − w_$·LLM $ and publishes it, plus one status per drone, on /diagnostics, so the
jev_ai dashboard shows the swarm next to its decisions. Publishes ground-truth ``Label``s for
``ros2 jev calibrate``: W2 ``detection_class`` per drone-local track and C3 ``track_verdict`` per
coordinator track. At DONE (or T_max + 60 s) it writes the episode summary JSON
(``summary_path``) that ``drone_swarm.episode`` reads.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from jev_ai_msgs.msg import DecisionRecord, Label
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from swarm_interfaces.msg import DetectionArray, DroneStatus, GroundTruth, TrackArray

from drone_swarm import scenario
from drone_swarm.drone_agent import mmss
from jev_ai.ros.engine import run

MATCH_M = 5.0


class Scorer(Node):
    def __init__(self):
        super().__init__('mission_scorer')
        self.p = scenario.from_node(self)
        self.run_id = self.declare_parameter('run_id', '').value
        self.summary_path = self.declare_parameter('summary_path', '').value
        self.drones = scenario.drones(self.p)
        self.cov = scenario.Coverage(self.p)
        self.gt: GroundTruth | None = None
        self.status: dict[str, DroneStatus] = {}
        self.tracks: dict[str, object] = {}
        self.phase, self.t0, self.t_done = 'IDLE', None, None
        self.cost_usd = 0.0
        self.decisions = 0
        self.labelled: set[tuple[str, str]] = set()
        self.pen = {'collision': set(), 'depleted': set(), 'nfz_s': 0.0, 'separation': 0}
        self.close: set[tuple[str, str]] = set()
        self.last_t: float | None = None
        self.written = False
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(GroundTruth, '/env/ground_truth', self._gt, latched)
        self.create_subscription(DroneStatus, '/swarm/status', self._status, 50)
        self.create_subscription(TrackArray, '/coordinator/tracks', self._tracks, 10)
        self.create_subscription(String, '/coordinator/phase', self._phase, 10)
        self.create_subscription(DecisionRecord, '/decisions', self._record, QoSProfile(
            depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        for d in self.drones:
            self.create_subscription(DetectionArray, f'/{d}/detections', self._detections, 10)
        self.labels = self.create_publisher(Label, '/labels', QoSProfile(
            depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- inputs ----------------------------------------------------------------------------------

    def _gt(self, m: GroundTruth) -> None:
        self.gt = m

    def _phase(self, m: String) -> None:
        if self.t0 is None and m.data not in ('IDLE', 'DONE'):
            self.t0 = self.now()
        if m.data == 'DONE' and self.t_done is None and self.t0 is not None:
            self.t_done = self.now() - self.t0
        self.phase = m.data

    def _record(self, m: DecisionRecord) -> None:
        self.cost_usd += m.cost_usd
        self.decisions += 1

    def _tracks(self, m: TrackArray) -> None:
        for t in m.tracks:
            self.tracks[t.id] = t
            if t.status != 'unresolved' and t.status != 'inspecting':
                truth = self.truth(t.position.x, t.position.y)
                verdict = {'survivor': 'confirm_survivor', None: 'dismiss'}.get(
                    truth, 'confirm_non_survivor')
                self.label(t.id, 'track_verdict', verdict)

    def _detections(self, m: DetectionArray) -> None:
        for d in m.detections:
            if (d.track_id, 'detection_class') not in self.labelled:
                self.label(d.track_id, 'detection_class',
                           self.truth(d.position.x, d.position.y, 8.0) or 'nothing')

    def _status(self, m: DroneStatus) -> None:
        self.status[m.drone] = m
        if not m.airborne or self.t0 is None:
            return
        self.cov.see(m.position.x, m.position.y, m.position.z,
                     math.hypot(m.velocity.x, m.velocity.y))

    # --- truth -----------------------------------------------------------------------------------

    def truth(self, x: float, y: float, within: float = MATCH_M) -> str | None:
        """True class of the nearest target within ``within`` m, else None (a false alarm)."""
        if not self.gt:
            return None
        best = min(((math.hypot(p.x - x, p.y - y), c) for p, c in
                    zip(self.gt.target_positions, self.gt.target_classes)), default=None)
        return best[1] if best and best[0] <= within else None

    def label(self, subject: str, question: str, true_class: str) -> None:
        if (subject, question) in self.labelled:
            return
        self.labelled.add((subject, question))
        lab = Label(run_id=self.run_id, subject=subject, question=question,
                    true_class=true_class, decision_id=0, source='mission_scorer')
        lab.header.stamp = self.get_clock().now().to_msg()
        self.labels.publish(lab)

    # --- score -----------------------------------------------------------------------------------

    def penalties(self, dt: float) -> None:
        for d, s in self.status.items():
            if any(math.hypot(s.position.x - x, s.position.y - y) < r and s.position.z < h
                   for x, y, r, h in scenario.obstacles(self.p)):
                self.pen['collision'].add(d)
            if s.airborne and s.energy_frac <= 0.0:
                self.pen['depleted'].add(d)
            if s.airborne and any(math.hypot(s.position.x - x, s.position.y - y) < r
                                  for x, y, r in scenario.nfzs(self.p)):
                self.pen['nfz_s'] += dt
        names = sorted(self.status)
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                sa, sb = self.status[a], self.status[b]
                close = sa.airborne and sb.airborne and math.dist(
                    (sa.position.x, sa.position.y, sa.position.z),
                    (sb.position.x, sb.position.y, sb.position.z)) < self.p['d_sep']
                if close and (a, b) not in self.close:
                    self.pen['separation'] += 1
                (self.close.add if close else self.close.discard)((a, b))

    def score(self) -> dict:
        p = self.p
        surv = [(q.x, q.y) for q, c in zip(self.gt.target_positions, self.gt.target_classes)
                if c == 'survivor'] if self.gt else []
        conf = [t for t in self.tracks.values() if t.status.startswith('confirmed')]
        conf_surv = [t for t in conf if t.status == 'confirmed_survivor']
        found = {i for i, (x, y) in enumerate(surv) for t in conf_surv
                 if math.hypot(t.position.x - x, t.position.y - y) <= MATCH_M}
        correct = sum((self.truth(t.position.x, t.position.y) == 'survivor') ==
                      (t.status == 'confirmed_survivor') and
                      self.truth(t.position.x, t.position.y) is not None for t in conf)
        recall = len(found) / len(surv) if surv else 1.0
        precision = correct / len(conf) if conf else 0.0
        elapsed = self.now() - self.t0 if self.t0 is not None else 0.0
        t_done = self.t_done if self.t_done is not None else max(elapsed, p['t_max_s'])
        used = sum(1.0 - s.energy_frac for s in self.status.values()) * p['E_0']
        n = len(self.drones)
        late = elapsed > p['t_max_s'] and self.t_done is None
        unrecovered = sum(s.airborne for s in self.status.values()) if late else 0
        pen = (100 * len(self.pen['collision']) + 50 * len(self.pen['depleted']) +
               30 * unrecovered + 20 * self.pen['nfz_s'] + 5 * self.pen['separation'])
        J = 100 * (p['w_R'] * recall + p['w_P'] * precision + p['w_C'] * self.cov.fraction() +
                   p['w_T'] * max(0.0, 1 - t_done / p['t_max_s']) +
                   p['w_E'] * max(0.0, 1 - used / (n * p['E_0']))) - pen - \
            p['w_usd'] * self.cost_usd
        return {'J': round(J, 2), 'recall': round(recall, 3), 'precision': round(precision, 3),
                'coverage': round(self.cov.fraction(), 3), 't_done_s': round(t_done, 1),
                'energy_wh': round(used, 2), 'llm_usd': round(self.cost_usd, 4),
                'penalties': round(pen, 1), 'survivors': len(surv),
                'survivors_found': len(found), 'confirmations': len(conf),
                'collisions': len(self.pen['collision']), 'depleted': len(self.pen['depleted']),
                'nfz_s': round(self.pen['nfz_s'], 1),
                'separation_breaches': self.pen['separation'],
                'unrecovered': unrecovered, 'decisions': self.decisions, 'phase': self.phase,
                'elapsed_s': round(elapsed, 1), 'seed': p['seed'], 'num_drones': n,
                'run_id': self.run_id}

    def tick(self) -> None:
        now = self.now()
        dt = 0.0 if self.last_t is None else now - self.last_t
        self.last_t = now
        if self.t0 is not None:
            self.penalties(dt)
        sc = self.score()
        st = [DiagnosticStatus(
            name='mission_scorer', hardware_id='swarm', level=DiagnosticStatus.OK,
            message=f'J = {sc["J"]:.1f} · {sc["survivors_found"]}/{sc["survivors"]} survivors · '
                    f'coverage {sc["coverage"]:.0%} · {sc["phase"]} {mmss(sc["elapsed_s"])}',
            values=[KeyValue(key=k, value=str(v)) for k, v in sc.items()
                    if k not in ('run_id', 'seed', 'num_drones')])]
        for d in self.drones:
            s = self.status.get(d)
            if s is None:
                continue
            lvl = DiagnosticStatus.ERROR if s.critical else DiagnosticStatus.WARN if (
                s.reflexes or s.faults or s.margin < 0.1) else DiagnosticStatus.OK
            st.append(DiagnosticStatus(
                name=d, hardware_id='swarm', level=lvl,
                message=f'{s.task.lower()} {s.task_detail}'.strip() + f' {s.progress:.0%}',
                values=[KeyValue(key=k, value=v) for k, v in (
                    ('position', f'({s.position.x:.0f}, {s.position.y:.0f}, {s.position.z:.0f})'),
                    ('energy', f'{s.energy_frac:.0%} (margin {s.margin:+.2f})'),
                    ('link', f'{s.link_q:.2f}'),
                    ('reflexes', ', '.join(s.reflexes) or '-'),
                    ('faults', ', '.join(s.faults) or '-'))]))
        a = DiagnosticArray(status=st)
        a.header.stamp = self.get_clock().now().to_msg()
        self.diag.publish(a)
        end = self.phase == 'DONE' or (self.t0 is not None and
                                       now - self.t0 > self.p['t_max_s'] + 60)
        if end and not self.written:
            self.written = True
            self.get_logger().info(f'episode: {json.dumps(sc)}')
            if self.summary_path:
                Path(self.summary_path).parent.mkdir(parents=True, exist_ok=True)
                Path(self.summary_path).write_text(json.dumps(sc, indent=1))


def main(args=None):
    run(Scorer, args, threads=1)
