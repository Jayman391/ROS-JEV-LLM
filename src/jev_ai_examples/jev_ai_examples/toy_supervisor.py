"""toy_patrol supervisor (A7): reports up, a decision, directives down.

The supervisor is itself a jev_ai app: its DecisionEngine (catalog ``toy_supervisor.yaml``)
decides the fleet's standing ``order`` (carry_on | converge) whenever a robot reports a new
detection. A *final* ``converge`` (COMMITTED/KEPT, never a provisional that augmentation might
revise) becomes a ``converge`` directive to every robot, carrying the most urgent reported target.

    ros2 launch jev_ai_examples toy_patrol.launch.py supervisor:=true
"""
from __future__ import annotations

from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from jev_ai import Decision, DecisionEngine, Hierarchy
from jev_ai.ros.engine import run

FRESH_S = 30.0       # reports older than this are ignored
TTL_S = 15.0         # how long a converge order stands


class ToySupervisor(Node):
    def __init__(self):
        super().__init__('supervisor')
        default = get_package_share_directory('jev_ai_examples') + '/config/toy_supervisor.yaml'
        catalog = self.declare_parameter('catalog', default).value
        self.reports: dict[str, tuple[float, dict, float]] = {}   # agent -> (t, payload, prio)
        self.active: tuple[str, float] | None = None               # (target, until)
        self.engine = DecisionEngine(self, catalog, encode_state=self.encode,
                                     run_id=self.declare_parameter('run_id', '').value or None,
                                     on_decision=self.act, all_phases=True)
        self.hierarchy = Hierarchy(self, on_report=self.on_report)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_report(self, msg, payload: dict) -> None:
        prev = self.reports.get(msg.agent)
        self.reports[msg.agent] = (self.now(), payload, msg.priority)
        det = payload.get('detection', 'none')
        if det != 'none' and (prev is None or prev[1].get('detection') != det):
            self.engine.trigger('report', subject=det)

    def _targets(self) -> list[tuple[float, str, str]]:
        """(urgency, agent, detection) of fresh reports with a detection, most urgent first."""
        now = self.now()
        return sorted(((pr, a, p['detection']) for a, (t, p, pr) in self.reports.items()
                       if now - t < FRESH_S and p.get('detection', 'none') != 'none'),
                      reverse=True)

    def encode(self, qset: str) -> dict[str, str]:
        now = self.now()
        fresh = [p for t, p, _ in self.reports.values() if now - t < FRESH_S]
        energies = [p['energy'] for p in fresh if isinstance(p.get('energy'), (int, float))]
        reports = '; '.join(f'{a.lstrip("/")}: {d} (urgency {pr:.2f})'
                            for pr, a, d in self._targets()[:3]) or 'none'
        fleet = f'{len(fresh)} robots reporting, min energy {min(energies)}%' if energies \
            else 'no recent reports'
        order = f'converge on {self.active[0]} ({self.active[1] - now:.0f} s left)' \
            if self.active and now < self.active[1] else 'none'
        return {'reports': reports, 'fleet': fleet, 'order': order}

    def act(self, d: Decision) -> None:
        if not d.final or d.selected.get('order') != 'converge':
            return
        targets = self._targets()
        if not targets:
            return
        _, reporter, target = targets[0]
        now = self.now()
        if self.active and self.active[0] == target and now < self.active[1]:
            return                                      # already ordered
        did = self.hierarchy.direct('converge', ttl_s=TTL_S, payload={
            'target': target, 'subject': target, 'reporter': reporter, 'ttl_s': TTL_S})
        self.active = (target, now + TTL_S)
        self.get_logger().info(f'directive #{did}: converge on {target} '
                               f'(reported by {reporter}; {d.phase})')


def main(args=None):
    run(ToySupervisor, args, threads=4)
