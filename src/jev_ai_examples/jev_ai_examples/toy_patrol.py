"""toy_patrol: a simulated patrol robot that decides with jev_ai (no simulator needed).

The world is a seeded random walk: battery drains (faster while inspecting), detections and
smoke come and go, the link flickers. The app supplies what §5.1 asks of any app:
- ``encode`` turns numbers into short interpreted strings (Jev is weak at arithmetic);
- ``mask``: ``inspect`` is forbidden when there is nothing to inspect;
- ``require``: ``halt_now`` is forced deterministically at ≤ 5 % battery (real emergencies
  never wait on an API, §6.4), while at < 15 % Jev itself starts proposing ``halt_now`` —
  the irreversible path that needs LLM agreement and a margin;
- ``on_decision`` executes the selection; inspections pass a commit point after 1 s and
  complete after 4 s.
A halted robot gets an "operator battery swap" after 8 s so the demo keeps cycling. The world
itself is ``toy_world.ToyWorld``; its oracle is published as Labels on every final decision, for
``ros2 jev calibrate`` and ``ros2 jev ope`` (A10).

Hierarchy (A7): when its own ``report_now`` decision says true, the robot reports upward; a
``converge`` directive from the supervisor shares the target, enters the state as ``orders`` and
is a decision event in the directive's epoch, so robots answering it share their noise.
"""
from __future__ import annotations

import random

from ament_index_python.packages import get_package_share_directory
from jev_ai_msgs.msg import Label
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from jev_ai import Decision, DecisionEngine, Hierarchy
from jev_ai.ros.engine import run
from jev_ai_examples.toy_world import STEP_S, ToyWorld


class ToyPatrol(Node):
    def __init__(self):
        super().__init__('toy_patrol')
        default_catalog = get_package_share_directory('jev_ai_examples') + '/config/toy_patrol.yaml'
        catalog = self.declare_parameter('catalog', default_catalog).value
        seed = self.declare_parameter('seed', 0).value
        self.world = ToyWorld(random.Random(f'{self.get_name()}:{seed}'))
        # one run_id for the whole fleet: group noise is H(run_id, group, ...), so coupling (§6.3)
        # only happens between agents that share it ('' = this agent alone)
        run_id = self.declare_parameter('run_id', '').value or None
        self.engine = DecisionEngine(self, catalog, run_id=run_id, encode_state=self.encode,
                                     mask=lambda qs: self.world.mask(),
                                     require=lambda qs: self.world.require(),
                                     on_decision=self.act,
                                     all_phases=True,      # KEPT too: reports wait for final
                                     on_degraded=lambda qs: self.get_logger().warn(
                                         f'{qs}: Jev degraded, holding fallback'))
        supervisor = self.declare_parameter('supervisor', '/supervisor').value
        self.hierarchy = Hierarchy(self, engine=self.engine, qset='next_step',
                                   on_directive=self.obey, issuer_allowlist=[supervisor])
        self.labels = self.create_publisher(
            Label, 'labels', QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.create_timer(STEP_S, self.step)

    # --- the app callbacks (§5.1) ---------------------------------------------------------------

    def encode(self, qset: str) -> dict[str, str]:
        return self.world.encode(self.now())

    def obey(self, msg, payload: dict) -> None:
        """A supervisor directive (before the engine decides on it)."""
        if msg.kind == 'converge':
            self.world.converge(str(payload.get('target', '')), msg.issuer,
                                float(payload.get('ttl_s', 15.0)), self.now())

    def act(self, d: Decision) -> None:
        w = self.world
        w.act(d.selected.get('next_action') if d.act else None, self.now())  # KEPT: executing
        if not d.final:
            return
        for question, true_class in w.oracle().items():
            self.labels.publish(Label(run_id=self.engine.run_id, subject=self.engine.agent,
                                      question=question, true_class=true_class,
                                      decision_id=d.decision_id, source='toy_world'))
        if d.selected.get('report_now') == 'true':
            self.hierarchy.report('status', subject=w.detection,
                                  priority=float(d.values.get('urgency.level_mean', 0.0)),
                                  beliefs=d.fused, payload={
                                      'task': w.task, 'energy': round(w.energy),
                                      'detection': w.detection, 'hazards': w.hazard})
            self.get_logger().info(f'report: {w.task}, {w.detection}, {w.hazard}',
                                   throttle_duration_sec=10.0)

    # --- the simulated world ----------------------------------------------------------------------

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def step(self) -> None:
        for kind, *args in self.world.step(self.now()):
            if kind == 'trigger':
                self.engine.trigger(args[0], priority=args[1])
            else:
                getattr(self.engine, kind)(args[0])     # commit_point / completed


def main(args=None):
    run(ToyPatrol, args, threads=4)
