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
A halted robot gets an "operator battery swap" after 8 s so the demo keeps cycling.
"""
from __future__ import annotations

import random

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from jev_ai import Decision, DecisionEngine


class ToyPatrol(Node):
    def __init__(self):
        super().__init__('toy_patrol')
        default_catalog = get_package_share_directory('jev_ai_examples') + '/config/toy_patrol.yaml'
        catalog = self.declare_parameter('catalog', default_catalog).value
        seed = self.declare_parameter('seed', 0).value
        self.rng = random.Random(f'{self.get_name()}:{seed}')
        self.energy = self.rng.uniform(60, 95)
        self.task, self.detection, self.hazard, self.link = 'patrolling', 'none', 'none', 'good'
        self.inspect = None            # (commit_at, done_at) while inspecting
        self.reset_at = None
        self.engine = DecisionEngine(self, catalog, encode_state=self.encode, mask=self.mask,
                                     require=self.require, on_decision=self.act,
                                     on_degraded=lambda qs: self.get_logger().warn(
                                         f'{qs}: Jev degraded, holding fallback'))
        self.create_timer(0.5, self.step)

    # --- the four app callbacks (§5.1) ----------------------------------------------------------

    def encode(self, qset: str) -> dict[str, str]:
        e = round(self.energy)
        level = 'ok' if e >= 40 else 'low' if e >= 15 else 'critical'
        return {'task': self.task, 'energy': f'{level} ({e}%)', 'link': self.link,
                'hazards': self.hazard, 'detection': self.detection}

    def mask(self, qset: str) -> dict[str, set[str]]:
        return {'next_action': {'inspect'}} if self.detection == 'none' else {}

    def require(self, qset: str) -> dict[str, str]:
        return {'next_action': 'halt_now'} if self.energy <= 5 and self.task != 'halted' else {}

    def act(self, d: Decision) -> None:
        now = self.now()
        action = d.selected.get('next_action')
        if action == 'halt_now' and self.task != 'halted':
            self.task, self.inspect, self.reset_at = 'halted', None, now + 8.0
        elif action == 'inspect' and self.inspect is None and self.task != 'halted':
            self.task = f'inspecting {self.detection.split(" at ")[0]}'
            self.inspect = (now + 1.0, now + 4.0)
        elif action == 'continue' and self.task != 'halted':
            self.task, self.inspect = 'patrolling', None
        if d.selected.get('report_now') == 'true' and d.final:
            self.get_logger().info(f'report: {self.task}, {self.detection}, {self.hazard}',
                                   throttle_duration_sec=10.0)

    # --- the simulated world ----------------------------------------------------------------------

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def step(self) -> None:
        now, r = self.now(), self.rng.random
        if self.task == 'halted':
            if now >= self.reset_at:
                self.energy, self.task, self.reset_at = 100.0, 'patrolling', None
                self.engine.trigger('battery_swapped')
            return
        self.energy = max(0.0, self.energy - (0.45 if self.inspect else 0.25))
        if self.inspect:
            commit_at, done_at = self.inspect
            if commit_at and now >= commit_at:
                self.engine.commit_point('next_action')
                self.inspect = (None, done_at)
            if now >= done_at:
                self.inspect, self.detection, self.task = None, 'none', 'patrolling'
                self.engine.completed('next_action')
        if self.detection == 'none' and r() < 0.03:
            what = 'person-like shape' if r() < 0.4 else 'debris'
            self.detection = f'{what} at {self.rng.randint(5, 40)} m'
            self.engine.trigger('detection', priority='high')
        elif self.detection != 'none' and not self.inspect and r() < 0.02:
            self.detection = 'none'
        if self.hazard == 'none' and r() < 0.01:
            self.hazard = 'smoke'
            self.engine.trigger('hazard')
        elif self.hazard != 'none' and r() < 0.05:
            self.hazard = 'none'
        self.link = 'weak' if r() < 0.1 else 'good'


def main(args=None):
    rclpy.init(args=args)
    node = ToyPatrol()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
