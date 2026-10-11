"""environment (B2): the episode's hidden ground truth (SYSTEM_MODEL §2).

Publishes ``/env/ground_truth`` (GroundTruth, latched, 1 Hz: targets with animals on a 0.3 m/s
random walk, wind, visibility, faults) and ``/env/wind`` (the wind broadcast every drone may know).
Fault start times are episode offsets from this node's first sim-clock tick.
ponytail: animals move in the ground truth only; their Gazebo boxes stay put (moving a model
needs the set_pose service bridge).
"""
from __future__ import annotations

import math
import random

from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from swarm_interfaces.msg import GroundTruth

from drone_swarm import scenario
from jev_ai.ros.engine import run

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class Environment(Node):
    def __init__(self):
        super().__init__('environment')
        self.p = scenario.from_node(self)
        self.ep = scenario.episode(self.p)
        self.targets = [list(t) for t in self.ep['targets']]
        self.rng = random.Random(f'animals|{self.p["seed"]}')
        self.heading = {t[0]: self.rng.uniform(0, 2 * math.pi) for t in self.targets}
        self.t0: float | None = None
        self.gt_pub = self.create_publisher(GroundTruth, '/env/ground_truth', LATCHED)
        self.wind_pub = self.create_publisher(Vector3, '/env/wind', 10)
        self.create_timer(1.0, self.tick)
        targets = ', '.join(f'{c} {i}' for i, c, _, _ in self.targets)
        self.get_logger().info(
            f'episode seed {self.p["seed"]}: {targets}; wind {math.hypot(*self.ep["wind"]):.1f} '
            f'm/s, visibility {self.ep["visibility"]:.2f}, faults {self.ep["faults"] or "none"}')

    def tick(self) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        x0, x1, y0, y1 = self.p['ao']
        for t in self.targets:
            if t[1] == 'animal':                     # 0.3 m/s random walk, inside the AO
                self.heading[t[0]] += self.rng.gauss(0, 0.5)
                t[2] = min(x1 - 5, max(x0 + 5, t[2] + 0.3 * math.cos(self.heading[t[0]])))
                t[3] = min(y1 - 5, max(y0 + 5, t[3] + 0.3 * math.sin(self.heading[t[0]])))
        m = GroundTruth()
        m.header.stamp = self.get_clock().now().to_msg()
        m.target_ids = [t[0] for t in self.targets]
        m.target_classes = [t[1] for t in self.targets]
        m.target_positions = [Point(x=float(t[2]), y=float(t[3])) for t in self.targets]
        m.wind = Vector3(x=self.ep['wind'][0], y=self.ep['wind'][1])
        m.visibility = self.ep['visibility']
        m.fault_drones = [f[0] for f in self.ep['faults']]
        m.fault_kinds = [f[1] for f in self.ep['faults']]
        m.fault_start_s = [self.t0 + f[2] for f in self.ep['faults']]
        m.blackouts = [Point(x=b[0], y=b[1], z=b[2]) for b in self.ep['blackouts']]
        self.gt_pub.publish(m)
        self.wind_pub.publish(m.wind)


def main(args=None):
    run(Environment, args, threads=1)
