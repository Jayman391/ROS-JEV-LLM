"""turtlesim example (A9): turtles that patrol, avoid walls and answer alerts with jev_ai.

    ros2 launch jev_ai_examples turtlesim.launch.py         # watch at http://localhost:6080/vnc.html
    ros2 topic pub --once /alerts geometry_msgs/msg/Point "{x: 8.0, y: 3.0}"   # inject an alert

``turtle_agent`` is the Python variant: a DecisionEngine in-process. The question is the movement
(forward | turn_left | turn_right | go_to_alert). Walls are avoided by a **mask**, not by trusting
either model: ``forward`` is forbidden while a wall is under 1.2 m ahead (§11: masks are
absolute). A turn *completes* after ~90° and reaching the alert completes ``go_to_alert``; both are
decision events (``engine.completed``). The C++ and BT variants (jev_ai_bt ``turtle_agent``) do
the same through decide_server.

``turtle_arena`` spawns the turtles, colours their pens, and (``auto_alerts``) injects an alert at
a random spot every ``period_s`` while none is active. ``/alerts`` carries geometry_msgs/Point:
z = 0 a new alert at (x, y), z = 1 cleared (the turtle that reaches it publishes this).
"""
from __future__ import annotations

import math
import random

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, Spawn

from jev_ai import Decision, DecisionEngine
from jev_ai.ros.engine import run

ARENA = 11.088889          # turtlesim's arena is [0, ARENA]^2
WALL_MASK_M = 1.2          # forward is masked when a wall is closer than this ahead
REACHED_M = 0.5
TURN_S = 1.0               # ~90° at TURN_RATE
TURN_RATE = 1.6


def wall_distance(x: float, y: float, th: float) -> float:
    """Distance along the heading to the arena boundary."""
    c, s = math.cos(th), math.sin(th)
    hits = [t for t in ((ARENA - x) / c if c > 1e-6 else None, -x / c if c < -1e-6 else None,
                        (ARENA - y) / s if s > 1e-6 else None, -y / s if s < -1e-6 else None)
            if t is not None]
    return max(0.0, min(hits)) if hits else ARENA


def relative(x: float, y: float, th: float, tx: float, ty: float) -> tuple[float, float]:
    """(distance, bearing in rad, + = left) from a pose to a point."""
    b = math.atan2(ty - y, tx - x) - th
    return math.hypot(tx - x, ty - y), math.atan2(math.sin(b), math.cos(b))


def side(b: float) -> str:
    return f'{abs(math.degrees(b)):.0f}° ' + ('left' if b > 0 else 'right')


class TurtleAgent(Node):
    def __init__(self):
        super().__init__('turtle_agent')
        self.name = self.declare_parameter('turtle', 'turtle1').value
        others = [t for t in self.declare_parameter('turtles', ['turtle1']).value
                  if t != self.name]
        catalog = self.declare_parameter('catalog', get_package_share_directory(
            'jev_ai_examples') + '/config/turtle_patrol.yaml').value
        run_id = self.declare_parameter('run_id', '').value or None
        self.pose: Pose | None = None
        self.poses: dict[str, Pose] = {}
        self.alert: tuple[float, float] | None = None
        self.action, self.since = None, 0.0
        self.cmd = self.create_publisher(Twist, f'/{self.name}/cmd_vel', 10)
        self.alerts = self.create_publisher(Point, '/alerts', 10)
        self.create_subscription(Pose, f'/{self.name}/pose', self._pose, 10)
        for t in others:
            self.create_subscription(Pose, f'/{t}/pose',
                                     lambda m, t=t: self.poses.__setitem__(t, m), 10)
        self.create_subscription(Point, '/alerts', self._alert, 10)
        self.engine = DecisionEngine(self, catalog, run_id=run_id, agent=f'/{self.name}',
                                     encode_state=self.encode, mask=self.mask,
                                     on_decision=self.act)
        self.create_timer(0.1, self.drive)

    # --- jev_ai callbacks --------------------------------------------------------------------------

    def encode(self, qset: str) -> dict[str, str]:
        p = self.pose or Pose(x=ARENA / 2, y=ARENA / 2)
        wall = wall_distance(p.x, p.y, p.theta)
        alert = 'none'
        if self.alert:
            d, b = relative(p.x, p.y, p.theta, *self.alert)
            nearer = [t for t, q in self.poses.items()
                      if math.hypot(q.x - self.alert[0], q.y - self.alert[1]) < d]
            alert = (f'at ({self.alert[0]:.1f}, {self.alert[1]:.1f}): {d:.1f} m, {side(b)}; ' +
                     (f'{nearer[0]} is nearer' if nearer else 'you are nearest'))
        neighbour = 'none'
        if self.poses:
            t, q = min(self.poses.items(), key=lambda kv: math.hypot(kv[1].x - p.x,
                                                                     kv[1].y - p.y))
            d, b = relative(p.x, p.y, p.theta, q.x, q.y)
            neighbour = f'{t} {d:.1f} m, {side(b)}'
        return {'wall_ahead': f'{"close" if wall < 2 * WALL_MASK_M else "clear"} ({wall:.1f} m)',
                'position': f'({p.x:.1f}, {p.y:.1f})', 'alert': alert, 'neighbour': neighbour}

    def mask(self, qset: str) -> dict[str, set[str]]:
        p = self.pose
        out = set()
        if p and wall_distance(p.x, p.y, p.theta) < WALL_MASK_M:
            out.add('forward')
        if self.alert is None:
            out.add('go_to_alert')
        return {'action': out}

    def act(self, d: Decision) -> None:
        a = d.selected.get('action')
        if a != self.action:
            self.get_logger().info(f'{d.phase}: {a}')
        self.action, self.since = a, self.now()

    # --- the turtle --------------------------------------------------------------------------------

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _pose(self, msg: Pose) -> None:
        self.pose = msg

    def _alert(self, msg: Point) -> None:
        if msg.z == 0.0:
            self.alert = (msg.x, msg.y)
            self.engine.trigger('alert', priority='high')
        elif self.alert and math.hypot(msg.x - self.alert[0], msg.y - self.alert[1]) < 1e-3:
            self.alert = None
            if self.action == 'go_to_alert':
                self.engine.completed('action')

    def drive(self) -> None:
        p, a, tw = self.pose, self.action, Twist()
        if p is None or a is None:
            return self.cmd.publish(tw)
        elapsed = self.now() - self.since
        if a == 'forward':
            # app-level hard stop: the mask only applies at decision events (2 Hz)
            tw.linear.x = 1.5 if wall_distance(p.x, p.y, p.theta) > 0.4 else 0.0
        elif a in ('turn_left', 'turn_right'):
            tw.angular.z = TURN_RATE if a == 'turn_left' else -TURN_RATE
            if elapsed > TURN_S:
                self.action = None
                self.engine.completed('action')
        elif a == 'go_to_alert' and self.alert:
            d, b = relative(p.x, p.y, p.theta, *self.alert)
            tw.angular.z = 4.0 * b
            tw.linear.x = min(1.5, d) if abs(b) < 0.5 else 0.0
            if d < REACHED_M:
                self.alerts.publish(Point(x=self.alert[0], y=self.alert[1], z=1.0))
        self.cmd.publish(tw)


class TurtleArena(Node):
    """Spawns the turtles and injects alerts (the 'world' of the example)."""

    COLOURS = [(255, 80, 80), (80, 200, 80), (80, 120, 255), (230, 180, 40), (200, 80, 200)]

    def __init__(self):
        super().__init__('turtle_arena')
        turtles = self.declare_parameter('turtles', ['turtle1']).value
        self.period = self.declare_parameter('period_s', 15.0).value
        self.auto = self.declare_parameter('auto_alerts', True).value
        self.rng = random.Random(self.declare_parameter('seed', 0).value)
        self.alert, self.since = None, self.now()
        self.pub = self.create_publisher(Point, '/alerts', 10)
        self.create_subscription(Point, '/alerts', self._alert, 10)
        spawn = self.create_client(Spawn, '/spawn')
        spawn.wait_for_service(timeout_sec=10.0)
        for i, t in enumerate(turtles):
            if t != 'turtle1':
                a = 2 * math.pi * i / len(turtles)
                spawn.call_async(Spawn.Request(x=ARENA / 2 + 3 * math.cos(a),
                                               y=ARENA / 2 + 3 * math.sin(a), theta=a, name=t))
            pen = self.create_client(SetPen, f'/{t}/set_pen')
            if pen.wait_for_service(timeout_sec=5.0):
                r, g, b = self.COLOURS[i % len(self.COLOURS)]
                pen.call_async(SetPen.Request(r=r, g=g, b=b, width=3))
        self.create_timer(1.0, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _alert(self, msg: Point) -> None:
        if msg.z == 0.0:
            self.alert, self.since = (msg.x, msg.y), self.now()
        else:
            if self.alert:
                self.get_logger().info(f'alert at ({msg.x:.1f}, {msg.y:.1f}) cleared after '
                                       f'{self.now() - self.since:.1f} s')
            self.alert, self.since = None, self.now()

    def tick(self) -> None:
        if self.alert and self.now() - self.since > 3 * self.period:      # nobody came: expire
            self.pub.publish(Point(x=self.alert[0], y=self.alert[1], z=1.0))
        elif self.auto and self.alert is None and self.now() - self.since > self.period:
            x, y = self.rng.uniform(2, ARENA - 2), self.rng.uniform(2, ARENA - 2)
            self.get_logger().info(f'alert at ({x:.1f}, {y:.1f})')
            self.pub.publish(Point(x=x, y=y, z=0.0))


def main(args=None):
    run(TurtleAgent, args, threads=4)


def arena_main(args=None):
    run(TurtleArena, args, threads=2)
