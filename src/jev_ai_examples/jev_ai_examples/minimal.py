"""minimal: the smallest jev_ai agent (docs/GUIDE.md). A robot patrols while its battery drains;
every second Jev (System 1) proposes continue / return_home and the LLM (System 2) may revise.

    ros2 launch jev_ai_bringup bringup.launch.py        # servers + dashboard (keyless mocks)
    ros2 run jev_ai_examples minimal                     # this agent; watch http://localhost:8080
"""
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from jev_ai import Decision, DecisionEngine
from jev_ai.ros.engine import run

CATALOG = get_package_share_directory('jev_ai_bringup') + '/config/catalog_template.yaml'


class Minimal(Node):
    def __init__(self):
        super().__init__('minimal')
        self.battery = 100.0
        self.engine = DecisionEngine(self, CATALOG, encode_state=self.encode, mask=self.mask,
                                     on_decision=self.act)
        self.create_timer(1.0, self.drain)

    def drain(self):
        self.battery = max(0.0, self.battery - 2.0)

    def encode(self, qset: str) -> dict[str, str]:
        """The catalog's state_keys as short strings: words for meaning, numbers for detail."""
        level = 'ample' if self.battery > 50 else 'low' if self.battery > 20 else 'critical'
        return {'task': 'patrolling the yard', 'battery': f'{level} ({self.battery:.0f}% left)'}

    def mask(self, qset: str) -> dict[str, set[str]]:
        """Hard rules: classes Jev and the LLM may not pick right now."""
        return {'next_action': {'continue'}} if self.battery <= 20 else {}

    def act(self, d: Decision) -> None:
        """Called when the app must act: PROVISIONAL (Jev, at once), then REVISED if the LLM
        changed the pick."""
        self.get_logger().info(f'{d.phase}: {d.selected["next_action"]} (battery '
                               f'{self.battery:.0f}%)')
        if d.selected['next_action'] == 'return_home':
            self.battery = 100.0                     # pretend it docked and recharged


def main(args=None):
    run(Minimal, args)
