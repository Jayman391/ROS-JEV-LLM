"""decision_logger (§12): DecisionRecords and Labels → JSONL (one record per line, NaN/inf → null).

ponytail: JSONL only. For rosbag2, record the topics directly: ``ros2 bag record /decisions
/labels``. Prompts are not logged (they are rebuilt from records at DEBUG level only).
"""
from __future__ import annotations

import os
import time
from pathlib import Path

from jev_ai_msgs.msg import DecisionRecord, Label
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py import message_to_ordereddict

from jev_ai.core.records import dumps
from jev_ai.ros.engine import run


class DecisionLogger(Node):
    def __init__(self):
        super().__init__('decision_logger')
        default_dir = Path(os.environ.get('ROS_HOME', Path.home() / '.ros')) / 'jev_ai'
        stamp = time.strftime('%Y%m%d-%H%M%S')
        path = self.declare_parameter('path', '').value or \
            str(default_dir / f'decisions-{stamp}.jsonl')
        labels = self.declare_parameter('labels_path', '').value or \
            str(Path(path).with_name(Path(path).stem + '-labels.jsonl'))
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.files = {'decisions': open(path, 'a', buffering=1),       # line-buffered
                      'labels': open(labels, 'a', buffering=1)}
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(DecisionRecord, 'decisions', lambda m: self._write('decisions', m),
                                 qos)
        self.create_subscription(Label, 'labels', lambda m: self._write('labels', m), qos)
        self.count = 0
        self.get_logger().info(f'logging decisions to {path}')

    def _write(self, kind: str, msg) -> None:
        self.files[kind].write(dumps(dict(message_to_ordereddict(msg))) + '\n')
        self.count += 1

    def destroy_node(self):
        for f in self.files.values():
            f.close()
        super().destroy_node()


def main(args=None):
    run(DecisionLogger, args, threads=1)
