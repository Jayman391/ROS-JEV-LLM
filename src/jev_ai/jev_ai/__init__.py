"""jev_ai: Jev (System 1) + LLM (System 2) decisions for ROS 2.

`jev_ai.core` is pure Python (no rclpy) and holds all decision maths. `jev_ai.ros` attaches it to
rclpy nodes; `from jev_ai import DecisionEngine` imports it lazily so the core stays ROS-free.
"""


def __getattr__(name):
    if name in ('DecisionEngine', 'Decision', 'Hierarchy'):
        from jev_ai import ros
        return getattr(ros, name)
    raise AttributeError(name)
