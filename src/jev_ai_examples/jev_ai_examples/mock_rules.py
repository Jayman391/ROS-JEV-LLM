"""Rule hook for the mock Jev backend (``mock.rules: jev_ai_examples.mock_rules:patrol``),
for the toy_patrol, supervisor and turtlesim catalogs.

Makes the keyless mock answer the toy_patrol questions plausibly from the state, so the demo
behaves like a patrol robot rather than noise. Returning None falls back to the seeded Dirichlet.
"""
from __future__ import annotations


# Each rule below lists probabilities in this class order; patrol() keys them by class name, so
# the answer survives a reordered catalog (ros2 jev replay measures position bias that way).
CLASSES = {'next_action': ['continue', 'inspect', 'halt_now'], 'urgency': ['none', 'low', 'high'],
           'report_now': ['true', 'false'],
           'action': ['forward', 'turn_left', 'turn_right', 'go_to_alert'],
           'order': ['carry_on', 'converge']}


def patrol(state: dict, question) -> dict[str, float] | None:
    p = _rules(state, question)
    return None if p is None else dict(zip(CLASSES[question.name], p))


def _rules(state: dict, question) -> list[float] | None:
    det = state.get('detection', 'none') != 'none'
    haz = state.get('hazards', 'none') != 'none'
    energy = state.get('energy', '')
    if question.name == 'next_action':          # continue, inspect, halt_now
        if energy.startswith('critical'):
            return [0.15, 0.05, 0.80]
        if det and state.get('orders', 'none').startswith('converge'):
            return [0.25, 0.70, 0.05]
        if det:
            return [0.35, 0.60, 0.05] if 'person' in state['detection'] else [0.55, 0.42, 0.03]
        return [0.90, 0.07, 0.03] if not haz else [0.70, 0.20, 0.10]
    if question.name == 'urgency':              # none, low, high
        return [0.10, 0.40, 0.50] if det else [0.30, 0.50, 0.20] if haz else [0.80, 0.15, 0.05]
    if question.name == 'report_now':           # true, false
        return [0.70, 0.30] if det or haz else [0.15, 0.85]
    if question.name == 'action':               # turtles: forward, turn_left, turn_right, go_to_alert
        return turtle(state)
    if question.name == 'order':                # supervisor: carry_on, converge
        reports = state.get('reports', 'none')
        if state.get('order', 'none') != 'none':
            return [0.30, 0.70]                 # an order is running: keep it
        return [0.20, 0.80] if "person" in reports else [0.85, 0.15]
    return None


def turtle(state: dict) -> list[float]:
    """turtlesim (A9): forward when clear, turn near walls, the nearest turtle takes the alert."""
    alert = state.get('alert', 'none')
    if alert != 'none' and 'you are nearest' in alert:
        return [0.10, 0.05, 0.05, 0.80]
    if state.get('wall_ahead', '').startswith('close'):
        return [0.04, 0.48, 0.44, 0.04]
    if alert != 'none':
        return [0.70, 0.10, 0.10, 0.10]
    return [0.80, 0.09, 0.09, 0.02]
