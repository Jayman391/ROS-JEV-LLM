"""Rule hook for the mock Jev backend (``mock.rules: jev_ai_examples.mock_rules:patrol``).

Makes the keyless mock answer the toy_patrol questions plausibly from the state, so the demo
behaves like a patrol robot rather than noise. Returning None falls back to the seeded Dirichlet.
"""
from __future__ import annotations


def patrol(state: dict, question) -> list[float] | None:
    det = state.get('detection', 'none') != 'none'
    haz = state.get('hazards', 'none') != 'none'
    energy = state.get('energy', '')
    if question.name == 'next_action':          # continue, inspect, halt_now
        if energy.startswith('critical'):
            return [0.15, 0.05, 0.80]
        if det:
            return [0.35, 0.60, 0.05] if 'person' in state['detection'] else [0.55, 0.42, 0.03]
        return [0.90, 0.07, 0.03] if not haz else [0.70, 0.20, 0.10]
    if question.name == 'urgency':              # none, low, high
        return [0.10, 0.40, 0.50] if det else [0.30, 0.50, 0.20] if haz else [0.80, 0.15, 0.05]
    if question.name == 'report_now':           # true, false
        return [0.70, 0.30] if det or haz else [0.15, 0.85]
    return None
