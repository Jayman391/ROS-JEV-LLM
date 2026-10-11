"""Rule hook for the keyless mock Jev (``mock.rules: drone_swarm.mock_rules:swarm``).

Answers the drone (W1-W7) and coordinator (C1-C8) questions plausibly from the encoded state, so
the demo flies a sensible mission without API keys. It reads only the text the agents encode
(exactly what live Jev sees). Returning None falls back to the mock's seeded Dirichlet.
"""
from __future__ import annotations

import re

CLASSES = {
    'next_action': ['continue', 'inspect', 'hold', 'return_to_base', 'request_support',
                    'land_now'],
    'detection_class': ['survivor', 'animal', 'vehicle', 'debris', 'nothing'],
    'detection_priority': ['ignore', 'low', 'moderate', 'high', 'urgent'],
    'energy_risk': ['comfortable', 'adequate', 'tight', 'critical'],
    'report_now': ['true', 'false'], 'unusual': ['true', 'false'],
    'situation': ['nominal', 'candidate_found', 'energy_concern', 'hazard', 'link_concern',
                  'fault', 'task_complete'],
    'phase_transition': ['stay', 'advance', 'recover_all', 'abort'],
    'transit_formation': ['column', 'line', 'v', 'diamond', 'circle'],
    'mission_risk': ['low', 'moderate', 'elevated', 'severe'],
    'replan_needed': ['true', 'false'], 'relay_needed': ['true', 'false'],
    'track_verdict': ['await_data', 'confirm_survivor', 'confirm_non_survivor',
                      'request_inspection', 'dismiss'],
    'report_response': ['acknowledge', 'dispatch_support', 'reassign_sector', 'recall_drone',
                        'replan'],
}
FIT = ['poor', 'fair', 'good', 'excellent']


def swarm(state: dict, question) -> dict[str, float] | None:
    name = question.name
    if name.startswith('fit_'):
        p, classes = _fit(state, name), FIT
    else:
        rule = RULES.get(name)
        p, classes = (rule(state) if rule else None), CLASSES.get(name)
    if p is None:
        return None
    p = [x + 0.005 for x in p]            # like real Jev, never exactly 0 (log-linear pooling)
    return {c: x / sum(p) for c, x in zip(classes, p)}


def _scores(text: str) -> dict[str, float]:
    """'scores survivor 0.46, animal 0.31' -> {survivor: 0.46, animal: 0.31}"""
    return {k: float(v) for k, v in re.findall(r'(survivor|animal|vehicle|debris|nothing) '
                                               r'(\d\.\d+)', text)}


def _energy(state) -> str:
    return state.get('energy', '').split(' ')[0]


# --- drone ---------------------------------------------------------------------------------------

def _next_action(s):
    e, det, coord = _energy(s), s.get('detection', 'none'), s.get('coordinator', '')
    if s.get('health', '').startswith('critical'):
        return [0.05, 0.0, 0.05, 0.3, 0.0, 0.6]
    if e in ('critical',) or s.get('link', '').startswith('lost'):
        return [0.15, 0.0, 0.15, 0.65, 0.03, 0.02]
    if 'order: inspect' in coord or 'order: rtb' in coord:
        return [0.85, 0.05, 0.03, 0.04, 0.02, 0.01]          # the order is the task: continue
    if det != 'none' and 'not inspected' in det and e in ('ample', 'adequate'):
        sc = _scores(det)
        surv = sc.get('survivor', 0.0) + 0.5 * sc.get('animal', 0.0)
        if surv > 0.35 and 'another drone' not in det:
            return [0.25, 0.65, 0.03, 0.03, 0.03, 0.01]
        return [0.75, 0.15, 0.03, 0.02, 0.04, 0.01]
    if e == 'tight':
        return [0.55, 0.0, 0.05, 0.35, 0.04, 0.01]
    if 'order done' in s.get('task', '') or 'no task' in s.get('task', ''):
        return [0.1, 0.03, 0.8, 0.04, 0.02, 0.01]           # wait for the next order
    return [0.9, 0.03, 0.03, 0.02, 0.015, 0.005]


def _detection_class(s):
    sc = _scores(s.get('detection', ''))
    if not sc:
        return None
    order = CLASSES['detection_class']
    base = [sc.get(c, 0.02) + 0.02 for c in order]
    z = sum(base)
    return [b / z for b in base]


def _detection_priority(s):
    sc = _scores(s.get('detection', ''))
    surv = sc.get('survivor', 0.0)
    if not sc:
        return [0.8, 0.15, 0.03, 0.01, 0.01]
    if surv > 0.5:
        return [0.02, 0.08, 0.2, 0.4, 0.3]
    if surv > 0.25:
        return [0.05, 0.2, 0.45, 0.2, 0.1]
    return [0.3, 0.4, 0.2, 0.07, 0.03]


def _energy_risk(s):
    return {'ample': [0.8, 0.15, 0.04, 0.01], 'adequate': [0.3, 0.55, 0.12, 0.03],
            'tight': [0.05, 0.25, 0.55, 0.15]}.get(_energy(s), [0.02, 0.08, 0.3, 0.6])


def _report_now(s):
    det = s.get('detection', 'none')
    if 'not reported' in det or s.get('task', '').startswith('inspect') and 'done' in s['task']:
        return [0.85, 0.15]
    if _energy(s) in ('tight', 'critical') or s.get('health', 'nominal') != 'nominal':
        return [0.7, 0.3]
    return [0.1, 0.9]


def _unusual(s):
    odd = (s.get('health', 'nominal') != 'nominal') + (s.get('hazards', 'none') != 'none') + \
        s.get('link', '').startswith(('poor', 'lost'))
    return [0.1 + 0.3 * odd, 0.9 - 0.3 * odd] if odd < 3 else [0.95, 0.05]


def _situation(s):
    if s.get('health', 'nominal') != 'nominal':
        return [0.05, 0.05, 0.1, 0.05, 0.05, 0.65, 0.05]
    if _energy(s) in ('tight', 'critical'):
        return [0.1, 0.05, 0.7, 0.05, 0.05, 0.03, 0.02]
    if s.get('link', '').startswith(('poor', 'lost')):
        return [0.1, 0.05, 0.05, 0.05, 0.7, 0.03, 0.02]
    if s.get('hazards', 'none') != 'none':
        return [0.3, 0.05, 0.05, 0.55, 0.02, 0.02, 0.01]
    if s.get('detection', 'none') != 'none' and 'not reported' in s['detection']:
        return [0.15, 0.75, 0.03, 0.03, 0.02, 0.01, 0.01]
    if 'done' in s.get('task', '') or s.get('task', 'none') == 'none':
        return [0.3, 0.02, 0.02, 0.02, 0.02, 0.02, 0.6]
    return [0.85, 0.05, 0.03, 0.03, 0.02, 0.01, 0.01]


# --- coordinator ---------------------------------------------------------------------------------

def _phase_transition(s):
    m = s.get('mission', '')
    if 'ready to advance' in m:
        return [0.15, 0.82, 0.02, 0.01]
    if 'severe' in s.get('risk_events', ''):
        return [0.4, 0.05, 0.5, 0.05]
    return [0.9, 0.07, 0.025, 0.005]


def _transit_formation(s):
    return [0.15, 0.1, 0.55, 0.15, 0.05]


def _mission_risk(s):
    r = s.get('risk_events', 'none')
    if r == 'none':
        return [0.7, 0.22, 0.06, 0.02]
    return [0.2, 0.4, 0.3, 0.1] if 'critical' not in r else [0.05, 0.2, 0.4, 0.35]


def _replan_needed(s):
    return [0.7, 0.3] if 'lost' in s.get('swarm', '') or 'recalled' in s.get('swarm', '') \
        else [0.08, 0.92]


def _relay_needed(s):
    return [0.75, 0.25] if 'link poor' in s.get('comms', '') or 'lost' in s.get('comms', '') \
        else [0.1, 0.9]


def _track_verdict(s):
    ev = s.get('evidence', '')
    sc = _scores(ev)
    low = 'seen from 1' in ev or 'seen from 8' in ev or 'seen from 9' in ev     # <= 19 m
    m = re.search(r'seen from (\d+)', ev)
    best = int(m.group(1)) if m else 99
    surv, nothing = sc.get('survivor', 0.0), sc.get('nothing', 0.0)
    if 'inspection under way' in s.get('inspection', ''):
        return [0.85, 0.03, 0.03, 0.06, 0.03]
    if best <= 15:
        if surv > 0.5:
            return [0.05, 0.85, 0.04, 0.03, 0.03]
        if nothing > 0.5:
            return [0.05, 0.02, 0.08, 0.05, 0.8]
        return [0.08, 0.05, 0.75, 0.07, 0.05]
    if nothing > 0.45 and surv < 0.15:
        return [0.3, 0.0, 0.0, 0.1, 0.6]
    if surv + sc.get('animal', 0.0) > 0.3 or low:
        return [0.15, 0.0, 0.0, 0.8, 0.05]
    return [0.5, 0.0, 0.0, 0.35, 0.15]


def _report_response(s):
    r = s.get('report', '').split(' ')[0]                      # the report kind
    if r == 'request_support':
        return [0.2, 0.7, 0.05, 0.03, 0.02]
    if r == 'energy_concern':                 # H6 already brings a drone home in time
        return [0.75, 0.03, 0.15, 0.05, 0.02]
    if r == 'fault':
        return [0.35, 0.05, 0.25, 0.33, 0.02]
    if r == 'link_concern':
        return [0.6, 0.1, 0.2, 0.08, 0.02]
    return [0.9, 0.04, 0.03, 0.02, 0.01]


def _fit(state, name):
    """fit_<drone>_<sector>: nearer + more energy = better."""
    _, drone, sector = name.split('_', 2)
    line = next((ln for ln in state.get('pairs', '').split('; ')
                 if ln.startswith(f'{drone}->{sector}:')), '')
    m = re.search(r'(\d+) m away', line)
    d = int(m.group(1)) if m else 200
    tight = 'tight' in line or 'critical' in line
    if tight:
        return [0.6, 0.3, 0.08, 0.02]
    if d < 120:
        return [0.05, 0.15, 0.45, 0.35]
    if d < 220:
        return [0.1, 0.35, 0.4, 0.15]
    return [0.3, 0.4, 0.25, 0.05]


RULES = {'next_action': _next_action, 'detection_class': _detection_class,
         'detection_priority': _detection_priority, 'energy_risk': _energy_risk,
         'report_now': _report_now, 'unusual': _unusual, 'situation': _situation,
         'phase_transition': _phase_transition, 'transit_formation': _transit_formation,
         'mission_risk': _mission_risk, 'replan_needed': _replan_needed,
         'relay_needed': _relay_needed, 'track_verdict': _track_verdict,
         'report_response': _report_response}
