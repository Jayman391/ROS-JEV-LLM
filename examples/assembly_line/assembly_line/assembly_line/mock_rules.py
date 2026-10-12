"""Rule hook for the keyless mock Jev (``mock.rules: assembly_line.mock_rules:line``).

Answers every line question plausibly from the encoded words (exactly what live Jev reads), with
real probability on every class: test_line checks that each class of each question wins for some
representative state, because classes a mock never picks are where live runs surprise you.
Returning None falls back to the mock's seeded Dirichlet.
"""
from __future__ import annotations

import re

CLASSES = {
    'next_product': ['hold', 'A', 'B', 'C'],
    'response': ['wait_recover', 'reroute', 'slow_line', 'pause_station'],
    'escalate': ['monitor', 'inspect_tool', 'order_tool_change'],
    'action': ['proceed', 'retry', 'tool_change', 'hand_off'],
    'coop_action': ['wait', 'abort_step'],
    'disposition': ['pass', 'rework', 'quarantine', 'scrap'],
}


def line(state: dict, question) -> dict[str, float] | None:
    rule = RULES.get(question.name)
    p = rule(state) if rule else None
    if p is None:
        return None
    p = [x + 0.02 for x in p]               # like real Jev, never certain (log-linear pooling)
    return {c: x / sum(p) for c, x in zip(CLASSES[question.name], p)}


def _int(pattern: str, text: str, default: int = 0) -> int:
    m = re.search(pattern, text)
    return int(m.group(1)) if m else default


# --- supervisor ----------------------------------------------------------------------------------

def _next_product(s):
    w = {}
    for seg in s.get('orders', '').split('; '):
        pr = seg[:1]
        if not pr or 'nothing left' in seg:
            continue
        w[pr] = 3.0 if '(late)' in seg else 2.0 if '(due soon)' in seg else 1.0
    for seg in s.get('routes', '').split('; '):
        pr = seg[:1]
        if pr in w and 'blocked' in seg:
            w[pr] *= 0.02
        elif pr in w and 'lap' in seg:
            w[pr] *= 0.5
    hold = 0.4
    ln = s.get('line', '')
    wip, cap = _int(r'WIP (\d+)', ln), _int(r'cap (\d+)', ln, 99)
    if wip >= cap - 1:
        hold += 2.0
    if 'full buffers: S' in ln:
        hold += 1.5
    return [hold] + [w.get(pr, 0.0) for pr in 'ABC']


def _response(s):
    ev = s.get('event', '')
    alts = any(not seg.endswith('none') for seg in s.get('alternatives', '').split('; ') if seg)
    if _int(r'fault (\d+) this shift', ev) >= 3 or 'lost its sensors' in ev:
        return [0.1, 0.25, 0.1, 0.6]
    if 'backed up' in ev:
        return [0.2, 0.3 if alts else 0.05, 0.5, 0.05]
    if 'clear soon' in ev:
        return [0.7, 0.15, 0.1, 0.05]
    if alts:
        return [0.2, 0.6, 0.15, 0.05]
    return [0.25, 0.05, 0.6, 0.1]


def _escalate(s):
    m = re.search(r'(\d+) of its last (\d+)', s.get('qa_record', ''))
    ratio = int(m.group(1)) / max(1, int(m.group(2))) if m else 0.0
    worn = re.search(r'wear (high|worn out)', s.get('tool', '')) is not None
    if ratio >= 0.4 and worn:
        return [0.05, 0.25, 0.7]
    if ratio >= 0.3 or worn:
        return [0.25, 0.6, 0.15]
    return [0.75, 0.2, 0.05]


# --- arm -----------------------------------------------------------------------------------------

def _action(s):
    grip, tool, ctx = s.get('grip', ''), s.get('tool', ''), s.get('context', '')
    regrips = _int(r'(\d+) regrip', s.get('operation', ''))
    if 'inspect the tool' in ctx or 'worn out' in tool:
        return [0.15, 0.05, 0.7, 0.1]
    if grip.startswith('slipping'):
        return [0.05, 0.7, 0.05, 0.2] if regrips < 2 else [0.1, 0.05, 0.05, 0.8]
    if grip.startswith('marginal') or s.get('alignment', '').startswith('misaligned'):
        return [0.25, 0.65, 0.05, 0.05] if regrips < 2 else [0.6, 0.1, 0.1, 0.2]
    if 'wear high' in tool:
        return [0.35, 0.05, 0.55, 0.05]
    if 'dropout' in s.get('sensors', ''):
        return [0.45, 0.05, 0.05, 0.45 if 'no other arm' not in ctx else 0.0]
    return [0.8, 0.1, 0.05, 0.05]


def _coop_action(s):
    w, partner = s.get('waiting', ''), s.get('partner', '')
    if w.startswith('very late') or 'jammed' in partner or 'changing its tool' in partner:
        return [0.2, 0.8]
    return [0.65, 0.35] if w.startswith('late') else [0.9, 0.1]


# --- QA ------------------------------------------------------------------------------------------

def _disposition(s):
    r = s.get('readings', '')
    n_missing, n_fail = r.count(': missing'), r.count(': fail')
    n_suspect = r.count(': suspect')
    m = re.search(r'reworked (\d+) of at most (\d+)', s.get('part', ''))
    left = m is None or int(m.group(1)) < int(m.group(2))
    if n_fail + n_missing >= 2 and not left:
        return [0.0, 0.0, 0.3, 0.7]
    if n_missing or n_fail:
        return [0.05, 0.75, 0.15, 0.05] if left else [0.05, 0.0, 0.6, 0.35]
    if n_suspect:
        return [0.3, 0.55, 0.15, 0.0] if left else [0.4, 0.0, 0.55, 0.05]
    # a low marginal reading is near a coin flip on q_min (0.75); an escape costs far more than a redo
    lows = [float(x) for x in re.findall(r'marginal \((\d\.\d+)\)', r) if float(x) < 0.76]
    if lows:
        return [0.3, 0.6, 0.1, 0.0] if left else [0.35, 0.0, 0.6, 0.05]
    return [0.75, 0.2, 0.05, 0.0]


RULES = {'next_product': _next_product, 'response': _response, 'escalate': _escalate,
         'action': _action, 'coop_action': _coop_action, 'disposition': _disposition}
