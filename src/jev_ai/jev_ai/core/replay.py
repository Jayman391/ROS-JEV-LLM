"""Replay statistics (FRAMEWORK §12; phase A10): how Jev's answers on logged states move under a
variant catalog (wording, criteria) and under a reversed class order.

``position bias`` = mean over states of p(first class | listed first) − p(first class | listed
last): what the status-quo class gains from its catalog position alone. It is the gate for the
status-quo-first ordering (§6.3): above ``GATE`` the catalog order is doing work that κ should.
Pure; the ``ros2 jev replay`` verb asks the backend.
"""
from __future__ import annotations

from jev_ai.core.metrics import _num, group_decisions

GATE = 0.05


def logged_states(records: list[dict], limit: int = 200) -> list[dict]:
    """Distinct (question set, state) with Jev's logged answer, evenly thinned to ``limit``."""
    seen, out = set(), []
    for d in group_decisions(records):
        r = d['records'][0]
        state = [(kv['key'], kv['value']) for kv in r.get('state') or []]
        key = (r.get('question_set', ''), tuple(state))
        p = {t['question']: [_num(x) for x in t.get('p_jev') or []]
             for t in r.get('questions') or []}
        p = {q: v for q, v in p.items() if v and None not in v}
        if key in seen or not p:
            continue
        seen.add(key)
        out.append({'question_set': key[0], 'state': state, 'p_jev': p, 'final': d['final']})
    if len(out) > limit:
        out = [out[int(i * len(out) / limit)] for i in range(limit)]
    return out


def _tv(p, q):
    return 0.5 * sum(abs(a - b) for a, b in zip(p, q))


def _argmax(p):
    return max(range(len(p)), key=p.__getitem__)


def summarize(rows: list[dict]) -> dict:
    """``rows``: {question, classes, logged, variant, reversed, truth} with distributions in the
    catalog's class order (``reversed`` already mapped back). Per question: shift and bias."""
    by = {}
    for r in rows:
        by.setdefault(r['question'], []).append(r)
    out = {}
    for q, rs in by.items():
        n = len(rs)
        acc = {}
        lab = [r for r in rs if r.get('truth') in r['classes']]
        for k in ('logged', 'variant', 'reversed'):
            if lab:
                acc[k] = sum(r['classes'][_argmax(r[k])] == r['truth'] for r in lab) / len(lab)
        bias = sum(r['variant'][0] - r['reversed'][0] for r in rs) / n
        out[q] = {
            'n': n, 'first_class': rs[0]['classes'][0],
            'shift_tv': sum(_tv(r['logged'], r['variant']) for r in rs) / n,
            'argmax_changed': sum(_argmax(r['logged']) != _argmax(r['variant']) for r in rs) / n,
            'first_option_rate': sum(_argmax(r['variant']) == 0 for r in rs) / n,
            'first_option_rate_reversed': sum(
                _argmax(r['reversed']) == len(r['classes']) - 1 for r in rs) / n,
            'position_bias': bias, 'gate': 'ok' if abs(bias) < GATE else 'FAIL',
            'accuracy': acc, 'n_labelled': len(lab)}
    return out
