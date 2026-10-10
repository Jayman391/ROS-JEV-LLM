"""Golden vectors: freeze fusion, noise and selection outputs.

Logged records are re-derived by `ros2 jev ope/replay` (§12), so any change here silently
invalidates old logs. If a change is intentional, regenerate and review the diff:
    JEV_AI_REGEN_GOLDEN=1 python3 -m pytest src/jev_ai/test/test_golden.py
"""
import json
import os
from pathlib import Path

import pytest

from jev_ai.core.fusion import NoiseKey, decide_question

GOLDEN = Path(__file__).parent / 'golden' / 'vectors.json'
FAR = [0.2, 0.05, 0.75]
CASES = {
    'plain': dict(q='next_action', p=[0.5, 0.3, 0.2]),
    'llm': dict(q='next_action', p=[0.5, 0.3, 0.2], q_llm=[0.1, 0.8, 0.1]),
    'llm_aged': dict(q='next_action', p=[0.5, 0.3, 0.2], q_llm=[0.1, 0.8, 0.1], age=12.0),
    'masked': dict(q='next_action', p=[0.5, 0.3, 0.2], masked=['inspect']),
    'kappa': dict(q='next_action', p=[0.5, 0.3, 0.2], current='inspect', epoch=3),
    'irrev_pending': dict(q='next_action', p=FAR, current='inspect'),
    'irrev_commit': dict(q='next_action', p=FAR, q_llm=[0.0, 0.0, 1.0], augmented=True),
    'directive': dict(q='next_action', p=[0.4, 0.4, 0.2], directive=True),
    'expectation': dict(q='urgency', p=[0.2, 0.3, 0.5], q_llm=[0.6, 0.3, 0.1]),
    'threshold': dict(q='report_now', p=[0.45, 0.55], q_llm=[0.9, 0.1]),
}
FIELDS = ('logits', 'beta', 'noise', 'group_noise', 'selected', 'propensity', 'path', 'fused',
          'values', 'jev_confidence')


def run(qset, c):
    key = NoiseKey('golden-run', 'a1', 'fleet', f"t:{c.get('epoch', 0)}", c.get('directive', False))
    t = decide_question(qset.questions[c['q']], c['p'], key=key, q_llm=c.get('q_llm'),
                        llm_age_s=c.get('age', 0.0), masked=set(c.get('masked', ())),
                        current=c.get('current', ''), augmented=c.get('augmented', False))
    return {f: getattr(t, f) for f in FIELDS}


def test_golden_vectors(qset):
    got = {name: run(qset, c) for name, c in CASES.items()}
    if os.environ.get('JEV_AI_REGEN_GOLDEN'):
        GOLDEN.parent.mkdir(exist_ok=True)
        GOLDEN.write_text(json.dumps(got, indent=1, sort_keys=True) + '\n')
    want = json.loads(GOLDEN.read_text())
    assert set(want) == set(got)
    for name in CASES:
        for f in FIELDS:
            w, g = want[name][f], got[name][f]
            if isinstance(w, (list, float)):
                assert g == pytest.approx(w, rel=1e-12, abs=1e-12), (name, f)
            else:
                assert g == w, (name, f)
