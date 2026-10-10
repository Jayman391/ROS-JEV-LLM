"""A10 offline tools: calibrate, ope, replay statistics, search (pure, no ROS)."""
import math
import random

import pytest

from jev_ai.core.calibrate import calibrate, fit_question, merge
from jev_ai.core.catalog import CatalogError, load_catalog
from jev_ai.core.fusion import confidence, decide_question, llm_logpref, NoiseKey
from jev_ai.core.ope import ope, overrides
from jev_ai.core.replay import summarize
from jev_ai.core.search import grid, successive_halving
from jev_ai.viz.sample import CATALOG, generate


@pytest.fixture(scope='module')
def sample():
    return generate()


def test_calibrate_recovers_known_policy():
    """Labels drawn from the §6.2 model with known β_min, β_max, γ, b_q are fitted back."""
    rng = random.Random(3)
    bmin, bmax, gamma, bias = 0.2, 1.5, 2.0, [-0.4, 0.3, 0.1]
    rows = []
    for _ in range(4000):
        p = [rng.gammavariate(0.5, 1.0) + 1e-6 for _ in range(3)]   # Jev from unsure to sure
        p = [x / sum(p) for x in p]
        q = [rng.gammavariate(0.7, 1.0) + 1e-6 for _ in range(3)]
        q = [x / sum(q) for x in q]
        c = confidence(p)
        ell = llm_logpref(q, [False] * 3, 1e-3)
        z = [math.log(p[i]) + (bmin + (bmax - bmin) * (1 - c) ** gamma) * ell[i] + bias[i]
             for i in range(3)]
        e = [math.exp(v) for v in z]
        y = rng.choices(range(3), weights=e)[0]
        rows.append({'off': [math.log(x) for x in p], 'ell': ell, 'c': c,
                     'masked': [False] * 3, 'y': y, 'p': p, 'q': q, 'logits': z})
    fit = fit_question(rows, ['a', 'b', 'c'])
    assert fit['gamma'] in (1.5, 2.0, 2.5)
    assert fit['beta_min'] == pytest.approx(bmin, abs=0.12)
    assert fit['beta_max'] == pytest.approx(bmax, abs=0.3)
    centred = [b - sum(bias) / 3 for b in bias]
    for got, want in zip(fit['bias'].values(), centred):
        assert got == pytest.approx(want, abs=0.12)


def test_calibrate_patch_corrects_first_option_bias(sample):
    records, labels = sample
    out = calibrate(records, labels)
    fit = out['questions']['next_step/next_action']
    assert fit['bias']['continue'] < 0          # sample.py inflates class 0 (first option)
    for f in out['questions'].values():
        assert f['scores']['calibrated']['nll'] <= f['scores']['jev']['nll'] + 1e-9
    assert 'beta_min' not in out['patch']['question_sets']['route']['questions']['route']
    cat = load_catalog(merge(CATALOG, out['patch']))   # the patch is a valid catalog
    q = cat.question_sets['next_step'].questions['next_action']
    assert q.bias[0] == fit['bias']['continue'] and q.policy.beta_max == fit['beta_max']
    key = NoiseKey('r', 'a', 'a', 't:0')
    with_bias = decide_question(q, [0.5, 0.3, 0.2], key=key)
    without = decide_question(q, [0.5, 0.3, 0.2], key=key, bias=[0.0] * 3)
    assert with_bias.logits[0] - without.logits[0] == pytest.approx(q.bias[0])


def test_catalog_bias_validation():
    raw = merge(CATALOG, {'question_sets': {'route': {'questions': {'route': {
        'bias': {'nowhere': 0.1}}}}}})
    with pytest.raises(CatalogError, match='bias'):
        load_catalog(raw)


def test_ope_identity_reproduces_the_log(sample):
    records, labels = sample
    cat = load_catalog(CATALOG)
    out = ope(records, cat, cat, labels=labels, boot=20)
    c = out['counts']
    assert c['reproduced'] > 0.8 * (c['reproduced'] + c['held'])
    assert c['logits_mismatch'] == 0 and c['catalog_mismatch'] == 0
    for e in out['questions'].values():
        assert e['changed'] == 0
        assert e['snips'] == pytest.approx(e['logged'])
        assert e['direct'] == pytest.approx(e['logged'])
        assert e['ess'] == pytest.approx(e['n'])


def test_ope_new_policy_and_bandit_rewards(sample):
    records, labels = sample
    old = load_catalog(CATALOG)
    new = load_catalog(overrides(CATALOG, {'tau': 1.2}))
    full = ope(records, old, new, labels=labels, boot=50)['questions']['next_step/next_action']
    assert full['changed'] > 0 and full['ess'] < full['n']
    assert full['ci95']['snips'][0] <= full['snips'] <= full['ci95']['snips'][1]
    # the same rewards as bandit feedback (logged action only): no direct estimate
    truth = {(x['run_id'], x['subject'], x['decision_id'], x['question']): x['true_class']
             for x in labels}
    rewards = []
    for r in records:
        for t in r['questions']:
            y = truth.get((r['run_id'], r['agent'], r['decision_id'], t['question']))
            if y and t['selected']:
                rewards.append({'run_id': r['run_id'], 'agent': r['agent'],
                                'decision_id': r['decision_id'], 'question': t['question'],
                                'reward': float(t['selected'] == y)})
    bandit = ope(records, old, new, rewards=rewards, boot=0)['questions']['next_step/next_action']
    assert 'direct' not in bandit
    assert bandit['snips'] == pytest.approx(full['snips'], abs=0.02)


def test_ope_overrides_reach_question_level_keys():
    raw = overrides(CATALOG, {'tau': 0.3, 'next_action.kappa': 1.5, 'T_dwell': 9})
    cat = load_catalog(raw)
    q = cat.question_sets['next_step'].questions['next_action']
    assert q.policy.tau == 0.3 and q.policy.kappa == 1.5 and q.policy.T_dwell == 9
    assert cat.question_sets['route'].questions['route'].policy.kappa == 0.7


def test_replay_summary_flags_position_bias():
    rng = random.Random(1)
    rows = []
    for _ in range(100):
        p = [rng.random() + 0.1 for _ in range(3)]
        p = [x / sum(p) for x in p]
        bumped = [p[0] * 1.5, p[1], p[2]]                # class listed first gets extra mass
        bumped = [x / sum(bumped) for x in bumped]
        rev = [p[0], p[1], p[2] * 1.5]                    # reversed: class 2 listed first
        rev = [x / sum(rev) for x in rev]
        rows.append({'question': 'q', 'classes': ['a', 'b', 'c'], 'logged': bumped,
                     'variant': bumped, 'reversed': rev, 'truth': 'a'})
    s = summarize(rows)['q']
    assert s['shift_tv'] == 0 and s['position_bias'] > 0.05 and s['gate'] == 'FAIL'
    for r in rows:
        r['reversed'] = r['variant']
    assert summarize(rows)['q']['gate'] == 'ok'


def test_search_finds_the_best_with_common_random_numbers():
    def run(params, seed):                               # same seed -> same world noise
        return -(params['x'] - 3) ** 2 + random.Random(seed).gauss(0, 2)
    rows = successive_halving(grid({'x': [0, 1, 2, 3, 4, 5]}), run, min_seeds=2, max_seeds=8)
    assert rows[0]['params'] == {'x': 3} and rows[0]['n'] == 8
    assert all(r['diff_se'] == pytest.approx(0, abs=1e-9) for r in rows if r['n'] > 1)
    assert sum(r['n'] for r in rows) < 6 * 8              # halving saved episodes
