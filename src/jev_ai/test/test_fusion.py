import copy
import math

import pytest

from jev_ai.core.catalog import load_catalog
from jev_ai.core.fusion import (NoiseKey, beta, confidence, decide_question, epoch_key,
                                llm_logpref, pool, softmax, unit)

LOG = math.log


def key(epoch=0, agent='a1', group='fleet', directive=False, run='r'):
    return NoiseKey(run, agent, group, f't:{epoch}', directive)


def test_confidence():
    assert confidence([1 / 3] * 3) == pytest.approx(0.0)
    assert confidence([1.0, 0.0, 0.0]) == pytest.approx(1.0)
    assert confidence([0.5, 0.5]) == pytest.approx(0.0)
    assert 0 < confidence([0.7, 0.2, 0.1]) < 1


def test_beta_default_schedule(catalog):
    p = catalog.policy
    assert beta(1.0, p) == pytest.approx(0.1)        # high-confidence nondeterminism floor
    assert beta(0.0, p) == pytest.approx(1.0)
    assert beta(0.5, p) == pytest.approx(0.325)      # gamma = 2
    assert beta(confidence([0.8, 0.1, 0.1]), p) == pytest.approx(0.40, abs=0.01)
    assert beta(1.7, p) == pytest.approx(0.1)        # clamped


def test_pool_hand_computed(catalog):
    """p uniform (c=0, beta=1); LLM all-in on class 0; eta=1e-3; B=log 20."""
    p = catalog.policy
    ell = llm_logpref([1.0, 0.0, 0.0], [False] * 3, p.eta)
    assert ell == pytest.approx([2 / 3 * -LOG(1e-3), 1 / 3 * LOG(1e-3), 1 / 3 * LOG(1e-3)])
    z, b = pool([1 / 3] * 3, [1.0, 0.0, 0.0], [False] * 3, [False] * 3, 0.0, 0.0, p)
    assert b == pytest.approx(1.0)
    assert z[0] == pytest.approx(LOG(1 / 3) + LOG(20))          # clipped at +B
    assert z[1] == pytest.approx(LOG(1 / 3) + LOG(1e-3) / 3)     # -2.30 nats, inside the bound


def test_pool_no_llm_is_log_p(catalog):
    z, b = pool([0.6, 0.3, 0.1], None, [False] * 3, [False] * 3, 0.5, 0.0, catalog.policy)
    assert b == 0.0 and z == pytest.approx([LOG(0.6), LOG(0.3), LOG(0.1)])


def test_pool_irreversible_one_sided_and_masks(catalog):
    p = catalog.policy
    z, _ = pool([0.4, 0.3, 0.3], [0.0, 0.0, 1.0], [False] * 3, [False, False, True], 0.0, 0.0, p)
    assert z[2] == pytest.approx(LOG(0.3))            # LLM cannot raise the irreversible class
    z, _ = pool([0.4, 0.3, 0.3], [1.0, 0.0, 0.0], [False] * 3, [False, False, True], 0.0, 0.0, p)
    assert z[2] < LOG(0.3)                            # ...but can lower it
    # masked class: -inf, and its LLM weight does not leak into the others' ℓ
    za, _ = pool([0.4, 0.3, 0.3], [0.5, 0.5, 0.0], [False, False, True], [False] * 3, 0.0, 0.0, p)
    zb, _ = pool([0.4, 0.3, 0.3], [0.5, 0.5, 0.9], [False, False, True], [False] * 3, 0.0, 0.0, p)
    assert za[2] == zb[2] == -math.inf and za[:2] == pytest.approx(zb[:2])


def test_pool_age_decay(catalog):
    p = catalog.policy
    _, b0 = pool([0.5, 0.5], [0.9, 0.1], [False] * 2, [False] * 2, 0.0, 0.0, p)
    _, b1 = pool([0.5, 0.5], [0.9, 0.1], [False] * 2, [False] * 2, 0.0, p.T_stale, p)
    assert b1 == pytest.approx(b0 / math.e)


def test_unit_and_epoch():
    us = [unit('x', i) for i in range(1000)]
    assert all(0 < u < 1 for u in us) and 0.45 < sum(us) / len(us) < 0.55
    assert epoch_key(12.0, 5.0) == ('t:2', False)
    assert epoch_key(12.0, 5.0, 'coord', 7) == ('d:coord:7', True)


# --- selection ------------------------------------------------------------------------------

P = [0.5, 0.3, 0.2]


def nxt(qset):
    return qset.questions['next_action']


def test_sampling_matches_softmax(qset):
    """Gumbel-max: P(select a) = softmax(z / tau) over eligible classes."""
    q = nxt(qset)
    p = [0.5, 0.4, 0.1]
    counts = {c: 0 for c in q.class_names}
    n = 4000
    for e in range(n):
        t = decide_question(q, p, key=key(e), irreversible={'halt_now'})
        counts[t.selected] += 1
    expect = softmax([LOG(0.5) / 0.5, LOG(0.4) / 0.5])
    assert counts['halt_now'] == 0
    assert counts['continue'] / n == pytest.approx(expect[0], abs=0.03)


def test_propensity_is_softmax_of_eligible(qset):
    t = decide_question(nxt(qset), P, key=key(3), current='inspect')
    base = [(LOG(0.5)) / 0.5, (LOG(0.3) + 0.7) / 0.5]
    assert t.propensity == pytest.approx(softmax(base)[['continue', 'inspect'].index(t.selected)])


def test_order_free(raw):
    """Permuting catalog class order (and p accordingly) never changes the selection."""
    q1 = nxt(load_catalog(raw).question_sets['next_step'])
    r2 = copy.deepcopy(raw)
    r2['question_sets']['next_step']['questions']['next_action']['classes'].reverse()
    q2 = nxt(load_catalog(r2).question_sets['next_step'])
    p1 = [0.45, 0.45, 0.1]
    for e in range(300):
        a = decide_question(q1, p1, key=key(e), irreversible=set())
        b = decide_question(q2, p1[::-1], key=key(e), irreversible=set())
        assert a.selected == b.selected


def test_stable_under_masking_and_masks_absolute(qset):
    q = nxt(qset)
    for e in range(300):
        t = decide_question(q, P, key=key(e), irreversible=set())
        others = [c for c in q.class_names if c not in (t.selected, q.fallback)]
        for c in others:
            assert decide_question(q, P, key=key(e), masked={c}, irreversible=set()).selected \
                == t.selected
        if t.selected != q.fallback:
            m = decide_question(q, P, key=key(e), masked={t.selected}, irreversible=set())
            assert m.selected != t.selected


def test_fallback_never_masked_and_require(qset):
    q = nxt(qset)
    t = decide_question(q, P, key=key(), masked={'continue', 'inspect'})
    assert t.selected == 'continue' and 'fallback_mask_ignored' in t.flags
    t = decide_question(q, P, key=key(), required='halt_now')
    assert (t.selected, t.path, t.propensity) == ('halt_now', 'forced', 1.0)
    t = decide_question(q, P, key=key(), required='inspect', masked={'inspect'})
    assert t.selected == 'inspect' and 'require_overrides_mask' in t.flags
    with pytest.raises(ValueError):
        decide_question(q, P, key=key(), required='fly')


def test_irreversible_rule(qset):
    q = nxt(qset)
    near = [0.3, 0.2, 0.5]          # halt argmax but margin log(0.5/0.3)=0.51 < log 2
    far = [0.2, 0.05, 0.75]         # margin log(3.75) > log 2
    for e in range(200):
        assert decide_question(q, near, key=key(e), augmented=True).selected != 'halt_now'
    t = decide_question(q, far, key=key(), current='inspect')
    assert (t.selected, t.path) == ('inspect', 'irreversible_pending')     # hold current
    t = decide_question(q, far, key=key(), current='halt_now')
    assert t.selected == 'continue'          # current not selectable -> fallback
    t = decide_question(q, far, key=key(), augmented=True)
    assert (t.selected, t.irreversible_path) == ('halt_now', True)
    # an LLM that disagrees lowers the irreversible class below the margin
    t = decide_question(q, far, key=key(), q_llm=[0.9, 0.1, 0.0], augmented=True)
    assert t.selected != 'halt_now'


def test_llm_cannot_create_an_irreversible_action(qset):
    """Jev prefers continue; an LLM all-in on halt_now pushes the reversible classes down by
    ~2.3 nats, which on fused logits alone would make halt_now win by > log 2."""
    q = nxt(qset)
    p = [0.4, 0.25, 0.35]
    for e in range(200):
        t = decide_question(q, p, key=key(e), q_llm=[0.0, 0.0, 1.0], augmented=True)
        assert t.selected != 'halt_now' and not t.irreversible_path
    z = t.logits
    assert z[2] - max(z[0], z[1]) > math.log(2)     # the fused-only rule would have fired


def test_llm_input_validation(catalog):
    p = catalog.policy
    with pytest.raises(ValueError):
        pool([0.5, 0.5], [1.0], [False] * 2, [False] * 2, 0.0, 0.0, p)
    _, b_neg = pool([0.5, 0.5], [0.9, 0.1], [False] * 2, [False] * 2, 0.0, -100.0, p)
    assert b_neg == pytest.approx(beta(0.0, p))     # negative age cannot amplify beta
    assert unit('a\x1fb') != unit('a', 'b')         # no key-separator forgery


def test_coupling(qset):
    q = nxt(qset)
    p = [0.4, 0.35, 0.25]
    agree = lambda k1, k2: sum(  # noqa: E731
        decide_question(q, p, key=k1(e), irreversible=set()).selected ==
        decide_question(q, p, key=k2(e), irreversible=set()).selected for e in range(400)) / 400
    same_group = agree(lambda e: key(e, 'a1', directive=True), lambda e: key(e, 'a2', directive=True))
    assert same_group == 1.0                               # rho = 1.0 for directives
    autonomous = agree(lambda e: key(e, 'a1'), lambda e: key(e, 'a2'))
    independent = agree(lambda e: key(e, 'a1', group='a1'), lambda e: key(e, 'a2', group='a2'))
    assert independent < autonomous < 1.0                  # rho = 0.5 sits in between


def test_dwell_and_jev_error(qset):
    q = nxt(qset)
    t = decide_question(q, [0.05, 0.9, 0.05], key=key(), current='continue', may_switch=False)
    assert (t.selected, t.path) == ('continue', 'dwell')
    t = decide_question(q, [0.05, 0.9, 0.05], key=key(), current='continue', may_switch=False,
                        masked={'continue'})
    assert t.selected == 'continue'     # fallback can't be masked; dwell keeps it
    t = decide_question(q, None, key=key(), current='inspect')
    assert (t.selected, t.path) == ('inspect', 'jev_error')
    t = decide_question(q, None, key=key(), current='inspect', masked={'inspect'})
    assert t.selected == 'continue'


def test_expectation_and_threshold(qset):
    t = decide_question(qset.questions['urgency'], [0.2, 0.4, 0.4], key=key())
    assert t.values['urgency.level_mean'] == pytest.approx((0.4 + 2 * 0.4) / 2)
    assert t.values['urgency.level_median'] == 'low' and t.selected == 'low'
    t = decide_question(qset.questions['report_now'], [0.6, 0.4], key=key())
    assert t.selected == 'true' and t.values['report_now.p_true'] == pytest.approx(0.6)
    t = decide_question(qset.questions['report_now'], [0.4, 0.6], key=key())
    assert t.selected == 'false'


def test_noise_is_reproducible(qset):
    a = decide_question(nxt(qset), P, key=key(9))
    b = decide_question(nxt(qset), P, key=key(9))
    assert a.noise == b.noise and a.selected == b.selected
    assert decide_question(nxt(qset), P, key=key(9, run='other')).noise != a.noise
