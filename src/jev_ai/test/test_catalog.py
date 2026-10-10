import math

import pytest

from jev_ai.core.catalog import CatalogError, Policy, load_catalog

from conftest import STATE


def test_example_loads(catalog, qset):
    assert catalog.name == 'patrol'
    q = qset.questions['next_action']
    assert q.class_names == ['continue', 'inspect', 'halt_now']
    assert q.irreversible() == {'halt_now'}
    assert q.irreversible(after_commit_point=True) == {'halt_now', 'inspect'}
    assert q.policy.tau == 0.5 and q.policy.coupling == 0.5
    assert qset.questions['report_now'].class_names == ['true', 'false']
    assert qset.questions['report_now'].criteria[0] == 'Something changed worth reporting.'
    assert qset.questions['urgency'].policy.tau == 0.5          # catalog default
    assert qset.llm_params == {'inspect_altitude_m': (8.0, 15.0, 10.0)}
    assert catalog.policy.T_wait == pytest.approx(6.5)


def test_hashes_are_stable_and_sensitive(raw):
    a, b = load_catalog(raw), load_catalog(raw)
    assert a.catalog_hash == b.catalog_hash and a.policy_hash == b.policy_hash
    raw['question_sets']['next_step']['questions']['next_action']['tau'] = 0.6
    assert load_catalog(raw).catalog_hash != a.catalog_hash
    assert load_catalog(raw, {'kappa': 1.0}).policy_hash != a.policy_hash


def test_ordered_state(qset):
    assert [k for k, _ in qset.ordered_state(dict(reversed(list(STATE.items()))))] == \
        list(qset.state_keys)
    with pytest.raises(CatalogError, match='missing'):
        qset.ordered_state({'task': 'x'})
    with pytest.raises(CatalogError, match='strings'):
        qset.ordered_state({**STATE, 'energy': 0.7})


def _q(raw, name='next_action'):
    return raw['question_sets']['next_step']['questions'][name]


@pytest.mark.parametrize('mutate, match', [
    (lambda r: _q(r).update(fallback='halt_now'), 'reversible: true'),
    (lambda r: _q(r).update(fallback='inspect'), 'reversible: true'),       # until_commit
    (lambda r: _q(r).update(fallback='nope'), 'not a class'),
    (lambda r: _q(r).update(act='expectation'), 'not allowed'),
    (lambda r: _q(r, 'report_now').update(act='expectation'), 'not allowed'),
    (lambda r: _q(r).update(coupling=1.5), 'coupling'),
    (lambda r: _q(r).update(tau=0.0), 'tau'),
    (lambda r: _q(r).update(typo=1), 'unknown keys'),
    (lambda r: _q(r)['classes'].append({'name': 'continue'}), 'duplicate'),
    (lambda r: _q(r).update(classes=['only']), '2-255'),
    (lambda r: _q(r, 'urgency').update(classes=[str(i) for i in range(11)]), '2-10'),
    (lambda r: _q(r, 'urgency')['classes'][0].update(reversible=False), 'sample|argmax'),
    (lambda r: _q(r).update(instructions=' '), 'empty'),
    (lambda r: r['question_sets']['next_step'].update(augment=False), 'augment: true'),
    (lambda r: r['question_sets']['next_step'].update(
        llm_params={'x': {'min': 5, 'max': 1, 'default': 2}}), 'min <= default'),
    (lambda r: r.update(policy={'T_wait': 3.0}), 'T_wait'),
    (lambda r: r.update(policy={'beta_max': 9}), 'beta_max'),
    (lambda r: r.update(policy={'bogus': 1}), 'unknown policy'),
    (lambda r: r.update(catalog_version=2), 'catalog_version'),
])
def test_validation_rejects(raw, mutate, match):
    mutate(raw)
    with pytest.raises(CatalogError, match=match):
        load_catalog(raw)


def test_policy_defaults_match_spec():
    p = Policy()
    assert (p.beta_min, p.beta_max, p.gamma) == (0.1, 1.0, 2.0)
    assert p.B == pytest.approx(math.log(20)) and p.m_irrev == pytest.approx(math.log(2))
    assert (p.tau, p.kappa, p.coupling, p.coupling_directive) == (0.5, 0.7, 0.5, 1.0)
    assert p.T_wait == p.llm_timeout + 0.5
