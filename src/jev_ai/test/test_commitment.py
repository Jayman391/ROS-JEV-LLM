import pytest

from jev_ai.core.catalog import load_catalog
from jev_ai.core.commitment import (COMMITTED, FALLBACK, FORCED, KEPT, PROVISIONAL, REVISED,
                                    Commitment, TokenBucket, argmax_changed,
                                    augmentation_priority)

from conftest import STATE

FAR = [0.2, 0.05, 0.75]       # halt_now argmax with margin > log 2


def P(next_action, urgency=(0.6, 0.3, 0.1), report=(0.3, 0.7)):
    return {'next_action': list(next_action), 'urgency': list(urgency), 'report_now': list(report)}


def onehot(qset, cls):
    return {'next_action': [1.0 if c == cls else 0.0
                            for c in qset.questions['next_action'].class_names]}


def machine(qset):
    return Commitment(qset, agent='a1', run_id='r')


def test_provisional_then_kept(qset):
    m = machine(qset)
    s = m.on_event(0.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]), triggers=['app:start'])
    assert (s.phase, s.act, s.final) == (PROVISIONAL, True, False)
    assert s.request_llm == 'high' and 'low_confidence' in s.triggers
    assert set(s.selected) == {'next_action', 'urgency', 'report_now'}
    steps = m.on_llm(1.0, generation=s.generation,
                     q_llm=onehot(qset, s.selected['next_action']))
    assert [x.phase for x in steps] == [KEPT]
    assert steps[0].final and not steps[0].act and steps[0].decision_id == s.decision_id
    assert steps[0].traces['next_action'].q_llm                       # augmentation recorded


def test_revised_then_committed(qset):
    for now in range(0, 500, 5):
        m = machine(qset)
        s = m.on_event(float(now), state=STATE, p_jev=P([0.5, 0.45, 0.05]))
        if s.selected['next_action'] == 'continue':
            break
    steps = m.on_llm(now + 1.0, generation=s.generation, q_llm=onehot(qset, 'inspect'))
    assert [x.phase for x in steps] == [REVISED, COMMITTED]
    assert steps[0].act and not steps[1].act and steps[1].final
    assert steps[0].selected['next_action'] == 'inspect' == m.current['next_action']
    # same noise in both passes (§6.3 minimal revisions)
    assert steps[0].traces['next_action'].noise == s.traces['next_action'].noise


def test_stale_results_are_discarded(qset):
    m = machine(qset)
    s1 = m.on_event(0.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    s2 = m.on_event(1.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    q = onehot(qset, 'continue')
    assert m.on_llm(2.0, generation=s1.generation, q_llm=q) == []          # generation advanced
    assert m.on_llm(2.0, generation=s2.generation, q_llm=q,
                    masked={'next_action': {'inspect'}}) == []           # masks changed
    assert m.on_llm(2.0, generation=s2.generation, q_llm=q) != []


def test_timeout_and_failure_commit_provisional(qset):
    m = machine(qset)
    s = m.on_event(0.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    assert m.poll(1.0) == []
    steps = m.poll(qset.policy.T_wait)
    assert [x.phase for x in steps] == [COMMITTED] and steps[0].selected == s.selected
    assert not steps[0].act
    m.on_event(10.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    assert [x.phase for x in m.on_llm_failed(11.0, m.generation)] == [COMMITTED]


def test_irreversible_waits_then_falls_back_on_timeout(qset):
    m = machine(qset)
    s = m.on_event(0.0, state=STATE, p_jev=P(FAR), current={'next_action': 'inspect'})
    t = s.traces['next_action']
    assert (t.path, t.selected) == ('irreversible_pending', 'inspect')   # hold current
    assert s.request_llm == 'reserved'
    steps = m.poll(qset.policy.T_wait)
    assert [x.phase for x in steps] == [FALLBACK] and steps[0].act
    assert steps[0].selected['next_action'] == 'continue'


def test_irreversible_commits_with_augmentation(qset):
    m = machine(qset)
    s = m.on_event(0.0, state=STATE, p_jev=P(FAR))
    steps = m.on_llm(1.0, generation=s.generation, q_llm=onehot(qset, 'halt_now'))
    assert [x.phase for x in steps] == [REVISED, COMMITTED]
    assert steps[0].selected['next_action'] == 'halt_now'
    assert steps[0].traces['next_action'].irreversible_path


def test_irreversible_vetoed_by_llm_repicks_reversible(qset):
    """A2 decision: the LLM's advice is used to re-pick among reversible classes, not FALLBACK."""
    m = machine(qset)
    s = m.on_event(0.0, state=STATE, p_jev=P(FAR), current={'next_action': 'inspect'})
    assert s.selected['next_action'] == 'inspect'                       # held while pending
    steps = m.on_llm(1.0, generation=s.generation, q_llm=onehot(qset, 'continue'))
    assert [x.phase for x in steps] == [REVISED, COMMITTED]
    t = steps[0].traces['next_action']
    assert (t.selected, t.path, t.irreversible_path) == ('continue', 'sample', False)


def test_llm_answer_without_weights_is_not_augmentation(qset):
    """A result that parsed but carried no weights for the question can't unlock an irreversible
    class."""
    m = machine(qset)
    s = m.on_event(0.0, state=STATE, p_jev=P(FAR))
    steps = m.on_llm(1.0, generation=s.generation, q_llm={'urgency': [0.2, 0.3, 0.5]})
    assert [x.phase for x in steps] == [FALLBACK]
    assert steps[0].selected['next_action'] == 'continue'


def test_forced_and_skip(raw, qset):
    m = machine(qset)
    s = m.on_event(0.0, state=STATE, p_jev=P(FAR), required={'next_action': 'halt_now'})
    assert (s.phase, s.final, s.request_llm) == (FORCED, True, '')
    assert s.selected['next_action'] == 'halt_now'
    skipping = load_catalog(raw, {'skip_llm_above_c': 0.0}).question_sets['next_step']
    s = machine(skipping).on_event(0.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    assert (s.phase, s.final, s.act, s.request_llm) == (COMMITTED, True, True, '')


def test_per_question_skip_gates_only_that_question(raw):
    """skip_llm_above_c on next_action alone: the always-uncertain yes/no report_now can't keep
    the LLM firing."""
    raw['question_sets']['next_step']['questions']['next_action']['skip_llm_above_c'] = 0.5
    qs = load_catalog(raw).question_sets['next_step']
    s = machine(qs).on_event(0.0, state=STATE, p_jev=P([0.98, 0.01, 0.01], report=(0.5, 0.5)))
    assert (s.phase, s.request_llm) == (COMMITTED, '')              # next_action c = 0.90
    s = machine(qs).on_event(0.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    assert s.phase == PROVISIONAL and s.request_llm                  # next_action c = 0.18


def test_dwell_blocks_voluntary_switch_not_mask_forced(qset):
    m = machine(qset)
    m.on_event(0.0, state=STATE, p_jev=P(FAR), required={'next_action': 'inspect'})
    strong_continue = P([0.98, 0.01, 0.01])
    s = m.on_event(1.0, state=STATE, p_jev=strong_continue)
    assert (s.selected['next_action'], s.traces['next_action'].path) == ('inspect', 'dwell')
    s = m.on_event(2.0, state=STATE, p_jev=strong_continue, masked={'next_action': {'inspect'}})
    assert s.selected['next_action'] == 'continue'                       # immediate
    m2 = machine(qset)
    m2.on_event(0.0, state=STATE, p_jev=P(FAR), required={'next_action': 'inspect'})
    s = m2.on_event(qset.policy.T_dwell + 1, state=STATE, p_jev=strong_continue)
    assert s.selected['next_action'] == 'continue'                       # dwell elapsed


def test_committed_class_is_kept_until_completed(qset):
    """A5 decision: an until_commit class past commit_point() is held, not aborted."""
    m = machine(qset)
    m.on_event(0.0, state=STATE, p_jev=P(FAR), required={'next_action': 'inspect'})
    m.commit_point('next_action')
    strong_continue = P([0.98, 0.01, 0.01])
    s = m.on_event(60.0, state=STATE, p_jev=strong_continue)
    t = s.traces['next_action']
    assert (t.selected, t.path) == ('inspect', 'committed')
    s = m.on_event(61.0, state=STATE, p_jev=strong_continue, masked={'next_action': {'inspect'}})
    assert s.selected['next_action'] == 'continue'                       # masks still win
    m.on_event(62.0, state=STATE, p_jev=P(FAR), required={'next_action': 'inspect'})
    m.completed('next_action')
    s = m.on_event(120.0, state=STATE, p_jev=strong_continue)
    assert s.selected['next_action'] == 'continue'                       # released


def test_jev_failures_keep_current_then_degrade(qset):
    m = machine(qset)
    m.on_event(0.0, state=STATE, p_jev=P(FAR), required={'next_action': 'inspect'})
    for i in (1, 2):
        s = m.on_event(float(i), state=STATE, p_jev={})
        assert s.selected['next_action'] == 'inspect' and not s.degraded
        assert 'report_now' not in s.selected        # no Jev, no report-type output
    s = m.on_event(3.0, state=STATE, p_jev={})
    assert (s.phase, s.degraded, s.selected['next_action']) == (FALLBACK, True, 'continue')
    s = m.on_event(4.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    assert not s.degraded


def test_cached_llm_reused_only_for_same_state(qset):
    m = machine(qset)
    s = m.on_event(0.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    m.on_llm(1.0, generation=s.generation, q_llm=onehot(qset, 'continue'))
    s = m.on_event(2.0, state=STATE, p_jev=P([0.6, 0.3, 0.1]))
    assert s.traces['next_action'].q_llm and s.traces['next_action'].llm_age_s == 1.0
    s = m.on_event(3.0, state={**STATE, 'energy': 'low'}, p_jev=P([0.6, 0.3, 0.1]))
    assert not s.traces['next_action'].q_llm


def test_token_bucket():
    b = TokenBucket(6, now=0.0)
    assert [b.take(0.0) for _ in range(6)] == [True] * 5 + [False]       # one slot held back
    assert b.take(0.0, 'reserved') and not b.take(0.0, 'reserved')
    assert not b.take(10.0, 'normal') and b.take(10.0, 'reserved')       # 10 s refills one
    full = TokenBucket(6, now=0.0)
    assert full.take(0.0, 'low') and full.take(0.0, 'low') and full.take(0.0, 'low')
    assert not full.take(0.0, 'low') and full.take(0.0, 'normal')


def test_argmax_changed():
    assert argmax_changed(None, [0.5, 0.5], 0.1)
    assert not argmax_changed([0.5, 0.4, 0.1], [0.45, 0.5, 0.05], 0.1)   # within margin
    assert argmax_changed([0.5, 0.4, 0.1], [0.3, 0.6, 0.1], 0.1)
    assert not argmax_changed([0.5, 0.4, 0.1], [0.6, 0.3, 0.1], 0.1)


@pytest.mark.parametrize('triggers, c, pending, app, expect', [
    ([], 0.9, False, '', 'normal'),
    (['periodic'], 0.9, False, '', 'low'),
    (['periodic'], 0.2, False, '', 'high'),
    (['periodic'], 0.9, True, '', 'reserved'),
    (['periodic'], 0.9, False, 'high', 'high'),
])
def test_augmentation_priority(triggers, c, pending, app, expect):
    assert augmentation_priority(triggers, c, 0.5, pending, app) == expect
