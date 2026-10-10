"""Metrics (A5.5), the synthetic generator and the static report: known answers on tiny hand-built
records, invariants on the synthetic fleet."""
import json
import math

import pytest

from jev_ai.core import metrics
from jev_ai.core.records import dumps
from jev_ai.viz import report, sample

LOG = math.log


def q(name='a', classes=('x', 'y'), p=(0.6, 0.4), ql=(), logits=None, masked=None,
      selected='x', beta=0.0, propensity=1.0, group_noise=False, **kw):
    n = len(classes)
    return {'question': name, 'classes': list(classes), 'p_jev': list(p), 'jev_confidence': 0.5,
            'q_llm': list(ql), 'llm_age_s': 0.0, 'beta': beta,
            'masked': list(masked or [False] * n), 'required': '',
            'logits': list(logits if logits is not None else [LOG(x) for x in p]),
            'tau': 0.5, 'kappa': 0.7, 'noise': [0.0] * n, 'group_noise': group_noise,
            'selected': selected, 'propensity': propensity, 'irreversible_path': False, **kw}


def rec(did, phase, t, questions=None, agent='a1', qs='s', node_t=None, **kw):
    node_t = t if node_t is None else node_t
    return {'header': {'stamp': {'sec': int(node_t), 'nanosec': int(node_t % 1 * 1e9)},
                       'frame_id': ''},
            'wall_stamp': {'sec': int(1000 + t), 'nanosec': int(t % 1 * 1e9)},
            'run_id': 'r', 'agent': agent, 'group': 'g', 'question_set': qs,
            'decision_id': did, 'generation': did, 'phase': phase, 'triggers': [],
            'subject': agent, 'state': [], 'questions': questions or [q()], 'llm_params': '{}',
            'llm_rationale': '', 'llm_flags': [], 'catalog_hash': '', 'policy_hash': '',
            'jev_model': '', 'llm_model': '', 'jev_latency_ms': 0.0, 'llm_latency_ms': 0.0,
            'jev_input_tokens': 0, 'llm_input_tokens': 0, 'llm_output_tokens': 0,
            'cost_usd': 0.0, **kw}


PHASE_PATHS = {1: ['PROVISIONAL', 'KEPT'], 2: ['PROVISIONAL', 'REVISED', 'COMMITTED'],
               3: ['PROVISIONAL', 'KEPT'], 4: ['PROVISIONAL', 'COMMITTED'], 5: ['COMMITTED'],
               6: ['FORCED'], 7: ['PROVISIONAL', 'FALLBACK'], 8: ['PROVISIONAL']}


def phase_records():
    return [rec(did, ph, did * 10 + i) for did, path in PHASE_PATHS.items()
            for i, ph in enumerate(path)]


def test_outcomes_and_rates():
    ph = metrics.compute(phase_records())['overall']['phases']
    assert ph['outcomes'] == {'COMMITTED': 1, 'KEPT': 2, 'REVISED': 1, 'LLM_MISSING': 1,
                              'FALLBACK': 1, 'FORCED': 1, 'OPEN': 1}
    assert ph['revision_rate'] == pytest.approx(1 / 3)
    assert ph['fallback_rate'] == ph['forced_rate'] == ph['llm_missing_rate'] == 1 / 8
    assert ph['degraded_rate'] == 2 / 8
    assert ph['final_phase']['COMMITTED'] == 3
    assert sum(sum(v) for v in ph['over_time']['series'].values()) == 8


def test_volume_and_records_order_does_not_matter():
    rs = phase_records()
    a = metrics.compute(rs)['overall']
    b = metrics.compute(list(reversed(rs)))['overall']
    assert a['phases']['outcomes'] == b['phases']['outcomes']
    assert a['volume']['decisions'] == 8 and a['volume']['records'] == len(rs)
    assert a['volume']['decisions_per_min'] == pytest.approx(8 / ((80 - 10) / 60))


def test_agreement_masking_delta_and_flip():
    agree_masked = q(classes=('x', 'y', 'z'), p=(0.1, 0.5, 0.4), ql=(0.1, 0.2, 0.7),
                     masked=[False, True, False], logits=[LOG(.1), None, LOG(.4)])
    disagree = q(p=(0.7, 0.3), ql=(0.2, 0.8))
    flipped = q(p=(0.6, 0.4), ql=(0.55, 0.45), logits=[LOG(.6) - 1, LOG(.4) + 1], beta=0.8)
    no_llm = q()
    rs = [rec(1, 'KEPT', 1, [agree_masked]), rec(2, 'KEPT', 2, [disagree]),
          rec(3, 'KEPT', 3, [flipped]), rec(4, 'COMMITTED', 4, [no_llm])]
    o = metrics.compute(rs)['overall']
    aug = o['augmentation']
    assert aug['agreement'] == pytest.approx(2 / 3)          # masked class ignored
    assert aug['llm_share'] == 3 / 4
    assert aug['argmax_flip_rate'] == pytest.approx(1 / 3)
    assert aug['mean_abs_delta'] == pytest.approx(1 / 3)     # 0, 0, 1
    assert sum(aug['beta']['counts']) == 3
    assert o['jev']['first_option_rate'] == pytest.approx(3 / 4)
    assert o['by_question']['a']['selection'] == {'x': 4, 'y': 0, 'z': 0}


def test_reliability_ece_nll_toy():
    r = metrics.reliability([([0.8, 0.2], 0), ([0.8, 0.2], 1)])
    assert r['ece'] == pytest.approx(0.3)
    assert r['nll'] == pytest.approx((-LOG(0.8) - LOG(0.2)) / 2)
    b = [x for x in r['bins'] if x['n']]
    assert len(b) == 1 and b[0]['acc'] == 0.5 and b[0]['conf'] == pytest.approx(0.8)
    assert metrics.reliability([([1.0, 0.0], 0)])['ece'] == 0.0


def test_calibration_joins_labels():
    rs = [rec(1, 'KEPT', 1, [q(p=(0.8, 0.2), ql=(0.9, 0.1))]),
          rec(2, 'COMMITTED', 2, [q(p=(0.8, 0.2))])]
    labels = [{'run_id': 'r', 'subject': 'a1', 'question': 'a', 'true_class': 'y',
               'decision_id': 1, 'source': 't'},
              {'run_id': 'r', 'subject': 'a1', 'question': 'a', 'true_class': 'x',
               'decision_id': 2, 'source': 't'}]
    m = metrics.compute(rs, labels)
    cal = m['overall']['calibration']['a']
    assert (cal['jev']['n'], cal['llm']['n'], cal['fused']['n']) == (2, 1, 2)
    assert cal['jev']['ece'] == pytest.approx(0.3)
    assert cal['llm']['nll'] == pytest.approx(-LOG(0.1))
    assert metrics.compute(rs)['overall']['calibration'] is None


def test_softmax_handles_masked_null_and_inf():
    assert metrics.softmax([0.0, None]) == [1.0, 0.0]
    assert metrics.softmax([0.0, float('-inf'), 0.0]) == [0.5, 0.0, 0.5]


def test_percentiles_nearest_rank():
    p = metrics.percentiles([float(x) for x in range(1, 101)])
    assert (p['p50'], p['p95'], p['p99'], p['n']) == (50, 95, 99, 100)
    assert metrics.percentiles([])['p50'] is None


def test_coupling_split_by_group_noise():
    def at(agent, sel, gn, did, t):
        return rec(did, 'COMMITTED', t, [q(selected=sel, group_noise=gn)], agent=agent)
    rs = [at('a1', 'x', True, 1, 1.0), at('a2', 'x', True, 1, 2.0),      # epoch 0: shared
          at('a1', 'x', False, 2, 11.0), at('a2', 'y', True, 2, 12.0)]   # epoch 2: own noise
    c = metrics.compute(rs)['overall']['coupling']
    assert (c['agreement_shared'], c['pairs_shared']) == (1.0, 1)
    assert (c['agreement_independent'], c['pairs_independent']) == (0.0, 1)
    assert c['agreement'] == 0.5


def test_switch_rate_and_cost_counted_once():
    rs = [rec(1, 'PROVISIONAL', 0, [q(selected='x')]),
          rec(1, 'REVISED', 1, [q(selected='y')], cost_usd=0.01, llm_latency_ms=900.0),
          rec(1, 'COMMITTED', 1, [q(selected='y')], cost_usd=0.01, llm_latency_ms=900.0),
          rec(2, 'COMMITTED', 60, [q(selected='y')])]
    o = metrics.compute(rs)['overall']
    assert o['behaviour']['switches'] == 1
    assert o['behaviour']['switch_rate'] == pytest.approx(1.0)      # 1 switch in 1 agent-minute
    pf = o['performance']
    assert pf['cost_usd'] == pytest.approx(0.01) and pf['llm_latency_ms']['n'] == 1
    assert pf['cost_series'] == [[1001.0, pytest.approx(0.01)]]


def test_empty_is_safe():
    m = metrics.compute([])
    assert m['overall']['volume']['decisions'] == 0
    json.dumps(m, allow_nan=False)


# --- synthetic fleet -------------------------------------------------------------------------

@pytest.fixture(scope='module')
def demo():
    return sample.generate()


def test_sample_is_deterministic_and_covers_every_phase(demo):
    records, labels = demo
    again, _ = sample.generate()
    assert [dumps(r) for r in records] == [dumps(r) for r in again]
    assert {r['phase'] for r in records} == set(metrics.PHASES)
    assert labels and all(x['true_class'] for x in labels)


def test_metrics_on_sample(demo):
    records, labels = demo
    m = metrics.compute(records, labels)
    json.dumps(m, allow_nan=False)                     # JSON-safe for browsers
    o = m['overall']
    assert m['question_sets'] == ['next_step', 'route']
    assert sum(o['phases']['outcomes'].values()) == o['volume']['decisions']
    assert o['volume']['agents'] == 4
    assert 0 < o['augmentation']['agreement'] < 1
    assert o['augmentation']['mean_abs_delta'] <= math.log(20) + 1e-9   # |delta| <= B (§6.2)
    assert o['behaviour']['irreversible'] > 0
    assert m['by_question_set']['route']['augmentation']['llm_share'] == 0
    assert set(o['calibration']) == {'next_action', 'report_now', 'route'}
    assert all(k in m['help'] for k in ('phases.revision_rate', 'coupling.agreement'))
    # JSONL round trip (null for -inf / NaN) gives the same numbers
    back = [json.loads(dumps(r)) for r in records]
    m2 = metrics.compute(back, labels)
    assert m2['overall']['augmentation'] == o['augmentation']


def test_report_embeds_escaped_payload(demo, tmp_path):
    records, labels = demo
    evil = dict(records[-1], llm_rationale='</script><script>alert(1)</script>')
    data = report.payload(records + [evil], metrics.compute(records, labels))
    html = report.render(data)
    assert html.count('</script>') == report.render().count('</script>')
    assert '"mode":"static"' in html
    # every (agent, question set) card has its latest decision even beyond the recent window
    cards = {(r['agent'], r['question_set']) for r in records}
    assert {(r['agent'], r['question_set']) for r in report.recent(records, 5)} == cards
    out = tmp_path / 'r.html'
    p = tmp_path / 'd.jsonl'
    p.write_text(''.join(dumps(r) + '\n' for r in records))
    report.main([str(p), '-o', str(out)])
    assert 'id="jev-data">{' in out.read_text()
