"""Pure checks of the assembly line example (no ROS graph): scenario and routing, catalogs, the
encoders feeding the keyless mock (every class of every question must win somewhere), scoring."""
from pathlib import Path
from types import SimpleNamespace as NS

import yaml

from assembly_line import mock_rules, scenario
from assembly_line.arm_agent import encode_coop, encode_op_check, routine_ok
from assembly_line.qa_agent import encode_inspect
from assembly_line.scorer import score, true_disposition
from assembly_line.supervisor import encode_disruption, encode_quality, encode_release, schedule
from jev_ai.core.catalog import load_catalog

CONFIG = Path(__file__).resolve().parents[1] / 'config'
NAN = float('nan')


def params(**kw):
    return scenario.load(CONFIG / 'scenario.yaml', **kw)


# --- scenario and routing ------------------------------------------------------------------------

def test_every_solo_op_has_two_arms_and_fasten_one_pair():
    p = params()
    for kind in p['op_kinds']:
        n = len(scenario.capable(p, kind))
        assert n == 1 if kind == 'fasten' else n >= 2, kind
    assert scenario.capable(p, 'fasten') == [(3, 'arm_d+arm_c')]


def test_routes_with_every_arm_up_are_one_pass():
    p = params()
    for pr, kinds in scenario.recipes(p).items():
        entries, laps = scenario.route(p, kinds)
        assert laps == 0 and '' not in entries, (pr, entries)
        stations = [scenario.arm(p, e.split('+')[0])['station'] for e in entries]
        assert stations == sorted(stations)


def test_routes_around_a_missing_arm():
    p = params()
    a, laps = scenario.route(p, scenario.recipes(p)['A'], exclude={'arm_b'})
    assert 'arm_b' not in a and '' not in a and laps == 0 and a[2] == 'arm_e'   # weld -> S4
    assert scenario.describe_route(p, 'C', {'arm_b'}) == 'needs 1 lap'   # weld moves to S4
    assert scenario.describe_route(p, 'B', {'arm_c'}).startswith('blocked')  # fasten: no backup
    entries, _ = scenario.route(p, ['screw', 'weld'], at=3)            # already at S3
    assert entries == ['arm_d', 'arm_e']


def test_load_spreads_work():
    p = params()
    busy, _ = scenario.route(p, ['pick'], load={'arm_a': 5})
    assert busy != ['arm_a']


def test_orders_are_due_within_the_shift():
    p = params()
    due = scenario.orders(p)
    assert [len(due[pr]) for pr in p['products']] == p['order_counts']
    assert all(0 < t <= p['t_max_s'] for ts in due.values() for t in ts)


def test_catalogs_load_and_match_the_mock():
    for name in ('supervisor', 'arm', 'qa'):
        cat = load_catalog(CONFIG / f'{name}.yaml')
        for qs in cat.question_sets.values():
            for q in qs.questions.values():
                assert q.class_names == mock_rules.CLASSES[q.name], q.name
                assert q.policy.T_dwell < 0.1          # per-item decisions must not pin the next
    qa = load_catalog(CONFIG / 'qa.yaml').question_sets['inspect'].questions['disposition']
    assert qa.irreversible() == {'scrap'} and qa.fallback == 'quarantine'


# --- encoders -> mock: every class wins somewhere ------------------------------------------------

def arm_status(**kw):
    base = dict(arm='arm_b', station=2, state='idle', part_id=7, op_index=1, op_kind='screw',
                role='solo', partner='', partner_state='', coop_wait_s=0.0, grip=NAN,
                partner_grip=NAN, pose_mm=0.6, wear_est=0.2, current_a=1.0, dropout=False,
                jam_clear_est_s=0.0, regrips=0, queue=1, ops_done=10, ops_since_change=10,
                jams=0)
    return NS(**{**base, **kw})


def top(question: str, state: dict) -> str:
    p = mock_rules.line(state, NS(name=question))
    return max(p, key=p.get)


def test_mock_covers_every_class():
    p = params()
    part = NS(n_ops=5, product='B')
    alts = ['arm_e']
    op = {
        'proceed': encode_op_check(arm_status(dropout=True, pose_mm=NAN, wear_est=NAN), part, p,
                                   '', []),
        'retry': encode_op_check(arm_status(op_kind='pick', grip=0.6), part, p, '', alts),
        'tool_change': encode_op_check(arm_status(wear_est=0.9), part, p, '', alts),
        'hand_off': encode_op_check(arm_status(op_kind='pick', grip=0.3, regrips=2), part, p,
                                    '', alts),
    }
    coop = {
        'wait': encode_coop(arm_status(state='coop_wait', coop_wait_s=2, role='primary',
                                       partner='arm_c', partner_state='idle'), p),
        'abort_step': encode_coop(arm_status(state='coop_wait', coop_wait_s=30, role='primary',
                                             partner='arm_c', partner_state='regrip'), p),
    }

    def qa(readings, reworks):
        m = NS(part_id=3, product='A', reworks=reworks, op_kinds=['pick', 'screw', 'weld', 'glue'],
               op_arms=['arm_a', 'arm_a', 'arm_b', 'arm_d'], readings=readings)
        return encode_inspect(m, p, {'arm_b': [True, False]}, 'on schedule')
    disp = {'pass': qa([0.9, 0.9, 0.76, 0.9], 0), 'rework': qa([0.9, 0.9, 0.45, 0.9], 0),
            'quarantine': qa([0.9, 0.9, 0.45, 0.9], 2), 'scrap': qa([0.9, 0.3, 0.45, 0.9], 2)}

    def rel(urgent: str, wip: int):
        sched = schedule(p, p['products'], {}, {}, 0.0)
        for o in sched:
            o['urgency'] = 'late' if o['product'] == urgent else 'on time'
        line = f'WIP {wip} of cap 6; no full buffers; every arm healthy'
        return encode_release(p, sched, {pr: 'one pass' for pr in p['products']}, line, 100.0)
    release = {'hold': rel('', 5), 'A': rel('A', 1), 'B': rel('B', 1), 'C': rel('C', 1)}

    def dis(kind, est, count, alternatives):
        ev = {'arm': 'arm_b', 'kind': kind, 't': 0.0, 'count': count, 'est_clear_s': est,
              'station': 2}
        return encode_disruption(p, ev, None, alternatives, 'WIP 4 of cap 6', 'on schedule',
                                 10.0)
    response = {'wait_recover': dis('jam', 10, 1, 'weld -> arm_e (S4)'),
                'reroute': dis('jam', 80, 1, 'weld -> arm_e (S4)'),
                'slow_line': dis('jam', 80, 1, 'fasten -> none'),
                'pause_station': dis('jam', 80, 3, 'weld -> arm_e (S4)')}

    def qw(flags, wear):
        rec = [('weld', i < flags) for i in range(10)]
        return encode_quality(p, 'arm_b', {'kind': 'weld', 'reading': 0.6, 'part': 4}, rec, 0.1,
                              arm_status(wear_est=wear), 300.0)
    escalate = {'monitor': qw(1, 0.2), 'inspect_tool': qw(3, 0.4),
                'order_tool_change': qw(5, 0.7)}

    for question, cases in (('action', op), ('coop_action', coop), ('disposition', disp),
                            ('next_product', release), ('response', response),
                            ('escalate', escalate)):
        assert set(cases) == set(mock_rules.CLASSES[question]), question
        for want, state in cases.items():
            assert top(question, state) == want, (question, want, state)


def test_routine_path_skips_the_model():
    p = params()
    assert routine_ok(arm_status(), p, '')
    assert routine_ok(arm_status(op_kind='pick', grip=0.92), p, '')
    assert not routine_ok(arm_status(op_kind='pick', grip=0.6), p, '')
    assert not routine_ok(arm_status(pose_mm=4.5), p, '')
    assert not routine_ok(arm_status(), p, 'QA flagged its weld')
    assert not routine_ok(arm_status(dropout=True), p, '')


def test_report_payload_may_carry_kind():
    """Smoke-run regression: report('fault', kind='jam') crashed the arm agent (two kinds)."""
    from assembly_line.arm_agent import ArmAgent
    sent = []
    me = NS(status=arm_status(), name='arm_b',
            hierarchy=NS(report=lambda what, **kw: sent.append((what, kw))))
    ArmAgent.report(me, 'fault', priority=0.8, kind='jam', est_clear_s=40)
    ArmAgent.report(me, 'hand_off', priority=0.6, part=3, op=1, kind='weld')
    assert [w for w, _ in sent] == ['fault', 'hand_off']
    assert sent[0][1]['payload']['kind'] == 'jam' and sent[1][1]['payload']['kind'] == 'weld'


# --- scoring -------------------------------------------------------------------------------------

def outcome(pid, outcome, defective, done, product='A', reworks=0):
    return NS(part_id=pid, product=product, outcome=outcome, defective=defective, done_s=done,
              reworks=reworks)


def test_score():
    p = params()
    due = scenario.orders(p)['A']
    s = score(p, [outcome(1, 'shipped', False, due[0] - 1), outcome(2, 'shipped', False, 1e6),
                  outcome(3, 'shipped', True, 10), outcome(4, 'scrapped', False, 10),
                  outcome(5, 'scrapped', True, 10), outcome(6, 'quarantined', True, 10)], 0.5)
    assert (s['good_shipped'], s['good_on_time'], s['escaped_defects']) == (2, 1, 1)
    assert s['J'] == 10 * 2 + 3 * 1 - 40 - 10 - 3 - 5 - 10 * 0.5


def test_labels():
    assert true_disposition(outcome(1, 'shipped', False, 0), 2) == 'pass'
    assert true_disposition(outcome(1, 'shipped', True, 0, reworks=1), 2) == 'rework'
    assert true_disposition(outcome(1, 'scrapped', True, 0, reworks=2), 2) == 'scrap'


def test_scenario_file_is_ros_params():
    raw = yaml.safe_load((CONFIG / 'scenario.yaml').read_text())
    assert set(raw) == {'/**'} and 'ros__parameters' in raw['/**']
