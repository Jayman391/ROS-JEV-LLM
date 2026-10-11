"""viz.traffic: a copy on a relay's far side is a delivery, a missing one a drop after GRACE_S."""
from jev_ai.viz.traffic import GRACE_S, Traffic


def test_relayed_traffic_delivery_drop_and_broadcast():
    tr = Traffic()
    # broadcast: the relay copies it to uav1 (100 ms later) but not to uav2
    tr.observe('down', '/coordinator/directives', ('D', '/coordinator', 1), sender='/coordinator',
               target='*', kind='phase', stamp=10.0, wall=0.0)
    tr.observe('down', '/uav1/directives', ('D', '/coordinator', 1), sender='/coordinator',
               target='*', kind='phase', stamp=10.0, wall=0.1)
    tr.observe('down', '/uav2/directives', ('D', '/coordinator', 0), sender='/coordinator',
               target='/uav2', kind='survey', stamp=5.0, wall=-5.0)      # uav2 is an endpoint
    # a report up from uav2, delivered; one from uav1, never relayed
    for topic in ('/uav2/reports', '/coordinator/reports'):
        tr.observe('up', topic, ('R', '/uav2', 11, 0, 'nominal'), sender='/uav2',
                   kind='nominal', stamp=11.0, wall=1.0)
    tr.observe('up', '/uav1/reports', ('R', '/uav1', 12, 0, 'nominal'), sender='/uav1',
               kind='nominal', stamp=12.0, wall=2.0)

    m = tr.msgs[('D', '/coordinator', 1)]
    assert tr.links(m, now=1.0) == [('/uav1', 'ok', 100.0), ('/uav2', 'pending', None)]
    assert tr.links(m, now=GRACE_S + 0.1)[1] == ('/uav2', 'lost', None)
    assert tr.links(tr.msgs[('R', '/uav1', 12, 0, 'nominal')], now=GRACE_S + 2.1) == \
        [('/coordinator', 'lost', None)]

    snap = tr.snapshot(now=10.0)
    pairs = {p['worker']: p for p in snap['pairs']}
    assert set(pairs) == {'/uav1', '/uav2'}
    assert pairs['/uav1']['down'] == {'sent': 1, 'ok': 1, 'lost': 0, 'pending': 0}
    assert pairs['/uav1']['up']['lost'] == 1
    assert pairs['/uav2']['down'] == {'sent': 2, 'ok': 1, 'lost': 1, 'pending': 0}
    assert pairs['/uav2']['up']['ok'] == 1 and pairs['/uav2']['latency_ms_p50'] == 0.0
    assert sum(pairs['/uav2']['lanes']['down_lost']) == 1
    assert snap['messages'][0]['kind'] == 'nominal' and len(snap['messages']) == 4


def test_unrelayed_traffic_is_just_sent():
    tr = Traffic()
    tr.observe('down', '/directives', ('D', '/sup', 1), sender='/sup', target='*', kind='go',
               stamp=1.0, wall=0.0)
    tr.observe('up', '/reports', ('R', '/bot1', 2, 0, 'status'), sender='/bot1', kind='status',
               stamp=2.0, wall=0.5)
    assert tr.links(tr.msgs[('D', '/sup', 1)], now=10.0) == []    # '/' is no agent: no relay
    snap = tr.snapshot(now=10.0)
    assert [(p['supervisor'], p['worker'], p['down']['sent'], p['up']['sent'], p['down']['ok'])
            for p in snap['pairs']] == [('*', '/bot1', 0, 1, 0), ('/sup', '*', 1, 0, 0)]
