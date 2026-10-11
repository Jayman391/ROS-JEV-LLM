"""In-process test of the dashboard node (A5): DecisionRecords and /diagnostics published over
ROS come back out of /api/state as JSON, with metrics; records_path preloads a JSONL log."""
import copy
import json
import threading
import time
import urllib.error
import urllib.request

import pytest
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from jev_ai_msgs.msg import DecisionRecord, Directive, Report
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py import set_message_fields

from jev_ai.core.records import dumps
from jev_ai.viz import sample
from jev_ai_servers.dashboard import Dashboard, _host_ok


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope='module')
def records():
    return sample.generate(agents=2, duration_s=60.0)[0]


def get(port, path):
    with urllib.request.urlopen(f'http://127.0.0.1:{port}{path}', timeout=5) as r:
        return r.headers.get('Content-Type'), r.read()


def wait_for(fn, timeout=10.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        v = fn()
        if v:
            return v
        time.sleep(0.1)
    raise AssertionError('timed out')


def start(ns, **params):
    node = Dashboard(namespace=ns, parameter_overrides=[
        Parameter(k, value=v) for k, v in {'host': '127.0.0.1', 'port': 0, **params}.items()])
    pub_node = rclpy.create_node('dashboard_test_pub', namespace=ns)
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(node)
    ex.add_node(pub_node)
    threading.Thread(target=ex.spin, daemon=True).start()
    return node, pub_node, ex


def stop(node, pub_node, ex):
    ex.shutdown()
    node.destroy_node()
    pub_node.destroy_node()


def test_live_records_and_diagnostics(records):
    node, pub_node, ex = start('/dash_live')
    try:
        pub = pub_node.create_publisher(
            DecisionRecord, 'decisions', QoSProfile(depth=1000,
                                                    reliability=ReliabilityPolicy.RELIABLE))
        diag = pub_node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        sent = records[:12]
        wait_for(lambda: pub.get_subscription_count() > 0)
        for r in sent:
            msg = DecisionRecord()
            # Humble's set_message_fields mutates nested lists in place
            set_message_fields(msg, copy.deepcopy(r))
            pub.publish(msg)
        diag.publish(DiagnosticArray(status=[DiagnosticStatus(
            level=DiagnosticStatus.WARN, name='/llm_server', message='circuit breaker half_open',
            values=[KeyValue(key='breaker', value='half_open'),
                    KeyValue(key='usd_last_hour', value='0.5')])]))

        def ready():
            s = json.loads(get(node.port, '/api/state')[1])
            return s if len(s['records']) == len(sent) and s['diagnostics'] else None
        state = wait_for(ready)
        assert state['mode'] == 'live'
        got = state['records'][0]
        assert (got['agent'], got['decision_id'], got['phase']) == \
            (sent[0]['agent'], sent[0]['decision_id'], sent[0]['phase'])
        body = get(node.port, '/api/state')[1]
        assert b'Infinity' not in body and b'NaN' not in body      # -inf logits -> null
        d = state['diagnostics']['/llm_server']
        assert (d['level'], d['values']['breaker']) == (1, 'half_open')
        wait_for(lambda: json.loads(get(node.port, '/api/state')[1])['metrics']['overall']
                 ['volume']['records'] == len(sent), timeout=5)
        ctype, page = get(node.port, '/')
        assert ctype.startswith('text/html') and b'id="jev-data">null<' in page
        with pytest.raises(urllib.error.HTTPError):
            get(node.port, '/nope')
    finally:
        stop(node, pub_node, ex)


def test_records_path_preload(records, tmp_path):
    p = tmp_path / 'decisions.jsonl'
    p.write_text(''.join(dumps(r) + '\n' for r in records))
    node, pub_node, ex = start('/dash_preload', records_path=str(p), max_records=len(records))
    try:
        state = json.loads(get(node.port, '/api/state')[1])
        m = state['metrics']['overall']
        assert m['volume']['records'] == len(records)
        assert m['volume']['agents'] == 2
        assert 0 < len(state['records']) <= len(records)
    finally:
        stop(node, pub_node, ex)


def test_hierarchy_traffic_is_tapped_across_a_relay():
    node, pub_node, ex = start('/dash_tr')
    try:
        sup, uav = '/dash_tr/coordinator', '/dash_tr/uav1'
        pubs = {t: pub_node.create_publisher(cls, t, 10) for t, cls in (
            (sup + '/directives', Directive), (uav + '/directives', Directive),
            (uav + '/reports', Report))}
        wait_for(lambda: all(p.get_subscription_count() > 0 for p in pubs.values()))
        order = Directive(issuer=sup, directive_id=7, target=uav, kind='survey',
                          payload='{"sector": 2}')

        def delivered():
            # best effort: the tap may not have matched yet, so resend (same keys, seen once)
            pubs[uav + '/reports'].publish(Report(agent=uav, kind='nominal', payload='{}'))
            pubs[sup + '/directives'].publish(order)
            pubs[uav + '/directives'].publish(order)             # the relay's copy
            t = json.loads(get(node.port, '/api/state')[1])['traffic']
            p = [p for p in t['pairs'] if (p['supervisor'], p['worker']) == (sup, uav)]
            return t if p and p[0]['down']['ok'] == 1 else None
        t = wait_for(delivered)
        m = next(m for m in t['messages'] if m['kind'] == 'survey')
        assert m['from'] == sup and m['to'][0][:2] == [uav, 'ok'] and '"sector"' in m['payload']
    finally:
        stop(node, pub_node, ex)


def test_host_header_guard():
    local = {'localhost', '127.0.0.1', '::1'}
    for ok in ('localhost:8080', '127.0.0.1', '[::1]:8080', 'LOCALHOST'):
        assert _host_ok(ok, local), ok
    for bad in ('evil.example:8080', 'localhost.evil.example', '', '10.0.0.5:8080'):
        assert not _host_ok(bad, local), bad          # DNS rebinding: attacker's name, our IP
    assert _host_ok('robot-7:8080', {'*'})
