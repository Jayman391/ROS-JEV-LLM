"""In-process integration tests for jev_server (FRAMEWORK §8, §9, §11): a real JevServer on a
MultiThreadedExecutor, called through ``jev/decide`` by a ROS client."""
import math
import threading
import time
from contextlib import contextmanager

import pytest
import rclpy
from diagnostic_msgs.msg import KeyValue
from jev_ai_msgs.msg import Question
from jev_ai_msgs.srv import JevDecide
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from jev_ai.core.fusion import confidence
from jev_ai_servers.jev_server import JevServer

CHOICE, SCORE, NOUL = Question.CHOICE, Question.SCORE, Question.NOUL


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def params(overrides: dict) -> list[Parameter]:
    return [Parameter(k, value=v) for k, v in overrides.items()]


@contextmanager
def server(ns: str, **overrides):
    node = JevServer(namespace=ns, parameter_overrides=params(
        {'backend': 'mock', 'mock.latency_s': 0.0, **overrides}))
    client_node = rclpy.create_node('jev_test_client', namespace=ns)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.add_node(client_node)
    threading.Thread(target=executor.spin, daemon=True).start()
    cli = client_node.create_client(JevDecide, 'jev/decide')
    try:
        assert cli.wait_for_service(timeout_sec=10)
        yield lambda req: call(cli, req)
    finally:
        executor.shutdown()
        node.loop.stop()
        node.destroy_node()
        client_node.destroy_node()


def call(cli, req, timeout=10.0):
    fut = cli.call_async(req)
    end = time.monotonic() + timeout
    while not fut.done():
        assert time.monotonic() < end, 'jev/decide timed out'
        time.sleep(0.01)
    return fut.result()


def q(name, qtype, classes, criteria=(), instructions='?'):
    return Question(name=name, type=qtype, instructions=instructions, classes=list(classes),
                    criteria=list(criteria))


def request(*questions, state=(('task', 'patrolling'), ('battery', 'low (18%)')), model=''):
    return JevDecide.Request(state=[KeyValue(key=k, value=v) for k, v in state],
                             questions=list(questions), model=model)


GOOD = [q('next_action', CHOICE, ['continue', 'inspect', 'return']),
        q('urgency', SCORE, ['none', 'low', 'high']),
        q('report_now', NOUL, ['true', 'false'])]


def summary(res) -> str:
    """Comparable form of a response (repr, because NaN != NaN)."""
    return repr((res.success, res.error, res.model, res.input_tokens, list(res.question_errors),
                 [(d.question, d.type, list(d.classes), list(d.probabilities), d.confidence,
                   d.source_confidence) for d in res.answers]))


def test_decide_with_mock():
    with server('t_mock') as decide:
        res = decide(request(*GOOD))
        assert res.success and res.error == '' and res.model == 'mock'
        assert res.input_tokens > 0 and res.latency_ms > 0
        assert list(res.question_errors) == ['', '', '']
        for d, g in zip(res.answers, GOOD):
            assert (d.question, d.type, list(d.classes)) == (g.name, g.type, list(g.classes))
            assert math.isclose(sum(d.probabilities), 1.0)
            assert d.confidence == pytest.approx(confidence(list(d.probabilities)))
            assert d.source_confidence == pytest.approx(max(d.probabilities))
        assert summary(decide(request(*GOOD))) == summary(res)        # deterministic
        empty = decide(request())
        assert not empty.success and empty.error == 'no questions'


def test_per_question_fault_isolation():
    bad = [q('one_class', CHOICE, ['only']),
           q('yes_no', NOUL, ['yes', 'no']),
           q('misaligned', CHOICE, ['a', 'b'], criteria=['x']),
           q('next_action', CHOICE, ['a', 'b']),                       # duplicate name
           q('bad_type', 7, ['a', 'b']),
           q('silent', NOUL, ['true', 'false'], instructions='')]
    with server('t_faults') as decide:
        res = decide(request(GOOD[0], *bad, GOOD[2]))
        assert res.success and len(res.answers) == len(res.question_errors) == 8
        errors = list(res.question_errors)
        assert errors[0] == errors[7] == ''
        assert all(errors[1:7]), errors
        assert not res.answers[1].probabilities and math.isnan(res.answers[1].confidence)
        assert math.isclose(sum(res.answers[7].probabilities), 1.0)
        only_bad = decide(request(*(b for b in bad if b.name != 'next_action')))
        assert not only_bad.success and only_bad.error == 'no valid questions'


def test_backend_question_failures_do_not_fail_the_call():
    with server('t_fail', **{'mock.fail_rate': 1.0}) as decide:
        res = decide(request(*GOOD))
        assert res.success
        assert list(res.question_errors) == ['mock: injected failure'] * 3


def test_budget_refusal():
    with server('t_budget', max_rps=0.5) as decide:
        assert decide(request(*GOOD)).success
        refused = decide(request(*GOOD))
        assert not refused.success and refused.error == 'budget: max_rps'


def test_typesafe_without_key_fails_configure(monkeypatch):
    monkeypatch.delenv('TYPESAFE_API_KEY', raising=False)
    node = JevServer(namespace='t_nokey', parameter_overrides=params({'autostart': False}))
    try:
        assert node.p('backend') == 'typesafe'           # live by default; mock only when asked
        assert node.trigger_configure() == TransitionCallbackReturn.FAILURE
        assert 'TYPESAFE_API_KEY' in node.last_error and node.srv is None
    finally:
        node.loop.stop()
        node.destroy_node()


def test_record_then_replay(tmp_path):
    cassette = str(tmp_path / 'jev.jsonl')
    req = request(*GOOD, q('one_class', CHOICE, ['only']), model='jev-test')
    with server('t_record', record_path=cassette, **{'mock.seed': 3}) as decide:
        recorded = decide(req)
    assert recorded.success and recorded.model == 'mock'
    with server('t_replay', backend='replay', **{'replay.path': cassette}) as decide:
        assert summary(decide(req)) == summary(recorded)
        miss = decide(request(*GOOD, state=[('task', 'idle')], model='jev-test'))
        assert not miss.success and miss.error.startswith('replay miss')
