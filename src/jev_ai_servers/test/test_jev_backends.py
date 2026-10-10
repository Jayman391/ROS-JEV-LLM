"""Unit tests for the built-in Jev backends (FRAMEWORK §8). No ROS graph, no network."""
import asyncio
import json
import math
import sys
import types

import httpx2
import pytest
from rclpy.task import Future
from typesafe_sdk import AsyncTypeSafeClient, RetryPolicy

from jev_ai.backends import BackendError, JevQuestion, JevResult
from jev_ai_servers.common import cassette_append
from jev_ai_servers.jev_backends.llama_ros import LlamaRosJev
from jev_ai_servers.jev_backends.mock import MockJev
from jev_ai_servers.jev_backends.replay import ReplayJev, key, to_record
from jev_ai_servers.jev_backends.typesafe_backend import TypeSafeJev

STATE = [('task', 'patrolling'), ('battery', 'low (18%)')]
CHOICE_Q = JevQuestion('next_action', 1, 'Choose the next action.',
                       ['continue', 'inspect', 'return'], ['', 'Look closer.', ''])
SCORE_Q = JevQuestion('urgency', 2, 'How urgent?', ['none', 'low', 'high'], [])
NOUL_Q = JevQuestion('report_now', 0, 'Report now?', ['true', 'false'], ['Something changed.', ''])
QS = [CHOICE_Q, SCORE_Q, NOUL_Q]


def decide(backend, qs=QS, state=STATE, model='') -> JevResult:
    return asyncio.run(backend.decide(state, qs, model))


def mock(**params) -> MockJev:
    return MockJev({'latency_s': 0.0, **params})


# --- mock ----------------------------------------------------------------------------------------

def test_mock_is_deterministic_and_valid():
    r = decide(mock())
    assert r == decide(mock())
    assert r.model == 'mock' and r.input_tokens > 0 and r.question_errors == ['', '', '']
    for a, q in zip(r.answers, QS):
        assert len(a.probabilities) == len(q.classes)
        assert math.isclose(sum(a.probabilities), 1.0)
        assert a.source_confidence == max(a.probabilities)
    assert decide(mock(seed=1)).answers != r.answers
    assert decide(mock(), state=[('task', 'idle')]).answers != r.answers


def test_mock_alpha_and_first_bias():
    flat, peaked = decide(mock(alpha=100.0)), decide(mock(alpha=0.05))
    assert max(flat.answers[0].probabilities) < max(peaked.answers[0].probabilities)
    p = decide(mock()).answers[0].probabilities
    pb = decide(mock(first_bias=3.0)).answers[0].probabilities
    assert math.isclose(pb[0] / pb[1], 4 * p[0] / p[1])
    with pytest.raises(BackendError):
        mock(alpha=0.0)


def test_mock_rules_hook(monkeypatch):
    def rule(state, q):
        if q.name == 'next_action' and state['battery'].startswith('low'):
            return [0.0, 0.0, 1.0]
        return None
    monkeypatch.setitem(sys.modules, 'fake_rules', types.SimpleNamespace(rule=rule))
    r = decide(mock(rules='fake_rules:rule'))
    assert r.answers[0].probabilities == [0.0, 0.0, 1.0]
    assert r.answers[1:] == decide(mock()).answers[1:]          # None → Dirichlet
    with pytest.raises(BackendError, match='mock.rules'):
        mock(rules='fake_rules:missing')


def test_mock_fail_rate():
    assert decide(mock(fail_rate=1.0)).answers == [None, None, None]
    many = [JevQuestion(f'q{i}', 0, '?', ['true', 'false'], []) for i in range(200)]
    r = decide(mock(fail_rate=0.5), qs=many)
    failed = sum(a is None for a in r.answers)
    assert 60 < failed < 140 and r == decide(mock(fail_rate=0.5), qs=many)
    assert all(e == 'mock: injected failure' for a, e in zip(r.answers, r.question_errors)
               if a is None)


# --- replay --------------------------------------------------------------------------------------

def test_replay_hit_and_miss(tmp_path):
    path = str(tmp_path / 'jev.jsonl')
    recorded = decide(mock(fail_rate=0.3, seed=7))
    cassette_append(path, key(STATE, QS, 'jev-1.13.0'), to_record(recorded))
    b = ReplayJev({'path': path})
    assert decide(b, model='jev-1.13.0') == recorded
    with pytest.raises(BackendError, match='replay miss') as e:
        decide(b, model='jev-latest')
    assert not e.value.retryable
    with pytest.raises(BackendError, match='replay.path'):
        ReplayJev({'path': str(tmp_path / 'missing.jsonl')})


# --- typesafe (real SDK, mocked HTTP) ------------------------------------------------------------

ANSWERS = {
    'next_action': {'type': 'choice', 'choice': 'inspect', 'confidence': 0.7,
                    'probabilities': {'return': 0.1, 'inspect': 0.7, 'continue': 0.2}},
    'urgency': {'type': 'score', 'score': 1.2, 'confidence': 0.5,
                'legend': {'0': 'none', '1': 'low', '2': 'high'},
                'probabilities': {'0': 0.2, '1': 0.4, '2': 0.4}},
    'report_now': {'type': 'noul', 'noul': 0.8},
}


def typesafe(handler, **params) -> TypeSafeJev:
    b = TypeSafeJev(params, 'test-key')
    b.client = AsyncTypeSafeClient(api_key='test-key', retry=RetryPolicy(max_retries=0),
                                   transport=httpx2.MockTransport(handler))
    return b


def test_typesafe_request_and_answer_mapping():
    seen = {}

    def handler(request):
        seen['url'], seen['auth'] = str(request.url), request.headers['authorization']
        seen['body'] = json.loads(request.content)
        return httpx2.Response(200, json={'model': 'jev-1.13.0', 'answers': ANSWERS,
                                          'usage': {'input_tokens': 120, 'output_tokens': 3}})

    r = decide(typesafe(handler), model='jev-1.13.0')
    assert seen['url'] == 'https://api.typesafe.ai/v1/systemone'
    assert seen['auth'] == 'Bearer test-key'
    body = seen['body']
    assert body['state'] == {'task': 'patrolling', 'battery': 'low (18%)'}
    assert body['model'] == 'jev-1.13.0'
    assert body['questions'] == {
        'next_action': {'type': 'choice', 'instructions': 'Choose the next action.',
                        'criteria': {'continue': None, 'inspect': 'Look closer.',
                                     'return': None}},
        'urgency': {'type': 'score', 'instructions': 'How urgent?',
                    'criteria': ['none', 'low', 'high']},
        'report_now': {'type': 'noul', 'instructions': 'Report now?',
                       'criteria': {'true': 'Something changed.'}},
    }
    assert r.model == 'jev-1.13.0' and r.input_tokens == 120
    assert r.answers[0].probabilities == [0.2, 0.7, 0.1] and r.answers[0].source_confidence == 0.7
    assert r.answers[1].probabilities == [0.2, 0.4, 0.4]
    assert r.answers[2].probabilities == pytest.approx([0.8, 0.2])
    assert math.isnan(r.answers[2].source_confidence)


def test_typesafe_missing_answer_is_a_question_error():
    answers = {k: v for k, v in ANSWERS.items() if k != 'urgency'}

    def handler(request):
        return httpx2.Response(200, json={'model': 'jev-1.13.0', 'answers': answers,
                                          'usage': {'input_tokens': 1, 'output_tokens': 0}})

    r = decide(typesafe(handler))
    assert r.answers[1] is None and 'urgency' in r.question_errors[1]
    assert r.answers[0] is not None and r.question_errors[0] == ''


@pytest.mark.parametrize('status,retryable', [(429, True), (503, True), (401, False),
                                              (422, False)])
def test_typesafe_http_error_mapping(status, retryable):
    b = typesafe(lambda request: httpx2.Response(status, json={'error': 'nope'}))
    with pytest.raises(BackendError, match='nope') as e:
        decide(b)
    assert e.value.retryable is retryable


def test_typesafe_network_error_is_retryable():
    def handler(request):
        raise httpx2.ConnectError('unreachable')
    with pytest.raises(BackendError) as e:
        decide(typesafe(handler))
    assert e.value.retryable


def test_typesafe_check():
    def models(*names):
        return lambda request: httpx2.Response(200, json={'models': [
            {'name': n, 'description': '', 'release_date': '2026-09-15'} for n in names]})

    asyncio.run(typesafe(models('jev-latest', 'jev-1.13.0'), model='jev-1.13.0').check())
    asyncio.run(typesafe(models('jev-latest')).check())       # default model
    with pytest.raises(BackendError, match='not offered; available: jev-latest'):
        asyncio.run(typesafe(models('jev-latest'), model='jev-9').check())
    with pytest.raises(BackendError) as e:
        asyncio.run(typesafe(lambda r: httpx2.Response(401, json={'error': 'bad key'})).check())
    assert not e.value.retryable
    # Not TypeSafe's /v1/models shape (e.g. OpenRouter): model unverifiable, not a failure.
    asyncio.run(typesafe(lambda r: httpx2.Response(200, json={'data': []})).check())


def test_typesafe_rejects_malformed_key():
    with pytest.raises(BackendError):
        TypeSafeJev({}, 'has whitespace')


# --- llama_ros (fake llama_msgs) ----------------------------------------------------------------

class FakeQuestion(types.SimpleNamespace):
    CHOICE, SCORE, NOUL = 0, 1, 2


class FakeEvaluate:
    Request = types.SimpleNamespace


class FakeClient:
    def __init__(self, handler, ready=True):
        self.handler, self.ready, self.requests = handler, ready, []

    def wait_for_service(self, timeout_sec):
        return self.ready

    def call_async(self, req):
        self.requests.append(req)
        fut = Future()
        fut.set_result(types.SimpleNamespace(answers=self.handler(req)))
        return fut


class FakeNode:
    cb_group = None

    def __init__(self, client):
        self.client = client

    def create_client(self, srv, name, callback_group=None):
        self.service = name
        return self.client


@pytest.fixture
def llama_msgs(monkeypatch):
    msg, srv = types.ModuleType('llama_msgs.msg'), types.ModuleType('llama_msgs.srv')
    msg.DecisionQuestion, srv.EvaluateDecisions = FakeQuestion, FakeEvaluate
    monkeypatch.setitem(sys.modules, 'llama_msgs', types.ModuleType('llama_msgs'))
    monkeypatch.setitem(sys.modules, 'llama_msgs.msg', msg)
    monkeypatch.setitem(sys.modules, 'llama_msgs.srv', srv)


def answer(**kw):
    return types.SimpleNamespace(**{'success': True, 'error': '', 'noul': 0.0, 'confidence': 0.0,
                                    'keys': [], 'probabilities': [], **kw})


def llama(client) -> LlamaRosJev:
    b = LlamaRosJev({})
    b.node = FakeNode(client)
    asyncio.run(b.check())
    return b


def test_llama_ros_mapping(llama_msgs):
    client = FakeClient(lambda req: [
        answer(keys=['return', 'continue', 'inspect'], probabilities=[0.1, 0.2, 0.7],
               confidence=0.6),
        answer(success=False, error='decode failed'),
        answer(noul=0.25),
    ])
    b = llama(client)
    assert b.node.service == 'llama/evaluate_decisions'
    r = decide(b)
    req = client.requests[0]
    assert json.loads(req.state) == dict(STATE)
    choice, score, noul = req.questions
    assert (choice.type, choice.keys, choice.descriptions) == (0, CHOICE_Q.classes,
                                                                CHOICE_Q.criteria)
    assert (score.type, score.descriptions) == (1, ['none', 'low', 'high'])
    assert (noul.type, noul.descriptions) == (2, ['', 'Something changed.'])    # [false, true]
    assert r.answers[0].probabilities == pytest.approx([0.2, 0.7, 0.1])
    assert r.answers[1] is None and r.question_errors[1] == 'llama_ros: decode failed'
    assert r.answers[2].probabilities == pytest.approx([0.25, 0.75])
    assert math.isnan(r.answers[2].source_confidence)
    assert r.model == 'llama_ros'


def test_llama_ros_score_and_key_mismatch(llama_msgs):
    b = llama(FakeClient(lambda req: [answer(keys=['x', 'y', 'z'], probabilities=[1, 0, 0]),
                                      answer(keys=['0', '1', '2'], probabilities=[0.1, 0.3, 0.6],
                                             confidence=0.4)]))
    r = decide(b, qs=[CHOICE_Q, SCORE_Q])
    assert r.answers[0] is None and 'keys' in r.question_errors[0]
    assert r.answers[1].probabilities == pytest.approx([0.1, 0.3, 0.6])


def test_llama_ros_check_failures(llama_msgs, monkeypatch):
    with pytest.raises(BackendError, match='not available'):
        llama(FakeClient(lambda req: [], ready=False))
    for name in ('llama_msgs', 'llama_msgs.msg', 'llama_msgs.srv'):
        monkeypatch.setitem(sys.modules, name, None)
    with pytest.raises(BackendError, match='not installed'):
        asyncio.run(LlamaRosJev({}).check())
