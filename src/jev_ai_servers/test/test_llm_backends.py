"""LLM backends without ROS or network: mock, replay, anthropic (faked HTTP), openai_compat."""
import asyncio
import json
import sys
from pathlib import Path

import anthropic
import httpx
import httpx2
import pytest

from jev_ai.backends import BackendError, ReasonRequest
from jev_ai.core.catalog import load_catalog
from jev_ai.core.prompt import context, parse_response, response_schema, system_prompt
from jev_ai_servers.common import cassette_append
from jev_ai_servers.llm_backends.anthropic_backend import AnthropicLlm
from jev_ai_servers.llm_backends.mock import MockLlm, jev_lines
from jev_ai_servers.llm_backends.openai_compat import OpenAICompatLlm, is_local
from jev_ai_servers.llm_backends.replay import ReplayLlm, request_key

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'jev_ai' / 'test'))
from conftest import PATROL, STATE  # noqa: E402  (the jev_ai test catalog)

QSET = load_catalog(PATROL).question_sets['next_step']
P_JEV = {'next_action': [0.6, 0.3, 0.1], 'urgency': [0.5, 0.3, 0.2], 'report_now': [0.2, 0.8]}
SCHEMA = json.dumps(response_schema(QSET))


def patrol_req(masked=None, seed='s') -> ReasonRequest:
    ctx = context(QSET, QSET.ordered_state(STATE), P_JEV, masked=masked, seed=seed)
    return ReasonRequest(system_prompt(QSET, 'uav1'), ctx, SCHEMA)


def reason(backend, req):
    stages = []
    result = asyncio.run(backend.reason(req, lambda s, d: stages.append((s, d))))
    return result, stages


# --- mock ---------------------------------------------------------------------------------------

def test_mock_is_schema_valid_for_patrol():
    r, stages = reason(MockLlm({'latency_s': 0.0}), patrol_req())
    advice = parse_response(QSET, r.json)
    assert advice.errors == [] and set(advice.q_llm) == set(P_JEV)
    assert advice.params == {'inspect_altitude_m': 10.0}       # the schema's "default 10"
    assert advice.flags == [] and r.model == 'mock' and r.cost_usd == 0.0
    assert r.input_tokens > 0 and r.output_tokens > 0
    assert [s for s, _ in stages][:3] == ['started', 'first_token', 'streaming']
    assert stages[-1][0] == 'done'
    assert ''.join(d for s, d in stages if s == 'streaming') == r.text


def test_mock_is_deterministic_and_seeded():
    a = reason(MockLlm({'latency_s': 0.0}), patrol_req())[0].json
    assert a == reason(MockLlm({'latency_s': 0.0}), patrol_req())[0].json
    assert a != reason(MockLlm({'latency_s': 0.0, 'seed': 1}), patrol_req())[0].json


def test_mock_disagreement_zero_echoes_jev():
    r, _ = reason(MockLlm({'latency_s': 0.0, 'disagreement': 0.0}), patrol_req())
    for q, p in parse_response(QSET, r.json).q_llm.items():
        assert p == pytest.approx(P_JEV[q], abs=1e-3)


def test_mock_masked_classes_get_zero():
    req = patrol_req(masked={'next_action': {'halt_now', 'inspect'}})
    assert jev_lines(req.context)[1] == {'next_action': {'halt_now', 'inspect'}}
    w = json.loads(reason(MockLlm({'latency_s': 0.0, 'disagreement': 1.0}), req)[0].json)
    w = w['weights']['next_action']
    assert w['inspect'] == w['halt_now'] == 0.0 and w['continue'] > 0
    req = patrol_req(masked={'next_action': {'continue', 'inspect', 'halt_now'}})
    w = json.loads(reason(MockLlm({'latency_s': 0.0}), req)[0].json)['weights']['next_action']
    assert set(w.values()) == {0.0}                           # nothing left to weight


def test_mock_free_text_faults_and_invalid_json():
    r, _ = reason(MockLlm({'latency_s': 0.0}), ReasonRequest('sys', 'hello'))
    assert r.text and r.json == ''
    with pytest.raises(BackendError) as e:
        reason(MockLlm({'latency_s': 0.0, 'fail_rate': 1.0}), patrol_req())
    assert e.value.retryable
    r, _ = reason(MockLlm({'latency_s': 0.0, 'invalid_json_rate': 1.0}), patrol_req())
    with pytest.raises(ValueError):
        json.loads(r.json)


# --- replay -------------------------------------------------------------------------------------

def test_replay_hit_and_miss(tmp_path):
    req = patrol_req()
    path = tmp_path / 'c.jsonl'
    value = {'text': '{}', 'json': '{}', 'model': 'm', 'input_tokens': 3, 'output_tokens': 4,
             'cost_usd': 0.5}
    cassette_append(str(path), request_key(req), value)
    b = ReplayLlm({'path': str(path)})
    asyncio.run(b.check())
    r, _ = reason(b, req)
    assert (r.json, r.model, r.input_tokens, r.cost_usd) == ('{}', 'm', 3, 0.0)
    with pytest.raises(BackendError, match='replay miss') as e:
        reason(b, patrol_req(seed='other'))
    assert not e.value.retryable
    with pytest.raises(BackendError):
        asyncio.run(ReplayLlm({}).check())


# --- anthropic (faked HTTP through the real SDK) ------------------------------------------------

def sse(events) -> bytes:
    return ''.join(f'event: {e["type"]}\ndata: {json.dumps(e)}\n\n' for e in events).encode()


def anthropic_stream(text: str, stop_reason: str = 'end_turn') -> bytes:
    half = len(text) // 2
    usage = {'input_tokens': 10, 'output_tokens': 1, 'cache_read_input_tokens': 90,
             'cache_creation_input_tokens': 0}
    return sse([
        {'type': 'message_start', 'message': {
            'id': 'msg_1', 'type': 'message', 'role': 'assistant', 'model': 'claude-haiku-5-5',
            'content': [], 'stop_reason': None, 'stop_sequence': None, 'usage': usage}},
        {'type': 'content_block_start', 'index': 0, 'content_block': {'type': 'text', 'text': ''}},
        {'type': 'content_block_delta', 'index': 0,
         'delta': {'type': 'text_delta', 'text': text[:half]}},
        {'type': 'content_block_delta', 'index': 0,
         'delta': {'type': 'text_delta', 'text': text[half:]}},
        {'type': 'content_block_stop', 'index': 0},
        {'type': 'message_delta', 'delta': {'stop_reason': stop_reason, 'stop_sequence': None},
         'usage': {'output_tokens': 7}},
        {'type': 'message_stop'},
    ])


def fake_anthropic(handler) -> AnthropicLlm:
    b = AnthropicLlm({}, 'sk-test')
    b.client = anthropic.AsyncAnthropic(
        api_key='sk-test', max_retries=0,
        http_client=httpx2.AsyncClient(transport=httpx2.MockTransport(handler)))
    return b


def test_anthropic_request_shape_feedback_and_usage():
    seen = []

    def handler(request):
        seen.append(json.loads(request.content))
        return httpx2.Response(200, headers={'content-type': 'text/event-stream'},
                               content=anthropic_stream('{"ok": true}'))

    req = ReasonRequest('SYS', 'CTX', '{"type": "object"}',
                        options={'effort': 'low', 'max_tokens': '800'})
    r, stages = reason(fake_anthropic(handler), req)
    body = seen[0]
    assert body['model'] == 'claude-haiku-5-5' and body['max_tokens'] == 800
    assert body['stream'] is True
    assert body['system'] == [{'type': 'text', 'text': 'SYS',
                               'cache_control': {'type': 'ephemeral'}}]
    assert body['messages'] == [{'role': 'user', 'content': 'CTX'}]
    assert body['output_config'] == {'effort': 'low', 'format': {
        'type': 'json_schema', 'schema': {'type': 'object'}}}
    assert [s for s, _ in stages] == ['started', 'first_token', 'streaming', 'streaming', 'done']
    assert ''.join(d for _, d in stages) == r.text == r.json == '{"ok": true}'
    assert (r.model, r.input_tokens, r.output_tokens) == ('claude-haiku-5-5', 100, 7)
    assert r.cost_usd != r.cost_usd                          # NaN: the server prices it


def test_anthropic_free_text_has_no_output_config():
    seen = []

    def handler(request):
        seen.append(json.loads(request.content))
        return httpx2.Response(200, headers={'content-type': 'text/event-stream'},
                               content=anthropic_stream('hi'))

    r, _ = reason(fake_anthropic(handler), ReasonRequest('', 'CTX', model='claude-sonnet-5-5'))
    assert 'output_config' not in seen[0] and 'system' not in seen[0]
    assert seen[0]['model'] == 'claude-sonnet-5-5' and seen[0]['max_tokens'] == 1024
    assert r.text == 'hi' and r.json == ''


@pytest.mark.parametrize('stop_reason,match', [('refusal', 'refused'),
                                               ('max_tokens', 'truncated at max_tokens')])
def test_anthropic_refusal_and_truncation_have_their_own_errors(stop_reason, match):
    """User decision: not reported as 'response is not valid JSON'; not retried."""
    def handler(request):
        return httpx2.Response(200, headers={'content-type': 'text/event-stream'},
                               content=anthropic_stream('{"ok": ', stop_reason))

    with pytest.raises(BackendError, match=match) as e:
        reason(fake_anthropic(handler), ReasonRequest('s', 'c', '{"type": "object"}'))
    assert not e.value.retryable


@pytest.mark.parametrize('host,local', [
    ('localhost', True), ('127.0.0.1', True), ('::1', True), ('ollama', True),
    ('vllm.default.svc', True), ('gpu-box.local', True), ('10.0.0.5', True),
    ('192.168.1.20', True), ('api.openai.com', False), ('8.8.8.8', False)])
def test_openai_local_hosts_need_no_key(host, local):
    """User decision: compose/k8s service names and private IPs count as local."""
    assert is_local(host) is local


@pytest.mark.parametrize('status,retryable', [(429, True), (500, True), (529, True),
                                              (400, False), (401, False), (404, False)])
def test_anthropic_error_mapping(status, retryable):
    def handler(request):
        return httpx2.Response(status, json={'type': 'error',
                                             'error': {'type': 'x', 'message': 'nope'}})

    with pytest.raises(BackendError) as e:
        reason(fake_anthropic(handler), ReasonRequest('s', 'c'))
    assert e.value.retryable is retryable and str(status) in str(e.value)


def test_anthropic_connection_error_is_retryable_and_check_uses_models():
    def down(request):
        raise httpx2.ConnectError('down', request=request)

    with pytest.raises(BackendError) as e:
        reason(fake_anthropic(down), ReasonRequest('s', 'c'))
    assert e.value.retryable

    paths = []

    def models(request):
        paths.append(request.url.path)
        return httpx2.Response(200, json={'id': 'claude-haiku-5-5', 'type': 'model',
                                          'display_name': 'Haiku',
                                          'created_at': '2026-01-01T00:00:00Z'})

    asyncio.run(fake_anthropic(models).check())
    assert paths == ['/v1/models/claude-haiku-5-5']


# --- openai_compat ------------------------------------------------------------------------------

def fake_openai(params, handler, key='') -> OpenAICompatLlm:
    b = OpenAICompatLlm({'model': 'gpt-x', **params}, key)
    b.http = httpx.AsyncClient(headers=b.http.headers, transport=httpx.MockTransport(handler))
    return b


def ok_models(request):
    return httpx.Response(200, json={'data': []})


def test_openai_key_rule(monkeypatch):
    monkeypatch.delenv('OPENAI_API_KEY', raising=False)
    with pytest.raises(BackendError, match='needs an API key'):
        asyncio.run(fake_openai({}, ok_models).check())
    asyncio.run(fake_openai({'base_url': 'http://localhost:11434/v1'}, ok_models).check())
    asyncio.run(fake_openai({'base_url': 'http://127.0.0.1:8000/v1'}, ok_models).check())
    monkeypatch.setenv('OPENAI_API_KEY', 'sk-env')
    asyncio.run(fake_openai({}, ok_models).check())


def test_openai_schema_mode_request_and_result():
    seen = []

    def handler(request):
        seen.append((request.url, request.headers.get('authorization'),
                     json.loads(request.content)))
        return httpx.Response(200, json={
            'model': 'gpt-x-2026', 'choices': [{'message': {'content': '{"a": 1}'}}],
            'usage': {'prompt_tokens': 12, 'completion_tokens': 5}})

    req = ReasonRequest('SYS', 'CTX', '{"type": "object"}', options={'max_tokens': '300'})
    r, stages = reason(fake_openai({}, handler, key='sk-1'), req)
    url, auth, body = seen[0]
    assert str(url) == 'https://api.openai.com/v1/chat/completions' and auth == 'Bearer sk-1'
    assert body['model'] == 'gpt-x' and body['max_tokens'] == 300
    assert body['response_format'] == {'type': 'json_schema', 'json_schema': {
        'name': 'response', 'schema': {'type': 'object'}, 'strict': True}}
    assert body['messages'] == [{'role': 'system', 'content': 'SYS'},
                                {'role': 'user', 'content': 'CTX'}]
    assert [s for s, _ in stages] == ['started', 'done']
    assert (r.json, r.model, r.input_tokens, r.output_tokens) == ('{"a": 1}', 'gpt-x-2026', 12, 5)


def test_openai_json_object_mode_puts_schema_in_prompt_and_strips_fences():
    seen = []

    def handler(request):
        seen.append(json.loads(request.content))
        return httpx.Response(200, json={'choices': [{'message': {
            'content': '```json\n{"a": 1}\n```'}}]})

    r, _ = reason(fake_openai({'json_mode': 'json_object', 'base_url': 'http://localhost/v1'},
                              handler), ReasonRequest('SYS', 'CTX', '{"type": "object"}'))
    assert seen[0]['response_format'] == {'type': 'json_object'}
    assert seen[0]['messages'][0]['content'].endswith('{"type": "object"}')
    assert r.json == '{"a": 1}'


@pytest.mark.parametrize('status,retryable', [(429, True), (503, True), (401, False),
                                              (400, False)])
def test_openai_error_mapping(status, retryable):
    with pytest.raises(BackendError) as e:
        reason(fake_openai({}, lambda r: httpx.Response(status, text='err'), key='k'),
               ReasonRequest('s', 'c'))
    assert e.value.retryable is retryable
