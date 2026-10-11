"""llm_server in-process: goals over a real action client, cancel, breaker, keys, record/replay."""
import json
import threading
import time

import pytest
import rclpy
from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import KeyValue
from jev_ai_msgs.action import Reason
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.parameter import Parameter

from jev_ai_servers.llm_server import LlmServer, goal_timeout

SCHEMA = json.dumps({'type': 'object', 'additionalProperties': False, 'required': ['weights'],
                     'properties': {'weights': {'type': 'object', 'properties': {
                         'go': {'type': 'object', 'properties': {
                             'a': {'type': 'number'}, 'b': {'type': 'number'}}}}}}})
CONTEXT = 'Fast model (Jev) probabilities:\n- go: a 0.70, b 0.30'


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    def __init__(self, **params):
        self.server = LlmServer(parameter_overrides=[Parameter(k, value=v)
                                                     for k, v in params.items()])
        self.client_node = rclpy.create_node('llm_test_client')
        self.client = ActionClient(self.client_node, Reason, 'llm/reason')
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.server)
        self.executor.add_node(self.client_node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        until(lambda: self.server.active, 10)
        assert self.client.wait_for_server(timeout_sec=5)

    def send(self, **goal):
        self.feedback = []
        fut = self.client.send_goal_async(
            Reason.Goal(**goal), feedback_callback=lambda m: self.feedback.append(m.feedback))
        return until(fut.done, 5) and fut.result()

    def result(self, handle, timeout=10):
        fut = handle.get_result_async()
        until(fut.done, timeout)
        return fut.result()

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=5)
        self.client.destroy()
        self.client_node.destroy_node()
        self.server.loop.stop()
        self.server.destroy_node()


def until(cond, timeout):
    end = time.monotonic() + timeout
    while not cond():
        assert time.monotonic() < end, 'timed out'
        time.sleep(0.01)
    return True


def test_goal_succeeds_with_json_and_feedback():
    h = Harness(backend='mock', **{'mock.latency_s': 0.05})
    try:
        handle = h.send(system='sys', context=CONTEXT, response_schema=SCHEMA, stream_text=True,
                        options=[KeyValue(key='max_tokens', value='50')])
        assert handle.accepted
        out = h.result(handle)
        r = out.result
        assert out.status == GoalStatus.STATUS_SUCCEEDED and r.success, r.error
        assert json.loads(r.json)['weights']['go']['a'] > 0
        assert r.model == 'mock' and r.input_tokens > 0 and r.latency_ms > 0
        stages = [f.stage for f in h.feedback]
        assert stages[0] == 'queued' and stages[-1] == 'done'
        assert {'started', 'first_token', 'streaming'} <= set(stages)
        assert ''.join(f.delta for f in h.feedback) == r.text

        # coarse feedback by default: no streaming stage, no deltas (A27)
        r2 = h.result(h.send(system='sys', context=CONTEXT, response_schema=SCHEMA))
        assert r2.result.success and 'streaming' not in [f.stage for f in h.feedback]
        assert all(f.delta == '' for f in h.feedback)
        assert h.server.calls == 2 and h.server.budget.tokens_last_min > 0
    finally:
        h.close()


def test_invalid_json_fails_the_goal():
    h = Harness(backend='mock', **{'mock.latency_s': 0.0, 'mock.invalid_json_rate': 1.0})
    try:
        out = h.result(h.send(system='s', context=CONTEXT, response_schema=SCHEMA))
        assert out.status == GoalStatus.STATUS_ABORTED
        assert not out.result.success and out.result.error == 'response is not valid JSON'
    finally:
        h.close()


def test_cancel_mid_call():
    h = Harness(backend='mock', timeout_s=30.0, **{'mock.latency_s': 20.0})
    try:
        handle = h.send(system='s', context=CONTEXT, response_schema=SCHEMA)
        until(lambda: any(f.stage == 'started' for f in h.feedback), 5)
        t0 = time.monotonic()
        cancel = handle.cancel_goal_async()
        until(cancel.done, 5)
        out = h.result(handle, timeout=5)
        assert out.status == GoalStatus.STATUS_CANCELED
        assert out.result.error == 'cancelled' and not out.result.success
        assert time.monotonic() - t0 < 3
        assert h.server.errors == 0                       # a cancel is not a backend failure
    finally:
        h.close()


def test_breaker_open_rejects_goals():
    h = Harness(backend='mock')
    try:
        h.server.breaker.opened_at = time.monotonic()
        assert not h.send(system='s', context='c').accepted
    finally:
        h.close()


def test_pricing_ceiling_and_read_only_params():
    h = Harness(backend='mock', max_tokens=100, **{'mock.latency_s': 0.0})
    try:
        s = h.server
        haiku = s.price('claude-haiku-5-5', 10**6, 0)
        assert haiku == 0.10
        assert s.price('claude-haiku-5-5-20261001', 10**6, 0) == haiku          # dated id
        assert s.price('some-new-model', 10**6, 0) == 0.0                       # reported, no cap

        # a goal may lower max_tokens but never raise it past the server's ceiling
        seen = []
        real = s.backend.reason

        async def spy(req, feedback):
            seen.append(req.options['max_tokens'])
            return await real(req, feedback)
        s.backend.reason = spy
        for asked in ('100000', '7', 'lots'):
            h.result(h.send(system='s', context=CONTEXT, response_schema=SCHEMA,
                            options=[KeyValue(key='max_tokens', value=asked)]))
        assert seen == ['100', '7', '100']

        # parameters are launch-time only: no runtime redirect of record_path and friends
        res = s.set_parameters([Parameter('record_path', value='/tmp/pwn')])
        assert not res[0].successful and s.p('record_path') == ''

        # spend is tracked but never refuses a call
        s.budget.record(time.monotonic(), 1, 1e6)
        assert s.budget.admit(time.monotonic()) == '' and s.budget.usd_last_hour >= 1e6
    finally:
        h.close()


def test_goal_timeout_only_shortens_within_bounds():
    for asked, want in (('4', 4.0), ('30', 6.0), ('0.001', 1.0), ('-1', 1.0), ('nan', 6.0),
                        ('inf', 6.0), ('', 6.0), ('soon', 6.0)):
        assert goal_timeout(asked, 6.0) == want, asked


def test_anthropic_without_key_fails_configure(monkeypatch):
    monkeypatch.delenv('ANTHROPIC_API_KEY', raising=False)
    node = LlmServer(parameter_overrides=[Parameter('autostart', value=False)])
    try:
        assert node.p('backend') == 'anthropic'
        assert node.trigger_configure() == TransitionCallbackReturn.FAILURE
        assert 'ANTHROPIC_API_KEY' in node.last_error
    finally:
        node.loop.stop()
        node.destroy_node()


def test_record_then_replay(tmp_path):
    path = str(tmp_path / 'llm.jsonl')
    goal = dict(system='s', context=CONTEXT, response_schema=SCHEMA)
    h = Harness(backend='mock', record_path=path, **{'mock.latency_s': 0.0})
    try:
        first = h.result(h.send(**goal)).result
        assert first.success
    finally:
        h.close()
    h = Harness(backend='replay', **{'replay.path': path})
    try:
        again = h.result(h.send(**goal)).result
        assert again.success and (again.json, again.text, again.model) == \
            (first.json, first.text, first.model)
        miss = h.result(h.send(**{**goal, 'context': 'other'})).result
        assert not miss.success and 'replay miss' in miss.error
    finally:
        h.close()
