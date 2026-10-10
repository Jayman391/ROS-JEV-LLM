"""``llm_server`` (§9): LLMs as the ROS action ``llm/reason`` (jev_ai_msgs/action/Reason, §3).

A lifecycle BackendServer (base.py owns keys, budgets, breaker, deadline, retries, diagnostics).
Goals are rejected fast while inactive or while the breaker is open; the engine then degrades
to delta = 0 (§11). Server params on top of base.PARAMS:

- ``effort`` ('low'), ``max_tokens`` (1024): merged into goal options unless the goal sets them;
  ``max_tokens`` is also a ceiling on what a goal may ask for;
- ``pricing``: JSON {model: [usd per Mtok in, usd per Mtok out]}, used when the backend does not
  price a call itself; an unknown model is charged at the highest listed rate (warned once).
"""
from __future__ import annotations

import json
import math
import time

from jev_ai_msgs.action import Reason
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from jev_ai.backends import ReasonRequest
from jev_ai_servers.base import BackendServer, spin
from jev_ai_servers.llm_backends.replay import request_key

# Anthropic first-party list prices, from the claude-api skill model table (cached 2026-10-06);
# not checked against the live pricing page. Haiku 5.5 doubles above 100K-token prompts.
PRICING = {'claude-haiku-5-5': [0.10, 0.50], 'claude-sonnet-5-5': [2.0, 10.0],
           'claude-opus-5-5': [4.0, 20.0]}


class LlmServer(BackendServer):
    GROUP = 'jev_ai.llm_backends'
    BUILTIN = {
        'mock': 'jev_ai_servers.llm_backends.mock:MockLlm',
        'replay': 'jev_ai_servers.llm_backends.replay:ReplayLlm',
        'anthropic': 'jev_ai_servers.llm_backends.anthropic_backend:AnthropicLlm',
        'openai_compat': 'jev_ai_servers.llm_backends.openai_compat:OpenAICompatLlm',
    }
    DEFAULT_BACKEND = 'anthropic'
    DEFAULTS = {'timeout_s': 6.0, 'effort': 'low', 'max_tokens': 1024,
                'pricing': json.dumps(PRICING)}

    def __init__(self, **kwargs):
        super().__init__('llm_server', **kwargs)
        self.action_server = None
        self.pricing: dict[str, list[float]] = {}
        self._unpriced: set[str] = set()

    def setup_endpoint(self) -> None:
        self.pricing = json.loads(self.p('pricing'))      # bad JSON fails the transition
        self.action_server = ActionServer(
            self, Reason, 'llm/reason', execute_callback=self._execute,
            goal_callback=self._on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=self.cb_group)

    def teardown_endpoint(self) -> None:
        if self.action_server:
            self.action_server.destroy()
            self.action_server = None

    def _on_goal(self, _goal) -> GoalResponse:
        ok = self.active and self.backend is not None and self.breaker.allow(time.monotonic())
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def price(self, model: str, tokens_in: int, tokens_out: int) -> float:
        """USD for a call. Exact model, else the longest priced prefix (dated ids such as
        claude-haiku-5-5-20261001), else the most expensive priced model: an unknown model must
        never be free, or any client could pick one and spend past max_usd_per_hour."""
        known = [m for m in self.pricing if model == m or model.startswith(m + '-')]
        if known:
            usd_in, usd_out = self.pricing[max(known, key=len)]
        else:
            if model not in self._unpriced:
                self._unpriced.add(model)
                self.get_logger().warning(
                    f'no pricing for model {model!r}: charged at the highest listed rate; add it '
                    'to `pricing` (e.g. [0, 0] for a local model) or set max_usd_per_hour: 0')
            if not self.pricing:
                return math.inf
            usd_in, usd_out = max(self.pricing.values(), key=lambda r: r[0] + r[1])
        return (tokens_in * usd_in + tokens_out * usd_out) / 1e6

    def _execute(self, goal_handle):
        goal = goal_handle.request
        defaults = {'effort': self.p('effort'), 'max_tokens': str(self.p('max_tokens'))}
        options = {k: v for k, v in defaults.items() if v != ''}
        options.update({kv.key: kv.value for kv in goal.options})
        # The server's max_tokens is a ceiling a goal may lower but not raise (cost per call).
        try:
            asked = int(options.get('max_tokens', self.p('max_tokens')))
        except ValueError:
            asked = self.p('max_tokens')
        options['max_tokens'] = str(max(1, min(asked, self.p('max_tokens'))))
        req = ReasonRequest(goal.system, goal.context, goal.response_schema,
                            goal.model or self.p('model'), options, goal.stream_text)

        def feedback(stage: str, delta: str) -> None:
            # Coarse feedback unless stream_text (A27); runs on the asyncio thread.
            if (stage == 'streaming' and not goal.stream_text) or not goal_handle.is_active:
                return
            goal_handle.publish_feedback(
                Reason.Feedback(stage=stage, delta=delta if goal.stream_text else ''))

        async def run():
            r = await self.backend.reason(req, feedback)
            if math.isnan(r.cost_usd):
                r.cost_usd = self.price(r.model, r.input_tokens, r.output_tokens)
            return r

        feedback('queued', '')
        r, error, ms = self.call(
            run, cancelled=lambda: goal_handle.is_cancel_requested,
            account=lambda r: (r.input_tokens + r.output_tokens, r.cost_usd))
        res = Reason.Result(success=False, error=error, latency_ms=ms)
        if error == 'cancelled':
            goal_handle.canceled()
            return res
        if r is not None:
            res.text, res.model, res.cost_usd = r.text, r.model, r.cost_usd
            res.input_tokens, res.output_tokens = r.input_tokens, r.output_tokens
            if req.response_schema:
                res.json = r.json or r.text
                try:
                    json.loads(res.json)
                except ValueError:
                    res.json, res.error = '', 'response is not valid JSON'  # engine retries, §7
            res.success = not res.error
        if not res.success:
            goal_handle.abort()
            return res
        goal_handle.succeed()
        self.record(request_key(req), {
            'text': res.text, 'json': res.json, 'model': res.model,
            'input_tokens': res.input_tokens, 'output_tokens': res.output_tokens,
            'cost_usd': res.cost_usd})
        return res


def main(args=None) -> None:
    spin(LlmServer, args)
