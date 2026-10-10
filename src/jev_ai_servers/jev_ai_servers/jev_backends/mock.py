"""Keyless, deterministic Jev (FRAMEWORK §8 ``mock``): only runs when asked for (backend:=mock).

Each answer is a Dirichlet(alpha) draw seeded by sha256(seed, state, question name, classes), so
the same request always gets the same answer. Parameters (ROS ``mock.<name>``):

- ``seed`` (0), ``alpha`` (1.0): Dirichlet concentration; small = peaked, large = flat;
- ``first_bias`` (0.0): class 0 is multiplied by (1 + first_bias), then renormalised, to imitate
  Jev's first-option bias (RESEARCH §1);
- ``latency_s`` (0.02): simulated latency;
- ``fail_rate`` (0.0): seeded per-question fault injection;
- ``rules`` (''): ``module:function`` hook ``fn(state: dict, question: JevQuestion)`` returning
  probabilities aligned with the classes, or None to fall back to the Dirichlet. It lets example
  apps make the mock behave plausibly (e.g. "battery low → return_home is likely").
"""
from __future__ import annotations

import asyncio
import hashlib
import json
import random
from importlib import import_module

from jev_ai.backends import BackendError, JevAnswer, JevBackend, JevQuestion, JevResult


class MockJev(JevBackend):
    name = 'mock'

    def __init__(self, params, api_key=''):
        super().__init__(params, api_key)
        self.seed = params.get('seed', 0)
        self.alpha = float(params.get('alpha', 1.0))
        self.first_bias = float(params.get('first_bias', 0.0))
        self.latency_s = float(params.get('latency_s', 0.02))
        self.fail_rate = float(params.get('fail_rate', 0.0))
        if self.alpha <= 0:
            raise BackendError('mock.alpha must be > 0')
        self.rules = None
        if params.get('rules'):
            module, _, fn = params['rules'].partition(':')
            try:
                self.rules = getattr(import_module(module), fn)
            except (ImportError, AttributeError, ValueError) as e:
                raise BackendError(f'mock.rules {params["rules"]!r}: {e}') from None

    def _rng(self, *parts) -> random.Random:
        blob = json.dumps([self.seed, *parts], sort_keys=True).encode()
        return random.Random(hashlib.sha256(blob).digest())

    async def decide(self, state, questions: list[JevQuestion], model: str) -> JevResult:
        await asyncio.sleep(self.latency_s)
        state_d = dict(state)
        answers, errors = [], []
        for q in questions:
            if self._rng(state, q.name, 'fail').random() < self.fail_rate:
                answers.append(None)
                errors.append('mock: injected failure')
                continue
            p = self.rules(state_d, q) if self.rules else None
            if p is None:
                rng = self._rng(state, q.name, q.classes)
                p = [rng.gammavariate(self.alpha, 1.0) for _ in q.classes]
            p = list(p)
            if p:
                p[0] *= 1.0 + self.first_bias
            s = sum(p)
            p = [x / s for x in p] if s > 0 else p    # the server rejects a zero-sum answer
            answers.append(JevAnswer(p, max(p, default=0.0)))
            errors.append('')
        text = json.dumps([state, [[q.instructions, q.classes, q.criteria] for q in questions]])
        return JevResult(model='mock', answers=answers, question_errors=errors,
                         input_tokens=len(text) // 4)
