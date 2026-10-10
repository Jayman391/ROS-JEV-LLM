"""Local Jev-style decisions through llama_ros (FRAMEWORK §8 ``llama_ros``, D4): offline, keyless.

Calls ``llama_msgs/srv/EvaluateDecisions`` (llama_ros main, checked 2026-10-10) on a GGUF
decision model (OpenJEV, Kev, Laya, ...). Parameters (ROS ``llama_ros.<name>``): ``service``
('llama/evaluate_decisions', the llama node's namespace in the upstream launch files) and
``model`` (a label for records; the service does not report one).

Interface notes (upstream msg files): llama's type constants differ from ours (CHOICE=0, SCORE=1,
NOUL=2); NOUL descriptions are ordered [false, true]; answers carry per-question success/error,
``keys`` aligned with ``probabilities``, and ``noul`` = P(true) with ``confidence`` left at 0.
``llama_msgs`` is imported lazily so the server runs without llama_ros installed.
"""
from __future__ import annotations

import asyncio
import json
import math

from jev_ai.backends import BackendError, JevAnswer, JevBackend, JevQuestion, JevResult
from jev_ai.core.catalog import CHOICE, NOUL, SCORE


def _settle(done: asyncio.Future, fut) -> None:
    if not done.cancelled():
        e = fut.exception()
        done.set_exception(e) if e else done.set_result(fut.result())


class LlamaRosJev(JevBackend):
    name = 'llama_ros'

    def __init__(self, params, api_key=''):
        super().__init__(params, api_key)
        self.client = None

    async def check(self) -> None:
        try:
            from llama_msgs.msg import DecisionQuestion
            from llama_msgs.srv import EvaluateDecisions
        except ImportError:
            raise BackendError('llama_ros: llama_msgs is not installed; build llama_ros '
                               '(github.com/mgonzs13/llama_ros) in this workspace') from None
        if self.node is None:
            raise BackendError('llama_ros: needs the server node to create a client')
        self.msg, self.srv = DecisionQuestion, EvaluateDecisions
        service = self.params.get('service', 'llama/evaluate_decisions')
        self.client = self.node.create_client(EvaluateDecisions, service,
                                              callback_group=self.node.cb_group)
        # ponytail: fixed 3 s wait (configure's budget is 5 s); retry configure if the model is
        # still loading.
        if not await asyncio.to_thread(self.client.wait_for_service, 3.0):
            raise BackendError(f'llama_ros: service {service!r} not available '
                               '(is llama_node running with a decision model?)')

    def _question(self, q: JevQuestion):
        m = self.msg
        crit = q.criteria or [''] * len(q.classes)
        if q.type == CHOICE:
            return m(type=m.CHOICE, instructions=q.instructions, keys=q.classes,
                     descriptions=q.criteria)
        if q.type == SCORE:
            return m(type=m.SCORE, instructions=q.instructions,
                     descriptions=[f'{c}: {d}' if d else c for c, d in zip(q.classes, crit)])
        # ours: ["true", "false"]; llama: [false, true]
        return m(type=m.NOUL, instructions=q.instructions,
                 descriptions=[crit[1], crit[0]] if any(crit) else [])

    async def decide(self, state, questions, model) -> JevResult:
        req = self.srv.Request(state=json.dumps(dict(state)),
                               questions=[self._question(q) for q in questions])
        fut = self.client.call_async(req)
        loop = asyncio.get_running_loop()
        done = loop.create_future()
        fut.add_done_callback(lambda f: loop.call_soon_threadsafe(_settle, done, f))
        try:
            resp = await done
        except asyncio.CancelledError:
            self.client.remove_pending_request(fut)
            raise
        if len(resp.answers) != len(questions):
            raise BackendError(f'llama_ros: {len(resp.answers)} answers for '
                               f'{len(questions)} questions')
        answers, errors = [], []
        for q, a in zip(questions, resp.answers):
            if not a.success:
                answers.append(None)
                errors.append(f'llama_ros: {a.error or "failed"}')
                continue
            if q.type == NOUL:
                p, conf = [a.noul, 1.0 - a.noul], math.nan
            elif q.type == CHOICE:
                by_key = dict(zip(a.keys, a.probabilities))
                if set(by_key) != set(q.classes):
                    answers.append(None)
                    errors.append(f'llama_ros: answer keys {list(a.keys)} != classes')
                    continue
                p, conf = [by_key[c] for c in q.classes], a.confidence
            else:
                # UNVERIFIED: what ``keys`` holds for SCORE answers; levels are taken by position
                # (the server rejects a length mismatch).
                p, conf = list(a.probabilities), a.confidence
            answers.append(JevAnswer([float(x) for x in p], float(conf)))
            errors.append('')
        return JevResult(model=model or 'llama_ros', answers=answers, question_errors=errors)

    async def close(self) -> None:
        if self.client is not None and self.node is not None:
            self.node.destroy_client(self.client)
            self.client = None
