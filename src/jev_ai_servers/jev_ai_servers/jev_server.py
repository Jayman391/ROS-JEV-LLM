"""jev_server (FRAMEWORK §8): System 1 as the ROS service ``jev/decide`` (JevDecide.srv, §3).

Every question and every answer is validated on its own, so one malformed question or answer
becomes a ``question_errors`` entry and never fails the others; ``success`` is false only when
the whole backend call failed. Answers are aligned with the request's questions (failed ones carry
no probabilities). Budgets, breaker, deadline and retries come from BackendServer.call (§9).
"""
from __future__ import annotations

import math

from jev_ai_msgs.msg import Distribution
from jev_ai_msgs.srv import JevDecide

from jev_ai.backends import JevQuestion
from jev_ai.core.catalog import CLASS_COUNT, NOUL
from jev_ai.core.fusion import confidence
from jev_ai_servers.base import BackendServer, spin
from jev_ai_servers.jev_backends import replay

BUILTIN = {    # mirrors setup.py, so the server also runs from a source tree
    'mock': 'jev_ai_servers.jev_backends.mock:MockJev',
    'replay': 'jev_ai_servers.jev_backends.replay:ReplayJev',
    'typesafe': 'jev_ai_servers.jev_backends.typesafe_backend:TypeSafeJev',
    'llama_ros': 'jev_ai_servers.jev_backends.llama_ros:LlamaRosJev',
}


def question_error(q, seen: set[str]) -> str:
    """'' if the question is well-formed (§1 types and class counts, §3 alignment)."""
    if not q.name or q.name in seen:
        return f'question name {q.name!r} is empty or duplicated'
    if q.type not in CLASS_COUNT:
        return f'unknown question type {q.type}'
    classes = list(q.classes)
    lo, hi = CLASS_COUNT[q.type]
    if q.type == NOUL and classes != ['true', 'false']:
        return 'noul classes must be ["true", "false"]'
    if not lo <= len(classes) <= hi:
        return f'{len(classes)} classes; this type needs {lo}-{hi}'
    if not all(classes) or len(set(classes)) != len(classes):
        return 'class names must be non-empty and unique'
    if q.criteria and len(q.criteria) != len(classes):
        return f'{len(q.criteria)} criteria for {len(classes)} classes (empty or aligned)'
    if q.type == NOUL and not (q.instructions or any(q.criteria)):
        return 'noul question needs instructions or criteria'
    return ''


def normalise(p: list[float], n: int) -> tuple[list[float], str]:
    """Renormalised probabilities, or an error if the answer is unusable."""
    if len(p) != n:
        return [], f'answer has {len(p)} probabilities for {n} classes'
    if not all(math.isfinite(x) and x >= 0 for x in p):
        return [], 'answer has negative or non-finite probabilities'
    s = sum(p)
    if s <= 0:
        return [], 'answer probabilities sum to 0'
    return [x / s for x in p], ''


class JevServer(BackendServer):
    GROUP = 'jev_ai.jev_backends'
    BUILTIN = BUILTIN
    DEFAULT_BACKEND = 'typesafe'
    DEFAULTS = {'timeout_s': 1.0, 'usd_per_mtok_input': 0.042}   # TypeSafe price (RESEARCH §1)

    def __init__(self, **kwargs):
        super().__init__('jev_server', **kwargs)
        self.srv = None

    def setup_endpoint(self) -> None:
        self.srv = self.create_service(JevDecide, 'jev/decide', self._decide,
                                       callback_group=self.cb_group)

    def teardown_endpoint(self) -> None:
        if self.srv is not None:
            self.destroy_service(self.srv)
            self.srv = None

    def _decide(self, req, resp):
        errors, seen, ok = [], set(), []
        for i, q in enumerate(req.questions):
            errors.append(question_error(q, seen))
            seen.add(q.name)
            if not errors[-1]:
                ok.append(i)
        state = [(kv.key, kv.value) for kv in req.state]
        qs = [JevQuestion(q.name, q.type, q.instructions, list(q.classes), list(q.criteria))
              for q in (req.questions[i] for i in ok)]
        # Model precedence: request, <backend>.model (pinned), server model, backend default.
        pinned = self.backend.params.get('model', '') if self.backend else ''
        model = req.model or pinned or self.p('model')
        resp.model = model

        if not ok:
            result, error = None, 'no valid questions' if req.questions else 'no questions'
        else:
            usd = self.p('usd_per_mtok_input') / 1e6
            result, error, resp.latency_ms = self.call(
                lambda: self.backend.decide(state, qs, model),
                account=lambda r: (r.input_tokens, r.input_tokens * usd))
            if result and {len(result.answers), len(result.question_errors)} != {len(qs)}:
                result, error = None, f'backend misaligned: {len(result.answers)} answers ' \
                    f'for {len(qs)} questions'
        if result is None:
            resp.success, resp.error = False, error
            resp.question_errors = [e or error for e in errors]
            return resp

        self.record(replay.key(state, qs, model), replay.to_record(result))
        by_index = dict(zip(ok, zip(result.answers, result.question_errors)))
        for i, q in enumerate(req.questions):
            d = Distribution(question=q.name, type=q.type, classes=list(q.classes),
                             confidence=math.nan, source_confidence=math.nan)
            if i in by_index:
                answer, err = by_index[i]
                if answer is None:
                    errors[i] = err or 'no answer'
                else:
                    p, errors[i] = normalise(answer.probabilities, len(q.classes))
                    if p:
                        d.probabilities, d.confidence = p, confidence(p)
                        d.source_confidence = float(answer.source_confidence)
            resp.answers.append(d)
        resp.success = True
        resp.model = result.model
        resp.question_errors = errors
        resp.input_tokens = max(0, int(result.input_tokens))
        return resp


def main(args=None) -> None:
    spin(JevServer, args)
