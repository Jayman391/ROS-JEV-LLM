"""TypeSafe Jev (FRAMEWORK §8 ``typesafe``, RESEARCH §1) through typesafe-sdk's native async
client, ``AsyncTypeSafeClient`` (POST /v1/systemone, GET /v1/models). OpenRouter is the same
backend with ``typesafe.base_url:=https://openrouter.ai/api``.

Parameters (ROS ``typesafe.<name>``): ``model`` (pinned, e.g. ``jev-1.13.0``; else the server's
``model``, else ``jev-latest``), ``base_url`` ('' = the SDK default or ``TYPESAFE_BASE_URL``).
SDK retries are off: the server owns retries and the wall-time deadline (base.call), and
cancelling the asyncio task aborts the HTTP request.
"""
from __future__ import annotations

from typesafe_sdk import (AsyncTypeSafeClient, Choice, Noul, RetryPolicy, Score,
                          TypeSafeAPIConnectionError, TypeSafeAPIError,
                          TypeSafeAPIResponseValidationError, TypeSafeError)

from jev_ai.backends import BackendError, JevAnswer, JevBackend, JevQuestion, JevResult
from jev_ai.core.catalog import CHOICE, NOUL

DEFAULT_MODEL = 'jev-latest'


def _error(e: TypeSafeError) -> BackendError:
    """429/408/5xx and network failures are worth one more try (§8 max_retries); 4xx are not."""
    retryable = isinstance(e, TypeSafeAPIConnectionError) or (
        isinstance(e, TypeSafeAPIError) and (e.status in (408, 429) or e.status >= 500))
    return BackendError(f'typesafe: {e}', retryable)


def _question(q: JevQuestion):
    instructions = q.instructions or None
    criteria = q.criteria or [''] * len(q.classes)
    if q.type == NOUL:                        # classes are ["true", "false"] (server-validated)
        described = {c: d for c, d in zip(q.classes, criteria) if d}
        return Noul(instructions=instructions, criteria=described or None)
    if q.type == CHOICE:
        return Choice(instructions=instructions,
                      criteria={c: d or None for c, d in zip(q.classes, criteria)})
    # Score levels are positional; keep the level name visible to the model.
    return Score(instructions=instructions,
                 criteria=[f'{c}: {d}' if d else c for c, d in zip(q.classes, criteria)])


def _answer(q: JevQuestion, a) -> JevAnswer:
    """Map an SDK answer onto the question's class order. KeyError = missing/mismatched answer."""
    if a is None:
        raise KeyError('no answer')
    if q.type == NOUL:
        return JevAnswer([a.noul, 1.0 - a.noul])    # noul = P(yes); no model confidence
    keys = q.classes if q.type == CHOICE else range(len(q.classes))   # score: int levels
    return JevAnswer([a.probabilities[k] for k in keys], a.confidence)


class TypeSafeJev(JevBackend):
    name = 'typesafe'
    key_env = 'TYPESAFE_API_KEY'

    def __init__(self, params, api_key=''):
        super().__init__(params, api_key)
        try:
            self.client = AsyncTypeSafeClient(api_key=api_key, retry=RetryPolicy(max_retries=0),
                                              base_url=params.get('base_url') or None)
        except TypeSafeError as e:                 # malformed key
            raise BackendError(f'typesafe: {e}') from None

    async def check(self) -> None:
        """§8 on_configure: the key works and the model is offered."""
        model = (self.params.get('model') or (self.node.p('model') if self.node else '')
                 or DEFAULT_MODEL)
        try:
            listed = await self.client.models.list()
        except TypeSafeAPIResponseValidationError:
            # A /v1/models that is not TypeSafe's shape (e.g. OpenRouter's catalogue): the model
            # cannot be verified here; the first decide() reports a bad key or model.
            if self.node:
                self.node.get_logger().warning('typesafe: /v1/models not in TypeSafe format; '
                                               f'model {model!r} not verified')
            return
        except TypeSafeError as e:
            raise _error(e) from None
        names = sorted(m.name for m in listed.models)
        if model not in names:
            raise BackendError(f'typesafe: model {model!r} not offered; available: '
                               f'{", ".join(names)}')

    async def decide(self, state, questions, model) -> JevResult:
        try:
            r = await self.client.system_one(dict(state), {q.name: _question(q) for q in questions},
                                             model=model or DEFAULT_MODEL)
        except TypeSafeError as e:
            raise _error(e) from None
        answers, errors = [], []
        for q in questions:
            try:
                answers.append(_answer(q, r.answers.get(q.name)))
                errors.append('')
            except (KeyError, AttributeError) as e:
                answers.append(None)
                errors.append(f'typesafe: bad answer for {q.name!r}: {e}')
        return JevResult(model=r.model, answers=answers, question_errors=errors,
                         input_tokens=r.usage.input_tokens or 0)

    async def close(self) -> None:
        await self.client.aclose()
