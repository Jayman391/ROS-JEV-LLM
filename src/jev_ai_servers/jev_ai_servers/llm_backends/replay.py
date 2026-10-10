"""``replay`` LLM backend (§9): answers from a cassette written by ``llm_server`` (record_path).

Params: ``path`` (JSONL cassette). A miss is a non-retryable error, never a made-up answer."""
from __future__ import annotations

from jev_ai.backends import BackendError, Feedback, LlmBackend, ReasonRequest, ReasonResult
from jev_ai_servers.common import cassette_key, cassette_load


def request_key(req: ReasonRequest) -> str:
    """The cassette key; llm_server records under it and this backend looks it up."""
    return cassette_key('llm', req.system, req.context, req.response_schema, req.model,
                        sorted(req.options.items()))


class ReplayLlm(LlmBackend):
    name = 'replay'

    def __init__(self, params, api_key=''):
        super().__init__(params, api_key)
        self.cassette: dict[str, dict] = {}

    async def check(self) -> None:
        path = self.params.get('path', '')
        if not path:
            raise BackendError('replay.path is not set')
        try:
            self.cassette = cassette_load(path)
        except (OSError, ValueError, KeyError) as e:
            raise BackendError(f'cannot read cassette {path}: {e}') from None

    async def reason(self, req: ReasonRequest, feedback: Feedback) -> ReasonResult:
        value = self.cassette.get(request_key(req))
        if value is None:
            raise BackendError(f'replay miss: no recorded answer for model={req.model!r} '
                               f'context={req.context[:60]!r}')
        feedback('started', '')
        feedback('done', '')
        return ReasonResult(**{**value, 'cost_usd': 0.0})    # a replay spends nothing
