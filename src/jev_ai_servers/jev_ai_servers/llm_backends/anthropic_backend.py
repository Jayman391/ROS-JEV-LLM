"""``anthropic`` LLM backend (§9, RESEARCH §2): the native SDK (anthropic 1.x), streamed.

- ``system`` goes in one text block with ``cache_control`` (stable per caller, so it is cached);
  ``context`` is the user message.
- ``options.effort`` -> ``output_config.effort``; ``response_schema`` ->
  ``output_config.format`` (json_schema); ``options.max_tokens`` -> ``max_tokens``.
- Text deltas map to feedback: started, first_token, streaming(delta), done. Cancelling the task
  exits ``messages.stream`` and closes the HTTP stream.
- 429/5xx/connection errors are retryable; auth and 4xx are not. SDK retries are off: the
  server owns retries and the deadline.

Params: ``model`` (else the server's ``model``, else claude-haiku-5-5), ``base_url``.
"""
from __future__ import annotations

import json

import anthropic

from jev_ai.backends import BackendError, Feedback, LlmBackend, ReasonRequest, ReasonResult

DEFAULT_MODEL = 'claude-haiku-5-5'


class AnthropicLlm(LlmBackend):
    name = 'anthropic'
    key_env = 'ANTHROPIC_API_KEY'

    def __init__(self, params, api_key=''):
        super().__init__(params, api_key)
        self.client = anthropic.AsyncAnthropic(api_key=api_key, max_retries=0,
                                               base_url=params.get('base_url') or None)

    def default_model(self) -> str:
        return self.params.get('model') or (self.node.p('model') if self.node else '') or \
            DEFAULT_MODEL

    async def check(self) -> None:
        """Validates key and model with ``models.retrieve`` (no tokens spent)."""
        try:
            await self.client.models.retrieve(self.default_model())
        except anthropic.APIError as e:
            raise _error(e) from None

    async def reason(self, req: ReasonRequest, feedback: Feedback) -> ReasonResult:
        kwargs = {
            'model': req.model or self.default_model(),
            'max_tokens': int(req.options.get('max_tokens', 1024)),
            'messages': [{'role': 'user', 'content': req.context}],
        }
        if req.system:                            # the API rejects empty text blocks
            kwargs['system'] = [{'type': 'text', 'text': req.system,
                                 'cache_control': {'type': 'ephemeral'}}]
        output_config = {}
        if req.options.get('effort'):
            output_config['effort'] = req.options['effort']
        if req.response_schema:
            try:
                schema = json.loads(req.response_schema)
            except ValueError as e:
                raise BackendError(f'response_schema is not valid JSON: {e}') from None
            output_config['format'] = {'type': 'json_schema', 'schema': schema}
        if output_config:
            kwargs['output_config'] = output_config

        feedback('started', '')
        try:
            async with self.client.messages.stream(**kwargs) as stream:
                first = True
                async for delta in stream.text_stream:
                    if first:
                        feedback('first_token', '')
                        first = False
                    feedback('streaming', delta)
                msg = await stream.get_final_message()
        except anthropic.APIError as e:
            raise _error(e) from None
        feedback('done', '')
        if msg.stop_reason == 'refusal':          # distinct from a schema failure: no retry
            raise BackendError('anthropic: the model refused to answer (stop_reason=refusal)')
        if msg.stop_reason == 'max_tokens':
            raise BackendError(f'anthropic: output truncated at max_tokens={kwargs["max_tokens"]}'
                               ' (raise llm_server max_tokens)')
        text = ''.join(b.text for b in msg.content if b.type == 'text')
        u = msg.usage
        # ponytail: cached tokens are counted (and priced) as plain input, so spend is
        # over-estimated when the cache hits; price cache reads/writes separately if that matters.
        tokens_in = u.input_tokens + (u.cache_read_input_tokens or 0) + \
            (u.cache_creation_input_tokens or 0)
        return ReasonResult(text=text, json=text if req.response_schema else '', model=msg.model,
                            input_tokens=tokens_in, output_tokens=u.output_tokens)

    async def close(self) -> None:
        await self.client.close()


def _error(e: anthropic.APIError) -> BackendError:
    if isinstance(e, anthropic.APIConnectionError):          # includes timeouts
        return BackendError(f'anthropic: {e}', retryable=True)
    status = getattr(e, 'status_code', 0)
    return BackendError(f'anthropic {status}: {e.message}',
                        retryable=status in (408, 409, 429) or status >= 500)
