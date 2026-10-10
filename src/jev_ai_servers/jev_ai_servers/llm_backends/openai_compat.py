"""``openai_compat`` LLM backend (§9): any OpenAI-style ``/chat/completions`` endpoint (OpenAI,
Gemini, Ollama, vLLM) over httpx. Non-streaming: feedback is started, then done.

Params:
- ``base_url`` ('https://api.openai.com/v1'), ``model`` (else the server's ``model``);
- ``api_key_env`` ('OPENAI_API_KEY'); a key is required unless base_url is local: loopback,
  a private IP, or a private hostname such as a compose service (§10 egress: local models need
  none);
- ``json_mode``: 'schema' (``response_format`` json_schema, strict), 'json_object' (JSON mode,
  schema in the system prompt, output validated by the server) or 'none' (schema in the prompt
  only, for endpoints without JSON mode).

``options.effort`` is ignored: ``reasoning_effort`` is rejected by many models and endpoints.
"""
from __future__ import annotations

import ipaddress
import json
from urllib.parse import urlparse

import httpx

from jev_ai.backends import BackendError, Feedback, LlmBackend, ReasonRequest, ReasonResult
from jev_ai_servers.common import read_key

LOCAL_SUFFIXES = ('.local', '.internal', '.svc', '.cluster.local', '.lan')


def is_local(host: str) -> bool:
    """Endpoints that need no key: loopback, private/link-local IPs, and private names such as
    compose/k8s services (``ollama``, ``vllm.default.svc``) or ``*.local`` (user decision, A4)."""
    try:
        ip = ipaddress.ip_address(host)
        return ip.is_loopback or ip.is_private or ip.is_link_local
    except ValueError:
        return host == 'localhost' or '.' not in host or host.endswith(LOCAL_SUFFIXES)


class OpenAICompatLlm(LlmBackend):
    name = 'openai_compat'
    key_env = ''             # checked in check(): only remote endpoints need a key

    def __init__(self, params, api_key=''):
        super().__init__(params, api_key)
        self.base_url = params.get('base_url', 'https://api.openai.com/v1').rstrip('/')
        self.api_key = api_key or read_key(params.get('api_key_env', 'OPENAI_API_KEY'))
        self.json_mode = params.get('json_mode', 'schema')
        headers = {'Authorization': f'Bearer {self.api_key}'} if self.api_key else {}
        self.http = httpx.AsyncClient(headers=headers, timeout=None)   # the server's deadline

    async def check(self) -> None:
        if self.json_mode not in ('schema', 'json_object', 'none'):
            raise BackendError(f'openai_compat.json_mode must be schema|json_object|none, '
                               f'not {self.json_mode!r}')
        if not self.api_key and not is_local(urlparse(self.base_url).hostname or ''):
            raise BackendError(f'openai_compat: {self.base_url} needs an API key: set '
                               f'{self.params.get("api_key_env", "OPENAI_API_KEY")} or '
                               'api_key_file')
        if not self.default_model():
            raise BackendError('openai_compat: set openai_compat.model (or the server model)')
        await self._send('GET', '/models')          # endpoint + key, no tokens spent

    def default_model(self) -> str:
        return self.params.get('model') or (self.node.p('model') if self.node else '')

    async def reason(self, req: ReasonRequest, feedback: Feedback) -> ReasonResult:
        system = req.system
        body = {'model': req.model or self.default_model(),
                'max_tokens': int(req.options.get('max_tokens', 1024))}
        if req.response_schema:
            try:
                schema = json.loads(req.response_schema)
            except ValueError as e:
                raise BackendError(f'response_schema is not valid JSON: {e}') from None
            if self.json_mode == 'schema':
                body['response_format'] = {'type': 'json_schema', 'json_schema': {
                    'name': 'response', 'schema': schema, 'strict': True}}
            else:
                system += '\n\nAnswer with JSON only, matching this JSON Schema:\n' + \
                    req.response_schema
                if self.json_mode == 'json_object':
                    body['response_format'] = {'type': 'json_object'}
        body['messages'] = [{'role': 'system', 'content': system},
                            {'role': 'user', 'content': req.context}]
        feedback('started', '')
        data = await self._send('POST', '/chat/completions', body)
        feedback('done', '')
        try:
            choice = data['choices'][0]
            text = choice['message'].get('content') or ''
            refusal = choice['message'].get('refusal')
            usage = data.get('usage') or {}
        except (KeyError, IndexError, TypeError, AttributeError):
            raise BackendError(f'openai_compat: unexpected response {str(data)[:200]}') from None
        if refusal or choice.get('finish_reason') == 'content_filter':
            raise BackendError(f'openai_compat: the model refused to answer: {refusal or ""}'[:200])
        if choice.get('finish_reason') == 'length':
            raise BackendError(f'openai_compat: output truncated at max_tokens={body["max_tokens"]}'
                               ' (raise llm_server max_tokens)')
        if req.response_schema and self.json_mode != 'schema':
            text = _strip_fence(text)
        return ReasonResult(text=text, json=text if req.response_schema else '',
                            model=data.get('model', body['model']),
                            input_tokens=usage.get('prompt_tokens', 0),
                            output_tokens=usage.get('completion_tokens', 0))

    async def _send(self, method: str, path: str, body: dict | None = None) -> dict:
        try:
            r = await self.http.request(method, self.base_url + path, json=body)
        except httpx.TransportError as e:                  # connect/read errors, timeouts
            raise BackendError(f'openai_compat: {type(e).__name__}: {e}', retryable=True) \
                from None
        if r.status_code != 200:
            raise BackendError(f'openai_compat {r.status_code}: {r.text[:200]}',
                               retryable=r.status_code in (408, 409, 429) or
                               r.status_code >= 500)
        try:
            return r.json()
        except ValueError:
            raise BackendError(f'openai_compat: non-JSON response {r.text[:200]!r}') from None

    async def close(self) -> None:
        await self.http.aclose()


def _strip_fence(text: str) -> str:
    """Prompt-only JSON often arrives as a ```json fenced block."""
    t = text.strip()
    if t.startswith('```'):
        t = t.split('\n', 1)[1] if '\n' in t else ''
        t = t.rsplit('```', 1)[0]
    return t.strip()
