"""``mock`` LLM backend (§9): keyless, deterministic, schema-valid.

For the jev_ai response schema (§7) it echoes the Jev distributions found in the context
("Fast model (Jev) probabilities:" lines, see ``jev_ai.core.prompt.context``) mixed with a seeded
Dirichlet draw; classes listed as "forbidden now" get weight 0. Any other schema gets a minimal
valid instance. Params:

- ``seed`` (0): content seed; the same request always gets the same answer;
- ``disagreement`` (0.3): 0 = pure echo of Jev, 1 = pure seeded Dirichlet;
- ``latency_s`` (0.3): simulated, streamed in a few deltas so feedback and cancel work;
- ``fail_rate`` (0.0): seeded fault injection (retryable BackendError);
- ``invalid_json_rate`` (0.0): seeded truncated output (exercises the engine's retry, §7).
"""
from __future__ import annotations

import asyncio
import json
import random
import re

from jev_ai.backends import BackendError, Feedback, LlmBackend, ReasonRequest, ReasonResult

_DEFAULT = re.compile(r'default (-?[\d.]+(?:e-?\d+)?)')


class MockLlm(LlmBackend):
    name = 'mock'

    def __init__(self, params, api_key=''):
        super().__init__(params, api_key)
        self.seed = params.get('seed', 0)
        self.faults = random.Random(f'faults|{self.seed}')   # per instance: retries can pass

    async def reason(self, req: ReasonRequest, feedback: Feedback) -> ReasonResult:
        feedback('started', '')
        if self.faults.random() < float(self.params.get('fail_rate', 0.0)):
            raise BackendError('mock: injected failure', retryable=True)
        rng = random.Random(f'{self.seed}|{req.system}|{req.context}|{req.response_schema}')
        if req.response_schema:
            text = json.dumps(_instance(_schema(req.response_schema), req.context, rng,
                                        float(self.params.get('disagreement', 0.3))))
            if self.faults.random() < float(self.params.get('invalid_json_rate', 0.0)):
                text = text[:len(text) // 2]
        else:
            text = 'mock: no response schema, so no advice.'

        chunks = 4
        step = float(self.params.get('latency_s', 0.3)) / (chunks + 1)
        size = -(-len(text) // chunks)
        await asyncio.sleep(step)
        for i in range(chunks):
            await asyncio.sleep(step)
            if i == 0:
                feedback('first_token', '')
            feedback('streaming', text[i * size:(i + 1) * size])
        feedback('done', '')
        tokens_in = (len(req.system) + len(req.context) + len(req.response_schema)) // 4
        return ReasonResult(text=text, json=text if req.response_schema else '', model='mock',
                            input_tokens=tokens_in, output_tokens=len(text) // 4, cost_usd=0.0)


def _schema(text: str) -> dict:
    try:
        return json.loads(text)
    except ValueError as e:
        raise BackendError(f'response_schema is not valid JSON: {e}') from None


def jev_lines(context: str) -> tuple[dict[str, dict[str, float]], dict[str, set[str]]]:
    """Parse the Jev distributions and masks out of a jev_ai context (prompt.context format)."""
    p_jev, masked = {}, {}
    lines = context.split('\n')
    try:
        start = lines.index('Fast model (Jev) probabilities:') + 1
    except ValueError:
        return p_jev, masked
    q = None
    for line in lines[start:]:
        if not line:
            break                                 # the history block follows a blank line
        if line.startswith('- '):
            q, _, rest = line[2:].partition(': ')
            if rest != '(unavailable)':
                p_jev[q] = {c: float(p) for c, p in
                            (item.rsplit(' ', 1) for item in rest.split(', '))}
        elif line.startswith('  forbidden now: ') and q:
            masked[q] = set(line[len('  forbidden now: '):].split(', '))
    return p_jev, masked


def _weights(props: dict, context: str, rng: random.Random, d: float) -> dict:
    p_jev, masked = jev_lines(context)
    out = {}
    for q, spec in props.items():
        classes = list(spec.get('properties', {}))
        draw = [rng.gammavariate(1.0, 1.0) for _ in classes]
        total = sum(draw) or 1.0
        p = p_jev.get(q, {})
        w = [0.0 if c in masked.get(q, ()) else
             ((1 - d) * p[c] + d * x / total if c in p else x / total)
             for c, x in zip(classes, draw)]
        if not any(w):                            # Jev's mass was all on masked classes
            w = [0.0 if c in masked.get(q, ()) else 1.0 for c in classes]
        out[q] = {c: round(x, 4) for c, x in zip(classes, w)}
    return out


def _instance(schema: dict, context: str, rng: random.Random, d: float, key: str = ''):
    """A minimal instance of a JSON Schema; ``weights`` is filled from the context (§7)."""
    if 'enum' in schema:
        return schema['enum'][0]
    kind = schema.get('type')
    if kind == 'object':
        props = schema.get('properties', {})
        if key == 'weights':
            return _weights(props, context, rng, d)
        return {k: _instance(v, context, rng, d, k) for k, v in props.items()}
    if kind == 'array':
        return []
    if kind in ('number', 'integer'):
        m = _DEFAULT.search(schema.get('description', ''))
        x = float(m.group(1)) if m else 0.0
        return int(x) if kind == 'integer' else x
    if kind == 'boolean':
        return False
    return 'mock'
