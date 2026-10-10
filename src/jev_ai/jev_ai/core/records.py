"""DecisionRecord as a plain dict (§3, §12): one shape for the engine, the logger's JSONL, the
dashboard and the metrics.

The dict mirrors ``rosidl_runtime_py.message_to_ordereddict(DecisionRecord)`` field for field, so
the engine builds the ROS message from it and offline tools read JSONL without ROS. Non-finite
floats (masked logits are -inf, missing confidences NaN) become ``null`` only when serialised
(``json_safe``), because JSON has no infinities.
"""
from __future__ import annotations

import json
import math

from jev_ai.core.fusion import Trace

TRACE_FIELDS = ('question', 'classes', 'p_jev', 'jev_confidence', 'q_llm', 'llm_age_s', 'beta',
                'masked', 'required', 'logits', 'tau', 'kappa', 'noise', 'group_noise',
                'selected', 'propensity', 'irreversible_path')


def stamp(t: float) -> dict:
    sec = math.floor(t)
    return {'sec': int(sec), 'nanosec': int(round((t - sec) * 1e9)) % 1_000_000_000}


def from_stamp(s: dict) -> float:
    return s['sec'] + s['nanosec'] * 1e-9


def trace_dict(t: Trace) -> dict:
    return {f: list(getattr(t, f)) if isinstance(getattr(t, f), (list, tuple)) else getattr(t, f)
            for f in TRACE_FIELDS}


def record_dict(step, *, question_set: str, agent: str, run_id: str, group: str,
                state: list[tuple[str, str]], t_node: float, t_wall: float,
                catalog_hash: str = '', policy_hash: str = '', subject: str = '',
                jev: dict | None = None, llm: dict | None = None) -> dict:
    """``step`` is a jev_ai.core.commitment.Step. ``jev``: model, latency_ms, input_tokens.
    ``llm``: model, latency_ms, input_tokens, output_tokens, cost_usd, params, rationale, flags
    (only for steps that used an LLM answer)."""
    jev, llm = jev or {}, llm or {}
    return {
        'header': {'stamp': stamp(t_node), 'frame_id': ''},
        'wall_stamp': stamp(t_wall),
        'run_id': run_id,
        'agent': agent,
        'group': group,
        'question_set': question_set,
        'decision_id': step.decision_id,
        'generation': step.generation,
        'phase': step.phase,
        'triggers': list(step.triggers),
        'subject': subject,
        'state': [{'key': k, 'value': v} for k, v in state],
        'questions': [trace_dict(t) for t in step.traces.values()],
        'llm_params': json.dumps(llm.get('params') or {}),
        'llm_rationale': llm.get('rationale', ''),
        'llm_flags': list(llm.get('flags', [])),
        'catalog_hash': catalog_hash,
        'policy_hash': policy_hash,
        'jev_model': jev.get('model', ''),
        'llm_model': llm.get('model', ''),
        'jev_latency_ms': float(jev.get('latency_ms', 0.0)),
        'llm_latency_ms': float(llm.get('latency_ms', 0.0)),
        'jev_input_tokens': int(jev.get('input_tokens', 0)),
        'llm_input_tokens': int(llm.get('input_tokens', 0)),
        'llm_output_tokens': int(llm.get('output_tokens', 0)),
        'cost_usd': float(llm.get('cost_usd', 0.0) or 0.0),
    }


def json_safe(obj):
    """Replace NaN/±inf with None, recursively (JSON, and browsers' JSON.parse, reject them)."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    return obj


def dumps(record: dict) -> str:
    return json.dumps(json_safe(record), separators=(',', ':'))
