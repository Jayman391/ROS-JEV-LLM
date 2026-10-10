"""Jev answers from a recorded cassette (FRAMEWORK §8 ``replay``), keyed by hash(state, questions,
model). Cassettes are written by jev_server's ``record_path``. Parameter: ``replay.path``."""
from __future__ import annotations

from dataclasses import astuple

from jev_ai.backends import BackendError, JevAnswer, JevBackend, JevQuestion, JevResult
from jev_ai_servers.common import cassette_key, cassette_load


def key(state, questions: list[JevQuestion], model: str) -> str:
    """The one cassette key, shared by the recorder (jev_server) and this backend."""
    return cassette_key('jev', [list(kv) for kv in state], [astuple(q) for q in questions], model)


def to_record(r: JevResult) -> dict:
    return {'model': r.model, 'question_errors': r.question_errors,
            'input_tokens': r.input_tokens,
            'answers': [None if a is None else [a.probabilities, a.source_confidence]
                        for a in r.answers]}


class ReplayJev(JevBackend):
    name = 'replay'

    def __init__(self, params, api_key=''):
        super().__init__(params, api_key)
        path = params.get('path', '')
        try:
            self.cassette = cassette_load(path)
        except (OSError, ValueError, KeyError) as e:
            raise BackendError(f'replay.path {path!r}: {e}') from None

    async def decide(self, state, questions, model) -> JevResult:
        row = self.cassette.get(key(state, questions, model))
        if row is None:
            names = ', '.join(q.name for q in questions)
            raise BackendError(f'replay miss: no recorded answer for questions [{names}] '
                               f'with this state and model {model!r}')
        return JevResult(model=row['model'], question_errors=row['question_errors'],
                         input_tokens=row['input_tokens'],
                         answers=[None if a is None else JevAnswer(a[0], a[1])
                                  for a in row['answers']])
