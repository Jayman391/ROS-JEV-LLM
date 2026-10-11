"""Commitment machine, decision events, augmentation priority, rate limit (§5.2-§5.3, §7)."""
from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field, replace

from jev_ai.core.catalog import SELECT_ACTS, QuestionSet
from jev_ai.core.fusion import NoiseKey, Trace, decide_question, epoch_key

PROVISIONAL, REVISED, KEPT = 'PROVISIONAL', 'REVISED', 'KEPT'
COMMITTED, FALLBACK, FORCED = 'COMMITTED', 'FALLBACK', 'FORCED'
PRIORITIES = ('low', 'normal', 'high', 'reserved')
TRIGGER_PRIORITY = {'periodic': 'low', 'low_confidence': 'high', 'irreversible_pending': 'reserved'}


def revise_or_keep(provisional: Trace, revised: Trace) -> str:
    """Act-then-revise rule (§5.3): which class stands once augmentation arrives.

    Default: adopt the augmented selection whenever it differs (different -> REVISED,
    same -> KEPT). Both traces share the same Gumbel noise, so they differ only when
    augmentation moved the fused distribution enough to change the winner. Dwell is not
    re-checked: a revision belongs to the decision event that produced the provisional and
    inherits that event's switch permission, already applied inside decide_question.
    """
    return revised.selected


def argmax_changed(prev: list[float] | None, new: list[float], margin: float) -> bool:
    """Decision event: Jev's argmax moved, by more than `margin` in probability."""
    if not prev:
        return True
    a = max(range(len(prev)), key=prev.__getitem__)
    b = max(range(len(new)), key=new.__getitem__)
    return a != b and new[b] - new[a] > margin


def decision_due(qset: QuestionSet, triggers, p: dict, last_p: dict, current: dict,
                 masked: dict, required: dict, pending: bool = False) -> bool:
    """§5.3 decision event: an app/periodic trigger, no Jev answer or no current class yet, Jev's
    argmax moved, or the current class became masked or differs from a require().

    pending = an augmentation is still open: Jev's argmax moving is then *not* an event, so a
    flickering Jev cannot supersede the LLM answer on its way (live, 2x sim: Haiku's ~7 sim s
    answer lost to a new decision every ~1.3 s)."""
    return bool(triggers) or not p or not current or not pending and any(
        argmax_changed(last_p.get(qn), p[qn], qset.questions[qn].policy.argmax_margin)
        for qn in p) or any(current.get(qn) in m for qn, m in masked.items()) or any(
        current.get(qn) != c for qn, c in required.items())


def augmentation_priority(triggers, min_c: float, c_low: float, pending: bool = False,
                          app_priority: str = '') -> str:
    """§7 trigger table -> the highest priority among what fired (default 'normal')."""
    names = list(triggers) + (['low_confidence'] if min_c < c_low else []) + \
        (['irreversible_pending'] if pending else [])
    ranks = [PRIORITIES.index(TRIGGER_PRIORITY.get(n, 'normal')) for n in names]
    if app_priority:
        ranks.append(PRIORITIES.index(app_priority))
    return PRIORITIES[max(ranks, default=1)]


class TokenBucket:
    """Per-engine augmentation cap (§7): k_max requests/min.

    One token is held back for the reserved (irreversible) slot; 'low' only spends from a
    bucket at least half full. ponytail: with k_max < 2 nothing can be held back.
    """

    def __init__(self, per_min: float, now: float = 0.0):
        self.cap = max(1.0, per_min)
        self.rate = per_min / 60.0
        self.tokens = self.cap
        self.t = now

    def take(self, now: float, priority: str = 'normal') -> bool:
        self.tokens = min(self.cap, self.tokens + max(0.0, now - self.t) * self.rate)
        self.t = now
        keep = {'reserved': 0.0, 'high': 1.0, 'normal': 1.0, 'low': self.cap / 2}[priority]
        if self.tokens - 1.0 >= min(keep, self.cap - 1.0):
            self.tokens -= 1.0
            return True
        return False


@dataclass
class Step:
    """One emission of the commitment machine (one Decision message / one DecisionRecord)."""

    decision_id: int
    generation: int
    phase: str
    selected: dict[str, str]
    traces: dict[str, Trace]
    final: bool
    act: bool                      # call on_decision: the app should (re)execute `selected`
    triggers: list[str] = field(default_factory=list)
    request_llm: str = ''          # '' or the augmentation priority to request with
    state_hash: str = ''
    mask_hash: str = ''
    degraded: bool = False         # N_fail consecutive Jev failures -> on_degraded


@dataclass
class _Open:
    """A decision awaiting augmentation."""

    decision_id: int
    generation: int
    key: NoiseKey
    state_hash: str
    mask_hash: str
    deadline: float
    p_jev: dict
    current: dict
    may_switch: dict
    traces: dict
    triggers: list


class Commitment:
    """PROVISIONAL -> (REVISED | KEPT) -> COMMITTED, plus FALLBACK and FORCED (§5.3).

    Pure: the caller supplies node-clock time, Jev answers, fresh masks and LLM results, and
    executes the returned Steps. One instance per (agent, question set).
    """

    def __init__(self, qset: QuestionSet, *, agent: str, run_id: str, group: str | None = None):
        self.qs = qset
        self.pol = qset.policy
        self.agent = agent
        self.run_id = run_id
        self.group = group or qset.group or agent
        self.decision_id = 0
        self.generation = 0
        self.current: dict[str, str] = {}
        self.last_switch: dict[str, float] = {}
        self.committed_points: set[str] = set()
        self.jev_failures = 0
        self.open: _Open | None = None
        self.llm_cache = None          # (state_hash, mask_hash, t, q_llm)

    # --- app hooks ---------------------------------------------------------------------------

    def commit_point(self, question: str):
        """`until_commit` classes of `question` become irreversible."""
        self.committed_points.add(question)

    def completed(self, question: str):
        self.committed_points.discard(question)

    # --- events ------------------------------------------------------------------------------

    def on_event(self, now: float, *, state: dict, p_jev: dict, masked: dict | None = None,
                 required: dict | None = None, triggers=(), current: dict | None = None,
                 directive_issuer: str = '', directive_id: int = 0,
                 app_priority: str = '') -> Step:
        """A decision event (§5.3): sample, and maybe request augmentation."""
        masked = {q: set(v) for q, v in (masked or {}).items()}
        required = {q: c for q, c in (required or {}).items() if c}
        if current:
            self.current.update(current)
        self.generation += 1
        self.decision_id += 1
        questions = self.qs.questions
        sh, mh = _hash(state), _mask_hash(masked, required)

        failed = all(p_jev.get(qn) is None for qn in questions)
        self.jev_failures = self.jev_failures + 1 if failed else 0
        degraded = self.jev_failures >= self.pol.N_fail

        epoch, directive = epoch_key(now, self.pol.epoch_s, directive_issuer, directive_id)
        key = NoiseKey(self.run_id, self.agent, self.group, epoch, directive)
        q_cached, age = self._cached_llm(sh, mh, now)
        cur = dict(self.current)
        may = {qn: self._may_switch(qn, now, masked.get(qn, set())) for qn in questions}

        traces = {}
        for qn, q in questions.items():
            t = decide_question(q, p_jev.get(qn), key=key, q_llm=q_cached.get(qn),
                                llm_age_s=age, masked=masked.get(qn, ()),
                                required=required.get(qn, ''), current=cur.get(qn, ''),
                                irreversible=q.irreversible(qn in self.committed_points),
                                may_switch=may[qn], committed=self._held(qn, cur))
            if degraded and q.act in SELECT_ACTS and not t.required:
                t.selected, t.path = q.fallback, 'degraded'
            traces[qn] = t

        selecting = [qn for qn, q in questions.items() if q.act in SELECT_ACTS]
        pending = any(t.path == 'irreversible_pending' for t in traces.values())
        min_c = min((t.jev_confidence for t in traces.values() if t.p_jev), default=1.0)
        triggers = list(triggers)
        if min_c < self.pol.c_low:
            triggers.append('low_confidence')
        if pending:
            triggers.append('irreversible_pending')

        self.open = None
        meta = dict(triggers=triggers, state_hash=sh, mask_hash=mh, degraded=degraded)
        if selecting and all(qn in required for qn in selecting):
            return self._emit(now, FORCED, traces, final=True, act=True, **meta)
        if degraded:
            return self._emit(now, FALLBACK, traces, final=True, act=True, **meta)
        # skip the LLM when every gated question (skip_llm_above_c set) is confident enough;
        # a set-wide value gates on all questions, a per-question one only on that question
        gate = {qn: q.policy.skip_llm_above_c for qn, q in questions.items()
                if q.policy.skip_llm_above_c is not None}
        skip = bool(gate) and all(traces[qn].jev_confidence >= c for qn, c in gate.items()
                                  if traces[qn].p_jev)
        if not pending and (not self.qs.augment or skip):
            return self._emit(now, COMMITTED, traces, final=True, act=True, **meta)

        prio = augmentation_priority(triggers, min_c, self.pol.c_low, pending, app_priority)
        self.open = _Open(self.decision_id, self.generation, key, sh, mh,
                          now + self.pol.T_wait, dict(p_jev), cur, may, traces, triggers)
        return self._emit(now, PROVISIONAL, traces, final=False, act=True, request_llm=prio,
                          **meta)

    def on_llm(self, now: float, *, generation: int, q_llm: dict, masked: dict | None = None,
               required: dict | None = None) -> list[Step]:
        """Augmentation arrived: re-fuse with the same noise (§5.2 step 7).

        Discarded ([]) when the generation advanced or the freshly computed masks differ.
        """
        o = self.open
        masked = {q: set(v) for q, v in (masked or {}).items()}
        required = {q: c for q, c in (required or {}).items() if c}
        if o is None or generation != o.generation or _mask_hash(masked, required) != o.mask_hash:
            return []
        self.llm_cache = (o.state_hash, o.mask_hash, now, q_llm)
        self.open = None

        traces, fell_back = {}, False
        for qn, q in self.qs.questions.items():
            prov = o.traces[qn]
            t = decide_question(q, o.p_jev.get(qn), key=o.key, q_llm=q_llm.get(qn),
                                masked=masked.get(qn, ()), required=required.get(qn, ''),
                                current=o.current.get(qn, ''),
                                irreversible=q.irreversible(qn in self.committed_points),
                                # only real weights for *this* question count as augmentation
                                augmented=bool(q_llm.get(qn)), may_switch=o.may_switch[qn],
                                committed=self._held(qn, o.current))
            if t.path == 'irreversible_pending':
                # no LLM weights for this question: same as a timeout (§11)
                t.selected, t.path = q.fallback, 'fallback'
                fell_back = True
            elif q.act in SELECT_ACTS:
                t.selected = revise_or_keep(prov, t)
            traces[qn] = t

        meta = dict(triggers=o.triggers, state_hash=o.state_hash, mask_hash=o.mask_hash)
        if fell_back:
            return [self._emit(now, FALLBACK, traces, final=True, act=True, decision=o, **meta)]
        if _selected(traces) != _selected(o.traces):
            return [self._emit(now, REVISED, traces, final=False, act=True, decision=o, **meta),
                    self._emit(now, COMMITTED, traces, final=True, act=False, decision=o, **meta)]
        return [self._emit(now, KEPT, traces, final=True, act=False, decision=o, **meta)]

    def on_llm_failed(self, now: float, generation: int) -> list[Step]:
        """LLM error, invalid schema after retry, or open breaker: delta = 0 (§11)."""
        if self.open is None or generation != self.open.generation:
            return []
        return self._resolve_without_llm(now)

    def poll(self, now: float) -> list[Step]:
        """Call on a timer: resolves the open decision at T_wait."""
        if self.open is None or now < self.open.deadline:
            return []
        return self._resolve_without_llm(now)

    # --- internals ---------------------------------------------------------------------------

    def _resolve_without_llm(self, now: float) -> list[Step]:
        o, self.open = self.open, None
        traces, fell_back = {}, False
        for qn, t in o.traces.items():
            if t.path == 'irreversible_pending':   # §11: a timeout never commits irreversibly
                t = replace(t, selected=self.qs.questions[qn].fallback, path='fallback')
                fell_back = True
            traces[qn] = t
        meta = dict(triggers=o.triggers, state_hash=o.state_hash, mask_hash=o.mask_hash)
        if fell_back:
            return [self._emit(now, FALLBACK, traces, final=True, act=True, decision=o, **meta)]
        return [self._emit(now, COMMITTED, traces, final=True, act=False, decision=o, **meta)]

    def _emit(self, now, phase, traces, *, final, act, decision: _Open | None = None,
              **meta) -> Step:
        for qn, t in traces.items():
            if self.qs.questions[qn].act in SELECT_ACTS and t.selected:
                if t.selected != self.current.get(qn):
                    self.last_switch[qn] = now
                self.current[qn] = t.selected
        did = decision.decision_id if decision else self.decision_id
        gen = decision.generation if decision else self.generation
        return Step(decision_id=did, generation=gen, phase=phase, selected=_selected(traces),
                    traces=traces, final=final, act=act, **meta)

    def _held(self, qn: str, current: dict) -> bool:
        """The current class is an ``until_commit`` class past commit_point(): it is kept until
        completed() (A5 decision; masks, require and a winning irreversible class still win)."""
        if qn not in self.committed_points:
            return False
        q = self.qs.questions[qn]
        return current.get(qn) in q.irreversible(True) - q.irreversible(False)

    def _may_switch(self, qn: str, now: float, masked_q: set) -> bool:
        cur = self.current.get(qn)
        if not cur or cur in masked_q:           # mask-forced switches are immediate
            return True
        return now - self.last_switch.get(qn, -math.inf) >= self.pol.T_dwell

    def _cached_llm(self, state_hash: str, mask_hash: str, now: float) -> tuple[dict, float]:
        if self.llm_cache is None:
            return {}, 0.0
        sh, mh, t, q = self.llm_cache
        if (sh, mh) == (state_hash, mask_hash) or self.pol.reuse_stale_llm:
            return q, max(0.0, now - t)
        return {}, 0.0


def _selected(traces: dict[str, Trace]) -> dict[str, str]:
    return {qn: t.selected for qn, t in traces.items() if t.selected}


def _mask_hash(masked: dict, required: dict) -> str:
    return _hash({'mask': {q: sorted(v) for q, v in masked.items() if v}, 'require': required})


def _hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:16]
