"""DecisionEngine (§5): attaches the pure commitment machine to any rclpy node.

All server calls are async (§5.2): Jev via ``jev/decide`` (JevDecide), the LLM via ``llm/reason``
(Reason). Policy timers run on the node clock (sim time works); API deadlines on wall time (§5.4).
Every phase transition calls ``on_decision`` (when the app must act) and publishes a
DecisionRecord on ``decisions`` (§12). When a server is down, unready or over budget the engine
degrades exactly as §11 says: no LLM → δ = 0 (Jev only); no Jev → keep the current class, then
the fallback after N_fail.
"""
from __future__ import annotations

import copy
import json
import threading
import time
import uuid
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

from diagnostic_msgs.msg import KeyValue
from jev_ai_msgs.action import Reason
from jev_ai_msgs.msg import DecisionRecord, Question
from jev_ai_msgs.srv import JevDecide
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosidl_runtime_py import set_message_fields

from jev_ai.core.catalog import Catalog, QuestionSet, load_catalog
from jev_ai.core.commitment import Commitment, Step, TokenBucket, decision_due
from jev_ai.core.prompt import SchemaError, context, parse_response, response_schema, \
    system_prompt
from jev_ai.core.records import record_dict

TICK_S = 0.1      # ponytail: one 10 Hz engine timer drives ticks, deadlines and T_wait polling;
#                   tick_hz above 10 needs its own timer.


@dataclass
class Decision:
    """What ``on_decision`` receives: the app (re)executes ``selected``."""

    question_set: str
    decision_id: int
    phase: str
    selected: dict[str, str]
    values: dict[str, float | str] = field(default_factory=dict)
    llm_params: dict[str, float] = field(default_factory=dict)   # clamped (§7), defaults if none
    rationale: str = ''
    flags: list[str] = field(default_factory=list)   # e.g. option_missing (§7)
    final: bool = False
    degraded: bool = False
    generation: int = 0
    act: bool = True                 # False only for KEPT/COMMITTED-after-REVISED (all_phases)
    fused: dict[str, tuple[list[str], list[float]]] = field(default_factory=dict)


class _Set:
    """Runtime state of one question set."""

    def __init__(self, qset: QuestionSet, machine: Commitment, k_max: float):
        self.qset = qset
        self.m = machine
        self.bucket = TokenBucket(k_max)
        self.questions = [Question(name=q.name, type=q.type, instructions=q.instructions,
                                   classes=q.class_names, criteria=q.criteria)
                          for q in qset.questions.values()]
        self.schema = json.dumps(response_schema(qset))
        self.system = ''
        self.state: list[tuple[str, str]] = []   # last encoded state, catalog order
        self.next_tick = 0.0
        self.last_refresh = None
        self.triggers: list[str] = []
        self.app_priority = ''
        self.jev_seq = 0
        self.jev_sent: float | None = None       # wall time of the in-flight Jev call
        self.last_p: dict[str, list[float]] = {}
        self.jev_meta: dict = {}
        self.goal = None                         # in-flight Reason goal handle
        self.llm_params = {n: d for n, (_, _, d) in qset.llm_params.items()}
        self.history: deque[str] = deque(maxlen=3)
        self.subject = ''
        self.directive: tuple[str, int] = ('', 0)   # (issuer, id) of the directive being answered


class DecisionEngine:
    """``autonomous=False`` (decide_server): no ticks, no start/periodic events; decisions happen
    only on ``trigger()``. ``all_phases``: ``on_decision`` also receives the non-acting KEPT and
    COMMITTED steps (``Decision.act`` is False), so a host can finish its goals."""

    def __init__(self, node, catalog: str | dict | Catalog,
                 encode_state: Callable[[str], dict[str, str]],
                 mask: Callable[[str], dict[str, set[str]]] | None = None,
                 require: Callable[[str], dict[str, str]] | None = None,
                 on_decision: Callable[[Decision], None] | None = None,
                 on_degraded: Callable[[str], None] | None = None,
                 jev: str = 'jev/decide', llm: str = 'llm/reason',
                 group: str | None = None, run_id: str | None = None, agent: str | None = None,
                 policy: dict | None = None, current: Callable[[str], dict[str, str]] | None = None,
                 autonomous: bool = True, all_phases: bool = False,
                 on_idle: Callable[[str], None] | None = None):
        self.node = node
        self.cat = catalog if isinstance(catalog, Catalog) else load_catalog(catalog, policy)
        self.encode, self.mask, self.require = encode_state, mask, require
        self.current_cb = current
        self.autonomous, self.all_phases = autonomous, all_phases
        self.on_decision, self.on_degraded, self.on_idle = on_decision, on_degraded, on_idle
        self.agent = agent or node.get_fully_qualified_name()
        self.run_id = run_id or uuid.uuid4().hex[:12]
        self.log = node.get_logger()
        self.lock = threading.RLock()            # ponytail: one lock per engine, not per set
        cbg = ReentrantCallbackGroup()
        self.jev = node.create_client(JevDecide, jev, callback_group=cbg)
        self.llm = ActionClient(node, Reason, llm, callback_group=cbg)
        self.pub = node.create_publisher(
            DecisionRecord, 'decisions',
            QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE))
        self.sets: dict[str, _Set] = {}
        for name, qs in self.cat.question_sets.items():
            m = Commitment(qs, agent=self.agent, run_id=self.run_id, group=group)
            s = self.sets[name] = _Set(qs, m, qs.policy.k_max)
            s.system = system_prompt(qs, self.agent)
            if autonomous:
                s.triggers.append('start')
        self.timer = node.create_timer(TICK_S, self._tick, callback_group=cbg)

    # --- app API (§5.1) --------------------------------------------------------------------------

    def trigger(self, name: str, qset: str | None = None, priority: str = '', *,
                fresh: bool = False, subject: str = '',
                directive: tuple[str, int] | None = None) -> None:
        """An app event: a decision event at the next Jev answer (asked immediately).

        ``name=''``: no trigger, just a tick: ask Jev and decide only if a decision event is due
        (``on_idle(qset)`` is called when none is).
        ``fresh``: drop an in-flight Jev answer and re-encode now (the state just changed).
        ``directive``: (issuer, id) when this decision answers a directive; its noise epoch is
        the directive's (§6.3), so every worker answering it shares the group noise.
        A name containing ':' (``directive:goto``) is recorded as is, otherwise as ``app:<name>``.
        """
        with self.lock:
            for s in self._targets(qset):
                if name:                                         # '' = a tick: ask Jev only
                    s.triggers.append(name if ':' in name else f'app:{name}')
                s.app_priority = max(s.app_priority, priority, key=_prio_rank)
                if subject:
                    s.subject = subject
                if directive:
                    s.directive = directive
                if fresh:
                    s.jev_seq += 1
                    s.jev_sent = None
                self._ask_jev(s)

    def completed(self, question: str) -> None:
        """The current action of ``question`` finished → decision event."""
        with self.lock:
            for s in self.sets.values():
                if question in s.qset.questions:
                    s.m.completed(question)
                    s.triggers.append('completed')
                    self._ask_jev(s)

    def commit_point(self, question: str) -> None:
        """``until_commit`` classes of ``question`` become irreversible."""
        with self.lock:
            for s in self.sets.values():
                if question in s.qset.questions:
                    s.m.commit_point(question)

    def set_subject(self, subject: str, qset: str | None = None) -> None:
        """What the next decisions are about (records' ``subject``; Labels join on it). Safe to
        call from ``encode_state``, e.g. when the encoder picks which detection to describe."""
        with self.lock:
            for s in self._targets(qset):
                s.subject = subject

    def current(self, qset: str) -> dict[str, str]:
        return dict(self.sets[qset].m.current)

    # --- cycle -------------------------------------------------------------------------------------

    def _now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def _tick(self) -> None:
        now = self._now()
        with self.lock:
            for s in self.sets.values():
                pol = s.qset.policy
                self._apply(s, s.m.poll(now))                   # T_wait without an LLM answer
                if s.m.open is None and s.goal is not None:      # resolved: stop paying for it
                    s.goal.cancel_goal_async()
                    s.goal = None
                if s.jev_sent is not None and time.monotonic() - s.jev_sent > pol.jev_timeout:
                    s.jev_seq += 1                               # abandon the late answer
                    s.jev_sent = None
                    self._on_jev(s, None, {'error': 'jev timeout'})
                if not self.autonomous:
                    continue                                     # the host decides when
                if s.last_refresh is None:
                    s.last_refresh = now
                elif now - s.last_refresh >= pol.T_refresh:
                    s.last_refresh = now
                    s.triggers.append('periodic')
                if s.qset.tick_hz > 0 and now >= s.next_tick or s.triggers:
                    s.next_tick = now + (1.0 / s.qset.tick_hz if s.qset.tick_hz > 0 else 0.0)
                    self._ask_jev(s)

    def _targets(self, qset):
        return [self.sets[qset]] if qset else list(self.sets.values())

    def _ask_jev(self, s: _Set) -> None:
        if s.jev_sent is not None:
            return                                               # coalesce while in flight
        try:
            s.state = s.qset.ordered_state(self.encode(s.qset.name))
        except Exception as e:                                   # noqa: BLE001 - app callback
            self.log.error(f'{s.qset.name}: encode_state failed: {e}')
            return
        if not self.jev.service_is_ready():
            self._on_jev(s, None, {'error': 'jev/decide unavailable'})
            return
        req = JevDecide.Request(state=[KeyValue(key=k, value=v) for k, v in s.state],
                                questions=s.questions)
        s.jev_seq += 1
        s.jev_sent = time.monotonic()
        seq = s.jev_seq
        self.jev.call_async(req).add_done_callback(lambda f: self._jev_done(s, seq, f))

    def _jev_done(self, s: _Set, seq: int, fut) -> None:
        with self.lock:
            if seq != s.jev_seq:
                return
            s.jev_sent = None
            try:
                res = fut.result()
            except Exception as e:                               # noqa: BLE001
                return self._on_jev(s, None, {'error': str(e)})
            if not res.success:
                return self._on_jev(s, None, {'error': res.error})
            p = {}
            for q, d, err in zip(s.questions, res.answers, res.question_errors):
                if not err and list(d.classes) == list(q.classes):
                    p[q.name] = list(d.probabilities)
            self._on_jev(s, p, {'model': res.model, 'latency_ms': res.latency_ms,
                                'input_tokens': res.input_tokens})

    def _on_jev(self, s: _Set, p: dict | None, meta: dict) -> None:
        """A Jev answer (or failure): run a decision event if one is due (§5.3)."""
        now = self._now()
        p = p or {}
        masked, required = self._masks(s)
        pol = s.qset.policy
        due = decision_due(s.qset, s.triggers, p, s.last_p, s.m.current, masked, required)
        if not due or not s.state:
            if self.on_idle:
                self.on_idle(s.qset.name)
            return
        if not p and meta.get('error'):
            self.log.warning(f'{s.qset.name}: {meta["error"]}', throttle_duration_sec=5.0)
        triggers, s.triggers = s.triggers, []
        prio, s.app_priority = s.app_priority, ''
        (issuer, did), s.directive = s.directive, ('', 0)
        s.last_p.update(p)
        s.jev_meta = meta
        current = None
        if self.current_cb:
            try:
                current = self.current_cb(s.qset.name) or None
            except Exception as e:                               # noqa: BLE001 - app callback
                self.log.error(f'{s.qset.name}: current failed: {e}')
        step = s.m.on_event(now, state=dict(s.state), p_jev=p, masked=masked, required=required,
                            triggers=triggers, app_priority=prio, current=current,
                            directive_issuer=issuer, directive_id=did)
        self._apply(s, [step])
        if step.request_llm:
            self._ask_llm(s, step, p, masked, now, pol)

    def _ask_llm(self, s: _Set, step: Step, p: dict, masked: dict, now: float, pol) -> None:
        if s.goal is not None:                                   # superseded decision
            s.goal.cancel_goal_async()
            s.goal = None
        if not s.bucket.take(now, step.request_llm):
            self.log.debug(f'{s.qset.name}: augmentation rate-limited')
            return self._apply(s, s.m.on_llm_failed(now, step.generation))
        if not self.llm.server_is_ready():
            return self._apply(s, s.m.on_llm_failed(now, step.generation))
        goal = Reason.Goal(system=s.system, response_schema=s.schema, context=context(
            s.qset, s.state, p, masked=masked, current=s.m.current, history=list(s.history),
            seed=f'{self.run_id}:{self.agent}:{step.decision_id}'))
        self._send_goal(s, goal, step.generation, attempt=0)

    def _send_goal(self, s: _Set, goal, generation: int, attempt: int) -> None:
        def accepted(fut):
            with self.lock:
                gh = fut.result()
                if not gh.accepted:
                    return self._apply(s, s.m.on_llm_failed(self._now(), generation))
                s.goal = gh
                gh.get_result_async().add_done_callback(
                    lambda f: self._llm_done(s, goal, generation, attempt, f))
        self.llm.send_goal_async(goal).add_done_callback(accepted)

    def _llm_done(self, s: _Set, goal, generation: int, attempt: int, fut) -> None:
        with self.lock:
            s.goal = None
            now = self._now()
            o = s.m.open
            if o is None or o.generation != generation:
                return                                           # stale: discarded (§5.2 step 7)
            res = fut.result().result
            # invalid JSON is a schema failure (one retry below), not a backend failure
            if not res.success and res.error != 'response is not valid JSON':
                if res.error != 'cancelled':
                    self.log.warning(f'{s.qset.name}: llm failed: {res.error}',
                                     throttle_duration_sec=5.0)
                return self._apply(s, s.m.on_llm_failed(now, generation))
            try:
                advice = parse_response(s.qset, res.json or res.text)
            except SchemaError as e:
                if attempt == 0:                                 # one bounded retry (§7)
                    return self._send_goal(s, goal, generation, attempt=1)
                self.log.warning(f'{s.qset.name}: llm schema failure: {e}')
                return self._apply(s, s.m.on_llm_failed(now, generation))
            masked, required = self._masks(s)                    # fresh (§5.2 step 7)
            s.llm_params = advice.params
            meta = {'model': res.model, 'latency_ms': res.latency_ms,
                    'input_tokens': res.input_tokens, 'output_tokens': res.output_tokens,
                    'cost_usd': res.cost_usd, 'params': advice.params,
                    'rationale': advice.rationale, 'flags': advice.flags}
            self._apply(s, s.m.on_llm(now, generation=generation, q_llm=advice.q_llm,
                                      masked=masked, required=required), meta)

    def _masks(self, s: _Set) -> tuple[dict, dict]:
        name = s.qset.name
        try:
            masked = (self.mask(name) if self.mask else None) or {}
            required = (self.require(name) if self.require else None) or {}
        except Exception as e:                                   # noqa: BLE001 - app callback
            self.log.error(f'{name}: mask/require failed: {e}')
            return {}, {}
        return masked, required

    # --- output ------------------------------------------------------------------------------------

    def _apply(self, s: _Set, steps: list[Step], llm_meta: dict | None = None) -> None:
        for step in steps:
            rec = record_dict(step, question_set=s.qset.name, agent=self.agent,
                              run_id=self.run_id, group=s.m.group, state=s.state,
                              t_node=self._now(), t_wall=time.time(),
                              catalog_hash=self.cat.catalog_hash, subject=s.subject,
                              policy_hash=self.cat.policy_hash, jev=s.jev_meta, llm=llm_meta)
            msg = DecisionRecord()
            set_message_fields(msg, copy.deepcopy(rec))          # Humble mutates nested lists
            self.pub.publish(msg)
            if step.final:
                s.history.append(f'{step.phase}: ' + ', '.join(
                    f'{q}={c}' for q, c in step.selected.items()))
            if step.degraded and self.on_degraded:
                self.on_degraded(s.qset.name)
            if (step.act or self.all_phases) and self.on_decision:
                values = {k: v for t in step.traces.values() for k, v in t.values.items()}
                self.on_decision(Decision(
                    s.qset.name, step.decision_id, step.phase, dict(step.selected), values,
                    dict(s.llm_params), (llm_meta or {}).get('rationale', ''),
                    list((llm_meta or {}).get('flags', [])), step.final, step.degraded,
                    step.generation, step.act,
                    {qn: (list(t.classes), list(t.fused)) for qn, t in step.traces.items()}))


def _prio_rank(p: str) -> int:
    return ('', 'low', 'normal', 'high', 'reserved').index(p)


def run(make_node: Callable[[], object], args=None, threads: int = 4,
        on_exit: Callable[[object], None] | None = None) -> None:
    """Entry-point body: spin a node on a MultiThreadedExecutor until Ctrl-C or shutdown.

    Under ``ros2 launch`` Ctrl-C reaches a node twice (terminal, then launch), so the second SIGINT
    is ignored while cleaning up instead of surfacing as a KeyboardInterrupt traceback.
    """
    import signal

    import rclpy
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor

    rclpy.init(args=args)
    node = make_node()
    executor = MultiThreadedExecutor(num_threads=threads)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if on_exit:
            on_exit(node)                     # e.g. cancel backend calls: callbacks return now
        executor.shutdown(timeout_sec=2.0)    # in-flight callbacks finish before the node goes
        node.destroy_node()
        rclpy.try_shutdown()
