"""decide_server (FRAMEWORK §2, §5.1, D2): the one Python engine, hosted for remote clients.

C++, behaviour trees and the CLI send ``Decide`` goals (action ``decide``). The server keeps one
non-autonomous DecisionEngine per goal ``agent``; the goal supplies what an in-process app's
callbacks would (state, mask, require, current), and decisions come back as feedback
(PROVISIONAL, REVISED) and a result (COMMITTED | KEPT | FALLBACK | FORCED). So C++ gets exactly
the Python engine's decisions: same fusion, noise, commitment, records.

- A goal with triggers or a directive is a decision event. A goal with neither is a **tick**, like
  the in-process engine's ``tick_hz``: Jev is asked, and a decision happens only if an event is
  due (§5.3: argmax change, current class masked, ...); otherwise the standing decision is the
  result (same ``decision_id``).

- A newer goal for the same (agent, question set) preempts the older one, which is aborted with
  the last decision it saw: the newest state wins.
- Triggers ``completed:<question>`` and ``commit_point:<question>`` call the engine's
  ``completed()`` / ``commit_point()`` before the decision (remote clients have no other hook).
- An invalid goal (unknown set, wrong state keys) is aborted with ``rationale = "error: ..."``.

ponytail: a plain node, not lifecycle: there is no backend or key to configure. Engines are never
evicted; add an idle timeout when agents churn.
"""
from __future__ import annotations

import json
import threading
import uuid

from diagnostic_msgs.msg import KeyValue
from jev_ai_msgs.action import Decide
from jev_ai_msgs.msg import ClassRef, Distribution
from jev_ai_msgs.msg import Decision as DecisionMsg
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.task import Future

from jev_ai.core.catalog import load_catalog
from jev_ai.core.fusion import confidence
from jev_ai.ros.engine import Decision, DecisionEngine, run


def to_msg(d: Decision) -> DecisionMsg:
    fused = [Distribution(question=q, classes=c, probabilities=p,
                          confidence=confidence(p) if p else float('nan'),
                          source_confidence=float('nan'))
             for q, (c, p) in d.fused.items()]
    return DecisionMsg(
        decision_id=d.decision_id, generation=d.generation, phase=d.phase,
        selected=[ClassRef(question=q, cls=c) for q, c in d.selected.items()], fused=fused,
        values=[KeyValue(key=k, value=str(v)) for k, v in d.values.items()],
        llm_params=json.dumps(d.llm_params), rationale=d.rationale)


class _Pending:
    """One goal in flight for an (agent, question set)."""

    def __init__(self, handle, executor):
        self.handle = handle
        # an executor-bound future with a callback wakes the executor when it completes, so the
        # awaiting goal resumes at once (otherwise only at the next timer wake)
        self.done = Future(executor=executor)
        self.done.add_done_callback(lambda _: None)
        self.last = DecisionMsg()

    def finish(self, status: str, msg: DecisionMsg | None = None):
        if not self.done.done():
            self.done.set_result((status, msg or self.last))


class _Hosted:
    """One agent's engine and the latest goal inputs per question set."""

    def __init__(self, server: 'DecideServer', agent: str):
        self.inputs: dict[str, Decide.Goal] = {}
        self.pending: dict[str, _Pending] = {}
        self.last_final: dict[str, DecisionMsg] = {}
        self.engine = DecisionEngine(
            server, server.catalog, agent=agent, run_id=server.run_id,
            encode_state=lambda qs: {kv.key: kv.value for kv in self.inputs[qs].state},
            mask=lambda qs: _classes(self.inputs[qs].mask),
            require=lambda qs: {r.question: r.cls for r in self.inputs[qs].require},
            current=lambda qs: {r.question: r.cls for r in self.inputs[qs].current},
            on_decision=self._on_decision, on_idle=self._on_idle, autonomous=False,
            all_phases=True)

    def _on_decision(self, d: Decision):
        msg = to_msg(d)
        if d.final:
            self.last_final[d.question_set] = msg
        p = self.pending.get(d.question_set)
        if p is None:
            return
        p.last = msg
        if d.final:
            p.finish('succeeded', msg)
        elif d.act:
            p.handle.publish_feedback(Decide.Feedback(decision=msg))

    def _on_idle(self, qs: str):
        """A tick goal and no decision event was due: the standing decision answers it."""
        p = self.pending.get(qs)
        if p is not None and qs in self.last_final:
            p.finish('succeeded', self.last_final[qs])


class DecideServer(Node):
    def __init__(self, **kwargs):
        super().__init__('decide_server', **kwargs)
        path = self.declare_parameter('catalog', '').value
        if not path:
            raise SystemExit('decide_server: parameter "catalog" (a catalog YAML path) is required')
        self.catalog = load_catalog(path)
        self.run_id = self.declare_parameter('run_id', '').value or uuid.uuid4().hex[:12]
        self.lock = threading.Lock()
        self.hosted: dict[str, _Hosted] = {}
        ActionServer(self, Decide, 'decide', execute_callback=self._execute,
                     goal_callback=self._goal, cancel_callback=self._cancel,
                     callback_group=ReentrantCallbackGroup())
        self.get_logger().info(f'hosting catalog {self.catalog.name!r} '
                               f'({", ".join(self.catalog.question_sets)}), run_id {self.run_id}')

    def _goal(self, goal: Decide.Goal) -> GoalResponse:
        ok = goal.agent and goal.question_set in self.catalog.question_sets
        return GoalResponse.ACCEPT if ok else GoalResponse.REJECT

    def _cancel(self, handle) -> CancelResponse:
        with self.lock:
            for h in self.hosted.values():
                for p in h.pending.values():
                    if p.handle is handle:
                        p.finish('canceled')
        return CancelResponse.ACCEPT

    async def _execute(self, handle):
        goal: Decide.Goal = handle.request
        qs = goal.question_set
        error = ''
        try:
            self.catalog.question_sets[qs].ordered_state({kv.key: kv.value for kv in goal.state})
        except Exception as e:                                   # noqa: BLE001 - CatalogError
            error = str(e)
        if error:
            handle.abort()
            return Decide.Result(decision=DecisionMsg(rationale=f'error: {error}'))
        with self.lock:
            h = self.hosted.get(goal.agent)
            if h is None:
                h = self.hosted[goal.agent] = _Hosted(self, goal.agent)
            pending = _Pending(handle, self.executor)
            with h.engine.lock:
                old = h.pending.get(qs)
                if old is not None:
                    old.finish('preempted')
                h.pending[qs] = pending
                h.inputs[qs] = goal
                for t in goal.triggers:
                    kind, _, q = t.partition(':')
                    if kind == 'completed':
                        h.engine.completed(q)
                    elif kind == 'commit_point':
                        h.engine.commit_point(q)
                directive = (goal.directive_issuer, goal.directive_id) \
                    if goal.directive_issuer else None
                # no triggers and no directive = a tick (decides only if an event is due, §5.3)
                name = 'remote:' + ','.join(goal.triggers) if goal.triggers else \
                    'directive:remote' if directive else ''
                h.engine.trigger(name, qs, fresh=True, subject=goal.subject, directive=directive)
        status, msg = await pending.done
        with self.lock, h.engine.lock:
            if h.pending.get(qs) is pending:
                del h.pending[qs]
        if status == 'succeeded':
            handle.succeed()
        elif status == 'canceled':
            handle.canceled()
        else:
            handle.abort()
        return Decide.Result(decision=msg)


def _classes(refs) -> dict[str, set[str]]:
    out: dict[str, set[str]] = {}
    for r in refs:
        out.setdefault(r.question, set()).add(r.cls)
    return out


def main(args=None):
    run(DecideServer, args, threads=8)
