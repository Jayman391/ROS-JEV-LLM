"""arm_agent: one per arm (namespace /arm_x), a jev_ai worker (DECISION_MODEL §4).

Routine path, no model: an op is pending for this arm and its pre-op readings are nominal -> run
it. Otherwise the ``op_check`` set decides (proceed / retry / tool_change / hand_off). While
waiting for a late fasten partner, the ``coop`` set decides wait / abort_step. Every decision is
acted on only when final: an operation can't be taken back.

Reports to /line_supervisor: fault (jam, sensor_dropout), recovered, hand_off, tool_change.
Directives from it: pause, resume, inspect_tool, tool_change. The Hierarchy has no engine: a
pause is not a decision, and the arm triggers op_check itself when one is needed.
"""
from __future__ import annotations

import math

from ament_index_python.packages import get_package_share_directory
from line_interfaces.action import Operate
from line_interfaces.msg import ArmStatus, LineState
from rclpy.action import ActionClient
from rclpy.node import Node

from assembly_line import scenario
from jev_ai import Decision, DecisionEngine, Hierarchy
from jev_ai.ros.engine import run

SUPERVISOR = '/line_supervisor'
HAND_OFF_WAIT_S = 15.0     # no reroute by then: check the op again


def grip_word(g: float) -> str:
    return 'ok' if g >= 0.8 else 'marginal' if g >= 0.5 else 'slipping'


def wear_word(w: float) -> str:
    return 'low' if w < 0.3 else 'moderate' if w < 0.6 else 'high' if w < 0.8 else 'worn out'


PARTNER_WORDS = {'idle': 'not ready yet', 'coop_wait': 'ready and waiting',
                 'working': 'busy with another operation', 'regrip': 'regripping',
                 'tool_change': 'changing its tool', 'jammed': 'jammed'}


def routine_ok(s: ArmStatus, p: dict, inspect: str) -> bool:
    """The pre-op readings are nominal: run the op without asking (DECISION_MODEL §4.1)."""
    return (not s.dropout and not inspect and
            (math.isnan(s.grip) or s.grip >= 0.8) and
            (math.isnan(s.pose_mm) or s.pose_mm <= p['pose_tol_mm']) and
            (math.isnan(s.wear_est) or s.wear_est < p['wear_high']))


def encode_op_check(s: ArmStatus, part, p: dict, inspect: str, alternatives: list[str]) -> dict:
    """``part``: the PartState of s.part_id (product, n_ops), or None."""
    n = part.n_ops if part else s.op_index + 1
    product = part.product if part else '?'
    role = {'primary': f'; two-arm fasten, this arm screws while {s.partner} holds',
            'partner': f'; two-arm fasten, this arm holds while {s.partner} screws'}.get(s.role, '')
    operation = (f'{s.op_kind}, operation {s.op_index + 1} of {n} on part {s.part_id} '
                 f'(product {product}); {s.regrips} regrip(s) so far{role}')
    if s.dropout:
        grip = 'unknown (sensor dropout)'
    elif not math.isnan(s.grip):
        grip = f'{grip_word(s.grip)} ({s.grip:.2f})'
    elif not math.isnan(s.partner_grip):
        grip = f'partner grip {grip_word(s.partner_grip)} ({s.partner_grip:.2f})'
    else:
        grip = f'not applicable to {s.op_kind}'
    tol = p['pose_tol_mm']
    alignment = 'unknown (sensor dropout)' if math.isnan(s.pose_mm) else \
        f'{"within tolerance" if s.pose_mm <= tol else "misaligned"} ' \
        f'({s.pose_mm:.1f} mm offset, tolerance {tol:.0f} mm)'
    tool = 'wear unknown (sensor dropout)' if math.isnan(s.wear_est) else \
        f'wear {wear_word(s.wear_est)} ({s.wear_est:.0%} est.)'
    tool += f', {s.ops_since_change} operations since the last tool change'
    sensors = 'dropout: grip, alignment and wear readings unavailable' if s.dropout else 'nominal'
    ctx = [f'supervisor asked to inspect the tool ({inspect})'] if inspect else []
    ctx.append(f'{s.queue} part(s) queued for this arm')
    ctx.append(f'other arm(s) able to do it: {", ".join(alternatives)}' if alternatives else
               'no other arm can do this operation now')
    return {'operation': operation, 'grip': grip, 'alignment': alignment, 'tool': tool,
            'sensors': sensors, 'context': '; '.join(ctx)}


def encode_coop(s: ArmStatus, p: dict) -> dict:
    if s.state != 'coop_wait':
        return {'step': 'no two-arm step under way', 'partner': 'none', 'waiting': 'not waiting',
                'queue': f'{s.queue} part(s) queued'}
    me = 'screws' if s.role == 'primary' else 'holds'
    late = p['coop_late_s']
    w = s.coop_wait_s
    bucket = 'on time' if w < late else 'late' if w < 3 * late else 'very late'
    partner = f'{s.partner} {PARTNER_WORDS.get(s.partner_state, s.partner_state)}'
    if not math.isnan(s.partner_grip):
        partner += f', its grip {grip_word(s.partner_grip)} ({s.partner_grip:.2f})'
    return {'step': f'fasten on part {s.part_id}: this arm {me}, {s.partner} is the partner',
            'partner': partner,
            'waiting': f'{bucket}: waited {w:.0f} s (usually ready within {late:.0f} s; '
                       f'the step is abandoned at {p["coop_max_s"]:.0f} s)',
            'queue': f'{s.queue} part(s) queued behind'}


class ArmAgent(Node):
    def __init__(self):
        super().__init__('arm_agent')
        self.p = scenario.from_node(self)
        self.name = self.get_namespace().strip('/') or 'arm_a'
        share = get_package_share_directory('assembly_line')
        catalog = self.declare_parameter('catalog', share + '/config/arm.yaml').value
        run_id = self.declare_parameter('run_id', '').value or None
        jev = self.declare_parameter('jev_service', '/jev/decide').value
        llm = self.declare_parameter('llm_action', '/llm/reason').value

        self.status: ArmStatus | None = None
        self.peers: dict[str, ArmStatus] = {}
        self.parts: dict[int, object] = {}            # PartState by id
        self.goal: tuple[str, int, int] | None = None  # (kind, part, op) in flight
        self.goal_handle = None
        self.defer_after_cancel: tuple[int, int] | None = None
        self.paused_until = -math.inf
        self.inspect = ''                              # open inspect_tool request (why)
        self.tool_change_ordered = False
        self.checking = ''                             # subject of the open op_check
        self.coop_open = ''                            # subject of the coop wait under decision
        self.handed_off: dict[str, float] = {}         # subject -> when reported

        self.create_subscription(ArmStatus, 'status', self._status, 10)
        for a in scenario.arms(self.p):
            if a['name'] != self.name:
                self.create_subscription(ArmStatus, f'/{a["name"]}/status',
                                         lambda m: self.peers.__setitem__(m.arm, m), 10)
        self.create_subscription(LineState, '/line/state', self._line, 10)
        self.op = ActionClient(self, Operate, 'operate')
        self.engine = DecisionEngine(self, catalog, encode_state=self.encode, mask=self.mask,
                                     require=self.require, on_decision=self.on_decision,
                                     current=self.current, jev=jev, llm=llm, run_id=run_id,
                                     agent=f'/{self.name}', all_phases=True)
        self.hierarchy = Hierarchy(self, agent=f'/{self.name}', on_directive=self.on_directive,
                                   issuer_allowlist=[SUPERVISOR])

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def subject(self) -> str:
        s = self.status
        return f'part{s.part_id}:op{s.op_index}' if s and s.part_id >= 0 else ''

    # --- inputs ----------------------------------------------------------------------------------

    def _line(self, m: LineState) -> None:
        self.parts = {pt.id: pt for pt in m.parts}

    def _status(self, m: ArmStatus) -> None:
        prev, self.status = self.status, m
        if prev is not None:
            self.report_changes(prev, m)
        self.step()

    def report_changes(self, prev: ArmStatus, m: ArmStatus) -> None:
        if m.state == 'jammed' and prev.state != 'jammed':
            self.get_logger().warn(f'jammed (clear in ~{m.jam_clear_est_s:.0f} s)')
            self.report('fault', priority=0.8, kind='jam', est_clear_s=round(m.jam_clear_est_s),
                        jams=m.jams)
        elif prev.state == 'jammed' and m.state != 'jammed':
            self.report('recovered', priority=0.5, kind='jam')
        if m.dropout and not prev.dropout:
            self.get_logger().warn('sensor dropout')
            self.report('fault', priority=0.6, kind='sensor_dropout')
        elif prev.dropout and not m.dropout:
            self.report('recovered', priority=0.4, kind='sensor_dropout')

    def on_directive(self, msg, payload: dict) -> None:
        if msg.kind == 'pause':
            self.paused_until = self.now() + float(payload.get('seconds', self.p['pause_s']))
        elif msg.kind == 'resume':
            self.paused_until = -math.inf
        elif msg.kind == 'inspect_tool':
            self.inspect = str(payload.get('reason', 'QA flagged this arm'))
        elif msg.kind == 'tool_change':
            self.tool_change_ordered = True
        self.get_logger().info(f'directive {msg.kind} {payload}')

    # --- the routine path ------------------------------------------------------------------------

    def step(self) -> None:
        s, now = self.status, self.now()
        if s.state == 'coop_wait':
            subj = self.subject()
            if s.coop_wait_s >= self.p['coop_late_s'] and self.coop_open != subj:
                self.coop_open = subj                    # the coop set now ticks while it waits
                self.engine.trigger('partner_late', 'coop', subject=subj, fresh=True)
            return
        if self.goal or s.state != 'idle':
            return
        if self.tool_change_ordered:                     # obeyed even while paused
            if s.ops_since_change > 0:
                return self.send('tool_change', -1, -1)
            self.tool_change_ordered = False             # a fresh tool already
        if now < self.paused_until or s.part_id < 0:
            return
        subj = self.subject()
        if subj in self.handed_off:
            if now - self.handed_off[subj] < HAND_OFF_WAIT_S:
                return                                   # the supervisor is rerouting it
            del self.handed_off[subj]
        if self.checking == subj:
            return                                       # decision open
        if routine_ok(s, self.p, self.inspect):
            return self.send('op', s.part_id, s.op_index)
        self.checking = subj
        self.engine.trigger('pre_op_anomaly', 'op_check', subject=subj, fresh=True)

    def alternatives(self) -> list[str]:
        """Other arms (not jammed) that could take the pending op."""
        s = self.status
        if not s or s.part_id < 0:
            return []
        out = []
        for _, entry in scenario.capable(self.p, s.op_kind, exclude={self.name}):
            names = entry.split('+')
            if all(self.peers.get(n) is None or self.peers[n].state != 'jammed' for n in names):
                out.append(entry)
        return out

    # --- jev_ai callbacks ------------------------------------------------------------------------

    def encode(self, qset: str) -> dict[str, str]:
        s = self.status
        if qset == 'coop':
            if s is None:
                return {k: 'unknown' for k in ('step', 'partner', 'waiting', 'queue')}
            return encode_coop(s, self.p)
        if s is None or s.part_id < 0:
            return {k: 'none' for k in
                    ('operation', 'grip', 'alignment', 'tool', 'sensors', 'context')}
        return encode_op_check(s, self.parts.get(s.part_id), self.p, self.inspect,
                               self.alternatives())

    def current(self, qset: str) -> dict[str, str]:
        return {'action': 'proceed'} if qset == 'op_check' else {'coop_action': 'wait'}

    def mask(self, qset: str) -> dict[str, set[str]]:
        s = self.status
        if qset != 'op_check' or s is None or s.part_id < 0:
            return {}
        out = set()
        if math.isnan(s.grip) or s.regrips >= self.p['max_regrips']:
            out.add('retry')                             # nothing (more) to regrip
        if not self.alternatives():
            out.add('hand_off')
        if s.ops_since_change < self.p['min_ops_between_changes'] and not self.inspect:
            out.add('tool_change')
        return {'action': out}

    def require(self, qset: str) -> dict[str, str]:
        s = self.status
        if qset == 'coop':
            if s is None or s.state != 'coop_wait':
                return {'coop_action': 'wait'}           # nothing to decide
            if s.coop_wait_s >= self.p['coop_max_s'] or s.partner_state == 'jammed':
                return {'coop_action': 'abort_step'}
            return {}
        if s is None or s.part_id < 0 or not self.checking:
            return {'action': 'proceed'}                 # nothing to decide (acted on only with
        #                                                  a matching subject)
        if not math.isnan(s.grip) and s.grip < 0.5 and s.regrips >= self.p['max_regrips'] \
                and self.alternatives():
            return {'action': 'hand_off'}                # still slipping after every regrip
        return {}

    def on_decision(self, d: Decision) -> None:
        if not d.final:
            return                                       # an operation can't be taken back
        if d.question_set == 'coop':
            return self.after_coop(d)
        subj, self.checking = self.checking, ''
        s = self.status
        if not subj or subj != self.subject() or s.state != 'idle' or self.goal:
            return                                       # the part moved on, or a forced no-op
        a = d.selected.get('action')
        self.get_logger().info(f'op_check {subj}: {a} ({d.phase})' +
                               (f': {d.rationale[:80]}' if d.rationale else ''))
        self.inspect = ''
        if a == 'proceed':
            self.send('op', s.part_id, s.op_index)
        elif a == 'retry':
            self.send('regrip', s.part_id, s.op_index)
        elif a == 'tool_change':
            self.send('tool_change', -1, -1)
        elif a == 'hand_off':
            self.handed_off[subj] = self.now()
            self.report('hand_off', priority=0.6, part=s.part_id, op=s.op_index, kind=s.op_kind)

    def after_coop(self, d: Decision) -> None:
        s = self.status
        if d.selected.get('coop_action') != 'abort_step' or s is None or \
                s.state != 'coop_wait' or self.goal_handle is None:
            return
        self.get_logger().info(f'fasten on part {s.part_id}: abort_step ({d.phase})')
        self.defer_after_cancel = (s.part_id, s.op_index)
        self.goal_handle.cancel_goal_async()

    # --- acting ----------------------------------------------------------------------------------

    def send(self, kind: str, part: int, op: int) -> None:
        if not self.op.server_is_ready():
            self.get_logger().warn('operate server not ready', throttle_duration_sec=5.0)
            return
        self.goal = (kind, part, op)
        self.op.send_goal_async(Operate.Goal(kind=kind, part_id=part, op_index=op)) \
            .add_done_callback(self._accepted)

    def _accepted(self, fut) -> None:
        gh = fut.result()
        if not gh.accepted:
            self.goal = None
            return
        self.goal_handle = gh
        gh.get_result_async().add_done_callback(self._done)

    def _done(self, fut) -> None:
        res = fut.result().result
        kind = self.goal[0] if self.goal else ''
        self.goal, self.goal_handle = None, None
        if kind == 'op':
            self.coop_open = ''
        if kind == 'tool_change' and res.success:
            self.tool_change_ordered = False
            self.report('tool_change', priority=0.3)
        if res.message.startswith('rejected'):
            self.get_logger().warn(f'{kind}: {res.message}')
        if res.message == 'cancelled' and self.defer_after_cancel:
            part, op = self.defer_after_cancel
            self.defer_after_cancel = None
            self.send('defer', part, op)

    def report(self, what: str, priority: float, **payload) -> None:
        """A Report up; ``payload`` may carry its own ``kind`` (jam, the op kind...)."""
        s = self.status
        if s is not None:
            payload.setdefault('state', s.state)
            payload.setdefault('wear_est', None if math.isnan(s.wear_est) else
                               round(s.wear_est, 3))
        self.hierarchy.report(what, subject=self.name, priority=priority, payload=payload)


def main(args=None):
    run(ArmAgent, args, threads=1)      # callbacks never race over the goal state
