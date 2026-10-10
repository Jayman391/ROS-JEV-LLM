"""Optional hierarchy module (FRAMEWORK §3 Report/Directive, §10, REVIEW A32): reports go up,
directives come down.

    h = Hierarchy(node, engine=self.engine, qset='next_step', on_directive=self.obey)
    h.report('status', subject='person at 12 m', priority=0.7, beliefs=d.fused, payload={...})
    h.direct('converge', payload={'target': ...}, ttl_s=15)          # supervisor side

Receivers drop directives that are not for them (``target`` is neither the agent nor ``*``), have
expired (node clock), repeat an ``(issuer, id)`` already seen, or come from an issuer outside
``issuer_allowlist`` (empty = any; set it in deployments). With an ``engine``, every accepted
directive is a decision event whose noise epoch is the directive (§6.3, D3): workers in one group
answering the same directive share their Gumbel noise (ρ = ``coupling_directive``, default 1.0),
so they coordinate without talking to each other. ``on_directive`` runs first, so the app can
fold the order into its state before the engine encodes it.
"""
from __future__ import annotations

import json
import time
from collections import deque
from typing import Callable

from builtin_interfaces.msg import Time
from jev_ai_msgs.msg import Directive, Distribution, Report
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from jev_ai.core.fusion import confidence

REPORT_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
DIRECTIVE_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)


class Hierarchy:
    def __init__(self, node, *, agent: str | None = None,
                 on_directive: Callable[[Directive, dict], None] | None = None,
                 on_report: Callable[[Report, dict], None] | None = None,
                 engine=None, qset: str | None = None, issuer_allowlist=(),
                 reports: str = 'reports', directives: str = 'directives'):
        self.node = node
        self.agent = agent or (engine.agent if engine else node.get_fully_qualified_name())
        self.engine, self.qset = engine, qset
        self.on_directive_cb, self.on_report_cb = on_directive, on_report
        self.allow = set(issuer_allowlist)
        self.seen: deque[tuple[str, int]] = deque(maxlen=1000)
        self.next_id = time.time_ns() // 1000           # unique across restarts
        self.dropped = {'target': 0, 'expired': 0, 'duplicate': 0, 'issuer': 0}
        self.report_pub = node.create_publisher(Report, reports, REPORT_QOS)
        self.directive_pub = node.create_publisher(Directive, directives, DIRECTIVE_QOS)
        if on_directive or engine:
            node.create_subscription(Directive, directives, self._directive, DIRECTIVE_QOS)
        if on_report:
            node.create_subscription(Report, reports, self._report, REPORT_QOS)

    # --- up ---------------------------------------------------------------------------------------

    def report(self, kind: str, subject: str = '', priority: float = 0.0,
               beliefs: dict[str, tuple[list[str], list[float]]] | None = None,
               payload: dict | None = None) -> None:
        """``beliefs``: question -> (classes, probabilities), e.g. ``Decision.fused``."""
        msg = Report(agent=self.agent, kind=kind, subject=subject, priority=float(priority),
                     payload=json.dumps(payload or {}))
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.beliefs = [Distribution(question=q, classes=list(c), probabilities=list(p),
                                    confidence=confidence(p) if p else float('nan'),
                                    source_confidence=float('nan'))
                       for q, (c, p) in (beliefs or {}).items()]
        self.report_pub.publish(msg)

    def _report(self, msg: Report) -> None:
        try:
            payload = json.loads(msg.payload or '{}')
        except ValueError:
            payload = {}
        self.on_report_cb(msg, payload)

    # --- down -------------------------------------------------------------------------------------

    def direct(self, kind: str, target: str = '*', payload: dict | None = None,
               ttl_s: float = 10.0) -> int:
        """Publish a directive; returns its id. ``ttl_s <= 0``: never expires."""
        self.next_id += 1
        now = self.node.get_clock().now()
        msg = Directive(issuer=self.agent, directive_id=self.next_id, target=target, kind=kind,
                        payload=json.dumps(payload or {}))
        msg.header.stamp = now.to_msg()
        if ttl_s > 0:
            ns = now.nanoseconds + int(ttl_s * 1e9)
            msg.expires = Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)
        self.directive_pub.publish(msg)
        return self.next_id

    def accept(self, msg: Directive) -> str:
        """'' if the directive is for us and valid, else why it is dropped."""
        if msg.target not in ('*', self.agent):
            return 'target'
        if self.allow and msg.issuer not in self.allow:
            return 'issuer'
        exp = msg.expires.sec * 1_000_000_000 + msg.expires.nanosec
        if exp and self.node.get_clock().now().nanoseconds > exp:
            return 'expired'
        if (msg.issuer, msg.directive_id) in self.seen:
            return 'duplicate'
        return ''

    def _directive(self, msg: Directive) -> None:
        why = self.accept(msg)
        if why:
            self.dropped[why] += 1
            if why != 'target':
                self.node.get_logger().warn(
                    f'dropped directive {msg.issuer}#{msg.directive_id}: {why}',
                    throttle_duration_sec=5.0)
            return
        self.seen.append((msg.issuer, msg.directive_id))
        try:
            payload = json.loads(msg.payload or '{}')
        except ValueError:
            payload = {}
        if self.on_directive_cb:
            self.on_directive_cb(msg, payload)
        if self.engine:
            subject = str(payload.get('subject', '')) if isinstance(payload, dict) else ''
            self.engine.trigger(f'directive:{msg.kind}', self.qset, fresh=True, subject=subject,
                                directive=(msg.issuer, msg.directive_id))
