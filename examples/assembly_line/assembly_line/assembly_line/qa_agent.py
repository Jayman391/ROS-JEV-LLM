"""qa_agent: the QA station's jev_ai worker (DECISION_MODEL §5).

Every finished part gets one noisy reading per op. All readings ok -> pass by rule, no model.
Otherwise the ``inspect`` set decides pass / rework / quarantine / scrap (scrap is irreversible:
two keys). The part waits at QA until the decision is final, then ``Dispose``. Every part, rule
passes included, is reported to the supervisor as ``qa_result`` so its per-arm record has a
denominator.
"""
from __future__ import annotations

from collections import defaultdict, deque

from ament_index_python.packages import get_package_share_directory
from line_interfaces.msg import LineState, QaReading
from line_interfaces.srv import Dispose
from rclpy.node import Node

from assembly_line import scenario
from jev_ai import Decision, DecisionEngine, Hierarchy
from jev_ai.ros.engine import run

RECORD = 10                 # parts per arm in its QA record


def reading_word(r: float, ok_min: float) -> str:
    return 'ok' if r >= ok_min else 'marginal' if r >= 0.7 else 'suspect' if r >= 0.55 else \
        'fail' if r >= 0.4 else 'missing'


def encode_inspect(m: QaReading, p: dict, records: dict, line: str) -> dict:
    """``records``: arm -> recent flagged booleans; ``line``: the line-pressure sentence."""
    ok = p['ok_min']
    part = (f'part {m.part_id}, product {m.product}; reworked {m.reworks} of at most '
            f'{p["max_reworks"]} times')
    readings = '; '.join(f'{k} by {a}: {reading_word(r, ok)} ({r:.2f})'
                         for k, a, r in zip(m.op_kinds, m.op_arms, m.readings))
    flagged = sorted({a for a, r in zip(m.op_arms, m.readings) if r < ok})
    recs = []
    for a in flagged:
        rec = records.get(a, ())
        recs.append(f'{a}: {sum(rec)} of its last {len(rec)} parts flagged' if rec else
                    f'{a}: no record yet')
    return {'part': part, 'readings': readings, 'arm_records': '; '.join(recs) or 'none',
            'line': line}


def worst(m: QaReading) -> int:
    return min(range(len(m.readings)), key=lambda i: m.readings[i])


class QaAgent(Node):
    def __init__(self):
        super().__init__('qa_agent')
        self.p = scenario.from_node(self)
        share = get_package_share_directory('assembly_line')
        catalog = self.declare_parameter('catalog', share + '/config/qa.yaml').value
        run_id = self.declare_parameter('run_id', '').value or None
        jev = self.declare_parameter('jev_service', '/jev/decide').value
        llm = self.declare_parameter('llm_action', '/llm/reason').value
        self.total = sum(self.p['order_counts'])

        self.part: QaReading | None = None         # the part waiting at QA
        self.subject = ''                          # subject of the open decision
        self.line: LineState | None = None
        self.records: dict[str, deque] = defaultdict(lambda: deque(maxlen=RECORD))
        self.create_subscription(QaReading, '/qa/reading', self._reading, 10)
        self.create_subscription(LineState, '/line/state', lambda m: setattr(self, 'line', m), 10)
        self.dispose_cli = self.create_client(Dispose, '/line/dispose')
        self.engine = DecisionEngine(self, catalog, encode_state=self.encode, mask=self.mask,
                                     require=self.require, on_decision=self.on_decision,
                                     current=lambda q: {'disposition': 'pass'}, jev=jev, llm=llm,
                                     run_id=run_id, agent='/qa_agent', all_phases=True)
        self.hierarchy = Hierarchy(self, agent='/qa_agent')

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _reading(self, m: QaReading) -> None:
        self.part = m
        if all(r >= self.p['ok_min'] for r in m.readings):
            return self.dispose('pass', 'rule')
        self.subject = f'part{m.part_id}'
        self.engine.trigger('part_arrived', 'inspect', subject=self.subject, fresh=True)

    # --- jev_ai callbacks ------------------------------------------------------------------------

    def line_sentence(self) -> str:
        if self.line is None:
            return 'unknown'
        shipped = sum(self.line.shipped)
        expected = self.total * min(1.0, self.line.t / (0.85 * self.p['t_max_s']))
        pace = 'behind schedule' if shipped < expected - 2 else 'on schedule'
        lane = sum(pt.where == 'lane' for pt in self.line.parts)
        return (f'{pace}: {shipped} of {self.total} units shipped ({expected:.0f} expected by '
                f'now); {lane} part(s) in the rework lane')

    def encode(self, qset: str) -> dict[str, str]:
        if self.part is None:
            return {k: 'none' for k in ('part', 'readings', 'arm_records', 'line')}
        return encode_inspect(self.part, self.p, self.records, self.line_sentence())

    def mask(self, qset: str) -> dict[str, set[str]]:
        if self.part is not None and self.part.reworks >= self.p['max_reworks']:
            return {'disposition': {'rework'}}
        return {}

    def require(self, qset: str) -> dict[str, str]:
        m = self.part
        if m is None or not self.subject:
            return {'disposition': 'pass'}           # nothing at QA: a forced no-op
        if m.reworks >= self.p['max_reworks'] and min(m.readings) < 0.4:
            return {'disposition': 'quarantine'}     # rework masked, a feature missing
        return {}

    def on_decision(self, d: Decision) -> None:
        if not d.final:
            return                                   # the part waits: a disposal is final
        m, subj = self.part, self.subject
        if m is None or subj != f'part{m.part_id}':
            return
        self.subject = ''
        self.dispose(d.selected['disposition'], d.phase, d.rationale)

    # --- acting ----------------------------------------------------------------------------------

    def dispose(self, what: str, why: str, rationale: str = '') -> None:
        m = self.part
        op = worst(m)
        req = Dispose.Request(part_id=m.part_id, disposition=what, rework_op=op)
        self.get_logger().info(f'part {m.part_id}: {what} ({why})' +
                               (f': {rationale[:80]}' if rationale else ''))
        self.dispose_cli.call_async(req).add_done_callback(
            lambda f: self._disposed(f, m, what))

    def _disposed(self, fut, m: QaReading, what: str) -> None:
        res = fut.result()
        if not res.success:
            self.get_logger().error(f'dispose part {m.part_id} {what}: {res.message}')
            return
        if self.part is m:
            self.part = None
        ok = self.p['ok_min']
        for a, r in zip(m.op_arms, m.readings):
            for name in a.split('+'):
                self.records[name].append(r < ok)
        i = worst(m)
        flagged = [{'kind': k, 'arm': a, 'reading': round(r, 3)}
                   for k, a, r in zip(m.op_kinds, m.op_arms, m.readings) if r < ok]
        self.hierarchy.report('qa_result', subject=f'part{m.part_id}',
                              priority=0.7 if flagged else 0.1,
                              payload={'part': m.part_id, 'product': m.product,
                                       'disposition': what, 'reworks': m.reworks,
                                       'ops': [{'kind': k, 'arm': a, 'reading': round(r, 3)}
                                               for k, a, r in zip(m.op_kinds, m.op_arms,
                                                                  m.readings)],
                                       'flagged': flagged,
                                       'worst': {'kind': m.op_kinds[i], 'arm': m.op_arms[i],
                                                 'reading': round(m.readings[i], 3)}})


def main(args=None):
    run(QaAgent, args, threads=1)
