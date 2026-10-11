"""line_scorer: the only node that reads the truth (SYSTEM_MODEL §8).

J = w_good·good_shipped + w_on_time·good_on_time − w_escaped·escaped_defects
    − w_scrap_good·scrapped_good − w_scrap_bad·scrapped_defective − w_quarantine·quarantined
    − w_usd·LLM $

Publishes it on /diagnostics (summary bar), a ``disposition`` Label per part for ``ros2 jev
calibrate``, and at the end of the shift (every order shipped, or t_max_s) the episode summary
JSON that ``assembly_line.episode`` reads.
"""
from __future__ import annotations

import json
from collections import Counter
from pathlib import Path

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from jev_ai_msgs.msg import DecisionRecord, Label
from line_interfaces.msg import PartOutcome
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from assembly_line import scenario
from jev_ai.ros.engine import run


def true_disposition(o: PartOutcome, max_reworks: int) -> str:
    """What QA should have decided for this part, knowing the truth."""
    if not o.defective:
        return 'pass'
    return 'rework' if o.reworks < max_reworks else 'scrap'


def score(p: dict, outcomes: list[PartOutcome], cost_usd: float) -> dict:
    due = scenario.orders(p)
    c = Counter()
    k = Counter()
    for o in sorted(outcomes, key=lambda o: o.done_s):
        if o.outcome == 'shipped':
            if o.defective:
                c['escaped'] += 1
            else:
                c['good'] += 1
                i = k[o.product]
                k[o.product] += 1
                if i < len(due[o.product]) and o.done_s <= due[o.product][i]:
                    c['on_time'] += 1
        elif o.outcome == 'scrapped':
            c['scrap_bad' if o.defective else 'scrap_good'] += 1
        else:
            c['quarantined'] += 1
    j = (p['w_good'] * c['good'] + p['w_on_time'] * c['on_time'] - p['w_escaped'] * c['escaped']
         - p['w_scrap_good'] * c['scrap_good'] - p['w_scrap_bad'] * c['scrap_bad']
         - p['w_quarantine'] * c['quarantined'] - p['w_usd'] * cost_usd)
    return {'J': round(j, 2), 'good_shipped': c['good'], 'good_on_time': c['on_time'],
            'escaped_defects': c['escaped'], 'scrapped_good': c['scrap_good'],
            'scrapped_defective': c['scrap_bad'], 'quarantined': c['quarantined'],
            'reworks': sum(o.reworks for o in outcomes), 'llm_usd': round(cost_usd, 4)}


class Scorer(Node):
    def __init__(self):
        super().__init__('line_scorer')
        self.p = scenario.from_node(self)
        self.run_id = self.declare_parameter('run_id', '').value
        self.summary_path = self.declare_parameter('summary_path', '').value
        self.total = sum(self.p['order_counts'])
        self.outcomes: list[PartOutcome] = []
        self.cost_usd = 0.0
        self.decisions = 0
        self.written = False
        reliable = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PartOutcome, '/line/outcomes', self._outcome,
                                 QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(DecisionRecord, '/decisions', self._record, reliable)
        self.labels = self.create_publisher(Label, '/labels', reliable)
        self.diag = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.create_timer(1.0, self.tick)

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _record(self, m: DecisionRecord) -> None:
        self.cost_usd += m.cost_usd
        self.decisions += 1

    def _outcome(self, o: PartOutcome) -> None:
        self.outcomes.append(o)
        lab = Label(run_id=self.run_id, subject=f'part{o.part_id}', question='disposition',
                    true_class=true_disposition(o, int(self.p['max_reworks'])), decision_id=0,
                    source='line_scorer')
        lab.header.stamp = self.get_clock().now().to_msg()
        self.labels.publish(lab)
        if o.outcome == 'shipped' and o.defective:
            self.get_logger().warn(f'escaped defect: part {o.part_id} ({o.product}), quality '
                                   + ', '.join(f'{k} {q:.2f}' for k, q in
                                               zip(o.op_kinds, o.quality)))

    def tick(self) -> None:
        s = score(self.p, self.outcomes, self.cost_usd)
        t = self.now()
        shipped = s['good_shipped'] + s['escaped_defects']
        st = DiagnosticStatus(name='line_scorer', level=DiagnosticStatus.OK,
                              message=f'J {s["J"]:.1f}: {shipped} of {self.total} shipped')
        st.values = [KeyValue(key=k, value=str(v)) for k, v in s.items()] + \
            [KeyValue(key='decisions', value=str(self.decisions))]
        self.diag.publish(DiagnosticArray(status=[st]))
        if not self.written and (shipped >= self.total or t >= self.p['t_max_s']):
            self.written = True
            s.update(run_id=self.run_id, seed=self.p['seed'], t_end_s=round(t, 1),
                     decisions=self.decisions, shipped=shipped, orders=self.total)
            self.get_logger().info(f'shift over: {json.dumps(s)}')
            if self.summary_path:
                Path(self.summary_path).parent.mkdir(parents=True, exist_ok=True)
                Path(self.summary_path).write_text(json.dumps(s, indent=2))


def main(args=None):
    run(Scorer, args)
