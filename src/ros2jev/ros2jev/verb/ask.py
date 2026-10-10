"""ros2 jev ask: one decision from the command line, through the real DecisionEngine (§0).

    ros2 jev ask --catalog my_catalog.yaml --set next_step --state task=patrolling battery="low (18%)"
"""
import time
from dataclasses import replace

import rclpy
from jev_ai_msgs.msg import DecisionRecord
from rclpy.executors import MultiThreadedExecutor
from ros2cli.verb import VerbExtension

from jev_ai import DecisionEngine
from jev_ai.core.catalog import load_catalog


class AskVerb(VerbExtension):
    """Decide once for a given state and print Jev, LLM and fused answers."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('--catalog', required=True, help='catalog YAML')
        parser.add_argument('--set', dest='qset', help='question set (default: the first)')
        parser.add_argument('--state', nargs='+', default=[], metavar='KEY=VALUE')
        parser.add_argument('--no-llm', action='store_true', help='Jev only (no augmentation)')
        parser.add_argument('--timeout', type=float, default=15.0)

    def main(self, *, args):
        cat = load_catalog(args.catalog, {'skip_llm_above_c': 0.0} if args.no_llm else None)
        name = args.qset or next(iter(cat.question_sets))
        cat = replace(cat, question_sets={name: cat.question_sets[name]})
        state = dict(kv.split('=', 1) for kv in args.state)
        rclpy.init()
        node = rclpy.create_node('ros2jev_ask')
        records, final = [], []
        engine = DecisionEngine(node, cat, encode_state=lambda _: state, agent='ros2jev_ask',
                                on_decision=lambda d: final.append(d) if d.final else None)
        node.create_subscription(DecisionRecord, 'decisions',
                                 lambda r: records.append(r) if r.agent == 'ros2jev_ask' else None,
                                 100)
        ex = MultiThreadedExecutor()
        ex.add_node(node)
        end = time.monotonic() + args.timeout
        while time.monotonic() < end and not (records and records[-1].phase in
                                              ('COMMITTED', 'KEPT', 'FALLBACK', 'FORCED')):
            ex.spin_once(timeout_sec=0.1)
        for r in records:
            _print(r)
        node.destroy_node()
        rclpy.shutdown()
        return 0 if records else 1


def _print(r):
    print(f'{r.phase}  decision {r.decision_id}  jev={r.jev_model or "-"} '
          f'({r.jev_latency_ms:.0f} ms)  llm={r.llm_model or "-"} ({r.llm_latency_ms:.0f} ms, '
          f'${r.cost_usd:.5f})')
    for q in r.questions:
        print(f'  {q.question}: selected {q.selected or "-"}  (c={q.jev_confidence:.2f}, '
              f'beta={q.beta:.2f}, propensity={q.propensity:.2f})')
        for i, c in enumerate(q.classes):
            jev = f'{q.p_jev[i]:.2f}' if len(q.p_jev) else '  - '
            llm = f'{q.q_llm[i]:.2f}' if len(q.q_llm) else '  - '
            mark = ' masked' if len(q.masked) and q.masked[i] else ''
            print(f'    {c:<16} jev {jev}  llm {llm}{mark}')
    if r.llm_rationale:
        print(f'  rationale: {r.llm_rationale}')
