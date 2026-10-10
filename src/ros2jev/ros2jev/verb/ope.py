"""ros2 jev ope: evaluate new policy parameters on logged decisions, without re-running (§12).

    ros2 jev ope decisions.jsonl --catalog toy_patrol.yaml --labels labels.jsonl --set tau=0.2
    ros2 jev ope decisions.jsonl --catalog old.yaml --new-catalog calibrated.yaml --labels ...
    ros2 jev ope decisions.jsonl --catalog old.yaml --rewards rewards.jsonl --set kappa=1.5

``--set key=value`` applies to every question (``question.key=value`` to one). Rewards JSONL rows:
{"run_id", "agent", "decision_id", "question", "reward"} for the logged action.
"""
import json
from pathlib import Path

import yaml
from ros2cli.verb import VerbExtension

from ros2jev.api import parse_kv, print_table


class OpeVerb(VerbExtension):
    """Off-policy estimates (direct, SNIPS, DR, ESS) of new τ, κ, ρ, β, γ, B, b_q."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('records', help='decisions JSONL (decision_logger)')
        parser.add_argument('--catalog', required=True, help='the catalog the run used')
        parser.add_argument('--new-catalog', help='candidate catalog (default: --catalog + --set)')
        parser.add_argument('--set', nargs='+', default=[], metavar='KEY=VALUE')
        parser.add_argument('--labels', help='labels JSONL: reward 1 for the true class')
        parser.add_argument('--rewards', help='rewards JSONL for the logged actions')
        parser.add_argument('--boot', type=int, default=200, help='bootstrap resamples')
        parser.add_argument('--json', help='also write the estimates as JSON')

    def main(self, *, args):
        from jev_ai.core.catalog import load_catalog
        from jev_ai.core.ope import ope, overrides
        from jev_ai.viz.report import read_jsonl
        if not (args.labels or args.rewards):
            raise SystemExit('need --labels or --rewards')
        raw = yaml.safe_load(Path(args.catalog).read_text())
        new_raw = yaml.safe_load(Path(args.new_catalog).read_text()) if args.new_catalog else raw
        new_raw = overrides(new_raw, parse_kv(args.set))
        out = ope(read_jsonl(args.records), load_catalog(raw), load_catalog(new_raw),
                  labels=read_jsonl(args.labels) if args.labels else None,
                  rewards=read_jsonl(args.rewards) if args.rewards else None, boot=args.boot)
        c = out['counts']
        print(f'decisions re-derived exactly: {c["reproduced"]}; held (not sampled: dwell, '
              f'commit, forced, fallback...): {c["held"]}')
        for k in ('logits_mismatch', 'noise_kept', 'catalog_mismatch', 'unknown_question_set'):
            if c[k]:
                print(f'  {k}: {c[k]}' + ('  <- --catalog differs from the run\'s'
                                          if k == 'logits_mismatch' else ''))
        rows = []
        for q, e in out['questions'].items():
            ci = e.get('ci95', {})
            rows.append([q, e['n'], e.get('changed'), e.get('logged'), e.get('direct'),
                         ci.get('direct'), e.get('snips'), ci.get('snips'), e.get('dr'),
                         ci.get('dr'), e.get('ess')])
        print()
        print_table(['question', 'n', 'changed', 'logged', 'direct', '95% ci', 'snips',
                     '95% ci', 'dr', '95% ci', 'ess'], rows)
        print('\ndirect: exact on the logged states (labels give every class\'s reward); '
              'snips/dr use the logged reward only.')
        if args.json:
            Path(args.json).write_text(json.dumps(out, indent=1))
        return 0
