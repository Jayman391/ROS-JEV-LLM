"""ros2 jev calibrate: fit β_min, β_max, γ, B and per-class biases b_q from labelled records (§12).

    ros2 jev calibrate decisions.jsonl --labels decisions-labels.jsonl -o patch.yaml
    ros2 jev calibrate decisions.jsonl --labels ... --catalog my.yaml -o my.calibrated.yaml
"""
import json
from pathlib import Path

import yaml
from ros2cli.verb import VerbExtension

from ros2jev.api import print_table


class CalibrateVerb(VerbExtension):
    """Fit the fusion policy to labels; print scores and write a catalog patch."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('records', help='decisions JSONL (decision_logger)')
        parser.add_argument('--labels', required=True, help='labels JSONL')
        parser.add_argument('--catalog', help='write the patched catalog instead of the patch')
        parser.add_argument('-o', '--output', default='calibration.yaml')
        parser.add_argument('--json', help='also write the full fit as JSON')
        parser.add_argument('--min-labels', type=int, default=20)

    def main(self, *, args):
        from jev_ai.core.calibrate import calibrate, merge
        from jev_ai.viz.report import read_jsonl
        out = calibrate(read_jsonl(args.records), read_jsonl(args.labels),
                        min_labels=args.min_labels)
        rows = []
        for q, f in out['questions'].items():
            if 'skipped' in f:
                print(f'{q}: skipped ({f["skipped"]}, n={f["n"]})')
                continue
            s = f['scores']
            rows.append([q, f['n'], f['n_llm']] + [s[k][m] for k in
                        ('jev', 'llm', 'fused', 'calibrated') for m in ('ece', 'nll')])
        if not rows and not out['questions']:
            print('no labelled decisions (labels join on run_id, subject/agent, decision_id)')
            return 1
        print_table(['question', 'n', 'llm', 'jev ece', 'nll', 'llm ece', 'nll', 'fused ece',
                     'nll', 'calib ece', 'nll'], rows)
        print()
        for q, f in out['questions'].items():
            if 'skipped' not in f:
                fitted = ', '.join(f'{k}={f[k]}' for k in ('beta_min', 'beta_max', 'gamma', 'B')
                                   if k in f) or 'no LLM answers: β kept'
                clamp = f'  (clamped to §14 ranges: {", ".join(f["clamped"])}; raw {f["raw"]})' \
                    if f['clamped'] else ''
                print(f'{q}: {fitted}{clamp}')
                print(f'  bias b_q (nats): {f["bias"]}')
        doc = out['patch']
        if args.catalog:
            doc = merge(yaml.safe_load(Path(args.catalog).read_text()), out['patch'])
        Path(args.output).write_text(
            '# ros2 jev calibrate: fitted on ' + args.records + '\n' +
            yaml.safe_dump(doc, sort_keys=False))
        print(f'\nwrote {args.output}')
        if args.json:
            Path(args.json).write_text(json.dumps(out, indent=1))
        return 0
