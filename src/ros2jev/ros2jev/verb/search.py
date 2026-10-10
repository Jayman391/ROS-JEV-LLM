"""ros2 jev search: episode-level successive halving with common random numbers (§12).

    ros2 jev search --runner jev_ai_examples.toy_episode:ToyEpisode \\
        --space T_dwell=1,5,15 tick_hz=0.5,1,2 k_max=3,12 [--runner-arg duration_s=300]

The runner is ``module:Class``: ``Class(**runner_args).run(params, seed) -> float`` (higher is
better). Use it for what OPE can't evaluate from logs: dwell, deadlines, cadence, k_max.
"""
from importlib import import_module
from pathlib import Path

import yaml
from ros2cli.verb import VerbExtension

from ros2jev.api import parse_kv


class SearchVerb(VerbExtension):
    """Find the best episode parameters with successive halving."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('--runner', required=True, help='module:Class (an EpisodeRunner)')
        parser.add_argument('--runner-arg', nargs='+', default=[], metavar='KEY=VALUE')
        parser.add_argument('--space', nargs='+', required=True, metavar='KEY=V1,V2,...')
        parser.add_argument('--min-seeds', type=int, default=2)
        parser.add_argument('--max-seeds', type=int, default=16)
        parser.add_argument('--eta', type=int, default=2, help='keep 1/eta per round')
        parser.add_argument('-o', '--output', help='write the table (Markdown)')

    def main(self, *, args):
        from jev_ai.core.search import grid, successive_halving, table
        module, _, cls = args.runner.partition(':')
        runner = getattr(import_module(module), cls)(**parse_kv(args.runner_arg))
        space = {}
        for item in args.space:
            k, _, vs = item.partition('=')
            space[k] = [yaml.safe_load(v) for v in vs.split(',')]
        configs = grid(space)
        print(f'{len(configs)} configurations, {args.min_seeds}..{args.max_seeds} seeds each')
        rows = successive_halving(configs, runner.run, min_seeds=args.min_seeds,
                                  max_seeds=args.max_seeds, eta=args.eta)
        md = table(rows)
        print(md)
        best = rows[0]
        print(f'\nbest: {best["params"]}  score {best["mean"]:.4f} over {best["n"]} seeds')
        if args.output:
            Path(args.output).write_text(md + '\n')
        return 0
