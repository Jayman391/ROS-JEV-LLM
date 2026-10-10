"""Episode-level parameter search (FRAMEWORK §12; phase A10).

For the parameters OPE can't evaluate from logs because they change *when* decisions happen
(T_dwell, deadlines, tick cadence, k_max): run whole episodes through an app-supplied
``EpisodeRunner`` and keep the best configurations by **successive halving**. Every
configuration in a round runs on the same seeds (**common random numbers**), so differences are
paired: ``diff`` is each configuration minus the best, with the paired standard error.
"""
from __future__ import annotations

import itertools
import math
import statistics
from typing import Callable, Protocol


class EpisodeRunner(Protocol):
    def run(self, params: dict, seed: int) -> float:
        """One episode with these parameters; higher is better. Same seed = same world."""


def grid(space: dict[str, list]) -> list[dict]:
    keys = list(space)
    return [dict(zip(keys, v)) for v in itertools.product(*(space[k] for k in keys))]


def successive_halving(configs: list[dict], run: Callable[[dict, int], float], *,
                       min_seeds: int = 2, max_seeds: int = 16, eta: int = 2,
                       on_result: Callable[[dict, int, float], None] | None = None) -> list[dict]:
    """Rows sorted best first: params, mean, se, n (seeds run), round (last round survived),
    diff / diff_se (paired, vs the best, on their common seeds)."""
    cache: dict[tuple[int, int], float] = {}
    alive, n, rnd = list(range(len(configs))), min(min_seeds, max_seeds), 0
    reached = {i: 0 for i in alive}

    def mean(i, k):
        return statistics.fmean(cache[i, s] for s in range(k))

    while True:
        for i in alive:
            reached[i] = rnd
            for s in range(n):
                if (i, s) not in cache:
                    cache[i, s] = float(run(configs[i], s))
                    if on_result:
                        on_result(configs[i], s, cache[i, s])
        if len(alive) == 1 or n >= max_seeds:
            break
        alive.sort(key=lambda i: -mean(i, n))
        alive = alive[:max(1, math.ceil(len(alive) / eta))]
        n, rnd = min(max_seeds, n * eta), rnd + 1

    def seeds(i):
        return [s for s in range(max_seeds) if (i, s) in cache]

    order = sorted(range(len(configs)), key=lambda i: (-reached[i], -mean(i, len(seeds(i)))))
    best = order[0]
    rows = []
    for i in order:
        v = [cache[i, s] for s in seeds(i)]
        d = [cache[i, s] - cache[best, s] for s in seeds(i)]
        rows.append({'params': configs[i], 'mean': statistics.fmean(v),
                     'se': statistics.stdev(v) / math.sqrt(len(v)) if len(v) > 1 else None,
                     'n': len(v), 'round': reached[i], 'diff': statistics.fmean(d),
                     'diff_se': statistics.stdev(d) / math.sqrt(len(d)) if len(d) > 1 else None})
    return rows


def table(rows: list[dict]) -> str:
    """Markdown table of successive_halving rows."""
    keys = list(rows[0]['params']) if rows else []

    def f(x, p=4):
        return '' if x is None else f'{x:.{p}f}' if isinstance(x, float) else str(x)
    head = keys + ['mean', 'se', 'seeds', 'round', 'vs best', 'paired se']
    lines = ['| ' + ' | '.join(head) + ' |', '|' + '---|' * len(head)]
    for r in rows:
        lines.append('| ' + ' | '.join([f'{r["params"][k]:g}' if isinstance(
            r['params'][k], float) else str(r['params'][k]) for k in keys] + [
            f(r['mean']), f(r['se']), str(r['n']), str(r['round']), f(r['diff']),
            f(r['diff_se'])]) + ' |')
    return '\n'.join(lines)
