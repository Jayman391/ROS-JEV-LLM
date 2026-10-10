"""Small helpers shared by the ros2 jev verbs."""
from __future__ import annotations

import yaml


def parse_kv(items) -> dict:
    """['a=1', 'b=x'] -> {'a': 1, 'b': 'x'} (values parsed as YAML scalars)."""
    out = {}
    for kv in items or []:
        k, sep, v = kv.partition('=')
        if not sep:
            raise SystemExit(f'expected key=value, got {kv!r}')
        out[k] = yaml.safe_load(v)
    return out


def fmt(x, p: int = 3) -> str:
    if x is None:
        return '-'
    if isinstance(x, float):
        return f'{x:.{p}f}'
    if isinstance(x, (list, tuple)):
        return '[' + ', '.join(fmt(v, p) for v in x) + ']'
    return str(x)


def print_table(head: list[str], rows: list[list]) -> None:
    cells = [head] + [[fmt(c) for c in r] for r in rows]
    w = [max(len(r[i]) for r in cells) for i in range(len(head))]
    for i, r in enumerate(cells):
        print('  '.join(c.ljust(w[j]) for j, c in enumerate(r)).rstrip())
        if i == 0:
            print('  '.join('-' * x for x in w))
