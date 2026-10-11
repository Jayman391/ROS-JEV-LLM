"""The assembly line scenario (SYSTEM_MODEL §2): parameters, arms, recipes, orders and the routing
rule (DECISION_MODEL §3.4).

Everything derives from ``config/scenario.yaml``, the same file line_sim (C++) reads as ROS
parameters, plus an episode ``seed``.
"""
from __future__ import annotations

from pathlib import Path

import yaml


def load(path: str | Path, **overrides) -> dict:
    """The ``/**`` ros__parameters of a scenario file, with overrides (e.g. seed)."""
    p = dict(yaml.safe_load(Path(path).read_text())['/**']['ros__parameters'])
    p.update({k: v for k, v in overrides.items() if v is not None})
    return p


def from_node(node) -> dict:
    """The scenario for a ROS node: param ``scenario_file`` (default: this package's) with the
    launch's ``seed`` / ``t_max_s`` overrides (-1 / 0 = the file's)."""
    from ament_index_python.packages import get_package_share_directory
    default = get_package_share_directory('assembly_line') + '/config/scenario.yaml'
    path = node.declare_parameter('scenario_file', default).value
    over = {}
    for k, unset in (('seed', -1), ('t_max_s', 0.0)):
        v = node.declare_parameter(k, unset).value
        if v != unset:
            over[k] = v
    return load(path, **over)


def arms(p: dict) -> list[dict]:
    return [{'name': n, 'station': int(s), 'tools': set(t.split(','))}
            for n, s, t in zip(p['arms'], p['arm_station'], p['arm_tools'])]


def arm(p: dict, name: str) -> dict:
    return next(a for a in arms(p) if a['name'] == name)


def recipes(p: dict) -> dict[str, list[str]]:
    return {pr: r.split(',') for pr, r in zip(p['products'], p['recipes'])}


def orders(p: dict) -> dict[str, list[float]]:
    """Due times (sim s) per product: each product's units spread evenly over 85% of the shift."""
    t = 0.85 * float(p['t_max_s'])
    return {pr: [round((j + 1) / n * t) for j in range(int(n))]
            for pr, n in zip(p['products'], p['order_counts'])}


def capable(p: dict, kind: str, exclude=()) -> list[tuple[int, str]]:
    """(station, route entry) for every arm, or fasten pair, that can do ``kind``."""
    up = [a for a in arms(p) if a['name'] not in exclude]
    if kind == 'fasten':                       # a screw arm + a different hold arm, one station
        return [(s['station'], f'{s["name"]}+{h["name"]}') for s in up if 'screw' in s['tools']
                for h in up if h is not s and 'hold' in h['tools'] and h['station'] == s['station']]
    return [(a['station'], a['name']) for a in up if kind in a['tools']]


def route(p: dict, kinds: list[str], at: int = 0, exclude=(),
          load: dict[str, float] | None = None) -> tuple[list[str], int]:
    """The routing rule: one entry per op ('' = unroutable) and how many laps it needs.

    Per op, among capable arms not excluded: prefer a station at or after the pallet's position
    (no lap), then the least queued work (``load``: parts waiting per arm, plus what this route
    already gave it), then the shortest travel. ``at``: the stop the pallet is at or heading to
    (0 = feeder or rework lane).
    """
    load = dict(load or {})
    out, laps, here = [], 0, at
    for kind in kinds:
        cands = capable(p, kind, exclude)
        if not cands:
            out.append('')
            continue

        def cost(c):
            s, entry = c
            lap = s < here
            return lap, load.get(entry.split('+')[0], 0.0) + 0.25 * (s if lap else s - here), entry
        s, entry = min(cands, key=cost)
        laps += s < here
        here = s
        out.append(entry)
        primary = entry.split('+')[0]
        load[primary] = load.get(primary, 0.0) + 1.0
    return out, laps


def describe_route(p: dict, product: str, exclude=()) -> str:
    """'one pass' | 'needs 1 lap' | 'blocked (fasten)' for a product, given arms out of use."""
    entries, laps = route(p, recipes(p)[product], exclude=exclude)
    blocked = [k for k, e in zip(recipes(p)[product], entries) if not e]
    if blocked:
        return f'blocked (no arm for {", ".join(sorted(set(blocked)))})'
    return 'one pass' if laps == 0 else f'needs {laps} lap{"s" * (laps > 1)}'


def mmss(t: float) -> str:
    t = max(0.0, t)
    return f'{int(t // 60)}:{int(t % 60):02d}'
