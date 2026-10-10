"""Static decision report: dashboard.html with metrics and recent records embedded (§12; A5.5).

    python3 -m jev_ai.viz.report decisions.jsonl [--labels labels.jsonl] -o report.html
    python3 -m jev_ai.viz.report --demo -o report.html        # synthetic fleet (viz.sample)

Pure Python, no ROS. ``payload`` and ``render`` are shared with the live dashboard node.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from jev_ai.core import metrics

TEMPLATE = Path(__file__).with_name('dashboard.html')
DATA_TAG = '<script type="application/json" id="jev-data">{}</script>'
RECENT = 400


def read_jsonl(path: str | Path) -> list[dict]:
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def recent(records: list[dict], n: int = RECENT) -> list[dict]:
    """The last ``n`` records (by wall time) plus the latest decision of every
    (agent, question set), so each dashboard card has something to show."""
    rs = sorted(records, key=metrics.wall_time)
    tail = rs[-n:] if n else []
    keep = {metrics.decision_key(r) for r in tail}
    latest = {}
    for r in rs:
        latest[(r.get('agent'), r.get('question_set'))] = metrics.decision_key(r)
    extra = set(latest.values()) - keep
    return [r for r in rs[:-n or None] if metrics.decision_key(r) in extra] + tail


def payload(records: list[dict], metrics_dict: dict, diagnostics: dict | None = None,
            mode: str = 'static', n: int = RECENT) -> dict:
    return metrics.json_safe({'mode': mode, 'generated_at': time.time(),
                              'records': recent(records, n), 'diagnostics': diagnostics or {},
                              'metrics': metrics_dict})


def render(data: dict | None = None) -> str:
    """The page; with ``data`` embedded it works offline as a static report."""
    html = TEMPLATE.read_text()
    if data is None:
        return html
    # '</' would end the <script> element early (and lets record text inject markup)
    blob = json.dumps(data, separators=(',', ':')).replace('</', '<\\/').replace('<!--', '<\\!--')
    return html.replace(DATA_TAG.format('null'), DATA_TAG.format(blob), 1)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('records', nargs='?', help='DecisionRecord JSONL (decision_logger output)')
    ap.add_argument('--labels', help='Label JSONL (ground truth) for calibration')
    ap.add_argument('--demo', action='store_true', help='use synthetic records (viz.sample)')
    ap.add_argument('-o', '--output', default='report.html')
    a = ap.parse_args(argv)
    if a.demo:
        from jev_ai.viz import sample
        records, labels = sample.generate()
        diagnostics = sample.diagnostics()
    elif a.records:
        records = read_jsonl(a.records)
        labels = read_jsonl(a.labels) if a.labels else None
        diagnostics = None
    else:
        ap.error('give a records JSONL file or --demo')
    data = payload(records, metrics.compute(records, labels), diagnostics)
    Path(a.output).write_text(render(data))
    print(f'{len(records)} records -> {a.output}', file=sys.stderr)


if __name__ == '__main__':
    main()
