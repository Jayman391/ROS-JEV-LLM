"""ros2 jev replay: re-ask Jev on logged states; report the shift under a variant catalog and
Jev's first-option (position) bias (§12). The bias is the gate for status-quo-first ordering.

    ros2 jev replay decisions.jsonl --catalog toy_patrol.yaml --backend mock \\
        --param rules=jev_ai_examples.mock_rules:patrol first_bias=0.1
    ros2 jev replay decisions.jsonl --catalog cat.yaml --variant reworded.yaml --backend typesafe

The backend runs in-process (keys from the environment, as for jev_server); ``--param`` takes the
backend's own parameters without the ``<backend>.`` prefix. ponytail: Jev only — an LLM replay
needs the augmentation prompt rebuilt per record; add it when prompt variants need testing.
"""
import asyncio
import json
from pathlib import Path

from ros2cli.verb import VerbExtension

from ros2jev.api import parse_kv, print_table


class ReplayVerb(VerbExtension):
    """Re-ask Jev on logged states: shift, accuracy and first-option bias."""

    def add_arguments(self, parser, cli_name):
        parser.add_argument('records', help='decisions JSONL (decision_logger)')
        parser.add_argument('--catalog', required=True, help='the catalog the run used')
        parser.add_argument('--variant', help='variant catalog (wording, criteria); same classes')
        parser.add_argument('--backend', default='typesafe')
        parser.add_argument('--param', nargs='+', default=[], metavar='KEY=VALUE')
        parser.add_argument('--model', default='')
        parser.add_argument('--api-key-file', default='')
        parser.add_argument('--labels', help='labels JSONL (adds accuracy)')
        parser.add_argument('--limit', type=int, default=200, help='max distinct states')
        parser.add_argument('--json', help='also write the summary as JSON')

    def main(self, *, args):
        from jev_ai.backends import BackendError, load_backend
        from jev_ai.core.catalog import load_catalog
        from jev_ai.core.metrics import label_lookup
        from jev_ai.core.replay import GATE, logged_states, summarize
        from jev_ai.viz.report import read_jsonl
        from jev_ai_servers.common import read_key
        from jev_ai_servers.jev_server import BUILTIN

        base = load_catalog(args.catalog)
        variant = load_catalog(args.variant) if args.variant else base
        cls = load_backend('jev_ai.jev_backends', args.backend, BUILTIN)
        key = read_key(cls.key_env, args.api_key_file)
        if cls.key_env and not key:
            raise SystemExit(f'backend {args.backend!r} needs {cls.key_env} (or --api-key-file)')
        backend = cls(parse_kv(args.param), key)
        truth = label_lookup(read_jsonl(args.labels)) if args.labels else None
        states = logged_states(read_jsonl(args.records), args.limit)
        print(f'replaying {len(states)} distinct states with backend {args.backend}')
        try:
            rows = asyncio.run(_replay(backend, base, variant, states, args.model, truth))
        except BackendError as e:
            raise SystemExit(f'backend error: {e}')
        out = summarize(rows)
        print_table(['question', 'n', 'shift tv', 'argmax moved', 'first class',
                     'first-option rate', '(reversed)', 'position bias', 'gate',
                     'accuracy logged/variant/reversed'],
                    [[q, s['n'], s['shift_tv'], s['argmax_changed'], s['first_class'],
                      s['first_option_rate'], s['first_option_rate_reversed'],
                      s['position_bias'], s['gate'],
                      '/'.join(f'{s["accuracy"][k]:.3f}' for k in ('logged', 'variant',
                                                                   'reversed'))
                      if s['accuracy'] else '-'] for q, s in out.items()])
        print(f'\nposition bias: probability the first class gains from being listed first '
              f'(gate |bias| < {GATE}).')
        if args.json:
            Path(args.json).write_text(json.dumps(out, indent=1))
        return 0 if all(s['gate'] == 'ok' for s in out.values()) else 2


async def _replay(backend, base, variant, states, model, truth):
    from jev_ai.backends import JevQuestion
    await backend.check()
    rows = []
    try:
        for st in states:
            qs_base = base.question_sets.get(st['question_set'])
            qs_var = variant.question_sets.get(st['question_set'])
            if qs_base is None or qs_var is None:
                continue
            names = [qn for qn in st['p_jev']
                     if qn in qs_base.questions and qn in qs_var.questions]
            asks = {}
            for order in ('variant', 'reversed'):
                qs = []
                for qn in names:
                    q = qs_var.questions[qn]
                    pairs = list(zip(q.class_names, q.criteria))
                    if order == 'reversed':
                        pairs.reverse()
                    qs.append(JevQuestion(qn, q.type, q.instructions, [c for c, _ in pairs],
                                          [k for _, k in pairs]))
                res = await backend.decide(st['state'], qs, model)
                asks[order] = {q.name: dict(zip(q.classes, a.probabilities))
                               for q, a in zip(qs, res.answers) if a is not None}
            for qn in names:
                classes = qs_base.questions[qn].class_names
                if any(qn not in asks[o] or set(asks[o][qn]) != set(classes) for o in asks):
                    continue
                rows.append({'question': qn, 'classes': classes, 'logged': st['p_jev'][qn],
                             'variant': [asks['variant'][qn][c] for c in classes],
                             'reversed': [asks['reversed'][qn][c] for c in classes],
                             'truth': truth(st['final'], {'question': qn}) if truth else None})
    finally:
        await backend.close()
    return rows
