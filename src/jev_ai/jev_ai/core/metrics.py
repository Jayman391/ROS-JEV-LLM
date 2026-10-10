"""Decision metrics from DecisionRecords (FRAMEWORK §3, §5.3, §6, §11, §12; phase A5.5).

``compute(records, labels=None)`` turns records (``records.record_dict`` shape, from memory or
JSONL, so masked logits may be -inf or null) into one JSON-safe dict: an ``overall`` section, one
section per question set, and ``help`` with a plain-language description of every metric for
the dashboard's tooltips. Pure stdlib, no jev_ai imports, so it also runs on a bare python3.

A *decision* is all records sharing (run_id, agent, question_set, decision_id): one per phase
transition. Its *outcome* reads the phase path:

    COMMITTED    committed straight away (no augmentation requested)
    KEPT         augmentation arrived and agreed with the provisional pick
    REVISED      augmentation arrived and changed the pick (REVISED -> COMMITTED)
    LLM_MISSING  augmentation requested but timed out / failed: Jev-only commit (§11)
    FALLBACK     catalog fallback (irreversible timeout, N_fail Jev failures)
    FORCED       require() decided it
    OPEN         still PROVISIONAL (awaiting the LLM, or superseded by a newer decision)

Rates are per wall-clock time; coupling epochs use the node clock, as the sampler does (§6.3).
"""
from __future__ import annotations

import math
import time
from collections import Counter, defaultdict

OUTCOMES = ('COMMITTED', 'KEPT', 'REVISED', 'LLM_MISSING', 'FALLBACK', 'FORCED', 'OPEN')
PHASES = ('PROVISIONAL', 'REVISED', 'KEPT', 'COMMITTED', 'FALLBACK', 'FORCED')
NICE_BUCKETS_S = (5, 10, 30, 60, 300, 900, 1800, 3600, 6 * 3600, 86400)
MAX_SERIES_POINTS = 200

# key -> (label, unit, what it means / what good looks like). Keys are '<section>.<metric>'.
HELP = {
    'volume.decisions': ('Decisions', '', 'Distinct decisions (all phases of one decision_id '
                         'count once).'),
    'volume.records': ('Records', '', 'DecisionRecords: one per phase transition, 1-3 per '
                       'decision.'),
    'volume.agents': ('Agents', '', 'Distinct agents that made decisions.'),
    'volume.span_s': ('Time span', 's', 'Wall time between the first and last record.'),
    'volume.decisions_per_min': ('Decisions / min', '/min', 'Decision throughput across the '
                                 'fleet. Should match tick rates plus app triggers.'),
    'phases.outcomes': ('Outcome mix', '', 'How decisions ended. Mostly KEPT/COMMITTED is '
                        'healthy; many LLM_MISSING or FALLBACK means System 2 is not keeping '
                        'up.'),
    'phases.final_phase': ('Final phase', '', 'Phase of the last record of each decision.'),
    'phases.revision_rate': ('Revision rate', '%', 'REVISED / (REVISED + KEPT): how often the '
                             'LLM changed an already-acted provisional choice. A few percent '
                             'is normal; high means Jev and the LLM often disagree.'),
    'phases.fallback_rate': ('Fallback rate', '%', 'Share of decisions that ended on the '
                             'catalog fallback (LLM timeout on an irreversible class, or '
                             'repeated Jev failures). Should be near 0.'),
    'phases.forced_rate': ('Forced rate', '%', 'Share decided by the app\'s require() rule, '
                           'no models consulted.'),
    'phases.llm_missing_rate': ('LLM missing', '%', 'Augmentation was requested but never '
                                'arrived (timeout, error, open breaker): committed on Jev '
                                'alone.'),
    'phases.degraded_rate': ('Degraded rate', '%', 'LLM_MISSING + FALLBACK: decisions that ran '
                             'without System 2 when it was wanted. Lower is better.'),
    'phases.over_time': ('Outcomes over time', '', 'Decisions per time bucket, stacked by '
                         'outcome.'),
    'augmentation.llm_share': ('With LLM', '%', 'Share of decisions whose final record carries '
                               'LLM weights.'),
    'augmentation.agreement': ('LLM-Jev agreement', '%', 'Share of questions where the LLM\'s '
                               'top class equals Jev\'s top class (unmasked classes only). '
                               'Very high: the LLM adds little; very low: one of them is '
                               'miscalibrated.'),
    'augmentation.beta': ('LLM weight beta', '', 'Effective LLM weight per question (§6.2). It '
                          'grows as Jev\'s confidence falls and decays with the age of the '
                          'LLM answer.'),
    'augmentation.mean_abs_delta': ('Mean |delta|', 'nats', 'Average LLM nudge on unmasked '
                                    'classes, delta = logit - log p_jev. Bounded by B = log 20 '
                                    '= 3.0 nats.'),
    'augmentation.argmax_flip_rate': ('LLM flipped argmax', '%', 'Share of augmented questions '
                                      'where fusion moved the top class away from Jev\'s.'),
    'augmentation.changed_selection_rate': ('Augmentation changed pick', '%', 'REVISED / '
                                            'decisions with LLM: how often augmentation '
                                            'actually changed what the robot did.'),
    'jev.confidence': ('Jev confidence', '', 'c = 1 - H(p)/log|A| (§6.1). Mass near 0 means '
                       'Jev is unsure and the LLM gets more weight.'),
    'jev.first_option_rate': ('First-option rate', '%', 'Share of questions where Jev\'s top '
                              'class is the first listed class. Far above the true rate '
                              'signals Jev\'s first-option bias (§12 replay).'),
    'jev.selection': ('Selections per class', '', 'How often each class was finally selected, '
                      'per question.'),
    'behaviour.switch_rate': ('Switches / agent / min', '/min', 'How often an agent\'s selected '
                              'class changes. High values mean dithering; T_dwell and kappa '
                              'damp it.'),
    'behaviour.propensity': ('Propensity', '', 'Probability the sampler gave the class it '
                             'picked (sampled questions only). Needed for off-policy '
                             'evaluation.'),
    'behaviour.low_propensity_share': ('Propensity < 0.05', '%', 'Share of sampled picks with '
                                       'propensity below 0.05. These dominate OPE variance; '
                                       'keep it small.'),
    'behaviour.irreversible': ('Irreversible commits', '', 'Picks made via the irreversible '
                               'rule (§6.4): margin on Jev and fused logits, plus '
                               'augmentation.'),
    'behaviour.flags': ('LLM flags', '', 'Flags the LLM raised (novel_situation, '
                        'option_missing, data_suspect).'),
    'performance.jev_latency_ms': ('Jev latency', 'ms', 'Jev service round trip per decision. '
                                   'p95 should sit well under jev_timeout (1 s).'),
    'performance.llm_latency_ms': ('LLM latency', 'ms', 'LLM augmentation latency per decision. '
                                   'p95 must stay under llm_timeout (6 s) or decisions go '
                                   'LLM_MISSING.'),
    'performance.tokens': ('Tokens', '', 'Jev input, LLM input and LLM output tokens.'),
    'performance.cost_usd': ('Total cost', '$', 'LLM spend over the span.'),
    'performance.usd_per_hour': ('Cost / hour', '$/h', 'Spend rate over the span.'),
    'performance.cost_series': ('Cumulative cost', '$', 'LLM spend over time.'),
    'coupling.agreement': ('Fleet agreement', '%', 'Same group, question and 5 s epoch: share '
                           'of agent pairs that picked the same class.'),
    'coupling.agreement_shared': ('Agreement, shared noise', '%', 'Pairs where both agents '
                                  'used the group\'s Gumbel noise. Should be higher than with '
                                  'independent noise.'),
    'coupling.agreement_independent': ('Agreement, own noise', '%', 'Pairs where at least one '
                                       'agent used its own noise.'),
    'calibration': ('Calibration', '', 'Needs labels. Reliability: predicted top-class '
                    'probability vs how often it was right. ECE (lower is better, 0 = '
                    'perfect) and NLL for Jev, the LLM and the fused result.'),
}


# --- small helpers ---------------------------------------------------------------------------

def _num(x) -> float | None:
    """Finite float or None (JSONL has null where memory has NaN / -inf)."""
    if x is None:
        return None
    x = float(x)
    return x if math.isfinite(x) else None


def _t(stamp: dict | None) -> float:
    return stamp['sec'] + stamp['nanosec'] * 1e-9 if stamp else 0.0


def wall_time(r: dict) -> float:
    return _t(r.get('wall_stamp')) or _t((r.get('header') or {}).get('stamp'))


def _node(r: dict) -> float:
    return _t((r.get('header') or {}).get('stamp')) or wall_time(r)


def _rate(n, d):
    return n / d if d else None


def _mean(v):
    return sum(v) / len(v) if v else None


def _argmax(v: list, allowed=None) -> int | None:
    idx = [i for i in range(len(v)) if v[i] is not None and (allowed is None or allowed[i])]
    return max(idx, key=lambda i: v[i]) if idx else None


def _unmasked(t: dict) -> list[bool]:
    m = t.get('masked') or []
    return [not (m[i] if i < len(m) else False) for i in range(len(t.get('classes') or []))]


def softmax(logits: list) -> list[float]:
    """Fused distribution from logged logits; null / -inf (masked) -> 0."""
    v = [_num(x) for x in logits]
    finite = [x for x in v if x is not None]
    if not finite:
        return [1.0 / len(v)] * len(v) if v else []
    m = max(finite)
    e = [math.exp(x - m) if x is not None else 0.0 for x in v]
    s = sum(e)
    return [x / s for x in e]


def percentiles(v: list[float], ps=(50, 95, 99)) -> dict:
    """Nearest-rank percentiles: {'p50': .., 'p95': .., 'p99': .., 'n': ..}."""
    s = sorted(v)
    out = {f'p{p}': (s[max(0, math.ceil(p / 100 * len(s)) - 1)] if s else None) for p in ps}
    out['n'] = len(s)
    return out


def nice(x: float) -> float:
    """Smallest 1, 2, 2.5 or 5 x 10^k that is >= x (x > 0)."""
    p = 10.0 ** math.floor(math.log10(x))
    return next(m * p for m in (1, 2, 2.5, 5, 10) if m * p >= x * (1 - 1e-9))


def histogram(v: list[float], lo: float, hi: float, bins: int = 10) -> dict:
    """Equal-width bins on [lo, hi]; values outside clamp into the end bins."""
    hi = hi if hi > lo else lo + 1.0
    w = (hi - lo) / bins
    counts = [0] * bins
    for x in v:
        counts[min(bins - 1, max(0, int((x - lo) / w)))] += 1
    return {'edges': [lo + i * w for i in range(bins + 1)], 'counts': counts}


def _downsample(points: list) -> list:
    if len(points) <= MAX_SERIES_POINTS:
        return points
    k = math.ceil(len(points) / MAX_SERIES_POINTS)
    return points[::k] + ([points[-1]] if (len(points) - 1) % k else [])


# --- decisions -------------------------------------------------------------------------------

def decision_key(r: dict) -> tuple:
    return (r.get('run_id', ''), r.get('agent', ''), r.get('question_set', ''),
            int(r.get('decision_id', 0)))


def outcome(phases: list[str]) -> str:
    last = phases[-1]
    if last in ('FORCED', 'FALLBACK'):
        return last
    if 'REVISED' in phases:
        return 'REVISED'
    if last == 'KEPT':
        return 'KEPT'
    if last == 'COMMITTED':
        return 'LLM_MISSING' if 'PROVISIONAL' in phases else 'COMMITTED'
    return 'OPEN'


def group_decisions(records: list[dict]) -> list[dict]:
    """Records -> decisions, each {'key', 'records' (time order), 'final', 'outcome', 't0',
    't1'}, sorted by first record time."""
    by = defaultdict(list)
    for r in records:
        by[decision_key(r)].append(r)
    out = []
    for key, rs in by.items():
        rs.sort(key=wall_time)              # stable: same-stamp records keep emission order
        out.append({'key': key, 'records': rs, 'final': rs[-1],
                    'outcome': outcome([r.get('phase', '') for r in rs]),
                    't0': wall_time(rs[0]), 't1': wall_time(rs[-1])})
    out.sort(key=lambda d: d['t0'])
    return out


# --- sections --------------------------------------------------------------------------------

def _volume(decs, records):
    ts = [wall_time(r) for r in records]
    span = max(ts) - min(ts) if ts else 0.0
    return {'decisions': len(decs), 'records': len(records),
            'agents': len({r.get('agent', '') for r in records}),
            'span_s': span, 't0': min(ts) if ts else None, 't1': max(ts) if ts else None,
            'decisions_per_min': _rate(len(decs), span / 60.0) if span > 0 else None}


def _phases(decs):
    n = len(decs)
    oc = Counter(d['outcome'] for d in decs)
    fp = Counter(d['final'].get('phase', '') for d in decs)
    ts = [d['t0'] for d in decs]
    over = None
    if ts:
        span = max(ts) - min(ts)
        b = next((s for s in NICE_BUCKETS_S if span / s <= 40), NICE_BUCKETS_S[-1])
        t0 = math.floor(min(ts) / b) * b
        nb = int((max(ts) - t0) // b) + 1
        series = {o: [0] * nb for o in OUTCOMES}
        for d in decs:
            series[d['outcome']][int((d['t0'] - t0) // b)] += 1
        over = {'bucket_s': b, 't0': t0, 'series': {o: c for o, c in series.items() if any(c)}}
    return {
        'outcomes': {o: oc.get(o, 0) for o in OUTCOMES},
        'outcome_rates': {o: _rate(oc.get(o, 0), n) for o in OUTCOMES},
        'final_phase': {p: fp.get(p, 0) for p in PHASES},
        'revision_rate': _rate(oc['REVISED'], oc['REVISED'] + oc['KEPT']),
        'fallback_rate': _rate(oc['FALLBACK'], n),
        'forced_rate': _rate(oc['FORCED'], n),
        'llm_missing_rate': _rate(oc['LLM_MISSING'], n),
        'degraded_rate': _rate(oc['LLM_MISSING'] + oc['FALLBACK'], n),
        'over_time': over,
    }


def _per_question(decs):
    """Augmentation and Jev metrics, overall and per question, from each decision's final
    record."""
    qstats = defaultdict(lambda: {'n': 0, 'agree': [], 'flip': [], 'abs_delta': [],
                                  'first': [], 'selection': Counter(), 'classes': []})
    betas, conf, with_llm = [], [], 0
    for d in decs:
        has_llm = False
        for t in d['final'].get('questions') or []:
            qs = qstats[t.get('question', '')]
            qs['n'] += 1
            classes = t.get('classes') or []
            if len(classes) > len(qs['classes']):
                qs['classes'] = list(classes)
            if t.get('selected'):
                qs['selection'][t['selected']] += 1
            p = [_num(x) for x in t.get('p_jev') or []]
            if not p or None in p or sum(p) <= 0:
                continue                # Jev failed for this question
            c = _num(t.get('jev_confidence'))
            if c is not None:
                conf.append(c)
            qs['first'].append(_argmax(p) == 0)
            q = [_num(x) for x in t.get('q_llm') or []]
            if not q or len(q) != len(p):
                continue
            has_llm = True
            ok = _unmasked(t)
            qs['agree'].append(_argmax(q, ok) == _argmax(p, ok))
            b = _num(t.get('beta'))
            if b is not None:
                betas.append(b)
            s = sum(p)
            z = [_num(x) for x in t.get('logits') or []]
            deltas = [abs(z[i] - math.log(max(p[i] / s, 1e-12)))
                      for i in range(min(len(z), len(p))) if ok[i] and z[i] is not None]
            if deltas:
                qs['abs_delta'].append(_mean(deltas))
            jev_z = [math.log(max(x / s, 1e-12)) for x in p]
            if len(z) == len(p):
                qs['flip'].append(_argmax(z, ok) != _argmax(jev_z, ok))
        with_llm += has_llm

    def frac(v):
        return _rate(sum(v), len(v))

    def pooled(k):
        return [x for qs in qstats.values() for x in qs[k]]

    by_q = {name: {'n': qs['n'], 'agreement': frac(qs['agree']),
                   'mean_abs_delta': _mean(qs['abs_delta']),
                   'argmax_flip_rate': frac(qs['flip']),
                   'first_option_rate': frac(qs['first']),
                   'selection': {c: qs['selection'].get(c, 0) for c in qs['classes']}}
            for name, qs in qstats.items()}
    revised = sum(d['outcome'] == 'REVISED' for d in decs)
    aug = {'llm_share': _rate(with_llm, len(decs)), 'agreement': frac(pooled('agree')),
           'beta': histogram(betas, 0.0, max([1.0] + betas), 10),
           'beta_mean': _mean(betas),
           'mean_abs_delta': _mean(pooled('abs_delta')),
           'argmax_flip_rate': frac(pooled('flip')),
           'changed_selection_rate': _rate(revised, with_llm)}
    jev = {'confidence': histogram(conf, 0.0, 1.0, 10), 'confidence_mean': _mean(conf),
           'first_option_rate': frac(pooled('first'))}
    return aug, jev, by_q


def _behaviour(decs, records):
    seq = defaultdict(list)       # (run, agent, qset, question) -> [(t, selected)]
    agent_span = defaultdict(list)
    for r in records:
        t = wall_time(r)
        agent_span[(r.get('run_id', ''), r.get('agent', ''))].append(t)
        for tr in r.get('questions') or []:
            if tr.get('selected'):
                seq[(r.get('run_id', ''), r.get('agent', ''), r.get('question_set', ''),
                     tr.get('question', ''))].append((t, tr['selected']))
    switches, by_q = 0, Counter()
    for k, v in seq.items():
        v.sort(key=lambda x: x[0])
        n = sum(a[1] != b[1] for a, b in zip(v, v[1:]))
        switches += n
        by_q[k[3]] += n
    agent_min = sum(max(ts) - min(ts) for ts in agent_span.values()) / 60.0

    # Propensity only for sampled questions: ones never logged below 1.0 aren't sampled
    # (expectation/threshold/argmax). ponytail: a sampled question with one eligible class
    # looks the same; the act type isn't in the record.
    props = defaultdict(list)
    irreversible, flags = 0, Counter()
    for d in decs:
        f = d['final']
        flags.update(f.get('llm_flags') or [])
        for t in f.get('questions') or []:
            irreversible += bool(t.get('irreversible_path'))
            p = _num(t.get('propensity'))
            if p is not None and t.get('selected') and not t.get('required') \
                    and not t.get('irreversible_path') and f.get('phase') not in ('FORCED',
                                                                                  'FALLBACK'):
                props[t.get('question', '')].append(p)
    sampled = [p for v in props.values() if any(x < 1.0 for x in v) for p in v]
    return {'switches': switches,
            'switch_rate': _rate(switches, agent_min) if agent_min > 0 else None,
            'switch_rate_by_question': {q: (_rate(n, agent_min) if agent_min > 0 else None)
                                        for q, n in by_q.items()},
            'propensity': histogram(sampled, 0.0, 1.0, 20),
            'low_propensity_share': _rate(sum(p < 0.05 for p in sampled), len(sampled)),
            'irreversible': irreversible,
            'flags': dict(flags.most_common())}


def _performance(decs, span_s):
    jl, ll = [], []
    tok = {'jev_input': 0, 'llm_input': 0, 'llm_output': 0}
    spend = []
    for d in decs:
        rs = d['records']
        # the engine may repeat Jev/LLM accounting on every phase record: count once
        j = max(_num(r.get('jev_latency_ms')) or 0.0 for r in rs)
        m = max(_num(r.get('llm_latency_ms')) or 0.0 for r in rs)
        if j > 0:
            jl.append(j)
        if m > 0:
            ll.append(m)
        tok['jev_input'] += max(int(r.get('jev_input_tokens') or 0) for r in rs)
        tok['llm_input'] += max(int(r.get('llm_input_tokens') or 0) for r in rs)
        tok['llm_output'] += max(int(r.get('llm_output_tokens') or 0) for r in rs)
        c = max(_num(r.get('cost_usd')) or 0.0 for r in rs)
        if c > 0:
            spend.append((d['t1'], c))
    cost, cum = 0.0, []
    for t, c in sorted(spend):
        cost += c
        cum.append([t, cost])

    def lat(v):
        top = (percentiles(v)['p99'] or 20.0) * 1.05
        w = nice(top / 20)                  # round bin edges, 10-20 bins up to ~p99
        k = math.ceil(top / w)
        return {**percentiles(v), 'mean': _mean(v), 'hist': histogram(v, 0.0, k * w, k)}

    return {'jev_latency_ms': lat(jl), 'llm_latency_ms': lat(ll), 'tokens': tok,
            'cost_usd': cost, 'usd_per_hour': cost / max(span_s / 3600.0, 1 / 60.0),
            'cost_series': _downsample(cum)}


def _coupling(decs, epoch_s):
    """Within-group agreement per (group, question, epoch), split by group-noise use."""
    cells = defaultdict(dict)      # (run, group, question, epoch) -> agent -> (sel, group_noise)
    for d in decs:
        f = d['final']
        e = math.floor(_node(f) / epoch_s)
        for t in f.get('questions') or []:
            if t.get('selected'):
                cells[(f.get('run_id', ''), f.get('group', ''), t.get('question', ''), e)][
                    f.get('agent', '')] = (t['selected'], bool(t.get('group_noise')))
    tally = defaultdict(lambda: [0, 0])      # kind -> [agree, pairs]
    by_q = defaultdict(lambda: [0, 0])
    for (_, _, q, _), agents in cells.items():
        v = list(agents.values())
        for i in range(len(v)):
            for j in range(i + 1, len(v)):
                same = v[i][0] == v[j][0]
                kind = 'shared' if v[i][1] and v[j][1] else 'independent'
                for k in (tally[kind], tally['all'], by_q[q]):
                    k[0] += same
                    k[1] += 1
    return {'agreement': _rate(*tally['all']), 'pairs': tally['all'][1],
            'agreement_shared': _rate(*tally['shared']), 'pairs_shared': tally['shared'][1],
            'agreement_independent': _rate(*tally['independent']),
            'pairs_independent': tally['independent'][1],
            'by_question': {q: {'agreement': _rate(a, n), 'pairs': n}
                            for q, (a, n) in by_q.items()},
            'epoch_s': epoch_s}


def reliability(pairs: list[tuple[list[float], int]], bins: int = 10) -> dict:
    """Top-label reliability bins, ECE and NLL for (distribution, true index) pairs."""
    acc = [[0, 0.0, 0] for _ in range(bins)]   # n, sum conf, n correct
    nll = 0.0
    for p, y in pairs:
        k = max(range(len(p)), key=p.__getitem__)
        b = acc[min(bins - 1, int(p[k] * bins))]
        b[0] += 1
        b[1] += p[k]
        b[2] += k == y
        nll -= math.log(max(p[y], 1e-12))
    n = len(pairs)
    out = [{'lo': i / bins, 'hi': (i + 1) / bins, 'n': c, 'conf': _rate(s, c),
            'acc': _rate(r, c)} for i, (c, s, r) in enumerate(acc)]
    ece = sum(b['n'] / n * abs(b['acc'] - b['conf']) for b in out if b['n']) if n else None
    return {'n': n, 'ece': ece, 'nll': nll / n if n else None, 'bins': out}


def _calibration(decs, labels):
    """Labels join on (run_id, subject or agent, decision_id, question). ponytail: labels with
    decision_id 0 (join by subject + time window, Label.msg) are skipped for now."""
    truth = {(x.get('run_id', ''), x.get('subject', ''), int(x.get('decision_id') or 0),
              x.get('question', '')): x.get('true_class') for x in labels}
    src = defaultdict(lambda: {'jev': [], 'llm': [], 'fused': []})
    for d in decs:
        f = d['final']
        subject = f.get('subject') or f.get('agent', '')
        for t in f.get('questions') or []:
            y = truth.get((f.get('run_id', ''), subject, int(f.get('decision_id', 0)),
                           t.get('question', '')))
            classes = t.get('classes') or []
            if y not in classes:
                continue
            yi = classes.index(y)
            p = [_num(x) for x in t.get('p_jev') or []]
            if len(p) == len(classes) and None not in p and sum(p) > 0:
                src[t['question']]['jev'].append(([x / sum(p) for x in p], yi))
            q = [_num(x) for x in t.get('q_llm') or []]
            if len(q) == len(classes) and None not in q and sum(q) > 0:
                src[t['question']]['llm'].append(([x / sum(q) for x in q], yi))
            if len(t.get('logits') or []) == len(classes) and len(p) == len(classes):
                src[t['question']]['fused'].append((softmax(t['logits']), yi))
    return {q: {k: reliability(v) for k, v in s.items()} for q, s in src.items()}


def section(decs: list[dict], labels=None, epoch_s: float = 5.0) -> dict:
    records = [r for d in decs for r in d['records']]
    vol = _volume(decs, records)
    aug, jev, by_q = _per_question(decs)
    out = {'volume': vol, 'phases': _phases(decs), 'augmentation': aug, 'jev': jev,
           'by_question': by_q, 'behaviour': _behaviour(decs, records),
           'performance': _performance(decs, vol['span_s']),
           'coupling': _coupling(decs, epoch_s)}
    out['calibration'] = _calibration(decs, labels) if labels else None
    return out


def compute(records: list[dict], labels: list[dict] | None = None,
            epoch_s: float = 5.0) -> dict:
    """All metrics, JSON-safe. ``epoch_s`` is the policy's coupling epoch (§14, default 5 s)."""
    decs = group_decisions(records)
    by_qs = defaultdict(list)
    for d in decs:
        by_qs[d['key'][2]].append(d)
    return json_safe({
        'generated_at': time.time(),
        'has_labels': bool(labels),
        'question_sets': sorted(by_qs),
        'overall': section(decs, labels, epoch_s),
        'by_question_set': {qs: section(v, labels, epoch_s) for qs, v in sorted(by_qs.items())},
        'help': {k: {'label': lbl, 'unit': unit, 'desc': desc}
                 for k, (lbl, unit, desc) in HELP.items()},
    })


def json_safe(obj):
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: json_safe(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [json_safe(v) for v in obj]
    return obj
