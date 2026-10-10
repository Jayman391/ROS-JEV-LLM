"""Off-policy evaluation of new fusion/selection parameters (FRAMEWORK §6.3, §12; phase A10).

Every logged decision is re-derived **exactly** from its record: the logged logits, Gumbel noise,
τ and κ (with ``current`` = the agent's previous selection) must reproduce the logged selection and
propensity. Decisions that don't reproduce were not sampled (dwell, a commit hold, the
irreversible rule, require, fallback) and are assumed unchanged under the new policy.

Under the new policy (β, γ, B, η, b_q, τ, κ, ρ) the logits, scores and selection are recomputed
with the *same* noise (recomputed from its key when ρ changes, so the coupling draw can flip).
Estimates, per (question set, question):

- ``direct``: labels give the reward of every class (1 if it is the true class), so the new
  policy's value on the logged states is exact. Not available for outcome rewards.
- ``snips``: self-normalised IPS on the logged reward, weight π_new(a_log) / propensity.
- ``dr``: doubly robust, with a per-class mean-reward model.
- ``ess``: effective sample size of the weights; 95% CIs by a cluster bootstrap over
  (run, group, epoch) — the unit that shares noise, so its decisions are correlated.

The state distribution is the logged one: per-decision values are exact, whole-run ones are not
(a different policy visits different states) — check big changes with a re-run or ``search``.
"""
from __future__ import annotations

import math
import random

from jev_ai.core.catalog import SELECT_ACTS, Catalog
from jev_ai.core.fusion import NoiseKey, confidence, gumbel_noise, pool
from jev_ai.core.metrics import _num, group_decisions, label_lookup, softmax

HELD_PHASES = ('FORCED', 'FALLBACK')


def _vec(v, n, default=None):
    v = [_num(x) for x in (v or [])]
    return v if len(v) == n else [default] * n


def _scores(z, names, current, pol, elig):
    return {i: (z[i] + (pol.kappa if names[i] == current else 0.0)) / pol.tau for i in elig}


def _select(q, base: dict, noise):
    """(index, {index: probability}) under the question's act."""
    if not base:
        return None, {}
    p = dict(zip(base, softmax(list(base.values()))))
    if q.act == 'sample':
        return max(base, key=lambda i: base[i] + noise[i]), p
    k = max(base, key=base.get)
    return k, {k: 1.0}


def _units(records, old: Catalog, new: Catalog, reward, counts):
    current = {}                      # (run, agent, qset) -> {question: class}
    for d in group_decisions(records):
        f, first = d['final'], d['records'][0]
        sk = (f.get('run_id', ''), f.get('agent', ''), f.get('question_set', ''))
        cur = dict(current.get(sk, {}))
        for r in d['records']:
            current.setdefault(sk, {}).update(
                {t['question']: t['selected'] for t in r.get('questions') or []
                 if t.get('selected')})
        qs_old, qs_new = old.question_sets.get(sk[2]), new.question_sets.get(sk[2])
        if qs_old is None or qs_new is None:
            counts['unknown_question_set'] += 1
            continue
        t0 = (first.get('header') or {}).get('stamp') or {}
        t0 = t0.get('sec', 0) + t0.get('nanosec', 0) * 1e-9
        directive = any(str(x).startswith('directive') for x in f.get('triggers') or [])
        for t in f.get('questions') or []:
            qn, a_log = t.get('question', ''), t.get('selected', '')
            q, qn_new = qs_old.questions.get(qn), qs_new.questions.get(qn)
            if q is None or q.act not in SELECT_ACTS or not a_log:
                continue
            names = q.class_names
            if list(t.get('classes') or []) != names or qn_new is None:
                counts['catalog_mismatch'] += 1
                continue
            r_log, r_all = reward(f, t)
            if r_log is None:
                continue
            epoch = math.floor(t0 / q.policy.epoch_s)
            u = {'key': f'{sk[2]}/{qn}', 'cluster': (sk[0], f.get('group', ''), epoch),
                 'a_log': a_log, 'p_log': 1.0, 'a_new': a_log, 'pi_new': {a_log: 1.0},
                 'r_log': r_log, 'r_all': r_all, 'sampled': False}
            _rederive(u, f, t, q, qn_new, cur.get(qn, ''), epoch, directive, counts)
            yield u


def _rederive(u, f, t, q, qn_new, current, epoch, directive, counts):
    """Reproduce the logged pick; if it reproduces, recompute it under the new policy."""
    qn, names, a_log = q.name, q.class_names, u['a_log']
    n = len(names)
    p = _vec(t.get('p_jev'), n)
    if t.get('required') or None in p or f.get('phase') in HELD_PHASES or \
            t.get('irreversible_path'):
        counts['held'] += 1
        return
    masked = [bool(m) for m in _vec(t.get('masked'), n, False)]
    irr = q.irreversible()
    elig = [i for i in range(n) if not masked[i] and names[i] not in irr]
    z_log = [x if x is not None else -math.inf for x in _vec(t.get('logits'), n)]
    noise = _vec(t.get('noise'), n, 0.0)
    k, pi = _select(q, _scores(z_log, names, current, q.policy, elig), noise)
    if k is None or names[k] != a_log or \
            abs(pi.get(k, 0.0) - (_num(t.get('propensity')) or 0.0)) > 1e-6:
        counts['held'] += 1           # dwell, commit hold, irreversible pending...
        return
    counts['reproduced'] += 1
    ql = _vec(t.get('q_llm'), n)
    ql = ql if None not in ql else None
    c = confidence(p)
    age = _num(t.get('llm_age_s')) or 0.0
    z0, _ = pool(p, ql, masked, [a in irr for a in names], c, age, q.policy,
                 list(q.bias) or None)
    if any(abs(a - b) > 1e-6 for a, b in zip(z0, z_log) if b > -math.inf):
        counts['logits_mismatch'] += 1   # the catalog is not the one the run used
    z1, _ = pool(p, ql, masked, [a in irr for a in names], c, age, qn_new.policy,
                 list(qn_new.bias) or None)
    noise1 = _noise(qn, names, noise, q, qn_new, f, epoch, directive, counts)
    k1, pi1 = _select(qn_new, _scores(z1, names, current, qn_new.policy, elig),
                      noise1)
    u.update(p_log=pi[k], a_new=names[k1], pi_new={names[i]: v for i, v in pi1.items()},
             sampled=True)


def _noise(qn, names, logged, q_old, q_new, f, epoch, directive, counts):
    """Logged noise, or recomputed from its key when the coupling ρ changed."""
    attr = 'coupling_directive' if directive else 'coupling'
    rho_old, rho_new = getattr(q_old.policy, attr), getattr(q_new.policy, attr)
    if rho_old == rho_new:
        return logged
    if directive:                     # the directive's epoch is not in the record
        counts['noise_kept'] += 1
        return logged
    key = NoiseKey(f.get('run_id', ''), f.get('agent', ''), f.get('group', ''), f't:{epoch}')
    g, _ = gumbel_noise(qn, names, key, rho_old)
    if any(abs(a - b) > 1e-9 for a, b in zip(g, logged)):
        counts['noise_kept'] += 1     # epoch boundary or app-set group: can't re-key safely
        return logged
    return gumbel_noise(qn, names, key, rho_new)[0]


def estimates(units: list[dict]) -> dict:
    n = len(units)
    if not n:
        return {'n': 0}
    w = [u['pi_new'].get(u['a_log'], 0.0) / u['p_log'] for u in units]
    r = [u['r_log'] for u in units]
    sw, sw2 = sum(w), sum(x * x for x in w)
    by = {}
    for u in units:
        by.setdefault(u['a_log'], []).append(u['r_log'])
    v_log = sum(r) / n
    qhat = {a: sum(v) / len(v) for a, v in by.items()}
    dr = sum(sum(p * qhat.get(a, v_log) for a, p in u['pi_new'].items()) +
             wi * (u['r_log'] - qhat.get(u['a_log'], v_log)) for u, wi in zip(units, w)) / n
    out = {'n': n, 'logged': v_log, 'snips': sum(a * b for a, b in zip(w, r)) / sw if sw else None,
           'dr': dr, 'ess': sw * sw / sw2 if sw2 else 0.0,
           'changed': sum(u['a_new'] != u['a_log'] for u in units) / n}
    if all(u['r_all'] is not None for u in units):
        out['direct'] = sum(u['r_all'].get(u['a_new'], 0.0) for u in units) / n
    return out


def bootstrap(units: list[dict], boot: int = 200, seed: int = 0) -> dict:
    """95% percentile CIs by resampling (run, group, epoch) clusters."""
    clusters = {}
    for u in units:
        clusters.setdefault(u['cluster'], []).append(u)
    keys = list(clusters)
    if len(keys) < 2:
        return {}
    rng = random.Random(seed)
    draws = {'snips': [], 'dr': [], 'direct': [], 'logged': []}
    for _ in range(boot):
        e = estimates([u for k in rng.choices(keys, k=len(keys)) for u in clusters[k]])
        for m in draws:
            if e.get(m) is not None:
                draws[m].append(e[m])
    return {m: [sorted(v)[int(0.025 * (len(v) - 1))], sorted(v)[int(0.975 * (len(v) - 1))]]
            for m, v in draws.items() if v}


def ope(records: list[dict], old: Catalog, new: Catalog, *, labels=None, rewards=None,
        boot: int = 200, seed: int = 0) -> dict:
    """Evaluate catalog ``new`` against the logged behaviour of catalog ``old``.

    ``labels``: Label dicts (reward 1 for the true class, known for every class).
    ``rewards``: dicts {run_id, agent, decision_id, question, reward} for the logged action."""
    if labels:
        truth = label_lookup(labels)

        def reward(f, t):
            y = truth(f, t)
            if y is None:
                return None, None
            r_all = {c: float(c == y) for c in t['classes']}
            return r_all.get(t['selected']), r_all
    else:
        rw = {(x.get('run_id', ''), x.get('agent', ''), int(x.get('decision_id') or 0),
               x.get('question', '')): float(x['reward']) for x in rewards or []}

        def reward(f, t):
            return rw.get((f.get('run_id', ''), f.get('agent', ''), int(f.get('decision_id', 0)),
                           t.get('question', ''))), None

    counts = {k: 0 for k in ('reproduced', 'held', 'logits_mismatch', 'noise_kept',
                             'catalog_mismatch', 'unknown_question_set')}
    by = {}
    for u in _units(records, old, new, reward, counts):
        by.setdefault(u['key'], []).append(u)
    return {'counts': counts,
            'questions': {k: {**estimates(v), 'ci95': bootstrap(v, boot, seed)}
                          for k, v in sorted(by.items())}}


def overrides(raw: dict, settings: dict[str, float]) -> dict:
    """Apply ``key=value`` (every question; else the catalog policy) and ``question.key=value``
    to a raw catalog dict. Question-level keys win over the policy, so a global key is written
    into every question too (``tau`` set on a question would otherwise hide it)."""
    from jev_ai.core.catalog import QUESTION_OVERRIDES
    import copy
    out = copy.deepcopy(raw)
    for k, v in settings.items():
        qn, _, key = k.rpartition('.')
        for qs in out['question_sets'].values():
            for name, q in qs['questions'].items():
                if (not qn or qn == name) and key in QUESTION_OVERRIDES | {'bias'}:
                    q[key] = v
        if not qn:
            out.setdefault('policy', {})
            out['policy'] = {**(out['policy'] or {}), key: v} if key != 'bias' else out['policy']
    return out
