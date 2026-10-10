"""Calibration of the fusion policy from labelled records (FRAMEWORK §6.2, §12; phase A10).

Per (question set, question), a convex **conditional-logit** fit of the §6.2 pooling rule

    z(a) = log p_J(a) + w1·ℓ(a)·d + w2·(1−c)^γ·ℓ(a)·d + b(a)        d = exp(−age / T_stale)

which is exactly ``pool()`` with β_min = w1, β_max = w1 + w2 and per-class bias b_q = b. log p_J is
an offset (coefficient 1: Jev is the base model, the policy only moves it). γ is chosen on a
grid, then B (the clip on the LLM term) on a grid of the §14 range. Fitted values are clamped to
the §14 ranges and the reported "calibrated" scores are those of the clamped patch.

ponytail: in-sample scores (2 + |classes| parameters on hundreds of labels); the one-sided clip on
irreversible classes is not modelled (it only lowers them, so the fit is conservative there).
"""
from __future__ import annotations

import math

from jev_ai.core.fusion import P_FLOOR, confidence, llm_logpref
from jev_ai.core.metrics import _num, group_decisions, label_lookup, reliability, softmax

GAMMAS = (0.5, 1.0, 1.5, 2.0, 2.5, 3.0)
BOUNDS = tuple(math.log(x) for x in (2, 5, 10, 20, 50, 100))
LAMBDA_B, LAMBDA_W = 1.0, 1e-3          # ridge on biases (identifiability) and on the β weights


def labelled(records: list[dict], labels: list[dict]):
    """(final record, question trace, true class) per labelled question of each decision."""
    truth = label_lookup(labels)
    for d in group_decisions(records):
        f = d['final']
        for t in f.get('questions') or []:
            y = truth(f, t)
            if y in (t.get('classes') or []):
                yield f, t, y


def _rows(items, eta: float, T_stale: float):
    """Fit rows: offsets, LLM features ℓ·d, Jev confidence, masks, label index."""
    rows = []
    for _, t, y in items:
        classes = t['classes']
        p = [_num(x) for x in t.get('p_jev') or []]
        if len(p) != len(classes) or None in p or sum(p) <= 0:
            continue
        masked = [bool(m) for m in (t.get('masked') or [False] * len(classes))]
        yi = classes.index(y)
        if masked[yi]:
            continue                       # a masked label can't be predicted by any policy
        s = sum(p)
        q = [_num(x) for x in t.get('q_llm') or []]
        has_q = len(q) == len(classes) and None not in q and sum(q) > 0
        d = math.exp(-max(0.0, _num(t.get('llm_age_s')) or 0.0) / T_stale)
        ell = [v * d for v in llm_logpref(q, masked, eta)] if has_q else [0.0] * len(classes)
        rows.append({'off': [math.log(max(x / s, P_FLOOR)) for x in p], 'ell': ell,
                     'c': confidence([x / s for x in p]), 'masked': masked, 'y': yi,
                     'p': [x / s for x in p], 'q': [x / sum(q) for x in q] if has_q else None,
                     'logits': t.get('logits') or []})
    return rows


def _z(r, w1, w2, b, gamma, B=math.inf):
    k = (1.0 - r['c']) ** gamma
    return [-math.inf if r['masked'][i] else
            r['off'][i] + max(-B, min(B, (w1 + w2 * k) * r['ell'][i])) + b[i]
            for i in range(len(r['off']))]


def nll(rows, w1, w2, b, gamma, B=math.inf) -> float:
    total = 0.0
    for r in rows:
        z = _z(r, w1, w2, b, gamma, B)
        m = max(z)
        total += m + math.log(sum(math.exp(v - m) for v in z if v > -math.inf)) - z[r['y']]
    return total / max(1, len(rows))


def _fit(rows, K: int, gamma: float, use_llm: bool):
    """Newton on the convex penalised NLL; θ = [w1, w2, b_0..b_K-1]."""
    n = 2 + K
    theta = [0.0] * n
    lam = [LAMBDA_W, LAMBDA_W] + [LAMBDA_B] * K

    def feats(r):
        k = (1.0 - r['c']) ** gamma
        return [[r['ell'][a], k * r['ell'][a]] + [float(a == j) for j in range(K)]
                for a in range(K)]

    X = [feats(r) for r in rows]

    def objective(th):
        return nll(rows, th[0], th[1], th[2:], gamma) * len(rows) + \
            0.5 * sum(lam[j] * th[j] ** 2 for j in range(n))

    f = objective(theta)
    for _ in range(50):
        g = [lam[j] * theta[j] for j in range(n)]
        H = [[lam[i] if i == j else 0.0 for j in range(n)] for i in range(n)]
        for r, x in zip(rows, X):
            p = softmax(_z(r, theta[0], theta[1], theta[2:], gamma))
            mean = [sum(p[a] * x[a][j] for a in range(K)) for j in range(n)]
            for j in range(n):
                g[j] += mean[j] - x[r['y']][j]
            for a in range(K):
                if p[a]:
                    dx = [x[a][j] - mean[j] for j in range(n)]
                    for i in range(n):
                        for j in range(n):
                            H[i][j] += p[a] * dx[i] * dx[j]
        if not use_llm:                       # no LLM data: β weights are not identifiable
            g[0] = g[1] = 0.0
        step = _solve(H, g)
        t = 1.0
        while t > 1e-6:
            cand = [theta[j] - t * step[j] for j in range(n)]
            fc = objective(cand)
            if fc <= f:
                break
            t /= 2
        if t <= 1e-6 or f - fc < 1e-10:
            break
        theta, f = cand, fc
    return theta


def _solve(A, b):
    """Gauss-Jordan with partial pivoting (A is SPD and tiny)."""
    n = len(b)
    M = [row[:] + [b[i]] for i, row in enumerate(A)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        M[c], M[p] = M[p], M[c]
        if abs(M[c][c]) < 1e-12:
            continue
        for r in range(n):
            if r != c:
                k = M[r][c] / M[c][c]
                M[r] = [M[r][j] - k * M[c][j] for j in range(n + 1)]
    return [M[i][n] / M[i][i] if abs(M[i][i]) >= 1e-12 else 0.0 for i in range(n)]


def fit_question(rows, classes: list[str], eta: float = 1e-3) -> dict:
    """Best γ (grid), then B (grid) for one question; clamped to the §14 ranges."""
    K = len(classes)
    use_llm = any(r['q'] for r in rows)
    best = None
    # γ trades off against β_min (the NLL surface is flat along that ridge), so each γ is
    # scored by the *clamped* patch it would emit, not by its unconstrained optimum.
    for gamma in GAMMAS if use_llm else (2.0,):
        th = _fit(rows, K, gamma, use_llm)
        lo = min(0.5, max(0.0, th[0]))
        hi = min(3.0, max(0.5, th[0] + th[1]))
        v = nll(rows, lo, hi - lo, th[2:], gamma)
        if best is None or v < best[0] - 1e-9:
            best = (v, gamma, th, lo, hi)
    _, gamma, th, beta_min, beta_max = best
    raw = {'beta_min': th[0], 'beta_max': th[0] + th[1]}
    b = th[2:]
    mean_b = sum(b) / K
    b = [v - mean_b for v in b]                 # softmax-invariant: centre for readability
    clamped = [k for k, v in (('beta_min', beta_min), ('beta_max', beta_max))
               if v != raw[k] and use_llm]
    w1, w2 = beta_min, beta_max - beta_min
    B = math.log(20)
    if use_llm:     # ties go to the larger (less restrictive) bound
        B = min(reversed(BOUNDS), key=lambda x: nll(rows, w1, w2, b, gamma, x) - 1e-9 * x)
    bias = [max(-B, min(B, v)) for v in b]
    if bias != b:
        clamped.append('bias')
    after = [(softmax(_z(r, w1, w2, bias, gamma, B)), r['y']) for r in rows]
    out = {'n': len(rows), 'n_llm': sum(bool(r['q']) for r in rows),
           'bias': {c: round(v, 4) for c, v in zip(classes, bias)},
           'clamped': clamped, 'raw': {k: round(v, 4) for k, v in raw.items()}}
    if use_llm:
        out.update(beta_min=round(beta_min, 4), beta_max=round(beta_max, 4), gamma=gamma,
                   B=B)       # unrounded: log 2 rounded down would fail the §14 range check
    pairs = {'jev': [(r['p'], r['y']) for r in rows],
             'llm': [(r['q'], r['y']) for r in rows if r['q']],
             'fused': [(softmax(r['logits']), r['y']) for r in rows if len(r['logits']) == K],
             'calibrated': after}
    out['scores'] = {k: {m: reliability(v)[m] for m in ('n', 'ece', 'nll')} for k, v in
                     pairs.items()}
    return out


def calibrate(records: list[dict], labels: list[dict], eta: float = 1e-3,
              T_stale: float = 30.0, min_labels: int = 20) -> dict:
    """{'questions': {(qset, question): fit}, 'patch': catalog patch (question-level keys)}."""
    by = {}
    for f, t, y in labelled(records, labels):
        by.setdefault((f.get('question_set', ''), t['question']), []).append((f, t, y))
    fits, patch = {}, {}
    for (qs, qn), items in sorted(by.items()):
        rows = _rows(items, eta, T_stale)
        classes = items[0][1]['classes']
        if len(rows) < min_labels:
            fits[f'{qs}/{qn}'] = {'n': len(rows), 'skipped': f'fewer than {min_labels} labels'}
            continue
        fit = fits[f'{qs}/{qn}'] = fit_question(rows, classes, eta)
        entry = {k: fit[k] for k in ('beta_min', 'beta_max', 'gamma', 'B') if k in fit}
        entry['bias'] = fit['bias']
        patch.setdefault('question_sets', {}).setdefault(qs, {}).setdefault(
            'questions', {})[qn] = entry
    return {'questions': fits, 'patch': patch}


def merge(base: dict, patch: dict) -> dict:
    """Deep-merge a patch into a raw catalog dict (patch wins)."""
    out = dict(base)
    for k, v in patch.items():
        out[k] = merge(out[k], v) if isinstance(v, dict) and isinstance(out.get(k), dict) else v
    return out
