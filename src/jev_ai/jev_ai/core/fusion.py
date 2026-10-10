"""Fusion and selection (FRAMEWORK §6). Pure functions over plain lists."""
from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

from jev_ai.core.catalog import SELECT_ACTS, Policy, QuestionSpec

NEG_INF = float('-inf')
# ponytail: floor on Jev probabilities so log p_J stays finite. A Jev zero is then a very
# strong (not absolute) vote; only masks and require() are absolute (§6.2).
P_FLOOR = 1e-12


def confidence(p: list[float]) -> float:
    """§6.1: c = 1 - H(p)/log|A|, in [0, 1] for every question type."""
    s = sum(p)
    h = -sum(x / s * math.log(x / s) for x in p if x > 0)
    return min(1.0, max(0.0, 1.0 - h / math.log(len(p))))


def beta(c: float, policy: Policy) -> float:
    """§6.2: LLM weight at Jev confidence c.

    Default schedule from the spec: beta_min + (beta_max - beta_min) * (1 - c)^gamma.
    At c=1 the LLM still gets beta_min (high-confidence nondeterminism); at c=0 it gets beta_max.
    gamma > 1 concentrates augmentation on low-confidence decisions, gamma < 1 spreads it.
    """
    c = min(1.0, max(0.0, c))
    return policy.beta_min + (policy.beta_max - policy.beta_min) * (1.0 - c) ** policy.gamma


def llm_logpref(q: list[float], masked: list[bool], eta: float) -> list[float]:
    """§6.2 ℓ: centred log of the LLM weights, renormalised over unmasked classes."""
    unmasked = [i for i, m in enumerate(masked) if not m]
    s = sum(q[i] for i in unmasked)
    if s <= 0:
        return [0.0] * len(q)
    logs = {i: math.log(max(q[i] / s, eta)) for i in unmasked}
    mean = sum(logs.values()) / len(logs)
    return [logs[i] - mean if i in logs else 0.0 for i in range(len(q))]


def pool(p_jev: list[float], q_llm: list[float] | None, masked: list[bool],
         irreversible: list[bool], c: float, llm_age_s: float, policy: Policy,
         bias: list[float] | None = None) -> tuple[list[float], float]:
    """§6.2: fused logits z and the effective LLM weight beta_eff (0 when there is no q_L)."""
    n = len(p_jev)
    s = sum(p_jev)
    if q_llm:
        b = beta(c, policy) * math.exp(-llm_age_s / policy.T_stale)
        ell = llm_logpref(q_llm, masked, policy.eta)
    else:
        b, ell = 0.0, [0.0] * n
    z = []
    for i in range(n):
        if masked[i]:
            z.append(NEG_INF)
            continue
        d = min(policy.B, max(-policy.B, b * ell[i]))
        if irreversible[i]:
            d = min(d, 0.0)          # the LLM may only lower irreversible classes
        z.append(math.log(max(p_jev[i] / s, P_FLOOR)) + d + (bias[i] if bias else 0.0))
    return z, b


def softmax(x: list[float]) -> list[float]:
    m = max(x)
    if m == NEG_INF:
        return [1.0 / len(x)] * len(x)
    e = [math.exp(v - m) if v != NEG_INF else 0.0 for v in x]
    s = sum(e)
    return [v / s for v in e]


# --- Coupled noise (§6.3) -------------------------------------------------------------------

def unit(*parts) -> float:
    """H(parts) -> U in (0, 1): a pure, platform-independent function of logged keys."""
    h = hashlib.sha256('\x1f'.join(map(str, parts)).encode()).digest()
    # 52 bits so that (k + 0.5) / 2^52 is exact in a double and never rounds to 0 or 1.
    return ((int.from_bytes(h[:8], 'big') >> 12) + 0.5) / 2.0 ** 52


@dataclass(frozen=True)
class NoiseKey:
    """Everything the Gumbel noise of one decision depends on (all logged in DecisionRecord)."""

    run_id: str
    agent: str
    group: str                   # == agent -> independent noise
    epoch: str
    directive: bool = False      # answering a directive -> coupling_directive


def epoch_key(t_node: float, epoch_s: float, directive_issuer: str = '',
              directive_id: int = 0) -> tuple[str, bool]:
    """§6.3 epoch: the directive for directive responses, else floor(t_node / epoch_s)."""
    if directive_issuer:
        return f'd:{directive_issuer}:{directive_id}', True
    return f't:{math.floor(t_node / epoch_s)}', False


def gumbel_noise(question: str, classes: list[str], key: NoiseKey,
                 rho: float) -> tuple[list[float], bool]:
    """Per-class Gumbel noise; with probability rho the group's shared noise is used."""
    use_group = key.group != key.agent and unit('couple', key.run_id, key.agent, question,
                                                key.epoch) < rho
    k = key.group if use_group else key.agent
    g = [-math.log(-math.log(unit('gumbel', key.run_id, k, question, key.epoch, a)))
         for a in classes]
    return g, use_group


# --- Per-question decision ------------------------------------------------------------------

@dataclass
class Trace:
    """One question of one decision. Fields up to `irreversible_path` mirror QuestionTrace.msg."""

    question: str
    classes: list[str]
    p_jev: list[float] = field(default_factory=list)
    jev_confidence: float = float('nan')
    q_llm: list[float] = field(default_factory=list)
    llm_age_s: float = 0.0
    beta: float = 0.0
    masked: list[bool] = field(default_factory=list)
    required: str = ''
    logits: list[float] = field(default_factory=list)
    tau: float = 0.0
    kappa: float = 0.0
    noise: list[float] = field(default_factory=list)
    group_noise: bool = False
    selected: str = ''
    propensity: float = 1.0
    irreversible_path: bool = False
    # engine-side
    fused: list[float] = field(default_factory=list)   # softmax(z): Decision.fused
    values: dict = field(default_factory=dict)          # Decision.values
    path: str = ''   # forced|jev_error|irreversible|irreversible_pending|dwell|sample|argmax|
    #                  expectation|threshold|none   (degraded is set by Commitment)
    flags: list[str] = field(default_factory=list)


def decide_question(q: QuestionSpec, p_jev: list[float] | None, *, key: NoiseKey,
                    q_llm: list[float] | None = None, llm_age_s: float = 0.0,
                    masked=(), required: str = '', current: str = '',
                    irreversible: set[str] | None = None, augmented: bool = False,
                    may_switch: bool = True, bias: list[float] | None = None) -> Trace:
    """Fuse, then select per the question's act (§6.2-§6.4).

    p_jev None = Jev failed for this question: keep the current class (§11).
    augmented = augmentation for *this* decision has arrived (irreversible rule).
    may_switch = False inside T_dwell: keep the current class unless it became masked.
    """
    pol = q.policy
    names = q.class_names
    flags = []
    masked = set(masked)
    if q.fallback in masked:                       # §11: the fallback is always executable
        masked.discard(q.fallback)
        flags.append('fallback_mask_ignored')
    if required:
        if required not in names:
            raise ValueError(f'{q.name}: required class {required!r} is not a class')
        if required in masked:                     # ponytail: require() beats mask(); flagged
            masked.discard(required)
            flags.append('require_overrides_mask')
    irr = q.irreversible() if irreversible is None else set(irreversible)
    mvec = [a in masked for a in names]
    rho = pol.coupling_directive if key.directive else pol.coupling
    g, group_noise = gumbel_noise(q.name, names, key, rho)
    t = Trace(question=q.name, classes=names, masked=mvec, required=required, tau=pol.tau,
              kappa=pol.kappa, noise=g, group_noise=group_noise, flags=flags,
              llm_age_s=llm_age_s if q_llm else 0.0)

    if p_jev is None:
        if q.act in SELECT_ACTS:
            t.selected = current if current in names and current not in masked else q.fallback
        t.path = 'jev_error'      # report-type acts produce nothing without Jev
        return t
    if len(p_jev) != len(names):
        raise ValueError(f'{q.name}: {len(p_jev)} probabilities for {len(names)} classes')

    ivec = [a in irr for a in names]
    c = confidence(p_jev)
    z, b = pool(p_jev, q_llm, mvec, ivec, c, llm_age_s, pol, bias)
    t.p_jev, t.jev_confidence, t.q_llm, t.beta, t.logits = list(p_jev), c, list(q_llm or []), b, z
    t.fused = softmax(z)

    if required:
        t.selected, t.path = required, 'forced'
    elif q.act == 'none':
        t.path = 'none'
    elif q.act == 'expectation':
        _expectation(q, t)
    elif q.act == 'threshold':
        p_true = t.fused[names.index('true')]
        t.selected = 'true' if p_true >= q.threshold else 'false'
        t.values = {f'{q.name}.p_true': p_true}
        t.path = 'threshold'
    else:
        _select(q, t, z, mvec, ivec, current, augmented, may_switch)
    return t


def _select(q, t, z, mvec, ivec, current, augmented, may_switch):
    pol, names = q.policy, q.class_names
    ci = names.index(current) if current in names else None
    cur_ok = ci is not None and not mvec[ci] and not ivec[ci]   # current is still selectable

    # §6.4 irreversible rule: argmax, margin m_irrev, and augmentation; never sampled.
    a = max(range(len(z)), key=lambda i: z[i])
    if ivec[a]:
        second = max((z[i] for i in range(len(z)) if i != a), default=NEG_INF)
        if z[a] - second >= pol.m_irrev:
            if augmented:
                t.selected, t.path, t.irreversible_path = names[a], 'irreversible', True
            else:   # hold the current class and wait for augmentation (§5.3)
                t.selected = current if cur_ok else q.fallback
                t.path = 'irreversible_pending'
            return

    if not may_switch and cur_ok:
        t.selected, t.path = current, 'dwell'
        return

    # §6.3 Gumbel-max over reversible, unmasked classes (the fallback is always among them).
    elig = [i for i in range(len(z)) if not mvec[i] and not ivec[i]]
    base = [(z[i] + (pol.kappa if names[i] == current else 0.0)) / pol.tau for i in elig]
    if q.act == 'sample':
        k = max(range(len(elig)), key=lambda j: base[j] + t.noise[elig[j]])
        t.propensity = softmax(base)[k]
    else:
        k = max(range(len(elig)), key=lambda j: base[j])
    t.selected, t.path = names[elig[k]], q.act


def _expectation(q, t):
    """§6.4 expectation for score questions: normalised mean level and median class."""
    pi = t.fused
    levels = len(pi) - 1
    mean = sum(i * p for i, p in enumerate(pi)) / levels
    cdf, median = 0.0, q.class_names[-1]
    for name, p in zip(q.class_names, pi):
        cdf += p
        if cdf >= 0.5:
            median = name
            break
    t.selected = median
    t.values = {f'{q.name}.level_mean': mean, f'{q.name}.level_median': median}
    t.path = 'expectation'
