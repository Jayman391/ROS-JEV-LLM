"""Catalog and policy: load, validate, hash (FRAMEWORK §4, §14)."""
from __future__ import annotations

import dataclasses
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import yaml

# Same values as jev_ai_msgs/Question constants.
NOUL, CHOICE, SCORE = 0, 1, 2
TYPES = {'noul': NOUL, 'choice': CHOICE, 'score': SCORE}
CLASS_COUNT = {NOUL: (2, 2), CHOICE: (2, 255), SCORE: (2, 10)}
SELECT_ACTS = {'sample', 'argmax'}
ACTS = {
    NOUL: SELECT_ACTS | {'threshold', 'none'},
    CHOICE: SELECT_ACTS | {'none'},
    SCORE: SELECT_ACTS | {'expectation', 'none'},
}
DEFAULT_ACT = {NOUL: 'threshold', CHOICE: 'sample', SCORE: 'sample'}
# Policy fields a single question may override in the catalog.
# beta_min..eta come from `ros2 jev calibrate`, which fits them per question (§12).
# skip_llm_above_c per question: only the questions that set it gate augmentation (§5.2 step 6).
QUESTION_OVERRIDES = {'tau', 'kappa', 'coupling', 'coupling_directive', 'argmax_margin', 'm_irrev',
                      'beta_min', 'beta_max', 'gamma', 'B', 'eta', 'skip_llm_above_c'}


class CatalogError(ValueError):
    """The catalog or policy is invalid; the engine refuses to start."""


@dataclass(frozen=True)
class Policy:
    """Fusion/selection/timing parameters (§14). Catalog `policy:` and ROS params override."""

    beta_min: float = 0.1
    beta_max: float = 1.0
    gamma: float = 2.0                   # A2 decision: LLM weight concentrates on uncertain Jev
    B: float = math.log(20)              # LLM influence bound, nats
    eta: float = 1e-3                    # log floor for LLM weights
    tau: float = 0.5
    kappa: float = 0.7                   # status-quo bonus, nats
    coupling: float = 0.5                # rho for autonomous decisions
    coupling_directive: float = 1.0      # rho when answering a directive
    epoch_s: float = 5.0
    m_irrev: float = math.log(2)
    argmax_margin: float = 0.1
    T_dwell: float = 5.0
    T_stale: float = 30.0
    T_wait: float | None = None          # None = llm_timeout + 0.5
    T_refresh: float = 20.0
    k_max: float = 6.0                   # augmentation requests per minute per engine
    c_low: float = 0.5
    skip_llm_above_c: float | None = None  # None = always augment
    reuse_stale_llm: bool = False
    jev_timeout: float = 1.0
    llm_timeout: float = 6.0
    N_fail: int = 3

    def __post_init__(self):
        if self.T_wait is None:
            object.__setattr__(self, 'T_wait', self.llm_timeout + 0.5)
        self._validate()

    _RANGES = {
        'beta_min': (0.0, 0.5), 'beta_max': (0.5, 3.0), 'gamma': (0.5, 3.0),
        'B': (math.log(2) - 1e-9, math.log(100) + 1e-9), 'eta': (1e-9, 0.5),
        'tau': (0.05, 1.5), 'kappa': (0.0, 3.0),
        'coupling': (0.0, 1.0), 'coupling_directive': (0.0, 1.0),
        'c_low': (0.0, 1.0), 'argmax_margin': (0.0, 1.0),
    }

    def _validate(self):
        for name, (lo, hi) in self._RANGES.items():
            v = getattr(self, name)
            if not (lo <= v <= hi):
                raise CatalogError(f'policy.{name}={v} outside [{lo:g}, {hi:g}]')
        for name in ('epoch_s', 'T_dwell', 'T_stale', 'T_refresh', 'k_max', 'jev_timeout',
                     'llm_timeout', 'N_fail'):
            if getattr(self, name) <= 0:
                raise CatalogError(f'policy.{name} must be > 0')
        if self.m_irrev < 0:
            raise CatalogError('policy.m_irrev must be >= 0')
        if self.T_wait < self.llm_timeout + 0.5:
            # §11: an LLM timeout must resolve before T_wait, so the timeout path is FALLBACK.
            raise CatalogError(
                f'policy.T_wait={self.T_wait} < llm_timeout + 0.5 = {self.llm_timeout + 0.5}')

    @classmethod
    def from_dict(cls, d: dict | None) -> Policy:
        d = dict(d or {})
        unknown = set(d) - {f.name for f in dataclasses.fields(cls)}
        if unknown:
            raise CatalogError(f'unknown policy keys: {sorted(unknown)}')
        return cls(**d)

    def merged(self, overrides: dict) -> Policy:
        return dataclasses.replace(self, **overrides) if overrides else self

    def hash(self) -> str:
        return _hash(dataclasses.asdict(self))


@dataclass(frozen=True)
class ClassSpec:
    name: str
    criteria: str = ''
    reversible: bool | str = True        # True | False | 'until_commit'


@dataclass(frozen=True)
class QuestionSpec:
    name: str
    type: int
    instructions: str
    act: str
    classes: tuple[ClassSpec, ...]
    fallback: str
    policy: Policy                       # catalog policy + this question's overrides
    threshold: float = 0.5
    bias: tuple[float, ...] = ()         # b_q per class, nats (§6.2; fitted by calibrate)

    @property
    def class_names(self) -> list[str]:
        return [c.name for c in self.classes]

    @property
    def criteria(self) -> list[str]:
        return [c.criteria for c in self.classes]

    def irreversible(self, after_commit_point: bool = False) -> set[str]:
        """Classes that may not be sampled (§6.4). `until_commit` counts after commit_point()."""
        return {c.name for c in self.classes
                if c.reversible is False or (after_commit_point and c.reversible == 'until_commit')}


@dataclass(frozen=True)
class QuestionSet:
    name: str
    state_keys: tuple[str, ...]
    questions: dict[str, QuestionSpec]
    policy: Policy
    tick_hz: float = 0.0
    augment: bool = True
    group: str | None = None             # None = the agent itself (no coupling)
    llm_params: dict[str, tuple[float, float, float]] = dataclasses.field(default_factory=dict)
    llm_system: str = ''

    def ordered_state(self, state: dict) -> list[tuple[str, str]]:
        """Validate an encoder's output against state_keys and return it in catalog order."""
        if set(state) != set(self.state_keys):
            missing = sorted(set(self.state_keys) - set(state))
            extra = sorted(set(state) - set(self.state_keys))
            raise CatalogError(f'{self.name}: state keys mismatch (missing {missing}, extra {extra})')
        bad = [k for k, v in state.items() if not isinstance(v, str)]
        if bad:
            raise CatalogError(f'{self.name}: state values must be strings: {bad}')
        return [(k, state[k]) for k in self.state_keys]


@dataclass(frozen=True)
class Catalog:
    name: str
    question_sets: dict[str, QuestionSet]
    policy: Policy
    catalog_hash: str
    policy_hash: str


def load_catalog(src: str | Path | dict, policy_overrides: dict | None = None) -> Catalog:
    """Load and validate a catalog from a YAML path or a dict. Raises CatalogError."""
    raw = src if isinstance(src, dict) else yaml.safe_load(Path(src).read_text())
    if not isinstance(raw, dict):
        raise CatalogError('catalog must be a mapping')
    _keys(raw, 'catalog', required={'catalog_version', 'name', 'question_sets'}, optional={'policy'})
    if raw['catalog_version'] != 1:
        raise CatalogError(f"unsupported catalog_version {raw['catalog_version']}")
    policy = Policy.from_dict({**(raw.get('policy') or {}), **(policy_overrides or {})})
    sets = raw['question_sets']
    if not isinstance(sets, dict) or not sets:
        raise CatalogError('question_sets must be a non-empty mapping')
    question_sets = {name: _question_set(name, body, policy) for name, body in sets.items()}
    content = {k: v for k, v in raw.items() if k != 'policy'}
    return Catalog(name=str(raw['name']), question_sets=question_sets, policy=policy,
                   catalog_hash=_hash(content), policy_hash=policy.hash())


def _question_set(name: str, body: dict, policy: Policy) -> QuestionSet:
    where = f'question_sets.{name}'
    _keys(body, where, required={'state_keys', 'questions'},
          optional={'tick_hz', 'augment', 'group', 'llm_params', 'llm'})
    keys = body['state_keys']
    if not isinstance(keys, list) or not keys or len(set(keys)) != len(keys):
        raise CatalogError(f'{where}.state_keys must be a non-empty list of unique names')
    qs = body['questions']
    if not isinstance(qs, dict) or not qs:
        raise CatalogError(f'{where}.questions must be a non-empty mapping')
    questions = {qn: _question(f'{where}.questions.{qn}', qn, qb, policy) for qn, qb in qs.items()}
    augment = bool(body.get('augment', True))
    if not augment and any(q.irreversible(True) for q in questions.values()):
        raise CatalogError(f'{where}: irreversible classes need augment: true '
                           '(use require() for deterministic overrides)')
    tick_hz = float(body.get('tick_hz', 0.0))
    if tick_hz < 0:
        raise CatalogError(f'{where}.tick_hz must be >= 0')
    llm_params = {}
    for pn, pb in (body.get('llm_params') or {}).items():
        try:
            lo, hi, d = float(pb['min']), float(pb['max']), float(pb['default'])
        except (KeyError, TypeError, ValueError):
            raise CatalogError(f'{where}.llm_params.{pn} needs numeric min, max, default') from None
        if not lo <= d <= hi:
            raise CatalogError(f'{where}.llm_params.{pn}: need min <= default <= max')
        llm_params[pn] = (lo, hi, d)
    return QuestionSet(name=name, state_keys=tuple(keys), questions=questions, policy=policy,
                       tick_hz=tick_hz, augment=augment, group=body.get('group'),
                       llm_params=llm_params, llm_system=(body.get('llm') or {}).get('system', ''))


def _question(where: str, name: str, body: dict, policy: Policy) -> QuestionSpec:
    _keys(body, where, required={'type', 'instructions'},
          optional={'act', 'classes', 'criteria', 'fallback', 'threshold', 'bias'}
          | QUESTION_OVERRIDES)
    qtype = TYPES.get(body['type'])
    if qtype is None:
        raise CatalogError(f'{where}.type must be one of {sorted(TYPES)}')
    if not str(body['instructions']).strip():
        raise CatalogError(f'{where}.instructions is empty')

    if qtype == NOUL and 'classes' not in body:
        crit = body.get('criteria') or {}
        classes = (ClassSpec('true', str(crit.get('true', crit.get(True, '')))),
                   ClassSpec('false', str(crit.get('false', crit.get(False, '')))))
    else:
        classes = tuple(_class(f'{where}.classes', c) for c in body.get('classes') or [])
    names = [c.name for c in classes]
    lo, hi = CLASS_COUNT[qtype]
    if not lo <= len(names) <= hi:
        raise CatalogError(f'{where}: {body["type"]} needs {lo}-{hi} classes, got {len(names)}')
    if len(set(names)) != len(names):
        raise CatalogError(f'{where}: duplicate class names')
    if qtype == NOUL and names != ['true', 'false']:
        raise CatalogError(f'{where}: noul classes must be [true, false]')

    act = body.get('act', DEFAULT_ACT[qtype])
    if act not in ACTS[qtype]:
        raise CatalogError(f'{where}.act={act} not allowed for {body["type"]} '
                           f'(allowed: {sorted(ACTS[qtype])})')
    has_irrev = any(c.reversible is not True for c in classes)
    if has_irrev and act not in SELECT_ACTS:
        raise CatalogError(f'{where}: irreversible classes need act sample|argmax')

    # Default: the status quo (first class); for yes/no, "don't" is the safe default.
    fallback = body.get('fallback', 'false' if qtype == NOUL else names[0])
    spec = {c.name: c for c in classes}
    if fallback not in spec:
        raise CatalogError(f'{where}.fallback={fallback} is not a class')
    if spec[fallback].reversible is not True:
        raise CatalogError(f'{where}.fallback={fallback} must be reversible: true')

    threshold = float(body.get('threshold', 0.5))
    if not 0.0 <= threshold <= 1.0:
        raise CatalogError(f'{where}.threshold must be in [0, 1]')
    try:
        qpolicy = policy.merged({k: body[k] for k in QUESTION_OVERRIDES if k in body})
    except CatalogError as e:
        raise CatalogError(f'{where}: {e}') from None
    bias = body.get('bias') or {}
    if not isinstance(bias, dict) or set(bias) - set(names) or not all(
            isinstance(v, (int, float)) and abs(v) <= qpolicy.B for v in bias.values()):
        raise CatalogError(f'{where}.bias must map class names to nats within ±B')
    return QuestionSpec(name=name, type=qtype, instructions=str(body['instructions']), act=act,
                        classes=classes, fallback=fallback, policy=qpolicy, threshold=threshold,
                        bias=tuple(float(bias.get(n, 0.0)) for n in names) if bias else ())


def _class(where: str, c) -> ClassSpec:
    if isinstance(c, str):
        return ClassSpec(c)
    if not isinstance(c, dict) or 'name' not in c:
        raise CatalogError(f'{where}: each class is a name or {{name, criteria, reversible}}')
    _keys(c, f'{where}.{c["name"]}', required={'name'}, optional={'criteria', 'reversible'})
    rev = c.get('reversible', True)
    if rev not in (True, False, 'until_commit'):
        raise CatalogError(f'{where}.{c["name"]}.reversible must be true, false or until_commit')
    if not str(c['name']).strip():
        raise CatalogError(f'{where}: empty class name')
    return ClassSpec(str(c['name']), str(c.get('criteria', '')), rev)


def _keys(d, where: str, required: set, optional: set):
    if not isinstance(d, dict):
        raise CatalogError(f'{where} must be a mapping')
    missing = required - set(d)
    if missing:
        raise CatalogError(f'{where}: missing {sorted(missing)}')
    unknown = set(d) - required - optional
    if unknown:
        raise CatalogError(f'{where}: unknown keys {sorted(unknown)}')


def _hash(obj) -> str:
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()[:16]
