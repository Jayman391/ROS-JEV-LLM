"""Deterministic synthetic DecisionRecords for tests and ``--demo`` (FRAMEWORK §3, §5.3, §12).

A small patrol fleet runs the real ``jev_ai.core.Commitment`` machine over simulated time: a
shared hidden scenario (calm / detection / hazard) drives Jev's distributions (with its
first-option bias), a mostly-right LLM, masks, a few ``require()`` overrides, LLM timeouts and
errors, and the odd Jev failure. Every Step becomes a record via ``records.record_dict``; the
hidden scenario becomes ground-truth labels.

    python3 -m jev_ai.viz.sample OUT_DIR [--end-now]   # writes decisions.jsonl + labels.jsonl
"""
from __future__ import annotations

import argparse
import json
import math
import random
import time
from pathlib import Path

from jev_ai.core.catalog import load_catalog
from jev_ai.core.commitment import Commitment
from jev_ai.core.records import dumps, record_dict

T_WALL0 = 1_760_000_000.0         # fixed, so output is byte-for-byte reproducible

# The FRAMEWORK §4 patrol catalog, plus a non-augmented `route` set.
CATALOG = {
    'catalog_version': 1,
    'name': 'patrol',
    'question_sets': {
        'next_step': {
            'state_keys': ['task', 'energy', 'link', 'hazards', 'detection'],
            'tick_hz': 0.1,
            'augment': True,
            'group': 'fleet',
            'questions': {
                'next_action': {
                    'type': 'choice',
                    'instructions': "Choose the robot's next action.",
                    'act': 'sample',
                    'classes': [
                        {'name': 'continue', 'criteria': 'Keep doing the assigned task.'},
                        {'name': 'inspect', 'criteria': 'Look closer at the detection.',
                         'reversible': 'until_commit'},
                        {'name': 'halt_now', 'criteria': 'Emergency stop; critical failure only.',
                         'reversible': False},
                    ],
                    'fallback': 'continue',
                    'coupling': 0.5,
                },
                'urgency': {
                    'type': 'score',
                    'instructions': 'How urgent is resolving the detection?',
                    'classes': ['none', 'low', 'high'],
                    'act': 'expectation',
                },
                'report_now': {
                    'type': 'noul',
                    'instructions': 'Should the robot report now?',
                    'criteria': {'true': 'Something changed worth reporting.',
                                 'false': 'Nothing new.'},
                },
            },
            'llm_params': {'inspect_altitude_m': {'min': 8, 'max': 15, 'default': 10}},
            'llm': {'system': 'You are the onboard reasoning module of {agent}.'},
        },
        'route': {
            'state_keys': ['task', 'energy', 'link'],
            'tick_hz': 1 / 30,
            'augment': False,
            'group': 'fleet',
            'questions': {
                'route': {
                    'type': 'choice',
                    'instructions': 'Which route should the robot take?',
                    'classes': ['primary', 'alternate', 'return_home'],
                },
            },
        },
    },
    'policy': {},
}

# One hidden scenario per minute, shared by the fleet (scripted so every demo shows each case).
STORY = [{'c': 'calm', 'd': 'detection', 'h': 'hazard'}[x] for x in 'ccdcdhccdcchdcc']
TRUTH = {
    'next_action': {'calm': 'continue', 'detection': 'inspect', 'hazard': 'halt_now'},
    'report_now': {'calm': 'false', 'detection': 'true', 'hazard': 'true'},
    'urgency': {'calm': 'none', 'detection': 'low', 'hazard': 'high'},
    'route': {'calm': 'primary', 'detection': 'alternate', 'hazard': 'return_home'},
}
# Jev's mean answer per scenario; class 0 is inflated (its known first-option bias).
JEV_MEAN = {
    'next_action': {'calm': [.78, .17, .05], 'detection': [.45, .45, .10],
                    'hazard': [.15, .10, .75]},
    'urgency': {'calm': [.70, .25, .05], 'detection': [.25, .50, .25], 'hazard': [.05, .25, .70]},
    'report_now': {'calm': [.25, .75], 'detection': [.60, .40], 'hazard': [.85, .15]},
    'route': {'calm': [.80, .15, .05], 'detection': [.45, .45, .10], 'hazard': [.30, .20, .50]},
}
RATIONALE = {
    'calm': 'Nothing notable in view and energy is ample; keep patrolling.',
    'detection': 'A detection is reported nearby; a closer look is cheap and reversible.',
    'hazard': 'Hazard confirmed by two sensors; stopping is the safe choice.',
}


def _dirichlet(rng: random.Random, mean: list[float], conc: float) -> list[float]:
    g = [rng.gammavariate(max(m * conc, 1e-3), 1.0) for m in mean]
    s = sum(g)
    return [x / s for x in g]


def _llm_weights(rng, classes, truth, accuracy=0.8):
    """Mostly-right LLM: concentrates on the true class, sometimes on a random one."""
    target = truth if rng.random() < accuracy else rng.choice(classes)
    mean = [0.85 if c == target else 0.15 / (len(classes) - 1) for c in classes]
    return _dirichlet(rng, mean, 6.0)


def generate(seed: int = 7, agents: int = 4, duration_s: float = 900.0, run_id: str = 'demo',
             t_wall0: float = T_WALL0) -> tuple[list[dict], list[dict]]:
    """Return (records, labels), records sorted by wall time."""
    rng = random.Random(seed)
    cat = load_catalog(CATALOG)
    names = [f'drone_{i + 1}' for i in range(agents)]
    machines = {(a, qs): Commitment(cat.question_sets[qs], agent=a, run_id=run_id)
                for a in names for qs in cat.question_sets}
    records, labels = [], []

    def emit(step, a, qs, t, state, jev=None, llm=None):
        records.append(record_dict(
            step, question_set=qs, agent=a, run_id=run_id, group='fleet', subject=a,
            state=list(state.items()), t_node=t, t_wall=t_wall0 + t + 0.05,
            catalog_hash=cat.catalog_hash, policy_hash=cat.policy_hash, jev=jev, llm=llm))

    for k in range(int(duration_s // 10)):
        s = STORY[(k // 6) % len(STORY)]
        # the fleet sees the same scene: a shared Jev answer, perturbed per agent
        scene = {qn: _dirichlet(rng, JEV_MEAN[qn][s], 10) for qn in JEV_MEAN}
        for a in names:
            t = k * 10.0 + rng.uniform(0.0, 0.8)    # same 5 s coupling epoch across the fleet
            link = 'poor' if rng.random() < 0.1 else 'good'
            state = {'task': 'patrolling', 'energy': f'ok ({90 - int(t / duration_s * 60)}%)',
                     'link': link, 'hazards': 'confirmed' if s == 'hazard' else 'none',
                     'detection': 'person-sized object' if s != 'calm' else 'none'}
            jev = {'model': 'jev-1.13.0', 'latency_ms': rng.lognormvariate(math.log(110), 0.35),
                   'input_tokens': rng.randint(300, 420)}
            for qs in ('next_step', 'route') if k % 3 == 0 else ('next_step',):
                m = machines[a, qs]
                qset = cat.question_sets[qs]
                if qs == 'route':
                    rstate = {key: state[key] for key in qset.state_keys}
                    p = {'route': _dirichlet(rng, scene['route'], 60)}
                    step = m.on_event(t, state=rstate, p_jev=p, triggers=['periodic'])
                    emit(step, a, qs, t, rstate, jev)
                    labels.append(_label(run_id, a, step.decision_id, 'route', s))
                    continue
                p = {} if rng.random() < 0.01 else \
                    {qn: _dirichlet(rng, scene[qn], 60) for qn in qset.questions}
                masked = {'next_action': {'inspect'}} if link == 'poor' else {}
                required = {'next_action': 'halt_now'} if s == 'hazard' and rng.random() < .15 \
                    else {}
                step = m.on_event(t, state=state, p_jev=p, masked=masked, required=required,
                                  triggers=['periodic'] if k % 2 else ['argmax_change'])
                emit(step, a, qs, t, state, jev)
                for qn in ('next_action', 'report_now'):
                    labels.append(_label(run_id, a, step.decision_id, qn, s))
                if not step.request_llm:
                    continue
                outcome = rng.random()
                if outcome < 0.85:                     # answered
                    lat = min(6.0, rng.lognormvariate(math.log(1.6), 0.45))
                    q = {qn: _llm_weights(rng, qset.questions[qn].class_names, TRUTH[qn][s])
                         for qn in qset.questions}
                    tin, tout = rng.randint(1500, 2300), rng.randint(80, 180)
                    flags = (['data_suspect'] if link == 'poor' else []) + \
                        (['novel_situation'] if s == 'hazard' and rng.random() < .4 else [])
                    llm = {'model': 'claude-haiku-5-5', 'latency_ms': lat * 1000,
                           'input_tokens': tin, 'output_tokens': tout,
                           'cost_usd': tin * 1e-6 + tout * 5e-6,
                           'params': {'inspect_altitude_m': rng.randint(8, 15)},
                           'rationale': RATIONALE[s], 'flags': flags}
                    for st in m.on_llm(t + lat, generation=step.generation, q_llm=q,
                                       masked=masked, required=required):
                        emit(st, a, qs, t + lat, state, jev, llm)
                elif outcome < 0.95:                   # timeout: resolved at T_wait
                    for st in m.poll(t + qset.policy.T_wait):
                        emit(st, a, qs, t + qset.policy.T_wait, state, jev)
                else:                                  # error / open breaker
                    for st in m.on_llm_failed(t + 0.3, step.generation):
                        emit(st, a, qs, t + 0.3, state, jev)
    records.sort(key=lambda r: (r['wall_stamp']['sec'], r['wall_stamp']['nanosec']))
    return records, labels


def _label(run_id, subject, decision_id, question, scenario):
    return {'run_id': run_id, 'subject': subject, 'question': question,
            'true_class': TRUTH[question][scenario], 'decision_id': decision_id,
            'source': 'sim'}


def diagnostics(t_wall: float = T_WALL0 + 900) -> dict:
    """Server health as the dashboard keeps it: latest DiagnosticStatus per name (§8, §9)."""
    def status(name, level, msg, backend, model, p50, p95, usd, breaker='closed'):
        return {'name': name, 'level': level, 'message': msg, 'hardware_id': backend,
                'stamp': t_wall, 'values': {
                    'backend': backend, 'model': model, 'breaker': breaker,
                    'latency_ms_p50': str(p50), 'latency_ms_p95': str(p95),
                    'usd_last_hour': str(usd), 'max_usd_per_hour': '5.0'}}
    return {'/jev_server': status('/jev_server', 0, 'ok', 'typesafe', 'jev-1.13.0', 108, 171, 0),
            '/llm_server': status('/llm_server', 1, 'circuit breaker half_open', 'anthropic',
                                  'claude-haiku-5-5', 1620, 3480, 0.61, 'half_open')}


def main(argv=None):
    ap = argparse.ArgumentParser(description='Write synthetic decisions.jsonl + labels.jsonl.')
    ap.add_argument('out_dir')
    ap.add_argument('--end-now', action='store_true',
                    help='stamp the run so it ends now (for a live dashboard demo)')
    a = ap.parse_args(argv)
    out = Path(a.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    records, labels = generate(t_wall0=time.time() - 900.0 if a.end_now else T_WALL0)
    (out / 'decisions.jsonl').write_text(''.join(dumps(r) + '\n' for r in records))
    (out / 'labels.jsonl').write_text(''.join(json.dumps(x) + '\n' for x in labels))
    print(f'{len(records)} records, {len(labels)} labels -> {out}')


if __name__ == '__main__':
    main()
