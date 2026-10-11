"""``EpisodeRunner`` for ``ros2 jev search`` (A10): one toy_patrol robot in simulated time, no ROS.

The real ``Commitment`` machine and §5.3 decision-event rule drive ``ToyWorld``; Jev is the mock
rules perturbed per call (seeded), the LLM a mostly-right oracle with lognormal latency (answers
slower than ``llm_timeout`` fail). A 10-minute episode runs in well under a second.

Score = share of time the executed action matches the oracle
        − ``cost_per_call`` × LLM calls/min − ``switch_cost`` × action switches/min.

    ros2 jev search --runner jev_ai_examples.toy_episode:ToyEpisode \\
        --space T_dwell=1,5,10 tick_hz=0.5,1,2 k_max=3,6,12
"""
from __future__ import annotations

import math
import random
from types import SimpleNamespace

import yaml
from ament_index_python.packages import get_package_share_directory

from jev_ai.core.catalog import load_catalog
from jev_ai.core.commitment import Commitment, TokenBucket, decision_due
from jev_ai.core.ope import overrides
from jev_ai_examples.mock_rules import patrol
from jev_ai_examples.toy_world import STEP_S, ToyWorld


class ToyEpisode:
    def __init__(self, catalog: str = '', duration_s: float = 600.0, cost_per_call: float = 0.01,
                 switch_cost: float = 0.01, llm_accuracy: float = 0.8,
                 llm_latency_s: float = 1.5, jev_concentration: float = 50.0):
        path = catalog or get_package_share_directory('jev_ai_examples') + \
            '/config/toy_patrol.yaml'
        with open(path) as f:
            self.raw = yaml.safe_load(f)
        self.duration_s, self.cost_per_call = float(duration_s), float(cost_per_call)
        self.switch_cost, self.llm_accuracy = float(switch_cost), float(llm_accuracy)
        self.llm_latency_s, self.conc = float(llm_latency_s), float(jev_concentration)

    def run(self, params: dict, seed: int) -> float:
        params = dict(params)
        raw = overrides(self.raw, {k: v for k, v in params.items() if k != 'tick_hz'})
        if 'tick_hz' in params:
            raw['question_sets']['next_step']['tick_hz'] = float(params['tick_hz'])
        qset = load_catalog(raw).question_sets['next_step']
        pol = qset.policy
        # common random numbers: the world, Jev and the LLM draw from their own seeded streams
        world = ToyWorld(random.Random(f'world:{seed}'))
        jrng, lrng = random.Random(f'jev:{seed}'), random.Random(f'llm:{seed}')
        m = Commitment(qset, agent='toy', run_id=f'episode:{seed}')
        bucket = TokenBucket(pol.k_max)
        triggers, last_p, llm = ['start'], {}, None
        next_tick, last_refresh = 0.0, 0.0
        correct = calls = switches = 0
        steps = int(self.duration_s / STEP_S)
        doing = world.executing()

        def apply(now, emitted):
            for st in emitted:
                if st.act:
                    world.act(st.selected.get('next_action'), now)

        for k in range(steps):
            now = k * STEP_S
            for kind, *args in world.step(now):
                if kind == 'trigger':
                    triggers.append(f'app:{args[0]}')
                else:
                    getattr(m, kind)(args[0])
                    if kind == 'completed':
                        triggers.append('completed')
            masked, required = world.mask(), world.require()
            if llm and now >= llm.at:
                apply(now, m.on_llm(now, generation=llm.gen, q_llm=llm.q, masked=masked,
                                    required=required) if llm.q else
                      m.on_llm_failed(now, llm.gen))
                llm = None
            apply(now, m.poll(now))
            if now - last_refresh >= pol.T_refresh:
                last_refresh = now
                triggers.append('periodic')
            if (qset.tick_hz > 0 and now >= next_tick) or triggers:
                next_tick = now + (1.0 / qset.tick_hz if qset.tick_hz > 0 else math.inf)
                state = world.encode(now)
                p = {qn: self._jev(jrng, state, q) for qn, q in qset.questions.items()}
                if decision_due(qset, triggers, p, last_p, m.current, masked, required,
                                pending=m.open is not None):
                    step = m.on_event(now, state=state, p_jev=p, masked=masked, required=required,
                                      triggers=triggers)
                    triggers, llm = [], None
                    last_p.update(p)
                    apply(now, [step])
                    if step.request_llm:
                        if bucket.take(now, step.request_llm):
                            calls += 1
                            llm = self._llm(lrng, now, step.generation, qset, world, p, pol)
                        else:
                            apply(now, m.on_llm_failed(now, step.generation))
            now_doing = world.executing()
            switches += now_doing != doing
            doing = now_doing
            correct += doing == world.oracle()['next_action']
        minutes = self.duration_s / 60.0
        return correct / steps - self.cost_per_call * calls / minutes - \
            self.switch_cost * switches / minutes

    def _jev(self, rng, state, q):
        """Mock rules around which each call is a Dirichlet draw (Jev is not deterministic)."""
        mean = patrol(state, SimpleNamespace(name=q.name)) or {}
        g = [rng.gammavariate(max(1e-3, mean.get(c, 1e-3) * self.conc), 1.0)
             for c in q.class_names]
        return [x / sum(g) for x in g]

    def _llm(self, rng, now, generation, qset, world, p, pol):
        """A mostly-right LLM: arrives after a lognormal latency, or fails at llm_timeout."""
        latency = rng.lognormvariate(math.log(self.llm_latency_s), 0.5)
        truth = world.oracle()
        q = {}
        for qn, spec in qset.questions.items():
            names = spec.class_names
            if qn not in truth:
                q[qn] = p[qn]
                continue
            target = truth[qn] if rng.random() < self.llm_accuracy else rng.choice(names)
            q[qn] = [0.85 if c == target else 0.15 / (len(names) - 1) for c in names]
        if latency > pol.llm_timeout:
            return SimpleNamespace(at=now + pol.llm_timeout, gen=generation, q=None)
        return SimpleNamespace(at=now + latency, gen=generation, q=q)
