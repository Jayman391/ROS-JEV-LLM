"""DecisionEngine end to end against in-process fake servers (§5, §11). Needs rclpy and built
jev_ai_msgs; skipped in the pure-core run."""
import copy
import json
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('jev_ai_msgs')

from jev_ai_msgs.action import Reason  # noqa: E402
from jev_ai_msgs.msg import DecisionRecord, Distribution  # noqa: E402
from jev_ai_msgs.srv import JevDecide  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

from jev_ai import DecisionEngine  # noqa: E402
from jev_ai.core.fusion import confidence  # noqa: E402

from conftest import PATROL, STATE  # noqa: E402

P = {'next_action': [0.5, 0.45, 0.05], 'urgency': [0.6, 0.3, 0.1], 'report_now': [0.3, 0.7]}


class World:
    """Fake jev_server + llm_server, the engine's node, and a record collector."""

    def __init__(self, llm=True, jev=True, llm_weights=None, mask=None, catalog=None):
        rclpy.init()
        self.server = rclpy.create_node('fake_servers')
        self.agent = rclpy.create_node('agent')
        self.p = dict(P)
        self.llm_weights = llm_weights or {'next_action': {'continue': 0.9, 'inspect': 0.1,
                                                           'halt_now': 0.0}}
        if jev:
            self.server.create_service(JevDecide, 'jev/decide', self._jev)
        if llm:
            self.action = ActionServer(self.server, Reason, 'llm/reason', self._llm)
        self.records, self.decisions = [], []
        self.agent.create_subscription(DecisionRecord, 'decisions', self.records.append, 100)
        self.engine = DecisionEngine(
            self.agent, catalog or copy.deepcopy(PATROL), encode_state=lambda _: dict(STATE),
            mask=lambda _: mask or {}, on_decision=self.decisions.append, run_id='test')
        self.ex = MultiThreadedExecutor(num_threads=4)
        self.ex.add_node(self.server)
        self.ex.add_node(self.agent)
        self.thread = threading.Thread(target=self.ex.spin, daemon=True)
        self.thread.start()

    def _jev(self, req, res):
        res.success, res.model = True, 'fake-jev'
        for q in req.questions:
            p = self.p[q.name]
            res.answers.append(Distribution(question=q.name, type=q.type, classes=q.classes,
                                            probabilities=p, confidence=confidence(p)))
            res.question_errors.append('')
        return res

    def _llm(self, goal_handle):
        goal_handle.succeed()
        weights = dict(self.llm_weights)
        return Reason.Result(success=True, model='fake-llm', json=json.dumps(
            {'weights': weights, 'params': {'inspect_altitude_m': 99}, 'rationale': 'ok',
             'flags': []}))

    def wait(self, pred, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(0.05)
        return False

    def phases(self):
        return [r.phase for r in self.records]

    def close(self):
        self.ex.shutdown()
        self.server.destroy_node()
        self.agent.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def world(request):
    w = World(**getattr(request, 'param', {}))
    yield w
    w.close()


def test_act_then_revise(world):
    assert world.wait(lambda: any(p in ('KEPT', 'COMMITTED') for p in world.phases()))
    first = world.records[0]
    assert first.phase == 'PROVISIONAL' and first.jev_model == 'fake-jev'
    final = next(r for r in world.records if r.phase in ('KEPT', 'COMMITTED'))
    q = {t.question: t for t in final.questions}['next_action']
    assert list(q.q_llm) == pytest.approx([0.9, 0.1, 0.0]) and q.beta > 0
    assert final.llm_model == 'fake-llm' and json.loads(final.llm_params) == \
        {'inspect_altitude_m': 15.0}                                     # clamped (§7)
    assert world.decisions[0].phase == 'PROVISIONAL'                    # acted immediately
    assert final.run_id == 'test' and final.catalog_hash


@pytest.mark.parametrize('world', [{'llm': False}], indirect=True)
def test_llm_down_means_jev_only(world):
    assert world.wait(lambda: 'COMMITTED' in world.phases())
    r = next(r for r in world.records if r.phase == 'COMMITTED')
    assert all(len(t.q_llm) == 0 and t.beta == 0.0 for t in r.questions)   # delta = 0 (§11)


@pytest.mark.parametrize('world', [{'jev': False}], indirect=True)
def test_jev_down_degrades_to_fallback(world):
    world.engine.trigger('poke')
    for i in range(3):
        world.engine.trigger(f'poke{i}')
        time.sleep(0.15)
    assert world.wait(lambda: 'FALLBACK' in world.phases())
    assert world.decisions[-1].selected['next_action'] == 'continue'
    assert world.decisions[-1].degraded


@pytest.mark.parametrize('world', [{'mask': {'next_action': {'inspect', 'halt_now'}},
                                    'llm_weights': {'next_action': {'continue': 0.0,
                                                                    'inspect': 1.0,
                                                                    'halt_now': 0.0}}}],
                         indirect=True)
def test_masks_are_absolute(world):
    assert world.wait(lambda: any(p in ('KEPT', 'COMMITTED', 'REVISED') for p in world.phases()))
    assert all(d.selected['next_action'] == 'continue' for d in world.decisions)
