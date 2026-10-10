"""decide_server (A6): hosted decisions equal in-process ones (D2 parity), phases arrive as
feedback then result, masks/require come from the goal, bad goals fail loudly."""
import copy
import json
import threading
import time

import pytest
import yaml

rclpy = pytest.importorskip('rclpy')

from diagnostic_msgs.msg import KeyValue  # noqa: E402
from jev_ai_msgs.action import Decide, Reason  # noqa: E402
from jev_ai_msgs.msg import ClassRef, DecisionRecord, Distribution  # noqa: E402
from jev_ai_msgs.srv import JevDecide  # noqa: E402
from rclpy.action import ActionClient, ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

from jev_ai import DecisionEngine  # noqa: E402
from jev_ai.core.fusion import confidence  # noqa: E402
from jev_ai_servers.decide_server import DecideServer  # noqa: E402

from conftest import PATROL, STATE  # noqa: E402

P = {'next_action': [0.5, 0.45, 0.05], 'urgency': [0.6, 0.3, 0.1], 'report_now': [0.3, 0.7]}
WEIGHTS = {'next_action': {'continue': 0.2, 'inspect': 0.8, 'halt_now': 0.0}}


class World:
    def __init__(self, tmp_path):
        rclpy.init()
        path = tmp_path / 'patrol.yaml'
        path.write_text(yaml.safe_dump(copy.deepcopy(PATROL)))
        self.fake = rclpy.create_node('fake_servers')
        self.fake.create_service(JevDecide, 'jev/decide', self._jev)
        self.reason = ActionServer(self.fake, Reason, 'llm/reason', self._llm)
        self.server = DecideServer(parameter_overrides=[
            rclpy.parameter.Parameter('catalog', value=str(path)),
            rclpy.parameter.Parameter('run_id', value='parity')])
        self.client_node = rclpy.create_node('client')
        self.client = ActionClient(self.client_node, Decide, 'decide')
        self.records = []
        self.client_node.create_subscription(DecisionRecord, 'decisions', self.records.append,
                                             100)
        self.ex = MultiThreadedExecutor(num_threads=8)
        for n in (self.fake, self.server, self.client_node):
            self.ex.add_node(n)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        assert self.client.wait_for_server(timeout_sec=5)

    def _jev(self, req, res):
        res.success, res.model = True, 'fake-jev'
        for q in req.questions:
            p = P[q.name]
            res.answers.append(Distribution(question=q.name, type=q.type, classes=q.classes,
                                            probabilities=p, confidence=confidence(p)))
            res.question_errors.append('')
        return res

    def _llm(self, gh):
        gh.succeed()
        return Reason.Result(success=True, model='fake-llm', json=json.dumps(
            {'weights': WEIGHTS, 'params': {}, 'rationale': 'ok', 'flags': []}))

    def decide(self, agent='robot', state=STATE, mask=(), require=(), **kw):
        goal = Decide.Goal(agent=agent, question_set=kw.pop('question_set', 'next_step'),
                           state=[KeyValue(key=k, value=v) for k, v in state.items()],
                           mask=[ClassRef(question=q, cls=c) for q, c in mask],
                           require=[ClassRef(question=q, cls=c) for q, c in require], **kw)
        feedback = []
        fut = self.client.send_goal_async(goal, feedback_callback=lambda f: feedback.append(
            f.feedback.decision))
        gh = self._wait(fut)
        if not gh.accepted:
            return None, None, feedback
        res = self._wait(gh.get_result_async())
        return res.status, res.result.decision, feedback

    @staticmethod
    def _wait(fut, timeout=10.0):
        end = time.monotonic() + timeout
        while not fut.done():
            assert time.monotonic() < end, 'timed out'
            time.sleep(0.01)
        return fut.result()

    def close(self):
        self.ex.shutdown()
        for n in (self.fake, self.server, self.client_node):
            n.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def world(tmp_path):
    w = World(tmp_path)
    yield w
    w.close()


def selected(d):
    return {r.question: r.cls for r in d.selected}


def test_feedback_then_final_result(world):
    t0 = time.monotonic()
    status, d, feedback = world.decide(directive_issuer='sup', directive_id=1)
    assert status == 4 and d.phase in ('KEPT', 'COMMITTED')          # SUCCEEDED
    assert feedback and feedback[0].phase == 'PROVISIONAL'
    assert time.monotonic() - t0 < 2.0                                # no timer-wake latency
    fused = {f.question: list(f.probabilities) for f in d.fused}
    assert sum(fused['next_action']) == pytest.approx(1.0)
    assert {kv.key for kv in d.values} >= {'urgency.level_mean'}
    rec = [r for r in world.records if r.agent == 'robot']
    assert rec and rec[-1].run_id == 'parity' and 'directive:remote' in rec[0].triggers
    # a tick (no triggers) with nothing due returns the standing decision, no new event
    status, tick, feedback = world.decide()
    assert status == 4 and tick.decision_id == d.decision_id and not feedback
    # an app trigger is a decision event
    _, ev, _ = world.decide(triggers=['detection'])
    assert ev.decision_id == d.decision_id + 1


def test_parity_with_in_process_engine(world):
    """D2: the hosted engine is the engine. Same run_id, agent and directive epoch → the same
    noise, logits and selection as a DecisionEngine attached in-process."""
    _, remote, _ = world.decide(agent='twin', directive_issuer='sup', directive_id=7)
    time.sleep(0.3)                                                   # let records arrive
    before = len(world.records)
    got = []
    node = rclpy.create_node('twin_host')
    DecisionEngine(node, copy.deepcopy(PATROL), encode_state=lambda _: dict(STATE),
                   on_decision=got.append, run_id='parity', agent='twin', autonomous=False,
                   all_phases=True).trigger('decide', directive=('sup', 7))
    world.ex.add_node(node)
    end = time.monotonic() + 10
    while not any(d.final for d in got):
        assert time.monotonic() < end
        time.sleep(0.02)
    local = next(d for d in got if d.final)
    assert local.selected == selected(remote) and local.phase == remote.phase
    time.sleep(0.3)
    hosted = [r for r in world.records[:before] if r.agent == 'twin' and r.phase == remote.phase]
    in_proc = [r for r in world.records[before:] if r.agent == 'twin' and r.phase == local.phase]
    assert hosted and in_proc
    a, b = hosted[0].questions[0], in_proc[0].questions[0]
    assert list(a.noise) == list(b.noise) and list(a.logits) == list(b.logits)
    world.ex.remove_node(node)
    node.destroy_node()


def test_goal_masks_and_require(world):
    _, d, _ = world.decide(mask=[('next_action', 'inspect')])
    assert selected(d)['next_action'] != 'inspect'
    status, d, _ = world.decide(agent='r2', require=[('next_action', 'halt_now')])
    assert d.phase == 'FORCED' and selected(d)['next_action'] == 'halt_now'


def test_bad_goals(world):
    assert world.decide(question_set='nope')[0] is None              # rejected
    assert world.decide(agent='')[0] is None
    status, d, _ = world.decide(state={'task': 'x'})                   # wrong state keys
    assert status == 6 and d.rationale.startswith('error:')            # ABORTED
