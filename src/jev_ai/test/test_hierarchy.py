"""Hierarchy (A7): a directive makes every worker decide in the directive's epoch with shared
group noise; expired, duplicate, foreign and non-allowlisted directives are dropped; reports
arrive with beliefs and payload."""
import copy
import json
import threading
import time

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('jev_ai_msgs')

from builtin_interfaces.msg import Time  # noqa: E402
from jev_ai_msgs.action import Reason  # noqa: E402
from jev_ai_msgs.msg import DecisionRecord, Directive, Distribution  # noqa: E402
from jev_ai_msgs.srv import JevDecide  # noqa: E402
from rclpy.action import ActionServer  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

from jev_ai import DecisionEngine, Hierarchy  # noqa: E402
from jev_ai.core.fusion import confidence  # noqa: E402

from conftest import PATROL, STATE  # noqa: E402

P = {'next_action': [0.5, 0.45, 0.05], 'urgency': [0.6, 0.3, 0.1], 'report_now': [0.3, 0.7]}


class Fleet:
    def __init__(self, n=3):
        rclpy.init()
        self.fake = rclpy.create_node('fake_servers')
        self.fake.create_service(JevDecide, 'jev/decide', self._jev)
        self.reason = ActionServer(self.fake, Reason, 'llm/reason', self._llm)
        self.sup = rclpy.create_node('supervisor')
        self.reports = []
        self.boss = Hierarchy(self.sup, on_report=lambda m, p: self.reports.append((m, p)))
        self.records, self.orders = [], []
        self.sup.create_subscription(DecisionRecord, 'decisions', self.records.append, 1000)
        self.workers, self.links = [], []
        for i in range(n):
            node = rclpy.create_node(f'worker_{i}')
            eng = DecisionEngine(node, copy.deepcopy(PATROL), encode_state=lambda _: dict(STATE),
                                 run_id='fleet-run', autonomous=False)
            self.workers.append(node)
            self.links.append(Hierarchy(node, engine=eng, qset='next_step',
                                        issuer_allowlist=['/supervisor'],
                                        on_directive=lambda m, p: self.orders.append(m)))
        self.ex = MultiThreadedExecutor(num_threads=8)
        for nd in [self.fake, self.sup] + self.workers:
            self.ex.add_node(nd)
        threading.Thread(target=self.ex.spin, daemon=True).start()
        time.sleep(0.5)                                  # discovery

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
            {'weights': {'next_action': {'continue': 0.5, 'inspect': 0.5, 'halt_now': 0.0}},
             'params': {}, 'rationale': 'ok', 'flags': []}))

    def wait(self, pred, timeout=8.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if pred():
                return True
            time.sleep(0.05)
        return pred()

    def close(self):
        self.ex.shutdown()
        for nd in [self.fake, self.sup] + self.workers:
            nd.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def fleet():
    f = Fleet()
    yield f
    f.close()


def provisional(fleet, subject):
    return [r for r in fleet.records if r.phase == 'PROVISIONAL' and r.subject == subject]


def test_directive_couples_workers(fleet):
    did = fleet.boss.direct('converge', payload={'subject': 'd1'}, ttl_s=10)
    assert fleet.wait(lambda: len(provisional(fleet, 'd1')) == 3)
    recs = provisional(fleet, 'd1')
    assert {r.agent for r in recs} == {'/worker_0', '/worker_1', '/worker_2'}
    assert all('directive:converge' in r.triggers for r in recs)
    noise = [list(r.questions[0].noise) for r in recs]
    assert noise[0] == noise[1] == noise[2]                   # shared group noise (rho = 1)
    assert all(r.questions[0].group_noise for r in recs)
    assert len({r.questions[0].selected for r in recs}) == 1  # same fused dist + noise → same pick
    assert [m.directive_id for m in fleet.orders] == [did] * 3

    # a second directive is a new epoch: new noise, still shared
    fleet.boss.direct('converge', payload={'subject': 'd2'})
    assert fleet.wait(lambda: len(provisional(fleet, 'd2')) == 3)
    noise2 = [list(r.questions[0].noise) for r in provisional(fleet, 'd2')]
    assert noise2[0] == noise2[1] == noise2[2] != noise[0]


def test_drops(fleet):
    pub = fleet.boss.directive_pub
    now = fleet.sup.get_clock().now().nanoseconds

    def send(**kw):
        m = Directive(issuer='/supervisor', directive_id=kw.pop('id', 1), target='*',
                      kind='converge', payload='{}')
        for k, v in kw.items():
            setattr(m, k, v)
        pub.publish(m)

    send(id=5)
    assert fleet.wait(lambda: len(fleet.orders) == 3)
    send(id=5)                                                     # duplicate
    send(id=6, expires=Time(sec=(now // 10**9) - 10))              # expired
    send(id=7, issuer='/intruder')                                 # not allowlisted
    send(id=8, target='/worker_1')                                 # only worker_1
    assert fleet.wait(lambda: len(fleet.orders) == 4)
    time.sleep(0.5)
    assert len(fleet.orders) == 4 and fleet.orders[-1].directive_id == 8
    dropped = [h.dropped for h in fleet.links]
    assert all(d['duplicate'] == 1 and d['expired'] == 1 and d['issuer'] == 1 for d in dropped)
    assert sum(d['target'] for d in dropped) == 2


def test_reports(fleet):
    fleet.links[0].report('status', subject='person at 12 m', priority=0.7,
                          beliefs={'urgency': (['none', 'low', 'high'], [0.1, 0.2, 0.7])},
                          payload={'task': 'patrolling'})
    assert fleet.wait(lambda: fleet.reports)
    msg, payload = fleet.reports[0]
    assert msg.agent == '/worker_0' and msg.priority == pytest.approx(0.7)
    assert payload == {'task': 'patrolling'} and msg.beliefs[0].confidence > 0
