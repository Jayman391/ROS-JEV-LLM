"""Pure checks of the drone swarm example (no ROS graph, no Gazebo): scenario, world, coverage,
catalogs and the keyless mock rules."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import yaml

from drone_swarm import mock_rules, scenario
from drone_swarm.coordinator import CONFIRM_S, OPTIONAL, Coordinator
from drone_swarm.drone_agent import DroneAgent
from jev_ai.core.catalog import load_catalog

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def params(**kw):
    return scenario.load(CONFIG / 'scenario.yaml', **kw)


def test_episode_is_seeded_and_valid():
    p = params()
    a, b, c = scenario.episode(p, 3), scenario.episode(p, 3), scenario.episode(p, 4)
    assert a == b and a != c                                    # same seed = same world (CRN)
    kinds = [k for _, k, _, _ in a['targets']]
    assert len(kinds) >= p['num_targets'] - 1 and kinds.count('survivor') == p['num_survivors']
    assert math.hypot(*a['wind']) <= p['wind_max']
    x0, x1, y0, y1 = p['ao']
    for _, _, x, y in a['targets']:
        assert x0 < x < x1 and y0 < y < y1
        assert all(math.hypot(x - ox, y - oy) > r for ox, oy, r, _ in scenario.obstacles(p))
        assert all(math.hypot(x - nx, y - ny) > r for nx, ny, r in scenario.nfzs(p))


def test_world_has_every_drone_and_target():
    p = params(num_drones=6, seed=2)
    root = ET.fromstring(scenario.world_sdf(p))
    names = {m.get('name') for m in root.iter('model')}
    assert {f'uav{k}' for k in range(1, 7)} <= names
    assert {f'target_{t}' for t, _, _, _ in scenario.episode(p)['targets']} <= names
    bridged = {b['ros_topic_name'] for b in scenario.bridge_config(p)}
    assert '/uav6/odom' in bridged and '/clock' in bridged
    assert len({scenario.pad(p, k) for k in range(6)}) == 6


def test_coverage_and_energy():
    p = params()
    cov = scenario.Coverage(p)
    cov.see(100, 0, 30, p['v_survey'])                          # survey quality
    seen = len(cov.covered)
    cov.see(200, 0, 50, 4)                                      # too high: nothing
    cov.see(200, 0, 30, 9)                                      # too fast: nothing
    assert seen > 5 and len(cov.covered) == seen
    # every sector, out and home in the worst wind, plus a launch's 5 min of hovering and two
    # inspections' worth of slack, fits in the battery above the reserve (live: 60 Wh did not)
    usable = p['E_0'] * (1 - p['reserve'])
    slack = p['P_hover'] * 300 / 3600 + 2 * p['P_hover'] * 60 / 3600
    for wind in ((p['wind_max'], 0), (-p['wind_max'], 0), (0, p['wind_max'])):
        for s in scenario.sectors(p):
            back = scenario.leg_wh(p, (s[2], (s[1] + s[3]) / 2), (0, 0), wind)
            assert scenario.survey_wh(p, s, (0, 0), wind) + back + slack < usable
    assert abs(scenario.link_q(p, p['R_c']) - 0.5) < 1e-9


def test_catalogs_load_and_mock_answers_everything():
    cats = [load_catalog(yaml.safe_load(open(CONFIG / name)))
            for name in ('drone.yaml', 'coordinator.yaml')]
    assert set(cats[1].question_sets) == {'global', 'track', 'report'}     # no generated fit set
    assert all(c.policy.ask_jev == 'change' for c in cats)
    state = {'energy': 'ample (90% left)', 'detection': 'uav1-1, 40 m E: 2 sightings from 30 m; '
             'scores survivor 0.55, animal 0.30; not inspected; not reported',
             'evidence': '3 sightings, seen from 10 m at best; scores survivor 0.80',
             'report': 'hazard (priority 0.30): {"faults": []}',
             'mission': 'SEARCH, 2:00 of 10:00'}
    for cat in cats:
        for qs in cat.question_sets.values():
            for q in qs.questions.values():
                ans = mock_rules.swarm(state, SimpleNamespace(name=q.name))
                assert ans is not None, q.name
                assert set(ans) == set(q.class_names), q.name
                assert abs(sum(ans.values()) - 1) < 1e-6 and min(ans.values()) >= 0, q.name
    # a hazard report is acknowledged (regression: "faults" in the payload looked like a fault)
    r = mock_rules.swarm(state, SimpleNamespace(name='report_response'))
    assert max(r, key=r.get) == 'acknowledge'


def test_order_ack_tells_formation_slots_apart():
    """A stale status of the previous slot must not ack the next one (it was never sent)."""
    a = Coordinator.expect('formation', {'point': [-12.0, 24.0, 40.0]})
    b = Coordinator.expect('formation', {'point': [58.0, 20.0, 40.0]})
    assert a == 'formation (-12, 24)' and not a.startswith(b)
    assert Coordinator.expect('takeoff', {}) == 'launch'


def test_formation_tail_stays_ahead_of_the_pads():
    """Live regression: an 8-drone 10 m column trailed to x = -58, past the H1 fence at -20;
    four drones never reached their slots and TRANSIT timed out."""
    from drone_swarm.coordinator import SLOT_X_MIN, ahead_shift
    column = [12.0 - 10 * k for k in range(8)]                   # 12 .. -58
    dx = ahead_shift(column, SLOT_X_MIN)
    assert min(x + dx for x in column) == SLOT_X_MIN and dx == 70
    assert ahead_shift([20.0, 30.0], SLOT_X_MIN) == 0             # already ahead: unchanged


def test_relaunch_only_healthy_landed_drones_once():
    """Live: uav5/uav7 landed on their own at 84% and sat out the mission; uav8 had a motor
    fault and must stay down."""
    def st(**kw):
        return SimpleNamespace(**{'task': 'LANDED', 'airborne': False, 'critical': False,
                                  'faults': [], 'margin': 0.69, 'energy_frac': 0.84, **kw})
    orders, events = [], []
    me = SimpleNamespace(status={'uav5': st(), 'uav7': st(), 'uav8': st(faults=['motor']),
                                 'uav1': st(margin=0.04), 'uav2': st(task='SURVEY', airborne=True)},
                         recalled={'uav7'}, relaunched=set(), p={'alt_transit': 40.0},
                         order=lambda d, k, pl: orders.append((d, k)), event=events.append)
    Coordinator.relaunch(me)
    Coordinator.relaunch(me)                                         # once per drone
    assert orders == [('uav5', 'takeoff')] and len(events) == 1


def test_plan_cannot_change_survey_altitude():
    """Survey altitude is the scenario's (20 m), whatever the LLM plan says."""
    p = params()
    raw = {'survey_altitude_m': 40, 'transit_formation': 'v', 'formation_spacing_m': 8,
           'priority_sectors': ['A', 'Z']}
    plan = Coordinator.validate_plan(SimpleNamespace(p=p, snames=['A', 'B']), raw)
    assert plan['survey_altitude_m'] == p['alt_survey'] == 20 and plan['priority_sectors'] == ['A']
    assert 0.38 < scenario.p_detect(p, 20, p['v_survey']) < 0.42     # per 0.5 s frame at 6 m/s
    w = scenario.sectors(p)[0]
    assert (w[3] - w[1]) / scenario.lane_spacing(p) <= 4             # 4 whole lanes per strip
    assert p['v_max_h'] > p['v_survey']                              # the FC can fly the survey


def _status(**kw):
    base = {'airborne': True, 'critical': False, 'faults': [], 'reflexes': [], 'margin': 0.5,
            'link_lost_s': 0.0, 'over_obstacle': False, 'energy_frac': 0.8,
            'energy_needed_frac': 0.2, 'position': SimpleNamespace(x=200.0, y=100.0),
            'task': 'IDLE'}
    return SimpleNamespace(**{**base, **kw})


def test_idle_drone_holds_instead_of_picking_noise():
    """Live: a drone parked with no order had continue and inspect masked; the models' leftovers
    picked request_support, and the coordinator recalled it."""
    me = SimpleNamespace(status=_status(), mission={'phase': 'SEARCH'}, support_since=None,
                         p={'lost_link_hold_s': 10.0}, order=None, inspecting=None,
                         tracks={}, now=lambda: 0.0)
    me.unresolved = lambda: DroneAgent.unresolved(me)
    me.live_order = lambda: DroneAgent.live_order(me)
    me.mask = lambda qs: DroneAgent.mask(me, qs)
    masked = me.mask('step')['next_action']
    assert {'continue', 'inspect', 'request_support'} <= masked
    assert DroneAgent.require(me, 'step') == {'next_action': 'hold'}
    me.order = {'kind': 'survey', 'done': False}                     # an order: the models decide
    assert DroneAgent.require(me, 'step') == {}


def test_survey_runs_through_link_loss_and_resends():
    """46% of the AO is past the usable link: a survey there keeps going (H7 waits for the task to
    end) and the tracks it found are resent when the link comes back."""
    lost = _status(link_lost_s=40.0, task='SURVEY')
    me = SimpleNamespace(status=lost, mission={'phase': 'SEARCH'}, support_since=None,
                         p={'lost_link_hold_s': 10.0}, order={'kind': 'survey', 'done': False},
                         inspecting=None, tracks={}, now=lambda: 0.0)
    me.unresolved = lambda: DroneAgent.unresolved(me)
    me.live_order = lambda: DroneAgent.live_order(me)
    assert 'continue' not in DroneAgent.mask(me, 'step')['next_action']
    me.status = _status(link_lost_s=40.0, task='HOLD')
    assert 'continue' in DroneAgent.mask(me, 'step')['next_action']        # idle: H7 applies

    sent = []
    t = {'id': 'T1', 'sightings': [(20.0, {})], 'reported': 0}
    me = SimpleNamespace(link_bucket='lost', link_since=0.0, now=lambda: 1.0, tracks={'T1': t},
                         engine=SimpleNamespace(trigger=lambda *a, **k: None),
                         report=lambda *a, **k: sent.append(k['track']['id']))
    DroneAgent._link(me, SimpleNamespace(quality=0.9))
    assert sent == ['T1']


def test_recall_needs_a_cause():
    def coord(**s):
        return SimpleNamespace(report_focus=({'drone': 'uav1', 'payload': {}}, 0.0),
                               status={'uav1': _status(**s)}, assign={'uav1': 0})
    healthy = coord()
    assert Coordinator.report_masked(healthy) == OPTIONAL            # only acknowledge is left
    low = Coordinator.report_masked(coord(margin=0.1))
    assert 'recall_drone' not in low and 'reassign_sector' not in low
    assert 'recall_drone' not in Coordinator.report_masked(coord(faults=['motor']))


def test_search_ends_from_the_way_home():
    p = params()
    me = SimpleNamespace(p=p, t_max=900.0, phase='SEARCH', phase_t=0.0, now=lambda: 0.0,
                         status={'uav1': _status(position=SimpleNamespace(x=400.0, y=300.0))})
    me.recover_lead_s = lambda: Coordinator.recover_lead_s(me)
    lead = me.recover_lead_s()
    assert lead == 500 / p['v_transit'] + p['alt_transit'] / p['v_max_down'] + 8 + 30
    me.elapsed = lambda: 900 - lead - CONFIRM_S - 1
    assert not Coordinator.forced(me)
    me.elapsed = lambda: 900 - lead - CONFIRM_S + 1
    assert Coordinator.forced(me)
