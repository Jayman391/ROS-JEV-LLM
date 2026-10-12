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


def test_order_ack_tells_relay_points_apart():
    """A stale status of the previous point must not ack the next one (it was never sent)."""
    a = Coordinator.expect('relay', {'point': [-12.0, 24.0, 56.0]})
    b = Coordinator.expect('relay', {'point': [58.0, 20.0, 56.0]})
    assert a == 'relay (-12, 24)' and not a.startswith(b)
    assert Coordinator.expect('takeoff', {}) == 'launch'


def test_relaunch_only_healthy_landed_drones_once():
    """Live: uav5/uav7 landed on their own at 84% and sat out the mission; uav8 had a motor
    fault and must stay down."""
    def st(**kw):
        return SimpleNamespace(**{'task': 'LANDED', 'airborne': False, 'critical': False,
                                  'faults': [], 'margin': 0.69, 'energy_frac': 0.84, **kw})
    orders, events = [], []
    me = SimpleNamespace(status={'uav5': st(), 'uav7': st(), 'uav8': st(faults=['motor']),
                                 'uav1': st(margin=0.04), 'uav2': st(task='SURVEY', airborne=True)},
                         recalled={'uav7'}, relaunched=set(), p=params(),
                         order=lambda d, k, pl: orders.append((d, k)), event=events.append)
    Coordinator.relaunch(me)
    Coordinator.relaunch(me)                                         # once per drone
    assert orders == [('uav5', 'takeoff')] and len(events) == 1


def test_plan_cannot_change_survey_altitude():
    """Survey altitude is the scenario's (20 m), whatever the LLM plan says."""
    p = params()
    raw = {'survey_altitude_m': 40, 'priority_sectors': ['A', 'Z']}
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
                               phase=s.pop('phase', 'SEARCH'),
                               status={'uav1': _status(**s)}, assign={'uav1': 0})
    healthy = coord()
    assert Coordinator.report_masked(healthy) == OPTIONAL            # only acknowledge is left
    low = Coordinator.report_masked(coord(margin=0.1))
    assert 'recall_drone' not in low and 'reassign_sector' not in low
    assert 'recall_drone' not in Coordinator.report_masked(coord(faults=['motor']))
    # seed 0: in RECOVER, reassign_sector's hold replaced uav5's RTB for 5 minutes
    assert Coordinator.report_masked(coord(margin=0.1, phase='RECOVER')) == OPTIONAL


def test_search_ends_from_the_way_home():
    p = params()
    me = SimpleNamespace(p=p, t_max=900.0, phase='SEARCH', phase_t=0.0, now=lambda: 0.0,
                         status={'uav1': _status(position=SimpleNamespace(x=400.0, y=300.0))})
    me.recover_lead_s = lambda: Coordinator.recover_lead_s(me)
    lead = me.recover_lead_s()
    assert lead == 500 / p['v_transit'] + p['alt_relay'] / p['v_max_down'] + 8 + 30
    me.elapsed = lambda: 900 - lead - CONFIRM_S - 1
    assert not Coordinator.forced(me)
    me.elapsed = lambda: 900 - lead - CONFIRM_S + 1
    assert Coordinator.forced(me)


def test_fleet_has_one_relay_per_four_workers():
    p = params()
    assert scenario.workers(p)[-1] == 'uav8' and scenario.relays(p) == ['uav9', 'uav10']
    assert len(scenario.drones(p)) == 10 and len(scenario.drones(params(num_drones=3))) == 3
    bands = scenario.relay_bands(p)
    x0, x1, y0, y1 = p['ao']
    assert bands == {'uav9': (y0, 0.0), 'uav10': (0.0, y1)}           # sectors A-D, E-H
    layers = {scenario.cruise_alt(p, d) for d in scenario.workers(p)[:4]}
    assert layers == {40.0, 44.0, 48.0, 52.0}                        # d_sep apart: paths cross
    assert scenario.cruise_alt(p, 'uav9') == p['alt_relay'] > max(layers)
    assert max(o[3] for o in scenario.obstacles(p)) < p['alt_relay']
    pads = [scenario.pad(p, k)[1] for k in range(10)]
    assert pads[0] == -pads[-1]                                      # the row stays centred
    assert 'uav10' in scenario.world_sdf(p)


def test_relay_point_is_the_groups_centre_within_its_own_uplink():
    p = params()
    near = scenario.relay_point(p, [(100.0, 50.0), (140.0, 50.0), (120.0, 90.0)])
    assert math.dist(near[:2], (120.0, 65.0)) < 6 and near[2] == p['alt_relay']   # circumcentre
    far = scenario.relay_point(p, [(420.0, 288.0), (420.0, 216.0), (380.0, 250.0)])
    assert scenario.link_q(p, math.dist((0, 0, 0), far)) >= p['relay_uplink_q'] - 1e-6
    # what the far corner gets through it, vs direct (0.0): contact, if not a good link
    hub = [(*far, scenario.link_q(p, math.dist((0, 0, 0), far)))]
    assert scenario.q_at(p, (420.0, 288.0, 20.0), hub) > 0.1 > scenario.q_at(p, (420, 288, 20), [])
    for nx, ny, r in scenario.nfzs(p):                                # never parked in a NFZ
        at = scenario.relay_point(p, [(nx, ny)])
        assert math.hypot(at[0] - nx, at[1] - ny) >= r + p['nfz_buffer']


def test_leash_masks_inspections_beyond_the_relays():
    p = params()
    t = {'id': 'T1', 'x': 380.0, 'y': 250.0, 'sightings': [(20.0, {'survivor': 0.6})],
         'inspected': False, 'resolved': False, 'reported': 1}
    me = SimpleNamespace(status=_status(position=SimpleNamespace(x=370.0, y=240.0)),
                         mission={'phase': 'SEARCH'}, support_since=None, p=p,
                         order={'kind': 'survey', 'done': False}, inspecting=None,
                         tracks={'T1': t}, peers={}, relays={'uav9', 'uav10'}, home=(0.0, 0.0),
                         now=lambda: 0.0, evidence=DroneAgent.evidence)
    for m in ('unresolved', 'live_order', 'best_track', 'reach'):
        setattr(me, m, getattr(DroneAgent, m).__get__(me))
    assert 'inspect' in DroneAgent.mask(me, 'step')['next_action']        # no relay out there
    me.peers['uav10'] = SimpleNamespace(position=SimpleNamespace(x=200.0, y=120.0, z=56.0),
                                        link_q=0.8, airborne=True)
    assert me.reach(380.0, 250.0) >= p['leash_q']
    assert 'inspect' not in DroneAgent.mask(me, 'step')['next_action']    # within its reach
