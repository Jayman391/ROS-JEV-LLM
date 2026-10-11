"""Pure checks of the drone swarm example (no ROS graph, no Gazebo): scenario, world, coverage,
catalogs and the keyless mock rules."""
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import yaml

from drone_swarm import mock_rules, scenario
from drone_swarm.coordinator import catalog_for, Coordinator
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
    cov.see(100, 0, 30, 4)                                      # survey quality
    seen = len(cov.covered)
    cov.see(200, 0, 50, 4)                                      # too high: nothing
    cov.see(200, 0, 30, 9)                                      # too fast: nothing
    assert seen > 5 and len(cov.covered) == seen
    # every sector, out and home in the worst wind, fits in the battery above the reserve
    usable = p['E_0'] * (1 - p['reserve'])
    for wind in ((p['wind_max'], 0), (-p['wind_max'], 0), (0, p['wind_max'])):
        for s in scenario.sectors(p):
            back = scenario.leg_wh(p, (s[2], (s[1] + s[3]) / 2), (0, 0), wind)
            assert scenario.survey_wh(p, s, (0, 0), wind) + back < usable
    assert abs(scenario.link_q(p, p['R_c']) - 0.5) < 1e-9


def test_catalogs_load_and_mock_answers_everything():
    drones, sectors = ['uav1', 'uav2'], ['A', 'B']
    cats = [load_catalog(yaml.safe_load(open(CONFIG / 'drone.yaml'))),
            load_catalog(catalog_for(str(CONFIG / 'coordinator.yaml'), drones, sectors))]
    assert 'fit_uav2_B' in cats[1].question_sets['fit'].questions
    state = {'energy': 'ample (90% left)', 'detection': 'uav1-1, 40 m E: 2 sightings from 30 m; '
             'scores survivor 0.55, animal 0.30; not inspected; not reported',
             'evidence': '3 sightings, seen from 10 m at best; scores survivor 0.80',
             'report': 'hazard (priority 0.30): {"faults": []}', 'pairs': 'uav1->A: 90 m away',
             'mission': 'SEARCH, 2:00 of 10:00; ready to advance'}
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


def test_plan_cannot_change_survey_altitude():
    """Survey altitude is the scenario's (20 m), whatever the LLM plan says."""
    p = params()
    raw = {'survey_altitude_m': 40, 'transit_formation': 'v', 'formation_spacing_m': 8,
           'search_end_frac': 0.7, 'recover_frac': 0.85, 'priority_sectors': ['A', 'Z']}
    plan = Coordinator.validate_plan(SimpleNamespace(p=p, snames=['A', 'B']), raw)
    assert plan['survey_altitude_m'] == p['alt_survey'] == 20 and plan['priority_sectors'] == ['A']
    assert 0.5 < scenario.p_detect(p, 20, p['v_survey']) < 0.55
    w = scenario.sectors(p)[0]
    assert (w[3] - w[1]) / scenario.lane_spacing(p) <= 4             # 4 whole lanes per strip
