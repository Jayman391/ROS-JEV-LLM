"""The drone swarm scenario (SYSTEM_MODEL §2, §3.1, §8): parameters, episode randomisation, energy
model, the generated Gazebo world and the ros_gz bridge config.

Everything derives from ``config/scenario.yaml`` (the same file the C++ nodes read as ROS params)
plus an episode ``seed``. ``world_sdf`` and the ``environment`` node call ``episode()`` with the
same seed, so the targets drawn in Gazebo are exactly the hidden ground truth.
"""
from __future__ import annotations

import math
import random
from pathlib import Path

import yaml

CLASSES = ['survivor', 'vehicle', 'debris', 'animal']
DETECTOR_CLASSES = CLASSES + ['nothing']
DRONE_RGB = [(0.9, 0.15, 0.15), (0.15, 0.6, 0.95), (0.95, 0.8, 0.1), (0.7, 0.3, 0.9),
             (0.1, 0.85, 0.5), (1.0, 0.5, 0.0), (0.95, 0.4, 0.75), (0.1, 0.9, 0.9)]
# class -> (box size, rgb): survivor orange, vehicle blue, debris brown, animal green
TARGET_LOOK = {
    'survivor': ((1.8, 0.7, 0.5), (1.0, 0.45, 0.0)),
    'vehicle': ((4.2, 2.0, 1.5), (0.1, 0.25, 0.9)),
    'debris': ((2.5, 2.0, 0.6), (0.45, 0.3, 0.15)), 'animal': ((1.4, 0.5, 0.9), (0.1, 0.6, 0.1))}
FAULT_KINDS = ['battery_fault', 'gps_degraded', 'motor_degraded', 'comms_blackout']


def load(path: str | Path, **overrides) -> dict:
    """The ``/**`` ros__parameters of a scenario file, with overrides (e.g. num_drones, seed)."""
    p = dict(yaml.safe_load(Path(path).read_text())['/**']['ros__parameters'])
    p.update({k: v for k, v in overrides.items() if v is not None})
    return p


def from_node(node) -> dict:
    """The scenario for a ROS node: param ``scenario_file`` (default: this package's) with the
    launch's ``num_drones`` / ``seed`` / ``t_max_s`` overrides (-1 / -1 / 0 = the file's)."""
    from ament_index_python.packages import get_package_share_directory
    default = get_package_share_directory('drone_swarm') + '/config/scenario.yaml'
    path = node.declare_parameter('scenario_file', default).value
    over = {}
    for k, unset in (('num_drones', -1), ('seed', -1), ('t_max_s', 0.0)):
        v = node.declare_parameter(k, unset).value
        if v != unset:
            over[k] = v
    return load(path, **over)


def drones(p: dict) -> list[str]:
    return [f'uav{k + 1}' for k in range(int(p['num_drones']))]


def pad(p: dict, k: int) -> tuple[float, float]:
    """Landing spot of drone index k (0-based): a row along y through the base."""
    return 0.0, (k - (int(p['num_drones']) - 1) / 2) * p['pad_spacing']


def sectors(p: dict, n: int | None = None) -> list[tuple[float, float, float, float]]:
    """The AO split into n equal strips along y: (x0, y0, x1, y1). Lanes run along x."""
    x0, x1, y0, y1 = p['ao']
    n = n or int(p['num_drones'])
    w = (y1 - y0) / n
    return [(x0, y0 + i * w, x1, y0 + (i + 1) * w) for i in range(n)]


def sector_name(i: int) -> str:
    return chr(ord('A') + i)


def obstacles(p: dict) -> list[tuple[float, float, float, float]]:
    o = p['obstacles']
    return [tuple(o[i:i + 4]) for i in range(0, len(o), 4)]


def nfzs(p: dict) -> list[tuple[float, float, float]]:
    o = p['nfz']
    return [tuple(o[i:i + 3]) for i in range(0, len(o), 3)]


# --- episode -------------------------------------------------------------------------------------

def episode(p: dict, seed: int | None = None) -> dict:
    """Targets, wind, visibility and faults for one seeded episode (§2 'Episode randomisation')."""
    seed = p['seed'] if seed is None else seed
    rng = random.Random(f'episode|{seed}')
    n = int(p['num_targets'])
    surv = int(p['num_survivors'])
    veh, ani = rng.randint(1, max(2, n // 6)), rng.randint(1, max(2, n // 6))
    deb = max(2, n - surv - veh - ani)
    kinds = ['survivor'] * surv + ['vehicle'] * veh + ['debris'] * deb + ['animal'] * ani
    x0, x1, y0, y1 = p['ao']
    targets: list[tuple[str, str, float, float]] = []

    def free(x, y, gap=10.0):
        return (all(math.hypot(x - ox, y - oy) > r + 4 for ox, oy, r, _ in obstacles(p)) and
                all(math.hypot(x - nx, y - ny) > r + p['nfz_buffer'] + 5 for nx, ny, r in nfzs(p))
                and all(math.hypot(x - tx, y - ty) > gap for _, _, tx, ty in targets))

    paired = rng.randint(1, 2)        # survivors placed 8-15 m from a distractor
    for kind in kinds[surv:] + kinds[:surv]:
        for _ in range(1000):
            if kind == 'survivor' and paired and len(targets) > 0:
                _, _, tx, ty = rng.choice(targets)
                a, d = rng.uniform(0, 2 * math.pi), rng.uniform(8, 15)
                x, y, gap = tx + d * math.cos(a), ty + d * math.sin(a), 6.0
            else:
                x, y, gap = rng.uniform(x0 + 8, x1 - 8), rng.uniform(y0 + 8, y1 - 8), 10.0
            if x0 + 5 < x < x1 - 5 and y0 + 5 < y < y1 - 5 and free(x, y, gap):
                if kind == 'survivor' and paired and targets:
                    paired -= 1
                break
        targets.append((f'T{len(targets) + 1}', kind, round(x, 1), round(y, 1)))
    wind_speed, wind_dir = rng.uniform(0, p['wind_max']), rng.uniform(0, 2 * math.pi)
    faults, blackouts = [], []
    if p.get('faults', True):
        for _ in range(rng.randint(0, 2)):
            kind = rng.choice(FAULT_KINDS)
            drone = rng.choice(drones(p))
            faults.append((drone, kind, round(rng.uniform(60, 300), 1)))
            if kind == 'comms_blackout':
                blackouts.append((rng.uniform(x0 + 40, x1 - 40),
                                  rng.uniform(y0 + 40, y1 - 40), 40.0))
    return {'targets': targets, 'wind': (wind_speed * math.cos(wind_dir),
                                         wind_speed * math.sin(wind_dir)),
            'visibility': rng.uniform(0.5, 1.0), 'faults': faults, 'blackouts': blackouts}


# --- energy (§3.1; mirrored in swarm_control/world.hpp) ------------------------------------------

def power_w(p: dict, v_ground: tuple[float, float], vz: float, wind: tuple[float, float]) -> float:
    vh2 = v_ground[0] ** 2 + v_ground[1] ** 2
    va2 = (v_ground[0] - wind[0]) ** 2 + (v_ground[1] - wind[1]) ** 2
    return p['P_hover'] + p['k_h'] * vh2 + p['k_c'] * max(vz, 0.0) + p['k_w'] * va2


def leg_wh(p: dict, frm: tuple[float, float], to: tuple[float, float],
           wind: tuple[float, float], speed: float | None = None) -> float:
    """Energy to fly a straight leg at ``speed`` (default v_transit) in this wind."""
    v = speed or p['v_transit']
    d = math.hypot(to[0] - frm[0], to[1] - frm[1])
    if d < 1e-6:
        return 0.0
    u = ((to[0] - frm[0]) / d, (to[1] - frm[1]) / d)
    return power_w(p, (v * u[0], v * u[1]), 0.0, wind) * (d / v) / 3600.0


def survey_wh(p: dict, sector, frm: tuple[float, float], wind=(0.0, 0.0)) -> float:
    """Rough energy to fly to a sector and survey it (lawnmower at v_survey)."""
    x0, y0, x1, y1 = sector
    lanes = max(1, math.ceil(abs(y1 - y0) / lane_spacing(p)))
    length = lanes * abs(x1 - x0) + abs(y1 - y0)
    return (leg_wh(p, frm, (x0, (y0 + y1) / 2), wind) +
            power_w(p, (p['v_survey'], 0.0), 0.0, wind) * (length / p['v_survey']) / 3600.0)


def link_q(p: dict, d: float) -> float:
    """Link quality at distance d from the base (§4.8; mirrors world.hpp link_quality)."""
    return 1.0 / (1.0 + math.exp((d - p['R_c']) / p['link_slope']))


def p_detect(p: dict, h: float, speed: float) -> float:
    """Per-pass detection probability at altitude h (§4.6; sensor_sim's, before visibility)."""
    return p['det_p0'] * math.exp(-(h - p['det_h_opt']) ** 2 / (2 * p['det_sigma_h'] ** 2)) * \
        math.exp(-speed / p['det_v_blur'])


def lane_spacing(p: dict, altitude: float | None = None) -> float:
    """2 · footprint radius · 0.8 overlap (§5.1)."""
    h = altitude or p['alt_survey']
    return 2 * h * math.tan(math.radians(p['det_fov_deg']) / 2) * 0.8


class Coverage:
    """The AO's cell_m grid; a cell is covered once its centre is inside a drone's detector
    footprint at survey quality (h <= 40 m, v <= 5 m/s; §4.6)."""

    def __init__(self, p: dict):
        self.x0, x1, self.y0, y1 = p['ao']
        self.c = p['cell_m']
        self.nx, self.ny = int((x1 - self.x0) // self.c), int((y1 - self.y0) // self.c)
        self.tan = math.tan(math.radians(p['det_fov_deg']) / 2)
        self.covered: set[tuple[int, int]] = set()

    def see(self, x: float, y: float, h: float, v: float) -> None:
        if not 2.0 < h <= 40.0 or v > 5.5:
            return
        r = h * self.tan
        i0, i1 = int((x - r - self.x0) // self.c), int((x + r - self.x0) // self.c)
        j0, j1 = int((y - r - self.y0) // self.c), int((y + r - self.y0) // self.c)
        for i in range(max(0, i0), min(self.nx - 1, i1) + 1):
            for j in range(max(0, j0), min(self.ny - 1, j1) + 1):
                cx, cy = self.x0 + (i + 0.5) * self.c, self.y0 + (j + 0.5) * self.c
                if (cx - x) ** 2 + (cy - y) ** 2 <= r * r:
                    self.covered.add((i, j))

    def fraction(self, rect: tuple[float, float, float, float] | None = None) -> float:
        """Covered share of the AO, or of the cells whose centres are in rect (x0, y0, x1, y1)."""
        if rect is None:
            return len(self.covered) / (self.nx * self.ny)
        x0, y0, x1, y1 = rect
        cells = [(i, j) for i in range(self.nx) for j in range(self.ny)
                 if x0 <= self.x0 + (i + 0.5) * self.c <= x1 and
                 y0 <= self.y0 + (j + 0.5) * self.c <= y1]
        return sum(c in self.covered for c in cells) / max(1, len(cells))


# --- Gazebo world --------------------------------------------------------------------------------

def _box(name, pose, size, rgb, alpha=1.0, collide=True) -> str:
    x, y, z = pose
    geo = f'<geometry><box><size>{size[0]} {size[1]} {size[2]}</size></box></geometry>'
    mat = (f'<material><ambient>{rgb[0]} {rgb[1]} {rgb[2]} {alpha}</ambient>'
           f'<diffuse>{rgb[0]} {rgb[1]} {rgb[2]} {alpha}</diffuse></material>')
    col = f'<collision name="c">{geo}</collision>' if collide else ''
    tr = f'<transparency>{1 - alpha}</transparency>' if alpha < 1 else ''
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>'
            f'<link name="l">{col}<visual name="v">{geo}{mat}{tr}</visual></link></model>')


def _cyl(name, pose, r, h, rgb, alpha=1.0, collide=True) -> str:
    x, y, z = pose
    geo = f'<geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>'
    mat = (f'<material><ambient>{rgb[0]} {rgb[1]} {rgb[2]} {alpha}</ambient>'
           f'<diffuse>{rgb[0]} {rgb[1]} {rgb[2]} {alpha}</diffuse></material>')
    col = f'<collision name="c">{geo}</collision>' if collide else ''
    tr = f'<transparency>{1 - alpha}</transparency>' if alpha < 1 else ''
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} {z} 0 0 0</pose>'
            f'<link name="l">{col}<visual name="v">{geo}{mat}{tr}</visual></link></model>')


ROTORS = [((0.13, -0.22), 'ccw', 1), ((-0.13, 0.2), 'ccw', 1), ((0.13, 0.22), 'cw', -1),
          ((-0.13, -0.2), 'cw', -1)]    # the Fuel "X3 UAV" layout


def x3(name: str, x: float, y: float, rgb) -> str:
    """The Gazebo X3 quadcopter (Fuel 'X3 UAV' v4: same inertials, rotors and plugin gains) with
    primitive visuals, so the world needs no network. The halo ring is visual only (seen from
    far)."""
    r, g, b = rgb
    mat = f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>'
    links, plugins = [], []
    for i, ((rx, ry), turn, d) in enumerate(ROTORS):
        links.append(f"""
      <link name="X3/rotor_{i}"><pose>{rx} {ry} 0.023 0 0 0</pose>
        <inertial><mass>0.005</mass><inertia><ixx>9.75e-07</ixx><iyy>4.17041e-05</iyy>
          <izz>4.26041e-05</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><cylinder><length>0.005</length><radius>0.1</radius>
          </cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><length>0.01</length><radius>0.1</radius></cylinder>
          </geometry><material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse>
          </material></visual>
      </link>
      <joint name="X3/rotor_{i}_joint" type="revolute"><child>X3/rotor_{i}</child>
        <parent>X3/base_link</parent><axis><xyz>0 0 1</xyz>
        <limit><lower>-1e+16</lower><upper>1e+16</upper></limit></axis></joint>""")
        plugins.append(f"""
      <plugin filename="ignition-gazebo-multicopter-motor-model-system"
              name="gz::sim::systems::MulticopterMotorModel">
        <robotNamespace>{name}</robotNamespace><jointName>X3/rotor_{i}_joint</jointName>
        <linkName>X3/rotor_{i}</linkName><turningDirection>{turn}</turningDirection>
        <timeConstantUp>0.0125</timeConstantUp><timeConstantDown>0.025</timeConstantDown>
        <maxRotVelocity>800.0</maxRotVelocity><motorConstant>8.54858e-06</motorConstant>
        <momentConstant>0.016</momentConstant>
        <commandSubTopic>gazebo/command/motor_speed</commandSubTopic>
        <motorNumber>{i}</motorNumber><rotorDragCoefficient>8.06428e-05</rotorDragCoefficient>
        <rollingMomentCoefficient>1e-06</rollingMomentCoefficient>
        <motorSpeedPubTopic>motor_speed/{i}</motorSpeedPubTopic>
        <rotorVelocitySlowdownSim>10</rotorVelocitySlowdownSim><motorType>velocity</motorType>
      </plugin>""")
    rotor_cfg = ''.join(
        f'<rotor><jointName>X3/rotor_{i}_joint</jointName><forceConstant>8.54858e-06'
        f'</forceConstant><momentConstant>0.016</momentConstant><direction>{d}</direction></rotor>'
        for i, (_, _, d) in enumerate(ROTORS))
    return f"""
    <model name="{name}"><pose>{x} {y} 0.053302 0 0 0</pose>
      <link name="X3/base_link">
        <inertial><mass>1.5</mass><inertia><ixx>0.0347563</ixx><iyy>0.07</iyy><izz>0.0977</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
        <collision name="c"><geometry><box><size>0.47 0.47 0.11</size></box></geometry></collision>
        <visual name="body"><geometry><box><size>0.47 0.47 0.11</size></box></geometry>{mat}
        </visual>
        <visual name="halo"><pose>0 0 0.08 0 0 0</pose><geometry><cylinder><radius>1.2</radius>
          <length>0.04</length></cylinder></geometry>{mat}<transparency>0.35</transparency></visual>
      </link>{''.join(links)}{''.join(plugins)}
      <plugin filename="ignition-gazebo-multicopter-control-system"
              name="gz::sim::systems::MulticopterVelocityControl">
        <robotNamespace>{name}</robotNamespace><commandSubTopic>gazebo/command/twist</commandSubTopic>
        <enableSubTopic>enable</enableSubTopic><comLinkName>X3/base_link</comLinkName>
        <velocityGain>2.7 2.7 2.7</velocityGain><attitudeGain>2 3 0.15</attitudeGain>
        <angularRateGain>0.4 0.52 0.18</angularRateGain>
        <maximumLinearAcceleration>2 2 2</maximumLinearAcceleration>
        <rotorConfiguration>{rotor_cfg}</rotorConfiguration>
      </plugin>
      <plugin filename="ignition-gazebo-odometry-publisher-system"
              name="ignition::gazebo::systems::OdometryPublisher">
        <dimensions>3</dimensions>
      </plugin>
    </model>"""


def world_sdf(p: dict, seed: int | None = None) -> str:
    ep = episode(p, seed)
    x0, x1, y0, y1 = p['ao']
    m = []
    for name, pose, size in (('ao_x0', ((x0 + x1) / 2, y0, 0.02), (x1 - x0, 0.8, 0.04)),
                             ('ao_x1', ((x0 + x1) / 2, y1, 0.02), (x1 - x0, 0.8, 0.04)),
                             ('ao_y0', (x0, (y0 + y1) / 2, 0.02), (0.8, y1 - y0, 0.04)),
                             ('ao_y1', (x1, (y0 + y1) / 2, 0.02), (0.8, y1 - y0, 0.04))):
        m.append(_box(name, pose, size, (1, 1, 1), collide=False))
    for i, (_, sy0, _, _) in enumerate(sectors(p)[1:], start=1):
        m.append(_box(f'sector_{i}', ((x0 + x1) / 2, sy0, 0.02), (x1 - x0, 0.4, 0.03),
                      (0.95, 0.95, 0.3), collide=False))
    m.append(_cyl('base_pad', (0, 0, 0.01), 1 + p['pad_spacing'] * int(p['num_drones']) / 2, 0.02,
                  (0.3, 0.3, 0.35), collide=False))
    for i, (ox, oy, r, h) in enumerate(obstacles(p)):
        m.append(_cyl(f'obstacle_{i}', (ox, oy, h / 2), r, h, (0.55, 0.55, 0.55)))
    for i, (nx, ny, r) in enumerate(nfzs(p)):
        m.append(_cyl(f'nfz_{i}', (nx, ny, 40), r, 80, (0.9, 0.1, 0.1), alpha=0.25, collide=False))
    for tid, kind, tx, ty in ep['targets']:
        size, rgb = TARGET_LOOK[kind]
        m.append(_box(f'target_{tid}', (tx, ty, size[2] / 2), size, rgb, collide=False))
    for k, name in enumerate(drones(p)):
        m.append(x3(name, *pad(p, k), DRONE_RGB[k % len(DRONE_RGB)]))
    cx = (x0 + x1) / 2
    cam_z = 0.45 * (y1 - y0)            # whole AO in view, aimed at its centre at 0.6 rad pitch
    cam_x = cx - cam_z / math.tan(0.6)
    # Fortress' MinimalScene renders only; these invisible plugins sync the scene, give the mouse
    # orbit/pan/zoom and serve /gui/follow (GzScene3D, which bundled them, is deprecated in v6)
    helpers = ''.join(
        f'<plugin filename="{f}" name="{f}"><ignition-gui>'
        '<property key="resizable" type="bool">false</property>'
        '<property key="width" type="double">5</property>'
        '<property key="height" type="double">5</property>'
        '<property key="state" type="string">floating</property>'
        '<property key="showTitleBar" type="bool">false</property></ignition-gui></plugin>'
        for f in ('GzSceneManager', 'InteractiveViewControl', 'CameraTracking',
                  'SelectEntities', 'VisualizationCapabilities'))
    # the ground's visual in 100 m tiles (verified with ogre2 on llvmpipe; ogre1 there drops whole
    # visuals, so the GUI uses ogre2)
    ground = ''.join(
        f'<visual name="g{i}_{j}"><pose>{-150 + 100 * i} {-450 + 100 * j} -0.05 0 0 0</pose>'
        '<geometry><box><size>100 100 0.1</size></box></geometry><material>'
        '<ambient>0.45 0.55 0.4 1</ambient><diffuse>0.45 0.55 0.4 1</diffuse></material></visual>'
        for i in range(8) for j in range(10))
    return f"""<?xml version="1.0"?>
<!-- generated by drone_swarm.scenario.world_sdf (seed {p['seed'] if seed is None else seed}) -->
<sdf version="1.9">
  <world name="swarm">
    <physics name="step" type="ignored"><max_step_size>{p['physics_step_s']}</max_step_size>
      <real_time_factor>{p.get('real_time_factor', 1.0)}</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui><title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">docked</property></ignition-gui>
        <engine>ogre2</engine><scene>scene</scene><ambient_light>0.5 0.5 0.5</ambient_light>
        <background_color>0.75 0.85 0.95</background_color>
        <camera_pose>{cam_x:.0f} 0 {cam_z:.0f} 0 0.6 0</camera_pose>
        <camera_clip><near>0.25</near><far>5000</far></camera_clip>
      </plugin>
      {helpers}
      <!-- overlays need a size: unsized, they cover the 3D view and swallow every mouse event -->
      <plugin filename="WorldControl" name="World control">
        <ignition-gui><title>World control</title>
          <property type="double" key="width">72</property>
          <property type="double" key="height">121</property>
          <property type="bool" key="resizable">false</property>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="left" target="left"/>
            <line own="bottom" target="bottom"/>
          </anchors></ignition-gui>
        <play_pause>true</play_pause><step>true</step><start_paused>false</start_paused>
      </plugin>
      <plugin filename="WorldStats" name="World stats">
        <ignition-gui><title>World stats</title>
          <property type="double" key="width">290</property>
          <property type="double" key="height">110</property>
          <property type="bool" key="resizable">false</property>
          <property type="bool" key="showTitleBar">false</property>
          <property type="string" key="state">floating</property>
          <anchors target="3D View"><line own="right" target="right"/>
            <line own="bottom" target="bottom"/>
          </anchors></ignition-gui>
        <sim_time>true</sim_time><real_time>true</real_time><real_time_factor>true</real_time_factor>
      </plugin>
    </gui>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows>
      <pose>{cx} 0 200 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>1000 1000</size>
        </plane></geometry></collision>
      {ground}</link></model>
    {''.join(m)}
  </world>
</sdf>
"""


def bridge_config(p: dict) -> list[dict]:
    """ros_gz_bridge entries: /clock, and per drone odometry in, velocity command + enable out."""
    out = [{'ros_topic_name': '/clock', 'gz_topic_name': '/clock',
            'ros_type_name': 'rosgraph_msgs/msg/Clock', 'gz_type_name': 'ignition.msgs.Clock',
            'direction': 'GZ_TO_ROS'}]
    for d in drones(p):
        out += [{'ros_topic_name': f'/{d}/odom', 'gz_topic_name': f'/model/{d}/odometry',
                 'ros_type_name': 'nav_msgs/msg/Odometry',
                 'gz_type_name': 'ignition.msgs.Odometry',
                 'direction': 'GZ_TO_ROS'},
                {'ros_topic_name': f'/{d}/gazebo/command/twist',
                 'gz_topic_name': f'/{d}/gazebo/command/twist',
                 'ros_type_name': 'geometry_msgs/msg/Twist', 'gz_type_name': 'ignition.msgs.Twist',
                 'direction': 'ROS_TO_GZ'},
                {'ros_topic_name': f'/{d}/enable', 'gz_topic_name': f'/{d}/enable',
                 'ros_type_name': 'std_msgs/msg/Bool', 'gz_type_name': 'ignition.msgs.Boolean',
                 'direction': 'ROS_TO_GZ'}]
    return out


def main(argv=None):
    """``swarm_world <scenario.yaml> <out.sdf> [num_drones] [seed]``: write a world (debug)."""
    import sys
    a = list(sys.argv[1:] if argv is None else argv)
    p = load(a[0], num_drones=int(a[2]) if len(a) > 2 else None,
             seed=int(a[3]) if len(a) > 3 else None)
    Path(a[1]).write_text(world_sdf(p))
    print(f'{a[1]}: {len(drones(p))} drones, targets {episode(p)["targets"]}')
