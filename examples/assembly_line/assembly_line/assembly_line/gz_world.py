"""The Gazebo Fortress puppet view of the line: a world generated from the scenario, and the arm
poses line_gz plays. Display only: line_sim decides everything, nothing flows back from Gazebo.

The layout is line_sim's RViz layout (publish_markers): stations every DX m along x from S1, QA
last, arms 1.3 m behind the belt (0.4 m either side of their station when two share one), the
rework lane 2 m in front, pallets at marker z + Z0 (Gazebo has a floor; RViz does not).
"""
from __future__ import annotations

import math

from assembly_line import scenario

DX, ARM_Y, ARM_SIDE, LANE_Y, Z0 = 3.0, 1.3, 0.4, -2.0, 0.3
TABLE_H = 0.4                        # station tables and belt top; pallets (0.35 m) sit on it
H0, L1, L2 = 1.1, 0.9, 0.8           # arm: shoulder height, upper arm, forearm
PALLET_POOL = 16                     # pallets on the line at once never exceed this
PARK = (0.0, 0.0, -3.0)              # unused pallets wait under the floor


def arm_bases(p: dict) -> dict[str, tuple[float, float, float]]:
    """arm -> (x, y, yaw): beside its station, turned to face the pallet slot."""
    out, seen = {}, {}
    for a in scenario.arms(p):
        k = seen[a['station']] = seen.get(a['station'], -1) + 1
        xs = DX * a['station']
        x = xs + (-ARM_SIDE if k == 0 else ARM_SIDE)
        out[a['name']] = (x, ARM_Y, math.atan2(0.0 - ARM_Y, xs - x))
    return out


def reach(d: float, dz: float) -> tuple[float, float]:
    """Shoulder and elbow angles (from vertical, toward the slot) putting the forearm tip ``d``
    out and ``dz`` up from the shoulder; elbow up."""
    r2 = d * d + dz * dz
    q2 = math.acos(max(-1.0, min(1.0, (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2))))
    q1 = math.atan2(d, dz) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2


def poses() -> dict[str, list[float]]:
    """Joint targets [yaw, shoulder, elbow] line_gz plays per arm state; every arm faces its slot,
    so one set serves all."""
    q1, q2 = reach(math.hypot(ARM_SIDE, ARM_Y), Z0 + 0.3 + 0.175 + 0.07 - H0)  # tool on the pallet
    return {'home': [0.0, 0.25, 1.3], 'reach': [0.0, q1, q2], 'lift': [0.0, q1 - 0.35, q2],
            'tool': [math.pi, 0.6, 1.4]}


def _mat(rgb, emissive=False) -> str:
    c = f'{rgb[0]} {rgb[1]} {rgb[2]} 1'
    return (f'<material><ambient>{c}</ambient><diffuse>{c}</diffuse>'
            + (f'<emissive>{c}</emissive>' if emissive else '') + '</material>')


# visual_config finds a visual by its parent's plain (unscoped) name, so every link line_gz
# recolours has a world-unique name: pallet_<i>_link, <arm>_column

def _box(name, pose, size, rgb, static=True) -> str:
    x, y, z = pose
    return (f'<model name="{name}"><static>{str(static).lower()}</static>'
            f'<pose>{x} {y} {z} 0 0 0</pose><link name="{name}_link"><visual name="v"><geometry><box>'
            f'<size>{size[0]} {size[1]} {size[2]}</size></box></geometry>{_mat(rgb)}</visual>'
            '</link></model>')


def _link(name, pose_z, visuals) -> str:
    return (f'<link name="{name}"><pose>0 0 {pose_z} 0 0 0</pose><gravity>false</gravity>'
            '<inertial><mass>1</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz>'
            f'</inertia></inertial>{visuals}</link>')


def _vis(name, z, geo, rgb) -> str:
    return f'<visual name="{name}"><pose>0 0 {z} 0 0 0</pose><geometry>{geo}</geometry>{_mat(rgb)}</visual>'


def arm_sdf(name: str, x: float, y: float, yaw: float) -> str:
    """Column (its visual is the state lamp line_gz recolours), then yaw j0, shoulder j1 and
    elbow j2, each a JointPositionController on /model/<arm>/joint/<j>/0/cmd_pos. Velocity
    commands track line_gz's rate-limited targets exactly: no gains to tune."""
    grey, dark = (0.75, 0.75, 0.78), (0.2, 0.2, 0.22)
    links = (
        _link(f'{name}_column', 0, _vis('v', (H0 - 0.1) / 2, f'<box><size>0.35 0.35 {H0 - 0.1}</size></box>',
                              (0.6, 0.6, 0.6)))
        + _link('turret', H0 - 0.1, _vis('v', 0.05, '<cylinder><radius>0.16</radius><length>0.1'
                                         '</length></cylinder>', dark))
        + _link('upper', H0, _vis('v', L1 / 2, f'<box><size>0.12 0.12 {L1}</size></box>', grey))
        + _link('fore', H0 + L1, _vis('v', L2 / 2, f'<box><size>0.09 0.09 {L2}</size></box>', grey)
                + _vis('tool', L2, '<box><size>0.14 0.14 0.1</size></box>', dark)))
    joints = (f'<joint name="fix" type="fixed"><parent>world</parent><child>{name}_column</child>'
              '</joint>'
              + ''.join(f'<joint name="{j}" type="revolute"><parent>{par}</parent><child>{ch}'
                        f'</child><axis><xyz>{ax}</xyz><limit><lower>-4</lower><upper>4</upper>'
                        '</limit></axis></joint>'
                        for j, par, ch, ax in (('j0', f'{name}_column', 'turret', '0 0 1'),
                                               ('j1', 'turret', 'upper', '0 1 0'),
                                               ('j2', 'upper', 'fore', '0 1 0'))))
    ctl = ''.join('<plugin filename="ignition-gazebo-joint-position-controller-system" '
                  'name="ignition::gazebo::systems::JointPositionController">'
                  f'<joint_name>{j}</joint_name><use_velocity_commands>true</use_velocity_commands>'
                  '</plugin>' for j in ('j0', 'j1', 'j2'))
    return (f'<model name="{name}"><pose>{x:.3f} {y:.3f} 0 0 0 {yaw:.4f}</pose>{links}{joints}'
            f'{ctl}</model>')


GUI_HELPERS = ('GzSceneManager', 'InteractiveViewControl', 'CameraTracking', 'SelectEntities',
               'VisualizationCapabilities')


def world_sdf(p: dict) -> str:
    """The world for scenario ``p``. The GUI block is the drone swarm's (ogre2 on llvmpipe; the
    invisible helper plugins MinimalScene needs in Fortress; sized overlays)."""
    qa = int(p['n_stations']) + 1
    length = DX * (qa + 1)
    m = [_box('belt', (length / 2, 0, TABLE_H / 2 - 0.01), (length, 0.6, TABLE_H - 0.02),
              (0.2, 0.2, 0.2)),
         _box('lane', (length / 2, LANE_Y, TABLE_H / 2), (length, 0.4, TABLE_H), (0.35, 0.25, 0.1)),
         _box('feeder', (0, 0, TABLE_H / 2), (1.0, 1.4, TABLE_H), (0.45, 0.45, 0.5))]
    for s in range(1, qa + 1):
        m.append(_box(f'table_{s}', (DX * s, 0, TABLE_H / 2), (1.4, 1.4, TABLE_H),
                      (0.1, 0.6, 0.6) if s == qa else (0.5, 0.5, 0.5)))
    # QA: an inspection arch over the slot
    for side in (-0.75, 0.75):
        m.append(_box(f'qa_post_{"lr"[side > 0]}', (DX * qa, side, 0.9), (0.08, 0.08, 1.8),
                      (0.1, 0.5, 0.5)))
    m.append(_box('qa_bar', (DX * qa, 0, 1.8), (0.25, 1.6, 0.12), (0.1, 0.5, 0.5)))
    for name, (x, y, yaw) in arm_bases(p).items():
        m.append(arm_sdf(name, x, y, yaw))
    for i in range(PALLET_POOL):
        m.append(_box(f'pallet_{i}', PARK, (0.45, 0.45, 0.35), (0.8, 0.8, 0.8)))
    helpers = ''.join(
        f'<plugin filename="{f}" name="{f}"><ignition-gui>'
        '<property key="resizable" type="bool">false</property>'
        '<property key="width" type="double">5</property>'
        '<property key="height" type="double">5</property>'
        '<property key="state" type="string">floating</property>'
        '<property key="showTitleBar" type="bool">false</property></ignition-gui></plugin>'
        for f in GUI_HELPERS)
    cx = length / 2
    return f"""<?xml version="1.0"?>
<!-- generated by assembly_line.gz_world.world_sdf -->
<sdf version="1.9">
  <world name="line">
    <physics name="step" type="ignored"><max_step_size>0.005</max_step_size>
      <real_time_factor>1</real_time_factor></physics>
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
        <camera_pose>{cx:.1f} -8 5.5 0 0.55 1.5708</camera_pose>
        <camera_clip><near>0.1</near><far>500</far></camera_clip>
      </plugin>
      {helpers}
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
      <pose>{cx} 0 20 0 0 0</pose><diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.4 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><link name="link">
      <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>200 200</size>
        </plane></geometry></collision>
      <visual name="v"><pose>{cx} 0 -0.05 0 0 0</pose><geometry><box><size>40 20 0.1</size></box>
        </geometry>{_mat((0.82, 0.82, 0.8))}</visual></link></model>
    {''.join(m)}
  </world>
</sdf>
"""


def main(argv=None):
    """``line_world <scenario.yaml> <out.sdf>``: write the world (debug)."""
    import sys
    a = list(sys.argv[1:] if argv is None else argv)
    open(a[1], 'w').write(world_sdf(scenario.load(a[0])))
