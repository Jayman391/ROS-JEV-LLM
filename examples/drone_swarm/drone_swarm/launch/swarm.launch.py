"""Drone swarm reference example (Part B): Gazebo Fortress + N X3 drones + jev_ai.

    ros2 launch drone_swarm swarm.launch.py                          # keyless mocks, headless
    ros2 launch drone_swarm swarm.launch.py gui:=true                # Gazebo at :6080/vnc.html
    ros2 launch drone_swarm swarm.launch.py jev_backend:=typesafe llm_backend:=anthropic
    ros2 launch drone_swarm swarm.launch.py num_drones:=6 seed:=3 t_max_s:=1200
    # decisions + swarm status: http://localhost:8080      mission: auto-starts (auto_start:=false
    # and `ros2 action send_goal /coordinator/mission swarm_interfaces/action/Mission {}` instead)

sim.launch.py generates the world for (num_drones, seed) under out_dir and runs the Gazebo server
(and optionally the GUI) with ros_gz_bridge; then per drone flight_controller + sensor_sim (C++)
+ drone_agent,
and environment, comms_sim, allocator, mission_coordinator, mission_scorer, plus jev_ai
(jev_server, llm_server for the drones, /coordinator/llm_server, decision_logger, dashboard).
Everything shares one run_id, so drones answering the same directive share their noise.
"""
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node

from drone_swarm import scenario

SHARE = get_package_share_directory('drone_swarm')
SCENARIO = SHARE + '/config/scenario.yaml'
PARAMS = SHARE + '/config/swarm_params.yaml'
BRINGUP = get_package_share_directory('jev_ai_bringup') + '/launch/bringup.launch.py'
TO_ROOT = [('decisions', '/decisions')]          # engines publish records where the logger listens


def swarm(context):
    keys = ('num_drones', 'seed', 't_max_s', 'out_dir', 'gui', 'run_id', 'jev_backend',
            'llm_backend', 'auto_start', 'dashboard_port', 'drone_catalog', 'coordinator_catalog')
    a = {k: Arg(k).perform(context) for k in keys}
    seed = int(a['seed'])
    p = scenario.load(SCENARIO, num_drones=int(a['num_drones']) if a['num_drones'] else None,
                      seed=seed, t_max_s=float(a['t_max_s']) if a['t_max_s'] else None)
    n = int(p['num_drones'])
    run_id = a['run_id'] or f'swarm-s{seed}-{uuid.uuid4().hex[:6]}'
    out = Path(a['out_dir'])
    out.mkdir(parents=True, exist_ok=True)
    over = {'num_drones': n, 'seed': seed, 't_max_s': float(p['t_max_s']), 'use_sim_time': True}
    py = {'scenario_file': SCENARIO, **over}
    cat = {k: {'catalog': a[f'{k}_catalog']} if a[f'{k}_catalog'] else {}
           for k in ('drone', 'coordinator')}
    cpp = [SCENARIO, over]

    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(SHARE + '/launch/sim.launch.py'),
                                 launch_arguments={'num_drones': str(n), 'seed': str(seed),
                                                   'gui': a['gui'], 'out_dir': str(out)}.items()),
        Node(package='drone_swarm', executable='environment', parameters=[py], output='screen'),
        Node(package='swarm_control', executable='comms_sim', parameters=cpp, output='screen'),
        Node(package='swarm_control', executable='allocator', parameters=cpp,
             output='screen'),
        Node(package='drone_swarm', executable='mission_coordinator', namespace='coordinator',
             output='screen', remappings=TO_ROOT,
             parameters=[py, cat['coordinator'],
                         {'run_id': run_id, 'auto_start': a['auto_start'] == 'true'}]),
        Node(package='drone_swarm', executable='mission_scorer', output='screen',
             parameters=[py, {'run_id': run_id,
                              'summary_path': str(out / f'episode_{run_id}.json')}]),
        # the coordinator's own LLM server (Sonnet); the drones share bringup's (Haiku)
        Node(package='jev_ai_servers', executable='llm_server', name='llm_server',
             namespace='coordinator', output='screen',
             parameters=[PARAMS, {'backend': a['llm_backend']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(BRINGUP), launch_arguments={
            'jev_backend': a['jev_backend'], 'llm_backend': a['llm_backend'],
            'params_file': PARAMS, 'dashboard_port': a['dashboard_port']}.items()),
    ]
    for d in scenario.drones(p):
        nodes += [
            Node(package='swarm_control', executable='flight_controller', namespace=d,
                 parameters=cpp, output='screen'),
            Node(package='swarm_control', executable='sensor_sim', namespace=d, parameters=cpp,
                 output='screen'),
            Node(package='drone_swarm', executable='drone_agent', namespace=d, output='screen',
                 remappings=TO_ROOT, parameters=[py, cat['drone'], {'run_id': run_id}]),
        ]
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='', description='"" = scenario (8)'),
        DeclareLaunchArgument('seed', default_value='0', description='episode seed'),
        DeclareLaunchArgument('t_max_s', default_value='', description='"" = scenario (900)'),
        DeclareLaunchArgument('gui', default_value='false',
                              description='Gazebo GUI (needs a display: noVNC in the dev image)'),
        DeclareLaunchArgument('jev_backend', default_value='mock',
                              description='mock | typesafe | llama_ros | replay'),
        DeclareLaunchArgument('llm_backend', default_value='mock',
                              description='mock | anthropic | openai_compat | replay'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('run_id', default_value='', description='"" = a fresh one'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/drone_swarm'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        DeclareLaunchArgument('drone_catalog', default_value='', description='"" = shipped'),
        DeclareLaunchArgument('coordinator_catalog', default_value='', description='"" = shipped'),
        OpaqueFunction(function=swarm),
    ])
