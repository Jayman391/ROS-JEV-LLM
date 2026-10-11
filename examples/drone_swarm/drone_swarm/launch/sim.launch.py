"""Drone swarm simulator (B1): generate the world for (num_drones, seed), run the Gazebo Fortress
server (+ GUI) and ros_gz_bridge (/clock, /uavK/odom, /uavK/gazebo/command/twist, /uavK/enable).

    ros2 launch drone_swarm sim.launch.py num_drones:=4 seed:=0 gui:=true

Included by swarm.launch.py; compose and k8s run it alone as the sim container (Gazebo and the
bridge share one network namespace, so ign-transport discovery stays local).
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node

from drone_swarm import scenario

SCENARIO = get_package_share_directory('drone_swarm') + '/config/scenario.yaml'


def sim(context):
    n, seed = int(Arg('num_drones').perform(context)), int(Arg('seed').perform(context))
    p = scenario.load(SCENARIO, num_drones=n, seed=seed)
    out = Path(Arg('out_dir').perform(context))
    out.mkdir(parents=True, exist_ok=True)
    world = out / f'world_n{n}_s{seed}.sdf'
    world.write_text(scenario.world_sdf(p))
    bridge = out / f'bridge_n{n}.yaml'
    bridge.write_text(yaml.safe_dump(scenario.bridge_config(p)))
    nodes = [ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '1', str(world)],
                            output='log', name='gazebo'),
             Node(package='ros_gz_bridge', executable='parameter_bridge', name='bridge',
                  output='log', parameters=[{'config_file': str(bridge)}])]
    if Arg('gui').perform(context) == 'true':
        nodes.append(ExecuteProcess(cmd=['ign', 'gazebo', '-g', '-v', '1'], output='log',
                                    name='gazebo_gui'))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('num_drones', default_value='4'),
        DeclareLaunchArgument('seed', default_value='0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/drone_swarm'),
        OpaqueFunction(function=sim),
    ])
