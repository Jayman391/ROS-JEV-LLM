"""The Gazebo Fortress puppet view of the line: the world generated from the scenario, the Gazebo
server (+ GUI) and line_gz, which mirrors line_sim's pallets and arms into it. Display only.

    ros2 launch assembly_line gazebo.launch.py              # beside a running line
    ros2 launch assembly_line assembly_line.launch.py gazebo:=true

line_gz talks ign-transport, so it runs beside the Gazebo server (one container in compose).
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node

from assembly_line import gz_world, scenario

SCENARIO = get_package_share_directory('assembly_line') + '/config/scenario.yaml'


def gazebo(context):
    p = scenario.load(SCENARIO)
    out = Path(Arg('out_dir').perform(context))
    out.mkdir(parents=True, exist_ok=True)
    world = out / 'line_world.sdf'
    world.write_text(gz_world.world_sdf(p))
    poses = {f'pose_{k}': v for k, v in gz_world.poses().items()}
    nodes = [ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', '-v', '1', str(world)],
                            output='log', name='gazebo'),
             Node(package='line_gz', executable='line_gz', output='screen', parameters=[{
                 'arms': list(p['arms']), 'z_offset': gz_world.Z0,
                 'pallet_pool': gz_world.PALLET_POOL, 'park_z': gz_world.PARK[2], **poses}])]
    if Arg('gui').perform(context) == 'true':
        # xcb: without it Qt may hand Ogre2 a non-X window on Xvfb ("Invalid parentWindowHandle")
        nodes.append(ExecuteProcess(cmd=['ign', 'gazebo', '-g', '-v', '1'], output='log',
                                    name='gazebo_gui', additional_env={'QT_QPA_PLATFORM': 'xcb'}))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
                              description='Gazebo GUI (needs a display: noVNC in the image)'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/assembly_line'),
        OpaqueFunction(function=gazebo),
    ])
