"""toy_patrol: N simulated patrol robots deciding with jev_ai (keyless by default).

    ros2 launch jev_ai_examples toy_patrol.launch.py                    # 3 robots, mock Jev + LLM
    ros2 launch jev_ai_examples toy_patrol.launch.py agents:=5 llm_backend:=anthropic
    # dashboard: http://localhost:8080   records: /tmp/jev_ai/toy_patrol.jsonl
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node

BRINGUP = get_package_share_directory('jev_ai_bringup')
EXAMPLES = get_package_share_directory('jev_ai_examples')


def agents(context):
    n = int(Arg('agents').perform(context))
    return [Node(package='jev_ai_examples', executable='toy_patrol', name=f'patrol_{i}',
                 output='screen', parameters=[{'seed': i}]) for i in range(1, n + 1)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agents', default_value='3'),
        DeclareLaunchArgument('jev_backend', default_value='mock'),
        DeclareLaunchArgument('llm_backend', default_value='mock'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(BRINGUP + '/launch/bringup.launch.py'),
            launch_arguments={'jev_backend': Arg('jev_backend'),
                              'llm_backend': Arg('llm_backend'),
                              'dashboard_port': Arg('dashboard_port'),
                              'params_file': EXAMPLES + '/config/toy_params.yaml'}.items()),
        OpaqueFunction(function=agents),
    ])
