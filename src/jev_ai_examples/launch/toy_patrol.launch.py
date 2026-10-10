"""toy_patrol: N simulated patrol robots deciding with jev_ai (keyless by default).

    ros2 launch jev_ai_examples toy_patrol.launch.py                    # 3 robots, mock Jev + LLM
    ros2 launch jev_ai_examples toy_patrol.launch.py agents:=5 llm_backend:=anthropic
    ros2 launch jev_ai_examples toy_patrol.launch.py agents:=1 cpp_agents:=2   # + C++ agents
    ros2 launch jev_ai_examples toy_patrol.launch.py supervisor:=true    # + A7 supervisor
    ros2 launch jev_ai_examples toy_patrol.launch.py agents:=0 bt_agents:=1   # A8 BT toy
    ros2 launch jev_ai_examples toy_patrol.launch.py catalog:=calibrated.yaml   # A10 patch
    # dashboard: http://localhost:8080   records: /tmp/jev_ai/toy_patrol.jsonl (+ -labels.jsonl)

Python agents host their engine in-process; C++ agents (jev_ai_cpp toy_agent) and BT agents
(jev_ai_bt bt_toy) reach the same engine through decide_server, which starts when they exist.
"""
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node

BRINGUP = get_package_share_directory('jev_ai_bringup')
EXAMPLES = get_package_share_directory('jev_ai_examples')
CATALOG = EXAMPLES + '/config/toy_patrol.yaml'


def agents(context):
    n = int(Arg('agents').perform(context))
    # one run_id per launch: agents share group noise only when they share it (§6.3)
    run_id = Arg('run_id').perform(context) or uuid.uuid4().hex[:12]
    n_cpp = int(Arg('cpp_agents').perform(context))
    n_bt = int(Arg('bt_agents').perform(context))
    catalog = Arg('catalog').perform(context) or CATALOG
    nodes = [Node(package='jev_ai_examples', executable='toy_patrol', name=f'patrol_{i}',
                  output='screen', parameters=[{'seed': i, 'run_id': run_id,
                                                'catalog': catalog}])
             for i in range(1, n + 1)]
    nodes += [Node(package='jev_ai_cpp', executable='toy_agent', name=f'cpp_patrol_{i}',
                   output='screen', parameters=[{'seed': 100 + i}])
              for i in range(1, n_cpp + 1)]
    nodes += [Node(package='jev_ai_bt', executable='bt_toy', name=f'bt_patrol_{i}',
                   output='screen', parameters=[{'seed': 200 + i}])
              for i in range(1, n_bt + 1)]
    if Arg('supervisor').perform(context).lower() == 'true':
        nodes.append(Node(package='jev_ai_examples', executable='toy_supervisor',
                          name='supervisor', output='screen',
                          parameters=[{'run_id': run_id}]))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(BRINGUP + '/launch/bringup.launch.py'),
        launch_arguments={'jev_backend': Arg('jev_backend'),
                          'llm_backend': Arg('llm_backend'),
                          'dashboard_port': Arg('dashboard_port'),
                          'catalog': catalog if n_cpp or n_bt else '',
                          'run_id': run_id,
                          'params_file': EXAMPLES + '/config/toy_params.yaml'}.items())] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('agents', default_value='3'),
        DeclareLaunchArgument('cpp_agents', default_value='0'),
        DeclareLaunchArgument('bt_agents', default_value='0'),
        DeclareLaunchArgument('supervisor', default_value='false'),
        DeclareLaunchArgument('run_id', default_value='', description='"" = a fresh one'),
        DeclareLaunchArgument('catalog', default_value='',
                              description='"" = config/toy_patrol.yaml (e.g. a calibrated one)'),
        DeclareLaunchArgument('jev_backend', default_value='mock'),
        DeclareLaunchArgument('llm_backend', default_value='mock'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        OpaqueFunction(function=agents),
    ])
