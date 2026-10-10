"""turtlesim example (A9): three turtles, one per jev_ai flavour, keyless by default.

    ros2 launch jev_ai_examples turtlesim.launch.py
    ros2 launch jev_ai_examples turtlesim.launch.py jev_backend:=typesafe llm_backend:=anthropic
    ros2 topic pub --once /alerts geometry_msgs/msg/Point "{x: 8.0, y: 3.0}"    # inject an alert
    # turtles: http://localhost:6080/vnc.html   decisions: http://localhost:8080

turtle1 runs the Python engine in-process (red), turtle2 is a C++ DecisionClient (green),
turtle3 a behaviour tree (blue); the last two decide through decide_server. All share one run_id
and the coupling group ``turtles``.
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
CATALOG = EXAMPLES + '/config/turtle_patrol.yaml'


def turtles(context):
    run_id = Arg('run_id').perform(context) or uuid.uuid4().hex[:12]
    flavours = [f for f, n in (('py', Arg('py_turtles')), ('cpp', Arg('cpp_turtles')),
                               ('bt', Arg('bt_turtles')))
                for _ in range(int(n.perform(context)))]
    names = [f'turtle{i}' for i in range(1, len(flavours) + 1)]
    common = {'turtles': names, 'run_id': run_id}
    nodes = [Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
             Node(package='jev_ai_examples', executable='turtle_arena', name='turtle_arena',
                  output='screen', parameters=[{'turtles': names,
                                                'auto_alerts': Arg('auto_alerts')}])]
    for name, flavour in zip(names, flavours):
        if flavour == 'py':
            nodes.append(Node(package='jev_ai_examples', executable='turtle_agent',
                              name=f'{name}_agent', output='screen',
                              parameters=[common, {'turtle': name}]))
        else:
            nodes.append(Node(package='jev_ai_bt', executable='turtle_agent',
                              name=f'{name}_agent', output='screen',
                              parameters=[common, {'turtle': name, 'mode':
                                                   'client' if flavour == 'cpp' else 'bt'}]))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(BRINGUP + '/launch/bringup.launch.py'),
        launch_arguments={'jev_backend': Arg('jev_backend'), 'llm_backend': Arg('llm_backend'),
                          'catalog': CATALOG if set(flavours) - {'py'} else '',
                          'run_id': run_id,
                          'params_file': EXAMPLES + '/config/turtle_params.yaml'}.items())] + nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('py_turtles', default_value='1'),
        DeclareLaunchArgument('cpp_turtles', default_value='1'),
        DeclareLaunchArgument('bt_turtles', default_value='1'),
        DeclareLaunchArgument('auto_alerts', default_value='true'),
        DeclareLaunchArgument('jev_backend', default_value='mock'),
        DeclareLaunchArgument('llm_backend', default_value='mock'),
        DeclareLaunchArgument('run_id', default_value='', description='"" = a fresh one'),
        OpaqueFunction(function=turtles),
    ])
