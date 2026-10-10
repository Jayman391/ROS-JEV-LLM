"""jev_ai bringup (§0): jev_server + llm_server + decision_logger + dashboard
(+ decide_server with catalog:=<yaml>, for C++/BT clients).

    export TYPESAFE_API_KEY=... ANTHROPIC_API_KEY=...
    ros2 launch jev_ai_bringup bringup.launch.py
    ros2 launch jev_ai_bringup bringup.launch.py jev_backend:=mock llm_backend:=mock   # keyless

Live backends without a key fail at configure time (no silent mock). The dashboard serves
http://localhost:<dashboard_port>.
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, PythonExpression
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node

PARAMS = get_package_share_directory('jev_ai_bringup') + '/config/params.yaml'


def generate_launch_description():
    params = Arg('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=PARAMS),
        DeclareLaunchArgument('jev_backend', default_value='typesafe',
                              description='typesafe | llama_ros | mock | replay'),
        DeclareLaunchArgument('llm_backend', default_value='anthropic',
                              description='anthropic | openai_compat | mock | replay'),
        DeclareLaunchArgument('catalog', default_value='',
                              description='catalog YAML for decide_server ("" = no decide_server)'),
        DeclareLaunchArgument('logger', default_value='true'),
        DeclareLaunchArgument('dashboard', default_value='true'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        # no auth: loopback unless told otherwise (compose sets 0.0.0.0 behind a loopback port)
        DeclareLaunchArgument('dashboard_host', default_value=EnvironmentVariable(
            'JEV_AI_DASHBOARD_HOST', default_value='127.0.0.1')),
        Node(package='jev_ai_servers', executable='jev_server', name='jev_server', output='screen',
             parameters=[params, {'backend': Arg('jev_backend')}]),
        Node(package='jev_ai_servers', executable='llm_server', name='llm_server', output='screen',
             parameters=[params, {'backend': Arg('llm_backend')}]),
        # decide_server hosts engines for C++/BT/CLI clients; only when given a catalog
        Node(package='jev_ai_servers', executable='decide_server', name='decide_server',
             output='screen', parameters=[{'catalog': Arg('catalog')}],
             condition=IfCondition(PythonExpression(["'", Arg('catalog'), "' != ''"]))),
        Node(package='jev_ai_servers', executable='decision_logger', name='decision_logger',
             output='screen', parameters=[params], condition=IfCondition(Arg('logger'))),
        Node(package='jev_ai_servers', executable='dashboard', name='dashboard', output='screen',
             parameters=[params, {'port': Arg('dashboard_port'), 'host': Arg('dashboard_host')}],
             condition=IfCondition(Arg('dashboard'))),
    ])
