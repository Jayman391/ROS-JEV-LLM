"""Assembly line example: line_sim + 5 arm agents + QA + supervisor + scorer + jev_ai.

    ros2 launch assembly_line assembly_line.launch.py                       # keyless mocks
    ros2 launch assembly_line assembly_line.launch.py rviz:=true            # line view
    ros2 launch assembly_line assembly_line.launch.py jev_backend:=typesafe llm_backend:=anthropic
    ros2 launch assembly_line assembly_line.launch.py seed:=3 t_max_s:=900
    # decisions + line status: http://localhost:8080

Every agent shares one run_id (arms answering one directive share their noise), publishes its
decision records on /decisions and its reports / directives on /line/reports and
/line/directives, where the dashboard's Comms section finds them.
"""
import uuid
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as Arg
from launch_ros.actions import Node

from assembly_line import scenario

SHARE = get_package_share_directory('assembly_line')
SCENARIO = SHARE + '/config/scenario.yaml'
PARAMS = SHARE + '/config/line_params.yaml'
BRINGUP = get_package_share_directory('jev_ai_bringup') + '/launch/bringup.launch.py'
REMAP = [('decisions', '/decisions'), ('reports', '/line/reports'),
         ('directives', '/line/directives')]


def line(context):
    keys = ('seed', 't_max_s', 'out_dir', 'rviz', 'run_id', 'jev_backend', 'llm_backend',
            'dashboard_port', 'supervisor_catalog', 'arm_catalog', 'qa_catalog')
    a = {k: Arg(k).perform(context) for k in keys}
    seed = int(a['seed'])
    p = scenario.load(SCENARIO, seed=seed, t_max_s=float(a['t_max_s']) if a['t_max_s'] else None)
    run_id = a['run_id'] or f'line-s{seed}-{uuid.uuid4().hex[:6]}'
    out = Path(a['out_dir'])
    out.mkdir(parents=True, exist_ok=True)
    over = {'seed': seed, 't_max_s': float(p['t_max_s'])}
    py = {'scenario_file': SCENARIO, 'use_sim_time': True, 'run_id': run_id, **over}
    cat = {k: {'catalog': a[f'{k}_catalog']} if a[f'{k}_catalog'] else {}
           for k in ('supervisor', 'arm', 'qa')}

    nodes = [
        # the world owns /clock, so it is the one node not on sim time
        Node(package='line_control', executable='line_sim', output='screen',
             parameters=[SCENARIO, over]),
        Node(package='assembly_line', executable='line_supervisor', name='line_supervisor',
             output='screen', remappings=REMAP, parameters=[py, cat['supervisor']]),
        Node(package='assembly_line', executable='qa_agent', name='qa_agent', output='screen',
             remappings=REMAP, parameters=[py, cat['qa']]),
        Node(package='assembly_line', executable='line_scorer', name='line_scorer',
             output='screen', parameters=[py, {
                 'summary_path': str(out / f'episode_{run_id}.json')}]),
        # the supervisor's own LLM server (Sonnet); arms and QA share bringup's (Haiku)
        Node(package='jev_ai_servers', executable='llm_server', name='llm_server',
             namespace='line_supervisor', output='screen',
             parameters=[PARAMS, {'backend': a['llm_backend']}]),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(BRINGUP), launch_arguments={
            'jev_backend': a['jev_backend'], 'llm_backend': a['llm_backend'],
            'params_file': PARAMS, 'dashboard_port': a['dashboard_port']}.items()),
    ]
    for arm in p['arms']:
        nodes.append(Node(package='assembly_line', executable='arm_agent', namespace=arm,
                          output='screen', remappings=REMAP, parameters=[py, cat['arm']]))
    if a['rviz'] == 'true':
        nodes.append(Node(package='rviz2', executable='rviz2', output='log',
                          arguments=['-d', SHARE + '/config/line.rviz'],
                          parameters=[{'use_sim_time': True}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('seed', default_value='0', description='episode seed'),
        DeclareLaunchArgument('t_max_s', default_value='', description='"" = scenario (1500)'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='RViz line view (needs a display: noVNC in the image)'),
        DeclareLaunchArgument('jev_backend', default_value='mock',
                              description='mock | typesafe | llama_ros | replay'),
        DeclareLaunchArgument('llm_backend', default_value='mock',
                              description='mock | anthropic | openai_compat | replay'),
        DeclareLaunchArgument('run_id', default_value='', description='"" = a fresh one'),
        DeclareLaunchArgument('out_dir', default_value='/tmp/assembly_line'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        DeclareLaunchArgument('supervisor_catalog', default_value='', description='"" = shipped'),
        DeclareLaunchArgument('arm_catalog', default_value='', description='"" = shipped'),
        DeclareLaunchArgument('qa_catalog', default_value='', description='"" = shipped'),
        OpaqueFunction(function=line),
    ])
