"""Healthy reference launch. Derive optional sensor world in a private temp directory."""
import atexit
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    def arg(name): return LaunchConfiguration(name).perform(context)
    sim = Path(get_package_share_directory('ares_simulation'))
    loc = Path(get_package_share_directory('ares_localization'))
    desc = Path(get_package_share_directory('ares_jackal_description'))
    temp = tempfile.TemporaryDirectory(prefix='ares_baseline_')
    atexit.register(temp.cleanup)
    directory = Path(temp.name)
    camera = arg('camera').lower() == 'true'
    stage = arg('stage')
    if stage not in (
            'gazebo', 'bridge', 'sensors', 'ekf', 'localization', 'nav2',
            'nav2_week6_upstream'):
        raise ValueError('Unknown baseline stage: ' + stage)
    world_tree = ET.parse(sim / 'worlds/ares_test_world.sdf')
    world = world_tree.getroot().find('world')
    for actor in world.findall('actor'): world.remove(actor)
    include = world.find('include')
    model = ET.parse(sim / 'models/ares_jackal/model.sdf').getroot().find('model')
    model.find('pose').text = include.findtext('pose')
    ground_truth = arg('ground_truth').lower() == 'true'
    if ground_truth:
        plugin = ET.SubElement(model, 'plugin', {
            'filename': 'gz-sim-odometry-publisher-system',
            'name': 'gz::sim::systems::OdometryPublisher',
        })
        ET.SubElement(plugin, 'odom_frame').text = 'world'
        ET.SubElement(plugin, 'robot_base_frame').text = 'ares_jackal'
        ET.SubElement(plugin, 'odom_publish_frequency').text = '20'
    world.remove(include)
    if not camera:
        link = model.find("link[@name='camera_link']")
        for sensor in link.findall('sensor'): link.remove(sensor)
    else:
        model.find(".//sensor[@type='depth_camera']/always_on").text = 'true'
    world.append(model)
    world_tree.write(directory / 'world.sdf', encoding='unicode')
    bridge = yaml.safe_load((sim / 'config/bridge_baseline.yaml').read_text())
    clock_ros_topic = arg('clock_ros_topic')
    for mapping in bridge:
        if mapping.get('gz_topic_name') == '/clock':
            mapping['ros_topic_name'] = clock_ros_topic
    if ground_truth:
        bridge.append({
            'ros_topic_name': '/ares/ground_truth',
            'gz_topic_name': '/model/ares_jackal/odometry',
            'ros_type_name': 'nav_msgs/msg/Odometry',
            'gz_type_name': 'gz.msgs.Odometry',
            'direction': 'GZ_TO_ROS',
        })
    if stage == 'bridge':
        bridge = [
            mapping for mapping in bridge
            if (mapping.get('gz_topic_name') == '/clock' or
                mapping['ros_topic_name'] == '/ares/cmd_vel')
        ]
    if camera:
        bridge += yaml.safe_load((sim / 'config/bridge_camera.yaml').read_text())
    (directory / 'bridge.yaml').write_text(yaml.safe_dump(bridge))
    gz = ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', '--seed', arg('seed'), str(directory / 'world.sdf')], output='screen')
    actions = [gz, RegisterEventHandler(OnProcessExit(target_action=gz, on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))]))]
    def node(package, executable, name=None, parameters=None, delay=None,
             additional_env=None):
        n = Node(package=package, executable=executable, name=name,
                 parameters=(parameters or []) + [{'use_sim_time': True}],
                 output='screen', additional_env=additional_env or {})
        handler = RegisterEventHandler(OnProcessExit(
            target_action=n,
            on_exit=[EmitEvent(event=Shutdown(reason=executable + ' exited'))]))
        if delay is None:
            actions.extend([n, handler])
        else:
            # Let all managed nodes create their lifecycle services before
            # autostart begins. This avoids an observed WSL/DDS startup race.
            actions.extend([handler, TimerAction(period=delay, actions=[n])])
    if stage == 'gazebo': return actions
    bridge_env = {}
    if arg('bridge_keep_rmw_loaded').lower() == 'true':
        bridge_env['LD_PRELOAD'] = '/opt/ros/lyrical/lib/librmw_fastrtps_cpp.so'
    node('ros_gz_bridge', 'parameter_bridge', 'ros_gz_bridge',
         [{'config_file': str(directory / 'bridge.yaml')}],
         additional_env=bridge_env)
    if stage in ('bridge', 'sensors'): return actions
    robot = xacro.process_file(str(desc / 'urdf/jackal.urdf.xacro')).toxml()
    node('robot_state_publisher', 'robot_state_publisher', 'robot_state_publisher', [{'robot_description': robot}])
    node('robot_localization', 'ekf_node', 'ekf_filter_node', [str(loc / 'config/ekf_baseline.yaml')])
    if stage == 'ekf': return actions
    params = arg('nav2_params')
    node('nav2_map_server', 'map_server', 'map_server', [params, {'yaml_filename': arg('map')}])
    node('nav2_amcl', 'amcl', 'amcl', [params, {'set_initial_pose': True, 'initial_pose.x': 0.0, 'initial_pose.y': 0.0, 'initial_pose.yaw': 0.0}])
    names = ['map_server', 'amcl']
    if stage in ('nav2', 'nav2_week6_upstream'):
        nodes = [
            ('nav2_planner', 'planner_server'),
            ('nav2_controller', 'controller_server'),
            ('nav2_behaviors', 'behavior_server'),
            ('nav2_bt_navigator', 'bt_navigator'),
        ]
        for package, executable in nodes:
            node(package, executable, executable, [params])
            # Week 6 owns BT activation through its endpoint barrier and
            # separate lifecycle manager. The process must already exist so
            # the barrier can verify its lifecycle service and endpoints.
            if stage == 'nav2' or executable != 'bt_navigator':
                names.append(executable)
        node('ares_localization', 'cmd_vel_adapter.py', 'cmd_vel_adapter')
    # The Week 6 parent launch owns these transitions with response-loss-safe
    # orchestration. All production stages retain the stock lifecycle manager.
    if stage != 'nav2_week6_upstream':
        node('nav2_lifecycle_manager', 'lifecycle_manager',
             'lifecycle_manager_baseline',
             [{'autostart': True, 'node_names': names,
               'bond_timeout': 10.0}],
             delay=2.0)
    if arg('rviz').lower() == 'true':
        actions.append(Node(package='rviz2', executable='rviz2', parameters=[{'use_sim_time': True}], output='screen'))
    return actions


def generate_launch_description():
    sim = Path(get_package_share_directory('ares_simulation'))
    loc = Path(get_package_share_directory('ares_localization'))
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('camera', default_value='false'),
        # Opt-in measurement-only truth stream for Week 6. Default production
        # behavior and its topic graph remain unchanged.
        DeclareLaunchArgument('ground_truth', default_value='false'),
        DeclareLaunchArgument('seed', default_value='42'),
        DeclareLaunchArgument('stage', default_value='nav2'),
        DeclareLaunchArgument('bridge_keep_rmw_loaded', default_value='false'),
        # Default behavior is unchanged. Week 6 alone redirects the Gazebo
        # clock into its private bounded-clock input topic.
        DeclareLaunchArgument('clock_ros_topic', default_value='/clock'),
        DeclareLaunchArgument('map', default_value=str(sim / 'maps/ares_warehouse.yaml')),
        # Optional, backward-compatible override used by the isolated Week 6
        # campaign. Production and Week 4/5 retain this unchanged default.
        DeclareLaunchArgument(
            'nav2_params',
            default_value=str(loc / 'config/nav2_params.yaml')),
        OpaqueFunction(function=setup),
    ])
