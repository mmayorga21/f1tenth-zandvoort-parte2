"""Launch the two-vehicle integration scenario; controllers start disabled."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from f1tenth_gym_ros.dual_control import following_pose
import yaml


def generate_launch_description():
    controller_share = get_package_share_directory('f1tenth_controller')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            controller_share, 'config', 'two_vehicles.yaml')),
        OpaqueFunction(function=launch_nodes),
    ])


def launch_nodes(context):
    gym_share = get_package_share_directory('f1tenth_gym_ros')
    controller_share = get_package_share_directory('f1tenth_controller')
    sim_file = os.path.join(gym_share, 'config', 'sim_two_vehicles.yaml')
    with open(sim_file) as stream:
        sim = yaml.safe_load(stream)['bridge']['ros__parameters']
    with open(LaunchConfiguration('params_file').perform(context)) as stream:
        controller = yaml.safe_load(stream)['gap_follower']['ros__parameters']
    with open(os.path.join(controller_share, 'config', 'lap_tracker.yaml')) as stream:
        laps = yaml.safe_load(stream)['lap_tracker']['ros__parameters']
    actions = [IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        gym_share, 'launch', 'gym_bridge_launch.py')), launch_arguments={
            'config_file': sim_file, 'rviz': LaunchConfiguration('rviz').perform(context),
            'rviz_config': os.path.join(gym_share, 'launch', 'two_vehicles.rviz')
        }.items())]
    x, y, yaw = sim['sx'], sim['sy'], sim['stheta']
    offset = sim['opponent_start_offset_m']
    starts = [[x, y, yaw], following_pose([x, y, yaw], offset)]
    vehicles = (sim['ego_namespace'], sim['opp_namespace'])
    ratio = float(sim['opponent_speed_scale'])
    if not 0 < ratio <= 1:
        raise ValueError('opponent_speed_scale must be in (0, 1]')
    for vehicle, scale, start in zip(vehicles, (1., ratio), starts):
        actions.append(Node(
            package='f1tenth_controller', executable='gap_follower',
            name='gap_follower', namespace=vehicle, output='screen',
            parameters=[{**controller, 'vehicle_id': vehicle, 'speed_scale': scale,
                         'completed_speed_scale': ratio,
                         'yield_on_blocked_gap': True}],
            remappings=[('/scan', '/' + vehicle + '/scan'),
                        ('/drive', '/' + vehicle + '/drive'),
                        ('/sim/status', '/' + vehicle + '/sim/status'),
                        ('/controller/status', '/' + vehicle + '/controller/status'),
                        ('/controller/enable', '/' + vehicle + '/controller/enable')]))
        actions.append(Node(
            package='f1tenth_controller', executable='lap_tracker',
            name='lap_tracker', namespace=vehicle, output='screen',
            parameters=[{**laps, 'vehicle_id': vehicle, 'display': False,
                         'target_laps': sim['target_laps'], 'stop_counting_at_target': True,
                         'gate_x': start[0], 'gate_y': start[1], 'gate_yaw': start[2],
                         'event_log_path': '~/.ros/f1tenth_zandvoort/' + vehicle + '_laps.jsonl'}],
            remappings=[('/ego_racecar/odom', '/' + vehicle + '/odom'),
                        ('/sim/status', '/' + vehicle + '/sim/status'),
                        ('/laps/status', '/' + vehicle + '/laps/status'),
                        ('/laps/events', '/' + vehicle + '/laps/events')]))
    return actions
