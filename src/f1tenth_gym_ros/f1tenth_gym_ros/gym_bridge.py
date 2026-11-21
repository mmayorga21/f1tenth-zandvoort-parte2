# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as TimerClock, ClockType
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from .simulation_session import SimulationSession
from .dual_control import DualControlGate, following_pose
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gym
import numpy as np
from transforms3d import euler

class GymBridge(Node):
    def __init__(self):
        super().__init__('gym_bridge')

        self.declare_parameter('ego_namespace', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('ego_odom_topic', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('ego_opp_odom_topic', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('ego_scan_topic', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('ego_drive_topic', '')  # Tipo 'str' con valor predeterminado vacío

        self.declare_parameter('opp_namespace', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('opp_odom_topic', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('opp_ego_odom_topic', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('opp_scan_topic', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('opp_drive_topic', '')  # Tipo 'str' con valor predeterminado vacío

        self.declare_parameter('scan_distance_to_base_link', 0.0)  # Tipo 'float' con valor predeterminado
        self.declare_parameter('scan_fov', 0.0)  # Tipo 'float' con valor predeterminado
        self.declare_parameter('scan_beams', 0)  # Tipo 'int' con valor predeterminado

        self.declare_parameter('map_path', '')  # Tipo 'str' con valor predeterminado vacío
        self.declare_parameter('map_img_ext', '')  # Tipo 'str' con valor predeterminado vacío

        self.declare_parameter('num_agent', 0)  # Tipo 'int' con valor predeterminado

        self.declare_parameter('sx', 0.0)  # Tipo 'float' con valor predeterminado
        self.declare_parameter('sy', 0.0)  # Tipo 'float' con valor predeterminado
        self.declare_parameter('stheta', 0.0)  # Tipo 'float' con valor predeterminado

        self.declare_parameter('sx1', 0.0)  # Tipo 'float' con valor predeterminado
        self.declare_parameter('sy1', 0.0)  # Tipo 'float' con valor predeterminado
        self.declare_parameter('stheta1', 0.0)  # Tipo 'float' con valor predeterminado

        self.declare_parameter('kb_teleop', False)  # Tipo 'bool' con valor predeterminado
        self.declare_parameter('event_log_path', '~/.ros/f1tenth_zandvoort/events.jsonl')
        self.declare_parameter('drive_timeout_s', 0.3)
        self.declare_parameter('opponent_start_offset_m', 0.0)
        self.declare_parameter('target_laps', 10)
        self.declare_parameter('stopping_deceleration_mps2', 6.0)



        # check num_agents
        num_agents = self.get_parameter('num_agent').value
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        elif type(num_agents) != int:
            raise ValueError('num_agents should be an int.')

        # env backend
        self.env = gym.make('f110_gym:f110-v0',
                            map=self.get_parameter('map_path').value,
                            map_ext=self.get_parameter('map_img_ext').value,
                            num_agents=num_agents)

        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value
        self.ego_pose = [sx, sy, stheta]
        self.ego_speed = [0.0, 0.0, 0.0]
        self.ego_requested_speed = 0.0
        self.ego_steer = 0.0
        self.ego_collision = False
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_drive_topic = self.get_parameter('ego_drive_topic').value
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.
        self.angle_max = scan_fov / 2.
        self.angle_inc = scan_fov / (scan_beams - 1)
        self.ego_namespace = self.get_parameter('ego_namespace').value
        ego_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_odom_topic').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value
        
        if num_agents == 2:
            self.has_opp = True
            self.opp_namespace = self.get_parameter('opp_namespace').value
            sx1 = self.get_parameter('sx1').value
            sy1 = self.get_parameter('sy1').value
            stheta1 = self.get_parameter('stheta1').value
            offset = float(self.get_parameter('opponent_start_offset_m').value)
            if offset > 0:
                sx1, sy1, stheta1 = following_pose([sx, sy, stheta], offset)
            self.opp_pose = [sx1, sy1, stheta1]
            self.opp_speed = [0.0, 0.0, 0.0]
            self.opp_requested_speed = 0.0
            self.opp_steer = 0.0
            self.opp_collision = False
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta], [sx1, sy1, stheta1]]))
            self.ego_scan = list(self.obs['scans'][0])
            self.opp_scan = list(self.obs['scans'][1])

            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_odom_topic').value
            opp_drive_topic = self.get_parameter('opp_drive_topic').value

            ego_opp_odom_topic = self.ego_namespace + '/' + self.get_parameter('ego_opp_odom_topic').value
            opp_ego_odom_topic = self.opp_namespace + '/' + self.get_parameter('opp_ego_odom_topic').value
        else:
            self.has_opp = False
            self.obs, _ , self.done, _ = self.env.reset(np.array([[sx, sy, stheta]]))
            self.ego_scan = list(self.obs['scans'][0])

        # Physics scheduling must not depend on the ROS clock we publish.
        self.steady_clock = TimerClock(clock_type=ClockType.STEADY_TIME)
        self.last_drive_time_ns = None
        self.drive_timeout_s = float(self.get_parameter('drive_timeout_s').value)
        if self.drive_timeout_s <= 0:
            raise ValueError('drive_timeout_s must be positive')
        self.session = SimulationSession(float(self.env.unwrapped.timestep))
        self.manual_stop_requested = False
        self.target_laps = self.get_parameter('target_laps').value
        self.stopping_deceleration = float(self.get_parameter('stopping_deceleration_mps2').value)
        if type(self.target_laps) is not int or self.target_laps < 1 or not np.isfinite(self.stopping_deceleration) or self.stopping_deceleration <= 0:
            raise ValueError('Lap target and stopping deceleration must be positive')
        self.start_poses = [list(self.ego_pose)] + ([list(self.opp_pose)] if self.has_opp else [])
        self.dual_gate = None
        if self.has_opp:
            self.dual_gate = DualControlGate((self.ego_namespace, self.opp_namespace),
                                             self.drive_timeout_s)
            self.dual_gate.reset((self.session.run_id, self.session.session_id))
        self.last_sensor_time_ns = None
        self.clock_pub = self.create_publisher(
            Clock, '/clock', QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        self.status_pub = self.create_publisher(String, '/sim/status', status_qos)
        self.collision_pub = self.create_publisher(Bool, '/ego_racecar/collision', status_qos)
        self.event_pub = self.create_publisher(String, '/sim/events', 100)
        self.controller_metadata = None
        self.opponent_controller_metadata = None
        self.lap_statuses = {}
        self.lap_status_received = {}
        self.controller_status_sub = self.create_subscription(
            String, '/' + self.ego_namespace + '/controller/status' if self.has_opp else '/controller/status',
            self.controller_status_callback, status_qos)
        self.laps_status_sub = self.create_subscription(
            String, '/' + self.ego_namespace + '/laps/status' if self.has_opp else '/laps/status',
            self.laps_status_callback, status_qos)
        self.vehicle_status_pubs = {}
        if self.has_opp:
            for vehicle in (self.ego_namespace, self.opp_namespace):
                self.vehicle_status_pubs[vehicle] = self.create_publisher(
                    String, '/' + vehicle + '/sim/status', status_qos)
            self.opp_collision_pub = self.create_publisher(
                Bool, '/' + self.opp_namespace + '/collision', status_qos)
            self.opp_controller_status_sub = self.create_subscription(
                String, '/' + self.opp_namespace + '/controller/status',
                lambda msg: self.controller_status_callback(msg, opponent=True), status_qos)
            self.opp_laps_status_sub = self.create_subscription(
                String, '/' + self.opp_namespace + '/laps/status',
                lambda msg: self.laps_status_callback(msg, opponent=True), status_qos)
        # RViz clears its map on a backwards /clock jump. Replay the static
        # map even while physics is stopped so reset does not leave it blank.
        self.static_map = None
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', status_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map_source', self.map_callback, status_qos)
        self.map_timer = self.create_timer(2.0, self.publish_map, clock=self.steady_clock)
        log_path = Path(self.get_parameter('event_log_path').value).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self.event_log = log_path.open('a', buffering=1)
        self.configuration = {
            'map_path': self.get_parameter('map_path').value,
            'map_img_ext': self.get_parameter('map_img_ext').value,
            'map_yaml_sha256': hashlib.sha256(Path(
                self.get_parameter('map_path').value + '.yaml').read_bytes()).hexdigest(),
            'map_image_sha256': hashlib.sha256(Path(
                self.get_parameter('map_path').value +
                self.get_parameter('map_img_ext').value).read_bytes()).hexdigest(),
            'timestep_s': self.session.timestep,
            'drive_timeout_s': self.drive_timeout_s,
            'vehicle_parameters': self.env.unwrapped.params,
            'num_agents': num_agents,
            'starting_poses': self.start_poses,
            'opponent_start_offset_m': float(self.get_parameter('opponent_start_offset_m').value),
            'completion_policy': 'all_vehicles_target_laps' if self.has_opp else 'target_laps',
            'target_laps': self.target_laps,
            'stopping_deceleration_mps2': self.stopping_deceleration,
            'code_sha256': hashlib.sha256(
                Path(__file__).read_bytes() +
                Path(__file__).with_name('simulation_session.py').read_bytes() +
                Path(__file__).with_name('dual_control.py').read_bytes()).hexdigest(),
        }
        self.reset_service = self.create_service(Trigger, '/sim/reset', self.reset_callback)
        self.pause_service = self.create_service(SetBool, '/sim/pause', self.pause_callback)
        self.end_service = self.create_service(Trigger, '/sim/end_attempt', self.end_callback)
        self.drive_timer = self.create_timer(
            self.session.timestep, self.drive_timer_callback, clock=self.steady_clock)
        self.timer = self.create_timer(0.004, self.timer_callback, clock=self.steady_clock)
        self.status_timer = self.create_timer(0.1, self.publish_status, clock=self.steady_clock)

        # transform broadcaster
        self.br = TransformBroadcaster(self)

        # publishers
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False
        if num_agents == 2:
            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False

        # subscribers
        self.ego_drive_sub = self.create_subscription(
            AckermannDriveStamped,
            ego_drive_topic,
            self.drive_callback,
            10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)
        if num_agents == 2:
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp_drive_topic,
                self.opp_drive_callback,
                10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)
        self._update_sim_state()
        if self.ego_collision or (self.has_opp and self.opp_collision):
            self.session.mark_collision()
        self.publish_status()

    def drive_callback(self, drive_msg):
        if not self.session.can_step or (self.session.completion_requested and not self.has_opp):
            return
        if self.has_opp and drive_msg.header.frame_id != self.ego_namespace + '/base_link':
            self.clear_commands()
            self.session.abort('primary_command_frame_mismatch')
            self.publish_status()
            return
        if not np.isfinite([drive_msg.drive.speed, drive_msg.drive.steering_angle]).all():
            if self.has_opp:
                self.clear_commands()
                self.session.abort('invalid_primary_command')
                self.publish_status()
                return
            self.ego_requested_speed = self.ego_steer = 0.0
            return
        if abs(drive_msg.drive.speed) > 0.0:
            if not self.has_opp:
                self.session.start()
        self.ego_requested_speed = drive_msg.drive.speed
        self.ego_steer = drive_msg.drive.steering_angle
        self.last_drive_time_ns = self.steady_clock.now().nanoseconds
        self.ego_drive_published = True
        if self.dual_gate is not None:
            self.dual_gate.on_drive(self.ego_namespace, self.last_drive_time_ns / 1e9)

    def opp_drive_callback(self, drive_msg):
        if not self.session.can_step or (self.session.completion_requested and not self.has_opp):
            return
        if drive_msg.header.frame_id != self.opp_namespace + '/base_link':
            self.clear_commands()
            self.session.abort('opponent_command_frame_mismatch')
            self.publish_status()
            return
        if not np.isfinite([drive_msg.drive.speed, drive_msg.drive.steering_angle]).all():
            self.clear_commands()
            self.session.abort('invalid_opponent_command')
            self.publish_status()
            return
        self.opp_requested_speed = drive_msg.drive.speed
        self.opp_steer = drive_msg.drive.steering_angle
        self.opp_drive_published = True
        self.dual_gate.on_drive(self.opp_namespace, self.steady_clock.now().nanoseconds / 1e9)

    def ego_reset_callback(self, pose_msg):
        rx = pose_msg.pose.pose.position.x
        ry = pose_msg.pose.pose.position.y
        rqx = pose_msg.pose.pose.orientation.x
        rqy = pose_msg.pose.pose.orientation.y
        rqz = pose_msg.pose.pose.orientation.z
        rqw = pose_msg.pose.pose.orientation.w
        _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
        if self.has_opp:
            opp_pose = [self.obs['poses_x'][1], self.obs['poses_y'][1], self.obs['poses_theta'][1]]
            self.reset_environment(np.array([[rx, ry, rtheta], opp_pose]), 'initialpose')
        else:
            self.reset_environment(np.array([[rx, ry, rtheta]]), 'initialpose')

    def opp_reset_callback(self, pose_msg):
        if self.has_opp:
            rx = pose_msg.pose.position.x
            ry = pose_msg.pose.position.y
            rqx = pose_msg.pose.orientation.x
            rqy = pose_msg.pose.orientation.y
            rqz = pose_msg.pose.orientation.z
            rqw = pose_msg.pose.orientation.w
            _, _, rtheta = euler.quat2euler([rqw, rqx, rqy, rqz], axes='sxyz')
            self.reset_environment(np.array([list(self.ego_pose), [rx, ry, rtheta]]), 'goal_pose')
    def teleop_callback(self, twist_msg):
        if self.has_opp:
            return  # Dual operation requires the two independent controllers.
        if not self.session.can_step or self.session.completion_requested:
            return
        if abs(twist_msg.linear.x) > 0.0:
            self.session.start()
        if not self.ego_drive_published:
            self.ego_drive_published = True

        self.ego_requested_speed = twist_msg.linear.x
        self.last_drive_time_ns = self.steady_clock.now().nanoseconds

        if twist_msg.angular.z > 0.0:
            self.ego_steer = 0.3
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -0.3
        else:
            self.ego_steer = 0.0

    def clear_commands(self):
        self.ego_requested_speed = self.ego_steer = 0.0
        self.last_drive_time_ns = None
        self.ego_drive_published = False
        if self.has_opp:
            self.opp_requested_speed = self.opp_steer = 0.0
            self.opp_drive_published = False

    def reset_environment(self, poses, reason):
        self.clear_commands()
        # Persist the old attempt with its final pose before replacing state.
        self.session.abort(reason)
        self.flush_events()
        self.obs, _, self.done, _ = self.env.reset(poses)
        self.session.reset(reason)
        self.manual_stop_requested = False
        if self.dual_gate is not None:
            self.dual_gate.reset((self.session.run_id, self.session.session_id))
            self.lap_statuses.clear()
            self.lap_status_received.clear()
        self._update_sim_state()
        if self.ego_collision or (self.has_opp and self.opp_collision):
            self.session.mark_collision()
        self.last_sensor_time_ns = None
        self.publish_status()
        self.timer_callback()

    def reset_callback(self, request, response):
        poses = (self.start_poses if self.has_opp else
                 [[self.get_parameter(k).value for k in ('sx', 'sy', 'stheta')]])
        self.reset_environment(np.array(poses), 'reset_service')
        response.success = not self.session.collision
        response.message = self.session.session_id
        return response

    def pause_callback(self, request, response):
        self.clear_commands()
        self.session.pause(request.data)
        self.publish_status()
        response.success = True
        response.message = 'paused' if request.data else 'resumed with zero commands'
        return response

    def end_callback(self, request, response):
        if self.has_opp and self.session.state == 'running':
            self.manual_stop_requested = True
            response.success = True
            response.message = 'Braking both vehicles before ending the attempt'
            self.publish_status()
            return response
        self.clear_commands()
        response.success = self.session.end()
        response.message = 'attempt ended' if response.success else 'no running attempt'
        self.publish_status()
        return response

    def sim_stamp(self):
        stamp = Time()
        stamp.sec, stamp.nanosec = divmod(self.session.time_ns, 1_000_000_000)
        return stamp

    def controller_status_callback(self, msg, opponent=False):
        try:
            data = json.loads(msg.data)
            metadata = {key: data[key] for key in
                        ('configuration', 'code_sha256', 'config_sha256')}
            previous = self.opponent_controller_metadata if opponent else self.controller_metadata
            if metadata != previous:
                if opponent:
                    self.opponent_controller_metadata = metadata
                else:
                    self.controller_metadata = metadata
                self.session.emit('controller_configured')
            if self.dual_gate is not None:
                vehicle = self.opp_namespace if opponent else self.ego_namespace
                self.dual_gate.on_controller(vehicle, data, self.steady_clock.now().nanoseconds / 1e9)
        except (ValueError, TypeError, KeyError):
            self.get_logger().warning('Ignoring invalid controller configuration metadata')

    def map_callback(self, msg):
        self.static_map = msg
        self.publish_map()

    def publish_map(self):
        if self.static_map is not None:
            self.map_pub.publish(self.static_map)

    def laps_status_callback(self, msg, opponent=False):
        try:
            data = json.loads(msg.data)
            if self.has_opp:
                vehicle = self.opp_namespace if opponent else self.ego_namespace
                if all(data.get(k) == self.session.snapshot()[k]
                       for k in ('run_id', 'session_id', 'attempt_id')):
                    self.lap_statuses[vehicle] = data
                    self.lap_status_received[vehicle] = self.steady_clock.now().nanoseconds / 1e9
                    if self.session.state == 'running' and data.get('measurement_valid') is False:
                        self.clear_commands()
                        self.session.abort(vehicle + '_lap_measurement_invalid')
                        self.publish_status()
                    elif self.session.request_joint_completion(self.lap_statuses,
                            (self.ego_namespace, self.opp_namespace), self.target_laps):
                        self.publish_status()
                return
            if self.session.request_completion(data):
                self.clear_commands()
                self.publish_status()
        except (ValueError, TypeError, AttributeError):
            self.get_logger().warning('Ignoring invalid lap completion status')

    def publish_status(self):
        self.clock_pub.publish(Clock(clock=self.sim_stamp()))
        snapshot = self.session.snapshot()
        snapshot.update({'pose': list(self.ego_pose), 'velocity': list(self.ego_speed),
                         'configuration': self.configuration,
                         'controller': self.controller_metadata})
        if self.has_opp:
            snapshot['stopping_requested'] = self.manual_stop_requested
            snapshot['controlled_stopping'] = (self.manual_stop_requested or self.session.completion_requested) and self.session.state == 'running'
            snapshot['any_collision'] = self.session.collision
            snapshot['awaiting_joint_start'] = self.session.state == 'ready' and not self.dual_gate.released
            snapshot['vehicles'] = self.vehicle_states()
            for vehicle, publisher in self.vehicle_status_pubs.items():
                own = snapshot['vehicles'][vehicle]
                status = {**snapshot, **own, 'vehicle_id': vehicle,
                          'own_target_reached': self.lap_statuses.get(vehicle, {}).get('target_reached') is True,
                          'braking_speed_limit_mps': self.braking_limit(own['velocity'][0])}
                publisher.publish(String(data=json.dumps(status)))
            self.opp_collision_pub.publish(Bool(data=self.opp_collision))
        self.status_pub.publish(String(data=json.dumps(snapshot)))
        self.collision_pub.publish(Bool(data=self.ego_collision))
        self.flush_events()

    def vehicle_states(self):
        return {
            self.ego_namespace: {'pose': list(self.ego_pose), 'velocity': list(self.ego_speed),
                                 'vehicle_collision': self.ego_collision,
                                 'controller': self.controller_metadata},
            self.opp_namespace: {'pose': list(self.opp_pose), 'velocity': list(self.opp_speed),
                                 'vehicle_collision': self.opp_collision,
                                 'controller': self.opponent_controller_metadata},
        }

    def flush_events(self, publish=True):
        for event in self.session.events:
            event.update({'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
                          'configuration': self.configuration, 'pose': list(self.ego_pose),
                          'controller': self.controller_metadata})
            if self.has_opp:
                event['vehicles'] = self.vehicle_states()
            line = json.dumps(event)
            self.event_log.write(line + '\n')
            if publish:
                self.event_pub.publish(String(data=line))
        self.session.events.clear()

    def braking_limit(self, speed):
        return max(0., abs(speed)-self.stopping_deceleration*self.session.timestep)

    def drive_timer_callback(self):
        if self.has_opp and self.session.can_step:
            health = self.dual_gate.health(self.steady_clock.now().nanoseconds / 1e9, {
                self.ego_namespace: self.count_publishers(self.ego_drive_sub.topic_name),
                self.opp_namespace: self.count_publishers(self.opp_drive_sub.topic_name)})
            if self.session.state == 'ready':
                released = self.dual_gate.released
                self.dual_gate.released = health is None
                if released != self.dual_gate.released:
                    self.clear_commands()
                    self.publish_status()
                if self.dual_gate.released and self.ego_requested_speed > 0 and self.opp_requested_speed > 0:
                    self.session.start()
            elif health:
                self.clear_commands()
                self.session.abort(health)
                self.publish_status()
            if self.session.state == 'running' and self.session.snapshot()['elapsed_sim_s'] > 1.:
                now = self.steady_clock.now().nanoseconds / 1e9
                if any(now-self.lap_status_received.get(v, -np.inf) > 1. for v in (self.ego_namespace, self.opp_namespace)):
                    self.clear_commands()
                    self.session.abort('lap_status_timeout')
                    self.publish_status()
        if (not self.has_opp and self.session.can_step and self.ego_requested_speed != 0.0
                and self.last_drive_time_ns is not None
                and (self.steady_clock.now().nanoseconds - self.last_drive_time_ns)/1e9
                > self.drive_timeout_s):
            self.clear_commands()
            self.session.abort('drive_timeout')
            self.publish_status()
        if self.session.can_step:
            actions = [[self.ego_steer, self.ego_requested_speed]]
            if self.has_opp:
                actions.append([self.opp_steer, self.opp_requested_speed])
                if self.manual_stop_requested or self.session.completion_requested:
                    actions[0][1] = min(actions[0][1], self.braking_limit(self.ego_speed[0]))
                    actions[1][1] = min(actions[1][1], self.braking_limit(self.opp_speed[0]))
                if self.session.state == 'ready':
                    actions = [[0., 0.], [0., 0.]]
            self.obs, _, self.done, _ = self.env.step(np.array(actions))
            self._update_sim_state()
            self.session.step(collision=self.ego_collision or (self.has_opp and self.opp_collision))
            if (self.has_opp and self.manual_stop_requested and self.session.state == 'running'
                    and max(abs(self.ego_speed[0]), abs(self.opp_speed[0])) < .02):
                self.session.end('target_laps_completed' if self.session.completion_requested else 'manual_stop')
            if self.has_opp and self.session.completion_requested:
                self.session.finish_if_stopped(max(abs(self.ego_speed[0]), abs(self.opp_speed[0])))
            if not self.has_opp:
                self.session.finish_if_stopped(float(self.ego_speed[0]))
            if self.session.collision:
                self.clear_commands()
            # Backend done also includes a built-in lap heuristic, not just
            # collisions. It must not terminate a future ten-lap attempt.
            self.clock_pub.publish(Clock(clock=self.sim_stamp()))
            if self.session.events:
                self.publish_status()

    def timer_callback(self):
        if self.last_sensor_time_ns == self.session.time_ns:
            return
        self.last_sensor_time_ns = self.session.time_ns
        ts = self.sim_stamp()

        # pub scans
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = self.ego_namespace + '/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.
        scan.range_max = 30.
        scan.ranges = self.ego_scan
        self.ego_scan_pub.publish(scan)

        if self.has_opp:
            opp_scan = LaserScan()
            opp_scan.header.stamp = ts
            opp_scan.header.frame_id = self.opp_namespace + '/laser'
            opp_scan.angle_min = self.angle_min
            opp_scan.angle_max = self.angle_max
            opp_scan.angle_increment = self.angle_inc
            opp_scan.range_min = 0.
            opp_scan.range_max = 30.
            opp_scan.ranges = self.opp_scan
            self.opp_scan_pub.publish(opp_scan)

        # pub tf
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    def _update_sim_state(self):
        self.ego_scan = list(self.obs['scans'][0])
        self.ego_collision = bool(self.obs['collisions'][0])
        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])
            self.opp_collision = bool(self.obs['collisions'][1])
            self.opp_pose[0] = self.obs['poses_x'][1]
            self.opp_pose[1] = self.obs['poses_y'][1]
            self.opp_pose[2] = self.obs['poses_theta'][1]
            self.opp_speed[0] = self.obs['linear_vels_x'][1]
            self.opp_speed[1] = self.obs['linear_vels_y'][1]
            self.opp_speed[2] = self.obs['ang_vels_z'][1]

        self.ego_pose[0] = self.obs['poses_x'][0]
        self.ego_pose[1] = self.obs['poses_y'][0]
        self.ego_pose[2] = self.obs['poses_theta'][0]
        self.ego_speed[0] = self.obs['linear_vels_x'][0]
        self.ego_speed[1] = self.obs['linear_vels_y'][0]
        self.ego_speed[2] = self.obs['ang_vels_z'][0]

        

    def _publish_odom(self, ts):
        ego_odom = Odometry()
        ego_odom.header.stamp = ts
        ego_odom.header.frame_id = 'map'
        ego_odom.child_frame_id = self.ego_namespace + '/base_link'
        ego_odom.pose.pose.position.x = self.ego_pose[0]
        ego_odom.pose.pose.position.y = self.ego_pose[1]
        ego_quat = euler.euler2quat(0., 0., self.ego_pose[2], axes='sxyz')
        ego_odom.pose.pose.orientation.x = ego_quat[1]
        ego_odom.pose.pose.orientation.y = ego_quat[2]
        ego_odom.pose.pose.orientation.z = ego_quat[3]
        ego_odom.pose.pose.orientation.w = ego_quat[0]
        ego_odom.twist.twist.linear.x = self.ego_speed[0]
        ego_odom.twist.twist.linear.y = self.ego_speed[1]
        ego_odom.twist.twist.angular.z = self.ego_speed[2]
        self.ego_odom_pub.publish(ego_odom)

        if self.has_opp:
            opp_odom = Odometry()
            opp_odom.header.stamp = ts
            opp_odom.header.frame_id = 'map'
            opp_odom.child_frame_id = self.opp_namespace + '/base_link'
            opp_odom.pose.pose.position.x = self.opp_pose[0]
            opp_odom.pose.pose.position.y = self.opp_pose[1]
            opp_quat = euler.euler2quat(0., 0., self.opp_pose[2], axes='sxyz')
            opp_odom.pose.pose.orientation.x = opp_quat[1]
            opp_odom.pose.pose.orientation.y = opp_quat[2]
            opp_odom.pose.pose.orientation.z = opp_quat[3]
            opp_odom.pose.pose.orientation.w = opp_quat[0]
            opp_odom.twist.twist.linear.x = self.opp_speed[0]
            opp_odom.twist.twist.linear.y = self.opp_speed[1]
            opp_odom.twist.twist.angular.z = self.opp_speed[2]
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _publish_transforms(self, ts):
        ego_t = Transform()
        ego_t.translation.x = self.ego_pose[0]
        ego_t.translation.y = self.ego_pose[1]
        ego_t.translation.z = 0.0
        ego_quat = euler.euler2quat(0.0, 0.0, self.ego_pose[2], axes='sxyz')
        ego_t.rotation.x = ego_quat[1]
        ego_t.rotation.y = ego_quat[2]
        ego_t.rotation.z = ego_quat[3]
        ego_t.rotation.w = ego_quat[0]

        ego_ts = TransformStamped()
        ego_ts.transform = ego_t
        ego_ts.header.stamp = ts
        ego_ts.header.frame_id = 'map'
        ego_ts.child_frame_id = self.ego_namespace + '/base_link'
        self.br.sendTransform(ego_ts)

        if self.has_opp:
            opp_t = Transform()
            opp_t.translation.x = self.opp_pose[0]
            opp_t.translation.y = self.opp_pose[1]
            opp_t.translation.z = 0.0
            opp_quat = euler.euler2quat(0.0, 0.0, self.opp_pose[2], axes='sxyz')
            opp_t.rotation.x = opp_quat[1]
            opp_t.rotation.y = opp_quat[2]
            opp_t.rotation.z = opp_quat[3]
            opp_t.rotation.w = opp_quat[0]

            opp_ts = TransformStamped()
            opp_ts.transform = opp_t
            opp_ts.header.stamp = ts
            opp_ts.header.frame_id = 'map'
            opp_ts.child_frame_id = self.opp_namespace + '/base_link'
            self.br.sendTransform(opp_ts)

    def _publish_wheel_transforms(self, ts):
        ego_wheel_ts = TransformStamped()
        ego_wheel_quat = euler.euler2quat(0., 0., self.ego_steer, axes='sxyz')
        ego_wheel_ts.transform.rotation.x = ego_wheel_quat[1]
        ego_wheel_ts.transform.rotation.y = ego_wheel_quat[2]
        ego_wheel_ts.transform.rotation.z = ego_wheel_quat[3]
        ego_wheel_ts.transform.rotation.w = ego_wheel_quat[0]
        ego_wheel_ts.header.stamp = ts
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_left_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_left_wheel'
        self.br.sendTransform(ego_wheel_ts)
        ego_wheel_ts.header.frame_id = self.ego_namespace + '/front_right_hinge'
        ego_wheel_ts.child_frame_id = self.ego_namespace + '/front_right_wheel'
        self.br.sendTransform(ego_wheel_ts)

        if self.has_opp:
            opp_wheel_ts = TransformStamped()
            opp_wheel_quat = euler.euler2quat(0., 0., self.opp_steer, axes='sxyz')
            opp_wheel_ts.transform.rotation.x = opp_wheel_quat[1]
            opp_wheel_ts.transform.rotation.y = opp_wheel_quat[2]
            opp_wheel_ts.transform.rotation.z = opp_wheel_quat[3]
            opp_wheel_ts.transform.rotation.w = opp_wheel_quat[0]
            opp_wheel_ts.header.stamp = ts
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_left_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_left_wheel'
            self.br.sendTransform(opp_wheel_ts)
            opp_wheel_ts.header.frame_id = self.opp_namespace + '/front_right_hinge'
            opp_wheel_ts.child_frame_id = self.opp_namespace + '/front_right_wheel'
            self.br.sendTransform(opp_wheel_ts)

    def _publish_laser_transforms(self, ts):
        ego_scan_ts = TransformStamped()
        ego_scan_ts.transform.translation.x = self.scan_distance_to_base_link
        # ego_scan_ts.transform.translation.z = 0.04+0.1+0.025
        ego_scan_ts.transform.rotation.w = 1.
        ego_scan_ts.header.stamp = ts
        ego_scan_ts.header.frame_id = self.ego_namespace + '/base_link'
        ego_scan_ts.child_frame_id = self.ego_namespace + '/laser'
        self.br.sendTransform(ego_scan_ts)

        if self.has_opp:
            opp_scan_ts = TransformStamped()
            opp_scan_ts.transform.translation.x = self.scan_distance_to_base_link
            opp_scan_ts.transform.rotation.w = 1.
            opp_scan_ts.header.stamp = ts
            opp_scan_ts.header.frame_id = self.opp_namespace + '/base_link'
            opp_scan_ts.child_frame_id = self.opp_namespace + '/laser'
            self.br.sendTransform(opp_scan_ts)

def main(args=None):
    rclpy.init(args=args)
    gym_bridge = GymBridge()
    try:
        rclpy.spin(gym_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        gym_bridge.session.abort('shutdown')
        gym_bridge.flush_events(publish=False)
        gym_bridge.event_log.close()
        gym_bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
