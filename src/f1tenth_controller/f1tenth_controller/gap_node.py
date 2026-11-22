"""ROS adapter for Follow the Gap; decisions use only the LaserScan."""
from dataclasses import asdict, replace
from pathlib import Path
import hashlib
import json
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool
from ackermann_msgs.msg import AckermannDriveStamped

from .gap import FollowGap, GapParameters, Decision
from .control_gate import ControlGate


class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower')
        defaults = asdict(GapParameters())
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameters = GapParameters(**{k: self.get_parameter(k).value for k in defaults})
        self.core = FollowGap(parameters)
        self.declare_parameter('speed_scale', 1.0)
        self.declare_parameter('vehicle_id', 'ego_racecar')
        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.vehicle_id = self.get_parameter('vehicle_id').value
        if not math.isfinite(self.speed_scale) or not 0 < self.speed_scale <= 1:
            raise ValueError('speed_scale must be in (0, 1]')
        self.declare_parameter('scan_timeout_s', 0.3)
        self.declare_parameter('status_timeout_s', 0.5)
        self.declare_parameter('max_scan_age_s', 0.15)
        self.max_scan_age = float(self.get_parameter('max_scan_age_s').value)
        self.gate = ControlGate(float(self.get_parameter('scan_timeout_s').value),
                                float(self.get_parameter('status_timeout_s').value))
        self.previous_stamp = None
        self.last_decision = Decision(reason='disabled_at_startup')
        self.drive = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.debug = self.create_publisher(String, '/controller/status', retained)
        self.status_sub = self.create_subscription(String, '/sim/status', self.on_status, retained)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.on_scan,
                                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.service = self.create_service(SetBool, '/controller/enable', self.enable)
        # A safety watchdog must continue to run when /clock is paused.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.watchdog = self.create_timer(0.05, self.check_health, clock=self.steady_clock)
        self.debug_timer = self.create_timer(0.2, self.publish_status, clock=self.steady_clock)
        self.configuration = asdict(parameters)
        self.configuration.update(scan_timeout_s=self.gate.scan_timeout,
                                  status_timeout_s=self.gate.status_timeout,
                                  max_scan_age_s=self.max_scan_age)
        self.configuration.update(speed_scale=self.speed_scale, vehicle_id=self.vehicle_id)
        sources = [Path(__file__), Path(__file__).with_name('gap.py'),
                   Path(__file__).with_name('control_gate.py')]
        self.code_sha256 = hashlib.sha256(b''.join(p.read_bytes() for p in sources)).hexdigest()
        self.config_sha256 = hashlib.sha256(json.dumps(self.configuration, sort_keys=True).encode()).hexdigest()
        self.get_logger().info('Follow the Gap ready, DISABLED. Enable through its controller service.')

    def publish_command(self, speed=0.0, steering=0.0):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.vehicle_id + '/base_link'
        msg.drive.speed, msg.drive.steering_angle = float(speed), float(steering)
        self.drive.publish(msg)

    def stop(self, reason):
        self.gate.disarm(reason)
        self.core.reset()
        self.previous_stamp = None
        self.last_decision = Decision(reason=reason)
        self.publish_command()

    def on_status(self, msg):
        try:
            status = json.loads(msg.data)
            old_identity, was_armed = self.gate.identity, self.gate.armed
            self.gate.on_status(status, time.monotonic())
            if old_identity != self.gate.identity or (was_armed and not self.gate.armed):
                self.stop(self.gate.reason)
        except (ValueError, KeyError, TypeError):
            self.stop('invalid_simulator_status')

    def enable(self, request, response):
        if not request.data:
            self.stop('disabled_by_service')
            response.success = True
        else:
            response.success = self.gate.enable(time.monotonic(), self.count_publishers(self.drive.topic_name))
            if response.success:
                self.core.reset()
                self.previous_stamp = None
        response.message = self.gate.reason
        self.publish_status()
        return response

    def check_health(self):
        reason = self.gate.health(time.monotonic(), self.count_publishers(self.drive.topic_name))
        if self.gate.armed and reason:
            self.stop(reason)
        elif not self.gate.armed:
            self.publish_command()

    def on_scan(self, scan):
        stamp = scan.header.stamp.sec*1_000_000_000 + scan.header.stamp.nanosec
        now_ros = self.get_clock().now().nanoseconds
        age = (now_ros-stamp)/1e9
        if not math.isfinite(age) or age > self.max_scan_age or age < -0.05:
            if self.gate.armed:
                self.stop('old_or_future_scan')
            return
        self.gate.scan_received = time.monotonic()
        if not self.gate.armed:
            return
        reason = self.gate.health(time.monotonic(), self.count_publishers(self.drive.topic_name))
        if reason:
            self.stop(reason)
            return
        if self.gate.status.get('awaiting_joint_start'):
            self.core.reset()
            self.previous_stamp = None
            self.last_decision = Decision(reason='waiting_for_joint_start')
            self.publish_command()
            return
        if self.previous_stamp is not None and stamp <= self.previous_stamp:
            self.stop('nonincreasing_scan_time')
            return
        dt = 0.01 if self.previous_stamp is None else (stamp-self.previous_stamp)/1e9
        self.previous_stamp = stamp
        if self.gate.status.get('own_target_reached'):
            # Keep publishing and remain healthy while the other car finishes.
            # Do not let a passing car make the parked controller seek a new gap.
            self.last_decision = Decision(reason='vehicle_finished', steering=self.last_decision.steering)
            self.publish_command(0., self.last_decision.steering)
            return
        decision = self.core.decide(scan.ranges, scan.angle_min, scan.angle_increment,
                                    scan.range_min, scan.range_max, min(dt, 0.05))
        self.last_decision = replace(decision, speed=decision.speed * self.speed_scale)
        if self.gate.status.get('controlled_stopping'):
            limit = self.gate.status['braking_speed_limit_mps']
            if not math.isfinite(limit) or limit < 0:
                self.stop('invalid_braking_limit')
                return
            self.last_decision = replace(self.last_decision, speed=min(self.last_decision.speed, limit))
        if decision.reason != 'following_gap':
            self.stop(decision.reason)
            return
        self.publish_command(self.last_decision.speed, self.last_decision.steering)

    def publish_status(self):
        status = {'armed': self.gate.armed, 'reason': self.gate.reason,
                  'identity': self.gate.identity, 'decision': asdict(self.last_decision),
                  'configuration': self.configuration, 'config_sha256': self.config_sha256,
                  'code_sha256': self.code_sha256}
        self.debug.publish(String(data=json.dumps(status)))


def main(args=None):
    rclpy.init(args=args)
    node = GapFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publish_command()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
