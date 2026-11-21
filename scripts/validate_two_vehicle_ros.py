#!/usr/bin/env python3
"""Check dual integration, shared start and a bounded circulation trial.

This is not the ten-lap/overtaking certification. Controllers must be running
disabled, with no other command publishers in this ROS domain.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import signal

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped

from map_footprint import MapFootprint, rectangles_overlap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--fault-controller', choices=('ego_racecar', 'opp_racecar'))
    parser.add_argument('--fault-pid', type=int)
    args = parser.parse_args()
    if bool(args.fault_controller) != bool(args.fault_pid):
        parser.error('Fault injection requires both a controller and its PID')
    if args.fault_pid:
        command = Path(f'/proc/{args.fault_pid}/cmdline').read_bytes().replace(b'\0', b' ')
        if b'/f1tenth_controller/gap_follower' not in command or (
                '__ns:=/'+args.fault_controller).encode() not in command:
            parser.error('PID does not belong to the selected controller')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('validate_two_vehicle_integration')
    vehicles = ('ego_racecar', 'opp_racecar')
    latest, scans, scan_count, poses = {}, {}, dict.fromkeys(vehicles, 0), {}
    footprints, peaks, pair_count = {}, dict.fromkeys(vehicles, 0.), 0
    active = False
    stopped_pid = None
    length = width = 0.
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    report = {'result': 'failed', 'mode': 'dual_integration_not_race_certification'}

    def receive(key, msg):
        latest[key] = json.loads(msg.data)

    def on_scan(vehicle, msg):
        scans[vehicle] = msg
        scan_count[vehicle] += 1

    def on_pose(vehicle, msg):
        nonlocal pair_count
        if not active:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = [p.x, p.y, yaw]
        if footprints[vehicle].collides(*pose):
            raise ValueError(vehicle + '_map_footprint_collision')
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        peaks[vehicle] = max(peaks[vehicle], abs(msg.twist.twist.linear.x))
        poses.setdefault(stamp, {})[vehicle] = pose
        if len(poses[stamp]) == 2:
            pair_count += 1
            if rectangles_overlap(poses[stamp][vehicles[0]], poses[stamp][vehicles[1]], length, width):
                raise ValueError('vehicle_body_overlap')
        if len(poses) > 150:
            del poses[min(poses)]

    subscriptions = [node.create_subscription(String, '/sim/status',
                     lambda msg: receive('sim', msg), qos)]
    for vehicle in vehicles:
        for suffix, key in [('controller/status', 'controller'), ('laps/status', 'laps')]:
            subscriptions.append(node.create_subscription(String, '/' + vehicle + '/' + suffix,
                                 lambda msg, k=vehicle+'/'+key: receive(k, msg), qos))
        subscriptions.append(node.create_subscription(LaserScan, '/'+vehicle+'/scan',
                             lambda msg, v=vehicle: on_scan(v, msg), 10))
        subscriptions.append(node.create_subscription(Odometry, '/'+vehicle+'/odom',
                             lambda msg, v=vehicle: on_pose(v, msg), 100))
    clients = {v: node.create_client(SetBool, '/'+v+'/controller/enable') for v in vehicles}
    clients.update(reset=node.create_client(Trigger, '/sim/reset'),
                   end=node.create_client(Trigger, '/sim/end_attempt'))
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def spin(seconds):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)

    def wait(predicate, timeout=8.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            spin(.02)
        if not predicate():
            raise ValueError('observation_timeout')

    def call(name, value=None):
        client = clients[name]
        if not client.wait_for_service(timeout_sec=5.):
            raise ValueError('missing_service_'+name)
        future = client.call_async(Trigger.Request() if value is None else SetBool.Request(data=value))
        wait(future.done)
        response = future.result()
        if not response.success:
            raise ValueError(name+': '+response.message)
        spin(.15)

    try:
        wait(lambda: len(latest) == 5 and all(v in scans for v in vehicles), timeout=30.)
        if node.count_publishers('/clock') != 1 or node.count_publishers('/sim/status') != 1:
            raise ValueError('duplicate_or_missing_simulator')
        for v in vehicles:
            if node.count_publishers('/'+v+'/drive') != 1:
                raise ValueError('duplicate_or_missing_drive_publisher')
            call(v, False)
        call('reset')
        wait(lambda: latest['sim']['state'] == 'ready' and
             all(latest[v+'/laps']['state'] == 'ready' for v in vehicles))
        source = latest['sim']
        if source['configuration']['num_agents'] != 2:
            raise ValueError('expected_two_vehicles')
        starts = source['configuration']['starting_poses']
        offset = source['configuration']['opponent_start_offset_m']
        if offset <= 0 or not math.isclose(math.dist(starts[0][:2], starts[1][:2]), offset, abs_tol=1e-8):
            raise ValueError('wrong_starting_separation')
        length, width = (source['configuration']['vehicle_parameters'][k] for k in ('length', 'width'))
        for v, start, scale in zip(vehicles, starts, (1., source['configuration']['opponent_speed_scale'])):
            config = latest[v+'/controller']['configuration']
            if config['vehicle_id'] != v or config['speed_scale'] != scale or config['max_speed_mps'] != 12.:
                raise ValueError('wrong_controller_binding_or_profile')
            counter = latest[v+'/laps']['configuration']
            if not math.isclose(counter['gate_x'], start[0]) or not math.isclose(counter['gate_y'], start[1]):
                raise ValueError('counter_not_at_own_start')
            footprints[v] = MapFootprint(source['configuration']['map_path'], length, width)
            if footprints[v].collides(*start):
                raise ValueError('starting_body_overlap')
        report['initial_status'] = json.loads(json.dumps(latest))
        # At the starting pose the rear car is behind the leader's LiDAR FOV.
        # A temporary side-by-side placement puts both bodies within both FOVs.
        x, y, yaw = starts[1]
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x+.8*math.sin(yaw)
        msg.pose.pose.position.y = y-.8*math.cos(yaw)
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        before = dict(scan_count)
        pose_pub.publish(msg)
        wait(lambda: math.dist(latest['sim']['pose'][:2],
             [msg.pose.pose.position.x, msg.pose.pose.position.y]) < 1e-5 and
             all(scan_count[v] > before[v]+5 for v in vehicles))
        ray_ranges = {}
        for v, bearing in zip(vehicles, (math.pi/2, -math.pi/2)):
            scan = scans[v]
            index = round((bearing-scan.angle_min)/scan.angle_increment)
            distance = min(scan.ranges[index-3:index+4])
            if not .4 < distance < .9:
                raise ValueError('other_vehicle_not_seen_'+v)
            ray_ranges[v] = distance
        report['side_by_side_lidar_ranges_m'] = ray_ranges
        call('reset')
        call(vehicles[0], True)
        spin(.8)
        if latest['sim']['state'] != 'ready' or latest['sim']['attempt_id'] is not None:
            raise ValueError('one_controller_started_the_attempt')
        if any(math.dist(latest['sim']['vehicles'][v]['pose'][:2], s[:2]) > 1e-5
               for v, s in zip(vehicles, starts)):
            raise ValueError('vehicle_moved_before_joint_start')
        report['single_controller_start_blocked'] = True
        active = True
        call(vehicles[1], True)
        wait(lambda: latest['sim']['state'] == 'running')
        start_wall = time.monotonic()
        fault_observed = False
        while time.monotonic()-start_wall < args.seconds:
            spin(.02)
            source = latest['sim']
            if args.fault_pid and stopped_pid is None and time.monotonic()-start_wall > .5:
                os.kill(args.fault_pid, signal.SIGSTOP)
                stopped_pid = args.fault_pid
            if stopped_pid and source['state'] == 'ended':
                if source['end_reason'] != args.fault_controller+'_drive_timeout' or source['attempt_valid']:
                    raise ValueError('wrong_fault_response')
                before = [(source['sim_time_ns'], source['vehicles'][v]['pose']) for v in vehicles]
                spin(.4)
                after = [(latest['sim']['sim_time_ns'], latest['sim']['vehicles'][v]['pose']) for v in vehicles]
                if before != after:
                    raise ValueError('physics_not_frozen_after_command_loss')
                fault_observed = True
                break
            if source['state'] != 'running' or source['attempt_valid'] is not True:
                raise ValueError('attempt_interrupted: '+str(source['end_reason']))
            if any(not latest[v+'/controller']['armed'] for v in vehicles):
                raise ValueError('controller_disarmed')
        if args.fault_pid:
            if not fault_observed:
                raise ValueError('command_loss_not_detected')
            active = False
            report.update(result='passed', mode='dual_controller_loss_fault_injection',
                          fault_controller=args.fault_controller,
                          final_status=json.loads(json.dumps(latest)))
            print(json.dumps({'result': 'passed', 'fault_controller': args.fault_controller,
                              'end_reason': latest['sim']['end_reason']}), flush=True)
            return
        call('end')
        wait(lambda: latest['sim']['state'] == 'ended')
        active = False
        if any(abs(latest['sim']['vehicles'][v]['velocity'][0]) >= .02 for v in vehicles):
            raise ValueError('vehicles_not_stopped')
        if any(peaks[v] <= .1 for v in vehicles) or pair_count < 10:
            raise ValueError('insufficient_simultaneous_motion')
        report.update(result='passed', circulation_wall_s=args.seconds,
                      peaks_mps=peaks, same_time_body_checks=pair_count,
                      final_status=json.loads(json.dumps(latest)))
        print(json.dumps({'result': 'passed', 'peaks_mps': peaks,
                          'same_time_body_checks': pair_count}), flush=True)
    except BaseException as exc:
        report['error'] = str(exc)
        raise
    finally:
        active = False
        if stopped_pid:
            try:
                os.kill(stopped_pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
        try:
            if latest.get('sim', {}).get('state') == 'running':
                call('end')
                wait(lambda: latest['sim']['state'] == 'ended', timeout=4.)
            for v in vehicles:
                call(v, False)
            call('reset')
        except Exception as exc:
            report['cleanup_error'] = str(exc)
        output.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
