#!/usr/bin/env python3
"""Observe a full dual attempt; successful completion is exclusively automatic."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool

from f1tenth_controller.session_guard import SessionGuard
from lap_validation import LapValidation
from map_footprint import MapFootprint, rectangles_overlap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-wall-seconds', type=float, default=1500.)
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    vehicles = ('ego_racecar', 'opp_racecar')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('dual_consecutive_lap_session')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    latest, received, events = {}, {}, {v: [] for v in vehicles}
    references, crossings, footprints, targets = {}, {v: [] for v in vehicles}, {}, {}
    poses, previous, extra_distance, peaks = {}, {}, dict.fromkeys(vehicles, 0.), dict.fromkeys(vehicles, 0.)
    active, start_ns, body_checks = False, None, 0
    length = width = 0.
    wall_start = time.monotonic()
    report = {'result': 'initializing', 'mode': 'dual_complete_ros_system',
              'supervisor_sha256': hashlib.sha256(Path(__file__).read_bytes() +
                  Path(__file__).with_name('lap_validation.py').read_bytes() +
                  Path(__file__).with_name('map_footprint.py').read_bytes()).hexdigest()}

    def receive(key, msg):
        nonlocal start_ns
        latest[key] = json.loads(msg.data)
        received[key] = time.monotonic()
        if key == 'sim' and active and latest[key]['state'] == 'running' and start_ns is None:
            start_ns = latest[key]['sim_time_ns']-round(latest[key]['elapsed_sim_s']*1e9)

    def make_reference(vehicle):
        c = latest[vehicle+'/laps']['configuration']
        ref = LapValidation((c['gate_x'], c['gate_y'], c['gate_yaw']))
        ref.checkpoints = c['checkpoints']
        return ref

    def on_pose(vehicle, msg):
        nonlocal body_checks
        if not active or start_ns is None:
            return
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        if stamp < start_ns:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        pose = [p.x, p.y, yaw]
        if footprints[vehicle].collides(*pose):
            raise ValueError(vehicle+'_map_footprint_collision')
        peaks[vehicle] = max(peaks[vehicle], abs(msg.twist.twist.linear.x))
        if vehicle in targets and vehicle in previous:
            extra_distance[vehicle] += math.dist(pose[:2], previous[vehicle][:2])
            if vehicle == vehicles[0] and stamp-start_ns > round((targets[vehicle]['completed_time_s']+3.)*1e9):
                if abs(msg.twist.twist.linear.x) > .02 or extra_distance[vehicle] > .3:
                    raise ValueError('primary_did_not_remain_parked_at_finish')
        previous[vehicle] = pose
        poses.setdefault(stamp, {})[vehicle] = pose
        if len(poses[stamp]) == 2:
            body_checks += 1
            if rectangles_overlap(poses[stamp][vehicles[0]], poses[stamp][vehicles[1]], length, width):
                raise ValueError('vehicle_body_overlap')
        if len(poses) > 150:
            del poses[min(poses)]
        if len(crossings[vehicle]) < 10 and references[vehicle].update(p.x, p.y, (stamp-start_ns)/1e9):
            crossings[vehicle].append({'finish_sample_sim_s': (stamp-start_ns)/1e9,
                                       'route': references[vehicle].summary()})
            references[vehicle] = make_reference(vehicle)

    subs = [node.create_subscription(String, '/sim/status', lambda m: receive('sim', m), qos)]
    for v in vehicles:
        for suffix, key in [('controller/status', 'controller'), ('laps/status', 'laps')]:
            subs.append(node.create_subscription(String, '/'+v+'/'+suffix,
                        lambda m, k=v+'/'+key: receive(k, m), qos))
        subs.append(node.create_subscription(Odometry, '/'+v+'/odom',
                    lambda m, v=v: on_pose(v, m), 100))
        subs.append(node.create_subscription(String, '/'+v+'/laps/events',
                    lambda m, v=v: events[v].append(json.loads(m.data)), 100))
    clients = {v: node.create_client(SetBool, '/'+v+'/controller/enable') for v in vehicles}
    clients.update(reset=node.create_client(Trigger, '/sim/reset'),
                   end=node.create_client(Trigger, '/sim/end_attempt'))

    def spin(seconds):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.01)

    def wait(predicate, timeout=8.):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            spin(.02)
        if not predicate():
            raise ValueError('observation_timeout')

    def call(name, value=None):
        client = clients[name]
        if not client.wait_for_service(timeout_sec=5.):
            raise ValueError('missing_service_'+name)
        future = client.call_async(Trigger.Request() if value is None else SetBool.Request(data=value))
        wait(future.done)
        if not future.result().success:
            raise ValueError(name+': '+future.result().message)
        spin(.15)

    def checkpoint(result='running', error=None):
        report.update(result=result, error=error, last_status=latest,
                      independent_crossings=crossings, distance_after_target_m=extra_distance,
                      target_crossing_status=targets, peaks_mps=peaks,
                      same_time_body_checks=body_checks,
                      footprint_samples={v: f.samples for v, f in footprints.items()},
                      wall_elapsed_s=time.monotonic()-wall_start)
        temporary = output.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        temporary.replace(output)

    try:
        wait(lambda: len(latest) == 5, timeout=40.)
        if node.count_publishers('/clock') != 1 or node.count_publishers('/sim/status') != 1:
            raise ValueError('duplicate_or_missing_simulator')
        for v in vehicles:
            if node.count_publishers('/'+v+'/drive') != 1:
                raise ValueError('duplicate_or_missing_drive_publisher')
            call(v, False)
        call('reset')
        wait(lambda: latest['sim']['state'] == 'ready' and all(
             latest[v+'/laps']['state'] == 'ready' and latest[v+'/laps']['lap_count'] == 0
             and latest[v+'/laps']['session_id'] == latest['sim']['session_id'] for v in vehicles))
        config = latest['sim']['configuration']
        if config['completion_policy'] != 'all_vehicles_target_laps' or config['target_laps'] != 10:
            raise ValueError('wrong_completion_policy_or_target')
        starts = config['starting_poses']
        if not math.isclose(math.dist(starts[0][:2], starts[1][:2]), 5., abs_tol=1e-8):
            raise ValueError('wrong_starting_separation')
        length, width = (config['vehicle_parameters'][k] for k in ('length', 'width'))
        for v, start, scale in zip(vehicles, starts, (1., .5)):
            c = latest[v+'/laps']['configuration']
            controller = latest[v+'/controller']['configuration']
            if (c['target_laps'] != 10 or not c['stop_counting_at_target']
                    or not math.isclose(c['gate_x'], start[0]) or not math.isclose(c['gate_y'], start[1])
                    or controller['vehicle_id'] != v or controller['speed_scale'] != scale
                    or controller['max_speed_mps'] != 12.):
                raise ValueError('wrong_vehicle_configuration')
            footprints[v] = MapFootprint(config['map_path'], length, width)
            references[v] = make_reference(v)
        report['initial_status'] = json.loads(json.dumps(latest))
        active = True
        for v in vehicles:
            call(v, True)
        wait(lambda: latest['sim']['state'] == 'running' and all(
             latest[v+'/laps']['state'] == 'running' and latest[v+'/controller']['armed']
             and latest[v+'/laps']['attempt_id'] == latest['sim']['attempt_id'] for v in vehicles))
        guards = {v: SessionGuard(10, latest['sim'], latest[v+'/laps'], latest[v+'/controller'],
                                  max_lap_sim_s=160.) for v in vehicles}
        print('DUAL SESSION STARTED: ten laps each, no pauses or resets.', flush=True)
        counts, previous_print = dict.fromkeys(vehicles, 0), 0.
        while time.monotonic()-wall_start < args.max_wall_seconds:
            spin(.05)
            now = time.monotonic()
            for v in vehicles:
                reached = guards[v].observe(latest['sim'], latest[v+'/laps'], latest[v+'/controller'],
                    {'sim': received['sim'], 'laps': received[v+'/laps'],
                     'controller': received[v+'/controller']}, now)
                status = latest[v+'/laps']
                if reached and v not in targets:
                    targets[v] = json.loads(json.dumps(status))
                    other = vehicles[1] if v == vehicles[0] else vehicles[0]
                    if latest[other+'/laps']['lap_count'] < 10 and latest['sim']['completion_requested']:
                        raise ValueError('premature_joint_completion')
                if v in targets and status['lap_times_s'] != targets[v]['lap_times_s']:
                    raise ValueError('completed_result_changed_while_waiting')
                if status['lap_count'] != counts[v]:
                    counts[v] = status['lap_count']
                    print(json.dumps({'vehicle': v, 'lap': counts[v],
                        'last_lap_s': status['last_lap_time_s'], 'total_s': status['completed_time_s']}), flush=True)
                    checkpoint()
            if latest['sim']['state'] == 'ended' and all(latest[v+'/laps']['state'] == 'ended' for v in vehicles):
                break
            if now-previous_print >= 45.:
                previous_print = now
                print(json.dumps({'counts': counts, 'elapsed_sim_s': latest['sim']['elapsed_sim_s']}), flush=True)
                checkpoint()
        else:
            raise ValueError('wall_time_limit')
        wait(lambda: all(not latest[v+'/controller']['armed'] for v in vehicles))
        source = latest['sim']
        if (source['end_reason'] != 'target_laps_completed' or source['attempt_valid'] is not True
                or source['any_collision'] or not source['completion_requested']):
            raise ValueError('automatic_valid_finish_not_observed')
        for v in vehicles:
            if len(crossings[v]) != 10 or v not in targets:
                raise ValueError('independent_lap_count_disagrees_'+v)
            if abs(source['vehicles'][v]['velocity'][0]) >= .02:
                raise ValueError('vehicle_not_stopped_'+v)
            accumulated = 0.
            for duration, crossing in zip(latest[v+'/laps']['lap_times_s'], crossings[v]):
                accumulated += duration
                if abs(accumulated-crossing['finish_sample_sim_s']) > .03:
                    raise ValueError('independent_crossing_time_disagrees_'+v)
            completed_events = [e for e in events[v] if e['event'] == 'lap_completed'
                                and e['attempt_id'] == source['attempt_id']]
            if len(completed_events) != 10:
                raise ValueError('lap_event_count_disagrees_'+v)
        first = vehicles[0]
        gate = latest[first+'/laps']['configuration']
        pose = source['vehicles'][first]['pose']
        along = (pose[0]-gate['gate_x'])*math.cos(gate['gate_yaw'])+(pose[1]-gate['gate_y'])*math.sin(gate['gate_yaw'])
        if not -.02 <= along <= .3:
            raise ValueError('primary_not_parked_at_finish_gate')
        before = (source['sim_time_ns'], [source['vehicles'][v]['pose'] for v in vehicles])
        spin(.5)
        after = (latest['sim']['sim_time_ns'], [latest['sim']['vehicles'][v]['pose'] for v in vehicles])
        if before != after:
            raise ValueError('physics_not_frozen_after_completion')
        active = False
        report['lap_events'] = events
        checkpoint('passed')
        print(json.dumps({'result': 'passed', 'completed_time_s': {
            v: latest[v+'/laps']['completed_time_s'] for v in vehicles},
            'distance_after_target_m': extra_distance}, indent=2), flush=True)
    except BaseException as exc:
        checkpoint('failed', str(exc))
        raise
    finally:
        active = False
        if report['result'] != 'passed':
            try:
                if latest.get('sim', {}).get('state') == 'running':
                    call('end')
                    wait(lambda: latest['sim']['state'] != 'running', timeout=5.)
                for v in vehicles:
                    call(v, False)
            except Exception as exc:
                report['cleanup_error'] = str(exc)
                checkpoint('failed', report.get('error'))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
