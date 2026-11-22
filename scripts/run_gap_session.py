#!/usr/bin/env python3
"""Run consecutive laps with the unchanged controller/counter, without ROS."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time

import gym
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/f1tenth_controller'))
sys.path.insert(0, str(ROOT/'src/f1tenth_gym_ros'))
from f1tenth_controller.gap import FollowGap, GapParameters
from f1tenth_controller.laps import LapCounter, LapParameters
from f1tenth_gym_ros.simulation_session import SimulationSession
from lap_validation import LapValidation
from map_footprint import MapFootprint


def write_report(path, report):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(report, indent=2)+'\n')
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--laps', type=int, required=True)
    parser.add_argument('--seed', type=int, default=12345)
    parser.add_argument('--output', required=True)
    parser.add_argument('--config', default=str(ROOT/'src/f1tenth_controller/config/follow_the_gap.yaml'))
    parser.add_argument('--map-path', help='Map path without image or YAML extension')
    args = parser.parse_args()
    if args.laps < 1 or args.seed < 0:
        parser.error('laps must be positive and seed nonnegative')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    gap_file = Path(args.config)
    lap_file = ROOT/'src/f1tenth_controller/config/lap_tracker.yaml'
    gap_config = yaml.safe_load(gap_file.read_text())['gap_follower']['ros__parameters']
    lap_config = yaml.safe_load(lap_file.read_text())['lap_tracker']['ros__parameters']
    gap_parameters = GapParameters(**{k: v for k, v in gap_config.items()
                                     if k in GapParameters.__dataclass_fields__})
    values = {k: v for k, v in lap_config.items() if k in LapParameters.__dataclass_fields__}
    flat = lap_config['checkpoint_regions']
    values.update(target_laps=args.laps,
                  checkpoints=tuple(tuple(flat[i:i+3]) for i in range(0, len(flat), 3)))
    counter = LapCounter(LapParameters(**values))
    controller = FollowGap(gap_parameters)
    sim = yaml.safe_load((ROOT/'src/f1tenth_gym_ros/config/sim.yaml').read_text())['bridge']['ros__parameters']
    if args.map_path:
        sim['map_path'] = str(Path(args.map_path).expanduser().resolve())
    start = [sim[key] for key in ('sx', 'sy', 'stheta')]
    env = gym.make('f110_gym:f110-v0', map=sim['map_path'], map_ext=sim['map_img_ext'],
                   num_agents=1, seed=args.seed)
    obs, _, _, _ = env.reset(np.array([start]))
    footprint = MapFootprint(sim['map_path'], env.unwrapped.params['length'],
                             env.unwrapped.params['width'])
    footprint_collision = footprint.collides(*start)
    if footprint_collision:
        raise ValueError('Vehicle footprint overlaps the map at the initial pose')
    assert len(obs['scans'][0]) == sim['scan_beams']
    dt = float(env.unwrapped.timestep)
    source = SimulationSession(dt)
    counter.on_status(source.snapshot())
    reference = LapValidation(start)
    reference_finishes, rows = [], []
    wall_start = time.monotonic()
    result, failure = 'running', None
    report = {'mode': 'direct_f110_gym', 'seed': args.seed, 'target_laps': args.laps,
              'map_path': sim['map_path'],
              'footprint_checker_sha256': hashlib.sha256((ROOT/'scripts/map_footprint.py').read_bytes()).hexdigest(),
              'gap_parameters': asdict(gap_parameters),
              'gap_config_sha256': hashlib.sha256(gap_file.read_bytes()).hexdigest(),
              'map_image_sha256': hashlib.sha256(Path(sim['map_path']+sim['map_img_ext']).read_bytes()).hexdigest(),
              'controller_core_sha256': hashlib.sha256((ROOT/'src/f1tenth_controller/f1tenth_controller/gap.py').read_bytes()).hexdigest(),
              'counter_core_sha256': hashlib.sha256((ROOT/'src/f1tenth_controller/f1tenth_controller/laps.py').read_bytes()).hexdigest()}
    last_count = 0
    peak_actual_speed = peak_commanded_speed = 0.0
    try:
        for step in range(int((args.laps*400.+10.)/dt)):
            command = controller.decide(obs['scans'][0], -sim['scan_fov']/2,
                                         sim['scan_fov']/sim['scan_beams'], 0., 30., dt)
            if command.reason != 'following_gap':
                failure = command.reason
                break
            if source.state == 'ready' and command.speed > 0:
                source.start()
                counter.on_status(source.snapshot())
                counter.on_pose(source.time_ns, float(obs['poses_x'][0]), float(obs['poses_y'][0]))
            # Native done also includes its short lap heuristic. Collision is
            # the termination signal; our own counter validates full laps.
            obs, _, _, _ = env.step(np.array([[command.steering, command.speed]]))
            peak_commanded_speed = max(peak_commanded_speed, command.speed)
            peak_actual_speed = max(peak_actual_speed, abs(float(obs['linear_vels_x'][0])))
            collision = bool(obs['collisions'][0])
            footprint_collision = footprint.collides(
                float(obs['poses_x'][0]), float(obs['poses_y'][0]), float(obs['poses_theta'][0]))
            source.step(collision=collision)
            counter.on_status(source.snapshot())
            x, y = float(obs['poses_x'][0]), float(obs['poses_y'][0])
            counter.on_pose(source.time_ns, x, y)
            reference.update(x, y, source.time_ns/1e9)
            if step % 50 == 0:
                rows.append({'sim_time_s': source.time_ns/1e9, 'x': x, 'y': y,
                             'lap_count': len(counter.laps)})
            if collision or footprint_collision or counter.valid is not True:
                failure = 'map_footprint_collision' if footprint_collision else counter.reason
                break
            if len(counter.laps) != last_count:
                assert len(counter.laps) == last_count+1 and reference.complete
                assert abs(reference.previous_signed) < 0.1
                cross = counter.laps[-1]['finish_sim_time_ns']/1e9
                assert 0 <= source.time_ns/1e9-cross <= dt+1e-8
                reference_finishes.append(source.time_ns/1e9)
                last_count = len(counter.laps)
                progress = {'seed': args.seed, 'laps': last_count,
                            'last_sim_s': counter.laps[-1]['duration_s'],
                            'total_sim_s': counter.snapshot()['completed_time_s']}
                print(json.dumps(progress), flush=True)
                report.update(result='running', measurement=counter.snapshot(),
                              reference_finishes_sim_s=reference_finishes,
                              wall_elapsed_s=time.monotonic()-wall_start)
                write_report(output, report)
                if last_count == args.laps:
                    result = 'passed'
                    break
                reference = LapValidation(start)
            counter.events.clear()
        if result != 'passed':
            result = 'failed'
            failure = failure or 'time_limit'
            source.abort(failure)
        else:
            source.end()
        counter.on_status(source.snapshot())
    finally:
        report.update(result=result, failure=failure, measurement=counter.snapshot(),
                      collision=bool(obs['collisions'][0]),
                      footprint_collision=footprint_collision, footprint_samples=footprint.samples,
                      peak_actual_speed_mps=peak_actual_speed,
                      peak_commanded_speed_mps=peak_commanded_speed,
                      reference_finishes_sim_s=reference_finishes,
                      wall_elapsed_s=time.monotonic()-wall_start, trajectory=rows,
                      final_pose=[float(obs[k][0]) for k in ('poses_x','poses_y','poses_theta')],
                      last_decision=asdict(command))
        write_report(output, report)
        env.close()
    print(json.dumps({k: v for k, v in report.items() if k not in
                     ('trajectory', 'gap_parameters', 'measurement')}, indent=2), flush=True)
    if result != 'passed':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
