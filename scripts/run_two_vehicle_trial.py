#!/usr/bin/env python3
"""Exercise both controllers with deterministic LiDAR noise, without ROS timing."""
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
from f1tenth_gym_ros.dual_control import following_pose
from f1tenth_gym_ros.simulation_session import SimulationSession
from lap_validation import LapValidation
from map_footprint import MapFootprint, rectangles_overlap


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed', type=int, default=12345)
    parser.add_argument('--warmup-steps', type=int, default=0)
    parser.add_argument('--output', required=True)
    parser.add_argument('--config', default=str(ROOT/'src/f1tenth_controller/config/two_vehicles.yaml'))
    args = parser.parse_args()
    if args.seed < 0 or args.warmup_steps < 0:
        parser.error('Seed and warmup steps must be nonnegative')
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    sim = yaml.safe_load((ROOT/'src/f1tenth_gym_ros/config/sim_two_vehicles.yaml').read_text())['bridge']['ros__parameters']
    gap = yaml.safe_load(Path(args.config).read_text())['gap_follower']['ros__parameters']
    lap = yaml.safe_load((ROOT/'src/f1tenth_controller/config/lap_tracker.yaml').read_text())['lap_tracker']['ros__parameters']
    parameters = GapParameters(**{**{k: v for k, v in gap.items() if k in GapParameters.__dataclass_fields__},
                               'yield_on_blocked_gap': True})
    starts = [[sim[k] for k in ('sx', 'sy', 'stheta')]]
    starts.append(following_pose(starts[0], sim['opponent_start_offset_m']))
    names, scales = ('ego_racecar', 'opp_racecar'), (1., .5)
    map_path = str(ROOT/'src/f1tenth_gym_ros/maps'/sim['map_path'])
    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=2, seed=args.seed)
    obs, _, _, _ = env.reset(np.array(starts))
    length, width = (env.unwrapped.params[k] for k in ('length', 'width'))
    footprint = MapFootprint(map_path, length, width)
    dt = float(env.unwrapped.timestep)
    for _ in range(args.warmup_steps):
        obs, _, _, _ = env.step(np.zeros((2, 2)))
    source = SimulationSession(dt)
    controllers, counters, references = [], [], []
    for start in starts:
        flat = lap['checkpoint_regions']
        p = {k: v for k, v in lap.items() if k in LapParameters.__dataclass_fields__}
        p.update(gate_x=start[0], gate_y=start[1], gate_yaw=start[2], stop_counting_at_target=True,
                 checkpoints=tuple(tuple(flat[i:i+3]) for i in range(0, len(flat), 3)))
        counters.append(LapCounter(LapParameters(**p)))
        counters[-1].on_status(source.snapshot())
        references.append(LapValidation(start))
        controllers.append(FollowGap(parameters))
    independent, yields = [[], []], [0, 0]
    wall_start = time.monotonic()
    report = {'result': 'running', 'mode': 'direct_dual_gym_not_ros', 'seed': args.seed,
              'warmup_steps': args.warmup_steps, 'parameters': asdict(parameters),
              'post_target_speed_scale': .5,
              'trial_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'core_sha256': hashlib.sha256((ROOT/'src/f1tenth_controller/f1tenth_controller/gap.py').read_bytes()).hexdigest()}

    def save(result='running', error=None):
        report.update(result=result, error=error, measurements={v: c.snapshot() for v, c in zip(names, counters)},
                      source=source.snapshot(), waiting_for_gap_steps=yields,
                      independent_crossings=independent, footprint_samples=footprint.samples,
                      wall_elapsed_s=time.monotonic()-wall_start)
        report['observation'] = {k: np.asarray(obs[k]).tolist() for k in
            ('poses_x', 'poses_y', 'poses_theta', 'linear_vels_x', 'collisions', 'scans')}
        output.write_text(json.dumps(report, indent=2)+'\n')

    try:
        source.start()
        for c in counters:
            c.on_status(source.snapshot())
        for step in range(round(1600./dt)):
            actions = []
            for i, (controller, scale) in enumerate(zip(controllers, scales)):
                d = controller.decide(obs['scans'][i], -sim['scan_fov']/2,
                    sim['scan_fov']/(sim['scan_beams']-1), 0., 30., dt)
                if d.reason not in ('following_gap', 'waiting_for_gap'):
                    raise ValueError(names[i]+': '+d.reason)
                yields[i] += int(d.reason == 'waiting_for_gap')
                speed = d.speed*(.5 if counters[i].snapshot()['target_reached'] else scale)
                if source.completion_requested:
                    speed = min(speed, max(0., abs(float(obs['linear_vels_x'][i]))-sim['stopping_deceleration_mps2']*dt))
                actions.append([d.steering, speed])
            obs, _, _, _ = env.step(np.array(actions))
            source.step(collision=bool(np.any(obs['collisions'])))
            poses = [[float(obs[k][i]) for k in ('poses_x', 'poses_y', 'poses_theta')] for i in range(2)]
            if source.collision or any(footprint.collides(*pose) for pose in poses) or rectangles_overlap(*poses, length, width):
                raise ValueError('native_or_full_body_collision')
            for i, (counter, pose) in enumerate(zip(counters, poses)):
                before = len(counter.laps)
                counter.on_status(source.snapshot())
                counter.on_pose(source.time_ns, *pose[:2])
                if counter.valid is not True:
                    raise ValueError(names[i]+': '+counter.reason)
                if counter.snapshot()['current_lap_time_s'] > 180.:
                    raise ValueError(names[i]+': lap_time_limit')
                if len(independent[i]) < 10 and references[i].update(*pose[:2], source.time_ns/1e9):
                    independent[i].append(source.time_ns/1e9)
                    references[i] = LapValidation(starts[i])
                if len(counter.laps) != before:
                    print(json.dumps({'seed': args.seed, 'warmup': args.warmup_steps,
                        'vehicle': names[i], 'laps': len(counter.laps),
                        'total_s': counter.snapshot()['completed_time_s']}), flush=True)
                    save()
                counter.events.clear()
            source.request_joint_completion({v: {**c.snapshot(), 'vehicle_id': v} for v, c in zip(names, counters)}, names, 10)
            if source.finish_if_stopped(max(abs(float(v)) for v in obs['linear_vels_x'])):
                for counter, crossings in zip(counters, independent):
                    if len(crossings) != 10 or abs(counter.snapshot()['completed_time_s']-crossings[-1]) > .03:
                        raise ValueError('independent_lap_measurement_disagrees')
                save('passed')
                return
        raise ValueError('simulation_time_limit')
    except BaseException as exc:
        save('failed', str(exc))
        raise
    finally:
        env.close()


if __name__ == '__main__':
    main()
