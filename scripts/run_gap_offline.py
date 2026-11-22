#!/usr/bin/env python3
"""Run the exact controller core against F110 Gym without ROS overhead."""
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

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/f1tenth_controller'))
from f1tenth_controller.gap import FollowGap, GapParameters
from lap_validation import LapValidation


def main():
    root=Path(__file__).resolve().parents[1]
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default=str(root/'src/f1tenth_controller/config/follow_the_gap.yaml'))
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-sim-seconds', type=float, default=600.)
    args=parser.parse_args()
    config=yaml.safe_load(Path(args.config).read_text())['gap_follower']['ros__parameters']
    parameters=GapParameters(**{k:v for k,v in config.items() if k in GapParameters.__dataclass_fields__})
    core=FollowGap(parameters)
    sim_config=yaml.safe_load((root/'src/f1tenth_gym_ros/config/sim.yaml').read_text())['bridge']['ros__parameters']
    start=[sim_config[k] for k in ('sx','sy','stheta')]
    env=gym.make('f110_gym:f110-v0',map=sim_config['map_path'],map_ext='.png',num_agents=1)
    obs,_,_,_=env.reset(np.array([start]))
    validation=LapValidation(start)
    dt=env.unwrapped.timestep
    rows=[];stopped=0;wall_start=time.monotonic();reason='timeout'
    for step in range(int(args.max_sim_seconds/dt)):
        decision=core.decide(obs['scans'][0],-sim_config['scan_fov']/2,
                             sim_config['scan_fov']/sim_config['scan_beams'],0.,30.,dt)
        obs,_,_,_=env.step(np.array([[decision.steering,decision.speed]]))
        elapsed=(step+1)*dt
        x,y,yaw=[float(obs[k][0]) for k in ('poses_x','poses_y','poses_theta')]
        if step%10==0:
            rows.append({'time':elapsed,'x':x,'y':y,'yaw':yaw,**asdict(decision)})
        if bool(obs['collisions'][0]):
            reason='collision';break
        if validation.update(x,y,elapsed):
            reason='lap_complete';break
        stopped=stopped+1 if decision.speed<.05 else 0
        if stopped>500:
            reason='stalled';break
        if step%5000==0:
            print(json.dumps({'sim_s':elapsed,'pose':[x,y,yaw],
                              'checkpoints':validation.next_checkpoint,'reason':decision.reason}),flush=True)
    output={'result':reason,'elapsed_sim_s':elapsed,'wall_elapsed_s':time.monotonic()-wall_start,
            'collision':bool(obs['collisions'][0]),'validation':validation.summary(),
            'parameters':asdict(parameters),'config_sha256':hashlib.sha256(Path(args.config).read_bytes()).hexdigest(),
            'core_sha256':hashlib.sha256((root/'src/f1tenth_controller/f1tenth_controller/gap.py').read_bytes()).hexdigest(),
            'final_pose':[x,y,yaw],'last_decision':asdict(decision),'trajectory':rows}
    target=Path(args.output);target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({k:v for k,v in output.items() if k!='trajectory'},indent=2),flush=True)
    env.close()


if __name__=='__main__':
    main()
