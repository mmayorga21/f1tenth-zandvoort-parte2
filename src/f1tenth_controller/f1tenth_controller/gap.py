"""Reactive Follow the Gap, independent of ROS, map and vehicle pose."""
from dataclasses import dataclass, asdict
import math

import numpy as np


@dataclass
class GapParameters:
    front_half_angle_rad: float = 1.5707963267948966
    max_range_m: float = 6.0
    safety_radius_m: float = 0.30
    disparity_threshold_m: float = 0.45
    free_threshold_m: float = 1.0
    minimum_gap_angle_rad: float = 0.06
    filter_window: int = 3
    target_window: int = 31
    heading_weight: float = 0.12
    continuity_weight: float = 0.08
    wheelbase_m: float = 0.3302
    lookahead_m: float = 1.4
    max_steering_rad: float = 0.4189
    max_steering_rate_rad_s: float = 2.0
    steering_time_constant_s: float = 0.08
    max_speed_mps: float = 1.2
    min_speed_mps: float = 0.65
    max_acceleration_mps2: float = 1.0
    emergency_front_distance_m: float = 0.45
    front_clearance_half_angle_rad: float = 0.14
    lateral_acceleration_limit_mps2: float = 0.0
    braking_deceleration_mps2: float = 0.0
    yield_on_blocked_gap: bool = False

    def validate(self):
        values = asdict(self)
        if not all(math.isfinite(float(v)) for v in values.values()):
            raise ValueError('Parameters must be finite')
        optional = ('heading_weight', 'continuity_weight', 'lateral_acceleration_limit_mps2',
                    'braking_deceleration_mps2', 'yield_on_blocked_gap')
        if type(self.yield_on_blocked_gap) is not bool:
            raise ValueError('yield_on_blocked_gap must be boolean')
        if any(v <= 0 for k, v in values.items() if k not in optional):
            raise ValueError('Distances, limits and windows must be positive')
        if self.lateral_acceleration_limit_mps2 < 0 or self.braking_deceleration_mps2 < 0:
            raise ValueError('Optional speed limits cannot be negative')
        if (self.lateral_acceleration_limit_mps2 > 0) != (self.braking_deceleration_mps2 > 0):
            raise ValueError('Curvature and braking limits must be enabled together')
        if not 0 < self.min_speed_mps <= self.max_speed_mps:
            raise ValueError('Invalid speed interval')
        if self.front_half_angle_rad > math.pi / 2 + 1e-6:
            raise ValueError('Only forward targets are supported')
        if self.filter_window % 2 != 1 or self.target_window % 2 != 1:
            raise ValueError('Filter windows must be odd')
        if self.free_threshold_m >= self.max_range_m:
            raise ValueError('Free threshold must be less than maximum range')


@dataclass
class Decision:
    speed: float = 0.0
    steering: float = 0.0
    reason: str = 'invalid_scan'
    target_angle: float = 0.0
    target_range: float = 0.0
    gap_start_angle: float = 0.0
    gap_end_angle: float = 0.0
    front_clearance: float = 0.0


class FollowGap:
    def __init__(self, parameters=None):
        self.p = parameters or GapParameters()
        self.p.validate()
        self.reset()

    def reset(self):
        self.previous_target = 0.0
        self.previous_steering = 0.0
        self.previous_speed = 0.0

    def stop(self, reason, front=0.0):
        self.reset()
        return Decision(reason=reason, front_clearance=front)

    def decide(self, ranges, angle_min, angle_increment, range_min, range_max, dt):
        p = self.p
        raw = np.asarray(ranges, dtype=float)
        if (raw.ndim != 1 or raw.size < 10 or not math.isfinite(angle_min)
                or not math.isfinite(angle_increment) or angle_increment <= 0
                or not math.isfinite(range_max) or range_max <= 0
                or not math.isfinite(range_min) or range_min < 0
                or not math.isfinite(dt) or dt <= 0):
            return self.stop('invalid_scan')
        all_angles = angle_min + np.arange(raw.size) * angle_increment
        sector = np.abs(all_angles) <= p.front_half_angle_rad
        angles, raw = all_angles[sector], raw[sector]
        if len(raw) < 10 or not np.any(np.abs(angles) < p.front_clearance_half_angle_rad):
            return self.stop('missing_front_sector')
        # Positive infinity means no return up to sensor range. NaN, negative
        # infinity and too-small measurements never create free space.
        invalid = np.isnan(raw) | np.isneginf(raw) | (raw <= range_min)
        clean = np.minimum(np.nan_to_num(raw, nan=0.0, neginf=0.0, posinf=range_max), min(range_max, p.max_range_m))
        clean[invalid] = 0.0
        front = float(np.min(clean[np.abs(angles) <= p.front_clearance_half_angle_rad]))
        braking_reason = None
        if front < p.emergency_front_distance_m:
            front_invalid = np.any(invalid[np.abs(angles) <= p.front_clearance_half_angle_rad])
            if p.yield_on_blocked_gap and not front_invalid:
                braking_reason = 'waiting_for_gap'
            else:
                return self.stop('front_obstacle_or_invalid_data', front)
        # A minimum filter keeps a narrow obstacle instead of averaging it away.
        half = p.filter_window // 2
        padded = np.pad(clean, (half, half), mode='edge')
        filtered = np.min(np.lib.stride_tricks.sliding_window_view(padded, p.filter_window), axis=1)
        extended = filtered.copy()
        # At a discontinuity, extend the near obstacle onto the farther side.
        for i in np.flatnonzero(np.abs(np.diff(filtered)) > p.disparity_threshold_m):
            near = min(filtered[i], filtered[i+1])
            width = int(math.ceil(math.asin(min(1.0, p.safety_radius_m / max(near, 1e-6))) / angle_increment))
            if filtered[i] < filtered[i+1]:
                start, end = i+1, min(len(extended), i+width+2)
            else:
                start, end = max(0, i-width), i+1
            extended[start:end] = np.minimum(extended[start:end], near)
        closest = int(np.argmin(filtered))
        width = int(math.ceil(math.asin(min(1.0, p.safety_radius_m / max(filtered[closest], 1e-6))) / angle_increment))
        extended[max(0, closest-width):min(len(extended), closest+width+1)] = 0.0
        free = extended >= p.free_threshold_m
        transitions = np.diff(np.r_[False, free, False].astype(int))
        starts, ends = np.flatnonzero(transitions == 1), np.flatnonzero(transitions == -1)
        best = None
        for start, end in zip(starts, ends):
            if (end-start)*angle_increment < p.minimum_gap_angle_rad:
                continue
            window = min(p.target_window, end-start)
            # Padding with zero discourages a target right at the gap's edge.
            depth = np.convolve(extended[start:end], np.ones(window)/window, mode='same')
            local_angles = angles[start:end]
            score = (depth / p.max_range_m - p.heading_weight*np.abs(local_angles)
                     - p.continuity_weight*np.abs(local_angles-self.previous_target))
            idx = int(np.argmax(score))
            candidate = (float(score[idx]), start+idx, start, end-1)
            if best is None or candidate[0] > best[0]:
                best = candidate
        if best is None:
            if p.yield_on_blocked_gap:
                # A valid scan can be temporarily blocked by another car.
                # Brake without relinquishing control; reevaluate the next scan.
                score = (filtered/p.max_range_m-p.heading_weight*np.abs(angles)
                         -p.continuity_weight*np.abs(angles-self.previous_target))
                target = int(np.argmax(score))
                best = (float(score[target]), target, target, target)
                braking_reason = 'waiting_for_gap'
            else:
                return self.stop('no_safe_gap', front)
        _, target, start, end = best
        target_angle = float(angles[target])
        target_range = float(filtered[target] if braking_reason and start == end else extended[target])
        distance = min(p.lookahead_m, target_range)
        desired = math.atan2(2*p.wheelbase_m*math.sin(target_angle), distance)
        desired = float(np.clip(desired, -p.max_steering_rad, p.max_steering_rad))
        alpha = dt / (p.steering_time_constant_s + dt)
        steering = self.previous_steering + float(np.clip(
            alpha*(desired-self.previous_steering),
            -p.max_steering_rate_rad_s*dt, p.max_steering_rate_rad_s*dt))
        speed = self.speed_limit(desired, steering, front, dt)
        if braking_reason:
            speed = 0.
        self.previous_target, self.previous_steering, self.previous_speed = target_angle, steering, speed
        return Decision(float(speed), float(steering), braking_reason or 'following_gap', target_angle,
                        target_range, float(angles[start]), float(angles[end]), front)

    def speed_limit(self, desired, steering, front, dt):
        p = self.p
        if p.lateral_acceleration_limit_mps2 > 0:
            # Limit for both the upcoming and current commanded curvature:
            # do not accelerate while the wheels are still unwinding a turn.
            curvature = max(abs(math.tan(desired)), abs(math.tan(steering)))/p.wheelbase_m
            corner_speed = math.sqrt(p.lateral_acceleration_limit_mps2/max(curvature, 1e-6))
            stopping_distance = max(0.0, front-p.emergency_front_distance_m)
            braking_speed = math.sqrt(2*p.braking_deceleration_mps2*stopping_distance)
            speed = min(p.max_speed_mps, corner_speed, braking_speed)
        else:
            # Keep the validated conservative policy exactly reproducible.
            turn_fraction = abs(desired)/p.max_steering_rad
            speed = p.max_speed_mps-(p.max_speed_mps-p.min_speed_mps)*turn_fraction
            speed = min(speed, max(0.0, (front-p.emergency_front_distance_m)*0.8))
        return min(speed, self.previous_speed+p.max_acceleration_mps2*dt)
