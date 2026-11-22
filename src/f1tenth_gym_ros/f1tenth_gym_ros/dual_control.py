"""Shared start barrier and independent command/controller freshness checks."""
import math


def following_pose(pose, distance):
    """Place the second center ahead along the initial straight's heading."""
    x, y, yaw = pose
    if not math.isfinite(distance) or distance <= 0:
        raise ValueError('Starting separation must be positive and finite')
    return [x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw]


def finish_speed_limit(pose, start, laps, target, deceleration, stop_offset=.1):
    """Brake on the final straight and hold just beyond the own finish gate.

    Checkpoints prevent a nearby, earlier section of the circuit from triggering
    finish braking. The small positive offset lets the lap counter observe the
    crossing before the car comes to rest. Physics and collisions remain active.
    """
    if laps.get('target_reached') is True:
        return 0.
    if (laps.get('lap_count') != target-1 or not laps.get('checkpoint_count')
            or laps.get('checkpoints_passed') != laps['checkpoint_count']):
        return math.inf
    x, y, yaw = start
    dx, dy = pose[0]-x, pose[1]-y
    along = dx*math.cos(yaw)+dy*math.sin(yaw)
    lateral = -dx*math.sin(yaw)+dy*math.cos(yaw)
    if not -25. <= along <= stop_offset+1. or abs(lateral) > 1.3:
        return math.inf
    distance = max(0., stop_offset-along)
    # Include velocity-servo response instead of assuming instantaneous braking.
    response = deceleration*.06
    return max(0., math.sqrt(response*response+2*deceleration*distance)-response)


class DualControlGate:
    def __init__(self, vehicles, drive_timeout=.3, status_timeout=.5):
        self.vehicles = tuple(vehicles)
        if len(self.vehicles) != 2 or len(set(self.vehicles)) != 2:
            raise ValueError('Two distinct vehicle identifiers are required')
        self.drive_timeout, self.status_timeout = drive_timeout, status_timeout
        self.reset(None)

    def reset(self, identity):
        self.identity = identity
        self.released = False
        self.drive_received = dict.fromkeys(self.vehicles, -math.inf)
        self.status_received = dict.fromkeys(self.vehicles, -math.inf)
        self.armed = dict.fromkeys(self.vehicles, False)

    def on_drive(self, vehicle, now):
        self.drive_received[vehicle] = now

    def on_controller(self, vehicle, status, now):
        self.status_received[vehicle] = now
        config = status.get('configuration') or {}
        self.armed[vehicle] = (tuple(status.get('identity') or ()) == self.identity
                               and status.get('armed') is True and isinstance(config, dict)
                               and config.get('vehicle_id', vehicle) == vehicle)

    def health(self, now, publishers):
        for vehicle in self.vehicles:
            if publishers.get(vehicle) != 1:
                return vehicle + '_drive_publisher_count'
            if now-self.drive_received[vehicle] > self.drive_timeout:
                return vehicle + '_drive_timeout'
            if now-self.status_received[vehicle] > self.status_timeout:
                return vehicle + '_controller_timeout'
            if not self.armed[vehicle]:
                return vehicle + '_controller_disabled'
        return None
