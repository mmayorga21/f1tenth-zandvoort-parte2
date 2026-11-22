"""Both controllers must be live and belong to the current session."""
import math

import pytest

from f1tenth_gym_ros.dual_control import DualControlGate, following_pose, finish_speed_limit


def test_finish_braking_only_on_last_lap_after_all_checkpoints():
    final = {'lap_count': 9, 'checkpoints_passed': 6, 'checkpoint_count': 6}
    start = [0., 0., 0.]
    assert math.isinf(finish_speed_limit([-8., 0., 0.], start, {**final, 'lap_count': 8}, 10, 6.))
    assert math.isinf(finish_speed_limit([-8., 0., 0.], start, {**final, 'checkpoints_passed': 5}, 10, 6.))
    assert math.isinf(finish_speed_limit([-8., 4., 0.], start, final, 10, 6.))
    assert 0 < finish_speed_limit([-.1, 0., 0.], start, final, 10, 6.) < finish_speed_limit([-8., 0., 0.], start, final, 10, 6.)
    assert finish_speed_limit([.1, 0., 0.], start, final, 10, 6.) == 0.


def test_completed_vehicle_stays_stopped_and_reset_releases_finish_limit():
    start = [0., 0., 0.]
    assert finish_speed_limit([.05, .2, 0.], start, {'target_reached': True}, 10, 6.) == 0.
    assert math.isinf(finish_speed_limit([.05, .2, 0.], start, {}, 10, 6.))


def ready_gate():
    gate = DualControlGate(('ego', 'opp'))
    gate.reset(('run', 'session'))
    for vehicle in gate.vehicles:
        gate.on_drive(vehicle, 0.)
        gate.on_controller(vehicle, {'identity': ['run', 'session'], 'armed': True}, 0.)
    return gate


def test_offset_is_exactly_five_meters_with_the_same_heading():
    pose = [0., 0., 1.1954730407943466]
    other = following_pose(pose, 5.)
    assert math.dist(pose[:2], other[:2]) == pytest.approx(5.)
    assert other[2] == pose[2]
    assert (other[0]*math.cos(pose[2]) + other[1]*math.sin(pose[2])) == pytest.approx(5.)


def test_one_controller_is_not_enough_for_shared_start():
    gate = DualControlGate(('ego', 'opp'))
    gate.reset(('run', 'session'))
    gate.on_drive('ego', 0.)
    gate.on_controller('ego', {'identity': ['run', 'session'], 'armed': True}, 0.)
    assert gate.health(0., {'ego': 1, 'opp': 1}) == 'opp_drive_timeout'
    assert ready_gate().health(0., {'ego': 1, 'opp': 1}) is None


@pytest.mark.parametrize('vehicle', ['ego', 'opp'])
def test_loss_of_either_command_stream_is_rejected(vehicle):
    gate = ready_gate()
    gate.on_drive('opp' if vehicle == 'ego' else 'ego', .29)
    assert gate.health(.31, {'ego': 1, 'opp': 1}) == vehicle + '_drive_timeout'


def test_stale_controller_state_is_rejected_even_with_fresh_commands():
    gate = ready_gate()
    for vehicle in gate.vehicles:
        gate.on_drive(vehicle, .6)
    assert gate.health(.6, {'ego': 1, 'opp': 1}) == 'ego_controller_timeout'


def test_a_retained_state_from_an_old_session_cannot_arm_the_barrier():
    gate = ready_gate()
    gate.on_controller('opp', {'identity': ['run', 'old'], 'armed': True}, 0.)
    assert gate.health(0., {'ego': 1, 'opp': 1}) == 'opp_controller_disabled'
    gate.released = True
    gate.reset(('run', 'new'))
    assert not gate.released
    assert not any(gate.armed.values())


def test_duplicate_publishers_are_rejected_per_vehicle():
    assert ready_gate().health(0., {'ego': 1, 'opp': 2}) == 'opp_drive_publisher_count'


@pytest.mark.parametrize('vehicles', [('ego', 'ego'), ('ego',)])
def test_vehicle_identifiers_must_be_distinct(vehicles):
    with pytest.raises(ValueError):
        DualControlGate(vehicles)


def test_a_controller_for_the_wrong_vehicle_cannot_arm_the_barrier():
    gate = ready_gate()
    gate.on_controller('opp', {'identity': ['run', 'session'], 'armed': True,
                              'configuration': {'vehicle_id': 'ego'}}, 0.)
    assert gate.health(0., {'ego': 1, 'opp': 1}) == 'opp_controller_disabled'
