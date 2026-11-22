"""Plain terminal presentation; does not control or measure the vehicle."""
import math


STATES = {'waiting': 'Waiting for simulator', 'ready': 'Ready',
          'running': 'Running', 'ended': 'Finished', 'invalid': 'INVALID ATTEMPT'}
REASONS = {
    'waiting_for_simulator': 'Waiting for simulator data',
    'waiting_for_attempt': 'Enable the controller to start',
    'measuring': 'A full circuit and a forward finish-line crossing are required',
    'attempt_start_not_observed': 'Tracker started too late: reset the simulator',
    'session_reset': 'The session was reset',
    'collision': 'A collision was detected',
    'odometry_gap': 'Missing odometry samples: reset the simulator',
    'odometry_missing_at_start': 'Start was not observed: reset the simulator',
    'position_jump': 'Position jump detected: reset the simulator',
    'simulation_time_reversed': 'Simulation time moved backwards without a session reset',
    'odometry_time_reversed': 'Odometry time moved backwards',
    'source_status_timeout': 'Simulator status lost: reset the simulator',
    'odometry_timeout': 'Odometry lost: reset the simulator',
    'invalid_odometry': 'Invalid odometry: reset the simulator',
    'unexpected_odometry_frame': 'Odometry is not in the configured reference frame',
    'start_outside_finish_area': 'Starting pose is outside the configured finish area',
    'drive_timeout': 'Attempt interrupted: drive commands were lost',
    'shutdown': 'Attempt interrupted: simulator shut down',
    'manual_stop': 'Attempt ended manually',
    'target_laps_completed': 'Attempt finished automatically: target laps completed',
    'invalid_simulator_status': 'Invalid simulator status: reset the simulator',
    'odometry_buffer_overrun': 'Odometry buffer overflow: reset the simulator',
    'attempt_changed_without_reset': 'Attempt changed without a session reset',
    'attempt_interrupted': 'Attempt interrupted',
    'attempt_ended': 'Attempt ended',
    'tracker_shutdown': 'Attempt interrupted: lap tracker shut down',
}


def format_time(seconds):
    if seconds is None or not math.isfinite(seconds):
        return '--:--.--'
    hundredths = max(0, round(seconds*100))
    minutes, remainder = divmod(hundredths, 6000)
    whole_seconds, fraction = divmod(remainder, 100)
    return f'{minutes:02d}:{whole_seconds:02d}.{fraction:02d}'


def format_panel(status, width=78, height=26):
    title = STATES.get(status['state'], status['state'])
    if status['state'] == 'running' and status['paused']:
        title = 'PAUSED'
    if status['target_reached']:
        title += ' | TARGET REACHED'
    lines = [
        'F1TENTH - ZANDVOORT | LAP COUNTER AND TIMER',
        f'State: {title}',
        f'Completed laps: {status["lap_count"]} / {status["target_laps"]}',
        f'Current lap: {format_time(status["current_lap_time_s"])}',
        f'Last: {format_time(status["last_lap_time_s"])}    Best: {format_time(status["best_lap_time_s"])}',
        f'Attempt time: {format_time(status["elapsed_sim_s"])}',
        f'Completed-lap total: {format_time(status["completed_time_s"])}',
        f'Checkpoints passed: {status["checkpoints_passed"]} / {status["checkpoint_count"]}',
        '', '   LAP       TIME',
    ]
    if status.get('vehicle_id'):
        lines[0] = 'F1TENTH - ZANDVOORT | ' + status['vehicle_id'] + ' | LAP TIMER'
    available = max(1, height-len(lines)-3)
    times = status['lap_times_s']
    start = max(0, len(times)-available)
    if not times:
        lines.append('No completed laps')
    else:
        for index in range(start, len(times)):
            lines.append(f'{index+1:>6}       {format_time(times[index])}')
    reason = REASONS.get(status['reason'], status['reason'])
    if status['state'] == 'ready' and status.get('awaiting_joint_start'):
        reason = 'Enable both controllers to start together'
    elif status.get('vehicle_id') and status['state'] == 'running' and status['target_reached']:
        reason = 'Target completed; waiting for the other vehicle'
    lines += ['', reason]
    if status['pause_used']:
        lines.append('This attempt was paused')
    return '\n'.join(line[:max(20, width)] for line in lines[:max(12, height)])
