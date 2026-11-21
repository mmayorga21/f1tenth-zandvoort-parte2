# F1TENTH — Zandvoort, Part 2

## 1. Approach

This project uses Follow the Gap, a reactive navigation algorithm that selects the vehicle's steering and speed from LiDAR distance measurements.

The controller processes the readings, adds a safety margin around obstacles, and identifies contiguous areas of free space. It selects a target within the chosen gap and steers towards it. Speed is adjusted to the required turn and the available clearance ahead.

The vehicle responds to the observed surroundings without following a predefined driving trajectory. A separate lap tracker uses odometry to count complete circuits and measure lap times. In the two-vehicle scenario, a valid scan with no safe gap commands braking while the controller continues checking for free space. Invalid front readings, collisions and command loss still interrupt the attempt; a valid nearby obstacle triggers braking and continued checks for free space. In the two-vehicle integration, each vehicle runs its own Follow the Gap controller and lap tracker. The secondary vehicle starts five metres ahead and applies a factor of 0.5 to its controller's speed command.

## 2. Code structure

The main project files are organized as follows:

```text
F1Tenth-Repository-Parte2/
├── README.md
├── scenarios/                       # Scenario definitions and requirements
├── scripts/                         # Validation scripts
│   └── map_footprint.py              # Independent full-body collision check
└── src/
    ├── f1tenth_controller/
    │   ├── config/
    │   │   ├── follow_the_gap.yaml   # Original-track controller profile
    │   │   ├── conservative.yaml     # Original-track conservative profile
    │   │   ├── obstacles.yaml        # Fast single-vehicle obstacle-course profile
    │   │   ├── two_vehicles.yaml     # Two-vehicle controller profile
    │   │   ├── obstacles_conservative.yaml # 4 m/s obstacle-course reference
    │   │   └── lap_tracker.yaml      # Lap counter configuration
    │   ├── f1tenth_controller/
    │   │   ├── gap.py                # Follow the Gap algorithm
    │   │   ├── gap_node.py           # Controller ROS interface
    │   │   ├── control_gate.py       # Controller activation and deactivation
    │   │   ├── laps.py               # Lap counting and timing
    │   │   ├── lap_node.py           # Lap tracker ROS node
    │   │   ├── lap_panel.py          # Terminal lap and timing display
    │   │   ├── lap_display_node.py   # Additional read-only terminal viewer
    │   │   └── session_guard.py      # Attempt validation checks
    │   ├── launch/
    │   │   ├── follow_the_gap.launch.py # Single-vehicle controller
    │   │   └── two_vehicles.launch.py  # Simulator, two controllers and counters
    │   └── test/                     # Controller and lap counter tests
    └── f1tenth_gym_ros/
        ├── config/
        │   ├── sim.yaml             # Single-vehicle simulator configuration
        │   └── sim_two_vehicles.yaml # Two vehicles and five static obstacles
        ├── maps/
        │   ├── Zandvoort_map.png     # Original track image
        │   ├── Zandvoort_map.yaml    # Original map resolution and origin
        │   ├── Zandvoort_obstacles.png  # Track with five static obstacles
        │   └── Zandvoort_obstacles.yaml # Obstacle map metadata
        ├── f1tenth_gym_ros/
        │   ├── gym_bridge.py         # Simulator integration with ROS
        │   ├── dual_control.py       # Shared start and command freshness checks
        │   └── simulation_session.py # Attempt states and termination
        └── launch/
            ├── gym_bridge_launch.py # Simulator and RViz launch file
            ├── gym_bridge.rviz      # Single-vehicle visualization settings
            └── two_vehicles.rviz    # Map, two vehicles and independent LiDARs
```

The repository also retains packages from the original simulator. This project primarily uses `f1tenth_controller` and `f1tenth_gym_ros`.

## 3. Execution instructions

These instructions assume **Ubuntu 22.04, ROS 2 Humble, and F1TENTH Gym are installed**, together with `rosdep` and `colcon`.

Sections 3.2–3.5 run one vehicle. Section 3.6 runs two vehicles with five obstacles and automatic completion after ten laps by each vehicle.

### 3.1. Clone and prepare the project

Open a terminal and clone the repository:

```bash
cd ~
git clone https://github.com/mmayorga21/f1tenth-zandvoort-parte2.git F1Tenth-Repository-Parte2
cd ~/F1Tenth-Repository-Parte2
```

The simulator loads the bundled maps from the installed ROS package; no absolute map path is required.

Install the ROS dependencies and build the workspace:

```bash
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y

colcon build --symlink-install
source install/setup.bash
```

### 3.2. Start the simulator

In **terminal 1**, load the environment:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

For the original track, run:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

For the track with five obstacles, use this command instead:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py \
  map_path:="$PWD/src/f1tenth_gym_ros/maps/Zandvoort_obstacles"
```

Run only one simulator instance. Wait for RViz to display the selected map and the vehicle, and keep this terminal open.

https://github.com/user-attachments/assets/1d38aa8d-d977-45f5-8e1f-aa6f435003ab

### 3.3. Start the controller and lap tracker

Open **terminal 2** for the controller:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

For the original track, run:

```bash
ros2 launch f1tenth_controller follow_the_gap.launch.py
```

For the track with five obstacles, use its controller profile instead:

```bash
ros2 launch f1tenth_controller follow_the_gap.launch.py \
  params_file:="$PWD/src/f1tenth_controller/config/obstacles.yaml"
```

Run only one controller. It starts disabled, so the vehicle will remain stationary until explicitly enabled.

To use the validated 4 m/s reference profile, replace `obstacles.yaml` with `obstacles_conservative.yaml` in the command above.

https://github.com/user-attachments/assets/fda90ad6-1573-44f2-9abe-b83f9fd9939c

Open **terminal 3** for the lap counter and timer:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run f1tenth_controller lap_tracker --ros-args \
  --params-file install/f1tenth_controller/share/f1tenth_controller/config/lap_tracker.yaml
```

This terminal displays completed laps, individual lap times, the best lap, and the accumulated time. Keep the controller and lap tracker running throughout the attempt.

The recording below predates the English translation of the lap panel. Current terminal output is in English.

https://github.com/user-attachments/assets/42a19ed6-84ac-4e24-83c3-7dfc1afe9e51

### 3.4. Run the ten-lap attempt

Open **terminal 4**:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Reset the simulator to return the vehicle to its starting pose and prepare a new attempt:

```bash
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
```

After the reset completes, enable the controller:

```bash
ros2 service call /controller/enable std_srvs/srv/SetBool "{data: true}"
```

The vehicle will start driving autonomously. After ten valid laps, it will brake and the attempt will finish automatically. The panel will retain the lap times and display the completion message.

To run another attempt, repeat the reset and enable commands in that order.

The following video shows the start commands and the first seconds of driving:

https://github.com/user-attachments/assets/1b382e8a-57e8-459e-8228-c0e61c74e8b7

### 3.5. Stop execution

To stop the vehicle before completing ten laps:

```bash
ros2 service call /controller/enable std_srvs/srv/SetBool "{data: false}"
```

To close the programs, press `Ctrl+C` in the controller, lap tracker, and simulator terminals.

### 3.6. Two vehicles with five obstacles

Close the single-vehicle programs before starting this scenario. In terminal 1:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_controller two_vehicles.launch.py
```

This launches the obstacle map, RViz, both controllers and both lap trackers. Both controllers start disabled. The primary uses `two_vehicles.yaml`, with a 12 m/s speed limit; the secondary uses the same profile with its speed command multiplied by 0.5. Actual speed depends on each vehicle's LiDAR readings and turns. The two-vehicle profile uses more conservative steering, visibility and cornering limits, so its actual driving speed is lower than the fast single-vehicle profile.

Open terminals 2 and 3, source the same environment, then start one read-only lap display in each:

```bash
# Terminal 2: primary vehicle
ros2 run f1tenth_controller lap_display --ros-args \
  -r __ns:=/ego_racecar -r /laps/status:=/ego_racecar/laps/status
```

```bash
# Terminal 3: secondary vehicle
ros2 run f1tenth_controller lap_display --ros-args \
  -r __ns:=/opp_racecar -r /laps/status:=/opp_racecar/laps/status
```

In another terminal with the same environment, reset and enable both controllers:

```bash
ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
ros2 service call /ego_racecar/controller/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /opp_racecar/controller/enable std_srvs/srv/SetBool "{data: true}"
```

The shared attempt starts only after both controllers are ready. Each counter measures complete circuits from its own starting gate. A collision or loss of commands from either vehicle invalidates the shared attempt and freezes both vehicles.

After completing ten laps, the primary keeps driving at the secondary's nominal speed scale (0.5), reducing further overtakes while waiting. Its counter and lap times retain only its first ten laps. After both complete ten valid laps, both vehicles brake while continuing to steer with Follow the Gap, and the attempt closes automatically. The panels retain the results. Verify the overtake by observing the simulation.

To end an attempt early, request a controlled stop of both vehicles:

```bash
ros2 service call /sim/end_attempt std_srvs/srv/Trigger "{}"
```

To repeat the attempt, reset and enable both controllers again. Close the programs with `Ctrl+C`.
