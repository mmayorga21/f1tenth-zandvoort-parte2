# F1TENTH — Zandvoort, Part 2

## 1. Approach

This project uses **Follow the Gap** to drive two vehicles on the Zandvoort track with five static obstacles. Each vehicle processes its LiDAR measurements, adds a safety margin around obstacles, selects a target within a free gap, and adjusts its steering and speed to the available space.

The primary vehicle must overtake the secondary vehicle, which starts **five metres ahead** and applies a factor of **0.5** to its own controller's speed command. Both vehicles use the same algorithm and must complete **ten consecutive laps without collision**.

Each vehicle has an independent lap tracker that uses odometry and ordered checkpoints to count complete circuits and measure lap times. The primary brakes on its final approach and stops just beyond its finish line after completing ten laps. The secondary continues until it completes its own ten laps, and the session then closes automatically.

## 2. Code structure

Main files used by the two-vehicle scenario:

```text
F1Tenth-Repository-Parte2/
├── README.md
├── scenarios/                         # Scenario definitions
├── scripts/                           # Validation scripts
└── src/
    ├── f1tenth_controller/
    │   ├── config/
    │   │   ├── obstacles.yaml         # Follow the Gap parameters
    │   │   └── lap_tracker.yaml       # Lap counter parameters
    │   ├── f1tenth_controller/
    │   │   ├── gap.py                 # Follow the Gap algorithm
    │   │   ├── gap_node.py            # Controller ROS interface
    │   │   ├── control_gate.py        # Controller activation
    │   │   ├── laps.py                # Lap counting and timing
    │   │   ├── lap_node.py            # Lap tracker ROS interface
    │   │   ├── lap_panel.py           # Terminal display formatting
    │   │   ├── lap_display_node.py    # Read-only lap display
    │   │   └── session_guard.py       # Session validation
    │   └── launch/
    │       └── two_vehicles.launch.py # Simulator, controllers and trackers
    └── f1tenth_gym_ros/
        ├── config/
        │   └── sim_two_vehicles.yaml  # Map, vehicles and starting poses
        ├── maps/
        │   ├── Zandvoort_obstacles.png
        │   └── Zandvoort_obstacles.yaml
        ├── f1tenth_gym_ros/
        │   ├── gym_bridge.py          # ROS and simulator integration
        │   ├── dual_control.py        # Shared start and finish braking
        │   └── simulation_session.py  # Attempt lifecycle
        └── launch/
            ├── gym_bridge_launch.py
            └── two_vehicles.rviz      # Two-vehicle visualization
```

## 3. Execution instructions

These instructions assume **Ubuntu 22.04, ROS 2 Humble, F1TENTH Gym, rosdep and colcon** are installed.

### 3.1. Clone and build

```bash
cd ~
git clone https://github.com/mmayorga21/f1tenth-zandvoort-parte2.git F1Tenth-Repository-Parte2
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

### 3.2. Start the two-vehicle scenario

In **terminal 1**:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_controller two_vehicles.launch.py
```

This opens RViz with the obstacle map and both vehicles, and starts both controllers and lap trackers. The controllers start disabled. Wait for the map and vehicles to appear, and keep this terminal open.

**Video 1 — Start the scenario**

https://github.com/user-attachments/assets/7d4edd4f-4b33-4b8f-bd91-b4ee8d39e60a

### 3.3. Open the lap displays

In **terminal 2**, start the primary vehicle's display:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run f1tenth_controller lap_display --ros-args \
  -r __ns:=/ego_racecar -r /laps/status:=/ego_racecar/laps/status
```

**Video 2.1 — Primary vehicle lap display**

https://github.com/user-attachments/assets/19d14c1f-84dc-41dc-a590-1f35e4b783c9

In **terminal 3**, start the secondary vehicle's display:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run f1tenth_controller lap_display --ros-args \
  -r __ns:=/opp_racecar -r /laps/status:=/opp_racecar/laps/status
```

Each panel shows completed laps, individual lap times, the best lap and the accumulated time. Keep both displays open.

**Video 2.2 — Secondary vehicle lap display**

https://github.com/user-attachments/assets/8f304b96-c4dc-49cd-90fe-ba0f66300005

### 3.4. Start the ten-lap attempt

In **terminal 4**:

```bash
cd ~/F1Tenth-Repository-Parte2
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 service call /sim/reset std_srvs/srv/Trigger "{}"
ros2 service call /ego_racecar/controller/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /opp_racecar/controller/enable std_srvs/srv/SetBool "{data: true}"
```

Wait for each service response before running the next command. The attempt starts once both controllers are ready. Observe the primary vehicle overtaking the secondary vehicle while both avoid the five static obstacles.

**Video 3 — Start the attempt and observe the overtake**

https://github.com/user-attachments/assets/9fb2ee64-ede9-49a5-9e34-3ced50fbfdc0

After completing ten laps, the primary remains stopped at its finish line while the secondary continues. Each panel retains its first ten lap times. The session closes automatically after both vehicles complete ten valid laps and stop. A collision or controller failure invalidates the attempt.

### 3.5. Stop or repeat an attempt

To end an attempt early, run in **terminal 4**:

```bash
ros2 service call /sim/end_attempt std_srvs/srv/Trigger "{}"
```

To run another attempt, repeat the reset and enable commands from section 3.4. To close the programs, press `Ctrl+C` in their terminals.
