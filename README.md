
# MiniGirona Path Planning with OMPL and PID Controller

This repository implements path planning for the MiniGirona AUV using OMPL (Open Motion Planning Library) and follows the planned path using a PID controller.

## Features
- Path planning using OMPL
- Path following with PID controller
- Integration with StoneFish Simulator
- Occupancy grid mapping with OctoMap

## Prerequisites
- [StoneFish Simulator](https://stonefish.gitbook.io/stonefish-docs/) (AUV simulation environment)
- ROS (tested with [your ROS version])
- OMPL

## Installation

1. Clone the repository to your ROS workspace:
   ```bash
   cd ~/your_ws/src
   git clone https://github.com/Bilal1262/Minigirona-Path-Planning.git
   ```

2. Clone the rami_competition scenario (or prepare your own scenario):
   ```bash
   git clone [rami_competition_repository_url]
   ```

3. Build the workspace:
   ```bash
   cd ~/your_ws
   catkin_make  # or catkin_build
   ```

## Usage

1. Launch the scenario with OctoMap server:
   ```bash
   roslaunch rami_competition rami_occupancy.launch
   ```

2. Run the path planner:
   ```bash
   rosrun turtlebot_online_path_planning minigirona_path_planning_OMPL
   ```

3. Run the path follower:
   ```bash
   rosrun turtlebot_online_path_planning follow_path.py
   ```

4. If the robot isn't moving, enable the thrusters:
   ```bash
   rosservice call /minigirona/controller/enable_thrusters
   ```

## Configuration Notes

- If using a different simulator, you'll need to update the topics in the code
- For custom scenarios, ensure you have an OctoMap server running. See the [example launch file](https://github.com/Bilal1262/Minigirona-Path-Planning/blob/main/launch/rami_occupancy.launch) for reference

## File Structure
```
.
├── launch
│   └── rami_occupancy.launch    # Launch file for OctoMap server
├── src
│   ├── minigirona_path_planning_OMPL  # OMPL path planning implementation
│   └── follow_path.py           # PID path following implementation
└── README.md

