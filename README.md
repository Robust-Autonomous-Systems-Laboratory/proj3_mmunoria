# proj3_mmunoria Project 3 — Dead Reckoning State Estimation

## Package Overview

- **Workspace:** `proj3_ws`
- **Package name:** `dead_reckoning`
- **Node name:** `estimator_node`

## How to Build and Run

### 1️ Create the workspace

```bash
mkdir -p ~/proj3_ws/src
```
## 2 Clone the Repository

```bash
git clone https://github.com/Robust-Autonomous-Systems-Laboratory/proj3_mmunoria.git
```
### 3 Assuming the bag file is already present
Place the bag file in the workspace root
```bash
cp -r /path/to/proj3 ~/proj3_ws/
```
### 4 Build and Source the workspace
```bash
cd ~/proj3_ws
colcon build
source install/setup.bash
```

### 5 Run the estimator node
```bash
ros2 run dead_reckoning estimator
```
### 6 Before visualizing with RViz2, run the xacro file and the state publisher gui (2 new terminals)
```bash
terminal 1
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/proj3_ws/kart.urdf.xacro)"

terminal 2
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```
### 7 Visualization (RViz2) (new tweminal)
```bash
ros2 run rviz2 rviz2
```
# Set Fixed Frame to *"odom"*
# Path displays:
- /dead_reckoning/path
- /imu_integration/path
 - Use Top-Down Orthographic view
### 8 Play the bag file (new terminal)
```bash
ros2 bag play proj3
```
