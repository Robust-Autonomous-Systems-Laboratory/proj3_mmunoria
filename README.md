# proj3_mmunoria

## Package Overview

- **Workspace:** `proj3_ws`
- **Package name:** `dead_reckoning`
- **Node name:** `estimator_node`

## How to Build and Run :  Multiple terminals are required

### 1️ Create the workspace

```bash
mkdir -p ~/proj3_ws/src
```
### 2 Create a Pakcage : while in the src directory
```bash
ros2 pkg create --build-type ament_python --node-name estimator_node dead_reckoning

cd ..
```
## 3 Clone the Repository
```bash
git clone https://github.com/Robust-Autonomous-Systems-Laboratory/proj3_mmunoria.git
```

replace the estimator_node.py with the one from the above command located in
- *src/dead_reckoning/dead_reckoning*

### 4 Assuming the bag file is already present
Place the bag file in the workspace root
```bash
cp -r /path/to/proj3 ~/proj3_ws/
```

### 5 Build and Source the workspace
```bash
cd ~/proj3_ws
colcon build
source install/setup.bash
```

### 6 Run the estimator node
```bash
ros2 run dead_reckoning estimator
```
### 7 Before visualizing with RViz2, run the xacro file and the state publisher gui (new terminal)
```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/proj3_ws/kart.urdf.xacro)"
```
### 8 Visualization (RViz2) (new tweminal)
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

# Trajectory Estimation Comparison: IMU Integration vs Dead Reckoning

---

## 1. Trajectory Comparison

### Do the they agree initially?
Yes.  
Because they both start from the same initial pose and agree very closely at the beginning of the motion.

### When do they diverge?
They begin to diverge just after the robot starts turning or following a curved path.  
The divergence grows over time as errors accumulate through integration.

- The **IMU-based trajectory** continues smoothly in a straight, diagonal direction.
- The **dead-reckoning trajectory** while the motion of the dead reckoning has a loop-like motion, reflecting the robot’s actual turns.

---

## 2. Which Method Exhibits More Drift?

**IMU exhibits more drift**, especially in position.

### Why?
- I think because the position from IMU data requires double integration of acceleration, which greatly amplifies:
  - Sensor noise
  - Bias
  - Orientation errors
- and also the gyro or accelerometer biases lead to rapidly growing errors over time.

Dead reckoning is also drifts, but at a slower rate for short.

---

## 3. Sources of Error

### IMU Integration Errors
- Accelerometer bias
- Gyroscope bias
- Imperfect gravity compensation
- Sensor noise
- Numerical integration errors
- Timing or synchronization issues
- IMU to robot frame misalignment

### Dead Reckoning Errors
- Wheel slip especially during turns or acceleration
- Incorrect wheel radius or wheelbase calibration
- Unequal wheel diameters or wheel wear
- Encoder quantization or missed ticks
- Unmodeled kinematic effects 
- Assumption of perfectly planar motion

---

## 4. Combining Both Estimates for Better Accuracy

The best performance is achieved through **sensor fusion**, where a **Kalman filter** is used to combine IMU integration and dead reckoning. This allows each method to compensate for the other’s error while accounting for their uncertainties.

---

## 5. Motion Assumptions

### Dead Reckoning Assumptions
- Wheels roll without slipping
- Robot follows the assumed kinematic model
- Motion is planar
- Wheel parameters are constant and accurately known

### IMU Integration Assumptions
- Sensor biases are small or slowly varying
- Gravity can be accurately removed
- IMU orientation is estimated correctly
- Robot behaves as a rigid body
- Motion is approximately planar when estimating 2D pose
---
