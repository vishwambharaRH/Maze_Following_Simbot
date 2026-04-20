# Project Roadmap: Predictive Maze-Solver (ROS2 Humble)

## 1. Project Strategy: Predictive Wall-Following
Our team will implement a **Predictive Wall-Following** architecture. 

By analyzing the *rate of change* in distance sensors, the robot will anticipate corners before they are reached, allowing for smoother velocity curves and preventing the "jitter" often seen in basic solvers. This modular approach allows each of us to develop and test our nodes independently before final integration.

---

## 2. Team Roles & Ownership

### **Systems Architect: Logic & Integration**
* **Logic Engine:** Develop the core state machine (Searching, Following, and Turning states).
* **Velocity Control:** Implementation of the PID control laws for linear and angular velocity.
* **System Launch:** Create the top-level Python launch files to initialize all nodes and the Gazebo environment.
* **Final Integration:** Facilitate the final merge of perception, modeling, and environment modules.

### **Module Lead: Robot Modeling (URDF)**
* **Physical Chassis:** Author the Xacro/URDF files for a compact, differential-drive robot tailored for tight maze turns.
* **Coordinate Frames:** Configure the TF2 tree to ensure sensors and wheels are correctly positioned relative to `base_link`.
* **Simulation Physics:** Define Gazebo-specific tags for mass, inertia, and surface friction to ensure realistic movement.

### **Module Lead: Perception & Signal Processing**
* **Data Filtering:** Create a node to subscribe to raw `/scan` data and filter out noise or "Out of Range" values.
* **Spatial Analysis:** Segment 360-degree LiDAR data into prioritized Front, Left, and Right regional clusters.
* **Custom Interface:** Publish a simplified distance message that the logic engine can consume for high-speed decision-making.

### **Module Lead: Environment & Diagnostics**
* **Maze Synthesis:** Build a challenging `.sdf` maze world in Gazebo, including 90-degree turns and dead-ends.
* **Visualization:** Setup a custom Rviz2 environment to overlay sensor data and the robot’s trajectory for the demo.
* **Performance Metrics:** Implement a simple logger to track "Time to Solve" and a watchdog node to detect collisions.

---

## 3. 12-Hour Development Timeline

| Timeframe | Phase | Goal |
| :--- | :--- | :--- |
| **0h - 3h** | **Foundation** | URDF model complete; Maze world built; LiDAR filtering node active. |
| **3h - 7h** | **Development** | Integration of Logic Engine with Perception; Initial movement in simulation. |
| **7h - 10h** | **Optimization** | PID tuning for speed; Refining predictive curves for complex corners. |
| **10h - 12h** | **Validation** | Final stability runs; Data collection; Recording the successful demo. |

---

## 4. Communication & Standards
To ensure our modules plug into each other seamlessly, we will adhere to these standards:
* **Middleware:** ROS2 Humble (Ubuntu 22.04).
* **Interfaces:** Standard `geometry_msgs/msg/Twist` for movement and `sensor_msgs/msg/LaserScan` for sensing.
* **Build System:** All packages must be buildable via `colcon build --symlink-install`.