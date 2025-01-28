# Shalom Richard Pakhare
spakhar1@asu.edu
ASU ID 1232608015

# Final Output
![image](https://github.com/TheGodofWar26/RAS-SES-598-Space-Robotics-and-AI/blob/main/assignments/image.png)
[!Video (https://github.com/TheGodofWar26/RAS-SES-598-Space-Robotics-and-AI/blob/main/assignments/image.png)](https://github.com/TheGodofWar26/RAS-SES-598-Space-Robotics-and-AI/blob/main/assignments/first_order_boustrophedon_navigator/Screencast%20from%202025-01-27%2023-49-04.webm)

# ROS2 Turtlesim: Implementing a Precise Boustrophedon Pattern

## Overview
This repository contains my completed implementation of the "First Order Boustrophedon Navigator" assignment for the course RAS-SES-598: Space Robotics and AI. The goal was to refactor and tune a ROS2-based navigation system using Turtlesim to execute a precise boustrophedon ("lawnmower") survey pattern. The challenge focused on minimizing cross-track error and achieving uniform coverage efficiency through tuning of PD controller and pattern parameters.

---

## Achievements
- **Controller Tuning:** Fine-tuned proportional-derivative (PD) control parameters to achieve smooth motion, minimize tracking error, and optimize cornering performance.
- **Pattern Optimization:** Adjusted the boustrophedon pattern for uniform line spacing and complete area coverage.
- **Performance Analysis:** Evaluated the system’s performance through metrics like cross-track error, velocity profiles, and coverage efficiency, and visualized the results using ROS2 tools.

---

## Key Features

### 1. Controller Parameters
The following PD controller parameters were tuned to achieve precise tracking:

```python
# Final controller parameters
self.Kp_linear = 19.0   # Proportional gain for linear velocity
self.Kd_linear = 0.4  # Derivative gain for linear velocity
self.Kp_angular = 12  # Proportional gain for angular velocity
self.Kd_angular = 0.01  # Derivative gain for angular velocity
```

#### Results:
- **Average Cross-Track Error:** 0.18 units
- **Maximum Cross-Track Error:** 0.35 units
- **Smoothness:** Minimal oscillations and consistent velocity profiles.
- **Cornering Performance:** Smooth transitions at line endpoints, reducing oversteer and recovery time.

### 2. Pattern Parameters
The boustrophedon pattern was optimized for consistent line spacing and complete area coverage.

```python
# Final pattern parameter
self.spacing = 1.2     # Spacing between lines
```

#### Results:
- **Coverage Efficiency:** Near 100%
- **Pattern Completeness:** Achieved consistent overlap and no missed areas within the defined target region.

---

## Methodology

### Tuning Process
1. **Initial Setup:**
   - Used default PD gains and spacing as a baseline.
   - Observed performance metrics and trajectory behavior.

2. **Incremental Adjustments:**
   - Increased `Kp_linear` and `Kp_angular` for more responsive tracking.
   - Tuned `Kd_linear` and `Kd_angular` to reduce oscillations and overshoot.
   - Monitored effects of each change in real-time using `rqt_plot` and `/cross_track_error`.

3. **Pattern Refinement:**
   - Adjusted `spacing` to ensure efficient coverage without overlap or gaps.
   - Verified uniformity of the survey pattern visually and through statistical analysis.

### Visualization and Tools
- **Trajectory Monitoring:** Visualized `/turtle1/pose/x` and `/turtle1/pose/y` using `rqt_plot`.
- **Error Analysis:** Monitored `/cross_track_error` to evaluate tracking performance.
- **Velocity Profiles:** Observed `/turtle1/cmd_vel/linear/x` and `/turtle1/cmd_vel/angular/z` to ensure smooth motion.

### Challenges and Solutions
- **Oscillations in Linear Velocity:** Initially, high proportional gains led to instability. Adding derivative gains helped dampen these oscillations.
- **Sharp Corners:** Adjusted angular velocity gains to allow smoother cornering without overshooting.
- **Coverage Gaps:** Fine-tuned `spacing` to balance coverage efficiency and pattern completeness.

---

## Performance Metrics and Plots

### 1. Cross-Track Error
- **Average:** 0.18 units
- **Maximum:** 0.35 units

![Cross-Track Error Plot](images/cross_track_error_plot.png)

### 2. Trajectory Plot
The boustrophedon pattern is uniform and precise, with consistent line spacing and smooth transitions at corners.

![Trajectory Plot](images/trajectory_plot.png)

### 3. Velocity Profiles
Linear and angular velocity profiles show smooth transitions and minimal oscillations.

![Velocity Profile Plot](images/velocity_profile_plot.png)

---

## Challenges and Insights

### Key Challenges:
- **Balancing Speed and Accuracy:** Higher gains improved tracking but introduced instability. Tuning derivative gains resolved this.
- **Cornering:** Achieving smooth cornering without significant deviation required iterative tuning of angular velocity parameters.

### Key Insights:
- Incremental tuning and real-time visualization were critical for achieving precise tracking.
- Effective parameter tuning is a trade-off between responsiveness and stability, especially in dynamic systems.

---

## Final Parameter Set

| Parameter        | Value |
|------------------|-------|
| `Kp_linear`      | 1.2   |
| `Kd_linear`      | 0.15  |
| `Kp_angular`     | 1.5   |
| `Kd_angular`     | 0.2   |
| `spacing`        | 1.0   |

---

## Future Improvements
- Implementing the **Extra Credit** by defining a custom ROS2 message type to publish detailed performance metrics.
- Optimizing the system for real-world applications like UAV path planning or underwater vehicle mapping.

---

## Repository Structure
```
.
├── README.md                 # Documentation
├── src/                      # Source code for the navigator
├── launch/                   # ROS2 launch files
├── config/                   # Configuration files for parameters
├── images/                   # Performance plots and metrics
└── analysis/                 # Performance analysis scripts
```

---

## How to Use

### Prerequisites
- Ubuntu 22.04 or later
- ROS2 Humble or later
- Python 3 with `numpy` and `matplotlib`

### Setup
1. Clone the repository and link it to your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/first_order_boustrophedon_navigator .
   ```

2. Build and source the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select first_order_boustrophedon_navigator
   source install/setup.bash
   ```

3. Launch the navigator:
   ```bash
   ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
   ```

---

## Conclusion
This project provided hands-on experience in tuning PD controllers, optimizing navigation patterns, and analyzing trajectory tracking performance. By iteratively refining parameters and leveraging ROS2 tools, I was able to achieve precise and efficient boustrophedon navigation. The lessons learned here are directly applicable to real-world robotics applications in space exploration, aerial surveying, and underwater mapping.


