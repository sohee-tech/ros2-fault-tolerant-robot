# ROS2 Fault-Tolerant Robot

ROS2-based fault diagnosis and fault-tolerant control system for autonomous mobile robots.

This capstone project focuses on detecting runtime faults in a mobile robot, degrading operation according to fault severity, and transitioning to a safe state when necessary. The implementation uses ROS2 Humble, Gazebo Classic, and TurtleBot3, with fault injection and automated verification in simulation.

## Current Status

### Baseline simulation
- [x] TurtleBot3 Burger simulation in Gazebo Classic
- [x] `/scan`, `/odom`, `/cmd_vel`, `/imu`, `/tf`, `/joint_states`
- [x] Velocity command and odometry-based movement verification

### Fault 1 — LiDAR Dropout
- [x] Real scan-message dropout injection
- [x] Message-age based LiDAR monitoring
- [x] State transition: NORMAL → WARNING → DEGRADED → CRITICAL
- [x] 2 s recovery hysteresis before returning to NORMAL
- [x] CRITICAL-state safe stop
- [x] Automated test passed in GUI and headless simulation
- [x] CSV logging for fault state and runtime metrics

### Next
- [ ] Priority-based independent safety command path with `twist_mux`
- [ ] Odometry anomaly detection
- [ ] Control latency detection
- [ ] Navigation node failure detection
- [ ] Automated multi-scenario evaluation
- [ ] Status dashboard

## Fault 1 Test Result

The LiDAR fault scenario has been verified twice in simulation.

- Normal scan stream remained in NORMAL state
- Fault injection stopped `/scan` forwarding
- State changed in order: WARNING → DEGRADED → CRITICAL
- CRITICAL state generated repeated zero-velocity safety commands
- Odometry confirmed the robot stopped
- After LiDAR recovery, the system stayed in CRITICAL during the recovery hold and returned to NORMAL after approximately 2 s

Current thresholds:

| State | Scan age |
|---|---:|
| NORMAL | ≤ 0.3 s |
| WARNING | > 0.3 s |
| DEGRADED | > 0.7 s |
| CRITICAL | > 1.0 s |

A short consecutive-count hold is used for escalation, and a 2-second healthy scan window is required for recovery.

## Planned Architecture

```text
Sensors / Navigation
        |
        v
 Fault Monitoring
        |
        v
 Fault Classification
        |
        v
 NORMAL / WARNING / DEGRADED / CRITICAL
        |
        v
 Ignore / Limit / Recover / Safe Stop
```

The project will use a separate high-priority safety command path so that a stop command can override normal motion commands even when the navigation stack continues publishing velocity commands.

## Environment

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11
- TurtleBot3 Burger
- Python / rclpy

## Project Goals

The project is designed as an implementation-focused capstone rather than a paper-oriented study. The main deliverables are:

- reproducible fault injection
- runtime fault detection
- fault-dependent degraded operation
- independent safe-stop behavior
- quantitative evaluation such as detection latency, recovery success rate, false positives, and stopping behavior

Concepts such as fault detection, degraded operation, and minimal-risk behavior are used as design references. This project does **not** claim ISO 26262 or SOTIF compliance.

## Development Log

- **Milestone 1:** TurtleBot3 + Gazebo simulation baseline — complete
- **Milestone 2:** LiDAR dropout detection and safe stop — complete
- **Milestone 3:** Independent safety arbitration with `twist_mux` — in progress
