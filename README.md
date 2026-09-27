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
- [x] Automated test passed in GUI and headless simulation
- [x] CSV logging for fault state and runtime metrics

### Independent Safety Command Path
- [x] Separate normal command path: `/cmd_vel_nav`
- [x] Separate safety command path: `/cmd_vel_safety`
- [x] Priority arbitration with `twist_mux`
- [x] Safety command overrides continuous non-zero navigation command
- [x] Odometry-verified full stop in CRITICAL
- [x] Automatic resume after healthy recovery window
- [x] Cleanup script to prevent stale ROS2 test nodes between runs

### Next
- [x] Default idle-stop behavior when navigation commands disappear
- [x] Odometry anomaly detection
- [x] Control latency detection
- [x] Navigation node failure detection
- [ ] Automated multi-scenario evaluation
- [ ] Status dashboard

## Fault 1 Test Result

The LiDAR fault scenario has been verified repeatedly in simulation.

- Normal scan stream remained in NORMAL state
- Fault injection stopped `/scan` forwarding
- State changed in order: WARNING → DEGRADED → CRITICAL
- During CRITICAL, `/cmd_vel_nav` continued publishing `0.2 m/s`
- The higher-priority safety path forced final `/cmd_vel` to `0.0 m/s`
- Odometry confirmed zero velocity and zero movement during the measured CRITICAL interval
- After LiDAR recovery, the system held CRITICAL during the recovery hysteresis window
- After approximately 2 s of healthy scan data, the system returned to NORMAL and normal motion resumed automatically

Current thresholds:

| State | Scan age |
|---|---:|
| NORMAL | ≤ 0.3 s |
| WARNING | > 0.3 s |
| DEGRADED | > 0.7 s |
| CRITICAL | > 1.0 s |

A short consecutive-count hold is used for escalation, and a 2-second healthy scan window is required for recovery.

### Safety Arbitration Verification

| Phase | `/cmd_vel_nav` | Final `/cmd_vel` | Odom vx | Measured motion |
|---|---:|---:|---:|---:|
| Before fault | 0.20 m/s | 0.20 m/s | ~0.20 m/s | 20.4 cm/s |
| CRITICAL | 0.20 m/s | 0.00 m/s | 0.000 m/s | 0.0 mm/s |
| After NORMAL recovery | 0.20 m/s | 0.20 m/s | ~0.20 m/s | 19.7 cm/s |

Observed transition timing in one simulation run:

- 12.65 s: CRITICAL entered
- 12.70 s: final velocity output became zero
- 12.80 s: odometry reached zero velocity
- 17.80 s: NORMAL restored
- 18.05 s: normal velocity output restored
- 18.15 s: robot motion resumed





## Fault 4 — Navigation Command Source Failure

The navigation command source is implemented as a dedicated ROS2 process that publishes both the raw velocity command and a heartbeat containing its PID. The fault test terminates this process for real and relies on launch respawn for recovery.

Implemented behavior:

- navigation source publishes `0.20 m/s` and heartbeat at 10 Hz
- `/nav_fault/crash` terminates the navigation process with a non-zero exit
- launch respawns the process after a configured delay
- heartbeat age drives NORMAL → WARNING → DEGRADED → CRITICAL
- existing idle-stop and safety paths stop the robot while the source is unavailable
- a 2 s healthy heartbeat window is required before returning to NORMAL
- restart count and PID changes are tracked
- crash-loop mode verifies repeated restart failure while keeping the robot stopped

Observed in one GUI run:

- WARNING: ~0.35 s after heartbeat loss
- DEGRADED: ~0.80 s
- CRITICAL: ~1.30 s
- final command in CRITICAL: 0.00 m/s
- odometry in CRITICAL: 0.000 m/s
- measured movement during stop: 0.0 mm
- respawn: approximately 1.5–2.4 s after termination
- NORMAL recovery: about 2 s after healthy heartbeat resumes
- automatic motion recovery: ~0.20 m/s

Automated verification:
- Fault 4: 36/36 checks PASS
- Fault 1 regression: 23/23 PASS
- Fault 2 regression: 21/21 PASS
- Fault 3 regression: 36/36 PASS
- repeated runs were stabilized by using a workspace-specific Fast DDS UDP transport configuration

## Fault 3 — Control Latency

A real command-delay injector now holds incoming navigation commands before forwarding them, allowing the system to measure and respond to actual control latency rather than a simulated fault flag.

Implemented behavior:

- normal delay: approximately 0 ms
- DEGRADED injection: 300 ms
- CRITICAL injection: 700 ms
- measured latency is published and monitored at runtime
- 3 consecutive delayed commands are required for WARNING/DEGRADED escalation
- 2 consecutive commands above the CRITICAL threshold are required for CRITICAL
- 2 s of healthy latency is required before recovery to NORMAL
- stale queued commands are discarded when the delay fault is disabled

Response policy:

- DEGRADED: existing speed limiter reduces motion from 0.20 m/s to 0.10 m/s
- CRITICAL: existing high-priority safety path forces final velocity to 0.00 m/s
- recovery: normal motion resumes automatically after the healthy hold

| Phase | Injected delay | Measured latency | Final command | State |
|---|---:|---:|---:|---|
| Normal | 0 ms | ~0.1 ms | 0.20 m/s | NORMAL |
| DEGRADED | 300 ms | ~0.30 s | 0.10 m/s | DEGRADED |
| CRITICAL | 700 ms | ~0.70 s | 0.00 m/s | CRITICAL |
| Recovered | 0 ms | sub-ms | 0.20 m/s | NORMAL |

Automated verification:
- Fault 3: 36/36 checks PASS in GUI and headless simulation
- Fault 1 regression: 23/23 PASS
- Fault 2 regression: 21/21 PASS
- no stale-command burst observed after disabling the delay fault

## Fault 2 — Odometry Anomaly

Odometry faults are injected by relaying `/odom_raw` through an injector that can alter `twist.linear.x` before publishing `/odom`.

Implemented checks:

- physical speed bound: 0.5 m/s
- acceleration bound: 5 m/s² using the last valid sample
- command consistency: persistent command/odometry mismatch above 0.3 m/s
- invalid odometry samples are rejected from `/odom_validated`
- 5 consecutive invalid samples trigger DEGRADED
- 2 s of healthy odometry is required before recovery to NORMAL

A single injected spike to 2.5 m/s is rejected without changing the overall system state. Persistent injected anomalies trigger DEGRADED operation and reduce the commanded speed from 0.20 m/s to 0.10 m/s. After the fault is cleared and the healthy recovery window completes, the speed limit is removed and the robot returns to 0.20 m/s.

| Phase | Nav command | Final command | System state |
|---|---:|---:|---|
| Normal | 0.20 m/s | 0.20 m/s | NORMAL |
| Persistent odom anomaly | 0.20 m/s | 0.10 m/s | DEGRADED |
| 1 s after fault clear | 0.20 m/s | 0.10 m/s | DEGRADED |
| Healthy recovery complete | 0.20 m/s | 0.20 m/s | NORMAL |

Automated verification:
- Fault 2: 21/21 checks PASS in GUI and headless simulation
- Fault 1 regression: 23/23 checks PASS

## Navigation Command Loss Safety

A low-priority `idle_stop` source now publishes zero velocity continuously.

Priority order:

1. `/cmd_vel_safety` — priority 255
2. `/cmd_vel_nav` — priority 10
3. `/cmd_vel_idle_stop` — priority 1

This prevents stale non-zero velocity commands from being held when the navigation command publisher disappears.

| Phase | `/cmd_vel_nav` | Final `/cmd_vel` | Odom vx | Motion |
|---|---:|---:|---:|---:|
| Before nav loss | 0.20 m/s | 0.20 m/s | 0.200 m/s | 19.7 cm/s |
| Nav publisher removed | no messages | 0.00 m/s | 0.000 m/s | 0.0 mm/s |
| Nav publisher restored | 0.20 m/s | 0.20 m/s | 0.200 m/s | 19.7 cm/s |
| LiDAR CRITICAL | 0.20 m/s | 0.00 m/s | 0.000 m/s | 0.0 mm/s |

The full Fault 1 regression suite currently contains 23 checks and has passed in both GUI and headless simulation.

## Test Process Cleanup

Simulation runs are launched in their own process group. Cleanup now terminates only the process group created by the current test run, avoiding broad `pkill -f` matching. PGID tracking files are retained until the target group has actually exited.

## Planned Architecture

```text
             Normal control
              /cmd_vel_nav
                    |
                    v
Sensors ---> Fault Monitor ---> Safety Manager
   |                                |
   |                         /cmd_vel_safety
   |                                |
   +-------------------------> twist_mux
                                    |
                                    v
                                /cmd_vel
                                    |
                                    v
                                  Robot
```

The safety command path is independent from the normal motion command path. In CRITICAL state, the high-priority safety input overrides continuous non-zero navigation commands.

## Environment

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11
- TurtleBot3 Burger
- Python / rclpy
- twist_mux

## Project Goals

The project is designed as an implementation-focused capstone rather than a paper-oriented study. The current core is complete, and the project is being expanded toward autonomous navigation, compound-fault handling, recovery, visualization, and quantitative verification. The main deliverables are:

- reproducible fault injection
- runtime fault detection
- fault-dependent degraded operation
- independent safe-stop behavior
- quantitative evaluation such as detection latency, recovery success rate, false positives, and stopping behavior

Concepts such as fault detection, degraded operation, and minimal-risk behavior are used as design references. This project does **not** claim ISO 26262 or SOTIF compliance.


## Integrated Dashboard and Demo

A tkinter-based monitoring dashboard and automated integrated demo are now available.

The dashboard provides:

- overall system state: NORMAL / WARNING / DEGRADED / CRITICAL
- individual LiDAR, odometry, control-latency, and navigation states
- scan age, odometry validity, measured latency, navigation PID / restart information
- target velocity, final command, measured odometry velocity, speed-limit status, and safety-stop status
- service buttons for all four fault types
- event logging for state transitions and fault events

The automated integrated demo executes the four fault scenarios sequentially and verifies recovery between scenarios.

Validation:

- integrated demo: 25/25 PASS, four consecutive GUI runs
- Fault 1 regression: 23/23 PASS
- Fault 2 regression: 21/21 PASS
- Fault 3 regression: 36/36 PASS
- Fault 4 regression: 36/36 PASS
- no leftover project processes after demo shutdown

Representative runtime behavior:

- LiDAR dropout: NORMAL → WARNING → DEGRADED → CRITICAL → recovery
- odometry anomaly: isolated spike rejected; persistent anomaly triggers DEGRADED at reduced speed
- control latency: 300 ms causes DEGRADED, 700 ms causes CRITICAL safe stop
- navigation process failure: actual process crash, respawn, healthy hold, then autonomous motion recovery

Final summary outputs are stored in:

- `results/final_summary.csv`
- `results/final_summary.json`


## Warehouse Nav2 Autonomous Navigation

The warehouse simulation has been extended from scripted velocity motion to real ROS2 Nav2 waypoint navigation.

The current autonomous mission uses four waypoints through the warehouse and finishes in the Delivery zone. Nav2 velocity commands are routed through the existing fault-tolerant control pipeline rather than publishing directly to the robot:

```text
Nav2 controller / behavior
          |
          v
   /cmd_vel_nav_raw
          |
          v
 control_delay_injector
          |
          v
     speed_limiter
          |
          v
       twist_mux
     /cmd_vel
          |
          v
    TurtleBot3 Burger
```

Warehouse navigation results:

- 4/4 waypoints reached
- approximately 19 m mission distance
- approximately 123 s mission duration
- 0 collisions
- measured minimum map clearance approximately 0.40 m
- goal-position error approximately 0.13 m in a representative run

Fault-tolerant Nav2 behavior has also been verified during active missions:

- LiDAR dropout: CRITICAL safe stop, healthy recovery, mission continuation
- odometry anomaly: DEGRADED speed limiting while Nav2 continues commanding motion
- control latency: 300 ms causes DEGRADED operation, 700 ms causes CRITICAL safe stop
- after recovery, the active waypoint mission continues to the Delivery zone

The Nav2 demo passed 25/25 checks in four consecutive GUI runs while all existing Fault 1–4 regression suites remained passing.

Nav2 summary outputs are stored separately from the original fault-demo summary:

- `results/nav2_summary.csv`
- `results/nav2_summary.json`

## Expansion Roadmap

The current implementation establishes the fault-diagnosis and staged-response core. The project is now being expanded from a fault-monitoring prototype into a fault-tolerant autonomous mobile robot system.

### 1. Simulation Environment Enhancement

The basic TurtleBot3 world will be replaced with a more realistic indoor AMR / warehouse-style environment so that the final demonstration represents an actual mobile-robot mission rather than simple straight-line motion.

Planned elements include:

- structured indoor map
- shelves / walls / obstacles
- start, waypoint, and goal positions
- visualization suitable for presentation and portfolio use

### 2. Fault State Visualization

A dedicated monitoring dashboard will visualize:

- overall system state: NORMAL / WARNING / DEGRADED / CRITICAL
- LiDAR, odometry, control-latency, and navigation health
- scan age, odometry validity, measured latency, heartbeat age
- speed limit, final velocity command, safety-stop state
- recent state transitions and fault events

### 3. Real Navigation Integration

The current constant-velocity navigation source will be replaced by ROS2 Nav2-based autonomous navigation.

Target flow:

```text
Goal / Waypoints
      |
      v
     Nav2
      |
      v
Path Planning / Controller
      |
      v
Fault-Tolerant Control Layer
      |
      v
    Robot
```

The final demonstration will verify fault responses while the robot is actively navigating toward goals and avoiding obstacles.

### 4. Recovery Enhancement

The existing process-crash / respawn recovery will be extended toward Nav2 lifecycle-aware recovery.

Planned targets include:

- Nav2 node health monitoring
- lifecycle-state inspection
- restart / reactivate behavior
- mission continuation after healthy recovery

### 5. Multi-Fault Response

The project will verify simultaneous and sequential faults rather than testing only one fault at a time.

Example scenarios:

- LiDAR dropout + odometry anomaly
- odometry anomaly + control latency
- navigation-process failure during another active fault
- recovery of one fault while another remains active

The system will validate severity arbitration and response priorities under compound faults.

### 6. Scenario Manager and Quantitative Evaluation

Fault sequences will be automated through reusable scenarios so experiments can be reproduced consistently.

Planned quantitative outputs include:

- detection latency
- transition time to DEGRADED / CRITICAL
- stopping latency and stopping distance
- recovery time
- restart success
- false-positive behavior
- commanded vs. actual speed during degraded operation

Selected final results will be stored in a compact `results/` summary for GitHub, while raw logs remain excluded.

### 7. Real-Robot Deployment (Optional)

If time permits, the final simulation system will be ported to a physical mobile robot such as TurtleBot3 to evaluate sensor disconnection and communication-delay cases outside simulation.

## Development Log

- **Milestone 1:** TurtleBot3 + Gazebo simulation baseline — complete
- **Milestone 2:** LiDAR dropout detection and safe stop — complete
- **Milestone 3:** Independent safety arbitration with `twist_mux` — complete
- **Milestone 4:** Navigation-command timeout safety behavior — complete
- **Milestone 5:** Odometry anomaly detection — complete
- **Milestone 6:** Control latency detection — complete
- **Milestone 7:** Navigation node failure detection and recovery — complete
- **Milestone 8:** Simulation environment enhancement — complete
- **Milestone 9:** Fault monitoring dashboard — complete
- **Milestone 10:** Nav2 goal / waypoint navigation integration — complete
- **Milestone 11:** Nav2 lifecycle-based recovery — in progress
- **Milestone 12:** Multi-fault scenarios and policy verification — planned
- **Milestone 13:** Scenario manager and quantitative evaluation — planned
- **Milestone 14:** Real-robot deployment — optional extension
