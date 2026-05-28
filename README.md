# 🤖 Robust Navigation & Automated Path Recovery System for Agricultural Robots

> Master's Thesis | Biosystems Engineering | KMITL (King Mongkut's Institute of Technology Ladkrabang)

A simulation study on developing a robust navigation system and automated path recovery (APRS) for agricultural robots operating in unstructured farm environments — built on **ROS 2 Jazzy Jalisco** and **Gazebo Harmonic**.

---

## 📌 Overview

This project addresses two core challenges in agricultural robotics:

1. **Navigation performance** — comparing 6 algorithm combinations (Global Planner × Local Controller) across 3 farm-inspired simulation maps
2. **Stagnation recovery** — an Automated Path Recovery System (APRS) based on Finite State Machine (FSM) that detects and resolves stuck conditions autonomously

---

## 🧪 Algorithm Combinations Tested

| Global Planner | Local Controller | Result |
|---|---|---|
| Smac2D | **MPPI** ⭐ | Best overall (0.98 m/s avg) |
| Smac2D | DWB | 2nd tier |
| Smac2D | RPP | Lower performance at U-turns |
| NavFn | MPPI | 2nd best overall |
| NavFn | DWB | Mid-tier |
| NavFn | RPP | Lowest at critical segments |

**Key finding:** `Smac2D + MPPI` achieved 98–99% of max velocity across all maps and consistently outperformed other combinations at critical segments (U-turns, narrow passages).

---

## 🗺️ Simulation Environment

- **Simulator:** Gazebo Harmonic (physics-based)
- **Framework:** ROS 2 Jazzy Jalisco + Nav2 Stack
- **Robot type:** Tracked vehicle (Differential Drive) with 2D LiDAR + IMU + Odometry
- **Maps:** 3 custom Occupancy Grid Maps based on real agricultural field aerial imagery at KMITL

```
Map 1 — Grid orchard layout with U-turns (8 waypoints)
Map 2 — Sparse obstacles with narrow corridor (5 waypoints)  
Map 3 — Complex loop with L-shaped obstacles (8 waypoints)
```

---

## ⚙️ System Architecture

```
Sensors (LiDAR, IMU, Odometry)
        ↓
Perception & State Estimation (EKF Global + Local)
        ↓
Nav2 Stack (Costmap → Global Planner → Local Controller)
        ↓
Stagnation Manager (FSM) ←→ User Interface (Recovery Panel)
        ↓
Motor Driver
```

---

## 🔄 APRS — Automated Path Recovery System

When the robot detects a stuck condition (obstacle within 0.5m or Nav2 `ABORTED`), the FSM transitions through 4 states:

| State | Description |
|---|---|
| `s0` | Normal monitoring |
| `s1` | Waiting for user decision (10s countdown UI) |
| `s2` | Cancel in progress |
| `s3` | Async wait for Nav2 cancel confirmation |

**Recovery options available to operator:**
- 🔁 **Find New Path** — recalculate route around obstacle
- ⏸️ **Wait** — pause robot; resume when obstacle clears
- 🏠 **Return to Home** — abort mission and navigate back to start

---

## 📊 Key Results

- All 6 algorithm combinations achieved **100% waypoint success rate**
- `Smac2D+MPPI` averaged **0.9823 m/s** (Map 1), **0.9350 m/s** (Map 2), **0.9252 m/s** (Map 3)
- At critical U-turn segments, `Smac2D+MPPI` outperformed `NavFn+RPP` by up to **2.8×**
- APRS detected and resolved all stagnation events with **zero FSM state transition errors**
- Statistical validation: One-way ANOVA + Duncan's Multiple Range Test (α = 0.05)

---

## 🛠️ Tech Stack

| Component | Technology |
|---|---|
| Robot OS | ROS 2 Jazzy Jalisco |
| Simulator | Gazebo Harmonic |
| Navigation | Nav2 (Smac2D / NavFn + MPPI / DWB / RPP) |
| Recovery system | Custom FSM Node (Python) |
| Sensing | 2D LiDAR, IMU, Wheel Odometry |
| Localization | Dual EKF (global + local) |
| Visualization | RViz2 |
| Robot model | URDF/SDF (Differential Drive Tracked Vehicle) |

---

## 🚀 Getting Started

### Prerequisites
```bash
# ROS 2 Jazzy Jalisco
# Gazebo Harmonic
# Nav2
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-ros-gz-bridge
```

### Build
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Guygan/Master-degree-project.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Run Simulation
```bash
# Launch robot in Gazebo
ros2 launch my_robot_description robot_localization.launch.py

# Run experiment (all 8 waypoints, selected algorithm)
python3 experiment_runner.py

# Record waypoints manually
ros2 run my_robot_description waypoint_recorder
```

---

## 📁 Repository Structure

```
Master-degree-project/
├── config/                  # Nav2 params (MPPI, DWB, RPP, Smac2D, NavFn)
├── launch/                  # Launch files
├── my_robot_description/    # URDF/SDF robot model
├── maps/                    # Map 1, 2, 3 (yaml + pgm)
├── src/
│   ├── stuck_manager.py     # FSM-based APRS node
│   ├── experiment_runner.py # Automated experiment controller
│   ├── waypoint_recorder.py # Waypoint recording utility
│   └── path_recorder.py     # Path logging node
└── README.md
```

---

## 👤 Author

**Guygan Chantrawongphaisal**  
M.Eng. Biosystems Engineering — KMITL  
B.Eng. Smart Agricultural Engineering — KMITL (First-Class Honours 🥇 Gold Medal)

---

## 📄 Citation

```
Chantrawongphaisal, G. (2026). A Simulation Study: Development of Robust Navigation 
and Automated Path Recovery System for Agricultural Robots with LiDAR. 
Master's Thesis, Biosystems Engineering, KMITL.
```
