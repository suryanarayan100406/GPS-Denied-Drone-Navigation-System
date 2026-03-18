# GPS-Denied Autonomous Drone Navigation System

**Robothon 2026 | Problem Statement 1-a: Autonomous 2D Drone Navigation with Obstacle Avoidance**

---

## 📋 Executive Summary

This project presents a complete, production-ready ROS2/Webots autonomous navigation system for GPS-denied environments. Designed specifically for the Robothon hackathon, it demonstrates advanced autonomous capabilities through:

- **Real-time 2D SLAM & Path Planning** using A* with RRT fallback
- **PID-based Drone Control** with anti-windup stabilization
- **Dynamic Obstacle Avoidance** using potential field methods
- **Adaptive Navigation** responding to environmental changes in real-time
- **Comprehensive Performance Metrics** tracking safety, energy, and efficiency

**Target**: Shortlisting Round (Pre-Event Testing Phase) with full documentation of testing environments, constraints, and quantitative metrics.

---

## 🎯 Problem Statement Analysis

### 1-a: Autonomous 2D Navigation with Obstacle Avoidance

**Objective**: Simulate autonomous navigation from start to goal in 2D environment while avoiding obstacles.

**Our Solution**:
- ✅ Full autonomous pipeline (perception → planning → control)
- ✅ Multiple difficulty levels (easy/medium/hard)
- ✅ Dynamic obstacles and environmental challenges
- ✅ Real-time metrics tracking and evaluation
- ✅ Reproducible testing environments with documented constraints

---

## 🚀 Unique Features & USP

### Advanced Capabilities Beyond Standard Navigation

1. **🔄 Dynamic Environment Manager**
   - Runtime obstacle generation and movement
   - Difficulty scaling (Easy → Medium → Hard)
   - Environmental constraint enforcement
   - Safety zone definition and monitoring
   - Real-time constraint publication

2. **📊 Advanced Metrics Evaluator**
   - Path optimality tracking
   - Energy efficiency analysis
   - Computational performance profiling (CPU/memory)
   - Collision detection with safety scoring
   - Automated evaluation report generation

3. **🎛️ Adaptive Mission Control**
   - Environmental constraint compliance
   - Battery-aware mission planning
   - Dynamic replanning on obstacle detection
   - Safety margin enforcement

4. **🔬 Robothon Testing Suite**
   - Automated multi-level environment testing
   - Pre-configured test scenarios
   - Quantitative metric generation
   - Evaluation report automation

5. **📈 Performance Visualization**
   - Real-time RViz monitoring
   - Rosbag-based trajectory recording
   - Python visualization tools
   - Energy consumption graphs

---

## 🛠️ Installation

### Prerequisites
- WSL2 Ubuntu 22.04+ / Native Linux
- ROS2 Jazzy
- Webots R2025a
- Python 3.10+ with numpy, scipy

### Quick Install
```bash
# Clone and build
git clone https://github.com/suryanarayan100406/GPS-Denied-Drone-Navigation-System.git
cd GPS-Denied-Drone-Navigation-System

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build package
colcon build --packages-select drone_nav_2d
source install/setup.bash

# Launch
ros2 launch drone_nav_2d drone_nav_launch.py
```

---

## 🎮 Running Tests

### Automated Test Suite (Recommended)
```bash
python3 robothon_tester.py
# Runs easy → medium → hard with automatic metrics
```

### Manual Launch with Difficulty Levels
```bash
# Easy environment
ros2 launch drone_nav_2d drone_nav_launch.py use_hard_world:=false

# Medium with dynamic obstacles
ros2 launch drone_nav_2d drone_nav_launch.py \
  difficulty_level:=medium \
  enable_dynamic_obstacles:=true

# Hard environment
ros2 launch drone_nav_2d drone_nav_launch.py use_hard_world:=true
```

---

## 📊 Performance Metrics

The system automatically tracks and reports:

| Metric | Purpose | Target |
|--------|---------|--------|
| **Overall Score (0-100)** | Aggregated performance | >80 |
| **Safety Score** | Collision-free operation | >90 |
| **Path Optimality** | Efficiency of planned path | >0.85 |
| **Energy Efficiency** | Meters per battery unit | >5 |
| **CPU Usage** | Computational load | <30% avg |
| **Replans** | Adaptability to obstacles | <5 |
| **Min Obstacle Distance** | Safety margin | >0.15m |

After each test, detailed JSON reports are generated in `results/`

---

## 🏗️ System Architecture

```
Webots Simulator
    ↓
ROS2 Topics: /scan, /gps, /imu, /cmd_vel
    ↓
┌─ Map Publisher (Occupancy Grid)
├─ Path Planner (A* + RRT)
├─ Drone Controller (PID)
├─ Obstacle Avoidance (Potential Field)
├─ Metrics Logger (Basic KPIs)
├─ Dynamic Environment Manager (NEW)
└─ Advanced Metrics Evaluator (NEW)
    ↓
Visualization: RViz2 + rosbag2 + generated reports
```

---

## 📈 Testing Environments

### Easy Level
- Obstacle density: 15%
- Static obstacles only
- Flat terrain
- Expected score: >95

### Medium Level  
- Obstacle density: 25%
- 2 dynamic obstacles
- Rough terrain (0.3)
- Wind simulation (2 m/s)
- Expected score: >80

### Hard Level
- Obstacle density: 40%
- 5 dynamic obstacles
- Very rough terrain (0.7)
- Wind simulation (5 m/s)
- Expected score: >70

---

## 🔑 Algorithms Used

### 1. A* Path Planning
- Heuristic: 0.7×Manhattan + 0.3×Euclidean
- Obstacle inflation: 0.5m safety radius
- Fallback: RRT (2500 max iterations)

### 2. PID Drone Control
- Separate XYZ control loops
- Anti-windup limiting
- Proportional/Integral/Derivative gains
- Tunable via `config/nav_params.yaml`

### 3. Potential Field Avoidance
- Attractive force toward goal
- Repulsive force from obstacles
- Activation at 0.5m threshold
- Vector sum for net command

### 4. Dynamic Replanning
- Triggered by obstacle detection
- Battery level monitoring
- Path validity checking
- Real-time constraint enforcement

---

## 📁 Key Files

- `robothon_tester.py` - Automated testing framework
- `setup.py` - Entry points for all 7 ROS2 nodes
- `config/nav_params.yaml` - Tunable parameters
- `worlds/` - Easy + Hard test environments
- `results/` - Generated evaluation reports

---

## ✨ Robothon Evaluation Readiness

| Requirement | Status |
|-------------|--------|
| Simulation in documented environment | ✅ |
| Environment constraints documented | ✅ |
| Performance metrics defined + tracked | ✅ |
| Hardware/software constraints profiled | ✅ |
| ROS2 package + GitHub repo | ✅ |
| Detailed README | ✅ |
| Technical presentation ready | ✅ |
| Unique/advanced features | ✅ |
| Reproducibility support | ✅ |

---

## 📞 Quick Links

- **Repository**: https://github.com/suryanarayan100406/GPS-Denied-Drone-Navigation-System
- **Problem**: Autonomous 2D drone navigation with obstacle avoidance
- **Hackathon**: Robothon 2026 (Shortlisting Round)
- **Status**: Ready for evaluation

---

**Last Updated**: March 18, 2026 | **Status**: Submission Ready
