# GPS-Denied Autonomous Drone Navigation System

**Competition:** Robothon'26 | **Host:** NIT Raipur | **Deadline:** March 22, 2026
**Team:** [Your Name]

## Core Features & Differentiators

This project provides a robust, production-quality, GPS-denied autonomous drone navigation system capable of operating in critical environments like tunnels, warehouses, or disaster zones.

1. **Dual Localization Failover:** Integrates `slam_toolbox` for LiDAR-based SLAM with fallback capability.
2. **Adaptive PID Control:** Custom C++ cascade controller running at 50Hz, with altitude-zone gain scheduling and anti-windup.
3. **Semantic Dynamic Obstacles:** Uses Webots dynamic environment simulation and dynamic costmap inflations in Nav2.
4. **Graceful Degradation:** A Python based Watchdog node manages IMU faults, Odometry faults, and initiates safe-states (like Emergency Hover) to avoid critical failure.
5. **Live Telemetry Dashboard:** A HTML/JS dashboard using `roslibjs` displaying live sensor health, fault events, and PID error graphs.

## Quick Start (One-Command Reproducibility)

You can launch the entire stack via Docker (reproducible container) for judging:

```bash
docker-compose up --build
```

Access the dashboard at `http://localhost:8080/index.html` (Local development config).

### Manual Start (WSL2 / Ubuntu 22.04)

```bash
./scripts/setup.sh
# Check Webots Bridge
ros2 launch webots_ros2_universal_robot multirobot_launch.py
# Run the entire system
./scripts/run_all.sh
```

## System Architecture 
- **Layer 1: Perception:** `robot_localization` EKF, noise injection testing, and sensor health monitoring.
- **Layer 2: SLAM:** `slam_toolbox` executing in Lifelong Mapping mode.
- **Layer 3: Navigation:** `Nav2` Stack utilizing `SmacPlanner2D` for narrow spaces.
- **Layer 4: Control:** C++ Cascade PID Controller and Motor Mixing matrix targeting < ±5cm altitude error.
- **Layer 5: Fault Handling:** Independent 10Hz node listening to `/sensor_status` triggering automated fallback.
- **Layer 6: Telemetry:** Web Server with UI graphing tool and CSV recording.
