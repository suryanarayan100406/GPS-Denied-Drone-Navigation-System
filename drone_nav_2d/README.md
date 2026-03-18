# drone_nav_2d
Autonomous 2D drone navigation in Webots using ROS2 Humble with A* global planning, reactive local obstacle avoidance, and PID tracking.

## 1) System Requirements
- OS: **Ubuntu 22.04** (recommended native). This project is also supported on **Ubuntu 22.04 in WSL2** with WSLg + GPU acceleration.
- ROS2: **Humble Hawksbill** (desktop install)
- Webots: **R2023b**
- Python: 3.10+
- Required ROS packages: `nav_msgs`, `geometry_msgs`, `sensor_msgs`, `tf2_ros`, `webots_ros2_driver`, `launch_ros`
- Required Python packages: `numpy`, `scipy`, `matplotlib`

## 2) Installation (Step-by-step)
> Run these commands inside Ubuntu 22.04 terminal (native or WSL2 Ubuntu shell).

### 2.1 Base setup
```bash
sudo apt update
sudo apt install -y curl wget gnupg2 lsb-release software-properties-common git python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 2.2 Install ROS2 Humble (Desktop)
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y locales
sudo add-apt-repository universe -y
sudo apt update

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release; echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop
```

### 2.3 ROS tools + dependencies
```bash
sudo apt install -y ros-humble-tf2-ros ros-humble-nav-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-rviz2 ros-humble-launch-ros
sudo apt install -y python3-numpy python3-scipy python3-matplotlib
```

### 2.4 Install Webots R2023b and Webots ROS2 bridge
```bash
# Webots package repository
wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo gpg --dearmor -o /usr/share/keyrings/cyberbotics.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/cyberbotics.gpg] https://cyberbotics.com/debian/ binary-amd64/" | sudo tee /etc/apt/sources.list.d/cyberbotics.list >/dev/null
sudo apt update
sudo apt install -y webots=2023b* || sudo apt install -y webots

# Webots ROS2 packages for Humble
sudo apt install -y ros-humble-webots-ros2 ros-humble-webots-ros2-driver
```

### 2.5 Shell setup and verification
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo rosdep init || true
rosdep update

ros2 doctor --report
webots --version
ros2 pkg list | grep webots_ros2_driver
```

### 2.6 WSL2 GUI checks (only for WSL2 users)
```bash
echo $DISPLAY
glxinfo | grep "OpenGL renderer"
```
If `glxinfo` is missing:
```bash
sudo apt install -y mesa-utils
```

## 3) Workspace Setup
Assuming repository root is this folder (`drone`).

```bash
cd ~/drone
source /opt/ros/humble/setup.bash

rosdep install --from-paths drone_nav_2d --ignore-src -r -y
colcon build --packages-select drone_nav_2d
source install/setup.bash

ros2 pkg list | grep drone_nav_2d
```

## 4) Run (single command)
```bash
source /opt/ros/humble/setup.bash
cd ~/drone
source install/setup.bash
ros2 launch drone_nav_2d drone_nav_launch.py
```

### Run hard shortlist world
```bash
ros2 launch drone_nav_2d drone_nav_launch.py use_hard_world:=true
```

## 5) Package Structure
```text
drone_nav_2d/
├── drone_nav_2d/
│   ├── __init__.py
│   ├── drone_controller.py
│   ├── path_planner.py
│   ├── obstacle_avoidance.py
│   ├── map_publisher.py
│   └── metrics_logger.py
├── worlds/
│   ├── drone_world.wbt
│   └── drone_world_hard.wbt
├── controllers/
│   └── moving_wall_controller/
│       └── moving_wall_controller.py
├── resource/
│   └── drone_nav_2d
├── launch/
│   └── drone_nav_launch.py
├── config/
│   └── nav_params.yaml
├── rviz/
│   └── drone_nav.rviz
├── urdf/
│   └── drone.urdf
├── results/
│   └── .gitkeep
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 6) How to Configure Start/Goal and Planner
Edit `config/nav_params.yaml`:
- `path_planner.ros__parameters.start_xy`
- `path_planner.ros__parameters.goal_xy`
- `map_publisher.ros__parameters.resolution`, `width`, `height`, `inflation_radius_m`
- A* heuristic weights:
  - `heuristic_weight_manhattan`
  - `heuristic_weight_euclidean`

### A* Algorithm Details
- Occupancy grid: `100 x 100` at `0.1 m` resolution (10m x 10m arena)
- Obstacle inflation: map obstacle expansion by safety radius (`drone_radius + margin`)
- Cost: grid path length with optional diagonal motions
- Heuristic: weighted Manhattan + Euclidean
- Fallback: if A* fails, planner switches to RRT and logs warning

## 7) PID Tuning
Edit `config/nav_params.yaml` under `drone_controller.ros__parameters`:
- `kp_x`, `ki_x`, `kd_x`
- `kp_y`, `ki_y`, `kd_y`
- `kp_z`, `ki_z`, `kd_z`
- Anti-windup limits:
  - `integral_limit_xy`
  - `integral_limit_z`

Waypoint switch threshold: `waypoint_tolerance` (default `0.15 m`).

## 8) Topics and Outputs
- `/map` (`nav_msgs/OccupancyGrid`)
- `/planned_path` (`nav_msgs/Path`)
- `/drone_pose` (`geometry_msgs/PoseStamped`)
- `/drone_trajectory` (`nav_msgs/Path`)
- `/cmd_vel` (`geometry_msgs/Twist`)
- `/obstacle_detected` (`std_msgs/Bool`)
- `/replan_event` (`std_msgs/Bool`)
- `/min_obstacle_distance` (`std_msgs/Float32`)
- `/mission_complete` (`std_msgs/Bool`)
- `/obstacle_markers` (`visualization_msgs/MarkerArray`)

Metrics JSON output:
- `results/mission_metrics.json`

## 9) Pre-event Testing Environment (Hard World)
`worlds/drone_world_hard.wbt` includes:
- Maze-like layout with long corridors
- Narrow gaps near `1.2 m`
- SliderJoint-based moving wall obstacle
- Same start/goal frame conventions

### Environment assumptions
- Flat terrain, fixed altitude (`z=0.5m`)
- Known occupancy map
- Static obstacles + one dynamic obstacle (moving wall)
- 2D navigation abstraction (x-y planning)

### Simulation parameters
- Grid resolution: `0.1 m`
- Planner inflation radius: `0.4 m`
- Obstacle trigger threshold: `0.5 m`
- Lidar max range (world): `5.0 m`

### Hardware constraints considered
- Moderate CPU/GPU budget (hackathon laptop class)
- Real-time update rates reduced to practical defaults (20Hz controller)
- Sensor model simplified for deterministic reproducibility

## 10) RViz View
`rviz/drone_nav.rviz` preloads:
- Occupancy grid
- Planned path (green)
- Actual trajectory (blue)
- TF frame (`map -> drone_base`)
- Obstacle markers

## 11) Screenshot / GIF Placeholders
Add artifacts before submission:
- `docs/images/sim_overview.png`
- `docs/images/rviz_path_tracking.png`
- `docs/gifs/full_mission.gif`

## 12) Known Limitations
- Webots bridge topic names can vary by robot/controller plugin setup; remappings may need adjustment.
- Hard world moving wall kinematics may require custom Webots controller scripting for oscillatory behavior on every setup.
- Global map is static and does not update from online SLAM.

## 13) GitHub Repository Setup
From repository root:
```bash
git init
git add .
git commit -m "feat: initialize drone_nav_2d ROS2-Webots project"
git branch -M main
git remote add origin <YOUR_GITHUB_REPO_URL>
git push -u origin main
```

### What to commit
- Source code (`drone_nav_2d/`)
- `README.md`, `package.xml`, `setup.py`, launch/config/world/rviz files
- `.gitignore`

### What to ignore
- `build/`, `install/`, `log/`
- runtime bags and generated results JSON

### Suggested commit message structure
Use Conventional Commits:
- `feat(planner): add weighted A* with RRT fallback`
- `feat(control): add PID waypoint tracking + anti-windup`
- `feat(sim): add normal and hard Webots worlds`
- `docs(readme): add installation and demo instructions`

### Suggested public repo description
"ROS2 Humble + Webots R2023b autonomous 2D drone navigation with A* planning, reactive obstacle avoidance, PID tracking, RViz visualization, and mission metrics logging."

## 14) Deliverables Checklist
- [x] Working Webots simulation launching with one command
- [x] A* path planning with obstacle inflation
- [x] PID-based drone controller following waypoints
- [x] Reactive obstacle avoidance layer
- [x] RViz2 visualization of map, path, and trajectory
- [x] Performance metrics JSON output
- [x] Two test worlds (normal + hard)
- [x] Complete README with all installation steps
- [x] Proper ROS2 package structure (`package.xml`, `setup.py`)
- [x] Clean, documented code for hackathon submission

## 15) Team / Contact
- Team: Robothon NIT Raipur Submission Team
- Contact: `team@example.com` (update before public submission)

## 16) Technical Documentation
- Detailed architecture and algorithm notes: `docs/TECHNICAL_DOCUMENTATION.md`
