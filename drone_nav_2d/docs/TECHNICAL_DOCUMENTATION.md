# Technical Documentation — drone_nav_2d

## 1. Architecture Overview
This project implements layered autonomous navigation in simulation:

1. **Global layer (Path Planner)**
   - Occupancy-grid A* with obstacle inflation
   - Fallback RRT if A* cannot find a path
2. **Local layer (Obstacle Avoidance)**
   - Potential-field reactive avoidance from LiDAR scan
   - Triggers replanning when obstacle intrudes into safety radius
3. **Control layer (Drone Controller)**
   - PID waypoint tracking for x, y, z
   - Anti-windup integrator clipping
4. **Evaluation layer (Metrics Logger)**
   - Computes mission KPIs and writes JSON

## 2. Coordinate and Map Model
- World frame: `map`
- Arena: 10m x 10m centered near origin
- Altitude constrained to `z = 0.5m`
- Occupancy resolution default: `0.1m`
- Grid size default: `100 x 100`

Mapping equations:
- `gx = floor((x - origin_x)/resolution)`
- `gy = floor((y - origin_y)/resolution)`
- `x = origin_x + (gx + 0.5)*resolution`
- `y = origin_y + (gy + 0.5)*resolution`

## 3. A* Planner Details
### 3.1 Cost and heuristic
- Neighbor expansion: 4- or 8-connected (8-connected by default)
- Step cost:
  - Axial step = `1.0`
  - Diagonal step = `sqrt(2)`
- Heuristic:
  - `h = w_m * Manhattan + w_e * Euclidean`
  - defaults: `w_m = 0.7`, `w_e = 0.3`

### 3.2 Obstacle inflation
- Obstacles are expanded using binary dilation kernel
- Effective safety radius: `drone_radius + margin` (configured as `inflation_radius_m`)

### 3.3 RRT fallback
If A* returns no path:
- random sampling in map bounds
- nearest-node steering with fixed step size
- line-of-sight collision check
- path reconstructed from parent pointers

## 4. PID Controller Design
For each axis `u in {x,y,z}`:
- Error: `e_u = target_u - state_u`
- Integral with anti-windup clamp: `I_u = clamp(I_u + e_u*dt, -Imax_u, Imax_u)`
- Derivative: `D_u = (e_u - e_u_prev)/dt`
- Command: `cmd_u = kp_u*e_u + ki_u*I_u + kd_u*D_u`
- Velocity limits are applied independently for XY and Z.

Waypoint transition:
- Advance waypoint when planar distance `< waypoint_tolerance`.

## 5. Local Obstacle Avoidance
Input:
- Laser scan (`/webots/drone/scan`)

Behavior:
- if minimum range `< 0.5m` (default threshold):
  - compute repulsive vector from beams inside influence distance
  - add tangential term to avoid deadlocks
  - override global command temporarily with `/avoidance_cmd_vel`
  - trigger `/replan_request`

Outputs:
- `/obstacle_detected` (`Bool`)
- `/min_obstacle_distance` (`Float32`)
- `/avoidance_cmd_vel` (`Twist`)

## 6. Performance Metrics
Mission-end metrics in `results/mission_metrics.json`:
- `total_path_length_m`
- `mission_completion_time_s`
- `replanning_events`
- `minimum_obstacle_distance_m`
- `path_smoothness_score` (sum of absolute heading changes)
- `success` / `failure`

## 7. Simulation Worlds
### Normal world
- 10m x 10m flat floor
- drone at `[-4, 0, 0.5]`
- goal marker at `[4, 0, 0.5]`
- 8–10 static obstacle solids

### Hard world
- maze-like static walls
- narrow corridors near 1.2m
- one SliderJoint moving obstacle
- same start/goal convention

## 8. Runtime Graph
Main launch starts:
- Webots simulator
- webots_ros2 driver bridge
- map publisher
- path planner
- drone controller
- obstacle avoidance
- metrics logger
- RViz2
- rosbag record

## 9. Compute and Sensor Assumptions
- Known map (no online SLAM)
- Fixed-height 2D approximation
- LiDAR-like distance scan available from simulator bridge
- Suitable for laptop-class CPU/GPU under hackathon constraints
