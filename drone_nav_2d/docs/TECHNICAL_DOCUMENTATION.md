# Technical Documentation — drone_nav_2d

## 1. Architecture Overview
This project implements layered autonomous navigation with advanced path planning:

1. **Global layer (Path Planner)**
   - **PRM (Probabilistic Roadmap)** for initial roadmap construction (speed)
   - **Informed RRT*** for high-quality path planning (asymptotically optimal)
   - **D* Lite** for dynamic replanning when obstacles change or move
   - Fallback between algorithms ensures robust path finding
   
2. **Local layer (Obstacle Avoidance)**
   - Potential-field reactive avoidance from LiDAR scan
   - Triggers repanning when obstacle intrudes into safety radius
   
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

## 3. Path Planning Algorithms

### 3.1 PRM (Probabilistic Roadmap)
**Purpose**: Build reusable roadmap for fast planning

**Algorithm**:
1. Sample N random configurations uniformly in free space
2. Connect nearby nodes within `connection_radius`
3. Limit connections per node to `max_connections` (nearest neighbors)
4. Build bidirectional graph of collision-free paths

**Parameters**:
- `prm_num_samples`: 400 (2D), 600 (3D) — roadmap density
- `prm_connection_radius`: 2.0m (2D), 2.5m (3D) — connection threshold
- `prm_max_connections`: 15 (2D), 20 (3D) — max edges per node

**Advantages**:
- Fast planning after roadmap built
- Good for repeated queries
- Works well in static/slowly-changing environments

### 3.2 Informed RRT* (Asymptotically Optimal)
**Purpose**: Find high-quality paths with rapid exploration

**Algorithm**:
1. Grow tree from start toward goal with random sampling
2. **Informed sampling**: After obtaining a path, sample from ellipsoid defined by start, goal, and best path cost
3. **Rewiring**: For new nodes, rewire tree to nearby nodes that benefit from better path
4. Dynamic rewiring radius: `r = α * sqrt(ln(n)/n)` where α is `rewire_radius_factor`

**Parameters**:
- `irrt_max_iterations`: 3000 (2D), 4000 (3D) — planning budget
- `irrt_step_size_m`: 0.4m (2D), 0.5m (3D) — extension step
- `irrt_goal_sample_rate`: 0.12 — probability of sampling goal (12%)
- `irrt_rewire_radius_factor`: 30.0 (2D), 35.0 (3D) — rewiring coefficient

**Advantages**:
- Asymptotically optimal (converges to optimal solution)
- Superior path quality vs. standard RRT
- Handles complex, non-convex spaces
- Adaptive goal sampling improves efficiency

### 3.3 D* Lite (Dynamic Replanning)
**Purpose**: Efficiently handle dynamic obstacles without global replanning

**Algorithm**:
1. Maintains consistent heuristic search state
2. When environment changes:
   - Detect affected nodes
   - Update their cost estimates
   - Add to processing queue
3. Incremental search only processes affected regions

**Parameters**:
- `dstar_enabled`: `true` — enable dynamic replanning

**Advantages**:
- Handles moving obstacles efficiently
- Reuses computation from previous plans
- Much faster than global replanning
- Ideal for dynamic environments

### 3.4 Algorithm Selection Strategy
1. **Initial planning**: Build PRM roadmap for fast reference
2. **Primary planner**: Use Informed RRT* for high-quality paths
3. **Dynamic replanning**: Switch to D* Lite when obstacles move/appear
4. **Fallback**: Each algorithm tries independently for robustness

## 4. Obstacle Inflation
- Obstacles expanded by `inflation_radius_m` (default 0.4m)
- Effective safety margin = `drone_radius_m + inflation_radius_m`
- Collision checker uses inflated grid

## 5. PID Controller Design
For each axis `u in {x,y,z}`:
- Error: `e_u = target_u - state_u`
- Integral with anti-windup clamp: `I_u = clamp(I_u + e_u*dt, -Imax_u, Imax_u)`
- Derivative: `D_u = (e_u - e_u_prev)/dt`
- Command: `cmd_u = kp_u*e_u + ki_u*I_u + kd_u*D_u`

## 6. Local Obstacle Avoidance
Input:
- Laser scan (`/webots/drone/scan`)

Behavior:
- If minimum range < 0.5m:
  - Compute repulsive vector from beams
  - Add tangential term to avoid deadlocks
  - Trigger `/replan_request`

Outputs:
- `/obstacle_detected` (`Bool`)
- `/min_obstacle_distance` (`Float32`)

## 7. Performance Metrics
Mission-end metrics in `results/mission_metrics.json`:
- `total_path_length_m`
- `mission_completion_time_s`
- `replanning_events`
- `minimum_obstacle_distance_m`
- `path_smoothness_score`
- `success` / `failure`
- Algorithm statistics (PRM nodes, RRT* iterations, etc.)

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
