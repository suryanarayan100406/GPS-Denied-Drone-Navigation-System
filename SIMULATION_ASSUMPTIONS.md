# Simulation Assumptions (Short)

## 1) Environment assumptions
- Runs in WSL Ubuntu on Windows with ROS 2 Jazzy sourced.
- Webots binary: `/home/surya/webots/webots` and `WEBOTS_HOME=/home/surya/webots`.
- Workspace overlay is sourced from `install/setup.bash`.
- Default scenario: `world_profile:=realistic` using `drone_world_realistic.wbt`.
- External controller target is `WEBOTS_CONTROLLER_URL=drone`.

## 2) Simulation parameters
- **Controller:** `20 Hz`, tolerance `0.15 m`, max vel `1.2 m/s` (XY), `0.6 m/s` (Z), altitude `0.5 m`, PID gains set in `nav_params.yaml`.
- **Planner:** grid `0.1 m`, map `10x10 m`, inflation `0.4 m`, drone radius `0.2 m`, diagonal moves enabled, replan timer `1.0 Hz`, default start/goal `(-4,0)` to `(4,0)`, RRT fallback enabled (`2500` iters, `0.35 m` step).
- **Avoidance:** threshold `0.5 m`, influence `1.2 m`, repulsive `0.9`, tangential `0.45`, max avoidance vel `0.8 m/s`.
- **Logging:** metrics to `results/mission_metrics.json`; rosbag output auto-suffixed if folder exists.

## 3) Performance metrics
Latest run (`results/mission_metrics.json`):
- Success: `true`
- Path length: `10.2516 m`
- Completion time: `46.1356 s`
- Replans: `1`
- Min obstacle distance: `999.0 m`
- Smoothness score: `199.8779`

## 4) Hardware/software constraints considered
- WSL GUI and stale-process conflicts can break Webots/driver attachment.
- Only one active simulation launch should run at a time.
- RViz may show GLSL warnings on this setup; simulation can still run.
- Correct Webots path/env is required for stable startup.
- Real-time behavior depends on host CPU/GPU load.

Updated: **2026-03-18**