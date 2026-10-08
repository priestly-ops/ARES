# Graph Report - ares_localization  (2026-09-24)

## Corpus Check
- Corpus is ~1,718 words - fits in a single context window. You may not need a graph.

## Summary
- 74 nodes · 96 edges · 10 communities (8 shown, 2 thin omitted)
- Extraction: 96% EXTRACTED · 4% INFERRED · 0% AMBIGUOUS · INFERRED: 4 edges (avg confidence: 0.85)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Localization Runtime Dependencies
- Nav2 and SLAM Configuration
- Waypoint Mission Logic
- Initial Pose Synchronization
- EKF Sensor Fusion
- Localization Launch Files
- Velocity Command Adapter
- Localization Package Resources
- Python Packaging

## God Nodes (most connected - your core abstractions)
1. `WaypointMission` - 10 edges
2. `InitialPoseSync` - 7 edges
3. `Nav2 Navigation Stack Configuration` - 7 edges
4. `CmdVelAdapter` - 5 edges
5. `Baseline EKF Filter Configuration` - 4 edges
6. `/ares/scan LaserScan` - 4 edges
7. `SLAM Toolbox Mapping Configuration` - 4 edges
8. `EKF Filter Node Configuration` - 3 edges
9. `AMCL Localization` - 3 edges
10. `Filtered Odometry` - 3 edges

## Surprising Connections (you probably didn't know these)
- `EKF Filter Node Configuration` --semantically_similar_to--> `Baseline EKF Filter Configuration`  [INFERRED] [semantically similar]
  config/ekf.yaml → config/ekf_baseline.yaml
- `Baseline EKF Filter Configuration` --shares_data_with--> `Filtered Odometry`  [INFERRED]
  config/ekf_baseline.yaml → config/nav2_params.yaml
- `Map-Odom-Base Frame Chain` --conceptually_related_to--> `AMCL Localization`  [INFERRED]
  config/slam_toolbox.yaml → config/nav2_params.yaml
- `SLAM Toolbox Mapping Configuration` --conceptually_related_to--> `Warehouse Map Server`  [INFERRED]
  config/slam_toolbox.yaml → config/nav2_params.yaml
- `Laser Scan Matching` --shares_data_with--> `/ares/scan LaserScan`  [EXTRACTED]
  config/slam_toolbox.yaml → config/nav2_params.yaml

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Localization Sensor Dataflow** — src_ares_localization_config_ekf_ares_odom, src_ares_localization_config_ekf_baseline_imu_angular_velocity_fusion, src_ares_localization_config_nav2_params_ares_scan, src_ares_localization_config_nav2_params_filtered_odometry [INFERRED 0.85]
- **Nav2 Navigation Pipeline** — src_ares_localization_config_nav2_params_amcl, src_ares_localization_config_nav2_params_navfn_planner, src_ares_localization_config_nav2_params_regulated_pure_pursuit, src_ares_localization_config_nav2_params_navigation_orchestration, src_ares_localization_config_nav2_params_global_costmap, src_ares_localization_config_nav2_params_local_costmap [EXTRACTED 1.00]
- **Map-Frame Localization Modes** — src_ares_localization_config_nav2_params_amcl, src_ares_localization_config_nav2_params_map_server, src_ares_localization_config_slam_toolbox_slam_toolbox, src_ares_localization_config_slam_toolbox_map_odom_frames [INFERRED 0.85]

## Communities (10 total, 2 thin omitted)

### Community 0 - "Localization Runtime Dependencies"
Cohesion: 0.17
Nodes (13): action_msgs_msg, csv, datetime, geometry_msgs_msg, math, nav2_msgs_action, nav2_msgs_srv, rclpy (+5 more)

### Community 1 - "Nav2 and SLAM Configuration"
Cohesion: 0.20
Nodes (14): AMCL Localization, /ares/scan LaserScan, Filtered Odometry, Global Static Obstacle Costmap, Local Rolling Obstacle Costmap, Warehouse Map Server, Nav2 Navigation Stack Configuration, A-Star Navfn Global Planner (+6 more)

### Community 2 - "Waypoint Mission Logic"
Cohesion: 0.31
Nodes (3): main(), Node, WaypointMission

### Community 3 - "Initial Pose Synchronization"
Cohesion: 0.36
Nodes (4): InitialPoseSync, main(), Node, Read /world/ares_world/dynamic_pose/info and extract the ares_jackal world pose.

### Community 4 - "EKF Sensor Fusion"
Cohesion: 0.25
Nodes (8): /ares/odom Input, Baseline EKF Filter Configuration, IMU Angular Velocity Fusion, Independent Heading Sources, Wheel Planar Velocity Fusion, EKF Filter Node Configuration, Odom to Base Footprint Transform, Planar Odometry Fusion

### Community 5 - "Localization Launch Files"
Cohesion: 0.38
Nodes (3): ament_index_python_packages, launch_ros_actions, os

### Community 6 - "Velocity Command Adapter"
Cohesion: 0.40
Nodes (3): CmdVelAdapter, main(), Node

### Community 7 - "Localization Package Resources"
Cohesion: 0.67
Nodes (3): ares_localization Package, Localization Runtime Scripts, Localization Config and Launch Resources

## Knowledge Gaps
- **7 isolated node(s):** `Localization Runtime Scripts`, `Localization Config and Launch Resources`, `/ares/odom Input`, `Odom to Base Footprint Transform`, `Wheel Planar Velocity Fusion` (+2 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 27 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **2 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `WaypointMission` connect `Waypoint Mission Logic` to `Localization Runtime Dependencies`?**
  _High betweenness centrality (0.120) - this node is a cross-community bridge._
- **Why does `InitialPoseSync` connect `Initial Pose Synchronization` to `Localization Runtime Dependencies`?**
  _High betweenness centrality (0.092) - this node is a cross-community bridge._
- **Why does `CmdVelAdapter` connect `Velocity Command Adapter` to `Localization Runtime Dependencies`?**
  _High betweenness centrality (0.049) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `Baseline EKF Filter Configuration` (e.g. with `Filtered Odometry` and `EKF Filter Node Configuration`) actually correct?**
  _`Baseline EKF Filter Configuration` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Localization Runtime Scripts`, `Localization Config and Launch Resources`, `/ares/odom Input` to the rest of the system?**
  _7 weakly-connected nodes found - possible documentation gaps or missing edges._