# Graph Report - ares_simulation  (2026-09-24)

## Corpus Check
- Corpus is ~654 words - fits in a single context window. You may not need a graph.

## Summary
- 39 nodes · 42 edges · 5 communities (4 shown, 1 thin omitted)
- Extraction: 88% EXTRACTED · 10% INFERRED · 2% AMBIGUOUS · INFERRED: 4 edges (avg confidence: 0.9)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Simulation Launch Orchestration
- Core ROS Gazebo Bridges
- Simulation Assets and Maps
- Camera Sensor Bridges
- Launch Factory Helpers

## God Nodes (most connected - your core abstractions)
1. `Minimal ROS-Gazebo Bridge` - 7 edges
2. `Primary ROS-Gazebo Bridge` - 6 edges
3. `setup()` - 4 edges
4. `Baseline ROS-Gazebo Bridge` - 4 edges
5. `Camera ROS-Gazebo Bridge` - 4 edges
6. `Simulation Asset Installation` - 3 edges
7. `ARES Warehouse Occupancy Map` - 3 edges
8. `generate_launch_description()` - 2 edges
9. `Command Velocity Bridge` - 2 edges
10. `Odometry Bridge` - 2 edges

## Surprising Connections (you probably didn't know these)
- `Simulation Asset Installation` --references--> `Primary ROS-Gazebo Bridge`  [EXTRACTED]
  CMakeLists.txt → config/bridge.yaml
- `Simulation Asset Installation` --references--> `ARES Warehouse Occupancy Map`  [EXTRACTED]
  CMakeLists.txt → maps/ares_warehouse.yaml
- `Primary ROS-Gazebo Bridge` --semantically_similar_to--> `Minimal ROS-Gazebo Bridge`  [INFERRED] [semantically similar]
  config/bridge.yaml → config/bridge_minimal.yaml
- `Camera ROS-Gazebo Bridge` --semantically_similar_to--> `Legacy Sensor Bridge Entries`  [INFERRED] [semantically similar]
  config/bridge_camera.yaml → config/bridge.yaml
- `Baseline ROS-Gazebo Bridge` --semantically_similar_to--> `Minimal ROS-Gazebo Bridge`  [INFERRED] [semantically similar]
  config/bridge_baseline.yaml → config/bridge_minimal.yaml

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **ARES Simulation Bridge Configuration Suite** — src_ares_simulation_config_bridge_primary_bridge, src_ares_simulation_config_bridge_baseline_baseline_bridge, src_ares_simulation_config_bridge_camera_camera_bridge, src_ares_simulation_config_bridge_minimal_minimal_bridge [INFERRED 0.95]
- **Shared ARES Control and State Topics** — src_ares_simulation_config_bridge_command_velocity_bridge, src_ares_simulation_config_bridge_odometry_bridge, src_ares_simulation_config_bridge_imu_bridge, src_ares_simulation_config_bridge_baseline_baseline_motion_sensors, src_ares_simulation_config_bridge_minimal_minimal_control_and_sensing [EXTRACTED 1.00]

## Communities (5 total, 1 thin omitted)

### Community 0 - "Simulation Launch Orchestration"
Cohesion: 0.13
Nodes (14): ament_index_python_packages, atexit, launch_actions, Healthy reference launch. Derive optional sensor world in a private temp…, launch_event_handlers, launch_events, launch_ros_actions, launch_substitutions (+6 more)

### Community 1 - "Core ROS Gazebo Bridges"
Cohesion: 0.29
Nodes (10): Baseline ROS-Gazebo Bridge, Baseline Motion and Sensor Topics, Simulation Clock Bridge, Laser Scan Bridge, Command Velocity Bridge, IMU Bridge, Minimal ROS-Gazebo Bridge, Minimal Control and Sensing Topics (+2 more)

### Community 2 - "Simulation Assets and Maps"
Cohesion: 0.40
Nodes (5): ares_simulation Package, Simulation Asset Installation, ares_warehouse.pgm, Occupancy Grid Parameters, ARES Warehouse Occupancy Map

### Community 3 - "Camera Sensor Bridges"
Cohesion: 0.40
Nodes (5): Camera ROS-Gazebo Bridge, Camera Information Bridge, Camera Image Bridge, Camera Point Cloud Bridge, Legacy Sensor Bridge Entries

## Ambiguous Edges - Review These
- `Primary ROS-Gazebo Bridge` → `Legacy Sensor Bridge Entries`  [AMBIGUOUS]
  config/bridge.yaml · relation: references

## Knowledge Gaps
- **9 isolated node(s):** `ares_simulation Package`, `Baseline Motion and Sensor Topics`, `Simulation Clock Bridge`, `Camera Image Bridge`, `Camera Information Bridge` (+4 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 25 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **1 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **What is the exact relationship between `Primary ROS-Gazebo Bridge` and `Legacy Sensor Bridge Entries`?**
  _Edge tagged AMBIGUOUS (relation: references) - confidence is low._
- **Why does `Primary ROS-Gazebo Bridge` connect `Core ROS Gazebo Bridges` to `Simulation Assets and Maps`, `Camera Sensor Bridges`?**
  _High betweenness centrality (0.166) - this node is a cross-community bridge._
- **Why does `Simulation Asset Installation` connect `Simulation Assets and Maps` to `Core ROS Gazebo Bridges`?**
  _High betweenness centrality (0.090) - this node is a cross-community bridge._
- **Are the 2 inferred relationships involving `Minimal ROS-Gazebo Bridge` (e.g. with `Baseline ROS-Gazebo Bridge` and `Primary ROS-Gazebo Bridge`) actually correct?**
  _`Minimal ROS-Gazebo Bridge` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `ares_simulation Package`, `Baseline Motion and Sensor Topics`, `Simulation Clock Bridge` to the rest of the system?**
  _9 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Simulation Launch Orchestration` be split into smaller, more focused modules?**
  _Cohesion score 0.13333333333333333 - nodes in this community are weakly interconnected._