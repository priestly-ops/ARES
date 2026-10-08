# Graph Report - ares_jackal_description  (2026-09-24)

## Corpus Check
- Corpus is ~135 words - fits in a single context window. You may not need a graph.

## Summary
- 15 nodes · 15 edges · 3 communities
- Extraction: 100% EXTRACTED · 0% INFERRED · 0% AMBIGUOUS
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- Robot State Publisher
- Display Launch
- Description Assets

## God Nodes (most connected - your core abstractions)
1. `Description Asset Installation` - 4 edges
2. `ares_jackal_description` - 1 edges
3. `URDF Assets` - 1 edges
4. `Mesh Assets` - 1 edges
5. `Launch Assets` - 1 edges

## Surprising Connections (you probably didn't know these)
- None detected - all connections are within the same source files.

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **Installed Description Resources** — src_ares_jackal_description_cmakelists_urdf_assets, src_ares_jackal_description_cmakelists_mesh_assets, src_ares_jackal_description_cmakelists_launch_assets [EXTRACTED 1.00]

## Communities (3 total, 0 thin omitted)

### Community 0 - "Robot State Publisher"
Cohesion: 0.40
Nodes (3): ament_index_python_packages, os, xacro

### Community 1 - "Display Launch"
Cohesion: 0.40
Nodes (3): launch_ros_actions, launch_ros_parameter_descriptions, launch_substitutions

### Community 2 - "Description Assets"
Cohesion: 0.40
Nodes (5): ares_jackal_description, Description Asset Installation, Launch Assets, Mesh Assets, URDF Assets

## Knowledge Gaps
- **4 isolated node(s):** `ares_jackal_description`, `URDF Assets`, `Mesh Assets`, `Launch Assets`
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 9 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **What connects `ares_jackal_description`, `URDF Assets`, `Mesh Assets` to the rest of the system?**
  _4 weakly-connected nodes found - possible documentation gaps or missing edges._