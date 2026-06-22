# vacuum_coverage

Coverage path planning package for autonomous vacuum robots. Provides a
boustrophedon (lawnmower) coverage planner as a Nav2 global planner plugin.

---

## Concept: Coverage Path Planning

Coverage path planning (CPP) is the problem of finding a path that passes over
**every reachable free cell** in an environment, in contrast to traditional
A→B path planning which finds a path between two points.

### Boustrophedon ("lawnmower") pattern

The robot sweeps the environment in parallel rows, like a lawnmower:

```
┌─────────────────────┐
│ → → → → → → → → → → │   row 0 (left→right)
│ ← ← ← ← ← ← ← ← ← ← │   row 1 (right→left)
│ → → → → → → → → → → │   row 2 (left→right)
│ ← ← ← ← ← ← ← ← ← ← │   row 3 (right→left)
└─────────────────────┘
```

The planner decides the sweep direction (horizontal or vertical) based on the
room's aspect ratio. Transitions between rows and columns use A* pathfinding.

### Room decomposition

Real indoor environments are not a single convex room. Walls, furniture, and
doorways divide the space into disconnected components. The planner uses a
two-level decomposition:

1. **Original grid** (8-connectivity) — detects rooms using connected-component
   labelling. 8-connectivity means diagonal contact counts as connection, which
   prevents 1-pixel diagonal walls from splitting a room.

2. **Inflated grid** — the original grid dilated by the robot's radius. This is
   the space the robot can physically occupy. Sub-rooms within each original
   room are detected here. A single original room may contain multiple inflated
   sub-rooms (e.g., a narrow passage that becomes impassable when inflated).

### Why two A* variants?

| Variant | Used for | Grid | Corner-cutting |
|---------|----------|------|----------------|
| `astarCC` | Intra-room sweep transitions | Inflated | Yes (rejected) |
| `astarNoCC` | Inter-room connectors | Original | No (allowed) |

- **Corner-cutting** rejects diagonal moves where both orthogonal neighbours are
  obstacles. On the inflated grid, this keeps the robot's body away from convex
  corners. Rejected e.g.: moving diagonally past a wall corner.

- **Inter-room connectors** use `astarNoCC` on the **original** grid because
  the inflated grid may close narrow diagonal doorways that the robot could
  actually pass through. The resulting path is later validated against the
  original occupancy grid.

---

## Architecture

Two planners run side-by-side in the Nav2 `planner_server`:

| Plugin ID | Type | Role |
|---|---|---|
| `GridBased` | `nav2_navfn_planner::NavfnPlanner` | Normal A→B navigation (default) |
| `CoverageBased` | `vacuum_coverage::BoustrophedonCoveragePlanner` | Coverage on demand |

During normal operation, the behavior tree's `ComputePathToPose` action uses
`planner_id="GridBased"`, so Navfn handles all point-to-point goals. The
coverage plugin sits idle until explicitly triggered.

A separate `coverage_executor_node` subscribes to the published coverage path
and sends it as a `FollowWaypoints` action goal to execute the coverage
pattern.

### Flow diagram

```
User sends goal (RViz)                 User triggers coverage
        │                                      │
        ▼                                      ▼
┌──────────────────┐                ┌─────────────────────┐
│  Nav2 BT         │                │ /planner_server/    │
│  ComputePathTo-  │                │ replan (service)    │
│  Pose (GridBased)│                └─────────┬───────────┘
└────────┬─────────┘                          │
         │                                    ▼
         ▼                         ┌─────────────────────┐
┌──────────────────┐               │ Coverage plugin     │
│ NavfnPlanner     │               │ computes path,      │
│ (GridBased)      │               │ publishes to        │
└────────┬─────────┘               │ /planner_server/    │
         │                        │ coverage_path       │
         ▼                        └─────────┬───────────┘
┌──────────────────┐                          │
│ Controller       │                          ▼
│ follows path     │               ┌─────────────────────┐
└──────────────────┘               │ coverage_executor   │
                                   │ sends FollowWay-    │
                                   │ points goal         │
                                   └─────────┬───────────┘
                                             │
                                             ▼
                                   ┌─────────────────────┐
                                   │ Waypoint follower   │
                                   │ executes coverage   │
                                   └─────────────────────┘
```

---

## Usage

### Launch

```bash
ros2 launch vacuum_coverage vacuum.launch.py \
  slam:=False \
  map:=/path/to/map.yaml
```

Launches:
- Full Nav2 stack (planner_server, controller, costmaps, BT, AMCL, RViz)
- Coverage executor node

### Normal navigation

Send `NavigateToPose` goals in RViz as usual — Navfn handles all A→B planning.

### Coverage on demand

```bash
# Step 1 — compute the coverage path
ros2 service call /planner_server/replan std_srvs/srv/Trigger

# Step 2 — execute it
ros2 service call /start_coverage std_srvs/srv/Trigger
```

To recompute a fresh coverage path (e.g. after mapping new areas):

```bash
ros2 service call /planner_server/replan std_srvs/srv/Trigger
ros2 service call /start_coverage std_srvs/srv/Trigger
```

---

## Parameters

Defined in `vacuum_nav/param/waffle.yaml` under the `CoverageBased` namespace:

| Parameter | Default | Description |
|---|---|---|
| `spacing` | 0.3 m | Distance between boustrophedon sweep rows. Smaller values produce tighter coverage but longer paths. |
| `robot_radius` | 0.2 m | Robot body radius used for obstacle inflation. Increase to add more clearance from walls; decrease to access tighter passages. |
| `min_room_area` | 0.5 m² | Minimum connected-component area. Free-space regions smaller than this are treated as noise and ignored. |

### Tuning guidelines

- **Robot getting stuck on corners**: Increase `robot_radius`. The inflation
  pushes the planned path further from obstacles.
- **Missing narrow doorways**: Decrease `robot_radius`. The inflated grid may
  close passages the robot can physically fit through.
- **Coverage too sparse**: Decrease `spacing`. Sweep rows will be closer
  together.

---

## Algorithm details

### Pipeline (on trigger or first `createPlan()` call)

```
costmap        inflateGrid(rpx)        inflateGrid(rpx)
   │                  │                      │
   ▼                  ▼                      ▼
occ_grid         occ_grid (kept)        inf_grid
   │                                        │
   ▼                                        ▼
findRooms(8-conn)                      findRooms(8-conn)
   │                                        │
   ▼                                        ▼
orig_rooms[0] (interior)              inf_rooms (sub-rooms)
   │                                        │
   └───────────── overlap ──────────────────┘
                        │
                        ▼
           For each overlapping sub-room:
           ┌─────────────────────────┐
           │ coverRoom(inf_grid,     │
           │   sub_room, spx)        │
           │   → boustrophedon sweep │
           │   → astarCC transitions │
           └───────────┬─────────────┘
                       │
                       ▼
           Connect sub-rooms:
           astarNoCC(occ_grid)
                       │
                       ▼
                  validate(occ_grid)
                       │
                       ▼
               makePath → cached_path_
```

### Key functions

| Method | Description |
|---|---|
| `inflateGrid(grid, r)` | Chebyshev-disk dilation. Every obstacle cell expands by radius `r` (Manhattan disk: `dx²+dy² ≤ r²`). |
| `findRooms(grid, min_cells)` | BFS connected-component labelling using 8-connectivity. Returns rooms sorted by size descending. |
| `astarCC(grid, start, goal)` | A* with corner-cutting check. Rejects diagonal moves where both orthogonal neighbours are obstacles. Used on inflated grid for intra-row transitions. |
| `astarNoCC(grid, start, goal)` | A* without corner-cutting. Used on original grid for inter-room connectors so diagonal doorways remain passable. |
| `coverRoom(grid, cells, stride)` | Generates boustrophedon waypoints for one room. For each sweep line, free cells within the room are collected and traversed in alternating directions. Gaps larger than 1 px use astarCC. |
| `validate(path, occ_grid)` | Checks every waypoint and every transition segment against the original occupancy grid. Reports obstacle crossings (WAYPOINT on obstacle, TRANSITION through obstacle). |
| `orientPath(path)` | Assigns orientation (yaw) to each pose based on direction to the next pose. |

---

## Files

| File | Role |
|---|---|
| `include/vacuum_coverage/boustrophedon_coverage_planner.hpp` | Header |
| `src/boustrophedon_coverage_planner.cpp` | Plugin implementation |
| `boustrophedon_coverage_plugin.xml` | Pluginlib registration |
| `src/coverage_executor_node.cpp` | Subscribes to coverage path, sends FollowWaypoints |
| `launch/vacuum.launch.py` | Main launch file |
| `scripts/coverage_standalone.py` | Original offline Python version |
| `scripts/coverage_planner_offline.py` | Alternative offline Python planner |
| `scripts/publish_coverage_path.py` | Publishes a saved YAML path |
| `scripts/send_coverage_goal.py` | Sends coverage as Nav2 goal |

---

## Dependencies

- ROS 2 Jazzy
- Nav2 (`nav2_core`, `nav2_costmap_2d`, `nav2_msgs`, `nav2_bringup`)
- `pluginlib`, `tf2_ros`, `rclcpp_lifecycle`
