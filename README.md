# warehouse_bot_package

Three identical `warehouse_bot` robots (white body, black wheels) in the AWS
RoboMaker small-warehouse world, on **ROS 2 Humble + Gazebo Classic 11 + Nav2**.

Two demos:

* **Convoy (default):** `robot1` is the leader and navigates autonomously with
  Nav2; `robot2` and `robot3` follow it, tracing the leader's path at a fixed,
  collision-safe spacing.  ->  `convoy.launch.py`
* **Independent fleet:** all three navigate on their own namespaced Nav2 stacks,
  plus a fleet coordinator that shows inter-robot communication.
  ->  `bringup.launch.py`

---

## 0. Convoy demo (leader + 2 followers)

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  aws_robomaker_small_warehouse_world warehouse_bot_package
source install/setup.bash
ros2 launch warehouse_bot_package convoy.launch.py
```
Wait ~20 s (Gazebo + RViz open), then give the LEADER a goal:
* RViz **"2D Goal Pose"** (publishes `/robot1/goal_pose`), or
* `ros2 topic pub --once /robot1/goal_pose geometry_msgs/PoseStamped "{header: {frame_id: map}, pose: {position: {x: -1.5, y: -1.0}, orientation: {w: 1.0}}}"`

(This demo runs only the Nav2 **planner** for the leader, not `bt_navigator`, so
the goal goes on the `/robot1/goal_pose` topic - there is no `navigate_to_pose`
action here. The full `navigate_to_pose` / `fleet_send_goals.py` path is the
independent-fleet demo, `bringup.launch.py`, below.)

The leader plans with Nav2 and drives the planned path with a smooth
pure-pursuit law (`leader_controller.py`); the two followers
trail it in a line at ~0.9 m spacing along the leader's path. Send another goal
and the whole convoy moves again. Headless: add `gui:=false rviz:=false`.

**One-command there-and-back tour** - instead of sending goals by hand, run the
mission node and the convoy drives a list of waypoints (go somewhere, then back
to another position) on its own:

```bash
ros2 launch warehouse_bot_package convoy.launch.py mission:=true   # uses config
# or drive a custom tour by hand:
ros2 run warehouse_bot_package convoy_mission.py pick_a home_1
ros2 run warehouse_bot_package convoy_mission.py pick_a drop_x --loop
```

Waypoints (station names) and `loop` come from `config/fleet.yaml` (`mission:`).

**What happens if a robot gets stuck or is in the way**
- **Leader stuck** (wall, dead-end, no progress): it auto-recovers - rotates in
  place to find a way out (or backs up if no follower is close behind), replans,
  and retries. After a few failed attempts it gives up the goal and logs it
  rather than spinning forever.
- **Planner finds no path**: it keeps replanning (faster while blocked) and logs
  that the goal may be blocked/unreachable, instead of silently freezing.
- **Obstacle in a follower's way** (something dropped after the leader passed):
  the follower slows as it nears the obstacle, stops if it must, and if it stays
  blocked it **side-steps around** it (alternating sides) then re-acquires the
  leader's trail.
- **One robot in another's way**: every robot knows every other's pose and stops
  before contact; the traffic manager holds the lower-priority one until clear;
  and if you send the leader back through the convoy, the followers pull aside to
  let it pass. All of these are tunable in `config/fleet.yaml`
  (`safety:` / `recovery:`).

**How the convoy works**
- **Leader** - Nav2's real **planner** computes the global path on the warehouse
  map (`compute_path_to_pose`), and `leader_controller.py` drives along it with a
  smooth **pure-pursuit** law. (Pure pursuit is used instead of the stock
  controller because RPP/rotation-shim oscillates at the 180° turn-around on this
  diff-drive robot.) Localisation is a ground-truth `map->odom`
  (`gt_localization.py`, standing in for a fleet-localization source so the leader
  never gets lost).
- **Followers** - `convoy_controller.py` records the leader's travelled path as a
  breadcrumb trail and steers each follower along it with pure pursuit +
  feed-forward speed, holding its gap. They always head to a valid target (trail
  point, or a slot behind the leader on short moves) so they never freeze, and
  they re-acquire the path after yielding.
- **Collision avoidance** - every robot knows every other robot's pose, so each
  controller slows/stops before driving into another robot. No crashes even when
  paths cross or robots bunch up.
- **Yielding** - if you send the leader *back through* the convoy, the followers
  in its path pull aside to let it pass, then re-form behind it.
- The leader path is on `/robot1/plan`; the convoy trail on `/convoy/leader_path`.

---

## 1. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install \
  --packages-select aws_robomaker_small_warehouse_world warehouse_bot_package
source install/setup.bash
```

> The AWS warehouse package must be on its **ros2** branch (already checked out
> in `src/aws-robomaker-small-warehouse-world`). It is `ament_cmake`, ships the
> world, models and a pre-built map.

## 2. Run the full 3-robot autonomous demo

```bash
ros2 launch warehouse_bot_package bringup.launch.py
```

This starts, in order:
1. Gazebo Classic + the warehouse world,
2. `robot1/2/3` spawned in the open aisle (x≈2, y = -1 / -2.5 / -4),
3. one Nav2 stack per robot (localised on the pre-built map, auto initial pose),
4. RViz bound to `robot1`.

Send a goal from RViz with **“2D Goal Pose”** (it publishes to
`/robot1/goal_pose`), or command the whole fleet at once:

```bash
ros2 run warehouse_bot_package fleet_send_goals.py
```

Watch a different robot in its own RViz:

```bash
ros2 launch warehouse_bot_package rviz.launch.py namespace:=robot2
```

### Just the simulation (no Nav2), e.g. to drive manually
```bash
ros2 launch warehouse_bot_package warehouse_multi_spawn.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot1/cmd_vel
```

---

## 3. Layout

```
urdf/warehouse_bot.urdf.xacro     parameterized robot (namespace arg), white/black,
                                  2D lidar, diff-drive + joint-state plugins
launch/
  convoy.launch.py                leader/follower convoy (default)
  bringup.launch.py               world + 3 independent Nav2 stacks + RViz
  warehouse_multi_spawn.launch.py  world + 3 namespaced robots only
  nav2_multi.launch.py            3 namespaced Nav2 stacks
  rviz.launch.py                  per-robot RViz (namespace:=robotN)
config/fleet.yaml                 central config: roster, gains, safety, stations.
                                  Edit here and the whole stack scales.
params/
  nav2_leader.yaml                leader Nav2 params (planner used; RPP block unused)
  nav2_robot{1,2,3}.yaml          per-robot Nav2 params (DWB; independent-fleet mode)
maps/warehouse_map.yaml           pre-built occupancy map (AWS map 005, aligned to world)
rviz/multi_robot.rviz             RViz config (map, robot model, scan, costmap, path)
scripts/
  wb_common.py                    shared ROS-free helpers + config loader (unit-tested)
  leader_controller.py            drives the leader along the Nav2-planned path; stuck-recovery
  convoy_controller.py            drives followers along the leader's path; /scan slow + go-around
  convoy_mission.py               there-and-back waypoint tour for the leader (one command)
  gt_localization.py              perfect map->odom from ground truth (sim localization)
  fleet_coordinator.py            fleet bus (/fleet/status) + traffic manager (/fleet/hold)
  task_allocator.py               warehouse task dispatch: pick->drop to the nearest free robot
  submit_task.py                  CLI to submit a task on /fleet/task_request
  fleet_send_goals.py             dispatch a Nav2 goal to all robots at once
test/                             pytest unit tests (geometry, config, traffic logic)
```

### Configuration (`config/fleet.yaml`)
The roster, control gains, safety distances and warehouse stations live in one
file. Every node and launch file reads it, so to add a 4th robot you add one
roster line (plus a `params/nav2_robot4.yaml`) - spawn, Nav2, convoy, coordinator
and allocator all follow. Override the file at runtime with
`WAREHOUSE_FLEET_CONFIG=/path/to/fleet.yaml`.

### Topics / TF per robot
`/robotN/cmd_vel`, `/robotN/odom`, `/robotN/scan`, `/robotN/robot_description`,
and an isolated TF tree on `/robotN/tf` (frames `odom -> base_link -> base_scan`,
with `map -> odom` from that robot's AMCL). Frame names are **not** prefixed;
isolation comes from the namespaced tf topic - the proven Nav2 multi-robot
pattern.

---

## 4. Sensor: 2D LiDAR (and why)

The robot already had a `lidar_link`, the AWS map is a 2D occupancy grid, and
Nav2/AMCL localise against exactly that, so a 2D LiDAR is a good fit.
A `libgazebo_ros_ray_sensor` was added on a clean horizontal `base_scan` frame:
360 samples, 0.16–12 m, 10 Hz, publishing `/robotN/scan`.

If you later want richer perception, the same mount can carry a depth camera
(`libgazebo_ros_camera` / depth) for obstacle layers, but 2D LiDAR alone is
sufficient for warehouse navigation on this map.

---

## 5. Inter-robot communication, traffic management & task allocation

Each robot is namespaced and isolated, so cooperation needs a channel **outside**
the namespaces: a **shared “fleet bus” of global topics** plus central nodes.

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `/fleet/status` | coordinator -> all | consolidated `{robot: {x,y,yaw}}` snapshot (JSON, map frame) |
| `/fleet/alerts` | coordinator -> all | proximity-conflict events (JSON) |
| `/fleet/hold` | coordinator -> all | JSON list of robots told to pause (traffic management) |
| `/fleet/task_request` | operator -> allocator | `{"pickup": "...", "dropoff": "..."}` |
| `/fleet/tasks` | allocator -> all | queue + active assignments (JSON) |
| `/robotN/goal_pose`, `/robotN/navigate_to_pose` | commander -> robot | goal dispatch |

**Fleet state + traffic management** - `fleet_coordinator.py` subscribes to every
robot's **map-frame** `/robotN/ground_truth` (not `/odom`, whose origin is each
robot's own spawn point, so cross-robot distances there are meaningless),
republishes the snapshot on `/fleet/status`, and acts as a **traffic manager**:
when two robots get too close it holds the lower-priority one (roster order) on
`/fleet/hold` until they separate, with warn/clear hysteresis so the hold doesn't
flap. The convoy followers and the task allocator both obey `/fleet/hold`.

**Task allocation** - `task_allocator.py` is the warehouse application layer: it
takes `pickup -> dropoff` tasks, assigns each to the **nearest idle, un-held**
robot, and drives it pickup -> dropoff via that robot's Nav2 `navigate_to_pose`,
freeing the robot when done. Stations are defined in `config/fleet.yaml`.

```bash
# Full fleet (bringup.launch.py starts the coordinator + allocator for you):
ros2 launch warehouse_bot_package bringup.launch.py
ros2 run warehouse_bot_package submit_task.py pick_a drop_x   # dispatch a task
ros2 run warehouse_bot_package submit_task.py --list          # list stations
ros2 topic echo /fleet/tasks                                  # watch the queue
```

**Deliberate future extensions:** a `warehouse_bot_msgs` package (typed messages
instead of JSON-in-String once the schema stabilises), services/actions for
acknowledged one-to-one coordination (e.g. “reserve aisle A”), and aisle/segment
reservation in the traffic manager for true deadlock-free routing.

---

## 6. Tests & CI

ROS-free logic (geometry, arc-length interpolation, trail smoothing, the
traffic-manager decision, nearest-robot assignment) is factored into
`scripts/wb_common.py` and unit-tested under `test/`:

```bash
# fast, no ROS needed:
pytest test
# or as part of the colcon build:
colcon test --packages-select warehouse_bot_package
```

`.github/workflows/ci.yml` runs two jobs on every push/PR: a quick lint
(`flake8`) + `pytest` job, and a full `colcon build` + `colcon test` inside the
`ros:humble` image.

---

## 7. Robot model fixes (from the raw SolidWorks export)

The exported URDF needed several corrections to drive correctly in Gazebo; all
are in `warehouse_bot.urdf.xacro` with inline comments:

- **Wheels** -> clean black **cylinders, mounted outboard at y=±0.095 m** (clear
  of the 0.08 m body edge), replacing the SolidWorks wheel meshes that sat partly
  *inside* the body and rolled badly in ODE. Wheel diameter 0.042 -> **0.08 m** to
  match reality (the old value ~halved odometry); track widened, separation 0.19 m.
- **Wheel joints** rebuilt symmetric (the export gave the two wheels different
  tilted orientations, so they didn’t roll).
- **Chassis collision** -> a lifted box (the STL underside high-centred the robot).
- **Mass** 9.407 -> **2.5 kg** (the export mass was a density artifact).
- **COM** over the wheel axle + low-friction **front/rear skids**, so the robot
  rests level (≈0–2° pitch, laser horizontal), keeps its drive wheels loaded, and
  drives straight + turns in place.
- **Ground-truth pose** plugin (`p3d`) added for formation control / sim localisation.

Verified headless: straight ≈0.66 m / 5 s, in-place rotation ≈0.57 rad/s, wheels
outboard, level stance, `/scan` sees walls; the convoy leader navigates to
sequential goals while both followers trace its path at ~0.9 m spacing.
