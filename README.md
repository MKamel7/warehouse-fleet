# warehouse_bot_package

Three identical `warehouse_bot` robots (white body, black wheels) in the AWS
RoboMaker small-warehouse world, on **ROS 2 Humble + Gazebo Classic 11 + Nav2**.

Two demos:

* **Convoy (recommended):** `robot1` is the LEADER and navigates autonomously
  with Nav2; `robot2` and `robot3` FOLLOW it, tracing the leader's exact path at
  a fixed, collision-safe spacing.  ->  `convoy.launch.py`
* **Independent fleet:** all three navigate on their own namespaced Nav2 stacks,
  plus a fleet coordinator that shows inter-robot communication.
  ->  `bringup.launch.py`

---

## 0. Convoy demo (leader + 2 followers)  ⭐

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select \
  aws_robomaker_small_warehouse_world warehouse_bot_package
source install/setup.bash
ros2 launch warehouse_bot_package convoy.launch.py
```
Wait ~20 s (Gazebo + RViz open), then give the LEADER a goal:
* RViz **"2D Goal Pose"** (publishes `/robot1/goal_pose`), or
* `ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: -1.5, y: -1.0}}}}"`

The leader plans + drives there with Regulated Pure Pursuit; the two followers
trail it in a line at ~0.9 m spacing along the leader's path. Send another goal
and the whole convoy moves again. Headless: add `gui:=false rviz:=false`.

**How the convoy works**
- **Leader** — Nav2's real **planner** computes the global path on the warehouse
  map (`compute_path_to_pose`), and `leader_controller.py` drives along it with a
  smooth **pure-pursuit** law. (Pure pursuit is used instead of the stock
  controller because RPP/rotation-shim oscillates at the 180° turn-around on this
  diff-drive robot.) Localisation is a ground-truth `map->odom`
  (`gt_localization.py`, standing in for a fleet-localization source so the leader
  never gets lost).
- **Followers** — `convoy_controller.py` records the leader's travelled path as a
  breadcrumb trail and steers each follower along it with pure pursuit +
  feed-forward speed, holding its gap. They always head to a valid target (trail
  point, or a slot behind the leader on short moves) so they never freeze, and
  they re-acquire the path after yielding.
- **Collision avoidance** — every robot knows every other robot's pose, so each
  controller slows/stops before driving into another robot. No crashes even when
  paths cross or robots bunch up.
- **Yielding** — if you send the leader *back through* the convoy, the followers
  in its path pull aside to let it pass, then re-form behind it.
- The leader path is on `/robot1/plan`; the convoy trail on `/convoy/leader_path`.

Verified: leader drives to sequential goals (no oscillation); followers trail it
in a line at ~0.9 m spacing; sending the leader back through the convoy makes the
followers yield and no robots collide.

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
  convoy.launch.py                LEADER-FOLLOWER convoy (recommended)
  bringup.launch.py               world + 3 independent Nav2 stacks + RViz
  warehouse_multi_spawn.launch.py  world + 3 namespaced robots only
  nav2_multi.launch.py            3 namespaced Nav2 stacks
  rviz.launch.py                  per-robot RViz (namespace:=robotN)
params/
  nav2_leader.yaml                leader Nav2: Regulated Pure Pursuit controller
  nav2_robot{1,2,3}.yaml          per-robot Nav2 params (DWB; independent-fleet mode)
maps/warehouse_map.yaml           pre-built occupancy map (AWS map 005, aligned to world)
rviz/multi_robot.rviz             RViz config (map, robot model, scan, costmap, path)
scripts/
  leader_controller.py            drives the leader along the Nav2-planned path (pure pursuit)
  convoy_controller.py            drives followers along the leader's path (pure pursuit)
  gt_localization.py              perfect map->odom from ground truth (sim localization)
  fleet_coordinator.py            inter-robot comms: shared /fleet bus + conflict alerts
  fleet_send_goals.py             dispatch a Nav2 goal to all robots at once
```

### Topics / TF per robot
`/robotN/cmd_vel`, `/robotN/odom`, `/robotN/scan`, `/robotN/robot_description`,
and an isolated TF tree on `/robotN/tf` (frames `odom → base_link → base_scan`,
with `map → odom` from that robot's AMCL). Frame names are **not** prefixed;
isolation comes from the namespaced tf topic — the proven Nav2 multi-robot
pattern.

---

## 4. Sensor: 2D LiDAR (and why)

The robot already had a `lidar_link`, the AWS map is a 2D occupancy grid, and
Nav2/AMCL localise against exactly that — so a **2D LiDAR is the natural fit**.
A `libgazebo_ros_ray_sensor` was added on a clean horizontal `base_scan` frame:
360 samples, 0.16–12 m, 10 Hz, publishing `/robotN/scan`.

If you later want richer perception, the same mount can carry a depth camera
(`libgazebo_ros_camera` / depth) for obstacle layers, but 2D LiDAR alone is
sufficient for warehouse navigation on this map.

---

## 5. Inter-robot communication

Each robot is namespaced and isolated, so cooperation needs a channel **outside**
the namespaces. The approach used here is a **shared “fleet bus” of global
topics** plus a central coordinator:

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `/fleet/status` | coordinator → all | consolidated `{robot: {x,y,yaw}}` snapshot of the whole fleet (JSON) |
| `/fleet/alerts` | coordinator → all | conflict/coordination events (e.g. two robots too close) |
| `/robotN/goal_pose`, `/robotN/navigate_to_pose` | commander → robot | fleet-level goal dispatch |

```bash
ros2 run warehouse_bot_package fleet_coordinator.py   # gathers state, raises proximity alerts
ros2 topic echo /fleet/status
```

`fleet_coordinator.py` subscribes to every `/robotN/odom`, republishes the fleet
snapshot on `/fleet/status`, and flags pairs of robots that come within 0.6 m on
`/fleet/alerts` — the hook where a real **traffic manager / task allocator**
would live.

**Other valid options, by need:**
- **Services / actions** for request–response coordination (e.g. “reserve aisle
  A”, “claim pick task 7”) when you need acknowledged, one-to-one decisions.
- **A dedicated coordinator node** (extend `fleet_coordinator.py`) for central
  task allocation and traffic management — recommended as the fleet grows.
- **Custom messages** (a `warehouse_bot_msgs` package) instead of JSON-in-String
  once the schema stabilises, for type safety.

For this 3-robot demo the shared-topic bus + coordinator is the simplest thing
that works and extends cleanly.

---

## 6. Robot model fixes (from the raw SolidWorks export)

The exported URDF needed several corrections to drive correctly in Gazebo; all
are in `warehouse_bot.urdf.xacro` with inline comments:

- **Wheels** → clean black **cylinders, mounted outboard at y=±0.095 m** (clear
  of the 0.08 m body edge), replacing the SolidWorks wheel meshes that sat partly
  *inside* the body and rolled badly in ODE. Wheel diameter 0.042 → **0.08 m** to
  match reality (the old value ~halved odometry); track widened, separation 0.19 m.
- **Wheel joints** rebuilt symmetric (the export gave the two wheels different
  tilted orientations, so they didn’t roll).
- **Chassis collision** → a lifted box (the STL underside high-centred the robot).
- **Mass** 9.407 → **2.5 kg** (the export mass was a density artifact).
- **COM** over the wheel axle + low-friction **front/rear skids**, so the robot
  rests level (≈0–2° pitch, laser horizontal), keeps its drive wheels loaded, and
  drives straight + turns in place.
- **Ground-truth pose** plugin (`p3d`) added for formation control / sim localisation.

Verified headless: straight ≈0.66 m / 5 s, in-place rotation ≈0.57 rad/s, wheels
outboard, level stance, `/scan` sees walls; the convoy leader navigates to
sequential goals while both followers trace its path at ~0.9 m spacing.
