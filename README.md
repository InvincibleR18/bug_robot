# bug_robot — Tangent Bug navigation for the Fastbot

Autonomous point-to-point navigation for a differential-drive robot, after the **Tangent Bug**
algorithm (Kamon, Rimon & Rivlin, *IJRR* 1998).

**No map. No SLAM. No Nav2.** The robot knows three things: where it is, where the goal is, and
what the laser sees this instant. It has never seen the room and builds no model of it. Every
obstacle is discovered the moment a beam hits one. That is the point of bug algorithms — they
answer *"what if there is no map?"*, where a planner like Nav2 requires one built in advance.

ROS 2 Humble · C++ · Gazebo · package name `tangent_bug`.

## Interface

| Direction | Topic | Type |
|---|---|---|
| in | `/fastbot/scan` | `sensor_msgs/msg/LaserScan` |
| in | `/fastbot/odom` | `nav_msgs/msg/Odometry` |
| in | `/goal_pose` | `geometry_msgs/msg/PoseStamped` |
| out | `/fastbot/cmd_vel` | `geometry_msgs/msg/Twist` |
| out | `/tangent_bug/markers` | `visualization_msgs/msg/MarkerArray` |

One node, one 20 Hz timer, three subscribers, two publishers. No services, no actions, no
threads.

## Build and run

Clone into a workspace `src/`, then:

```bash
cd ~/ros2_ws
colcon build --packages-select tangent_bug
source install/setup.bash
ros2 run tangent_bug tangent_bug_node
```

The robot does not move on startup. It idles in `WAITING_FOR_GOAL` until a goal arrives.

```bash
rviz2 -d $(ros2 pkg prefix tangent_bug)/share/tangent_bug/rviz/tangent_bug.rviz
```

Then click **2D Goal Pose** and pick a point. RViz publishes a `PoseStamped` on `/goal_pose`;
the node reads the x/y out of it and starts. A new goal can be sent at any time and resets the
run.

Without RViz, the same thing from a terminal:

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'fastbot_odom'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}"
```

Goal coordinates are in the odom frame, whose origin is wherever the robot was when the
simulation started. Orientation is ignored — only position matters.

## How it works

Five states. Two of them are behaviours.

### `WAITING_FOR_GOAL`

Zero velocity, markers still published. Leaves on the first `/goal_pose` message.

### `MOTION_TO_GOAL`

If the straight line to the goal is clear, drive at the goal.

If it is blocked, scan for **discontinuities** — adjacent laser beams whose ranges jump sharply.
A jump is the *corner of an obstacle*, and corners are the only places worth steering toward,
because getting around an obstacle means passing one. Score every corner:

```
h(corner) = dist(robot, corner) + dist(corner, goal)
```

Drive at the lowest-scoring one, and keep going while that score shrinks. When it starts
**growing**, greedy has failed — the obstacle is concave and continuing would trap the robot in
a local minimum. Hand over to boundary following.

"Blocked" is a corridor test, not an angular sector: every laser hit is projected onto the
robot→goal line, and anything landing within `kRobotHalfWidth` of it counts. A narrow angular
sector misses obstacles that sit close to the goal but far from the robot. The test stops
`kGoalTolerance` short of the goal, so a goal placed beside a wall stays reachable.

### `BOUNDARY_FOLLOWING`

Hug the obstacle at `kSafeDistance`, continuing in the direction already chosen. Leave when
**both** hold:

1. The robot is closer to the goal than it was at the hit point, by `kLeaveMargin`.
2. The straight line to the goal is now clear.

Condition 1 forces monotonic progress, which is what makes the algorithm terminate. Condition 2
stops it leaving, instantly re-detecting the same obstacle, re-entering, and oscillating on the
spot.

**This is not the paper's rule, deliberately.** Kamon et al. state the leave condition as
`d_reached < d_followed`, where `d_reached` is the minimum distance from the goal to the
currently visible boundary and `d_followed` the same over the boundary already traversed. Taken
literally it deadlocks: `d_reached` minimises over *every* visible point, including the
traversed ones, so `d_reached <= d_followed` always holds and the robot circles forever. The
implementation uses the hit-point distance instead. `d_reached` is still computed and logged,
but takes no part in the decision.

### `GOAL_REACHED` / `GOAL_UNREACHABLE`

Reached: within `kGoalTolerance`. Prints `==== GOAL REACHED at (x, y) ====` and holds zero
velocity.

Unreachable, by either of two routes:

- **A full lap** — travelled more than `kLoopMinPath` and returned within `kLoopRadius` of the
  point where wall following began. This is the real Tangent Bug guarantee: the goal is
  enclosed.
- **Pinned** — the robot stayed inside a `kStuckRadius` circle for `kStuckCycles` (15 s).

Either way it stops and stays stopped until a new goal arrives.

## Safety layer

Every command from both behaviours passes through `sendCommand()` before publication. If
anything is within `kEmergencyRange` of the front cone, forward motion is cancelled and the
robot **reverses** at `kReverseSpeed`, after checking behind it.

The reverse matters. An earlier version only zeroed forward velocity, which meant that once the
robot was already inside the emergency range it could never drive forward again and had no way
out — it spun on the spot indefinitely. The stuck detector is the backstop for the same class of
failure: nothing can hang forever.

## Watching it

Gazebo shows the robot. It cannot show the goal or the candidate corners, because those are
numbers inside the program, not physical objects. For those, use RViz.

| Marker | Meaning |
|---|---|
| blue disc, `START` | odom origin, i.e. the robot's spawn point |
| green post, `GOAL` | current goal |
| orange line | every place the robot has been this run |
| blue dots | detected obstacle corners |
| red dot | the corner currently being driven at |
| yellow line | robot to current target |
| white text | state, `d_followed`, `d_reached` |

The shipped RViz config sets Fixed Frame to `fastbot_odom`, looks straight down, and gives the
`LaserScan` display a 30 s decay time so scans accumulate into a floorplan as the robot drives.
It also loads the `Measure` and `Publish Point` tools, which are the practical way to check a
gap width before tuning clearances.

The terminal prints one line a second:

```
BOUNDARY_FOLLOWING robot (  0.20, -0.64) yaw  -14 deg | goal (  0.53, -1.13) dist  0.59 | corners  0 | d_followed  1.54 | path clear
```

`path BLOCKED`/`clear` is the deciding term in the leave test, so it is the first thing to read
when the robot will not commit to a goal.

## Constants

Every threshold lives in one commented block at the top of
[`src/tangent_bug_node.cpp`](src/tangent_bug_node.cpp), derived from the measured setup rather
than guessed.

Measured from the running simulation:

| Property | Value |
|---|---|
| Room | 2.0 m × 3.0 m |
| Robot body | 0.25 m long × 0.12 m wide |
| Lidar mounting | `[0, 0, 0.108]` from `fastbot_base_link` — horizontally centred |
| Laser coverage | 360° (`angle_min` −3.1416, `angle_max` 3.1241) |
| Laser beams | 270, i.e. 1.33° apart |
| Laser range | 0.01 – 20.0 m, 25 Hz |
| Frames | `fastbot_odom`, `fastbot_base_link`, `fastbot_lidar` |
| Narrowest corridor | 0.50 m |

Because the lidar is horizontally centred, every threshold is a **radius from the middle of the
robot**. The binding constraint is therefore not the robot's width but its **swept radius** —
the circle it carves rotating on the spot, since the controller turns in place whenever heading
error exceeds `kHeadingGate`:

```
sqrt(0.125² + 0.06²) = 0.139 m
```

That gives an ordered ladder, smallest to largest:

| Value | Constant | Role |
|---|---|---|
| 0.139 | — | swept radius: physical collision |
| 0.17 | `kEmergencyRange` | hard stop and reverse, front cone |
| 0.18 | `kRobotHalfWidth` | minimum passable half-channel |
| 0.20 | `kCornerTrigger` | wall follower spins away from an inside corner |
| 0.22 | `kSafeDistance` | wall-following setpoint |

`kRobotHalfWidth` must stay under 0.25 or a 0.50 m corridor reads as solid wall. `kSafeDistance`
is a *setpoint*, not a limit, so it sits above `kRobotHalfWidth` to absorb control overshoot —
and deliberately not at 0.25, which would be dead centre of a 0.50 m corridor, where both walls
are equidistant and the closest-obstacle search flips between them every cycle.

`kDiscontinuityJump` is 0.20 m. On a wall at range `r`, neighbouring beams differ by roughly
`r · 0.0233 · tan(incidence)`, so at 3 m a 0.20 m jump needs a 71° grazing angle — real gaps are
caught, ordinary walls are not. `kMaxCornerRange` caps corner detection at 4 m, just past the
room's 3.6 m diagonal, because 270 beams over 360° put neighbours 0.35 m apart at 15 m and every
angled far-field wall would otherwise fabricate a corner.

### Tuning

| Symptom | Change |
|---|---|
| Oscillates along a wall | lower `kWallGain` or `kMaxAngular` |
| Clips obstacle corners | raise `kRobotHalfWidth` |
| Refuses to enter a gap it fits through | lower `kRobotHalfWidth` (floor: 0.14) |
| Flip-flops between behaviours | raise `kProgressEpsilon` and `kLeaveMargin` |
| Never detects corners | lower `kDiscontinuityJump` |
| Invents corners in open space | lower `kMaxCornerRange` |
| Touches walls before reacting | raise `kEmergencyRange` |
| Thrashes on the spot | widen the gap between `kEmergencyRange` and `kCornerTrigger` |

Bearings are always derived from the scan's own `angle_min` and `angle_increment`, never from
beam indices, so the node works with any laser field of view without code changes.

## Layout

```
src/tangent_bug_node.cpp   the whole implementation, one node
rviz/tangent_bug.rviz      RViz config: frame, markers, decay, measuring tools
CMakeLists.txt
package.xml
```

## Licence

MIT.
