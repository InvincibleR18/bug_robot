// Tangent Bug navigation for the Fastbot.
//
// Sensor-based planner: no map, no SLAM, no Nav2. The robot knows only where it
// is (/fastbot/odom), where the goal is, and what the laser sees right now
// (/fastbot/scan). Obstacles are discovered the instant a beam hits one.
//
// Two behaviours, plus an idle state and two terminal ones:
//   WAITING_FOR_GOAL    stopped until something publishes on /goal_pose
//   MOTION_TO_GOAL      drive at the goal, or at the obstacle corner that
//                       minimises dist(robot, corner) + dist(corner, goal)
//   BOUNDARY_FOLLOWING  hug the obstacle until the robot is closer to the goal
//                       than it was when it hit, AND the way there is clear
//   GOAL_REACHED / GOAL_UNREACHABLE
//
// After Kamon, Rimon & Rivlin, "TangentBug: A Range-Sensor-Based Navigation
// Algorithm", IJRR 1998. The leave condition here is the practical
// hit-point-distance form rather than the paper's d_reached < d_followed; see
// boundaryFollow() for why the literal version deadlocks.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {

// Values below are derived from the measured setup, not guessed:
//
//   room            2.0 m x 3.0 m
//   robot body      0.25 m long x 0.12 m wide
//   lidar           at the robot centre (tf fastbot_base_link -> fastbot_lidar
//                   is [0, 0, 0.108]), so every threshold here is a radius
//                   measured from the middle of the robot
//   swept radius    sqrt(0.125^2 + 0.06^2) = 0.14 m, the circle the robot
//                   carves when it turns on the spot. This, not the width, is
//                   the hard floor for every clearance value
//   narrowest gap   0.50 m, so walls sit at +/-0.25 m from its centreline
//   lidar           270 beams over 360 deg = 1.33 deg apart, 0.01 - 20 m

// ---- Goal ------------------------------------------------------------------
// There is no default goal: the node idles in WAITING_FOR_GOAL until something
// publishes on /goal_pose (RViz "2D Goal Pose" button).
constexpr double kGoalTolerance = 0.15; // m, close enough to call it arrived

// ---- Robot geometry --------------------------------------------------------
// Must exceed the 0.14 m swept radius and stay under 0.25 m or the 0.50 m
// corridor reads as a solid wall. 0.18 needs a 0.36 m channel.
constexpr double kRobotHalfWidth = 0.18; // m, swept radius + margin

// ---- Obstacle sensing ------------------------------------------------------
// Jump threshold: on a wall at range r, neighbouring beams differ by about
// r * 0.0233 * tan(incidence). At 3 m a 0.20 m jump needs a 71 deg grazing
// angle, so real gaps are caught and ordinary walls are not.
constexpr double kDiscontinuityJump = 0.20; // m, range jump that marks a corner
constexpr double kMaxCornerRange = 4.00;    // m, just over the 3.6 m room diagonal
// Setpoint, not a limit: it sits above kRobotHalfWidth (0.18) so ordinary
// control overshoot does not become a scrape. Deliberately NOT 0.25, which
// would be dead centre of a 0.50 m corridor - with both walls equidistant,
// closestObstacle() flips between them every cycle and the target bearing
// swings 180 deg. 0.22 commits the robot to one wall.
constexpr double kSafeDistance = 0.22;      // m, standoff held while wall following
constexpr double kFrontSector = 0.44;       // rad (~25 deg); at 25 deg the wall of a
                                            // 0.50 m corridor reads 0.59 m, well clear
                                            // of kCornerTrigger, so travelling down a
                                            // corridor does not read as a dead end
constexpr double kCornerTrigger = 0.20;     // m, wall follower spins away inside a corner
constexpr double kEmergencyRange = 0.17;    // m, stop and reverse out. Must sit clearly
                                            // below kCornerTrigger or the two fight and
                                            // the robot thrashes on the spot
constexpr double kEmergencySector = 0.44;   // rad (~25 deg), cone the hard stop watches

// ---- Velocity limits -------------------------------------------------------
constexpr double kMaxLinear = 0.15;   // m/s, slower suits a 2x3 m room
constexpr double kReverseSpeed = 0.07; // m/s, used only to escape a too-close wall
constexpr double kMaxAngular = 1.00;  // rad/s
constexpr double kHeadingGate = 0.35; // rad, above this error turn on the spot
constexpr double kLinGain = 0.60;
constexpr double kAngGain = 1.20;
constexpr double kWallGain = 1.00;    // lowered with kSafeDistance: high gain at a
                                      // short standoff oscillates

// ---- Behaviour switching ---------------------------------------------------
constexpr double kProgressEpsilon = 0.02; // m, hysteresis before declaring a local minimum
constexpr double kLeaveMargin = 0.10;     // m, must beat the hit-point distance by this
constexpr double kMinFollowBeforeLeave = 0.30; // m, travel before the leave test arms
constexpr double kLoopMinPath = 1.50;     // m, travel before the loop test arms
constexpr double kLoopRadius = 0.40;      // m, allows for odom drift over an ~8 m lap
constexpr double kStuckRadius = 0.10;     // m, movement below this counts as not moving
constexpr int kStuckCycles = 300;         // 300 * 50 ms = 15 s pinned => give up

constexpr int kControlPeriodMs = 50; // 20 Hz

constexpr double kInf = std::numeric_limits<double>::infinity();

enum class State {
  WAITING_FOR_GOAL,
  MOTION_TO_GOAL,
  BOUNDARY_FOLLOWING,
  GOAL_REACHED,
  GOAL_UNREACHABLE
};

const char *stateName(State s) {
  switch (s) {
  case State::WAITING_FOR_GOAL:
    return "WAITING_FOR_GOAL";
  case State::MOTION_TO_GOAL:
    return "MOTION_TO_GOAL";
  case State::BOUNDARY_FOLLOWING:
    return "BOUNDARY_FOLLOWING";
  case State::GOAL_REACHED:
    return "GOAL_REACHED";
  case State::GOAL_UNREACHABLE:
    return "GOAL_UNREACHABLE";
  }
  return "UNKNOWN";
}

// An obstacle corner seen in the scan: the endpoint of a range discontinuity.
struct Candidate {
  double x;       // world frame
  double y;       // world frame
  double bearing; // robot frame, rad
  double range;   // m
};

double normalizeAngle(double a) {
  while (a > M_PI)
    a -= 2.0 * M_PI;
  while (a < -M_PI)
    a += 2.0 * M_PI;
  return a;
}

} // namespace

class TangentBugNode : public rclcpp::Node {
public:
  TangentBugNode() : Node("tangent_bug_node") {
    cmd_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("fastbot/cmd_vel", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "tangent_bug/markers", 10);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "fastbot/scan", rclcpp::SensorDataQoS(),
        std::bind(&TangentBugNode::scanCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "fastbot/odom", 10,
        std::bind(&TangentBugNode::odomCallback, this, std::placeholders::_1));
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10,
        std::bind(&TangentBugNode::goalCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(kControlPeriodMs),
        std::bind(&TangentBugNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
                "Tangent Bug ready. Idle until a goal arrives - use the "
                "'2D Goal Pose' button in RViz to set one.");
  }

  // Called from main once spinning has stopped, so the robot does not coast.
  void stopRobot() {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_INFO(this->get_logger(), "Stop command sent.");
  }

private:
  // ---- Callbacks -----------------------------------------------------------

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_ = msg;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double nx = msg->pose.pose.position.x;
    const double ny = msg->pose.pose.position.y;

    // Boundary following needs the distance travelled along the wall, both to
    // arm the leave test and to detect a full loop around the obstacle.
    if (have_odom_ && state_ == State::BOUNDARY_FOLLOWING) {
      boundary_path_ += std::hypot(nx - x_, ny - y_);
    }

    // Trail point every 5 cm, capped so the marker stays cheap to draw.
    if (trail_.empty() ||
        std::hypot(nx - trail_.back().x, ny - trail_.back().y) > 0.05) {
      if (trail_.size() >= 2000) {
        trail_.erase(trail_.begin());
      }
      trail_.push_back(makePoint(nx, ny, 0.05));
    }

    x_ = nx;
    y_ = ny;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_ = yaw;

    odom_frame_ = msg->header.frame_id;
    have_odom_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    // A new goal invalidates everything the previous run learned.
    best_heuristic_ = kInf;
    d_followed_ = kInf;
    d_reached_ = kInf;
    boundary_path_ = 0.0;
    stuck_cycles_ = 0;
    stuck_ref_x_ = x_;
    stuck_ref_y_ = y_;
    trail_.clear();
    state_ = State::MOTION_TO_GOAL;
    RCLCPP_INFO(this->get_logger(),
                "==== NEW GOAL (%.2f, %.2f) in frame '%s' ====", goal_x_,
                goal_y_, msg->header.frame_id.c_str());
  }

  void controlLoop() {
    if (!scan_ || !have_odom_) {
      return; // wait for the first scan and odom
    }

    if (state_ == State::WAITING_FOR_GOAL) {
      stopHere();
      publishMarkers();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Waiting for a goal on /goal_pose. Click "
                           "'2D Goal Pose' in RViz.");
      return;
    }

    switch (state_) {
    case State::WAITING_FOR_GOAL:
      break;
    case State::MOTION_TO_GOAL:
      motionToGoal();
      break;
    case State::BOUNDARY_FOLLOWING:
      boundaryFollow();
      break;
    case State::GOAL_REACHED:
    case State::GOAL_UNREACHABLE:
      stopHere();
      break;
    }

    // Nothing may hang forever. If the robot has not moved out of a small
    // circle for kStuckCycles, whatever it is trying is not working.
    if (std::hypot(x_ - stuck_ref_x_, y_ - stuck_ref_y_) > kStuckRadius) {
      stuck_ref_x_ = x_;
      stuck_ref_y_ = y_;
      stuck_cycles_ = 0;
    } else if (state_ != State::GOAL_REACHED &&
               state_ != State::GOAL_UNREACHABLE &&
               ++stuck_cycles_ > kStuckCycles) {
      RCLCPP_WARN(this->get_logger(),
                  "Pinned at (%.2f, %.2f) for %.0f s - giving up.", x_, y_,
                  kStuckCycles * kControlPeriodMs / 1000.0);
      setState(State::GOAL_UNREACHABLE);
      stopHere();
    }

    // Once a second, say where everything is. Without RViz this terminal line
    // is the only way to see what the algorithm is doing.
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "%-18s robot (%6.2f,%6.2f) yaw %4.0f deg | goal (%6.2f,%6.2f) dist %5.2f"
        " | corners %2zu | d_followed %5.2f | path %s",
        stateName(state_), x_, y_, yaw_ * 180.0 / M_PI, goal_x_, goal_y_,
        std::hypot(goal_x_ - x_, goal_y_ - y_), candidates_.size(), d_followed_,
        pathToGoalBlocked() ? "BLOCKED" : "clear");

    publishMarkers();
  }

  // ---- Behaviour 1: motion to goal ----------------------------------------

  void motionToGoal() {
    const double d_goal = std::hypot(goal_x_ - x_, goal_y_ - y_);
    if (d_goal < kGoalTolerance) {
      setState(State::GOAL_REACHED);
      RCLCPP_INFO(this->get_logger(),
                  "==== GOAL REACHED at (%.2f, %.2f) ====", x_, y_);
      stopHere();
      return;
    }

    candidates_.clear();
    chosen_ = -1;

    // Straight line is clear: just drive at the goal.
    if (!pathToGoalBlocked()) {
      // Reset rather than seed from d_goal. A corner score is
      // dist(robot, corner) + dist(corner, goal), which the triangle
      // inequality makes >= d_goal, so seeding from d_goal would make the
      // first blocked cycle look like a local minimum. Progress is only
      // meaningful between corner scores within one blocked episode.
      best_heuristic_ = kInf;
      target_x_ = goal_x_;
      target_y_ = goal_y_;
      driveTo(goal_x_, goal_y_);
      return;
    }

    // Blocked. The only useful places to steer are the obstacle's corners.
    candidates_ = findDiscontinuities();
    if (candidates_.empty()) {
      // Blocked with no visible way round: hug the obstacle instead.
      enterBoundaryFollowing();
      return;
    }

    double best = kInf;
    int best_index = -1;
    for (size_t i = 0; i < candidates_.size(); ++i) {
      const Candidate &c = candidates_[i];
      const double h = std::hypot(c.x - x_, c.y - y_) +
                       std::hypot(goal_x_ - c.x, goal_y_ - c.y);
      if (h < best) {
        best = h;
        best_index = static_cast<int>(i);
      }
    }

    // The heuristic got worse: greedy has bottomed out in a local minimum,
    // which means the obstacle is concave. Switch to boundary following.
    if (best > best_heuristic_ + kProgressEpsilon) {
      // Chosen corner on the left means we pass on the left, which leaves the
      // obstacle on our right.
      side_ = (candidates_[best_index].bearing >= 0.0) ? +1 : -1;
      enterBoundaryFollowing();
      return;
    }

    best_heuristic_ = std::min(best_heuristic_, best);
    chosen_ = best_index;
    target_x_ = candidates_[best_index].x;
    target_y_ = candidates_[best_index].y;
    driveTo(target_x_, target_y_);
  }

  // ---- Behaviour 2: boundary following ------------------------------------

  void enterBoundaryFollowing() {
    setState(State::BOUNDARY_FOLLOWING);
    hit_x_ = x_;
    hit_y_ = y_;
    boundary_path_ = 0.0;

    // How far the goal was when this obstacle was hit. The robot must beat this
    // before it is allowed to leave, which is what forces monotonic progress
    // and guarantees the algorithm terminates.
    d_followed_ = std::hypot(goal_x_ - x_, goal_y_ - y_);
  }

  void boundaryFollow() {
    if (std::hypot(goal_x_ - x_, goal_y_ - y_) < kGoalTolerance) {
      setState(State::GOAL_REACHED);
      stopHere();
      return;
    }

    double range, bearing;
    if (!closestObstacle(&range, &bearing)) {
      // Wall out of range: nothing left to follow.
      best_heuristic_ = kInf;
      setState(State::MOTION_TO_GOAL);
      return;
    }

    // Leave test. Two conditions, both required:
    //
    //   1. The robot is strictly closer to the goal than it was at the hit
    //      point. This forces progress and makes the algorithm terminate.
    //   2. The straight line to the goal is clear. Without this the robot
    //      leaves, immediately re-detects the same obstacle, re-enters, and
    //      oscillates on the spot instead of going anywhere.
    //
    // The paper states this as d_reached < d_followed, where d_reached is the
    // minimum distance from the goal to the currently visible boundary and
    // d_followed the same over the boundary already traversed. Taken literally
    // that deadlocks: d_reached minimises over every visible point INCLUDING
    // the traversed ones, so d_reached <= d_followed always holds and the robot
    // never leaves the wall. d_reached_ is still computed, but only to report.
    d_reached_ = closestVisibleWallToGoal();
    const double d_now = std::hypot(goal_x_ - x_, goal_y_ - y_);
    if (boundary_path_ > kMinFollowBeforeLeave &&
        d_now < d_followed_ - kLeaveMargin && !pathToGoalBlocked()) {
      best_heuristic_ = kInf;
      setState(State::MOTION_TO_GOAL);
      return;
    }

    // Back at the hit point after a full lap: the goal cannot be reached.
    if (boundary_path_ > kLoopMinPath &&
        std::hypot(x_ - hit_x_, y_ - hit_y_) < kLoopRadius) {
      setState(State::GOAL_UNREACHABLE);
      stopHere();
      return;
    }

    geometry_msgs::msg::Twist cmd;
    const double front = minRangeInSector(0.0, kFrontSector);

    if (front < kCornerTrigger) {
      // Inside corner: rotate away from the wall without moving forward.
      cmd.linear.x = 0.0;
      cmd.angular.z = side_ * kMaxAngular;
    } else {
      // Travel along the wall (bearing +/- 90 deg), corrected to hold the
      // standoff: too far pushes the heading back toward the wall.
      const double err = std::clamp(range - kSafeDistance, -0.3, 0.3);
      const double desired = normalizeAngle(bearing + side_ * M_PI_2 -
                                            side_ * kWallGain * err);
      cmd.angular.z = std::clamp(kAngGain * desired, -kMaxAngular, kMaxAngular);
      // Taper speed with heading error rather than cutting it to zero. The
      // binary gate made the robot stop and pirouette every time the wall
      // curved, which is the yaw thrashing seen in the logs.
      cmd.linear.x = kMaxLinear * std::max(0.0, std::cos(desired));
    }
    sendCommand(cmd);

    candidates_.clear();
    chosen_ = -1;
    target_x_ = goal_x_;
    target_y_ = goal_y_;
  }

  // ---- Scan helpers --------------------------------------------------------

  // Sanitised reading: NaN, inf and out-of-limit values become range_max, which
  // callers treat as "nothing hit".
  double rangeAt(size_t i) const {
    const double r = scan_->ranges[i];
    if (!std::isfinite(r) || r < scan_->range_min || r > scan_->range_max) {
      return scan_->range_max;
    }
    return r;
  }

  // Never assume index 0 is the front, or that the laser covers 360 degrees.
  double bearingOf(size_t i) const {
    return scan_->angle_min + static_cast<double>(i) * scan_->angle_increment;
  }

  void toWorld(double range, double bearing, double *wx, double *wy) const {
    *wx = x_ + range * std::cos(yaw_ + bearing);
    *wy = y_ + range * std::sin(yaw_ + bearing);
  }

  bool closestObstacle(double *range, double *bearing) const {
    double best = kInf;
    int best_index = -1;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double r = rangeAt(i);
      if (r >= scan_->range_max) {
        continue;
      }
      if (r < best) {
        best = r;
        best_index = static_cast<int>(i);
      }
    }
    if (best_index < 0) {
      return false;
    }
    *range = best;
    *bearing = bearingOf(static_cast<size_t>(best_index));
    return true;
  }

  double minRangeInSector(double center, double half_width) const {
    double best = kInf;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      if (std::fabs(normalizeAngle(bearingOf(i) - center)) > half_width) {
        continue;
      }
      best = std::min(best, rangeAt(i));
    }
    return best;
  }

  double closestVisibleWallToGoal() const {
    double best = kInf;
    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double r = rangeAt(i);
      if (r >= scan_->range_max) {
        continue;
      }
      double wx, wy;
      toWorld(r, bearingOf(i), &wx, &wy);
      best = std::min(best, std::hypot(goal_x_ - wx, goal_y_ - wy));
    }
    return best;
  }

  // A sharp jump between neighbouring beams is the edge of an obstacle. The
  // nearer of the two readings is the corner itself; the farther one is
  // background seen past it.
  std::vector<Candidate> findDiscontinuities() const {
    std::vector<Candidate> out;
    const size_t n = scan_->ranges.size();
    const double sweep = (scan_->angle_increment >= 0.0) ? +1.0 : -1.0;

    for (size_t i = 0; i + 1 < n; ++i) {
      const double r0 = rangeAt(i);
      const double r1 = rangeAt(i + 1);
      if (std::fabs(r1 - r0) < kDiscontinuityJump) {
        continue;
      }

      const bool near_is_first = r0 < r1;
      const size_t index = near_is_first ? i : i + 1;
      const double r = near_is_first ? r0 : r1;
      if (r >= scan_->range_max) {
        continue; // nothing actually hit
      }
      // The Fastbot lidar is 270 beams over 360 deg, so neighbouring beams land
      // 0.35 m apart at 15 m. Out there any wall seen at an angle produces jumps
      // bigger than kDiscontinuityJump and would fake a corner. Distant corners
      // are not actionable anyway.
      if (r > kMaxCornerRange) {
        continue;
      }

      // Aim just past the corner rather than clipping it: nudge the bearing
      // toward the open side of the gap by the robot's half width.
      const double margin = std::atan2(kRobotHalfWidth * 2.0, std::max(r, 0.05));
      const double direction = (near_is_first ? +1.0 : -1.0) * sweep;

      Candidate c;
      c.bearing = normalizeAngle(bearingOf(index) + direction * margin);
      c.range = r;
      toWorld(c.range, c.bearing, &c.x, &c.y);
      out.push_back(c);
    }
    return out;
  }

  // Corridor test rather than a fixed angular sector: a narrow sector misses
  // obstacles that sit near the goal but far from the robot.
  bool pathToGoalBlocked() const {
    const double gx = goal_x_ - x_;
    const double gy = goal_y_ - y_;
    const double goal_distance = std::hypot(gx, gy);
    if (goal_distance < 1e-6) {
      return false;
    }
    const double ux = gx / goal_distance;
    const double uy = gy / goal_distance;

    // The robot only has to get within kGoalTolerance, so stop the test that
    // far short of the goal. This keeps a goal placed beside a wall reachable
    // WITHOUT ignoring walls standing between the robot and the goal - an
    // earlier version cleared a radius around the goal and did exactly that,
    // reporting "path clear" with a wall 0.11 m dead ahead.
    const double stop_at = goal_distance - kGoalTolerance;

    for (size_t i = 0; i < scan_->ranges.size(); ++i) {
      const double r = rangeAt(i);
      if (r >= scan_->range_max) {
        continue;
      }
      double wx, wy;
      toWorld(r, bearingOf(i), &wx, &wy);
      const double dx = wx - x_;
      const double dy = wy - y_;

      const double along = dx * ux + dy * uy;
      if (along <= 0.0 || along >= stop_at) {
        continue; // behind the robot, or level with / beyond the goal
      }
      if (std::fabs(-dx * uy + dy * ux) < kRobotHalfWidth) {
        return true;
      }
    }
    return false;
  }

  // ---- Motion --------------------------------------------------------------

  void driveTo(double tx, double ty) {
    const double error = normalizeAngle(std::atan2(ty - y_, tx - x_) - yaw_);

    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = std::clamp(kAngGain * error, -kMaxAngular, kMaxAngular);
    if (std::fabs(error) > kHeadingGate) {
      cmd.linear.x = 0.0; // turn on the spot before committing
    } else {
      cmd.linear.x =
          std::min(kMaxLinear, kLinGain * std::hypot(tx - x_, ty - y_));
    }
    sendCommand(cmd);
  }

  void stopHere() { cmd_pub_->publish(geometry_msgs::msg::Twist()); }

  // Last line of defence. Whatever a behaviour decided, never drive forward
  // into something already inside kEmergencyRange.
  void sendCommand(geometry_msgs::msg::Twist cmd) {
    const double front = minRangeInSector(0.0, kEmergencySector);
    if (front < kEmergencyRange) {
      // Zeroing forward speed is not enough. Once the robot is already inside
      // this range it can never drive forward again, so without a reverse it
      // spins on the spot forever. Back off until there is room, checking
      // behind first.
      const double rear = minRangeInSector(M_PI, kEmergencySector);
      cmd.linear.x = (rear > kEmergencyRange * 2.0) ? -kReverseSpeed : 0.0;
      if (std::fabs(cmd.angular.z) < 0.3) {
        cmd.angular.z = (side_ >= 0) ? kMaxAngular : -kMaxAngular;
      }
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Too close: %.2f m ahead, %.2f m behind, backing %s",
                           front, rear, cmd.linear.x < 0.0 ? "off" : "blocked");
    }
    cmd_pub_->publish(cmd);
  }

  void setState(State next) {
    if (next == state_) {
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "%s -> %s  (d_goal=%.2f d_followed=%.2f d_reached=%.2f)",
                stateName(state_), stateName(next),
                std::hypot(goal_x_ - x_, goal_y_ - y_), d_followed_,
                d_reached_);
    state_ = next;
  }

  // ---- Markers -------------------------------------------------------------

  visualization_msgs::msg::Marker baseMarker(int id, int32_t type) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = odom_frame_;
    m.header.stamp = this->now();
    m.ns = "tangent_bug";
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.color.a = 1.0;
    return m;
  }

  static geometry_msgs::msg::Point makePoint(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  visualization_msgs::msg::Marker labelMarker(int id, double x, double y,
                                              double z, const std::string &text,
                                              float r, float g, float b,
                                              double height = 0.14) {
    auto m = baseMarker(id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    m.pose.position = makePoint(x, y, z);
    m.scale.z = height;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.text = text;
    return m;
  }

  void publishMarkers() {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear);

    // START: the odom origin, which is where the robot spawned. Blue disc.
    auto start = baseMarker(1, visualization_msgs::msg::Marker::CYLINDER);
    start.pose.position = makePoint(0.0, 0.0, 0.02);
    start.scale.x = start.scale.y = 0.45;
    start.scale.z = 0.04;
    start.color.b = 1.0;
    start.color.g = 0.5;
    array.markers.push_back(start);
    array.markers.push_back(
        labelMarker(2, 0.0, -0.35, 0.05, "START", 0.4, 0.8, 1.0, 0.14));

    if (state_ == State::WAITING_FOR_GOAL) {
      array.markers.push_back(labelMarker(10, 0.0, -0.60, 0.05,
                                         "set a goal: 2D Goal Pose", 1.0, 0.9,
                                         0.3, 0.13));
      marker_pub_->publish(array);
      return;
    }

    // GOAL: tall green post, visible from any angle.
    auto goal = baseMarker(3, visualization_msgs::msg::Marker::CYLINDER);
    goal.pose.position = makePoint(goal_x_, goal_y_, 0.40);
    goal.scale.x = goal.scale.y = 0.18;
    goal.scale.z = 0.80;
    goal.color.g = 1.0;
    goal.color.a = 0.85;
    array.markers.push_back(goal);
    array.markers.push_back(
        labelMarker(4, goal_x_, goal_y_ + 0.35, 0.05, "GOAL", 0.2, 1.0, 0.2,
                    0.14));

    // Trail: everywhere the robot has actually been. This is what makes the
    // route readable at a glance.
    if (trail_.size() > 1) {
      auto trail = baseMarker(5, visualization_msgs::msg::Marker::LINE_STRIP);
      trail.scale.x = 0.05;
      trail.color.r = 1.0;
      trail.color.g = 0.55;
      trail.color.b = 0.0;
      trail.points = trail_;
      array.markers.push_back(trail);
    }

    // Every detected obstacle corner: blue.
    if (!candidates_.empty()) {
      auto corners =
          baseMarker(6, visualization_msgs::msg::Marker::SPHERE_LIST);
      corners.scale.x = corners.scale.y = corners.scale.z = 0.15;
      corners.color.b = 1.0;
      corners.color.g = 0.4;
      for (const Candidate &c : candidates_) {
        corners.points.push_back(makePoint(c.x, c.y, 0.10));
      }
      array.markers.push_back(corners);
    }

    // The corner currently chosen: red.
    if (chosen_ >= 0 && chosen_ < static_cast<int>(candidates_.size())) {
      auto pick = baseMarker(7, visualization_msgs::msg::Marker::SPHERE);
      pick.pose.position =
          makePoint(candidates_[chosen_].x, candidates_[chosen_].y, 0.12);
      pick.scale.x = pick.scale.y = pick.scale.z = 0.28;
      pick.color.r = 1.0;
      array.markers.push_back(pick);
    }

    // Robot to current target: yellow line.
    auto line = baseMarker(8, visualization_msgs::msg::Marker::LINE_STRIP);
    line.scale.x = 0.03;
    line.color.r = 1.0;
    line.color.g = 1.0;
    line.points.push_back(makePoint(x_, y_, 0.10));
    line.points.push_back(makePoint(target_x_, target_y_, 0.10));
    array.markers.push_back(line);

    // State readout, offset to the side so it does not cover the robot.
    char buffer[160];
    std::snprintf(buffer, sizeof(buffer), "%s  d_f %.2f  d_r %.2f",
                  stateName(state_), d_followed_, d_reached_);
    array.markers.push_back(
        labelMarker(9, x_ + 0.6, y_ + 0.6, 0.45, buffer, 1.0, 1.0, 1.0, 0.12));

    marker_pub_->publish(array);
  }

  // ---- Members -------------------------------------------------------------

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::LaserScan::SharedPtr scan_;
  bool have_odom_ = false;
  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
  std::string odom_frame_ = "fastbot_odom";

  double goal_x_ = 0.0;
  double goal_y_ = 0.0;

  State state_ = State::WAITING_FOR_GOAL;
  double best_heuristic_ = kInf;
  double d_followed_ = kInf;
  double d_reached_ = kInf; // reported only; not part of the leave test
  int side_ = 1; // +1 keeps the obstacle on the right

  double stuck_ref_x_ = 0.0;
  double stuck_ref_y_ = 0.0;
  int stuck_cycles_ = 0;

  double hit_x_ = 0.0;
  double hit_y_ = 0.0;
  double boundary_path_ = 0.0;

  std::vector<geometry_msgs::msg::Point> trail_;
  std::vector<Candidate> candidates_;
  int chosen_ = -1;
  double target_x_ = 0.0;
  double target_y_ = 0.0;
};

namespace {
std::atomic<bool> g_interrupted{false};
void handleSigint(int) { g_interrupted = true; } // async-signal-safe: flag only
} // namespace

int main(int argc, char **argv) {
  // Take over SIGINT so the stop command is published from the main thread with
  // a still-valid context. Publishing from inside a signal handler is not
  // async-signal-safe and segfaults on shutdown.
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handleSigint);

  auto node = std::make_shared<TangentBugNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (rclcpp::ok() && !g_interrupted) {
    executor.spin_once(std::chrono::milliseconds(kControlPeriodMs));
  }

  node->stopRobot();
  rclcpp::shutdown();
  return 0;
}
