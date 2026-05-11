"""Phase 3B MVP go-to-pose backend: a small proportional controller
closing the loop from `/odom` to `/cmd_vel`.

Explicit assumptions (all hold on the current Phase 0 sim):

  * `/odom` is published by Isaac Sim and expressed in the same global
    frame the goal uses (`odom` for Phase 0/1/2/3A).
  * `/cmd_vel` is consumed by Phase 0's CmdVelDriver which integrates
    body-frame velocity into the Go2 articulation root via
    SingleArticulation.set_world_pose. So publishing `Twist` here
    actually moves the robot.
  * There is no walking policy yet — the robot translates and yaws as
    a rigid articulation. This is the MVP contract; once a real gait
    ships, swap in a policy-aware backend via the `backend` parameter
    on `nav_executor_node` without touching the rest of the stack.

What this backend is NOT:
  * a Nav2 replacement (no planning, no costmap awareness, no recovery
    behaviours)
  * a gait / locomotion policy
  * a proper pose-tracking controller (no feed-forward, no yaw tracking
    at the goal — only "face the goal while approaching it")

State machine (also the strings published on `/navigation/status`):
  IDLE       — no goal
  ROTATING   — heading error > rotate_threshold_rad, rotate in place
  MOVING     — roughly facing the goal, drive forward with small yaw correction
  REACHED    — within stop_radius_m, zero cmd_vel, keep goal for arrival check
  CANCELED   — goal was explicitly dropped
"""

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .base_backend import NavigationBackend
from ..utils import yaw_from_quaternion


def _wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class SimplePControllerBackend(NavigationBackend):
    """Closed-loop P controller: odom -> cmd_vel."""

    IDLE = "IDLE"
    ROTATING = "ROTATING"
    MOVING = "MOVING"
    REACHED = "REACHED"
    CANCELED = "CANCELED"

    def __init__(self, node: Node) -> None:
        self._node = node

        # All controller knobs exposed as ROS 2 parameters on the owning
        # node, so a launch file or a `--ros-args -p ...` override can
        # tune them without a rebuild. Prefix keeps them namespaced so
        # they don't collide with other backend parameters.
        node.declare_parameter("controller.rotate_threshold_rad", 0.35)   # ~20°
        node.declare_parameter("controller.stop_radius_m", 0.25)
        node.declare_parameter("controller.goal_update_threshold_m", 0.15)
        node.declare_parameter("controller.max_linear", 0.40)             # m/s
        node.declare_parameter("controller.max_angular", 0.80)            # rad/s
        node.declare_parameter("controller.k_linear", 0.80)
        node.declare_parameter("controller.k_angular", 1.20)
        node.declare_parameter("controller.loop_hz", 10.0)
        # Phase 4 hook: when a separate search layer (e.g.
        # search_manager_node) is publishing on /cmd_vel while
        # nav_executor is IDLE, the default behaviour of spamming zero
        # Twist every tick would fight with it. Set this to False to
        # keep the controller completely silent in IDLE / CANCELED /
        # REACHED states. Default stays True so existing launches
        # (Phase 3B alone) keep their prior behaviour exactly.
        node.declare_parameter("controller.publish_zero_when_idle", True)

        self._rotate_thr = float(node.get_parameter("controller.rotate_threshold_rad").value)
        self._stop_r = float(node.get_parameter("controller.stop_radius_m").value)
        self._goal_upd_thr = float(node.get_parameter("controller.goal_update_threshold_m").value)
        self._max_v = float(node.get_parameter("controller.max_linear").value)
        self._max_w = float(node.get_parameter("controller.max_angular").value)
        self._k_v = float(node.get_parameter("controller.k_linear").value)
        self._k_w = float(node.get_parameter("controller.k_angular").value)
        loop_hz = float(node.get_parameter("controller.loop_hz").value)
        self._publish_zero_idle = bool(
            node.get_parameter("controller.publish_zero_when_idle").value
        )

        self._status = self.IDLE
        self._goal: Optional[PoseStamped] = None
        self._odom: Optional[Odometry] = None
        self._last_log_time = node.get_clock().now()

        self._cmd_pub = node.create_publisher(Twist, "/cmd_vel", 10)
        node.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._timer = node.create_timer(1.0 / max(loop_hz, 1.0), self._tick)

        node.get_logger().info(
            f"SimplePControllerBackend ready. "
            f"rotate_thr={self._rotate_thr:.2f}rad "
            f"stop_radius={self._stop_r:.2f}m "
            f"goal_update_threshold={self._goal_upd_thr:.2f}m "
            f"max_v={self._max_v:.2f}m/s max_w={self._max_w:.2f}rad/s "
            f"loop_hz={loop_hz:.1f}"
        )

    # ------------------------------------------------------------------
    # NavigationBackend interface
    # ------------------------------------------------------------------
    def send_goal(self, goal: PoseStamped) -> bool:
        # Hysteresis: Phase 3A's goal_planner re-selects the closest
        # approach point every 0.5 s, which means as the robot gets
        # near the chair the "best candidate" shifts slightly toward
        # the new robot position. Without hysteresis the controller
        # would follow that drift forever and never trigger REACHED.
        # A 15 cm dead-band is plenty in the current MVP scene.
        if self._goal is not None and self._status in (self.MOVING, self.ROTATING):
            old = self._goal.pose.position
            new = goal.pose.position
            if math.hypot(old.x - new.x, old.y - new.y) < self._goal_upd_thr:
                return True  # keep tracking the currently-locked goal

        self._goal = goal
        self._status = self.ROTATING
        self._node.get_logger().info(
            f"[nav-exec] NEW goal frame='{goal.header.frame_id}' "
            f"pos=({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) "
            f"yaw={yaw_from_quaternion(goal.pose.orientation):.2f}rad"
        )
        return True

    def cancel(self) -> None:
        self._goal = None
        self._status = self.CANCELED
        self._publish_stop()
        self._node.get_logger().info("[nav-exec] CANCELED (goal cleared)")

    def status(self) -> str:
        return self._status

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    def _tick(self) -> None:
        if self._goal is None:
            if self._publish_zero_idle:
                self._publish_stop()
            self._status = self.IDLE
            self._maybe_heartbeat()
            return
        if self._odom is None:
            if self._publish_zero_idle:
                self._publish_stop()
            self._maybe_heartbeat(reason="waiting for /odom")
            return

        rx = float(self._odom.pose.pose.position.x)
        ry = float(self._odom.pose.pose.position.y)
        r_yaw = yaw_from_quaternion(self._odom.pose.pose.orientation)
        gx = float(self._goal.pose.position.x)
        gy = float(self._goal.pose.position.y)
        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)
        desired_yaw = math.atan2(dy, dx)
        err_yaw = _wrap_angle(desired_yaw - r_yaw)

        twist = Twist()
        if dist < self._stop_r:
            # Keep goal in self._goal so arrival_verifier keeps seeing
            # the pose; just stop moving. Do NOT clear the goal here —
            # that would reset the state machine on the next send_goal.
            self._status = self.REACHED
        elif abs(err_yaw) > self._rotate_thr:
            self._status = self.ROTATING
            twist.angular.z = max(-self._max_w, min(self._max_w, self._k_w * err_yaw))
        else:
            self._status = self.MOVING
            twist.linear.x = max(0.0, min(self._max_v, self._k_v * dist))
            twist.angular.z = max(
                -self._max_w,
                min(self._max_w, 0.5 * self._k_w * err_yaw),
            )

        self._cmd_pub.publish(twist)
        self._maybe_heartbeat(dist=dist, err_yaw=err_yaw, twist=twist)

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _maybe_heartbeat(
        self,
        dist: Optional[float] = None,
        err_yaw: Optional[float] = None,
        twist: Optional[Twist] = None,
        reason: Optional[str] = None,
    ) -> None:
        now = self._node.get_clock().now()
        if (now - self._last_log_time).nanoseconds / 1e9 < 1.0:
            return
        self._last_log_time = now
        if self._goal is None:
            self._node.get_logger().info(f"[nav-exec] {self._status} no goal")
            return
        g = self._goal.pose.position
        if dist is None or err_yaw is None:
            tail = f" ({reason})" if reason else ""
            self._node.get_logger().info(
                f"[nav-exec] {self._status} goal=({g.x:.2f}, {g.y:.2f}){tail}"
            )
            return
        v = twist.linear.x if twist else 0.0
        w = twist.angular.z if twist else 0.0
        self._node.get_logger().info(
            f"[nav-exec] {self._status} goal=({g.x:.2f}, {g.y:.2f}) "
            f"dist={dist:.2f}m err_yaw={err_yaw:.2f}rad "
            f"cmd_vel=(v={v:.2f}, w={w:.2f})"
        )
