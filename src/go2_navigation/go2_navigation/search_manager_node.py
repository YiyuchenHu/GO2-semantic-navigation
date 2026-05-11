"""Phase 4 search / reacquisition layer.

This node sits *above* Phase 3B's nav_executor and *below* any future
behaviour tree or task coordinator. Its single responsibility is:

    If no chair is currently visible (or the chair has been lost for
    long enough), rotate in place so the RGB-D camera sweeps the
    scene, re-enter pursuit as soon as upstream semantic memory
    regains the chair, and stop (LOST) if the sweep goes on too long.

Phase boundary:

  * Phase 3B (nav_executor + simple_p_controller_backend) owns motion
    whenever it has a goal — we never publish /cmd_vel while
    /navigation/status is ROTATING / MOVING / REACHED.
  * Phase 3B's arrival_verifier owns /user_guidance/message — we do
    NOT publish there; we expose our state on /search/status and
    optional RViz markers on /search/markers.
  * Upstream (Phase 1/2/3A) is consumed read-only:
      /semantic_map/entities, /perception/objects_3d,
      /navigation/status, /arrival/status.

Phase 4 deliberately does NOT implement frontier exploration, SLAM
viewpoint selection, multi-object search, or a full behaviour tree.
It is a single-state in-place scan with explicit time budgets. See
docs/phase4_status.md for the deferred items.
"""

from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from go2_msgs.msg import ObjectObservationArray, SemanticEntityArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class SearchManagerNode(Node):
    # Public state strings — also published verbatim on /search/status.
    IDLE = "IDLE"               # nothing to do (no target class configured yet, etc.)
    SEARCHING = "SEARCHING"     # rotating in place to look for the chair
    REACQUIRED = "REACQUIRED"   # one-tick latch; chair just came back, let Phase 3A/3B take over
    PURSUING = "PURSUING"       # Phase 3B is actively driving toward the chair
    ARRIVED = "ARRIVED"         # Phase 3B reports REACHED and arrival is confirmed
    LOST = "LOST"               # searched too long without reacquisition; stop

    _PURSUING_NAV_STATES = ("ROTATING", "MOVING", "REACHED")

    def __init__(self) -> None:
        super().__init__("search_manager_node")

        # Parameters ----------------------------------------------------
        self.declare_parameter("target_class", "chair")
        # How recently a raw perception hit counts as "the chair is
        # still there". 2 s is comfortably larger than Phase 1's ~10 Hz
        # detection cadence so a single missed frame does not trigger
        # a state flip.
        self.declare_parameter("recent_visible_sec", 2.0)
        # How long to keep sweeping before declaring LOST.
        self.declare_parameter("search_timeout_sec", 30.0)
        # Angular rate used during SEARCHING. 0.4 rad/s matches the
        # Phase 3B max_angular / 2 so the scene does not blur.
        self.declare_parameter("search_angular_rate", 0.4)
        self.declare_parameter("loop_hz", 10.0)
        self.declare_parameter("log_period_sec", 2.0)
        self.declare_parameter("global_frame", "odom")

        self._target_cls = str(self.get_parameter("target_class").value).lower().strip()
        self._recent_visible_ns = int(
            float(self.get_parameter("recent_visible_sec").value) * 1e9
        )
        self._search_timeout_ns = int(
            float(self.get_parameter("search_timeout_sec").value) * 1e9
        )
        self._search_w = float(self.get_parameter("search_angular_rate").value)
        loop_hz = float(self.get_parameter("loop_hz").value)
        self._log_period_ns = int(float(self.get_parameter("log_period_sec").value) * 1e9)
        self._global_frame = str(self.get_parameter("global_frame").value)

        # State ---------------------------------------------------------
        self._state = self.IDLE
        self._state_enter_ns = self.get_clock().now().nanoseconds
        self._search_started_ns: Optional[int] = None
        self._last_log_ns = 0
        self._last_chair_seen_ns: Optional[int] = None
        self._has_ever_seen_chair = False

        self._entities: Optional[SemanticEntityArray] = None
        self._nav_status = "IDLE"
        self._arrival_status = "WAITING_FOR_TARGET"
        self._odom: Optional[Odometry] = None

        # I/O -----------------------------------------------------------
        self.create_subscription(
            SemanticEntityArray, "/semantic_map/entities", self._on_entities, 10
        )
        self.create_subscription(
            ObjectObservationArray, "/perception/objects_3d", self._on_objects, 10
        )
        self.create_subscription(String, "/navigation/status", self._on_nav_status, 10)
        self.create_subscription(String, "/arrival/status", self._on_arrival_status, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._status_pub = self.create_publisher(String, "/search/status", 10)
        # Keep the legacy /exploration/enabled bool alive so any
        # downstream code that was already subscribing to it (e.g. a
        # future task coordinator) keeps getting a signal. For Phase 4
        # it is simply `state == SEARCHING`.
        self._explore_pub = self.create_publisher(Bool, "/exploration/enabled", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/search/markers", 10)

        self.create_timer(1.0 / max(loop_hz, 1.0), self._tick)
        self.get_logger().info(
            f"Search manager ready. target='{self._target_cls}' "
            f"recent_visible={self._recent_visible_ns/1e9:.1f}s "
            f"search_timeout={self._search_timeout_ns/1e9:.1f}s "
            f"search_w={self._search_w:.2f}rad/s loop_hz={loop_hz:.1f}"
        )

    # ------------------------------------------------------------------
    # Subscription callbacks
    # ------------------------------------------------------------------
    def _on_entities(self, msg: SemanticEntityArray) -> None:
        self._entities = msg
        # Any currently_visible chair entity counts as "seen now".
        now_ns = self.get_clock().now().nanoseconds
        for e in msg.entities:
            if e.class_label.lower() == self._target_cls and e.currently_visible:
                self._last_chair_seen_ns = now_ns
                self._has_ever_seen_chair = True
                return

    def _on_objects(self, msg: ObjectObservationArray) -> None:
        now_ns = self.get_clock().now().nanoseconds
        for obs in msg.observations:
            if obs.class_label.lower() == self._target_cls:
                self._last_chair_seen_ns = now_ns
                self._has_ever_seen_chair = True
                return

    def _on_nav_status(self, msg: String) -> None:
        self._nav_status = msg.data

    def _on_arrival_status(self, msg: String) -> None:
        self._arrival_status = msg.data

    def _on_odom(self, msg: Odometry) -> None:
        self._odom = msg

    # ------------------------------------------------------------------
    # Main tick
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        chair_seen_recently = (
            self._last_chair_seen_ns is not None
            and (now_ns - self._last_chair_seen_ns) < self._recent_visible_ns
        )
        chair_in_memory = self._entities_has_chair()

        new_state = self._decide_state(
            now_ns=now_ns,
            chair_seen_recently=chair_seen_recently,
            chair_in_memory=chair_in_memory,
        )
        self._transition(new_state, now_ns)

        # Emit motion only in SEARCHING. In all other states, nav_executor
        # (Phase 3B) is either already driving or intentionally idle.
        if self._state == self.SEARCHING:
            self._publish_search_spin()

        # Always publish state + legacy exploration flag + heartbeat.
        self._publish_state_topics()
        self._publish_markers()
        self._maybe_heartbeat(now_ns, chair_seen_recently, chair_in_memory)

    def _decide_state(
        self, now_ns: int, chair_seen_recently: bool, chair_in_memory: bool
    ) -> str:
        # 1) Phase 3B is already driving or has arrived — defer.
        if self._nav_status in self._PURSUING_NAV_STATES:
            if self._arrival_status.startswith("ARRIVED_CONFIRMED"):
                return self.ARRIVED
            return self.PURSUING

        # 2) Phase 3B is IDLE / CANCELED / GOAL_REJECTED. Here we own
        #    the /cmd_vel output, so decide between REACQUIRED / SEARCHING /
        #    LOST.
        if chair_seen_recently:
            # Entities + perception both report a fresh chair — Phase 3A
            # will (re)publish a selected target + goal momentarily and
            # Phase 3B will pick up driving.
            return self.REACQUIRED

        # No fresh chair.
        # Stay LOST until a fresh detection appears (the
        # chair_seen_recently branch above is the only way out of LOST).
        if self._state == self.LOST:
            return self.LOST

        # If already sweeping, honour the timeout.
        if self._state == self.SEARCHING and self._search_started_ns is not None:
            if (now_ns - self._search_started_ns) > self._search_timeout_ns:
                return self.LOST

        # Otherwise sweep. _transition() arms _search_started_ns on the
        # IDLE/REACQUIRED/PURSUING/ARRIVED -> SEARCHING edge.
        return self.SEARCHING

    def _transition(self, new_state: str, now_ns: int) -> None:
        if new_state == self._state:
            return
        old = self._state
        self._state = new_state
        self._state_enter_ns = now_ns

        # Leaving SEARCHING: reset the stopwatch so the next sweep starts
        # fresh. Also push a single zero Twist so CmdVelDriver does not
        # keep integrating the last rotation.
        if old == self.SEARCHING and new_state != self.SEARCHING:
            self._search_started_ns = None
            self._cmd_pub.publish(Twist())
        # Entering SEARCHING: arm the stopwatch.
        if new_state == self.SEARCHING and old != self.SEARCHING:
            self._search_started_ns = now_ns

        self.get_logger().info(f"[search] {old} -> {new_state}")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _entities_has_chair(self) -> bool:
        if self._entities is None:
            return False
        for e in self._entities.entities:
            if e.class_label.lower() == self._target_cls:
                return True
        return False

    def _publish_search_spin(self) -> None:
        t = Twist()
        t.angular.z = self._search_w
        self._cmd_pub.publish(t)

    def _publish_state_topics(self) -> None:
        s = String()
        s.data = self._state
        self._status_pub.publish(s)
        b = Bool()
        b.data = self._state == self.SEARCHING
        self._explore_pub.publish(b)

    def _publish_markers(self) -> None:
        if self._odom is None:
            return
        rx = float(self._odom.pose.pose.position.x)
        ry = float(self._odom.pose.pose.position.y)
        arr = MarkerArray()

        label = Marker()
        label.header.frame_id = self._global_frame
        label.header.stamp = self.get_clock().now().to_msg()
        label.ns = "search_state"
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = rx
        label.pose.position.y = ry
        label.pose.position.z = 1.2
        label.pose.orientation.w = 1.0
        label.scale.z = 0.3
        label.color.a = 0.95
        if self._state == self.SEARCHING:
            label.color.r = 1.0; label.color.g = 0.6; label.color.b = 0.0  # amber
        elif self._state == self.LOST:
            label.color.r = 1.0; label.color.g = 0.1; label.color.b = 0.1  # red
        elif self._state == self.ARRIVED:
            label.color.r = 0.1; label.color.g = 0.9; label.color.b = 0.1  # green
        else:
            label.color.r = 0.9; label.color.g = 0.9; label.color.b = 0.9  # white
        label.text = f"SEARCH: {self._state}"
        arr.markers.append(label)

        # Amber ring around the robot while sweeping, to show "this
        # many metres of sensor radius are being scanned". Purely
        # informative, tiny.
        if self._state == self.SEARCHING:
            ring = Marker()
            ring.header = label.header
            ring.ns = "search_ring"
            ring.id = 0
            ring.type = Marker.CYLINDER
            ring.action = Marker.ADD
            ring.pose.position.x = rx
            ring.pose.position.y = ry
            ring.pose.position.z = 0.02
            ring.pose.orientation.w = 1.0
            ring.scale.x = 6.0
            ring.scale.y = 6.0
            ring.scale.z = 0.02
            ring.color.r = 1.0
            ring.color.g = 0.6
            ring.color.b = 0.0
            ring.color.a = 0.10
            arr.markers.append(ring)

        self._marker_pub.publish(arr)

    def _maybe_heartbeat(
        self, now_ns: int, chair_seen_recently: bool, chair_in_memory: bool
    ) -> None:
        if now_ns - self._last_log_ns < self._log_period_ns:
            return
        self._last_log_ns = now_ns
        seen_ago = (
            "never"
            if self._last_chair_seen_ns is None
            else f"{(now_ns - self._last_chair_seen_ns)/1e9:.1f}s ago"
        )
        sweep_age = (
            "-"
            if self._search_started_ns is None
            else f"{(now_ns - self._search_started_ns)/1e9:.1f}s"
        )
        self.get_logger().info(
            f"[search/hb] state={self._state} "
            f"nav_status={self._nav_status} arrival={self._arrival_status} "
            f"chair_in_memory={chair_in_memory} chair_seen={seen_ago} "
            f"sweep_age={sweep_age}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SearchManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
