"""Social obstacle publisher — projects person semantic landmarks into a
`sensor_msgs/PointCloud2` so Nav2's costmap obstacle layer can inflate
a personal-space halo around them.

Why a PointCloud2 instead of a custom cost layer
------------------------------------------------
nav2's stock ``ObstacleLayer`` already supports PointCloud2 observation
sources — adding a new source is a YAML edit, no plugin code required.
For an MVP demo this is the lowest-risk path: we don't have to build,
register or maintain a custom plugin, and any operator who knows
nav2's costmap pipeline immediately understands what's happening.

Design
------
* Subscribe ``/semantic_map/objects`` (``go2_msgs/SemanticEntityArray``)
  — the persistent semantic-landmark stream that
  ``semantic_memory_aggregator_node`` publishes. Cache the most recent
  list of ``person`` entities (``class_label.lower() == "person"``).
* On a fixed 5 Hz timer, materialise a ring of 8 points at radius 0.3 m
  in the map plane (z=0.1) around each cached person centroid. Each
  ring is a thick enough "obstacle hint" that nav2's
  ``ObstacleLayer.markPoint()`` registers a clear lethal cluster, which
  the ``InflationLayer`` then expands by ``inflation_radius`` (set to
  0.8 m in nav2_params for the social-aware demo).
* Publish a single ``sensor_msgs/PointCloud2`` containing every ring
  on ``/social_obstacles``.

QoS
---
* RELIABLE, depth=1, ``TRANSIENT_LOCAL`` durability. Nav2 obstacle
  layer subscribers default to RELIABLE; TRANSIENT_LOCAL on the
  publisher means a late-attached ``ObstacleLayer`` (or an operator
  running ``ros2 topic echo --once``) always sees the most recent
  cloud immediately, instead of waiting for the next 5 Hz tick.

References
----------
* Yao et al. 2026 — "Social-Aware Rewards for Pedestrian-Aware
  Navigation". The 0.3 m ring radius is intentionally smaller than
  the personal-space boundary d_side = 0.7 m used to tune the
  costmap's inflation_radius; the ring marks the body, inflation
  layer enforces the social distance.
"""

from __future__ import annotations

import math
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from go2_msgs.msg import SemanticEntityArray


# Default ring geometry. Kept as module-level constants so an operator
# can tweak them in one place; they're also overridable via ROS params.
_DEFAULT_RING_RADIUS_M = 0.3       # body footprint, NOT the personal-space halo
_DEFAULT_RING_POINTS = 8           # 45° spacing — dense enough that a 0.05 m
                                   # global_costmap resolution always catches
                                   # at least one cell per ring sample.
_DEFAULT_RING_Z = 0.1              # above the floor; inside Nav2 obstacle
                                   # layer's typical [min_obstacle_height,
                                   # max_obstacle_height] window.
_DEFAULT_PUBLISH_HZ = 5.0          # matches the recommended
                                   # `expected_update_rate: 0.5` (×10 margin)
                                   # set on the costmap source in the
                                   # accompanying YAML.


class SocialObstaclePublisherNode(Node):
    """See module docstring."""

    def __init__(self) -> None:
        super().__init__("social_obstacle_publisher")

        self.declare_parameter("entities_topic", "/semantic_map/objects")
        self.declare_parameter("output_topic", "/social_obstacles")
        # frame_id is fixed to map for the social cloud — semantic
        # landmarks already live there, so no TF lookup is needed and
        # the costmap source can use ``sensor_frame: map`` without
        # any time-extrapolation risk.
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate_hz", _DEFAULT_PUBLISH_HZ)
        self.declare_parameter("ring_radius_m", _DEFAULT_RING_RADIUS_M)
        self.declare_parameter("ring_points", _DEFAULT_RING_POINTS)
        self.declare_parameter("ring_z_m", _DEFAULT_RING_Z)
        # Filter: only count persons whose semantic-memory confidence
        # is at least this high. Stops a ghost detection from blocking
        # the planner. 0 disables filtering.
        self.declare_parameter("min_confidence", 0.0)
        self.declare_parameter("log_period_sec", 5.0)

        self._entities_topic = str(
            self.get_parameter("entities_topic").value
        )
        self._output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        publish_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_hz <= 0.0:
            self.get_logger().warn(
                f"publish_rate_hz={publish_hz} invalid; falling back to "
                f"{_DEFAULT_PUBLISH_HZ} Hz."
            )
            publish_hz = _DEFAULT_PUBLISH_HZ
        self._publish_period = 1.0 / publish_hz
        self._ring_radius = float(self.get_parameter("ring_radius_m").value)
        self._ring_points = max(
            3, int(self.get_parameter("ring_points").value)
        )
        self._ring_z = float(self.get_parameter("ring_z_m").value)
        self._min_confidence = float(
            self.get_parameter("min_confidence").value
        )
        self._log_period_ns = int(
            float(self.get_parameter("log_period_sec").value) * 1e9
        )

        # Pre-compute the per-ring (cos, sin) offsets — cheaper than
        # re-running trig on every timer tick when there are several
        # persons in the scene.
        self._ring_offsets: List[Tuple[float, float]] = []
        for k in range(self._ring_points):
            theta = 2.0 * math.pi * k / self._ring_points
            self._ring_offsets.append(
                (self._ring_radius * math.cos(theta),
                 self._ring_radius * math.sin(theta))
            )

        # Latest list of (x, y) person centroids in map frame. Refreshed
        # on every /semantic_map/objects message; consumed by the timer.
        self._person_xy: List[Tuple[float, float]] = []
        self._n_msgs_in = 0
        self._n_clouds_out = 0
        self._last_log_ns = 0

        self.create_subscription(
            SemanticEntityArray,
            self._entities_topic,
            self._on_entities,
            10,
        )

        # Latched RELIABLE so a late-attached costmap or `ros2 topic
        # echo --once /social_obstacles` always gets the most recent
        # cloud without waiting for the next 5 Hz tick.
        out_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._cloud_pub = self.create_publisher(
            PointCloud2, self._output_topic, out_qos
        )

        self.create_timer(self._publish_period, self._publish_cloud)

        self.get_logger().info(
            f"social_obstacle_publisher ready. "
            f"in={self._entities_topic!r} "
            f"out={self._output_topic!r} "
            f"frame={self._frame_id!r} "
            f"rate={publish_hz:.1f}Hz "
            f"ring=({self._ring_points}pts, r={self._ring_radius:.2f}m, "
            f"z={self._ring_z:.2f}m) "
            f"min_conf={self._min_confidence:.2f}"
        )

    def _on_entities(self, msg: SemanticEntityArray) -> None:
        """Cache person centroids; cheap (drops anything non-person)."""
        self._n_msgs_in += 1
        new_xy: List[Tuple[float, float]] = []
        for ent in msg.entities:
            cls = (ent.class_label or "").lower().strip()
            if cls != "person":
                continue
            if (
                self._min_confidence > 0.0
                and float(ent.confidence) < self._min_confidence
            ):
                continue
            new_xy.append(
                (
                    float(ent.pose_map.position.x),
                    float(ent.pose_map.position.y),
                )
            )
        self._person_xy = new_xy

    def _publish_cloud(self) -> None:
        """Materialise rings around each cached person, publish once."""
        header = Header()
        header.frame_id = self._frame_id
        header.stamp = self.get_clock().now().to_msg()

        # Always publish — including an empty cloud — so Nav2's
        # obstacle source `expected_update_rate` doesn't trip and
        # so the operator can `topic hz /social_obstacles` to
        # confirm the node is alive even before any person lands
        # in semantic memory.
        points = []
        for px, py in self._person_xy:
            for dx, dy in self._ring_offsets:
                points.append((px + dx, py + dy, self._ring_z))

        cloud = point_cloud2.create_cloud_xyz32(header, points)
        self._cloud_pub.publish(cloud)
        self._n_clouds_out += 1
        self._maybe_log(len(self._person_xy), len(points))

    def _maybe_log(self, n_persons: int, n_points: int) -> None:
        if self._log_period_ns <= 0:
            return
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - self._last_log_ns) < self._log_period_ns:
            return
        self._last_log_ns = now_ns
        self.get_logger().info(
            f"[social/hb] msgs_in={self._n_msgs_in} "
            f"clouds_out={self._n_clouds_out} "
            f"persons={n_persons} points={n_points}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SocialObstaclePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
