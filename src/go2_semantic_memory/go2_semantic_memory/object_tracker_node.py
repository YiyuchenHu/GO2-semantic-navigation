import uuid
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from go2_msgs.msg import ObjectObservationArray, TrackedObject, TrackedObjectArray
from rclpy.node import Node


@dataclass
class TrackState:
    track_id: str
    class_label: str
    centroid: np.ndarray
    size_xyz: np.ndarray
    velocity_xyz: np.ndarray
    confidence: float
    uncertainty: float
    observations_count: int
    first_seen_ns: int
    last_seen_ns: int
    currently_visible: bool
    is_dynamic: bool


class ObjectTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__("object_tracker_node")
        self.declare_parameter("association_distance_m", 1.0)
        self.declare_parameter("ema_alpha", 0.4)
        self.declare_parameter("dynamic_ttl_sec", 3.0)
        # static_ttl is a trade-off: too short and a chair disappears after a
        # few frames of occlusion; too long and a tracker outlives its entity
        # mapping. The semantic_map_node now re-associates by class+distance
        # on every tracker publish, so this TTL only bounds "how long a
        # silent track floats around" — 60s is plenty for chair-in-warehouse.
        self.declare_parameter("static_ttl_sec", 60.0)
        # Phase 0 TF tree is odom -> base_link -> camera_link. There is no
        # map frame yet, so the tracker's own tick-path (when no observation
        # came in this period) should publish in `odom`, not the hard-coded
        # "map" we used to emit.
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("log_period_sec", 1.0)

        self._assoc_dist = float(self.get_parameter("association_distance_m").value)
        self._alpha = float(self.get_parameter("ema_alpha").value)
        self._dynamic_ttl_ns = int(float(self.get_parameter("dynamic_ttl_sec").value) * 1e9)
        self._static_ttl_ns = int(float(self.get_parameter("static_ttl_sec").value) * 1e9)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)

        self._tracks: Dict[str, TrackState] = {}
        self._dynamic_classes = {"person"}

        # Diagnostic counters for the heartbeat.
        self._obs_msgs_received = 0
        self._obs_items_received = 0
        self._tracks_created_total = 0
        self._tracks_evicted_total = 0
        self._last_log_time = self.get_clock().now()

        self.create_subscription(ObjectObservationArray, "/perception/objects_3d", self._on_obs, 10)
        self._pub = self.create_publisher(TrackedObjectArray, "/semantic/tracked_objects", 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f"Object tracker ready. global_frame='{self._global_frame}' "
            f"assoc_dist={self._assoc_dist}m alpha={self._alpha} "
            f"static_ttl={float(self.get_parameter('static_ttl_sec').value):.1f}s"
        )

    def _on_obs(self, msg: ObjectObservationArray) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._obs_msgs_received += 1
        self._obs_items_received += len(msg.observations)
        unmatched_tracks = set(self._tracks.keys())
        for obs in msg.observations:
            pos = np.array([obs.centroid_map.x, obs.centroid_map.y, obs.centroid_map.z], dtype=np.float32)
            size = np.array([obs.size_xyz.x, obs.size_xyz.y, obs.size_xyz.z], dtype=np.float32)
            best_id, best_dist = self._find_best_track(obs.class_label, pos, unmatched_tracks)
            if best_id is None or best_dist > self._assoc_dist:
                track_id = str(uuid.uuid4())
                self._tracks[track_id] = TrackState(
                    track_id=track_id,
                    class_label=obs.class_label,
                    centroid=pos,
                    size_xyz=size,
                    velocity_xyz=np.zeros(3, dtype=np.float32),
                    confidence=float(obs.confidence),
                    uncertainty=float(obs.uncertainty),
                    observations_count=1,
                    first_seen_ns=now_ns,
                    last_seen_ns=now_ns,
                    currently_visible=True,
                    is_dynamic=obs.class_label in self._dynamic_classes,
                )
                self._tracks_created_total += 1
                self.get_logger().info(
                    f"[tracker] NEW track id={track_id[:8]} class='{obs.class_label}' "
                    f"pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})"
                )
            else:
                unmatched_tracks.discard(best_id)
                tr = self._tracks[best_id]
                dt = max((now_ns - tr.last_seen_ns) / 1e9, 1e-2)
                new_centroid = (1.0 - self._alpha) * tr.centroid + self._alpha * pos
                tr.velocity_xyz = (new_centroid - tr.centroid) / float(dt)
                tr.centroid = new_centroid
                tr.size_xyz = (1.0 - self._alpha) * tr.size_xyz + self._alpha * size
                tr.confidence = float((1.0 - self._alpha) * tr.confidence + self._alpha * obs.confidence)
                tr.uncertainty = float((1.0 - self._alpha) * tr.uncertainty + self._alpha * obs.uncertainty)
                tr.observations_count += 1
                tr.last_seen_ns = now_ns
                tr.currently_visible = True

        for track_id in unmatched_tracks:
            self._tracks[track_id].currently_visible = False

        self._evict_stale(now_ns)
        # Inherit the frame from the upstream ObjectObservationArray so that
        # /semantic/tracked_objects never disagrees with /perception/objects_3d
        # about what frame its coordinates are in.
        frame_id = msg.header.frame_id or self._global_frame
        self._publish(frame_id)
        self._maybe_log_heartbeat()

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        for tr in self._tracks.values():
            tr.currently_visible = (now_ns - tr.last_seen_ns) < int(0.8e9)
        self._evict_stale(now_ns)
        # On the pure-tick path (no fresh observation this period) publish in
        # the configured global frame rather than the legacy hard-coded "map".
        self._publish(self._global_frame)
        self._maybe_log_heartbeat()

    def _evict_stale(self, now_ns: int) -> None:
        dead: List[str] = []
        for track_id, tr in self._tracks.items():
            ttl = self._dynamic_ttl_ns if tr.is_dynamic else self._static_ttl_ns
            if (now_ns - tr.last_seen_ns) > ttl:
                dead.append(track_id)
        for track_id in dead:
            tr = self._tracks[track_id]
            self.get_logger().info(
                f"[tracker] EVICT track id={track_id[:8]} class='{tr.class_label}' "
                f"obs={tr.observations_count} age={(now_ns - tr.last_seen_ns) / 1e9:.1f}s"
            )
            del self._tracks[track_id]
            self._tracks_evicted_total += 1

    def _maybe_log_heartbeat(self) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds / 1e9
        if elapsed < self._log_period:
            return
        self._last_log_time = now
        visible = sum(1 for t in self._tracks.values() if t.currently_visible)
        # Per-class tally, chair-first.
        by_class: Dict[str, int] = {}
        for t in self._tracks.values():
            by_class[t.class_label] = by_class.get(t.class_label, 0) + 1
        breakdown = ", ".join(f"{k}:{v}" for k, v in sorted(by_class.items())) or "<none>"
        self.get_logger().info(
            f"[tracker] obs_msgs={self._obs_msgs_received} "
            f"obs_items={self._obs_items_received} "
            f"active_tracks={len(self._tracks)} visible={visible} "
            f"created_total={self._tracks_created_total} "
            f"evicted_total={self._tracks_evicted_total} "
            f"by_class=[{breakdown}]"
        )

    def _find_best_track(self, cls: str, pos: np.ndarray, candidates: set) -> Tuple[str, float]:
        best_id = None
        best_dist = float("inf")
        for track_id in candidates:
            tr = self._tracks[track_id]
            if tr.class_label != cls:
                continue
            dist = float(np.linalg.norm(tr.centroid - pos))
            if dist < best_dist:
                best_dist = dist
                best_id = track_id
        return best_id, best_dist

    def _publish(self, frame_id: str) -> None:
        msg = TrackedObjectArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        for tr in self._tracks.values():
            t = TrackedObject()
            t.header = msg.header
            t.track_id = tr.track_id
            t.class_label = tr.class_label
            t.confidence = tr.confidence
            t.centroid_map.x = float(tr.centroid[0])
            t.centroid_map.y = float(tr.centroid[1])
            t.centroid_map.z = float(tr.centroid[2])
            t.size_xyz.x = float(tr.size_xyz[0])
            t.size_xyz.y = float(tr.size_xyz[1])
            t.size_xyz.z = float(tr.size_xyz[2])
            t.velocity_xyz.x = float(tr.velocity_xyz[0])
            t.velocity_xyz.y = float(tr.velocity_xyz[1])
            t.velocity_xyz.z = float(tr.velocity_xyz[2])
            t.uncertainty = tr.uncertainty
            t.observations_count = tr.observations_count
            t.first_seen.sec = int(tr.first_seen_ns // int(1e9))
            t.first_seen.nanosec = int(tr.first_seen_ns % int(1e9))
            t.last_seen.sec = int(tr.last_seen_ns // int(1e9))
            t.last_seen.nanosec = int(tr.last_seen_ns % int(1e9))
            t.currently_visible = tr.currently_visible
            t.is_dynamic = tr.is_dynamic
            msg.tracks.append(t)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
