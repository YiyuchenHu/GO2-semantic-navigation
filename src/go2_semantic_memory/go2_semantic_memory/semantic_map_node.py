from dataclasses import dataclass
from typing import Dict, Optional, Set

import numpy as np
import rclpy
from go2_msgs.msg import SemanticEntity, SemanticEntityArray, TrackedObjectArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class EntityState:
    entity_id: str
    class_label: str
    pose: np.ndarray
    size_xyz: np.ndarray
    confidence: float
    observations_count: int
    first_seen_ns: int
    last_seen_ns: int
    currently_visible: bool
    is_dynamic: bool
    uncertainty: float


class SemanticMapNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_map_node")
        # Phase 2 promotion gate. THREE independent paths — ANY passes:
        #
        #   fast      — high-confidence burst.              kept as-is.
        #   stable    — long-lived track at modest conf.    thresholds relaxed.
        #   chair_mvp — chair-specific fallback for the single-chair MVP
        #               scene: if we have repeatedly seen something we call
        #               'chair' we are willing to commit it to memory
        #               regardless of detector confidence, because YOLO
        #               calls the EastRural chair `bench` and its raw
        #               confidence can sit well below 0.2 forever.
        #
        # Flip `chair_mvp_promotion_enabled` to False once Phase 3 brings
        # a real multi-class scene online.
        self.declare_parameter("promotion_min_observations", 3)
        self.declare_parameter("promotion_min_confidence", 0.45)
        self.declare_parameter("promotion_min_observations_stable", 8)
        self.declare_parameter("promotion_min_confidence_stable", 0.15)
        self.declare_parameter("chair_mvp_promotion_enabled", True)
        self.declare_parameter("chair_mvp_min_observations", 8)
        self.declare_parameter("ema_alpha", 0.35)
        self.declare_parameter("dynamic_entity_ttl_sec", 5.0)
        # Phase 2: a persistent chair entity should survive short perception
        # dropouts and also survive the upstream tracker evicting its track
        # after static_ttl_sec (60s by default). 180s is roomy enough that
        # an operator can duck the camera away from the chair and come back
        # without losing the map.
        self.declare_parameter("static_entity_ttl_sec", 180.0)
        # Entity-level re-association: when the upstream tracker issues a
        # brand-new track_id for what is obviously the same physical chair
        # (e.g. after a dropout), match it to an existing entity by
        # class_label + 3D distance rather than creating a duplicate.
        self.declare_parameter("entity_association_distance_m", 1.2)
        # Phase 0 TF tree is odom -> base_link -> camera_link. No map frame
        # exists yet, so the semantic entities/markers must be expressed in
        # the same frame as /semantic/tracked_objects — which is 'odom'.
        self.declare_parameter("global_frame", "odom")
        self.declare_parameter("log_period_sec", 1.0)

        self._promote_n = int(self.get_parameter("promotion_min_observations").value)
        self._promote_conf = float(self.get_parameter("promotion_min_confidence").value)
        self._promote_n_stable = int(self.get_parameter("promotion_min_observations_stable").value)
        self._promote_conf_stable = float(self.get_parameter("promotion_min_confidence_stable").value)
        self._chair_mvp_enabled = bool(self.get_parameter("chair_mvp_promotion_enabled").value)
        self._chair_mvp_min_n = int(self.get_parameter("chair_mvp_min_observations").value)
        self._alpha = float(self.get_parameter("ema_alpha").value)
        self._dyn_ttl_ns = int(float(self.get_parameter("dynamic_entity_ttl_sec").value) * 1e9)
        self._sta_ttl_ns = int(float(self.get_parameter("static_entity_ttl_sec").value) * 1e9)
        self._entity_assoc_dist = float(self.get_parameter("entity_association_distance_m").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)

        self._track_to_entity: Dict[str, str] = {}
        self._entities: Dict[str, EntityState] = {}

        # Diagnostic counters for the heartbeat / RViz marker cleanup state.
        self._tracks_msgs_received = 0
        self._tracks_items_received = 0
        self._promotions_total = 0
        self._entity_evicted_total = 0
        self._last_log_time = self.get_clock().now()
        # Remember which marker IDs we published last tick, so we can emit
        # explicit Marker.DELETE for entities that got evicted. Without this
        # an evicted entity's cube sticks around in RViz forever.
        self._last_entity_marker_ids: Set[int] = set()
        self._last_track_marker_ids: Set[int] = set()
        # Last tracked objects seen on /semantic/tracked_objects, cached so
        # that the heartbeat / marker path can visualize them even between
        # tracker messages.
        self._last_tracks_msg: Optional[TrackedObjectArray] = None

        self.create_subscription(TrackedObjectArray, "/semantic/tracked_objects", self._on_tracks, 10)
        self._entity_pub = self.create_publisher(SemanticEntityArray, "/semantic_map/entities", 10)
        self._marker_pub = self.create_publisher(MarkerArray, "/semantic_map/markers", 10)
        self.create_timer(1.0, self._tick)
        chair_mvp_txt = (
            f"promote_chair_mvp=(obs>={self._chair_mvp_min_n}, any conf)"
            if self._chair_mvp_enabled else "promote_chair_mvp=OFF"
        )
        self.get_logger().info(
            f"Semantic map node ready. global_frame='{self._global_frame}' "
            f"promote_fast=(obs>={self._promote_n}, conf>={self._promote_conf:.2f}) "
            f"promote_stable=(obs>={self._promote_n_stable}, conf>={self._promote_conf_stable:.2f}) "
            f"{chair_mvp_txt} "
            f"assoc_dist={self._entity_assoc_dist}m "
            f"static_ttl={float(self.get_parameter('static_entity_ttl_sec').value):.1f}s"
        )

    def _on_tracks(self, msg: TrackedObjectArray) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._tracks_msgs_received += 1
        self._tracks_items_received += len(msg.tracks)
        self._last_tracks_msg = msg

        for tr in msg.tracks:
            # 1) Fast path: this track_id is already bonded to an entity.
            if tr.track_id in self._track_to_entity:
                entity_id = self._track_to_entity[tr.track_id]
                if entity_id in self._entities:
                    self._update_entity(entity_id, tr, now_ns)
                    continue
                # The entity was evicted out from under us; drop the stale
                # bond and fall through to re-association / re-promotion.
                del self._track_to_entity[tr.track_id]

            tr_pos = np.array(
                [tr.centroid_map.x, tr.centroid_map.y, tr.centroid_map.z],
                dtype=np.float32,
            )

            # 2) Dropout-tolerant re-association: try to bind this fresh
            #    track to an existing entity by class + 3D distance. Without
            #    this step, whenever the upstream tracker evicts its track
            #    (static_ttl) the next observation creates a brand-new
            #    track_id and we would lose the persistent entity.
            reassoc_id = self._find_entity_for_track(tr.class_label, tr_pos)
            if reassoc_id is not None:
                self._track_to_entity[tr.track_id] = reassoc_id
                self._update_entity(reassoc_id, tr, now_ns)
                self.get_logger().info(
                    f"[semantic-map] RE-ASSOC track={tr.track_id[:8]} -> "
                    f"entity={reassoc_id[:8]} class='{tr.class_label}' "
                    f"dist={float(np.linalg.norm(self._entities[reassoc_id].pose - tr_pos)):.2f}m"
                )
                continue

            # 3) Promotion: a physical object seen long enough to earn a
            #    persistent entity. Three independent gates — ANY passes.
            #    See __init__ for the parameter defaults.
            fast_ok = (
                tr.observations_count >= self._promote_n
                and tr.confidence >= self._promote_conf
            )
            stable_ok = (
                tr.observations_count >= self._promote_n_stable
                and tr.confidence >= self._promote_conf_stable
            )
            chair_mvp_ok = (
                self._chair_mvp_enabled
                and tr.class_label == "chair"
                and tr.observations_count >= self._chair_mvp_min_n
            )
            if fast_ok or stable_ok or chair_mvp_ok:
                if fast_ok:
                    gate = "fast"
                elif stable_ok:
                    gate = "stable"
                else:
                    gate = "chair_mvp"
                entity_id = tr.track_id
                self._track_to_entity[tr.track_id] = entity_id
                self._entities[entity_id] = EntityState(
                    entity_id=entity_id,
                    class_label=tr.class_label,
                    pose=tr_pos,
                    size_xyz=np.array([tr.size_xyz.x, tr.size_xyz.y, tr.size_xyz.z], dtype=np.float32),
                    confidence=tr.confidence,
                    observations_count=tr.observations_count,
                    first_seen_ns=int(tr.first_seen.sec * 1e9 + tr.first_seen.nanosec),
                    last_seen_ns=now_ns,
                    currently_visible=tr.currently_visible,
                    is_dynamic=tr.is_dynamic,
                    uncertainty=tr.uncertainty,
                )
                self._promotions_total += 1
                self.get_logger().info(
                    f"[semantic-map] PROMOTED ({gate}) track={tr.track_id[:8]} -> "
                    f"entity={entity_id[:8]} class='{tr.class_label}' "
                    f"pose=({tr_pos[0]:.2f}, {tr_pos[1]:.2f}, {tr_pos[2]:.2f}) "
                    f"obs={tr.observations_count} conf={tr.confidence:.2f}"
                )

        self._evict_stale(now_ns)
        self._publish(self._global_frame)
        self._maybe_log_heartbeat()

    def _find_entity_for_track(
        self, class_label: str, tr_pos: np.ndarray
    ) -> Optional[str]:
        best_id: Optional[str] = None
        best_dist = float("inf")
        for entity_id, e in self._entities.items():
            if e.class_label != class_label:
                continue
            d = float(np.linalg.norm(e.pose - tr_pos))
            if d < best_dist:
                best_dist = d
                best_id = entity_id
        if best_id is None or best_dist > self._entity_assoc_dist:
            return None
        return best_id

    def _update_entity(self, entity_id: str, tr, now_ns: int) -> None:
        if entity_id not in self._entities:
            return
        e = self._entities[entity_id]
        tr_pose = np.array([tr.centroid_map.x, tr.centroid_map.y, tr.centroid_map.z], dtype=np.float32)
        tr_size = np.array([tr.size_xyz.x, tr.size_xyz.y, tr.size_xyz.z], dtype=np.float32)
        e.pose = (1.0 - self._alpha) * e.pose + self._alpha * tr_pose
        e.size_xyz = (1.0 - self._alpha) * e.size_xyz + self._alpha * tr_size
        e.confidence = float((1.0 - self._alpha) * e.confidence + self._alpha * tr.confidence)
        e.uncertainty = float((1.0 - self._alpha) * e.uncertainty + self._alpha * tr.uncertainty)
        e.observations_count = max(e.observations_count, tr.observations_count)
        e.last_seen_ns = now_ns
        e.currently_visible = tr.currently_visible
        e.is_dynamic = tr.is_dynamic

    def _tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        # Refresh currently_visible purely from time-since-last-update so the
        # RViz marker fades out even when no tracker message arrives.
        for e in self._entities.values():
            e.currently_visible = (now_ns - e.last_seen_ns) < int(1.0e9)
        self._evict_stale(now_ns)
        self._publish(self._global_frame)
        self._maybe_log_heartbeat()

    def _evict_stale(self, now_ns: int) -> None:
        dead = []
        for entity_id, e in self._entities.items():
            ttl = self._dyn_ttl_ns if e.is_dynamic else self._sta_ttl_ns
            if now_ns - e.last_seen_ns > ttl:
                dead.append(entity_id)
        for entity_id in dead:
            e = self._entities[entity_id]
            self.get_logger().info(
                f"[semantic-map] EVICT entity={entity_id[:8]} class='{e.class_label}' "
                f"age={(now_ns - e.last_seen_ns) / 1e9:.1f}s obs={e.observations_count}"
            )
            del self._entities[entity_id]
            self._entity_evicted_total += 1
            stale_tracks = [track_id for track_id, eid in self._track_to_entity.items() if eid == entity_id]
            for track_id in stale_tracks:
                del self._track_to_entity[track_id]

    def _maybe_log_heartbeat(self) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed = (now - self._last_log_time).nanoseconds / 1e9
        if elapsed < self._log_period:
            return
        self._last_log_time = now
        visible = sum(1 for e in self._entities.values() if e.currently_visible)
        by_class: Dict[str, int] = {}
        for e in self._entities.values():
            by_class[e.class_label] = by_class.get(e.class_label, 0) + 1
        breakdown = ", ".join(f"{k}:{v}" for k, v in sorted(by_class.items())) or "<none>"
        self.get_logger().info(
            f"[semantic-map] track_msgs={self._tracks_msgs_received} "
            f"track_items={self._tracks_items_received} "
            f"entities={len(self._entities)} visible={visible} "
            f"promotions_total={self._promotions_total} "
            f"evicted_total={self._entity_evicted_total} "
            f"by_class=[{breakdown}]"
        )

    def _publish(self, frame_id: str) -> None:
        entity_arr = SemanticEntityArray()
        entity_arr.header.stamp = self.get_clock().now().to_msg()
        entity_arr.header.frame_id = frame_id

        markers = MarkerArray()
        current_entity_ids: Set[int] = set()

        # --- Persistent entities (blue cubes, 'semantic_entities' ns) ---
        for idx, (entity_id, e) in enumerate(self._entities.items()):
            msg = SemanticEntity()
            msg.header = entity_arr.header
            msg.entity_id = entity_id
            msg.class_label = e.class_label
            msg.display_name = f"{e.class_label}_{entity_id[:8]}"
            msg.pose_map.position.x = float(e.pose[0])
            msg.pose_map.position.y = float(e.pose[1])
            msg.pose_map.position.z = float(e.pose[2])
            msg.pose_map.orientation.w = 1.0
            msg.size_xyz.x = float(max(0.05, e.size_xyz[0]))
            msg.size_xyz.y = float(max(0.05, e.size_xyz[1]))
            msg.size_xyz.z = float(max(0.05, e.size_xyz[2]))
            msg.confidence = e.confidence
            msg.observations_count = e.observations_count
            msg.first_seen.sec = int(e.first_seen_ns // int(1e9))
            msg.first_seen.nanosec = int(e.first_seen_ns % int(1e9))
            msg.last_seen.sec = int(e.last_seen_ns // int(1e9))
            msg.last_seen.nanosec = int(e.last_seen_ns % int(1e9))
            msg.currently_visible = e.currently_visible
            msg.is_dynamic = e.is_dynamic
            msg.uncertainty = e.uncertainty
            entity_arr.entities.append(msg)

            cube = Marker()
            cube.header = entity_arr.header
            cube.ns = "semantic_entities"
            cube.id = idx
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose = msg.pose_map
            cube.scale = msg.size_xyz
            # Blue cube = persistent entity. Bright when still being seen,
            # dimmer otherwise so the operator can visually tell which
            # entities are currently in view vs. remembered only.
            cube.color.r = 0.1 if e.is_dynamic else 0.0
            cube.color.g = 0.8 if e.currently_visible else 0.4
            cube.color.b = 1.0
            cube.color.a = 0.5
            markers.markers.append(cube)

            # A small floating label above the cube with the human-readable
            # display name, so RViz shows "chair_12ab34cd" instead of just
            # a coloured box.
            label = Marker()
            label.header = entity_arr.header
            label.ns = "semantic_entities_labels"
            label.id = idx
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose = msg.pose_map
            label.pose.position.z = msg.pose_map.position.z + max(0.4, float(msg.size_xyz.z) * 0.5 + 0.2)
            label.scale.z = 0.2
            label.color.r = 1.0
            label.color.g = 1.0
            label.color.b = 1.0
            label.color.a = 0.9
            label.text = msg.display_name
            markers.markers.append(label)
            current_entity_ids.add(idx)

        # --- Delete markers for entities that disappeared this tick ---
        for stale_id in self._last_entity_marker_ids - current_entity_ids:
            for ns in ("semantic_entities", "semantic_entities_labels"):
                d = Marker()
                d.header = entity_arr.header
                d.ns = ns
                d.id = stale_id
                d.action = Marker.DELETE
                markers.markers.append(d)
        self._last_entity_marker_ids = current_entity_ids

        # --- Current tracked objects (orange spheres, 'tracks' ns) ---
        #     These are not persistent — they appear / vanish with the
        #     tracker. Useful to visually confirm 'detector sees it NOW'.
        current_track_ids: Set[int] = set()
        if self._last_tracks_msg is not None:
            for idx, tr in enumerate(self._last_tracks_msg.tracks):
                if not tr.currently_visible:
                    continue
                s = Marker()
                s.header = entity_arr.header
                s.ns = "tracks"
                s.id = idx
                s.type = Marker.SPHERE
                s.action = Marker.ADD
                s.pose.position.x = float(tr.centroid_map.x)
                s.pose.position.y = float(tr.centroid_map.y)
                s.pose.position.z = float(tr.centroid_map.z)
                s.pose.orientation.w = 1.0
                s.scale.x = 0.3
                s.scale.y = 0.3
                s.scale.z = 0.3
                s.color.r = 1.0
                s.color.g = 0.5
                s.color.b = 0.0
                s.color.a = 0.8
                markers.markers.append(s)
                current_track_ids.add(idx)

        for stale_id in self._last_track_marker_ids - current_track_ids:
            d = Marker()
            d.header = entity_arr.header
            d.ns = "tracks"
            d.id = stale_id
            d.action = Marker.DELETE
            markers.markers.append(d)
        self._last_track_marker_ids = current_track_ids

        self._entity_pub.publish(entity_arr)
        self._marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
