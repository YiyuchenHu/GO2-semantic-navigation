import rclpy
from go2_msgs.msg import Detection2DArray, ObjectObservationArray, SelectedTarget, SemanticEntityArray, TrackedObjectArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


class SemanticMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__("semantic_markers_node")
        self._markers_pub = self.create_publisher(MarkerArray, "/debug/semantic_markers", 10)

        self.create_subscription(ObjectObservationArray, "/perception/objects_3d", self._on_obs, 10)
        self.create_subscription(TrackedObjectArray, "/semantic/tracked_objects", self._on_tracks, 10)
        self.create_subscription(SemanticEntityArray, "/semantic_map/entities", self._on_entities, 10)
        self.create_subscription(SelectedTarget, "/semantic_query/selected_target", self._on_target, 10)
        self.create_subscription(Detection2DArray, "/perception/detections_2d", self._on_det, 10)
        self.get_logger().info("Semantic markers node ready.")

    def _on_det(self, msg: Detection2DArray) -> None:
        del msg

    def _on_obs(self, msg: ObjectObservationArray) -> None:
        markers = MarkerArray()
        for i, obs in enumerate(msg.observations):
            m = Marker()
            m.header = msg.header
            m.ns = "object_observations"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = obs.centroid_map
            m.pose.orientation.w = 1.0
            m.scale.x = 0.12
            m.scale.y = 0.12
            m.scale.z = 0.12
            m.color.r = 1.0
            m.color.g = 0.7
            m.color.b = 0.1
            m.color.a = 0.9
            markers.markers.append(m)
        self._markers_pub.publish(markers)

    def _on_tracks(self, msg: TrackedObjectArray) -> None:
        markers = MarkerArray()
        for i, tr in enumerate(msg.tracks):
            m = Marker()
            m.header = msg.header
            m.ns = "tracked_objects"
            m.id = 1000 + i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = tr.centroid_map
            m.pose.orientation.w = 1.0
            m.scale = tr.size_xyz
            m.color.r = 0.2
            m.color.g = 0.9 if tr.currently_visible else 0.4
            m.color.b = 1.0
            m.color.a = 0.45
            markers.markers.append(m)
        self._markers_pub.publish(markers)

    def _on_entities(self, msg: SemanticEntityArray) -> None:
        markers = MarkerArray()
        for i, e in enumerate(msg.entities):
            m = Marker()
            m.header = msg.header
            m.ns = "semantic_entities"
            m.id = 2000 + i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose = e.pose_map
            m.pose.position.z += 0.35
            m.scale.z = 0.18
            m.text = f"{e.class_label}:{e.confidence:.2f}"
            m.color.r = 1.0
            m.color.g = 1.0
            m.color.b = 1.0
            m.color.a = 1.0
            markers.markers.append(m)
        self._markers_pub.publish(markers)

    def _on_target(self, msg: SelectedTarget) -> None:
        markers = MarkerArray()
        m = Marker()
        m.header = msg.header
        m.ns = "selected_target"
        m.id = 3000
        m.type = Marker.ARROW
        m.action = Marker.ADD
        m.pose = msg.target_pose_map
        m.scale.x = 0.4
        m.scale.y = 0.08
        m.scale.z = 0.08
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.2
        m.color.a = 1.0
        markers.markers.append(m)
        self._markers_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticMarkersNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
