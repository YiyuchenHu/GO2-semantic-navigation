import json
from pathlib import Path

import rclpy
from go2_msgs.msg import SelectedTarget, SemanticTask
from rclpy.node import Node
from std_msgs.msg import String


class RuntimeLoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("runtime_logger_node")
        self.declare_parameter("log_file", "/tmp/go2_semantic_nav_runtime.jsonl")
        self._log_file = Path(str(self.get_parameter("log_file").value))
        self._log_file.parent.mkdir(parents=True, exist_ok=True)

        self.create_subscription(SemanticTask, "/semantic_task/current", self._on_task, 10)
        self.create_subscription(SelectedTarget, "/semantic_query/selected_target", self._on_target, 10)
        self.create_subscription(String, "/navigation/status", self._on_nav_status, 10)
        self.create_subscription(String, "/arrival/status", self._on_arrival_status, 10)
        self.create_subscription(String, "/task/status", self._on_task_status, 10)
        self.get_logger().info(f"Runtime logger writing to: {self._log_file}")

    def _write(self, event: str, payload: dict) -> None:
        row = {
            "t_sec": self.get_clock().now().nanoseconds / 1e9,
            "event": event,
            "payload": payload,
        }
        with self._log_file.open("a", encoding="utf-8") as f:
            f.write(json.dumps(row, ensure_ascii=True) + "\n")

    def _on_task(self, msg: SemanticTask) -> None:
        self._write(
            "semantic_task",
            {"task_id": msg.task_id, "target_class": msg.target_class, "raw_command": msg.raw_command},
        )

    def _on_target(self, msg: SelectedTarget) -> None:
        self._write(
            "selected_target",
            {"task_id": msg.task_id, "entity_id": msg.entity_id, "score": msg.score, "reasons": list(msg.ranking_reasons)},
        )

    def _on_nav_status(self, msg: String) -> None:
        self._write("navigation_status", {"status": msg.data})

    def _on_arrival_status(self, msg: String) -> None:
        self._write("arrival_status", {"status": msg.data})

    def _on_task_status(self, msg: String) -> None:
        self._write("task_status", {"state": msg.data})


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RuntimeLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
