import re
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from go2_msgs.msg import SemanticTask
from rclpy.node import Node
from std_msgs.msg import String


@dataclass
class ParsedCommand:
    intent: str
    target_class: str
    target_label: str
    aliases: List[str]
    requires_search: bool


class RuleBasedParser:
    def __init__(self, target_config: Dict) -> None:
        self._targets = target_config.get("targets", {})
        self._patterns = [
            r"\bgo to(?: the)? (?P<obj>[a-z]+)\b",
            r"\btake me to(?: the)? (?P<obj>[a-z]+)\b",
            r"\bfind(?: the)? (?P<obj>[a-z]+)\b",
        ]

    def parse(self, raw: str) -> Optional[ParsedCommand]:
        text = raw.lower().strip()
        for cls, meta in self._targets.items():
            aliases = [a.lower() for a in meta.get("aliases", [])]
            if any(re.search(rf"\b{re.escape(alias)}\b", text) for alias in aliases):
                return ParsedCommand(
                    intent="navigate_to_object",
                    target_class=cls,
                    target_label=cls,
                    aliases=aliases,
                    requires_search=True,
                )

        for pattern in self._patterns:
            match = re.search(pattern, text)
            if match:
                obj = match.group("obj")
                return ParsedCommand(
                    intent="navigate_to_object",
                    target_class=obj,
                    target_label=obj,
                    aliases=[obj],
                    requires_search=True,
                )
        return None


class CommandParserNode(Node):
    def __init__(self) -> None:
        super().__init__("command_parser_node")
        default_cfg = Path(get_package_share_directory("go2_command_interface")) / "config" / "semantic_targets.yaml"
        self.declare_parameter("semantic_targets_file", str(default_cfg))
        cfg_file = Path(self.get_parameter("semantic_targets_file").value)
        self._target_config = self._load_target_config(cfg_file)
        self._parser = RuleBasedParser(self._target_config)

        self._sub = self.create_subscription(String, "/user_command", self._on_command, 10)
        self._pub = self.create_publisher(SemanticTask, "/semantic_task/request", 10)

        self.get_logger().info(f"Command parser ready with targets from: {cfg_file}")

    def _load_target_config(self, path: Path) -> Dict:
        if not path.exists():
            self.get_logger().warning(f"semantic target file not found: {path}")
            return {"targets": {}}
        with path.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {"targets": {}}

    def _on_command(self, msg: String) -> None:
        parsed = self._parser.parse(msg.data)
        if parsed is None:
            self.get_logger().warning(f"Cannot parse command: '{msg.data}'")
            return

        task = SemanticTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.header.frame_id = "map"
        task.task_id = str(uuid.uuid4())
        task.raw_command = msg.data
        task.intent = parsed.intent
        task.target_class = parsed.target_class
        task.target_label = parsed.target_label
        task.target_aliases = parsed.aliases
        task.frame_id = "map"
        task.requires_search = parsed.requires_search
        task.timeout_sec = 120.0
        self._pub.publish(task)
        self.get_logger().info(
            f"Published SemanticTask: task_id={task.task_id}, target={task.target_class}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
