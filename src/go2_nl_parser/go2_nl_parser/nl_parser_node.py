"""Day 8 Phase B — natural-language → SemanticTask parser.

Operator workflow
-----------------
After Phase A finishes (mapping_explorer publishes /mapping/status DONE),
the operator drops a string command on /user_command, e.g.::

    ros2 topic pub --once /user_command std_msgs/msg/String "data: 'go to chair'"

This node parses that string with a regex + keyword-fuzzy-match pipeline
and republishes a go2_msgs/SemanticTask on /semantic_task/request, which
task_coordinator (in target_driven mode) consumes exactly the way the
Day 7 stack expected.

The whole point is to keep this node DEPENDENCY-FREE: no spaCy, no
transformers, no ollama, no LLM API call. The Day 8 demo is a 10×10 m
warehouse with ~5 known classes; a small synonym table covers the
realistic command surface ("go to the desk" / "find me a chair" /
"please navigate over to the box"). Upgrading to an LLM later is a
single function swap (replace ``_match_class``).

Design choices
--------------
* Stop-words and verb prefixes (go, find, fetch, navigate to, please,
  the, a, an, ...) are stripped before matching. Reduces a free-form
  request to one or two content tokens.
* Each known class can declare a list of synonyms; matching uses (a)
  exact token containment, (b) prefix match, (c) ``difflib`` fuzzy
  ratio with a tunable floor. Returns the best class + a confidence
  number.
* Below ``min_match_confidence`` the message is ignored and a warning
  is published on /nl_parser/feedback so the operator sees what
  happened. We never emit a SemanticTask we are not at least somewhat
  confident about.
* task_id is monotonically increasing across the node lifetime
  (``nl-0001``, ``nl-0002``, ...) so target_selector / approach_planner
  logs can disambiguate consecutive commands.

Topic surface
-------------
* IN  ``/user_command``       std_msgs/String
* OUT ``/semantic_task/request``  go2_msgs/SemanticTask
* OUT ``/nl_parser/feedback``     std_msgs/String  (human-readable)

Parameters
----------
* ``known_classes``   list[str]   one entry per primary class label
* ``synonyms``        list[str]   "<class>:<comma-separated synonyms>"
                                  one entry per class. Optional.
* ``min_match_confidence``  float in [0,1]   default 0.55
* ``global_frame``    str         default "map"  (SemanticTask.frame_id)
"""

from __future__ import annotations

import difflib
import re
from typing import Dict, List, Optional, Tuple

import rclpy
from go2_msgs.msg import SemanticTask
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String


# Stop-words and command verbs that don't contribute semantic content.
# Stripped during normalization so "please go to the table" reduces to
# the single content token "table".
_STOPWORDS = frozenset(
    {
        "a", "an", "and", "any", "the",
        "go", "goto", "head", "move", "drive", "walk",
        "find", "fetch", "bring", "look", "search",
        "navigate", "navigation", "to", "towards", "over",
        "please", "could", "would", "you", "i", "me", "we",
        "for", "of", "at", "on", "in",
        "robot", "go2", "dog",
    }
)

# Default per-class synonyms used when the operator doesn't override
# via the `synonyms` launch parameter. Match the YOLOE class list in
# day7.launch.py so that whatever YOLOE detects can be commanded.
_DEFAULT_SYNONYMS: Dict[str, List[str]] = {
    # Day 8+ MVP targets are person and table; synonyms below match the
    # Day 8 YOLOE allowlist so anything the detector can emit also
    # parses as a user command.
    "person": [
        "person", "human", "people", "man", "woman", "guy", "lady",
        # `worker` / `worker on site` covers the Isaac construction-
        # helmet people USDs, which COCO `person` recognises but a
        # naive operator might describe by job rather than species.
        "worker", "construction worker", "pedestrian",
    ],
    "table": ["table", "desk", "dining table", "workbench"],
    "desk":  ["desk", "table", "workbench"],
    "chair": ["chair", "seat", "stool", "armchair", "office chair"],
    "box":   ["box", "crate", "package", "carton"],
    "microwave": ["microwave", "oven"],
}

# Punctuation we want to strip wholesale before tokenization. Keeps
# numbers + alpha intact.
_PUNCT_RE = re.compile(r"[^\w\s]")


class NlParserNode(Node):
    """Tiny offline NL → SemanticTask bridge. See module docstring."""

    def __init__(self) -> None:
        super().__init__("nl_parser_node")

        self.declare_parameter("input_topic", "/user_command")
        self.declare_parameter("task_topic", "/semantic_task/request")
        self.declare_parameter("feedback_topic", "/nl_parser/feedback")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter(
            "known_classes",
            # Day 8+ MVP order: person + table first so tier-1 exact
            # matches resolve those before falling through to the
            # synonym table. Chair / desk / box / microwave stay
            # available so legacy `go to chair` smoke tests keep
            # working — they're just no longer the demo headline.
            ["person", "table", "desk", "chair", "box", "microwave"],
        )
        # Per-class synonym overrides as "<class>:<comma-separated>"
        # entries; falls back to _DEFAULT_SYNONYMS when not provided.
        self.declare_parameter("synonyms", [""])
        # Reject anything below this. 0.65 is the empirical sweet spot
        # for the demo class set (chair / table / desk / box / person /
        # microwave): "chiar" → chair (0.80), "crates" → box (0.91),
        # "desks" → desk (0.89) all pass; "open the door" → microwave
        # (0.75 from "open" ≈ "oven") still slips through. The right
        # cure for the latter is to NOT list a class the scene doesn't
        # contain — set known_classes to exactly what's in the
        # warehouse. Raise this floor to 0.85 if you want the parser
        # to refuse near-misses outright.
        self.declare_parameter("min_match_confidence", 0.65)
        self.declare_parameter("log_period_sec", 5.0)

        self._input_topic = str(self.get_parameter("input_topic").value)
        self._task_topic = str(self.get_parameter("task_topic").value)
        self._feedback_topic = str(
            self.get_parameter("feedback_topic").value
        )
        self._global_frame = str(self.get_parameter("global_frame").value)
        self._min_conf = float(
            self.get_parameter("min_match_confidence").value
        )
        self._log_period_ns = int(
            float(self.get_parameter("log_period_sec").value) * 1e9
        )

        raw_classes = self.get_parameter("known_classes").value or []
        self._known_classes: List[str] = [
            str(c).lower().strip() for c in raw_classes if str(c).strip()
        ]
        if not self._known_classes:
            self.get_logger().warn(
                "nl_parser: known_classes is empty — no command will "
                "ever match. Set the parameter via launch."
            )

        self._synonyms = self._build_synonym_table()

        # Counters
        self._task_seq = 0
        self._n_received = 0
        self._n_parsed = 0
        self._n_rejected = 0
        self._last_log_ns = 0

        self.create_subscription(
            String, self._input_topic, self._on_user_command, 10
        )
        # TRANSIENT_LOCAL so a late `ros2 topic echo --once /nl_parser/feedback`
        # still sees the last RECEIVED/OK/REJECT line after the command landed.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._task_pub = self.create_publisher(
            SemanticTask, self._task_topic, latched
        )
        self._feedback_pub = self.create_publisher(
            String, self._feedback_topic, latched
        )

        self.create_timer(2.0, self._heartbeat_log)
        self.get_logger().info(
            f"nl_parser ready. input={self._input_topic!r} -> "
            f"task={self._task_topic!r} ; "
            f"known_classes={self._known_classes} "
            f"min_conf={self._min_conf:.2f} "
            f"synonyms_per_class="
            f"{ {c: len(s) for c, s in self._synonyms.items()} }"
        )

    # ------------------------------------------------------------------
    # Synonym table construction
    # ------------------------------------------------------------------
    def _build_synonym_table(self) -> Dict[str, List[str]]:
        """Combine the (optional) ``synonyms`` parameter with the
        built-in default table, restricted to ``known_classes``.

        Each entry's value list is lower-cased and de-duplicated, with
        the class label itself always first so the simplest commands
        ("chair") match by exact identity.
        """
        out: Dict[str, List[str]] = {}
        for cls in self._known_classes:
            base = _DEFAULT_SYNONYMS.get(cls, [cls])
            out[cls] = self._dedup_lower([cls] + list(base))

        raw_overrides = self.get_parameter("synonyms").value or []
        for entry in raw_overrides:
            entry_str = str(entry).strip()
            if not entry_str:
                continue
            if ":" not in entry_str:
                self.get_logger().warn(
                    f"nl_parser: ignoring synonym entry {entry_str!r} "
                    f"(expected '<class>:<comma,separated,synonyms>')"
                )
                continue
            cls, syns = entry_str.split(":", 1)
            cls = cls.lower().strip()
            if cls not in out:
                # Allow operator to add brand-new classes via this hook
                # without re-declaring known_classes. Feels surprising
                # otherwise.
                out[cls] = [cls]
                if cls not in self._known_classes:
                    self._known_classes.append(cls)
            out[cls] = self._dedup_lower(
                out[cls] + [s for s in syns.split(",") if s.strip()]
            )
        return out

    @staticmethod
    def _dedup_lower(values: List[str]) -> List[str]:
        seen = set()
        out: List[str] = []
        for v in values:
            v = v.lower().strip()
            if v and v not in seen:
                seen.add(v)
                out.append(v)
        return out

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------
    def _on_user_command(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        self._n_received += 1
        recv_fb = f"RECEIVED raw={raw!r}"
        self._feedback_pub.publish(String(data=recv_fb))
        self.get_logger().info(f"nl_parser: {recv_fb}")
        if not raw:
            self._reject(
                raw,
                "empty command",
                tokens=[],
                matched_class=None,
                confidence=None,
            )
            return

        cls, conf, content_tokens = self._match_class(raw)
        if cls is None or conf < self._min_conf:
            self._reject(
                raw,
                (
                    f"no class match (min_conf={self._min_conf:.2f}) "
                    f"best_match_class={cls!r} best_conf={conf:.2f}"
                ),
                tokens=content_tokens,
                matched_class=cls,
                confidence=conf,
            )
            return

        self._publish_task(raw, cls, conf, content_tokens)

    # ------------------------------------------------------------------
    # Core matching
    # ------------------------------------------------------------------
    def _match_class(
        self, command: str
    ) -> Tuple[Optional[str], float, List[str]]:
        """Return (best_class, confidence in [0,1], content_tokens).

        Confidence = 1.0 for an exact synonym hit, 0.7..0.99 for a
        substring/prefix hit, 0..1 from difflib for a fuzzy match.
        Tokens are returned for diagnostic feedback.
        """
        norm = _PUNCT_RE.sub(" ", command.lower()).strip()
        tokens = [
            t for t in norm.split() if t and t not in _STOPWORDS
        ]

        if not tokens:
            return None, 0.0, tokens

        # Tier 1a: exact class-label match. When `table` and `desk`
        # are mutually listed as each other's synonyms, naively
        # iterating ``synonyms.items()`` returns whichever class comes
        # first in dict order — usually wrong. Match the class label
        # itself first so "go to desk" → desk, not table.
        for cls in self._synonyms.keys():
            if cls in tokens:
                return cls, 1.0, tokens
            if " " in cls and cls in norm:
                return cls, 1.0, tokens

        # Tier 1b: exact synonym match (after class labels failed).
        for cls, syns in self._synonyms.items():
            for s in syns:
                if s == cls:
                    continue  # already tried in Tier 1a
                if s in tokens:
                    return cls, 1.0, tokens
                # Multi-word synonym: check substring of normalized
                # command (post stop-word strip would drop "office",
                # so we test the full normalized string too).
                if " " in s and s in norm:
                    return cls, 1.0, tokens

        # Tier 2: difflib fuzzy match. Score against the class label
        # AND its synonyms; if scores tie within an epsilon, prefer
        # the class whose own label scored highest (so "desks" → desk
        # rather than table).
        best_cls: Optional[str] = None
        best_score: float = 0.0
        # Track per-class best-of-label score so we can break fuzzy
        # ties in favour of the class whose label was the closest
        # match to the user's tokens.
        per_class_label_score: Dict[str, float] = {}
        for tok in tokens:
            for cls, syns in self._synonyms.items():
                for s in syns:
                    score = difflib.SequenceMatcher(
                        None, tok, s
                    ).ratio()
                    if s == cls:
                        per_class_label_score[cls] = max(
                            per_class_label_score.get(cls, 0.0), score
                        )
                    if score > best_score + 1e-6:
                        best_score = score
                        best_cls = cls
                    elif (
                        score > best_score - 1e-6
                        and best_cls is not None
                        and per_class_label_score.get(cls, 0.0)
                        > per_class_label_score.get(best_cls, 0.0)
                    ):
                        # Tie within epsilon — prefer the class whose
                        # own label is the closer match to this token.
                        best_cls = cls
        return best_cls, best_score, tokens

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------
    @staticmethod
    def _canonical_navigation_class(cls: str) -> str:
        """Map internal/parser class labels to Nav+memory canonical labels.

        YOLOE + semantic memory use ``table`` for desk-like furniture; the
        parser still exposes ``desk`` as its own known_class for phrase
        matching, but we always emit ``target_class=table`` for navigation.
        """
        c = (cls or "").lower().strip()
        if c in ("desk", "workbench"):
            return "table"
        if c in ("human", "man", "worker", "people"):
            return "person"
        return c

    def _publish_task(
        self,
        raw: str,
        cls: str,
        conf: float,
        tokens: List[str],
    ) -> None:
        self._task_seq += 1
        self._n_parsed += 1
        nav_cls = self._canonical_navigation_class(cls)
        task = SemanticTask()
        task.header.stamp = self.get_clock().now().to_msg()
        task.header.frame_id = self._global_frame
        task.task_id = f"nl-{self._task_seq:04d}"
        task.raw_command = raw
        task.intent = "find"
        task.target_class = nav_cls
        task.target_label = nav_cls
        syn_src = self._synonyms.get(cls, [])
        if nav_cls != cls:
            syn_src = list(self._synonyms.get(nav_cls, [])) + [cls]
        task.target_aliases = syn_src
        task.frame_id = self._global_frame
        task.requires_search = True
        task.timeout_sec = 0.0
        self._task_pub.publish(task)

        feedback = (
            f"OK task_id={task.task_id} target_class={nav_cls!r} "
            f"parsed_class={cls!r} conf={conf:.2f} raw={raw!r} tokens={tokens}"
        )
        self._feedback_pub.publish(String(data=feedback))
        self.get_logger().info(f"nl_parser: {feedback}")

    def _reject(
        self,
        raw: str,
        reason: str,
        *,
        tokens: Optional[List[str]] = None,
        matched_class: Optional[str] = None,
        confidence: Optional[float] = None,
    ) -> None:
        self._n_rejected += 1
        tok = tokens if tokens is not None else []
        mc = matched_class if matched_class is not None else ""
        conf_s = (
            f"{confidence:.3f}"
            if isinstance(confidence, (int, float))
            else "n/a"
        )
        feedback = (
            f"REJECT raw={raw!r} reason={reason!r} tokens={tok} "
            f"target_class={mc!r} confidence={conf_s}"
        )
        self._feedback_pub.publish(String(data=feedback))
        self.get_logger().warn(f"nl_parser: {feedback}")

    # ------------------------------------------------------------------
    # Liveness log
    # ------------------------------------------------------------------
    def _heartbeat_log(self) -> None:
        now = self.get_clock().now().nanoseconds
        if (now - self._last_log_ns) < self._log_period_ns:
            return
        self._last_log_ns = now
        self.get_logger().info(
            f"[nl_parser/hb] received={self._n_received} "
            f"parsed={self._n_parsed} rejected={self._n_rejected} "
            f"task_seq={self._task_seq}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NlParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
