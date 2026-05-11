"""Day 7 — semantic target selector.

Subscribes
----------
/semantic_map/objects (go2_msgs/SemanticEntityArray)
    Persistent object registry from Day 6's
    semantic_memory_aggregator.

Publishes
---------
/target/selected (go2_msgs/SelectedTarget)
    The current best entity matching the configured ``target_class``
    parameter. Always published — when no entity qualifies, the
    message is emitted with empty ``entity_id`` and ``score=0`` so
    downstream consumers can distinguish "no target" from "topic
    silent". The downstream goal planner uses the empty form as a
    cancel signal.

Selection logic
---------------
On each housekeeping tick (``select_period_sec``, default 0.5 s)
AND on every input message:

  1. Filter ``/semantic_map/objects`` to entities whose
     ``class_label`` matches ``target_class`` (case-insensitive,
     space → underscore normalised so 'office_chair' and
     'office chair' both match).
  2. Drop entities below ``min_confidence``.
  3. Score the remaining entities. Default scoring rewards
     currently-visible entities first (they are the most reliable
     to actually drive to), then high confidence, then proximity
     to the robot's base_link.
  4. Publish the top-scored entity as a SelectedTarget.

The Day 6 aggregator does NOT compute reachability (no costmap
look-up); ``SelectedTarget.reachable`` is left True here as an
optimistic default. Day 7's approach_goal_planner_node performs
the actual costmap-aware feasibility check on the approach pose
and either succeeds (planner publishes a goal) or fails (planner
gives up after retries) — the failure surfaces in NavigateToPose
action result, not in this selector's output.

Why a separate selector node (instead of folding into the planner)
------------------------------------------------------------------
Selection and goal generation are conceptually different:

  * Selection: scoring entities → "which object?"
  * Planning:  approach-pose ring sampling + costmap filtering →
    "where to stop relative to that object?"

Two nodes lets us debug them independently:

  * `ros2 topic echo /target/selected` shows what the selector
    picks every 0.5 s, with score breakdown in `ranking_reasons`.
  * If the selector picks a chair the planner can't reach, the
    planner aborts but the selector keeps re-publishing the same
    target — operator can see selection succeeded but planning
    failed without log archeology.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from go2_msgs.msg import (
    SelectedTarget,
    SemanticEntity,
    SemanticEntityArray,
)
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    LookupException,
    TransformException,
    TransformListener,
)


def _normalise_class(s: str) -> str:
    """Lower-case + space→underscore so 'office chair' == 'office_chair'."""
    return s.strip().lower().replace(" ", "_")


# Defensive client-side alias map. The Day 8+ semantic_memory
# aggregator already canonicalises class labels, so 'desk' detections
# already arrive as 'table' here. This second-line alias map kicks in
# only if the user requests target_class='desk' explicitly or if some
# non-canonicalising upstream (e.g. a legacy ObjectObservation source)
# pushes raw labels through. Keep in sync with
# _DEFAULT_CANONICAL_CLASS_MAP in semantic_memory_aggregator_node.
_DEFAULT_CLIENT_ALIASES: Dict[str, str] = {
    "person": "person", "human": "person", "man": "person",
    "woman": "person", "people": "person", "pedestrian": "person",
    "worker": "person", "construction_worker": "person",
    "table": "table", "desk": "table", "dining_table": "table",
    "workbench": "table", "office_desk": "table",
    "chair": "chair", "office_chair": "chair",
}


class TargetSelectorNode(Node):
    """Pick the best semantic entity matching the requested class."""

    def __init__(self) -> None:
        super().__init__("target_selector")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------
        self.declare_parameter("entities_topic", "/semantic_map/objects")
        self.declare_parameter("selected_topic", "/target/selected")
        # The class to look for. Day 7 MVP exposed a single class —
        # Day 10's command interface rewrites this parameter via
        # `ros2 param set` on receipt of "go to person" / "find the
        # table" / etc. Synonym matching happens upstream in YOLOE
        # via `set_classes()`, not here.
        #
        # Day 8+: default flipped from "chair" to "person" because
        # person + table are the official MVP demo targets. Chair is
        # still selectable (set this param at runtime) but is no
        # longer the headline target — it's too small/thin to land
        # cleanly in the SLAM occupancy map.
        self.declare_parameter("target_class", "person")
        # Minimum confidence on a SemanticEntity for it to be
        # considered a candidate. Day 6's aggregator already filters
        # raw detections at min_detection_confidence; this is a
        # second-line gate on the *aggregated* entity to skip very-
        # decayed ghosts.
        self.declare_parameter("min_confidence", 0.30)
        # Frame to use as the robot's reference for distance scoring.
        # base_link is the canonical Go2 body frame; its pose in the
        # global frame is looked up via tf2.
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("global_frame", "map")
        # Re-evaluate selection at this rate even when no fresh
        # /semantic_map/objects message arrived (useful when the
        # operator changes target_class via ros2 param set and the
        # entity stream is steady-state).
        self.declare_parameter("select_period_sec", 0.5)
        # Per-tick log heartbeat. Set <=0 to disable.
        self.declare_parameter("log_period_sec", 5.0)
        # Scoring weights. Final score = w_visible * (visible ? 1 : 0) +
        # w_confidence * confidence + w_proximity * (1 / (1 + dist)).
        # Defaults bias toward currently-visible entities heavily so
        # the Go2 doesn't repeatedly try to drive to a memory-only
        # ghost when a fresh chair is right in front of it.
        self.declare_parameter("score_weight_visible", 1.0)
        self.declare_parameter("score_weight_confidence", 0.5)
        self.declare_parameter("score_weight_proximity", 0.3)
        # Day 8+: prefer confirmed (= persistent / island-anchored)
        # landmarks. The bonus is large enough that a confirmed
        # not-currently-visible entity outranks a flickery candidate
        # entity that just popped up — exactly the behaviour we want
        # when "go to person" is issued and Go2 has already seen the
        # person earlier in the run.
        self.declare_parameter("score_weight_confirmed", 1.5)
        # Minimum observations before an entity can be selected.
        # 1 = anything goes (legacy behaviour). 2+ filters out
        # one-frame flickers without waiting for the full
        # permanent_after_observations promotion.
        self.declare_parameter("min_observations_count", 1)
        # Tag the entity as confirmed when its observations_count
        # crosses this threshold. Should match the aggregator's
        # ``permanent_after_observations`` parameter.
        self.declare_parameter("confirmed_observations_threshold", 2)
        # Hard requirement: only confirmed landmarks may be selected
        # for navigation. Default False keeps the legacy lenient
        # behaviour ("just pick the best candidate"); flip to True
        # for production demos where you want to refuse to navigate
        # toward a one-frame ghost.
        self.declare_parameter("require_confirmed_for_target", False)
        # ----------------------------------------------------------
        # Day 8++ quality knobs (Task 4). All bonuses are added to
        # the candidate's score; penalties are subtracted. Defaults
        # are calibrated so that a confirmed-with-island landmark
        # almost always outranks a confirmed-without-island ghost,
        # even if the latter is closer.
        # ----------------------------------------------------------
        # Per-observation bonus, capped to avoid the "ent with 9999
        # observations swamps everything" pathology.
        self.declare_parameter("score_weight_observations", 0.05)
        self.declare_parameter("score_observations_cap", 5.0)
        # Penalty for a confirmed landmark that has no island anchor
        # (display_name ends with "|-"). Such entities are usually
        # snapped to free-space — fine as a candidate, suspicious as
        # a navigation target.
        self.declare_parameter("no_island_anchor_penalty", 0.6)
        # Penalty for entities whose pose Z is clearly unrealistic
        # for an indoor person/table standing on the floor (e.g.
        # depth-projection wraparound that puts a marker at z = 4 m).
        self.declare_parameter("suspicious_z_penalty", 0.8)
        self.declare_parameter("suspicious_z_threshold_m", 1.5)
        # Hard skip if display_name flags ``|invalid|`` (Task 3
        # invalidation) or the entity is otherwise marked as a wall
        # artifact. Default True; flip to False for diagnosing.
        self.declare_parameter("skip_invalid_entities", True)
        # ----------------------------------------------------------
        # Day 8++ — Task 5: hard filter for classes that MUST have an
        # island anchor before they are selectable. Space-/comma-
        # separated list. Default "person" matches the aggregator
        # publication gate, so a confirmed-but-no-island person can
        # neither show on /semantic_map/markers nor become a
        # navigation target.
        # ----------------------------------------------------------
        self.declare_parameter(
            "require_island_anchor_for_classes", "person"
        )
        # ----------------------------------------------------------
        # Day 8++ — Task 6: TF-failure handling for the distance
        # term. Pre-Day-8 the selector silently set distance to 0.0
        # whenever the map->base_link lookup failed, which then
        # propagated as ``dist_to_base_link=0.00m`` in the
        # ranking_reasons output and made it look like the robot was
        # standing on top of every selected target. Day 8++ surfaces
        # the failure explicitly:
        #   * estimated_distance is set to NaN
        #   * proximity_score is set to 0 (no proximity bonus when
        #     we don't know the distance)
        #   * a ``distance_unknown_penalty`` is subtracted from the
        #     final score so a confirmed-island candidate beats a
        #     suspicious one whose distance we couldn't even compute
        #   * ``reject_if_distance_unknown`` (default False) hard-
        #     drops candidates instead of penalising them; flip
        #     during demos that absolutely require a real distance.
        # ----------------------------------------------------------
        self.declare_parameter("distance_unknown_penalty", 0.5)
        self.declare_parameter("reject_if_distance_unknown", False)
        # Client-side alias map (defensive — the aggregator already
        # canonicalises). Format: ["alias=canonical", ...].
        self.declare_parameter("class_aliases", [""])

        entities_topic = str(self.get_parameter("entities_topic").value)
        selected_topic = str(self.get_parameter("selected_topic").value)
        # Stash class as a normalised string AND keep the raw form
        # for re-publishing. Param callback below keeps both in sync.
        self._target_class_raw = str(
            self.get_parameter("target_class").value
        )
        self._target_class_norm = _normalise_class(self._target_class_raw)
        self._min_confidence = float(
            self.get_parameter("min_confidence").value
        )
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._global_frame = str(self.get_parameter("global_frame").value)
        select_period = float(self.get_parameter("select_period_sec").value)
        self._log_period = float(self.get_parameter("log_period_sec").value)
        self._w_visible = float(
            self.get_parameter("score_weight_visible").value
        )
        self._w_confidence = float(
            self.get_parameter("score_weight_confidence").value
        )
        self._w_proximity = float(
            self.get_parameter("score_weight_proximity").value
        )
        self._w_confirmed = float(
            self.get_parameter("score_weight_confirmed").value
        )
        self._min_obs = int(
            self.get_parameter("min_observations_count").value
        )
        self._confirmed_obs_thresh = int(
            self.get_parameter("confirmed_observations_threshold").value
        )
        self._require_confirmed = bool(
            self.get_parameter("require_confirmed_for_target").value
        )
        self._w_obs = float(
            self.get_parameter("score_weight_observations").value
        )
        self._obs_cap = float(
            self.get_parameter("score_observations_cap").value
        )
        self._no_island_pen = float(
            self.get_parameter("no_island_anchor_penalty").value
        )
        self._sus_z_pen = float(
            self.get_parameter("suspicious_z_penalty").value
        )
        self._sus_z_thresh = float(
            self.get_parameter("suspicious_z_threshold_m").value
        )
        self._skip_invalid = bool(
            self.get_parameter("skip_invalid_entities").value
        )
        require_anchor_raw = str(
            self.get_parameter(
                "require_island_anchor_for_classes"
            ).value or ""
        )
        self._require_island_classes: set = {
            _normalise_class(s)
            for s in require_anchor_raw.replace(",", " ").replace(";", " ").split()
            if s.strip()
        }
        self._dist_unknown_pen = float(
            self.get_parameter("distance_unknown_penalty").value
        )
        self._reject_if_dist_unknown = bool(
            self.get_parameter("reject_if_distance_unknown").value
        )
        # Build the canonicalisation map: defaults + operator overrides.
        self._aliases: Dict[str, str] = dict(_DEFAULT_CLIENT_ALIASES)
        for spec in (self.get_parameter("class_aliases").value or []):
            if not spec or "=" not in spec:
                continue
            alias, canonical = spec.split("=", 1)
            alias = _normalise_class(alias)
            canonical = _normalise_class(canonical)
            if alias and canonical:
                self._aliases[alias] = canonical

        # --------------------------------------------------------------
        # State + ROS infra
        # --------------------------------------------------------------
        self._latest: Optional[SemanticEntityArray] = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # /semantic_map/objects is RELIABLE (Day 6 aggregator
        # default); match it.
        in_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            SemanticEntityArray, entities_topic, self._on_entities, in_qos
        )
        self._pub = self.create_publisher(
            SelectedTarget, selected_topic, 10
        )
        self.create_timer(select_period, self._select_and_publish)

        # Param-change callback so `ros2 param set /target_selector
        # target_class table` retunes the selector live (Day 10
        # command interface uses this hook).
        self.add_on_set_parameters_callback(self._on_param_change)

        # Heartbeat / metrics
        self._n_selections = 0
        self._n_published_with_target = 0
        self._n_published_empty = 0
        self._last_log_time = self.get_clock().now()

        self.get_logger().info(
            f"target_selector ready. "
            f"in={entities_topic} out={selected_topic} "
            f"target_class={self._target_class_raw!r} "
            f"min_confidence={self._min_confidence} "
            f"base_frame={self._base_frame} "
            f"global_frame={self._global_frame}"
        )

    # ------------------------------------------------------------------
    # Param hot-swap
    # ------------------------------------------------------------------
    def _on_param_change(self, params):
        """Apply runtime updates to target_class / min_confidence /
        scoring weights. Returns SetParametersResult required by rclpy.
        """
        from rcl_interfaces.msg import SetParametersResult

        for p in params:
            if p.name == "target_class":
                old = self._target_class_raw
                self._target_class_raw = str(p.value)
                self._target_class_norm = _normalise_class(
                    self._target_class_raw
                )
                self.get_logger().info(
                    f"target_class changed: {old!r} -> {self._target_class_raw!r}"
                )
            elif p.name == "min_confidence":
                self._min_confidence = float(p.value)
            elif p.name == "score_weight_visible":
                self._w_visible = float(p.value)
            elif p.name == "score_weight_confidence":
                self._w_confidence = float(p.value)
            elif p.name == "score_weight_proximity":
                self._w_proximity = float(p.value)
            elif p.name == "score_weight_confirmed":
                self._w_confirmed = float(p.value)
            elif p.name == "min_observations_count":
                self._min_obs = int(p.value)
            elif p.name == "confirmed_observations_threshold":
                self._confirmed_obs_thresh = int(p.value)
            elif p.name == "require_confirmed_for_target":
                self._require_confirmed = bool(p.value)
            elif p.name == "score_weight_observations":
                self._w_obs = float(p.value)
            elif p.name == "score_observations_cap":
                self._obs_cap = float(p.value)
            elif p.name == "no_island_anchor_penalty":
                self._no_island_pen = float(p.value)
            elif p.name == "suspicious_z_penalty":
                self._sus_z_pen = float(p.value)
            elif p.name == "suspicious_z_threshold_m":
                self._sus_z_thresh = float(p.value)
            elif p.name == "skip_invalid_entities":
                self._skip_invalid = bool(p.value)
            elif p.name == "require_island_anchor_for_classes":
                raw = str(p.value or "")
                self._require_island_classes = {
                    _normalise_class(s)
                    for s in raw.replace(",", " ").replace(";", " ").split()
                    if s.strip()
                }
            elif p.name == "distance_unknown_penalty":
                self._dist_unknown_pen = float(p.value)
            elif p.name == "reject_if_distance_unknown":
                self._reject_if_dist_unknown = bool(p.value)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------
    # Subscription callback
    # ------------------------------------------------------------------
    def _on_entities(self, msg: SemanticEntityArray) -> None:
        self._latest = msg

    # ------------------------------------------------------------------
    # Selection — the hot path (timer + on-message)
    # ------------------------------------------------------------------
    def _canonical_class(self, raw: str) -> str:
        """Canonicalise a class label using the alias map."""
        norm = _normalise_class(raw)
        return self._aliases.get(norm, norm)

    def _is_confirmed_entity(self, e: SemanticEntity) -> bool:
        """Two signals can mark an entity as confirmed:

        1. The aggregator stamps ``display_name`` with
           ``"<raw>|confirmed|<island>"``. Cheap to parse and
           authoritative.
        2. Fallback: ``observations_count >=
           confirmed_observations_threshold``. Useful if the entity
           came from a non-canonicalising upstream that didn't set
           display_name.

        ``|invalid|`` always overrides — an invalidated entity is
        explicitly NOT confirmed for selection purposes.
        """
        dn = (e.display_name or "")
        if "|invalid|" in dn:
            return False
        if "|confirmed|" in dn:
            return True
        return int(e.observations_count) >= self._confirmed_obs_thresh

    def _is_invalid_entity(self, e: SemanticEntity) -> bool:
        """Aggregator-tagged ``|invalid|`` retired-confirmed landmark."""
        return "|invalid|" in (e.display_name or "")

    def _has_island_anchor(self, e: SemanticEntity) -> bool:
        """Parse the third pipe-separated field of display_name. Empty
        ('-') means the aggregator could not anchor this entity onto
        any obstacle island, which we treat as a quality penalty.
        """
        dn = (e.display_name or "")
        if dn.count("|") < 2:
            return False
        last = dn.rsplit("|", 1)[-1].strip()
        return bool(last) and last != "-"

    def _select_and_publish(self) -> None:
        self._n_selections += 1
        out = SelectedTarget()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._global_frame
        out.task_id = ""
        out.class_label = self._target_class_raw
        out.entity_id = ""
        out.score = 0.0
        out.reachable = False
        out.estimated_distance = 0.0
        out.ranking_reasons = []

        if self._latest is None:
            out.ranking_reasons = ["no /semantic_map/objects received yet"]
            self._publish_with_log(out)
            return

        # Filter by canonical class + confidence + observation count.
        # Track per-rejection reasons so a no-target outcome publishes
        # actionable diagnostics instead of a silent "" entity_id.
        target_canon = self._aliases.get(
            self._target_class_norm, self._target_class_norm,
        )
        rejected_low_conf = 0
        rejected_low_obs = 0
        rejected_unconfirmed = 0
        rejected_class_mismatch = 0
        rejected_invalid = 0
        rejected_no_island = 0
        rejected_dist_unknown = 0
        require_island = (
            target_canon in self._require_island_classes
        )
        candidates: List[SemanticEntity] = []
        for e in self._latest.entities:
            ent_canon = self._canonical_class(e.class_label)
            if ent_canon != target_canon:
                rejected_class_mismatch += 1
                continue
            # Hard skip: aggregator has retired this confirmed
            # landmark as a wall artifact (Task 3).
            if self._skip_invalid and self._is_invalid_entity(e):
                rejected_invalid += 1
                continue
            if float(e.confidence) < self._min_confidence:
                rejected_low_conf += 1
                continue
            if int(e.observations_count) < self._min_obs:
                rejected_low_obs += 1
                continue
            if (
                self._require_confirmed
                and not self._is_confirmed_entity(e)
            ):
                rejected_unconfirmed += 1
                continue
            # Day 8++ Task 5 — hard filter for classes that MUST
            # have an island anchor before they're selectable.
            if require_island and not self._has_island_anchor(e):
                rejected_no_island += 1
                continue
            candidates.append(e)

        # Day 8++ Task 6 — score each candidate against the latest
        # base_link pose. Robot pose unavailable ⇒ NaN distance and
        # an explicit penalty (or hard reject when configured).
        robot_xy_full = self._lookup_robot_xy()
        if robot_xy_full is None:
            robot_xy = None
            robot_stamp_ns = 0
            tf_failed = True
        else:
            robot_xy = (robot_xy_full[0], robot_xy_full[1])
            robot_stamp_ns = int(robot_xy_full[2])
            tf_failed = False

        if tf_failed and self._reject_if_dist_unknown:
            # Hard reject path: with no robot pose, every candidate's
            # distance is unknown, so refuse to publish a target.
            out.ranking_reasons = [
                "distance_unknown_tf_failed: map->base_link unavailable; "
                "reject_if_distance_unknown=True ⇒ refusing to select"
            ]
            self._publish_with_log(out)
            return

        if not candidates:
            island_diag = (
                f" require_island={require_island}"
                if require_island else ""
            )
            out.ranking_reasons = [
                f"no entities with canonical class="
                f"{target_canon!r} (req={self._target_class_raw!r}) "
                f"matching min_conf>={self._min_confidence} "
                f"min_obs>={self._min_obs} "
                f"require_confirmed={self._require_confirmed}"
                f"{island_diag}",
                (
                    f"rejected: class_mismatch={rejected_class_mismatch}, "
                    f"low_conf={rejected_low_conf}, "
                    f"low_obs={rejected_low_obs}, "
                    f"unconfirmed={rejected_unconfirmed}, "
                    f"invalid={rejected_invalid}, "
                    f"missing_required_island_anchor={rejected_no_island}"
                ),
            ]
            self._publish_with_log(out)
            return

        scored: List[Tuple[float, SemanticEntity, dict, bool]] = []
        for e in candidates:
            entity_xy = (e.pose_map.position.x, e.pose_map.position.y)
            if robot_xy is None:
                # Distance unavailable — surface as NaN, kill the
                # proximity bonus, and apply the configured penalty.
                dist = float("nan")
                proximity_score = 0.0
                dist_unknown_pen = self._dist_unknown_pen
                rejected_dist_unknown += 1
            else:
                dist = math.hypot(
                    robot_xy[0] - entity_xy[0],
                    robot_xy[1] - entity_xy[1],
                )
                proximity_score = 1.0 / (1.0 + max(0.0, dist))
                dist_unknown_pen = 0.0
            visibility_score = 1.0 if e.currently_visible else 0.0
            confirmed = self._is_confirmed_entity(e)
            confirmed_score = 1.0 if confirmed else 0.0
            obs = int(e.observations_count)
            obs_bonus = min(self._obs_cap, self._w_obs * obs)
            has_island = self._has_island_anchor(e)
            no_island = (not has_island) and confirmed
            no_island_pen = self._no_island_pen if no_island else 0.0
            ent_z = float(e.pose_map.position.z)
            sus_z = abs(ent_z) > self._sus_z_thresh
            sus_z_pen = self._sus_z_pen if sus_z else 0.0
            score = (
                self._w_visible * visibility_score
                + self._w_confidence * float(e.confidence)
                + self._w_proximity * proximity_score
                + self._w_confirmed * confirmed_score
                + obs_bonus
                - no_island_pen
                - sus_z_pen
                - dist_unknown_pen
            )
            breakdown = {
                "visible": visibility_score,
                "confidence": float(e.confidence),
                "proximity": proximity_score,
                "dist_m": dist,
                "confirmed": confirmed_score,
                "obs": obs,
                "obs_bonus": obs_bonus,
                "has_island": has_island,
                "no_island_pen": no_island_pen,
                "sus_z_pen": sus_z_pen,
                "dist_unknown_pen": dist_unknown_pen,
                "ent_z": ent_z,
            }
            scored.append((score, e, breakdown, confirmed))

        scored.sort(key=lambda t: t[0], reverse=True)
        top_score, top_e, top_bd, top_confirmed = scored[0]

        # Quality tag for the navigation layer:
        #   confirmed + currently_visible       => "confirmed_visible"
        #   confirmed + not currently_visible   => "stale_but_confirmed"
        #   not confirmed + currently_visible   => "candidate_visible"
        #   not confirmed + not visible         => "candidate_stale"
        if top_confirmed and top_e.currently_visible:
            quality = "confirmed_landmark"
        elif top_confirmed:
            quality = "stale_but_confirmed"
        elif top_e.currently_visible:
            quality = "candidate_visible"
        else:
            quality = "candidate_not_confirmed"

        out.entity_id = top_e.entity_id
        out.target_pose_map = top_e.pose_map
        out.score = float(top_score)
        # If TF lookup failed we cannot certify reachability — surface
        # estimated_distance as NaN and reachable=False so the
        # downstream goal planner doesn't blindly drive to "0 metres".
        if math.isnan(top_bd["dist_m"]):
            out.reachable = False
            # SelectedTarget.estimated_distance is float32; NaN
            # passes through ROS marshalling, so consumers can
            # ``math.isnan(msg.estimated_distance)`` to detect.
            out.estimated_distance = float("nan")
        else:
            out.reachable = True
            out.estimated_distance = float(top_bd["dist_m"])

        # Per-tag breadcrumb so "why did the selector pick THIS one?"
        # is answerable from /target/selected alone (no need to
        # cross-reference the aggregator log). The tags also
        # double as the canonical names referenced in HOW_TO_RUN.md
        # (Task 4).
        quality_tags = [quality]
        if top_bd["has_island"]:
            quality_tags.append("island_anchored")
        else:
            quality_tags.append("no_island_anchor")
        if top_bd["obs_bonus"] > 0.0:
            quality_tags.append("observations_bonus")
        if top_bd["no_island_pen"] > 0.0:
            quality_tags.append("near_wall_penalty")
        if top_bd["sus_z_pen"] > 0.0:
            quality_tags.append("suspicious_z_penalty")
        if top_bd["dist_unknown_pen"] > 0.0:
            quality_tags.append("distance_unknown_tf_failed")
        if require_island and top_bd["has_island"]:
            quality_tags.append("island_required_satisfied")

        if math.isnan(top_bd["dist_m"]):
            dist_str = "dist_to_base_link=NaN(TF_failed)"
        else:
            dist_str = f"dist_to_base_link={top_bd['dist_m']:.2f}m"
        if robot_xy is not None:
            base_str = (
                f" base_link@map=({robot_xy[0]:.2f},{robot_xy[1]:.2f}) "
                f"tf_stamp_ns={robot_stamp_ns}"
            )
        else:
            base_str = " base_link@map=UNKNOWN tf_stamp_ns=0"

        out.ranking_reasons = [
            f"chose {top_e.entity_id!r} ({quality}) "
            f"from {len(candidates)} candidates "
            f"of canonical class {target_canon!r}; tags="
            f"{','.join(quality_tags)}",
            (
                f"score={top_score:.3f} = "
                f"{self._w_visible:.2f}*visible({top_bd['visible']:.0f}) "
                f"+ {self._w_confidence:.2f}*conf({top_bd['confidence']:.2f}) "
                f"+ {self._w_proximity:.2f}*prox({top_bd['proximity']:.3f}) "
                f"+ {self._w_confirmed:.2f}*confirmed("
                f"{top_bd['confirmed']:.0f}) "
                f"+ obs_bonus({top_bd['obs_bonus']:.2f}) "
                f"- no_island({top_bd['no_island_pen']:.2f}) "
                f"- sus_z({top_bd['sus_z_pen']:.2f}) "
                f"- dist_unknown({top_bd['dist_unknown_pen']:.2f})"
            ),
            (
                f"{dist_str} "
                f"obs_count={top_bd['obs']} "
                f"z={top_bd['ent_z']:.2f}m "
                f"display_name={top_e.display_name!r}"
                f"{base_str}"
            ),
            (
                f"rejected so far: class_mismatch={rejected_class_mismatch}, "
                f"low_conf={rejected_low_conf}, "
                f"low_obs={rejected_low_obs}, "
                f"unconfirmed={rejected_unconfirmed}, "
                f"invalid={rejected_invalid}, "
                f"missing_required_island_anchor={rejected_no_island}, "
                f"distance_unknown_tf_failed={rejected_dist_unknown}"
            ),
        ]
        self._publish_with_log(out)

    def _publish_with_log(self, out: SelectedTarget) -> None:
        if out.entity_id:
            self._n_published_with_target += 1
        else:
            self._n_published_empty += 1
        self._pub.publish(out)
        self._tick_log()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _lookup_robot_xy(self) -> Optional[Tuple[float, float, int]]:
        """Return ``(x, y, transform_stamp_ns)`` for base_link in
        ``global_frame`` via tf2, or None.

        Uses ``Time()`` (latest available) because target selection
        doesn't need stamp-aligned proximity — being off by a frame
        on a slow-moving Go2 changes proximity by < 1 cm.

        Day 8++ Task 6 returns the transform stamp so downstream can
        log "selected at robot pose / TF age / etc." for debug; the
        third tuple element is the integer nanosecond timestamp the
        transform was published at (0 if the buffer doesn't carry it).
        On failure, throttle-warn and return None so
        ``_select_and_publish`` can surface ``distance_unknown_tf_failed``
        in ranking_reasons rather than fabricating a 0.0 distance.
        """
        try:
            t = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.2),
            )
        except (LookupException, TransformException) as exc:
            self.get_logger().warn(
                f"target_selector TF lookup {self._global_frame} -> "
                f"{self._base_frame} failed: {type(exc).__name__}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None
        stamp_ns = (
            int(t.header.stamp.sec) * 1_000_000_000
            + int(t.header.stamp.nanosec)
        )
        return (
            float(t.transform.translation.x),
            float(t.transform.translation.y),
            stamp_ns,
        )

    def _tick_log(self) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now()
        elapsed_ns = (now - self._last_log_time).nanoseconds
        if elapsed_ns < int(self._log_period * 1e9):
            return
        elapsed_s = elapsed_ns / 1e9
        sel_hz = self._n_selections / elapsed_s if elapsed_s > 0 else 0.0
        self.get_logger().info(
            f"target_selector @ {sel_hz:.1f} Hz; "
            f"with_target={self._n_published_with_target} "
            f"empty={self._n_published_empty} "
            f"target_class={self._target_class_raw!r}"
        )
        self._n_selections = 0
        self._n_published_with_target = 0
        self._n_published_empty = 0
        self._last_log_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetSelectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
