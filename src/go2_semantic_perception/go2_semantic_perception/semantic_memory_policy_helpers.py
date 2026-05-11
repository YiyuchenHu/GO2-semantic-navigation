"""Ros-free helpers for semantic_memory candidate vs confirmed behaviour.

Keeps deterministic logic unit-test importable without `rclpy` runtime.
"""

from __future__ import annotations


def confirmed_split_visibility_bucket(
    *,
    is_confirmed: bool,
    is_invalid: bool,
    currently_visible: bool,
    anchor_ok_for_marker: bool,
    publish_split: bool,
) -> str | None:
    """Where a semantic entity belongs for split MarkerArray topics.

    Returns ``"visible"``, ``"remembered"``, or ``None`` (not emitted on
    split topics — candidates / invalid / anchor-missing-confirmed).

    ``anchor_ok_for_marker`` mirrors the gate that sends a confirmed
    entity to ``/semantic_map/markers`` (not the REJECT/no-island path).
    """
    if not publish_split:
        return None
    if is_invalid or not is_confirmed or not anchor_ok_for_marker:
        return None
    return "visible" if currently_visible else "remembered"


def effective_detection_confidence_floor(
    global_min: float,
    canonical_class: str,
    cand_person: float,
    cand_table: float,
) -> float:
    """Lower admission floor only for canonical person/table vs ``global_min``."""
    floor = float(global_min)
    if canonical_class == "person":
        floor = min(floor, float(cand_person))
    elif canonical_class == "table":
        floor = min(floor, float(cand_table))
    return floor


def candidate_not_confirmed_hint_from_rejection(
    island_rejection_reason: str,
) -> str:
    """Map island association failure onto debug hint tags."""
    r = (island_rejection_reason or "").strip()
    if r in (
        "wall_like_island",
        "person_too_close_to_wall",
        "table_too_close_to_wall",
        "too_close_to_wall",
        "person_island_shape_invalid",
        "table_island_shape_invalid",
        "island_shape_invalid",
    ):
        return "wall_like_island"
    if r in ("unknown_cell", "outside_map"):
        return "near_unknown"
    return "no_anchor"


def promotion_blocked_without_anchor(
    promotion_path: str,
    class_label: str,
    island_id: str,
    must_anchor_classes: set[str],
) -> bool:
    """True when a promotion path fires but mandatory anchor is missing."""
    if not promotion_path:
        return False
    if class_label not in must_anchor_classes:
        return False
    return not bool((island_id or "").strip())


# /map island looked like furniture failed, while LiDAR pc_* is the main table path.
_ISLAND_SHAPE_FAIL_REASONS = frozenset({
    "wall_like_island",
    "table_too_close_to_wall",
    "table_island_shape_invalid",
    "island_shape_invalid",
    "person_island_shape_invalid",
})


def table_candidate_not_confirmed_tag(
    *,
    island_id: str,
    observations_count: int,
    table_min_observations: int,
    table_allow_single_pc_obs: bool,
    confidence: float,
    confirmed_min_confidence: float,
    pc_cluster_success: bool,
    island_rejection_reason: str,
) -> str:
    """Table-only RViz suffix for ``candidate_not_confirmed: <tag>`` (empty if OK)."""
    iid = (island_id or "").strip()
    if iid.startswith("pc_"):
        if float(confidence) + 1e-9 < float(confirmed_min_confidence):
            return "low_confidence"
        req_obs = 1 if table_allow_single_pc_obs else max(1, int(table_min_observations))
        if int(observations_count) < req_obs:
            return "pc_anchor_ok_waiting_obs"
        return ""
    if iid.startswith("isl_"):
        if float(confidence) + 1e-9 < float(confirmed_min_confidence):
            return "low_confidence"
        if int(observations_count) < max(1, int(table_min_observations)):
            return "needs_more_observations"
        return ""
    if not pc_cluster_success:
        r = (island_rejection_reason or "").strip()
        if r in _ISLAND_SHAPE_FAIL_REASONS:
            return "island_invalid_but_pc_missing"
        if r in ("unknown_cell", "outside_map"):
            return "near_unknown"
    return "no_anchor"


def table_promote_via_pc_anchor_path(
    *,
    observations_count: int,
    table_min_observations: int,
    table_allow_single_pc_obs: bool,
    confidence: float,
    confirmed_min_confidence: float,
    island_id: str,
    is_invalid: bool,
) -> bool:
    """Path A — table + ``pc_*``; occupancy island association is not required."""
    if is_invalid:
        return False
    if not (island_id or "").strip().startswith("pc_"):
        return False
    if float(confidence) + 1e-9 < float(confirmed_min_confidence):
        return False
    req = 1 if table_allow_single_pc_obs else max(1, int(table_min_observations))
    return int(observations_count) >= req


def merge_quality_tuple(
    *,
    is_invalid: bool,
    is_confirmed: bool,
    island_id: str,
    invalid_evidence_count: int,
    observations_count: int,
    confidence: float,
    currently_visible: bool,
    first_seen_ns: int,
) -> tuple:
    """Ordering vector matching ``SemanticMemoryAggregatorNode._quality_score``."""
    anchor = island_id or ""
    has_pc = 1 if anchor.startswith("pc_") else 0
    has_isl = 1 if anchor.startswith("isl_") else 0
    return (
        0 if is_invalid else 1,
        1 if is_confirmed else 0,
        has_pc,
        has_isl,
        -int(invalid_evidence_count),
        int(observations_count),
        float(confidence),
        1 if currently_visible else 0,
        -int(first_seen_ns),
    )
