#!/usr/bin/env python3
"""Offline smoke — semantic_memory demo stability helpers (no ROS spin)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_REPO = Path(__file__).resolve().parents[1]
_SRC = _REPO / "src" / "go2_semantic_perception"
if _SRC.is_dir():
    sys.path.insert(0, str(_SRC))

from go2_semantic_perception.semantic_memory_policy_helpers import (
    candidate_not_confirmed_hint_from_rejection,
    confirmed_split_visibility_bucket,
    effective_detection_confidence_floor,
    merge_quality_tuple,
    promotion_blocked_without_anchor,
    table_candidate_not_confirmed_tag,
    table_promote_via_pc_anchor_path,
)


class SemanticMemorySmoke(unittest.TestCase):
    def test_effective_floor_candidate_only_person_table(self) -> None:
        self.assertAlmostEqual(
            effective_detection_confidence_floor(
                0.45,
                "person",
                cand_person=0.35,
                cand_table=0.35,
            ),
            0.35,
        )
        self.assertAlmostEqual(
            effective_detection_confidence_floor(
                0.45,
                "table",
                cand_person=0.35,
                cand_table=0.35,
            ),
            0.35,
        )
        self.assertAlmostEqual(
            effective_detection_confidence_floor(
                0.45,
                "chair",
                cand_person=0.35,
                cand_table=0.35,
            ),
            0.45,
        )

    def test_candidate_hints(self) -> None:
        self.assertEqual(
            candidate_not_confirmed_hint_from_rejection("wall_like_island"),
            "wall_like_island",
        )
        self.assertEqual(
            candidate_not_confirmed_hint_from_rejection("outside_map"),
            "near_unknown",
        )
        self.assertEqual(
            candidate_not_confirmed_hint_from_rejection("unknown_cell"),
            "near_unknown",
        )
        self.assertEqual(
            candidate_not_confirmed_hint_from_rejection(""),
            "no_anchor",
        )

    def test_promotion_blocked_obs_count_without_anchor(self) -> None:
        must = frozenset({"person", "table"})
        self.assertTrue(
            promotion_blocked_without_anchor(
                "obs_count",
                "person",
                "",
                must,
            )
        )
        self.assertFalse(
            promotion_blocked_without_anchor(
                "obs_count",
                "person",
                "pc_+00002_-00003",
                must,
            )
        )

    def test_nearby_candidate_merges_survivor_is_confirmed(self) -> None:
        cand = merge_quality_tuple(
            is_invalid=False,
            is_confirmed=False,
            island_id="",
            invalid_evidence_count=0,
            observations_count=5,
            confidence=0.99,
            currently_visible=True,
            first_seen_ns=200,
        )
        conf = merge_quality_tuple(
            is_invalid=False,
            is_confirmed=True,
            island_id="pc_00001_00001",
            invalid_evidence_count=0,
            observations_count=2,
            confidence=0.6,
            currently_visible=False,
            first_seen_ns=900,
        )
        self.assertGreater(conf, cand)

    def test_same_pc_anchor_merge_same_anchor_string(self) -> None:
        a = merge_quality_tuple(
            is_invalid=False,
            is_confirmed=True,
            island_id="pc_00001_00001",
            invalid_evidence_count=0,
            observations_count=3,
            confidence=0.8,
            currently_visible=True,
            first_seen_ns=100,
        )
        b = merge_quality_tuple(
            is_invalid=False,
            is_confirmed=True,
            island_id="pc_00001_00001",
            invalid_evidence_count=0,
            observations_count=1,
            confidence=0.95,
            currently_visible=True,
            first_seen_ns=200,
        )
        self.assertTupleEqual(a[0:3], b[0:3])


class VisibilitySplitSmoke(unittest.TestCase):
    def test_confirmed_visible_goes_visible_only(self) -> None:
        self.assertEqual(
            confirmed_split_visibility_bucket(
                is_confirmed=True,
                is_invalid=False,
                currently_visible=True,
                anchor_ok_for_marker=True,
                publish_split=True,
            ),
            "visible",
        )

    def test_confirmed_not_visible_goes_remembered_only(self) -> None:
        self.assertEqual(
            confirmed_split_visibility_bucket(
                is_confirmed=True,
                is_invalid=False,
                currently_visible=False,
                anchor_ok_for_marker=True,
                publish_split=True,
            ),
            "remembered",
        )

    def test_publish_split_false_no_bucket(self) -> None:
        self.assertIsNone(
            confirmed_split_visibility_bucket(
                is_confirmed=True,
                is_invalid=False,
                currently_visible=True,
                anchor_ok_for_marker=True,
                publish_split=False,
            ),
        )

    def test_candidate_not_in_split_topics(self) -> None:
        self.assertIsNone(
            confirmed_split_visibility_bucket(
                is_confirmed=False,
                is_invalid=False,
                currently_visible=True,
                anchor_ok_for_marker=False,
                publish_split=True,
            ),
        )

    def test_invalid_not_in_split_topics(self) -> None:
        self.assertIsNone(
            confirmed_split_visibility_bucket(
                is_confirmed=True,
                is_invalid=True,
                currently_visible=True,
                anchor_ok_for_marker=True,
                publish_split=True,
            ),
        )

    def test_anchor_missing_confirmed_not_in_split_topics(self) -> None:
        self.assertIsNone(
            confirmed_split_visibility_bucket(
                is_confirmed=True,
                is_invalid=False,
                currently_visible=True,
                anchor_ok_for_marker=False,
                publish_split=True,
            ),
        )


class TablePolicySmoke(unittest.TestCase):
    def test_pc_path_confirms_with_two_obs(self) -> None:
        self.assertTrue(
            table_promote_via_pc_anchor_path(
                observations_count=2,
                table_min_observations=2,
                table_allow_single_pc_obs=False,
                confidence=0.62,
                confirmed_min_confidence=0.5,
                island_id="pc_+00001_-00002",
                is_invalid=False,
            )
        )

    def test_pc_path_ignores_island_logic(self) -> None:
        # Promotion bool does not see island — occupancy can fail freely.
        self.assertTrue(
            table_promote_via_pc_anchor_path(
                observations_count=2,
                table_min_observations=2,
                table_allow_single_pc_obs=False,
                confidence=0.7,
                confirmed_min_confidence=0.5,
                island_id="pc_00001_00001",
                is_invalid=False,
            )
        )

    def test_no_anchor_never_pc_promote(self) -> None:
        self.assertFalse(
            table_promote_via_pc_anchor_path(
                observations_count=9,
                table_min_observations=2,
                table_allow_single_pc_obs=False,
                confidence=0.99,
                confirmed_min_confidence=0.5,
                island_id="",
                is_invalid=False,
            )
        )

    def test_isl_only_not_pc_path_but_generic_paths_exist(self) -> None:
        self.assertFalse(
            table_promote_via_pc_anchor_path(
                observations_count=2,
                table_min_observations=2,
                table_allow_single_pc_obs=False,
                confidence=0.6,
                confirmed_min_confidence=0.5,
                island_id="isl_+00100_-00050",
                is_invalid=False,
            )
        )

    def test_candidate_tag_pc_waiting_obs(self) -> None:
        self.assertEqual(
            table_candidate_not_confirmed_tag(
                island_id="pc_00001_00001",
                observations_count=1,
                table_min_observations=2,
                table_allow_single_pc_obs=False,
                confidence=0.7,
                confirmed_min_confidence=0.5,
                pc_cluster_success=True,
                island_rejection_reason="table_island_shape_invalid",
            ),
            "pc_anchor_ok_waiting_obs",
        )

    def test_unanchored_shape_fail_tag(self) -> None:
        self.assertEqual(
            table_candidate_not_confirmed_tag(
                island_id="",
                observations_count=5,
                table_min_observations=2,
                table_allow_single_pc_obs=False,
                confidence=0.9,
                confirmed_min_confidence=0.5,
                pc_cluster_success=False,
                island_rejection_reason="table_island_shape_invalid",
            ),
            "island_invalid_but_pc_missing",
        )


_CHECK_TABLE_SH = _REPO / "scripts" / "check_table_semantic_health.sh"


class TestCheckTableScript(unittest.TestCase):
    def test_debug_stats_uses_full_length(self) -> None:
        txt = _CHECK_TABLE_SH.read_text(encoding="utf-8")
        self.assertIn("--full-length", txt)
        self.assertIn("/depth_projector/debug_stats", txt)


_REPO_TXT = (
    _REPO / "src/go2_semantic_perception/go2_semantic_perception"
    / "semantic_memory_aggregator_node.py"
).read_text(encoding="utf-8")


class SourceSplitPublishSmoke(unittest.TestCase):
    """Static checks — optional split MarkerArray publishers."""

    def test_split_publish_patterns(self) -> None:
        self.assertIn(
            "self._pub_mk_visible.publish(visible_mk)", _REPO_TXT)
        self.assertIn(
            "self._pub_mk_remembered.publish(remembered_mk)", _REPO_TXT,
        )

    def test_disable_split_branch(self) -> None:
        self.assertIn("_pub_mk_visible = None", _REPO_TXT)

    def test_deleteall_prepends_when_split_enabled(self) -> None:
        self.assertIn("visible_landmark_entities", _REPO_TXT)
        self.assertIn("remembered_landmark_entities", _REPO_TXT)
        self.assertRegex(_REPO_TXT, r"visible_mk\s*=\s*MarkerArray\(\)")


class SourceGuards(unittest.TestCase):
    def test_max_confirmed_table_landmarks_default_one(self) -> None:
        self.assertRegex(
            _REPO_TXT,
            r'"max_confirmed_table_landmarks"\s*,\s*1\b',
        )

    def test_split_visibility_params_declared(self) -> None:
        self.assertIn(
            '"publish_split_visibility_markers"', _REPO_TXT)
        self.assertIn('"visible_markers_topic"', _REPO_TXT)
        self.assertIn('"remembered_markers_topic"', _REPO_TXT)


def stale_feedback_classifier(feedback_line: str, user_cmd: str) -> bool:
    """Mirrors nl_parser ``raw=%r`` contract for STALE-vs-OK tests."""
    return ("raw=" + repr(user_cmd.strip())) in feedback_line


class TestDay8Stale(unittest.TestCase):
    def test_matching_feedback_ok(self) -> None:
        self.assertTrue(
            stale_feedback_classifier(
                "parsed_class='table' conf=0.8 raw='go to table'",
                "go to table",
            )
        )

    def test_stale_person_vs_table_ok(self) -> None:
        fb_bad = (
            "parsed_class='person' conf=0.92 raw='go to person' tokens=[]"
        )
        self.assertFalse(stale_feedback_classifier(fb_bad, "go to table"))

    def test_stale_but_selector_ok_should_not_fail_checker(self) -> None:
        self.assertFalse(
            stale_feedback_classifier(
                "RECEIVED raw='go to person'", "go to table"
            )
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
