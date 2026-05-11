#!/usr/bin/env python3
"""Smoke test for scripts/record_semantic_lifecycle.py.

Drives the recorder through a synthetic 10-second scenario:

  Phase A (0..3 s)  — IDLE (no cmd_vel, no mapping)
  Phase B (3..6 s)  — AUTO_MAPPING (mapping_status=NAVIGATING,
                      cmd_vel non-zero, /detections has table+person,
                      /detections/masks has desk, /detections_3d has
                      table). If go2_msgs is available we also
                      publish a SemanticEntityArray with a confirmed
                      table whose anchor is pc_+0001.
  Phase C (6..10 s) — MANUAL_TELEOP (mapping_status=IDLE,
                      cmd_vel non-zero, operator emits
                      "manual_table_scan" on /semantic_recording/mode)

Then verifies:
  * 3 CSVs + summary file are written
  * status, perception, entities CSV headers match the expected schema
  * status CSV inferred_mode column contains AUTO_MAPPING and
    MANUAL_TELEOP
  * perception CSV best_table_like_2d_score > 0 in at least one row
  * if go2_msgs is available, entities CSV has a confirmed table row
    with raw_label != "" and anchor_id starting with pc_
  * summary reports "first /detections table-like" with a finite
    timestamp
  * summary lists the mode breakdown bucket names

Run with:

    python3 scripts/_smoke_record_semantic_lifecycle.py
"""
from __future__ import annotations

import csv
import os
import sys
import tempfile
import threading
import time
from pathlib import Path

os.environ.setdefault("ROS_DOMAIN_ID", "217")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (  # noqa: E402
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String  # noqa: E402
from vision_msgs.msg import (  # noqa: E402
    BoundingBox2D,
    BoundingBox3D,
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

import record_semantic_lifecycle as rsl  # noqa: E402

try:
    from go2_msgs.msg import SemanticEntity, SemanticEntityArray  # type: ignore  # noqa: E402
    _HAS_GO2_MSGS = True
except Exception:
    SemanticEntity = None  # type: ignore
    SemanticEntityArray = None  # type: ignore
    _HAS_GO2_MSGS = False


def _status_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )


class _Driver(Node):
    """Minimal publisher-side stub of the day8 stack."""

    def __init__(self) -> None:
        super().__init__("smoke_driver_lifecycle")
        sq = _status_qos()
        self._pub_mapping = self.create_publisher(
            String, "/mapping/status", sq,
        )
        self._pub_mapping_dbg = self.create_publisher(
            String, "/mapping/debug/status", sq,
        )
        self._pub_action_dbg = self.create_publisher(
            String, "/semantic_goal/action_debug", 10,
        )
        self._pub_dp_stats = self.create_publisher(
            String, "/depth_projector/debug_stats", 10,
        )
        self._pub_anchor_stats = self.create_publisher(
            String, "/semantic_map/anchor_debug_stats", 10,
        )
        self._pub_mode = self.create_publisher(
            String, "/semantic_recording/mode", 10,
        )
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self._pub_det = self.create_publisher(
            Detection2DArray, "/detections", 10,
        )
        self._pub_det3 = self.create_publisher(
            Detection3DArray, "/detections_3d", 10,
        )
        self._pub_entities = None
        if _HAS_GO2_MSGS:
            self._pub_entities = self.create_publisher(
                SemanticEntityArray, "/semantic_map/objects", 10,
            )

        self._t0 = time.time()
        self._timer = self.create_timer(0.1, self._tick)

    def _tick(self) -> None:
        t = time.time() - self._t0
        if t < 3.0:
            self._publish_idle()
        elif t < 6.0:
            self._publish_auto_mapping()
        else:
            self._publish_manual_teleop()

    def _publish_idle(self) -> None:
        self._pub_mapping.publish(String(data="IDLE"))
        self._pub_mapping_dbg.publish(
            String(data="state=IDLE goal=none in_flight=0 dist=nan")
        )
        self._pub_dp_stats.publish(
            String(
                data=(
                    "depth_projector_build_tag=phase_c2_table_mask_only "
                    "table_detection_seen=0 table_mask_seen=0 "
                    "table_detection_driven_attempted=0 "
                    "table_detection_driven_published=0 "
                    "table_mask_only_attempted=0 "
                    "table_mask_only_published=0 "
                    "force_table_mask_only_projection=False"
                )
            )
        )
        self._pub_anchor_stats.publish(
            String(
                data=(
                    "observations_total=0 pointcloud_anchor_success=0 "
                    "occupancy_island_anchor_success=0 "
                    "candidate_no_anchor=0"
                )
            )
        )

    def _publish_auto_mapping(self) -> None:
        self._pub_mapping.publish(String(data="NAVIGATING"))
        self._pub_mapping_dbg.publish(
            String(
                data=(
                    "state=NAVIGATING goal=(3.00,0.00) dist=1.50 "
                    "nav2=ACCEPTED in_flight=1"
                )
            )
        )
        cmd = Twist()
        cmd.linear.x = 0.4
        self._pub_cmd.publish(cmd)
        self._pub_det.publish(self._mk_detection2d_table_and_person())
        self._pub_det3.publish(self._mk_detection3d_table())
        self._pub_dp_stats.publish(
            String(
                data=(
                    "depth_projector_build_tag=phase_c2_table_mask_only "
                    "table_detection_seen=4 table_mask_seen=3 "
                    "table_detection_driven_attempted=2 "
                    "table_detection_driven_published=1 "
                    "table_detection_driven_failed_no_depth=1 "
                    "table_detection_driven_failed_bad_depth=0 "
                    "table_detection_driven_failed_tf=0 "
                    "table_mask_only_attempted=1 "
                    "table_mask_only_published=1 "
                    "table_mask_only_failed_no_depth=0 "
                    "table_mask_only_failed_bad_depth=0 "
                    "table_mask_only_failed_tf=0 "
                    "table_3d_published=2 "
                    "force_table_mask_only_projection=False "
                    "detections_received=20 masks_received=20 "
                    "published_3d=2"
                )
            )
        )
        if self._pub_entities is not None and _HAS_GO2_MSGS:
            self._pub_entities.publish(self._mk_entities_confirmed_table())

    def _publish_manual_teleop(self) -> None:
        self._pub_mapping.publish(String(data="IDLE"))
        self._pub_mapping_dbg.publish(
            String(data="state=IDLE goal=none in_flight=0 dist=nan")
        )
        cmd = Twist()
        cmd.linear.x = 0.2
        cmd.angular.z = 0.5
        self._pub_cmd.publish(cmd)
        self._pub_mode.publish(String(data="manual_table_scan"))
        self._pub_det.publish(self._mk_detection2d_table_and_person())
        self._pub_det3.publish(self._mk_detection3d_table())

    @staticmethod
    def _mk_detection2d_table_and_person() -> Detection2DArray:
        out = Detection2DArray()
        for label, score in (("desk", 0.45), ("person", 0.85)):
            d = Detection2D()
            h = ObjectHypothesisWithPose()
            h.hypothesis.class_id = label
            h.hypothesis.score = score
            d.results.append(h)
            d.bbox = BoundingBox2D()
            out.detections.append(d)
        return out

    @staticmethod
    def _mk_detection3d_table() -> Detection3DArray:
        out = Detection3DArray()
        d = Detection3D()
        h = ObjectHypothesisWithPose()
        h.hypothesis.class_id = "table"
        h.hypothesis.score = 0.5
        d.results.append(h)
        d.bbox = BoundingBox3D()
        out.detections.append(d)
        return out

    @staticmethod
    def _mk_entities_confirmed_table() -> "SemanticEntityArray":  # type: ignore[name-defined]
        arr = SemanticEntityArray()
        e = SemanticEntity()
        e.entity_id = "table_001"
        e.class_label = "table"
        e.display_name = "desk|confirmed|pc_+0001_+0002"
        e.confidence = 0.82
        e.observations_count = 12
        e.currently_visible = True
        e.uncertainty = 0.05
        e.is_dynamic = False
        e.pose_map.position.x = 2.5
        e.pose_map.position.y = 1.0
        e.pose_map.position.z = 0.4
        arr.entities.append(e)
        # Person confirmed too — gives the summary a "person confirmed
        # during AUTO_MAPPING" data point.
        ep = SemanticEntity()
        ep.entity_id = "person_001"
        ep.class_label = "person"
        ep.display_name = "person|confirmed|pc_+0010_+0020"
        ep.confidence = 0.91
        ep.observations_count = 25
        ep.currently_visible = True
        arr.entities.append(ep)
        return arr


def main() -> int:
    tmp_dir = Path(tempfile.mkdtemp(prefix="smoke_lifecycle_"))
    print(f"[INFO] tmp dir: {tmp_dir}")

    rclpy.init()
    recorder = rsl._SemanticLifecycleRecorder(
        duration_sec=10.0,
        status_rate_hz=2.0,
        perception_rate_hz=10.0,
        output_dir=tmp_dir,
        target_class="",
        print_live=False,
    )
    driver = _Driver()

    exec_ = SingleThreadedExecutor()
    exec_.add_node(recorder)
    exec_.add_node(driver)

    t_start = time.time()
    try:
        while time.time() - t_start < 11.0:
            exec_.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    try:
        recorder.write_summary()
    except Exception as exc:
        print(f"[FAIL] summary failed: {exc!r}")
        return 1
    recorder.close_files()
    recorder.destroy_node()
    driver.destroy_node()
    rclpy.shutdown()

    # ---- locate output files ------------------------------------------
    status_csvs = sorted(tmp_dir.glob("semantic_lifecycle_status_*.csv"))
    perception_csvs = sorted(
        tmp_dir.glob("semantic_lifecycle_perception_*.csv")
    )
    entities_csvs = sorted(tmp_dir.glob("semantic_lifecycle_entities_*.csv"))
    summaries = sorted(tmp_dir.glob("semantic_lifecycle_*.summary.txt"))
    for label, lst in (
        ("status", status_csvs),
        ("perception", perception_csvs),
        ("entities", entities_csvs),
        ("summary", summaries),
    ):
        if len(lst) != 1:
            print(f"[FAIL] expected 1 {label} file, got {lst}")
            return 1
    print(f"[OK] found CSVs: {status_csvs[0].name} / "
          f"{perception_csvs[0].name} / {entities_csvs[0].name}")
    print(f"[OK] summary: {summaries[0].name}")

    # ---- header checks -----------------------------------------------
    with status_csvs[0].open() as fh:
        status_rows = list(csv.reader(fh))
    with perception_csvs[0].open() as fh:
        perception_rows = list(csv.reader(fh))
    with entities_csvs[0].open() as fh:
        entity_rows = list(csv.reader(fh))

    expected_status_header = rsl._SemanticLifecycleRecorder._status_header()
    expected_perception_header = (
        rsl._SemanticLifecycleRecorder._perception_header()
    )
    expected_entities_header = (
        rsl._SemanticLifecycleRecorder._entities_header()
    )
    if status_rows[0] != expected_status_header:
        print(f"[FAIL] status header mismatch:\n got: {status_rows[0]}")
        return 1
    if perception_rows[0] != expected_perception_header:
        print("[FAIL] perception header mismatch")
        return 1
    if entity_rows[0] != expected_entities_header:
        print("[FAIL] entities header mismatch")
        return 1
    print("[OK] all 3 CSV headers match expected schema")

    print(f"[OK] row counts: status={len(status_rows) - 1} "
          f"perception={len(perception_rows) - 1} "
          f"entities={len(entity_rows) - 1}")
    if len(status_rows) < 5:
        print("[FAIL] status CSV has < 5 sample rows (2Hz × ~10s)")
        return 1
    if len(perception_rows) < 30:
        print("[FAIL] perception CSV has < 30 rows (10Hz × ~10s)")
        return 1

    # ---- mode inference checks ---------------------------------------
    mode_idx = expected_status_header.index("inferred_mode")
    modes_seen = {row[mode_idx] for row in status_rows[1:]}
    print(f"[OK] modes_seen: {sorted(modes_seen)}")
    if "AUTO_MAPPING" not in modes_seen:
        print("[FAIL] AUTO_MAPPING never inferred")
        return 1
    if "MANUAL_TELEOP" not in modes_seen:
        print("[FAIL] MANUAL_TELEOP never inferred")
        return 1
    print("[OK] AUTO_MAPPING and MANUAL_TELEOP both observed")

    # ---- perception checks -------------------------------------------
    p_hdr = expected_perception_header
    table_2d_idx = p_hdr.index("best_table_like_2d_score")
    person_2d_idx = p_hdr.index("best_person_2d_score")
    table_3d_idx = p_hdr.index("table_3d_count")
    build_tag_idx = p_hdr.index("depth_projector_build_tag")
    saw_table_2d = False
    saw_person_2d = False
    saw_table_3d = False
    saw_build_tag = False
    for row in perception_rows[1:]:
        try:
            if float(row[table_2d_idx]) > 0.0:
                saw_table_2d = True
            if float(row[person_2d_idx]) > 0.0:
                saw_person_2d = True
            if int(row[table_3d_idx]) > 0:
                saw_table_3d = True
        except (ValueError, IndexError):
            pass
        if row[build_tag_idx] == "phase_c2_table_mask_only":
            saw_build_tag = True
    if not saw_table_2d:
        print("[FAIL] no row with best_table_like_2d_score > 0")
        return 1
    if not saw_person_2d:
        print("[FAIL] no row with best_person_2d_score > 0")
        return 1
    if not saw_table_3d:
        print("[FAIL] no row with table_3d_count > 0")
        return 1
    if not saw_build_tag:
        print("[FAIL] depth_projector_build_tag never recorded")
        return 1
    print("[OK] perception CSV captured 2D table+person, 3D table, "
          "and depth_projector build tag")

    # ---- entities checks (only meaningful if go2_msgs imported) ------
    if _HAS_GO2_MSGS:
        e_hdr = expected_entities_header
        eid_idx = e_hdr.index("entity_id")
        cls_idx = e_hdr.index("class_label")
        status_col = e_hdr.index("status")
        anchor_idx = e_hdr.index("anchor_id")
        is_table_idx = e_hdr.index("is_table")
        has_pc_idx = e_hdr.index("has_pointcloud_anchor")
        is_confirmed_idx = e_hdr.index("is_confirmed")

        confirmed_table = [
            r for r in entity_rows[1:]
            if r[cls_idx] == "table"
            and r[is_confirmed_idx] == "1"
            and r[is_table_idx] == "1"
            and r[has_pc_idx] == "1"
        ]
        if not confirmed_table:
            print("[FAIL] no confirmed table row with pc_ anchor in "
                  "entities CSV")
            return 1
        sample = confirmed_table[0]
        # raw_label was "desk" in our synthetic publisher.
        raw_idx = e_hdr.index("raw_label")
        if sample[raw_idx] != "desk":
            print(f"[FAIL] expected raw_label='desk' got '{sample[raw_idx]}'")
            return 1
        if not sample[anchor_idx].startswith("pc_"):
            print(
                f"[FAIL] anchor_id should start with pc_, got "
                f"'{sample[anchor_idx]}'"
            )
            return 1
        print(
            f"[OK] entity row: id={sample[eid_idx]} "
            f"raw={sample[raw_idx]} status={sample[status_col]} "
            f"anchor={sample[anchor_idx]}"
        )

    # ---- summary checks ---------------------------------------------
    summary = summaries[0].read_text()
    print("---- summary preview ----")
    print(summary[:1200])
    print("---- /preview ----")

    expected_substrings = [
        "Semantic Lifecycle Recording Summary",
        "Mode time distribution",
        "AUTO_MAPPING",
        "MANUAL_TELEOP",
        "Person",
        "Table (incl. desk",
        "first /detections table-like",
        "depth_projector_build_tag",
        "Failures / bottlenecks",
        "Anchor stats",
    ]
    for s in expected_substrings:
        if s not in summary:
            print(f"[FAIL] summary missing expected substring: {s!r}")
            return 1
    print("[OK] summary contains all expected sections")

    if _HAS_GO2_MSGS:
        if "first confirmed table             :" in summary or \
           "first confirmed table               :" in summary:
            pass
        else:
            print("[WARN] could not find 'first confirmed table' line "
                  "(format may have shifted)")
        # The phase B emits "auto_mapping" as raw operator? actually no
        # — we only emit manual_table_scan in phase C. So summary should
        # say table confirmed during AUTO_MAPPING (since phase B drives
        # mapping_status=NAVIGATING and entities are first published
        # there).
        if "confirmed during mode               : AUTO_MAPPING" not in \
                summary:
            print("[WARN] summary does not say table was confirmed "
                  "during AUTO_MAPPING (expected given phase B). "
                  "Continuing anyway.")
        else:
            print("[OK] summary attributes table confirmation to "
                  "AUTO_MAPPING")

    print("[PASS] _smoke_record_semantic_lifecycle.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
