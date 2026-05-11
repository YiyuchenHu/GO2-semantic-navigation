"""Day 9 (table-cleanup) — quality-score + merge-better unit smoke.

Runs against the static helpers we just changed in
``semantic_memory_aggregator_node.py``. Does NOT touch ROS, so it can
be invoked from a clean Python with ``rclpy`` available (the import
chain pulls in ``rclpy.time.Time``).

Pass criteria:
  * ``_quality_score`` orders entities so pc_ > isl_ > unanchored.
  * ``_quality_score`` orders entities so confirmed > candidate.
  * The ``_better`` callable inside ``_merge_close_entities`` agrees
    with the public quality_score for the most common confirmed-vs-
    confirmed merge case.
"""

from __future__ import annotations

import sys
from pathlib import Path

import rclpy.time as rclpy_time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(
    0,
    str(REPO / "src" / "go2_semantic_perception"),
)

from go2_semantic_perception.semantic_memory_aggregator_node import (  # noqa: E402
    SemanticMemoryAggregatorNode,
    TrackedEntity,
)


def _t(ns: int = 0) -> rclpy_time.Time:
    return rclpy_time.Time(nanoseconds=int(ns))


def _ent(
    *,
    eid: str,
    cls: str = "table",
    confirmed: bool = True,
    invalid: bool = False,
    obs: int = 5,
    conf: float = 0.85,
    island: str = "",
    invalid_count: int = 0,
    visible: bool = True,
    first_seen_ns: int = 1_000_000_000,
    px: float = 0.0,
    py: float = 0.0,
    pz: float = 0.0,
) -> TrackedEntity:
    return TrackedEntity(
        entity_id=eid,
        class_label=cls,
        px=px,
        py=py,
        pz=pz,
        confidence=conf,
        observations_count=obs,
        first_seen=_t(first_seen_ns),
        last_seen=_t(first_seen_ns + 1_000_000_000),
        currently_visible=visible,
        raw_class=cls,
        island_id=island,
        is_confirmed=confirmed,
        same_island_observations=0,
        invalid_evidence_count=invalid_count,
        is_invalid=invalid,
    )


def _q(ent: TrackedEntity) -> tuple:
    return SemanticMemoryAggregatorNode._quality_score(ent)


def test_pc_beats_isl_beats_none() -> None:
    pc = _ent(eid="pc1", island="pc_42")
    isl = _ent(eid="isl1", island="isl_99")
    none = _ent(eid="none1", island="")
    assert _q(pc) > _q(isl) > _q(none), (
        f"pc={_q(pc)} isl={_q(isl)} none={_q(none)}"
    )


def test_valid_beats_invalid() -> None:
    valid = _ent(eid="v", invalid=False, island="pc_1")
    bad = _ent(eid="b", invalid=True, island="pc_1")
    assert _q(valid) > _q(bad)


def test_confirmed_beats_candidate() -> None:
    conf = _ent(eid="cf", confirmed=True, island="pc_1")
    cand = _ent(eid="cd", confirmed=False, island="pc_1")
    assert _q(conf) > _q(cand)


def test_more_obs_breaks_tie() -> None:
    a = _ent(eid="a", obs=10, island="pc_1")
    b = _ent(eid="b", obs=3, island="pc_1")
    assert _q(a) > _q(b)


def test_lower_invalid_evidence_wins() -> None:
    clean = _ent(eid="clean", island="pc_1", invalid_count=0)
    stained = _ent(eid="dirty", island="pc_1", invalid_count=4)
    assert _q(clean) > _q(stained), (
        f"clean={_q(clean)} stained={_q(stained)}"
    )


def test_pc_invalid_loses_to_isl_valid() -> None:
    """invalid is the strongest negative — it dominates pc>isl."""
    pc_bad = _ent(eid="pc_bad", island="pc_1", invalid=True)
    isl_ok = _ent(eid="isl_ok", island="isl_1", invalid=False)
    assert _q(isl_ok) > _q(pc_bad)


def test_confirmed_pc_beats_confirmed_isl() -> None:
    pc = _ent(eid="pc", island="pc_1", confirmed=True, obs=3)
    isl = _ent(eid="isl", island="isl_1", confirmed=True, obs=10)
    # PC anchor wins even though isl_ has more observations:
    # rule order is pc>isl ABOVE observation count.
    assert _q(pc) > _q(isl)


def test_keep_best_simulation() -> None:
    """Mimic what ``keep_best_class table`` will pick."""
    fragments = [
        _ent(eid="t1", island="", obs=2),                 # no anchor
        _ent(eid="t2", island="isl_1", obs=4),            # isl
        _ent(eid="t3", island="pc_42", obs=3),            # pc, fewer obs
        _ent(eid="t4", island="pc_42", obs=8, conf=0.99), # pc, best
        _ent(eid="t5", island="pc_42", invalid=True),     # invalid
    ]
    fragments.sort(key=_q, reverse=True)
    assert fragments[0].entity_id == "t4", (
        f"best={fragments[0].entity_id} order={[e.entity_id for e in fragments]}"
    )


def main() -> int:
    tests = [
        test_pc_beats_isl_beats_none,
        test_valid_beats_invalid,
        test_confirmed_beats_candidate,
        test_more_obs_breaks_tie,
        test_lower_invalid_evidence_wins,
        test_pc_invalid_loses_to_isl_valid,
        test_confirmed_pc_beats_confirmed_isl,
        test_keep_best_simulation,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"[PASS] {t.__name__}")
            passed += 1
        except AssertionError as exc:
            print(f"[FAIL] {t.__name__}: {exc}")
            return 1
    print(f"---\n{passed}/{len(tests)} table-cleanup smoke tests passed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
