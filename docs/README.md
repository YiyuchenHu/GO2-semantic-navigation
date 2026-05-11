# Documentation index

This `docs/` directory uses **two parallel naming schemes** because
the project went through a re-phasing partway through. Both are
preserved.

## Scheme 1: Phase 0-5 (original chair-only MVP)

The first MVP plan was an end-to-end chair-finding pipeline with
seven phases. Documents `phase{0,1,2,3,3a,3b,4,5}_status.md` cover
that work; their README lives at the top of the repo
(`../README.md` "Phase status" section).

| File | Topic |
|------|-------|
| `phase0_status.md` | Isaac Sim platform bring-up, ROS 2 bridge |
| `phase1_status.md` | Chair-only YOLOv11l-seg perception |
| `phase2_status.md` | Object tracker + persistent semantic entities |
| `phase3a_status.md` | Target selection + approach-ring goal sampling |
| `phase3b_status.md` | P-controller goal execution + arrival verifier |
| `phase3_status.md` | (Day 3 SLAM, **misnamed** — see Scheme 2 below) |
| `phase4_status.md` | Search / reacquisition rotate-in-place sweep |
| `phase5_status.md` | Locomotion backend abstraction (scaffold) |

> ⚠ `phase3_status.md` was created during the Day-ladder rework
> for SLAM and accidentally collides with the Phase 3A/3B naming.
> Treat it as Scheme-2 content (Day 3 SLAM); leave it where it is
> for now to avoid breaking external links.

## Scheme 2: Day-ladder (acceptance-driven)

The second pass replaced the phase taxonomy with a Day 1, Day 2,
... acceptance ladder, where each Day is a one-axis hard test of
one stack layer. This is the **active** scheme for new work.

| File | Topic |
|------|-------|
| `phase3_status.md` | **Day 3 — `slam_toolbox` 2D SLAM** (filename is misleading) |
| `day4_nav2_status.md` | Day 4 — Nav2 stack (slam_toolbox or AMCL backend) |
| `day5_yoloe_status.md` | Day 5 — YOLOE open-vocab detection |

## Cross-cutting docs

These apply across both schemes:

| File | Topic |
|------|-------|
| `known_issues.md` | Running log of bugs / weird-state items per Week |
| `decisions.md` | ADR-style design rationale (slam_toolbox vs RTAB, YOLOE vs GroundingDINO, ...) |

## Acceptance scripts

Each Day has a matching `scripts/check_dayN.sh` automated check.
The scripts are NOT under `docs/`; they live in `scripts/`. Cross-
referenced from the corresponding `docs/dayN_*.md`.

| Day | Script |
|-----|--------|
| 1-2 (sim platform) | `scripts/check_day12.sh` |
| 3 (SLAM) | `scripts/check_day3.sh` |
| 4 (Nav2) | `scripts/check_day4.sh` |
| 5 (YOLOE) | `scripts/check_day5.sh` |

---

## Where work goes after Week 1

Week 2 starts with **Day 6 — depth reprojection + semantic memory**.
Documentation will continue under the Day-ladder scheme:

* `day6_semantic_memory.md` (planned)
* `day7_target_selection.md` (planned)
* ...

The Phase 0-5 documents stay frozen as historical record of the
chair-only MVP. The active stack moves to Day-ladder docs.
