# Go2 Semantic Navigation — Project Status Snapshot

> This document is compiled solely from source code and documentation verifiable within the repository; files not found in the repository are marked "unconfirmed".  
> **Note**: `docs/day7_completion.md` mentioned by the user does **not exist** in the current repository; Day 7-related content is referenced from `docs/day7_target_navigation_status.md`, `docs/known_issues.md`, etc.

---

## 1. Project Directory Tree (ROS 2 packages under `src/`)

| Package | Role (based on `package.xml` description and actual code usage) |
|------|-----------------------------------------------|
| `go2_msgs` | Custom messages/services (including `SemanticTask`, `GetFrontiers.srv`, etc.). |
| `go2_bringup_sim` | Isaac Sim-side bringup: launch files, `nav2`/SLAM configs, RViz config, etc. |
| `go2_navigation` | Frontier detection (`frontier_explorer_node`), autonomous mapping driver (`mapping_explorer_node`); `package.xml` description still leans toward Day 7 wording, but the code already includes Day 8 nodes. |
| `go2_perception` | YOLOE detection (e.g., `yoloe_detector_node`). |
| `go2_semantic_perception` | Depth projection, semantic memory aggregation, target selection, approach goal planning (Day 6/7 stack). |
| `go2_semantic_memory` | Early/parallel semantic map package (persistent entities, etc.); coexists with `semantic_memory_aggregator_node`; which one runs depends on the launch file. |
| `go2_object_localization` | RGB-D 3D object localization nodes. |
| `go2_task_coordinator` | Top-level FSM: `task_coordinator_node` (including Day 8 `EXPLORE`). |
| `go2_command_interface` | Command parsing: `command_parser_node`, YAML rules + `/user_command`→`/semantic_task/request`. |
| `go2_nl_parser` | Lightweight NL→`SemanticTask`: `nl_parser_node` (regex + fuzzy). |
| `go2_safety` | Safety monitoring node (MVP). |
| `go2_debug_tools` | Debug marker and runtime logging utilities. |

---

## 2. Per-Day Completion Status (Day 1 → Day 12)

Status legend: ✅ complete / 🟡 partial / ❌ not started. Day 1–7 evidence is primarily drawn from milestone documents and launch files in the repository; **fine-grained "per-day" files do not always exist in the repository** — missing items are noted.

### Day 1–3 (Simulation and Basic ROS Connectivity)

- **Status**: 🟡 (inferred from `docs/phase0_status.md` … `docs/phase2_status.md` as historical milestones; no single "Day1.md" exists)
- **Evidence**: `docs/phase0_status.md`, `docs/phase1_status.md`, `docs/phase2_status.md`
- **Missing**: A single-page "Phase→Day" mapping aligned with the current Day 8 stack is not separately maintained.

### Day 4 (Nav2)

- **Status**: ✅ (documentation and launch files complete)
- **Evidence**: `docs/day4_nav2_status.md`; `src/go2_bringup_sim/launch/nav2.launch.py` (`slam` defaults to `True`, ~L106–114)
- **Missing**: Simulated LiDAR lag remains a known limitation (see `docs/known_issues.md`, `docs/day4_nav2_status.md`).

### Day 5 (YOLOE)

- **Status**: ✅ (documentation + node)
- **Evidence**: `docs/day5_yoloe_status.md`; `src/go2_perception/go2_perception/yoloe_detector_node.py` (referenced by `day8_two_phase.launch.py`, ~L337–354)

### Day 6 (Depth + Semantic Memory)

- **Status**: ✅ (code and launch integrated)
- **Evidence**: `docs/day6_semantic_memory_status.md`; `src/go2_semantic_perception/go2_semantic_perception/depth_projector_node.py`, `semantic_memory_aggregator_node.py`

### Day 7 (Target Selection + Approach Planning + Coordinator Pipeline)

- **Status**: ✅ (feature verified)
- **Evidence**: `docs/day7_target_navigation_status.md` (e.g., L31–41 describes selector/planner logic); `src/go2_bringup_sim/launch/day7.launch.py`
- **Note**: Feature verified (see `day7_target_navigation_status.md`, which records `check_day7.sh` 17 PASS / 0 FAIL, Day 6.5 mean_err=0.27 m PASS, and Go2 end-to-end walking to and stopping beside a desk); a complete `day7_completion.md` remains pending (non-blocking for downstream).

### Day 8 (Frontier + Autonomous Mapping + Two-Phase NL)

- **Status**: 🟡 (implementation, launch files, and scripts are complete; end-to-end quality depends heavily on simulation and single-machine environment stability)
- **Evidence**:
  - **`frontier_explorer_node`**: `src/go2_navigation/go2_navigation/frontier_explorer_node.py` (launched by `day8.launch.py` and `day8_two_phase.launch.py`)
  - **`mapping_explorer_node`**: `src/go2_navigation/go2_navigation/mapping_explorer_node.py` (declares parameters such as `global_frame`, `max_consecutive_aborts`, `done_confirm_sec`, etc., ~L142–171)
  - **`task_coordinator` including `EXPLORE`**: `class FsmState` in `src/go2_task_coordinator/go2_task_coordinator/task_coordinator_node.py` (L62–79) contains `EXPLORE = "EXPLORE"`; EXPLORE driving logic is in the same file at L436–441, L497–539, etc.
  - **`day8_two_phase.launch.py` node list** (in source code order): `yoloe_detector`, `depth_projector`, `semantic_memory_aggregator`, `frontier_explorer`, `mapping_explorer`, `target_selector`, `approach_goal_planner`, `task_coordinator`, `nl_parser` (~L337–527)
  - **`day8.launch.py`**: `src/go2_bringup_sim/launch/day8.launch.py` (includes Day 7 + `frontier_explorer` + `task_coordinator`, file header L1–38)
  - **`check_day8.sh`**: **exists** at `scripts/check_day8.sh`; the script header defines **4 acceptance gates** (L9–27): FRONTIER UNIT, FRONTIER CONSUMPTION, AUTONOMOUS DISCOVERY (manual), EXHAUSTED→FAILED
  - **Additionally**: `scripts/check_day8_two_phase.sh` covers two-phase Phase A/B/NLP/E2E (a separate script from the "4 gates")

### Day 9 (FSM Decision Merging)

- **Status**: ✅
- **Evidence**: Decision logic has been merged into the `task_coordinator_node` FSM (including `EXPLORE`) rather than a separate package, consistent with the simplified design.
- **Missing**: None.

### Day 10 (Text Parsing + E2E Demo)

- **Status**: 🟡
- **Evidence**:
  - **`nl_parser_node`**: `src/go2_nl_parser/go2_nl_parser/nl_parser_node.py` (`/user_command`→`/semantic_task/request`, module docstring L39–43)
  - **`command_parser_node`**: `src/go2_command_interface/go2_command_interface/command_parser_node.py` (regex pattern L27–31; subscribes to `/user_command`, publishes `/semantic_task/request`, L69–70)
  - **Synonyms**: NL-side default table `_DEFAULT_SYNONYMS` (e.g., `nl_parser_node.py` ~L84); Command-side `src/go2_command_interface/config/semantic_targets.yaml` (`chair`/`table` aliases, etc.)
- **Note**: Feature-ready; two parallel pipelines exist — `nl_parser` (recommended) and `command_parser` — to be converged to a single parser; only one is launched at runtime.

### Day 11–12 (Parameter Tuning + Video)

- **Status**: ❌ / 🟡 (depends on individual recording and tuning logs; no mandatory acceptance gate in the repository)
- **Evidence**: No centralized checklist confirmed.
- **Missing**: Standardized recording script, video acceptance gate.

---

### Day 8 / Day 9–10 Focused Verification (Summary)

| Item | Conclusion | Evidence Location |
|----|------|----------|
| `frontier_explorer_node` | ✅ implemented | `src/go2_navigation/go2_navigation/frontier_explorer_node.py` |
| `mapping_explorer_node` | ✅ implemented | `src/go2_navigation/go2_navigation/mapping_explorer_node.py` |
| `task_coordinator` has `EXPLORE` | ✅ yes | `task_coordinator_node.py` L62–79 |
| `day8_two_phase.launch.py` nodes | see 9 nodes above | `day8_two_phase.launch.py` L337–527 |
| `check_day8.sh` | ✅ exists; **4 gates** | `scripts/check_day8.sh` L9–27 |
| `FsmState` full list | IDLE, PARSE_COMMAND, CHECK_MEMORY, TARGET_FOUND, TARGET_NOT_FOUND, EXPLORE, SEARCH(deprecated), PLAN_APPROACH_GOAL, NAVIGATE_TO_GOAL, VERIFY_TARGET, ARRIVED, FAILED, SAFETY_STOP | `task_coordinator_node.py` L62–79 |
| `command_parser_node` | ✅ YAML + regex | `command_parser_node.py` + `config/semantic_targets.yaml` |
| `/user_command`→`/semantic_task/request` | ✅ at least two pipelines: **nl_parser**, **command_parser** (do not launch both simultaneously) | `nl_parser_node.py`; `command_parser_node.py` L69–70 |

---

## 3. Current Actual Progress (One-Sentence Summary)

**Days 1–7 fully complete (features verified; some completion docs pending); Day 8 two-phase architecture code is in place (`day8_two_phase` + `mapping_explorer`), pending end-to-end hardware verification; Day 9 decision merging into `task_coordinator` FSM is implemented; Day 10 NL parsing is ready (`nl_parser` as primary, `command_parser` coexists pending convergence). The sole current blocker: running a complete Phase A → DONE → NL command → ARRIVED cycle in a clean environment.**

---

## 4. Gaps Remaining Before "Ideal Demo" (Concrete Items)

The following are actionable gaps, as opposed to vague "tuning" items:

1. **Hard blacklist / empty frontier → DONE**: `mapping_explorer_node.py` uses `_MAX_ATTEMPTS_PER_FRONTIER = 3`, etc. (~L115–117); if many "bad frontiers" occur in practice, there is a risk of premature DONE — must be verified in a real run.
2. **`map_max_aborts` launch default override**: `map_max_aborts` defaults to `"4"` (~L234–239) in `day8_two_phase.launch.py`, overriding the node-level `_DEFAULT_MAX_CONSECUTIVE_ABORTS = 8` (`mapping_explorer_node.py` L133); runtime behavior follows the launch value of 4, which may differ from the "code default" intuition. TODO: recommend documenting the "dual-source parameter" risk in `known_issues.md` (launch arg default overriding node `declare_parameter` default); currently known dual-source parameter: `map_max_aborts`.
3. **`approach_goal_planner` and `/navigation/status`**: `task_coordinator` may still carry legacy assumptions about the APPROACH/NAVIGATE handoff; the two-phase approach mitigates some issues by "filling memory in Phase A + minimizing EXPLORE", but the risk is not structurally eliminated.
4. **YOLOE class name vs. target class**: past logs show the simulated chair labeled as `stool`, etc. (`day7_target_navigation_status.md`); demos should use **`ros2 topic echo /semantic_map/objects`** to confirm the actual `class_label`.

---

## 5. Known Risks and Inconsistencies (Cross-referenced Against Day 7 Nav2/Perception Issues)

Since **`docs/day7_completion.md` does not exist**, the following is cross-referenced against `docs/day7_target_navigation_status.md`, `docs/day4_nav2_status.md`, and `docs/known_issues.md`.

| Risk Item | Current State (based on code defaults) | Evidence | Mitigation |
|--------|----------------------|------|------|
| `mapping_explorer` and `task_coordinator` concurrent Nav2 ActionClient | Day 8 coexistence mode; relies on FSM mutual exclusion (NL sent only after Phase A DONE) | `day8_two_phase.launch.py` + `task_coordinator` FSM | Warning added to `HOW_TO_RUN.md` §C Mode 1 |
| `target_frame=map` is still the launch default | ✅ yes (two-phase) | `day8_two_phase.launch.py` L54–57 | Keep launch default |
| `tf_fallback_latest_on_time_error` | ✅ default `true` | `day8_two_phase.launch.py` L97–99 | Keep default |
| `slam:=True` + `allow_unknown` | ✅ `nav2.launch.py` defaults `slam` to `True` (L106–114); `nav2_params.yaml` contains `allow_unknown: true` (~L279–281) | paths above | Keep Day 8 sim path |
| `/global_costmap/costmap` empty/mismatched | Documentation still recommends `ros2 topic info -v` to verify TRANSIENT_LOCAL and the publisher | `docs/day7_target_navigation_status.md` L394–400 | Check QoS/lifecycle first, then clean up and restart |
| Whether "recovery gestures" are still effective | No single scripted recovery found in the repository; practical recovery relies on Nav2 lifecycle management, restarting the costmap node, and full-stack cleanup (see `scripts/_debug_day8_cleanup_relaunch.sh`) | `scripts/_debug_day8_cleanup_relaunch.sh` L47–66 | Prefer the full-stack cleanup script |
| Isaac LiDAR low rate / lag | ⚠️ still a known simulation limitation | `docs/known_issues.md` (LiDAR ~4 Hz, etc.) | Restart simulation and reduce duplicate nodes before recording demo |

---

## 6. Is There a Chair in the Repository? Recommended End-to-End Target Class

- **Scene script**: `sim/warehouse_scene.py` builds both **table** and **chair** (`TABLE_XYZ`/`CHAIR_XYZ`, ~L100–101; `build_table`/`build_chair`, ~L419–459).
- **Note**: The chair is by default placed outside the spawn field of view, intended to drive the exploration narrative (~L88–96).
- **Conclusion**: The recommended end-to-end demo target class is **`chair`** (if the runtime `class_label` is not `chair`, defer to `/semantic_map/objects`).

---

## Document Revision History

- Generated by: static scan of the repository (colcon build not executed; simulation not started).
