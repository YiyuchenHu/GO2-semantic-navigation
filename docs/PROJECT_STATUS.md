# Go2 Semantic Navigation — 项目现状快照

> 本文档仅依据仓库内**可查证的源码与文档**整理；未在仓库中找到的文件会标明「未确认」。  
> **注意**：用户提到的 `docs/day7_completion.md` 在当前仓库中**不存在**；Day 7 相关内容以 `docs/day7_target_navigation_status.md`、`docs/known_issues.md` 等为替代依据。

---

## 1. 项目目录树（`src/` 下 ROS 2 包）

| 包名 | 职责（结合 `package.xml` 描述与代码实际用途） |
|------|-----------------------------------------------|
| `go2_msgs` | 自定义消息/服务（含 `SemanticTask`、`GetFrontiers.srv` 等）。 |
| `go2_bringup_sim` | Isaac Sim 侧 bringup：launch、`nav2`/SLAM 配置、RViz 配置等。 |
| `go2_navigation` | Frontier 检测（`frontier_explorer_node`）、自主扫图驱动（`mapping_explorer_node`）；`package.xml` 描述仍偏 Day 7 措辞，但代码已包含 Day 8 节点。 |
| `go2_perception` | YOLOE 检测（如 `yoloe_detector_node`）。 |
| `go2_semantic_perception` | 深度投影、语义记忆聚合、目标选择、接近目标规划（Day 6/7 栈）。 |
| `go2_semantic_memory` | 早期/并行语义地图包（持久实体等）；与 `semantic_memory_aggregator_node` 并存，具体运行栈以 launch 为准。 |
| `go2_object_localization` | RGB-D 3D 物体定位相关节点。 |
| `go2_task_coordinator` | 顶层 FSM：`task_coordinator_node`（含 Day 8 `EXPLORE`）。 |
| `go2_command_interface` | 命令解析：`command_parser_node`，YAML 规则 + `/user_command`→`/semantic_task/request`。 |
| `go2_nl_parser` | 轻量 NL→`SemanticTask`：`nl_parser_node`（regex + fuzzy）。 |
| `go2_safety` | 安全监控节点（MVP）。 |
| `go2_debug_tools` | 调试标记与运行时日志辅助。 |

---

## 2. 逐 Day 完成度（Day 1 → Day 12）

下列状态中：✅ 完成 / 🟡 部分完成 / ❌ 未开始。Day 1–7 以仓库内阶段性文档与 launch 为主证据；**细粒度「每一天」在仓库中并非总有单独文件**，缺失处标明。

### Day 1–3（仿真与基础 ROS 联通）

- **状态**：🟡（按 `docs/phase0_status.md` … `docs/phase2_status.md` 推断为历史里程碑；无单一「Day1.md」）
- **证据**：`docs/phase0_status.md`、`docs/phase1_status.md`、`docs/phase2_status.md`
- **缺失**：与当前 Day 8 栈对齐的一页式「Phase→Day」映射未单独维护。

### Day 4（Nav2）

- **状态**：✅（文档与 launch 齐全）
- **证据**：`docs/day4_nav2_status.md`；`src/go2_bringup_sim/launch/nav2.launch.py`（`slam` 默认 `True`，约 L106–114）
- **缺失**：仿真 LiDAR 卡顿仍是已知限制（见 `docs/known_issues.md`、`docs/day4_nav2_status.md`）。

### Day 5（YOLOE）

- **状态**：✅（文档 + 节点）
- **证据**：`docs/day5_yoloe_status.md`；`src/go2_perception/go2_perception/yoloe_detector_node.py`（由 `day8_two_phase.launch.py` 引用，约 L337–354）

### Day 6（深度 + 语义记忆）

- **状态**：✅（代码与 launch 接入）
- **证据**：`docs/day6_semantic_memory_status.md`；`src/go2_semantic_perception/go2_semantic_perception/depth_projector_node.py`、`semantic_memory_aggregator_node.py`

### Day 7（目标选择 + 接近规划 + coordinator 串联）

- **状态**：✅（功能验证通过）
- **证据**：`docs/day7_target_navigation_status.md`（例如 L31–41 描述 selector/planner 逻辑）；`src/go2_bringup_sim/launch/day7.launch.py`
- **备注**：功能验证通过（见 `day7_target_navigation_status.md`，包含 `check_day7.sh` 17 PASS / 0 FAIL、Day 6.5 mean_err=0.27 m PASS、Go2 端到端走到 desk 旁停下的记录）；完整 `day7_completion.md` 待补（不阻塞下游）。

### Day 8（Frontier + 自主扫图 + 两阶段 NL）

- **状态**：🟡（实现与 launch/脚本齐全，端到端强依赖仿真与单机环境稳定性）
- **证据**：
  - **`frontier_explorer_node`**：`src/go2_navigation/go2_navigation/frontier_explorer_node.py`（由 `day8.launch.py`、`day8_two_phase.launch.py` 启动）
  - **`mapping_explorer_node`**：`src/go2_navigation/go2_navigation/mapping_explorer_node.py`（声明参数例如 `global_frame`、`max_consecutive_aborts`、`done_confirm_sec` 等，约 L142–171）
  - **`task_coordinator` 含 `EXPLORE`**：`src/go2_task_coordinator/go2_task_coordinator/task_coordinator_node.py` 中 `class FsmState`（L62–79）含 `EXPLORE = "EXPLORE"`；EXPLORE 驱动逻辑见同文件 L436–441、L497–539 等
  - **`day8_two_phase.launch.py` 节点列表**（源码顺序）：`yoloe_detector`、`depth_projector`、`semantic_memory_aggregator`、`frontier_explorer`、`mapping_explorer`、`target_selector`、`approach_goal_planner`、`task_coordinator`、`nl_parser`（约 L337–527）
  - **`day8.launch.py`**：`src/go2_bringup_sim/launch/day8.launch.py`（Include Day 7 + `frontier_explorer` + `task_coordinator`，文档头 L1–38）
  - **`check_day8.sh`**：**存在**，`scripts/check_day8.sh`；脚本头注释定义 **4 门**验收（L9–27）：FRONTIER UNIT、FRONTIER CONSUMPTION、AUTONOMOUS DISCOVERY（人工）、EXHAUSTED→FAILED
  - **补充**：`scripts/check_day8_two_phase.sh` 覆盖两阶段 Phase A/B/NLP/E2E（与「4 门」是**另一套**脚本）

### Day 9（状态机决策合并）

- **状态**：✅
- **证据**：决策逻辑已合并到 `task_coordinator_node` FSM（含 `EXPLORE`），而非独立包，符合简化设计。
- **缺失**：无。

### Day 10（文本解析 + E2E demo）

- **状态**：🟡
- **证据**：
  - **`nl_parser_node`**：`src/go2_nl_parser/go2_nl_parser/nl_parser_node.py`（`/user_command`→`/semantic_task/request`，模块文档 L39–43）
  - **`command_parser_node`**：`src/go2_command_interface/go2_command_interface/command_parser_node.py`（正则模式 L27–31；订阅 `/user_command`、发布 `/semantic_task/request`，L69–70）
  - **同义词**：NL 侧默认表 `_DEFAULT_SYNONYMS`（例 `nl_parser_node.py` 约 L84 起）；Command 侧 `src/go2_command_interface/config/semantic_targets.yaml`（`chair`/`table` 等 aliases）
- **备注**：功能就绪，存在 `nl_parser`（推荐）和 `command_parser` 两条并行链路，后续收敛为单一 parser；运行时只起一条。

### Day 11–12（调参 + 视频）

- **状态**：❌ / 🟡（依赖个人录制与调参记录，仓库无强制门禁）
- **证据**：未确认集中 checklist。
- **缺失**：标准化录制脚本、Acceptance 录像门禁。

---

### Day 8 / Day 9–10 专题核对（摘要）

| 项 | 结论 | 证据位置 |
|----|------|----------|
| `frontier_explorer_node` | ✅ 已实现 | `src/go2_navigation/go2_navigation/frontier_explorer_node.py` |
| `mapping_explorer_node` | ✅ 已实现 | `src/go2_navigation/go2_navigation/mapping_explorer_node.py` |
| `task_coordinator` 是否有 `EXPLORE` | ✅ 有 | `task_coordinator_node.py` L62–79 |
| `day8_two_phase.launch.py` 节点 | 见上 9 个 Node | `day8_two_phase.launch.py` L337–527 |
| `check_day8.sh` | ✅ 存在；**4 门** | `scripts/check_day8.sh` L9–27 |
| `FsmState` 全列表 | IDLE, PARSE_COMMAND, CHECK_MEMORY, TARGET_FOUND, TARGET_NOT_FOUND, EXPLORE, SEARCH(deprecated), PLAN_APPROACH_GOAL, NAVIGATE_TO_GOAL, VERIFY_TARGET, ARRIVED, FAILED, SAFETY_STOP | `task_coordinator_node.py` L62–79 |
| `command_parser_node` | ✅ YAML + 正则 | `command_parser_node.py` + `config/semantic_targets.yaml` |
| `/user_command`→`/semantic_task/request` | ✅ 至少两条链路：**nl_parser**、**command_parser**（勿重复启动） | `nl_parser_node.py`；`command_parser_node.py` L69–70 |

---

## 3. 当前实际推进位置（一句话）

**Day 1-7 全部完成（功能验证通过，部分 completion 文档待补）；Day 8 两阶段架构代码就位（`day8_two_phase` + `mapping_explorer`），等待端到端真机跑通验证；Day 9 决策合并到 `task_coordinator` FSM 已实现；Day 10 NL 解析就绪（`nl_parser` 主链路，`command_parser` 并存待收敛）。当前唯一阻塞：在干净环境下完整跑一轮 Phase A → DONE → NL 命令 → ARRIVED。**

---

## 4. 距离「理想 demo」还差什么（具体项）

下列为**可落地的缺口**，避免泛泛「调参」：

1. **Hard blacklist / empty frontier → DONE**：`mapping_explorer_node.py` 使用 `_MAX_ATTEMPTS_PER_FRONTIER = 3` 等（约 L115–117）；若现实中「坏 frontier」较多，存在 **过早 DONE** 风险，需要在真实跑法中验证。
2. **`map_max_aborts` launch 默认覆盖**：`day8_two_phase.launch.py` 中 `map_max_aborts` 默认 `"4"`（约 L234–239），会覆盖节点内 `_DEFAULT_MAX_CONSECUTIVE_ABORTS = 8`（`mapping_explorer_node.py` L133）；行为以 launch **4** 为准，可能与「代码默认值」直觉不一致。TODO：建议后续在 `known_issues.md` 记录「参数双源」风险（launch arg 默认值覆盖节点 `declare_parameter` 默认值），目前已知双源参数：`map_max_aborts`。
3. **`approach_goal_planner` 与 `/navigation/status`**：`task_coordinator` 在 APPROACH/NAVIGATE 路径上仍可能存在历史记载的衔接假设；两阶段通过「Phase A 填满记忆 + 少走 EXPLORE」**规避**部分问题，但未从结构上删除风险。
4. **YOLOE 类名与目标类**：历史记录中出现过仿真椅子被标为 `stool` 等（`day7_target_navigation_status.md`）；demo 应用 **`ros2 topic echo /semantic_map/objects` 对齐真实 `class_label`**。

---

## 5. 已知风险与不一致（对照 Day 7 文档中的 Nav2/感知坑）

因 **`docs/day7_completion.md` 不存在**，此处对照 `docs/day7_target_navigation_status.md`、`docs/day4_nav2_status.md`、`docs/known_issues.md` 中的可操作条目。

| 风险项 | 现状（基于代码默认） | 证据 | 缓解 |
|--------|----------------------|------|------|
| `mapping_explorer` 与 `task_coordinator` 并发 Nav2 ActionClient | Day 8 共存模式，依赖 FSM 互斥（Phase A DONE 后才发 NL） | `day8_two_phase.launch.py` + `task_coordinator` FSM | `HOW_TO_RUN.md` §C 模式 1 第一行已加警告 |
| `target_frame=map` 仍为 launch 默认 | ✅ 是（两阶段） | `day8_two_phase.launch.py` L54–57 | 保持 launch 默认 |
| `tf_fallback_latest_on_time_error` | ✅ 默认 `true` | `day8_two_phase.launch.py` L97–99 | 保持默认 |
| `slam:=True` + `allow_unknown` | ✅ `nav2.launch.py` 默认 `slam` 为 `True`（L106–114）；`nav2_params.yaml` 含 `allow_unknown: true`（约 L279–281） | 上文路径 | 保持 Day 8 sim 路径 |
| `/global_costmap/costmap` 空/不匹配 | 文档仍建议用 `ros2 topic info -v` 核对 **TRANSIENT_LOCAL** 与发布端 | `docs/day7_target_navigation_status.md` L394–400 | 先查 QoS / lifecycle，再清理重启 |
| 「恢复手势」是否仍有效 | **未在仓库中检出单一脚本化 recovery**；实操仍以 Nav2 lifecycle、重启代价地图节点、全栈清理（见 `scripts/_debug_day8_cleanup_relaunch.sh`）为主 | `scripts/_debug_day8_cleanup_relaunch.sh` L47–66 | 优先用全栈清理脚本 |
| Isaac LiDAR 低速/卡顿 | ⚠️ 仍为已知 sim 限制 | `docs/known_issues.md`（LiDAR ~4 Hz 等） | 录 demo 前重启仿真并减少重复节点 |

---

## 6. 仓库里是否有 chair？端到端推荐目标类

- **场景脚本**：`sim/warehouse_scene.py` 同时构建 **table** 与 **chair**（`TABLE_XYZ`/`CHAIR_XYZ`，约 L100–101；`build_table`/`build_chair`，约 L419–459）。
- **注释**：椅子默认在 spawn **视野外**一侧，用于驱动探索叙事（约 L88–96）。
- **结论**：端到端示例目标类应优先使用 **`chair`**（若运行时 `class_label` 非 `chair`，以 `/semantic_map/objects` 为准）。

---

## 文档修订记录

- 生成方式：静态扫描仓库（未执行 `colcon build` / 未启动仿真）。
