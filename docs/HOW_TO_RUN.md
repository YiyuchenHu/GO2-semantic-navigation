# Go2 Semantic Navigation — 端到端调用手册

> 写给「几周后回来的自己」：以下为**可直接复制**的命令模板；launch 文件名已与仓库 `src/go2_bringup_sim/launch/` **静态核对**存在。  
> **未在本机执行**仿真或 `colcon build`；若包未编译或 `install/setup.bash` 过期，请先自行构建后再运行。

---

## A. 环境准备

### A.1 每个新终端都要做的 sourcing

```bash
# 若开过 conda，建议先退出（避免 Python/rclpy 版本错乱）
conda deactivate 2>/dev/null || true

cd /home/yiyuchenhu/Desktop/2026spring/2026spring/CINQ389/GO2/GO2-semantic-navigation

# 方式一：项目脚本（推荐，含 PROJECT_ROOT / ROS_DISTRO / workspace overlay）
source scripts/dev_env.sh

# 方式二手动（等价骨架）
# source /opt/ros/jazzy/setup.bash
# source install/setup.bash
```

### A.2 进程清理 — 干掉所有残留（推荐每次重启前都跑一次）

```bash
cd /home/yiyuchenhu/Desktop/2026spring/2026spring/CINQ389/GO2/GO2-semantic-navigation
bash scripts/kill_all.sh                  # 默认：保留 Isaac Sim、保留 RViz
bash scripts/kill_all.sh --include-rviz   # 顺便关掉 RViz
bash scripts/kill_all.sh --all            # 连 Isaac Sim 一起关（少用）
bash scripts/kill_all.sh --dry-run        # 只看会杀谁、不真杀
```

> 为什么需要它？普通的 `Ctrl+C` 或 `pkill -f "ros2 launch"` 只杀 launch
> **父**进程；像 `static_transform_publisher`、Nav2 的 `component_container_isolated`、
> `slam_toolbox` 这些 C++ 子进程经常变成孤儿继续跑。下次再 `ros2 launch` 时，
> 新旧两套同时存在，会出现：3 份相同的 TF publisher、2 个 SLAM 抢 `map->odom`、
> Nav2 controller 报 `Unable to transform robot pose into global plan's frame`、
> `frontier_explorer` 颜色闪烁、Go2 长时间"思考"后才行动等一系列怪现象。
> `kill_all.sh` 会先 SIGTERM、再 SIGKILL，最后用 `ps` 校验确认无残留。

### A.3 启动方式 — 推荐用 `launch_safe.sh` 包装

```bash
# 取代直接 `ros2 launch ...`：
bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py
bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py \
     abort_cooldown_sec:=10.0
```

它做的事：
1. 用 `setsid` 把 launch 放进**独立 session**，所有子孙都共享同一个 PGID。
2. Ctrl+C 时先优雅 SIGINT 整个进程组让 launch 跑 `OnShutdown`，等最多 4 秒；
3. 然后 SIGKILL 整个组，**包括所有已脱离的孤儿**——不会再有"上次没杀干净"。

直接用 `ros2 launch` 没有这层保护。养成习惯就用 `launch_safe.sh`。

### A.4 ROS 日志目录权限（曾导致 `ros2` CLI 异常）

```bash
sudo chown -R "$USER:$USER" ~/.ros
```

---

## B. 标准启动序列（T1–T5）

下列顺序与 `day8_two_phase.launch.py` 文件头说明一致（约 L27–36）。

### T1 — 仿真（Isaac + warehouse）

```bash
cd /home/yiyuchenhu/Desktop/2026spring/2026spring/CINQ389/GO2/GO2-semantic-navigation
bash scripts/run_warehouse_ros2.sh
```

**等待信号（肉眼/日志）**：仿真窗口出现 warehouse；ROS bridge 侧开始有传感器相关 topic（依你机器而定）。

**验证（另开终端，已 source）**：

```bash
ros2 topic info /clock
ros2 topic info /lidar/points
```

---

### T2 — 静态 TF + LaserScan（`tf_and_scan`，替代旧文档里的 chair_perception 作为主路径）

```bash
# 推荐（Ctrl+C 时干净退出）
bash scripts/launch_safe.sh go2_bringup_sim tf_and_scan.launch.py

# 或者裸跑（要记得用 kill_all.sh 清理）
ros2 launch go2_bringup_sim tf_and_scan.launch.py
```

**等待信号**：`/scan` 有 publisher。

**验证**：

```bash
ros2 topic info /scan | grep -E 'Publisher|Type'
ros2 topic hz /scan
```

**备选（旧式全感知 bringup）**：若仍使用旧教程命令，仓库也存在：

```bash
ros2 launch go2_bringup_sim chair_perception.launch.py
```

---

### T3 — Nav2 + SLAM

```bash
bash scripts/launch_safe.sh go2_bringup_sim nav2.launch.py slam:=True
```

**等待信号**：终端出现 Nav2 lifecycle / `Managed nodes are active` 一类就绪日志（具体措辞依 Nav2 版本）。

**验证**：

```bash
ros2 topic info /map
ros2 topic info /global_costmap/costmap
ros2 node list | grep -E 'slam_toolbox|controller_server|bt_navigator' || true
```

---

### T4 — Day 8 全栈（两阶段推荐）

```bash
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py

# 想缩短"思考时间"（每次 Nav2 ABORT 后的软冷却），降默认 15s 到 10s：
bash scripts/launch_safe.sh go2_bringup_sim day8_two_phase.launch.py \
     abort_cooldown_sec:=10.0
```

**验证（每个名字应只出现 1 次）**：

```bash
ros2 node list | sort | uniq -c | sort -rn | head
ros2 node list | grep -E 'mapping_explorer|frontier_explorer|nl_parser|task_coordinator|yoloe|semantic_memory'
ros2 service list | grep get_frontiers
ros2 topic info /frontier_markers --verbose | grep "Publisher count"   # 应为 1
ros2 topic info /semantic_map/markers --verbose | grep "Publisher count" # 应为 1
```

> 如果 `ros2 node list | uniq -c` 里某个节点 ≥ 2，说明 `kill_all.sh` 没跑、
> 或者上次没用 `launch_safe.sh` 留了孤儿。先 `bash scripts/kill_all.sh` 再重启。

**_legacy 单阶段 Day 8（target_class 驱动 + FSM EXPLORE）** 仍可用：

```bash
ros2 launch go2_bringup_sim day8.launch.py target_class:=chair
```

---

### T5 — RViz

```bash
cd /home/yiyuchenhu/Desktop/2026spring/2026spring/CINQ389/GO2/GO2-semantic-navigation
bash scripts/run_rviz.sh
```

脚本会为 RViz 打开 `use_sim_time:=true`（见 `scripts/run_rviz.sh` 约 L35–48），避免 sim time 与 RViz 默认 wall time 不一致。

**Semantic memory — RViz Marker 话题**（`semantic_memory_aggregator`）：

| Topic | 含义 |
| --- | --- |
| `/semantic_map/markers` | 全部 **confirmed** landmark（visible + remembered），**旧版合并 topic**，兼容老配置 |
| `/semantic_map/markers_visible` | **confirmed** 且 `currently_visible=true` |
| `/semantic_map/markers_remembered` | **confirmed** 且 `currently_visible=false` |
| `/semantic_map/debug_markers` | candidate、invalid、缺 anchor 的 confirmed 等调试流 |

参数（默认开启拆分）：`publish_split_visibility_markers`（默认 True）、`visible_markers_topic`、`remembered_markers_topic`。若为 False，仅发布 `/semantic_map/markers`，不发布两个拆分 topic。

**Demo 建议**：

- 录制「记住 table/person」：RViz 只开 **`/semantic_map/markers_remembered`**。
- 调试实时感知：开 **`/semantic_map/markers_visible`** + **`/semantic_map/debug_markers`**。
- 全量调试：可同时开四个；**`go2_semantic_nav.rviz`** 中 legacy 合并层默认关，拆分 **visible / remembered** 默认开。

**Semantic memory marker topics (English)**：

- **`/semantic_map/markers`** — all **confirmed** landmarks (visible + remembered), legacy combined topic.
- **`/semantic_map/markers_visible`** — confirmed landmarks currently **in view** (`currently_visible=true`).
- **`/semantic_map/markers_remembered`** — confirmed landmarks **remembered but not currently visible**.
- **`/semantic_map/debug_markers`** — candidates, invalid rejects, anchor-missing confirmed, etc.

**Demo tips**：

- Recording “semantic map memory” — show only **`/semantic_map/markers_remembered`**.
- Debugging live perception — **`/semantic_map/markers_visible`** + **`/semantic_map/debug_markers`**.
- Full stack — enable all topics as needed (`go2_semantic_nav.rviz` defaults: split layers on, legacy combined off).

---

## C. 端到端 Demo 测试流程

### 模式 1 — Sanity Check（spawn 附近能看见桌子；椅子可能需_exploration）

默认仓库布局下桌子更可能在初始相机视野内；**请以 RViz `/semantic_map/objects` 为准**。

⚠️ **必须等 `/mapping/status` 持续输出 `DONE` 后才能发 NL 命令**——`mapping_explorer` 和 `task_coordinator` 都持有 Nav2 ActionClient，双客户端会抢 `/navigate_to_pose` action server。

**命令序列**：完成 **T1→T5** 后：

```bash
# Phase A：观察 mapping 状态（latched 字符串）
ros2 topic echo /mapping/status --once

# Phase B：自然语言（示例）
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'go to chair'"
```

**RViz 期望现象**：

- `/map` 扩张、`Frontiers`（若配置启用）与语义 Marker 逐渐出现；
- `semantic_map` 实体列表中**出现目标类**（如 `chair`/`table`，以检测为准）。

**`/task/status`（`std_msgs/String`）**：成功路径典型片段为  
`... → CHECK_MEMORY → TARGET_FOUND → PLAN_APPROACH_GOAL → NAVIGATE_TO_GOAL → VERIFY_TARGET → ARRIVED`  
（定义见 `task_coordinator_node.py` 状态枚举约 L62–79）。

**ARRIVED 判定**：

- `ros2 topic echo /task/status --once` 含 `ARRIVED`；
- 机器人停在目标附近且语义上合理（结合 RViz 机器人与 entity marker）。

---

### 模式 2 — 「真」语义导航（物体先在视野外）

#### 调整椅子位置

编辑 `sim/warehouse_scene.py`：

- **位置常量**：`CHAIR_XYZ = (3.5, -3.5, 0.0)`（约 **L101**）
- 按需改成更远或转角后方；保存后 **重启 Isaac Sim** 使场景重建生效。

#### 重启栈

按 **A.2** 清理后重新执行 **T1→T5**。

#### 测试命令

仍建议 Phase A 跑完后发：

```bash
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'go to chair'"
```

#### 关于「抢占 mapping_explorer」的说明（务必读）

**当前 `day8_two_phase.launch.py` 没有在代码层实现**：「`mapping_explorer` 正在 NAVIGATING 时，一旦 `semantic_memory` 出现 chair 就自动停掉 Phase A 并切换 Approach」。  
可靠演示路径是：

1. 等待 **`/mapping/status` 为 `DONE`**（或 operator 发送 `/mapping/control`，字符串 `abort`/`restart` 语义见 `mapping_explorer_node.py` 约 L276–301）；  
2. 再发 `/user_command`。

若必须在记忆中尚无实体时导航：`nl_parser` 发出的 `SemanticTask.requires_search` 为 **True**（`nl_parser_node.py` L353），可能触发 `task_coordinator` 的 **EXPLORE**，与 `mapping_explorer` **同时占用 Nav2**——不推荐作为正式 demo 路径。

**若你要复现「单一 FSM 下边探索边找椅子」叙事**：改用 **`day8.launch.py target_class:=chair`**（`task_coordinator` 内置 EXPLORE），而非两阶段并行。

---

### 模式 3 — 失败场景（不存在实例的类别）

默认 **`day8_two_phase.launch.py` 的 `nl_known_classes` 不含 microwave**（约 L255–261）。要做「解析出 microwave → 再失败」需覆盖参数，例如：

```bash
ros2 launch go2_bringup_sim day8_two_phase.launch.py \
  nl_known_classes:='chair,table,desk,box,microwave'
```

否则 NL 层会拒绝识别 `microwave`，而不是触发 `task_coordinator` 的 `FAILED` 状态。

然后：

```bash
ros2 topic pub --once /user_command std_msgs/msg/String "data: 'go to microwave'"
```

**期望 `/task/status`**：在记忆中找不到实例时会进入 **`EXPLORE`**（若 frontier 最终为空）并由 coordinator 失败处理；典型失败串包含 **`environment fully explored`** 与 **`microwave`**（见 `task_coordinator_node.py` L533–537）。

> 若 **未** 把 `microwave` 加进 `nl_known_classes`，则可能只在 **`/nl_parser/feedback`** 看到低置信/拒绝提示，`task_coordinator` 仍在 **IDLE** —— 这也是一种「失败」，但是 **NL 层拒绝**而非导航 FSM 失败。

---

### 模式 4 — Phase A 没扫到目标？主动绕墙巡检（不重启栈）

`mapping_explorer` 的 frontier 评分倾向"未知空间最大化"，所以经常机器人停在屋子中央就 `DONE`，YOLOE 没机会扫到贴墙的物体。这时不必重启整套栈，可以临时让 Go2 沿墙绕一圈，让 YOLOE 把 chair / table / box 扫进 `semantic_memory_aggregator`。

```bash
# 看看默认会走哪几个点（不需要 source ROS）
python3 scripts/perimeter_patrol.py --dry-run

# 真正派发：CW 顺时针 4 个角落 + 每个角落原地转 360°，约 6–8 分钟
python3 scripts/perimeter_patrol.py

# 只想去 chair 所在的 SE 角（map(7.5, 0.5)）做一次快测
python3 scripts/perimeter_patrol.py --se-only

# 8 个 waypoint（角 + 边中点），不在每个点旋转，更快但扫描密度低
python3 scripts/perimeter_patrol.py --dense --no-spin

# 反方向（CCW），inset 扩大到 2 m 避开膨胀代价
python3 scripts/perimeter_patrol.py --ccw --inset 2.0
```

waypoint 计算基于仓库尺寸 10 m × 10 m + `world_to_map` 静态偏置 (-4, -4)，所以 map 帧可走范围约 `x∈[-1,9], y∈[-1,9]`，inset 1.5 m 后角点是 `(0.5,0.5) (7.5,0.5) (7.5,7.5) (0.5,7.5)`。chair 的 map 坐标 `(7.5, 0.5)` 正好就是 SE 角。

它直接调 `/navigate_to_pose`，所以**会抢占** `mapping_explorer` / `task_coordinator` 当前的 Nav2 goal——只在 `/mapping/status == DONE` 之后跑（或者在你确实想干预的时候跑）。Ctrl+C 会取消当前 goal、停下机器人。

跑完用 `ros2 topic echo /semantic_map/objects --once` 看 chair / table 是否进了 entity 列表，再发 `/user_command "go to chair"`。

---

## D. 故障排查速查表

| 现象 | 可能原因 | 诊断命令 |
|------|----------|----------|
| `/mapping/status` 长期非 DONE | frontier 不绝、Nav2 反复 ABORT、TF 未就绪 | `ros2 topic echo /mapping/status --once`；看 mapping_explorer 日志；`ros2 topic echo /tf_static --once` |
| DONE 但 `/semantic_map/objects` 空 | 机器人从未观测到物体；YOLOE 未运行或类别不匹配 | `ros2 topic echo /semantic_map/objects --once`；`ros2 topic hz /detections` |
| 发 NL 后 coordinator 无反应 | `nl_parser` 置信度不足；或 `/semantic_task/request` 无订阅 | `ros2 topic echo /nl_parser/feedback --once`；`ros2 topic info /semantic_task/request` |
| 导航中途 ABORTED | sim TF 滞后、代价地图、目标在障碍内 | Nav2 日志；`ros2 topic echo /task/status --once` |
| `/global_costmap/costmap` 异常空 | QoS/生命周期、上游 `/map` 或 SLAM 未起 | `ros2 topic info -v /global_costmap/costmap`；`ros2 topic hz /map` |
| `/scan` 频率异常 | pointcloud_to_laserscan 断流、TF 问题、重复节点 | `ros2 topic hz /scan`；`ros2 node list \| sort \| uniq -d` |

---

## E. 关键 `ros2` 命令速查

```bash
# 实体列表
ros2 topic echo /semantic_map/objects --once

# 检测频率
ros2 topic hz /detections

# Mapping explorer 常用参数（名称以节点声明为准）
ros2 param describe /mapping_explorer
ros2 param set /mapping_explorer max_consecutive_aborts 8
ros2 param set /mapping_explorer done_confirm_sec 8.0
ros2 param set /mapping_explorer done_fast true
# 缩短"思考时间"——每次 Nav2 ABORT 后该 frontier 的软冷却（默认 15 s）。
# 调到 5–10 s 可让 Go2 几乎立刻重试，但要小心 Nav2 真不可达时会更频繁打日志。
ros2 param set /mapping_explorer abort_cooldown_sec 10.0

# 取消当前导航目标（取消 action server 上活跃 goal）
ros2 action cancel /navigate_to_pose

# 键盘遥控（如需）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 强制停止 Phase A（映射 explorer）
ros2 topic pub --once /mapping/control std_msgs/msg/String "data: 'abort'"

# 主动巡检 / 找物体（绕墙一圈，不需要重启栈）
python3 scripts/perimeter_patrol.py --dry-run     # 看一眼计划
python3 scripts/perimeter_patrol.py               # 走起
python3 scripts/perimeter_patrol.py --se-only     # 只去 chair 所在的 SE 角
```

---

## F. 录 Demo 视频（OBS Studio）

Linux 常见启动方式（取决于安装来源）：

```bash
obs
# 或
flatpak run com.obsproject.Studio
```

**建议录制内容**：

1. Isaac Sim 视窗（机器人运动与环境）。  
2. RViz（`/map`、语义 markers、可选 frontiers）。  
3. 终端：`/mapping/status` echo、`/user_command` pub、`/task/status` echo。

**布局**：1080p 单屏可采用「左上 Sim + 右 RViz + 底部长终端」；双屏可将 Sim 与 RViz 分屏。

---

## 附：仓库内相关 Launch 文件索引（已核对存在）

`chair_execute_goal.launch.py`、`chair_perception.launch.py`、`chair_goto_goal.launch.py`、`chair_semantic_memory.launch.py`、`chair_with_search.launch.py`、`day6.launch.py`、`day7.launch.py`、`day8.launch.py`、`day8_two_phase.launch.py`、`mapping.launch.py`、`nav2.launch.py`、`sim_semantic_nav.launch.py`、`tf_and_scan.launch.py`、`yoloe.launch.py`

路径：`src/go2_bringup_sim/launch/`
