# Known issues — Week 1 snapshot

This file is the running log of "things that are broken or weird and
we haven't fixed yet, on purpose or otherwise". Update it as you
land Week 2+ work.

---

## 1. ✓ FIXED: Legacy `perception_node` cv_bridge import was broken on numpy 2.x

**Status**: **fixed** by `scripts/install_ml_deps.sh` +
`scripts/pip-constraints.txt`.  Was a symptom of cv_bridge's numpy
1.x ABI being shadowed by a user-site numpy 2.x install.

**Original symptom**: launching `chair_perception.launch.py` crashed
the `perception_node` and `object_localizer_3d_node` processes with:

```
ImportError: A module that was compiled using NumPy 1.x cannot be
run in NumPy 2.4.4
```

immediately after `from cv_bridge import CvBridge`.

**Root cause**: pip in `~/.local/lib/python3.12/site-packages/`
pulled in numpy 2.x as a transitive dep of (e.g.) `ultralytics`,
`opencv-python`, or `torch`.  Python's `sys.path` resolution puts
the user-site numpy ahead of the apt-installed numpy 1.26 that
`ros-jazzy-cv-bridge`'s C extension was compiled against.  Loading
the 1.x extension into a 2.x runtime is forbidden; import dies
before any user code runs.

**The fix** (`scripts/install_ml_deps.sh`):

`scripts/pip-constraints.txt` declares a hard cap
`numpy<2,>=1.26`. The installer script invokes pip with
`-c scripts/pip-constraints.txt` (or `PIP_CONSTRAINT=...` env
var), so pip is forbidden from upgrading numpy past 1.26 even
when a freshly-released ultralytics version requests it.  Re-run
`bash scripts/install_ml_deps.sh --check` after any
`pip install`/`pip install -U` to confirm the cap is still in
effect.

**Operational consequence**: any new pip install of an ML library
in this project must go through `install_ml_deps.sh` (or at least
respect the constraints file) until ROS Jazzy ships a cv_bridge
built against numpy 2.x.  Bumping the constraint is a deliberate
act, gated on verifying cv_bridge import in a clean env.

**Why this hit us specifically and not every Jazzy user**: most
people don't have a YOLOE / torch / numpy stack in `~/.local/`.
The combination of (Ubuntu 24.04 PEP 668) + (system Python) +
(pip --user --break-system-packages for ML deps) is what creates
the shadowing.  Conda envs that vendor their own everything (Isaac
Sim's bundled Python) don't see the issue.

---

## 2. ⚠ YOLOE auto-downloads a 572 MB TorchScript backbone on first run

**Status**: by design, not fixable.

**Symptom**: the first time `yoloe_detector_node` starts, the
`Loading YOLOE: model='yoloe-11s-seg.pt' ...` log line is followed
by a `Downloading https://github.com/ultralytics/assets/.../mobileclip_blt.ts`
progress bar that takes ~12 seconds on a fast connection. The
node's `YOLOE ready` log line only appears after the download
completes.

**Root cause**: YOLOE uses MobileCLIP-Blt as its text-prompt
encoder. The model isn't bundled in the `yoloe-11s-seg.pt` weights
file; it's auto-fetched on first use of `model.get_text_pe()`,
which our `YoloeBackend.set_classes()` calls during init.

**What this means for you**:

* On a clean machine, your first `ros2 launch ... yoloe.launch.py`
  blocks for 10-30 seconds in the import. That's not a hang.
* If your network blocks `github.com` releases (some campus VPNs
  do), pre-download manually:

  ```bash
  python3 -c "from ultralytics import YOLOE; YOLOE('yoloe-11s-seg.pt')"
  ```

  Run that once with internet access; the file lands in your CWD
  (and is `.gitignore`-protected by the `*.ts` glob).

* The downloaded `mobileclip_blt.ts` is **not** under
  `~/.cache/yoloe/` like you might expect. Ultralytics drops it in
  the working directory of the process that triggered the
  download. Move it to a stable location (e.g.
  `~/.config/Ultralytics/`) if you want it shared across runs.

---

## 3. ⚠ Isaac Sim RTX LiDAR runs at ~4 Hz with occasional 14 s stalls

**Status**: sim-side limitation, **not fixed**; documented in Day 4.

**Symptom**: `ros2 topic hz /lidar/points` reports an average rate
of ~4 Hz instead of the configured 10 Hz, with occasional gaps of
several seconds where no message arrives. Downstream:

* `slam_toolbox` keeps running but its `map → odom` TF stutters.
* Day 4 Nav2's `controller_server` sometimes aborts a `follow_path`
  with `Lookup would require extrapolation into the future. Requested
  time X but the latest data is at time X` — Go2 walks in spurts.

**Root cause**: the RTX LiDAR ScanBuffer shares the GPU with the
RGB and depth cameras. Under contention the LiDAR's per-frame
budget is starved, the buffer publishes partial scans, and the
ROS bridge ends up emitting fewer full clouds than the
`OS1_REV6_32ch10hz1024res` config requested.

**Why we accept it**: real Go2 hardware uses a Livox MID-360 with
its own dedicated firmware running at a hardware-clocked 10 Hz.
The contention failure mode is sim-only.

**Mitigations applied at Nav2 level** (already in
`config/nav2/nav2_params.yaml`):

* `bt_navigator.transform_tolerance: 5.0` (was 0.1)
* `collision_monitor.source_timeout: 15.0` (was 1.0)
* Plus all Day 4's other tolerance bumps. See
  `docs/day4_nav2_status.md` "Pitfalls hit during bring-up".

If you need sim-side reliability for a demo, lower Isaac Sim's
camera resolution in `sim/run_go2_warehouse_ros2.py` and the
LiDAR will steal less GPU time.

---

## 4. ⚠ slam_toolbox map keeps refining as Go2 drives (mapping mode, not localization)

**Status**: by design for the Day 4 sim configuration; documented.

**Symptom**: the `/map` published by Nav2's slam_toolbox backend
slowly grows / refines as Go2 explores. The static `warehouse_v1.pgm`
map saved in `maps/` is **not** the one Nav2's costmap uses when
launched with `slam:=True` (default).

**Why**: we use `slam_toolbox` in **mapping mode** as the Day 4
localization backend rather than AMCL. AMCL was unstable on our
~4 Hz Lidar (see Day 4 status doc, layer ②). slam_toolbox in
mapping mode is robust to the same input but, by definition, keeps
extending its map.

**Implications**:

* Don't expect bit-exact `/map` content across runs.
* Costmap obstacle layer is built from live `/scan`, which is fine.
* For a frozen reference map, switch to AMCL backend
  (`ros2 launch go2_bringup_sim nav2.launch.py slam:=False`) — but
  expect Day 4's AMCL pitfalls to come back. Or freeze
  slam_toolbox's serialized map (Day 6+, not done yet).

---

## 5. 📌 Architectural: chair-only pipeline is being deprecated

**Status**: tracking, not a bug.

The repo currently contains **two parallel perception stacks**:

| Stack | Topic prefix | Detector | Status |
|-------|--------------|----------|--------|
| Phase 1 (legacy) | `/perception/...` | YOLOv11l-seg, chair-only | **Deprecated**; broken since Day 5 (numpy ABI, see #1) |
| Day 5 (current) | `/detections...` | YOLOE-11s-seg, open-vocab | **Active** |

Day 6+ work consumes the Day 5 stack only. The Phase 1 launches
(`chair_perception.launch.py`, etc.) stay in the tree for now so
older Phase 2-4 launches that depend on them still parse, but they
will be removed once Day 7+ rebuilds the semantic memory and goal
stack on top of `/detections`.

If you're chasing `git log --grep "Phase"` history, expect to find
this duality preserved up through Week 2 Day 6, after which the
legacy launches get pruned.

---

## 6. 📌 No safety / e-stop layer wired

`go2_safety/safety_monitor_node` exists in the tree (Phase scaffolding)
but is not in any active launch. `/cmd_vel` flows directly from
Nav2's `collision_monitor` into the sim's `SubTwist` node. For real-
robot deployment a safety supervisor MUST be inserted in this path.

---

## 7. ⚠️ Isaac Sim 5.1 cold shader compile on RTX 5090 (sm_120) is brutal

**Status**: workaround in place, not a bug per se but biggest
operational footgun in Day 6/7 sessions.

**Symptoms**: first ever launch of `run_warehouse_ros2.sh` (or first
launch after the shader cache is wiped) sits silently at
`Priming 12 physics frames before main loop ...` for 5–25 minutes,
then either:
* (lucky) emits `[run_ros2] alive sim_time=...` and runs at full speed
  forever, OR
* (unlucky) is SIGKILL'd by mutter's stuck-window watchdog (when
  GUI mode) OR deadlocks indefinitely with GPU util ~3 % (when
  headless), OR
* (very unlucky) crashes Hydra RTX with no stack trace and no
  minidump from the embedded Breakpad reporter.

**Root cause**: NVIDIA driver 580.126.09 on RTX 5090 (sm_120) is
brand-new silicon. Every Vulkan / RTX raytracing pipeline that Kit
boots needs a fresh shader compile, and the compile pass for
**multiple render products at once** (RGB camera + depth camera +
RTX LiDAR's Ouster OS1) appears to deadlock in the driver under
specific orderings. Kit's logger stops draining (spdlog blocks on
GPU wait), so the operator sees nothing and assumes a hang.

**Workarounds in place**:

1. `sim/run_go2_warehouse_ros2.py` primes physics with
   `world.step(render=False)` (12 frames in <1 s). The first
   `render=True` only happens once we've entered the main loop,
   minimising the period the shader compile blocks the spdlog
   sink. Per-frame heartbeat + a banner make compile vs. deadlock
   distinguishable.
2. **Always run headless first** (`bash scripts/run_warehouse_ros2.sh
   --headless`). The mutter ping watchdog only kills GUI processes;
   a headless Kit will compile uninterrupted (just slowly).
3. Once the cache is built, **back it up immediately**:
   ```
   cp -a ~/isaacsim_5.1_backup/kit/cache/Kit ~/isaacsim_5.1_backup/kit/cache/Kit.warm
   cp -a ~/.nv/ComputeCache                  ~/.nv/ComputeCache.warm
   ```
   Future cold-boot pain is now reversible — just restore from the
   `.warm` copies before launching.
4. **NEVER run `rm -rf` on `~/isaacsim_5.1_backup/kit/cache/Kit` or
   `~/.nv/ComputeCache`**. The previous "clear-corrupt-cache"
   instinct cost 25 minutes of cold compile in this session.
5. If the sim deadlocks repeatedly, **reboot the system**, not just
   `kill -9`. SIGKILL leaks GPU memory pool / CUDA context / Vulkan
   device queues in the driver, and only a kernel module reload
   (or `nvidia-smi --gpu-reset` with all GPU processes stopped)
   recovers them.
6. For early bring-up, **`--no-lidar --rgb-resolution 640x480
   --depth-resolution 320x240`** drops the cold-compile footprint
   to one or two RT pipelines, which seems to dodge the deadlock
   reliably. Re-add LiDAR once cache is warm.

**Long-term fix**: NVIDIA driver 5xx → 580 series is supposed to land
sm_120 stability fixes on a Q3 2026 timeline; until then the
operational rules above are the project's working answer. Running
**Isaac Sim 4.5 instead of 5.1** would also work (the Hydra RTX
deadlock is 5.1-only) but reverts other Day-1 setup work.

**Diagnostic commands** for future incidents:

```
# Sim alive but no output? Watch the cache grow vs. CPU time grow:
watch -n 5 'ps -p <pid> -o pid,pcpu,etime,cputime; \
            du -sh ~/isaacsim_5.1_backup/kit/cache/Kit ~/.nv/ComputeCache'

# Cache size growing OR cputime growing  → still compiling, wait
# Cache size frozen 60s+ AND cputime frozen → real deadlock, kill + reboot

# Confirm GPU isn't OOM / hung:
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv
journalctl -k --since '10 min ago' | grep -iE 'NVRM|Xid'
```

---

## 8. ⚠️ depth_projector projects entities far outside their actual sim location

**Status**: blocking full end-to-end Nav2 goal-reaching, but does
NOT affect Day 7's algorithm verification. Fix in next session.

**Symptom**: with `day7.launch.py target_frame:=odom` and the sim's
default warehouse (table spawned at origin, room ±5 m enclosed),
`/semantic_map/objects` reports `desk_003.pose_map = (-1.89, -9.18,
+0.89)` and `estimated_distance ≈ 9.4 m`. Both are wrong:
* the table's actual sim position is near (0, 0, ~0.5);
* `y = -9.18` is outside the room walls;
* `z = +0.89` is plausible (table top height) but tied to the
  same projection bug.

Downstream consequence: `approach_goal_planner` ring-samples 16
points around the wrong entity position, every candidate falls in
the costmap's unknown/lethal region (outside the room), and the
planner correctly refuses to send a NavigateToPose goal — Go2
never moves, even though the perception → memory → selector →
planner chain is mechanically working.

**RViz cue**: the LiDAR PointCloud2 (cyan rings around base_link)
clearly shows the room interior, but the "Semantic memory (Day 6)"
cylinder and the "Approach candidates (Day 7)" red ring sit far
away, on or beyond the wall. Both are in the `odom` frame; the
visual offset *is* the bug — the entity's pose data is wrong.

**Suspected root causes** (one likely confirmed):

1. **camera optical-frame TF is rotated wrong**. Strongest evidence
   so far: with base_link at odom (0.023, 0, 0.131), yaw = -10.69°,
   and the sim table actually positioned in front of the robot
   (~5-6 m forward in `+x_baselink`), the math says the table
   should land at odom ~(5.9, -1.1, 0.6). Instead `depth_projector`
   reports it at odom (-1.89, -9.18, 0.89) — same magnitude
   distance but rotated almost 90° clockwise. Mask-edge depth
   noise can't produce a 90° angular error; only a wrong rotation
   transform can. The candidate is the static TF in
   `chair_perception.launch.py`:
   ```
   static_transform_publisher --x 0.30 --y 0.00 --z 0.12 \
       --qx 0.5 --qy -0.5 --qz 0.5 --qw -0.5 \
       --frame-id base_link --child-frame-id camera_link
   ```
   The `--qw -0.5` (negative) puts the quaternion in the opposite
   hemisphere from the standard "REP-103 optical convention"
   value. ROS canonicalises quaternions to the `qw >= 0`
   hemisphere internally, but mixing both forms in a TF chain
   has been known to flip rotation sign on at least one
   intermediate frame, producing exactly this kind of
   90°-cardinal-axis error. **Likely fix**: replace with
   `--qx -0.5 --qy 0.5 --qz -0.5 --qw 0.5` and re-launch.
2. **Mask-median depth bias** (less likely given the angular
   nature of the error): `depth_projector_node` takes the
   median depth over the YOLOE mask region. If the mask
   bleeds onto far walls / ceiling, median jumps and projection
   lands behind the wall. Easy to test by switching to bbox-
   centre depth instead of mask-median.
3. **Depth unit mismatch**: sim publishes
   `/camera/depth/image_rect_raw` as 32FC1 in metres
   (per Day-1 contract). If `depth_projector` accidentally
   treats it as 16UC1 mm, distances scale by 1000×; here
   the magnitude bias is ~1.7× not 1000×, so this isn't it.

**Diagnostic recipe**:

```
# A. Look at a single Detection3D's 3D position right after projection
ros2 topic echo --once /detections_3d | head -50
# Compare bbox.center.position to base_link pose:
timeout 3 ros2 run tf2_ros tf2_echo odom base_link

# B. Check the camera optical frame chain
timeout 3 ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# C. Look at the YOLOE mask vs RGB to see if mask edges bleed
ros2 topic echo --once /detections | head -100   # see InstanceMaskArray
# Or just visually: RViz "Image (YOLOE detections)" + zoom on chair —
# does the red mask hug the chair tightly or leak onto the floor?
```

**Workaround for Day 7 acceptance**: do not block on this. The Day 7
algorithm layer (target_selector + approach_goal_planner) works
correctly given the perception output it sees, and Go2 *does*
visibly start driving toward the goal — it just doesn't reach it
because the entity is pruned mid-traverse (see #9 below).

### Update (2026-04-30, afternoon): partial fix landed

The 90° rotation half of this bug (caused by `--qw +0.5` in
`chair_perception.launch.py`'s `_OPTICAL_FRAME_ARGS`) is fixed in
the `fix(day7): camera optical TF + approach planner marker spam`
commit. After the fix, desk_001 projects to odom (8.4, -2.1) —
direction is correct (forward + slight right of base_link, matching
where the table actually is in the sim scene), but the magnitude is
~9 m versus the geometric ground truth of ~5.7 m, i.e. a remaining
~1.7× distance bias. That residual is the mask-edge-bleed half of
the issue; candidate 2 above is still open.

---

## 9. ⚠️ Entity drops out of /semantic_map/objects mid-traverse

**Status**: blocks the "Go2 actually reaches the chair" finale.
Day 7 algorithm layer is unaffected (verified earlier this session
that target_selector + approach_goal_planner + Nav2 NavigateToPose
all behave correctly when an entity is present).

**Symptom**: launch the full Day 7 stack with sim, nav2 active,
chair_perception, and `day7.launch.py target_class:=desk
target_frame:=odom`. target_selector picks `desk_001` immediately,
approach_goal_planner sends a NavigateToPose action goal, and Go2
visibly starts driving toward the desk's approach pose. **After
~5–10 seconds of motion** Go2 stops:

```
ros2 topic echo --once /semantic_map/objects
  entities: []                              ← empty

ros2 topic echo --once /target/selected
  entity_id: ''
  ranking_reasons:
    - "no entities with class='desk' and confidence>=0.3"
```

The selector goes empty → approach_goal_planner sees an empty
target → it cancels the in-flight Nav2 goal → Nav2 stops emitting
/cmd_vel → Go2 halts.

**Root cause**: a chain of three perception-layer issues amplifying
each other:

1. As Go2 turns toward the goal-pose yaw (-45° in the test run),
   the camera FOV swings off the desk; `currently_visible` flips
   to False after `visibility_timeout_sec=2.0` s of no detection.
2. Once invisible, the entity's confidence decays at
   `confidence_decay_rate=0.05` per sec (`exp(-0.05·age_s)`),
   crossing the selector's `min_confidence=0.30` floor in ~24 s.
   By itself this would only matter if Go2 takes >24 s to turn
   back, which is borderline but survivable.
3. **The accelerator**: depth_projector mask-edge bleed (see #8)
   makes each detection's projected XY drift several decimetres
   between frames as the mask shape varies. When the drift exceeds
   `nms_radius_m=0.3 m`, the aggregator creates a NEW entity_id
   (e.g. `desk_002`, `desk_003`, …) instead of merging into
   `desk_001`. The new entity starts at `confidence=step_up=0.15`,
   which is BELOW the 0.30 selector floor. Each new id starts
   from scratch, so confidence never accumulates, and after a few
   short-lived tracks the registry is empty.

**Quick band-aid (next session)**: tune the aggregator parameters
to be more tolerant of projection jitter:

```bash
ros2 param set /semantic_memory_aggregator nms_radius_m 0.8
ros2 param set /semantic_memory_aggregator confidence_decay_rate 0.02
ros2 param set /semantic_memory_aggregator visibility_timeout_sec 5.0
ros2 param set /target_selector min_confidence 0.20
```

These keep the entity registered through Go2's traverse on the
existing perception output. They do NOT fix the underlying
mask-edge bleed; they just hide it.

**Real fix**: close out #8 candidate 2 (mask-edge bleed) so the
3D position drift stays well under nms_radius_m even on the
default 0.3 m. Then keep the original tighter parameters, which
are more robust against tracking ghost objects in clutter.

**Diagnostic recipe**: while Go2 is mid-traverse, watch the
entity registry live:

```bash
# Keep this running and watch entities[] grow / shrink:
ros2 topic echo /semantic_map/objects | grep -E 'entity_id|confidence|currently_visible|observations_count'
```

If you see `entity_id` cycling through `desk_001`, `desk_002`,
`desk_003` while `observations_count` stays small (<5), that's
diagnostic of the aggregator failing to merge across projection
jitter.
