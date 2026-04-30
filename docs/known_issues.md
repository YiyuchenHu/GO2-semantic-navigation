# Known issues — Week 1 snapshot

This file is the running log of "things that are broken or weird and
we haven't fixed yet, on purpose or otherwise". Update it as you
land Week 2+ work.

---

## 1. ❗ Legacy `perception_node` crashes on import (numpy ABI mismatch)

**Status**: known-broken since Day 5; **not fixed**, deprecated.

**Symptom**: launching `chair_perception.launch.py` crashes the
`perception_node` and `object_localizer_3d_node` processes with:

```
ImportError: A module that was compiled using NumPy 1.x cannot be
run in NumPy 2.4.4
```

immediately after the `from cv_bridge import CvBridge` import.

**Root cause**: Ubuntu 24.04's default Python ships numpy 2.x, but
the `ros-jazzy-cv-bridge` apt package was compiled against numpy 1.x.
Python doesn't allow loading a 1.x C extension into a 2.x runtime;
import fails before any user code runs. The `perception_node`
imports cv_bridge directly, so it dies on startup. The newer
`yoloe_detector_node` (Day 5) imports cv_bridge too but it tolerates
both numpy versions — apparently because ultralytics' own torch
loader was compiled against numpy 2.x and "wins" the resolution
order.

**Why we are not fixing it**: the chair-only `perception_node` is
the **deprecated** Phase 1 detector. Day 5's `yoloe_detector_node`
is its open-vocabulary replacement and publishes a richer, standard-
vision_msgs payload. From Day 6 onwards the project consumes
`/detections` (YOLOE) instead of `/perception/detections_2d`
(legacy). We keep the legacy launch in the tree for archeology /
fallback only.

**Workarounds if you actually need the legacy pipeline**:

1. `pip install --user --break-system-packages 'numpy<2'` —
   downgrades the user-site numpy. The cleanest fix; risks
   conflicting with ultralytics on the same Python.
2. Source-build `ros-jazzy-cv-bridge` against numpy 2.x. Painful;
   only worth it if you also need the legacy stack to coexist with
   YOLOE in the same process.
3. Run the legacy chair pipeline inside the Isaac Sim conda env
   (which ships its own numpy 1.x). Untested by us; documented
   here for completeness.

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
