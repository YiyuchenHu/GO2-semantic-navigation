#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Run the Go2 warehouse with full ROS 2 sensor publishing (Isaac Sim 5.1 + Jazzy).
#
# This script builds the warehouse FRESH on a brand-new stage (using
# sim/warehouse_scene.py) — it does NOT open a pre-saved USD. That's on
# purpose: Isaac Sim 5.1 on RTX 5090 has a reproducible crash inside
# libomni.graph.image.core.plugin.so whenever UsdContext::reopenUsd() runs
# on our saved warehouse USD. Building the scene live completely avoids
# that codepath.
#
# After the scene is up, the script:
#   1. Attaches an RGB-D Camera prim and an IMU sensor under /World/Go2/base.
#   2. Builds an OmniGraph that uses isaacsim.ros2.bridge to publish:
#        /clock                          rosgraph_msgs/Clock
#        /camera/color/image_raw         sensor_msgs/Image    (RGB, rgb8)
#        /camera/depth/image_rect_raw    sensor_msgs/Image    (depth, 32FC1, METERS)
#        /camera/color/camera_info       sensor_msgs/CameraInfo
#        /camera/depth/camera_info       sensor_msgs/CameraInfo  (same K as color)
#        /imu/data                       sensor_msgs/Imu
#        /odom                           nav_msgs/Odometry
#        /tf                             tf2_msgs/TFMessage   (odom -> base_link)
#      and to subscribe to:
#        /cmd_vel                        geometry_msgs/Twist
#
#   3. Creates an RTX LiDAR (Ouster OS1, 32-channel, 10 Hz, 1024 horiz
#      resolution) under /World/Go2/base/lidar and attaches a Replicator
#      writer that publishes:
#        /lidar/points                   sensor_msgs/PointCloud2
#      The 2D /scan (sensor_msgs/LaserScan) topic is NOT produced here.
#      Isaac's IsaacComputeRTXLidarFlatScan node refuses to run on a
#      non-2D lidar prim, and the RtxLidarROS2PublishLaserScan writer
#      is documented for 2D rotary configs only. Instead, /scan is
#      synthesised on the ROS side by a `pointcloud_to_laserscan` node
#      brought up by go2_bringup_sim/launch/chair_perception.launch.py
#      — that mirrors the pipeline used on real Go2 hardware (Livox
#      MID-360 -> PointCloud2 -> pointcloud_to_laserscan -> /scan), so
#      sim2real swaps the sensor driver only.
#
#   /tf_static (base_link -> camera_link / camera_color_optical_frame /
#   camera_depth_optical_frame / imu_link / lidar_link) is published by
#   the ROS-side launches (e.g.
#   go2_bringup_sim/launch/chair_perception.launch.py) via
#   tf2_ros::static_transform_publisher, NOT by this script — the
#   sim is the single source of truth for the dynamic odom -> base_link
#   transform and for sensor data; the rigid sensor extrinsics live in
#   the ROS launch where they can be tweaked without restarting Kit.
#   3. Each step, reads the latest /cmd_vel from the subscriber node and
#      applies it kinematically to the Go2 root xform. Legs don't animate
#      (no walking policy yet), but the base translates/rotates smoothly so
#      the camera, IMU, odom and tf topics are exercised end-to-end.
#
# Usage (Isaac 5.1) — prefer the wrapper which sets LD_LIBRARY_PATH/ROS_DISTRO:
#     bash scripts/run_warehouse_ros2.sh
#     # or, if you already sourced dev_env.sh manually:
#     "$ISAAC_SIM_ROOT/python.sh" sim/run_go2_warehouse_ros2.py --no-headless
#
# Verify from another shell (system ROS 2 — bridge talks via DDS, not shared libs):
#     source /opt/ros/jazzy/setup.bash
#     ros2 topic list
#     ros2 topic hz /imu
#     ros2 topic hz /camera/color/image_raw
#     ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.3}}'
# -----------------------------------------------------------------------------
import argparse
import math
import os
import sys
import time
import traceback
from pathlib import Path

# --- CLI (parse BEFORE SimulationApp boot) ----------------------------------
_parser = argparse.ArgumentParser()
_parser.add_argument("--no-headless", dest="headless", action="store_false",
                     help="Open the GUI while running (default).")
_parser.add_argument("--headless", dest="headless", action="store_true",
                     help="Force headless. NOTE: Isaac 5.1 headless has been "
                          "observed to crash in omni.graph.image.core on RTX 5090.")
_parser.add_argument("--rgb-resolution", default="1280x720",
                     help="RGB camera resolution, WxH (default 1280x720).")
_parser.add_argument("--depth-resolution", default="640x480",
                     help="Depth camera resolution, WxH (default 640x480).")
_parser.add_argument("--imu-hz", type=float, default=200.0)
_parser.add_argument("--camera-frame-skip", type=int, default=3,
                     help="Publish camera image every Nth physics frame "
                          "(3 -> ~20 Hz at 60 Hz physics).")
_parser.add_argument(
    "--lidar-config",
    default="OS1_REV6_32ch10hz1024res",
    help="RTX LiDAR config name from isaacsim.sensors.rtx/data/lidar_configs. "
         "Default OS1_REV6_32ch10hz1024res = Ouster OS1, 32 channels @ 10 Hz, "
         "1024 horizontal resolution. Used as a Livox MID-360 stand-in (NVIDIA "
         "doesn't ship a Livox MID-360 config in 5.1). Set to '' to disable.",
)
_parser.add_argument("--no-lidar", action="store_true",
                     help="Skip LiDAR creation entirely.")
_parser.add_argument(
    "--lidar-publish-step",
    type=int,
    default=9,
    help="Publish /lidar/points every Nth render frame. Acts as a divider on "
         "Replicator's PostProcessDispatchIsaacSimulationGate so the "
         "ScanBuffer accumulator has time to fill a full LiDAR rotation "
         "before each publish. Default 9 ≈ 10 Hz at ~90 Hz playback (one OS1 "
         "rotation period). Set 1 for raw render-rate firing (debug only — "
         "produces partial-arc scans that break SLAM).",
)
_parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
_parser.add_argument("--no-cmd-vel", action="store_true",
                     help="Skip /cmd_vel handling (publishers only).")
_parser.add_argument("--max-lin", type=float, default=1.0,
                     help="Clamp on linear velocity from /cmd_vel (m/s).")
_parser.add_argument("--max-ang", type=float, default=1.5,
                     help="Clamp on angular velocity from /cmd_vel (rad/s).")
# --- Phase 5: locomotion backend selection ---------------------------------
_parser.add_argument("--locomotion", choices=("kinematic", "policy"),
                     default="kinematic",
                     help="Locomotion backend. 'kinematic' (default) is the "
                          "Phase 0 set_world_pose integrator — no gait, no "
                          "joint motion. 'policy' loads an Isaac Lab Go2 "
                          "flat-terrain velocity policy via "
                          "--policy-checkpoint; it falls back to kinematic "
                          "on any load error.")
_parser.add_argument("--policy-checkpoint", default="",
                     help="TorchScript (.pt) Go2 locomotion checkpoint. "
                          "Required when --locomotion=policy.")
_parser.add_argument("--policy-decimation", type=int, default=4,
                     help="Policy decimation (physics ticks per policy step). "
                          "Isaac Lab default is 4 at 200 Hz sim = 50 Hz policy.")
_parser.add_argument("--diag", choices=("none", "boot-only", "after-build",
                                        "after-sensors", "after-graph"),
                     default="none",
                     help="Debug: exit after reaching the named checkpoint without "
                          "running the main loop. Useful to isolate where crashes happen.")
_parser.set_defaults(headless=False)  # Default to GUI — headless is known-crashy on 5.1/5090
ARGS, _ = _parser.parse_known_args()


# Echo env vars that matter for the ROS 2 bridge. These must be set BEFORE
# Kit boots; missing here means scripts/run_warehouse_ros2.sh didn't run (or
# Python was invoked directly). Visible before Kit eats stdout.
print("[run_ros2] env ROS_DISTRO=%r" % os.environ.get("ROS_DISTRO"))
print("[run_ros2] env RMW_IMPLEMENTATION=%r" % os.environ.get("RMW_IMPLEMENTATION"))
_ld = os.environ.get("LD_LIBRARY_PATH", "")
_bridge_lib_frag = "exts/isaacsim.ros2.bridge/jazzy/lib"
print("[run_ros2] bridge lib on LD_LIBRARY_PATH: %s" % (_bridge_lib_frag in _ld))
print("[run_ros2] headless=%s" % ARGS.headless)


def _parse_res(s, name):
    try:
        w, h = s.lower().split("x")
        return int(w), int(h)
    except Exception:
        raise SystemExit(f"--{name} must look like WxH, got {s!r}")


RGB_W, RGB_H = _parse_res(ARGS.rgb_resolution, "rgb-resolution")
DEPTH_W, DEPTH_H = _parse_res(ARGS.depth_resolution, "depth-resolution")


# --- Make `from sim import warehouse_scene` importable from repo root -------
_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))


# --- Boot Kit ---------------------------------------------------------------
# Same shim as build_go2_warehouse.py.
try:
    import isaacsim  # noqa: F401
except Exception:
    pass

SimulationApp = None
try:
    from isaacsim.simulation_app import SimulationApp as _Sa5
    SimulationApp = _Sa5
except Exception:
    try:
        from isaacsim import SimulationApp as _Sa4
        SimulationApp = _Sa4
    except Exception:
        pass
if SimulationApp is None:
    from omni.isaac.kit import SimulationApp  # pre-4.1 fallback

simulation_app = SimulationApp({
    "headless": ARGS.headless,
    "renderer": "RayTracedLighting",
    "width": RGB_W,
    "height": RGB_H,
})

# --- Imports that need Kit running ------------------------------------------
import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.kit.commands  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
import omni.graph.core as og  # noqa: E402
from pxr import Gf, Sdf, UsdGeom  # noqa: E402

# Only isaacsim.ros2.bridge is NOT auto-loaded by the default Isaac Sim 5.x
# Python boot. The other OG deps (omni.graph.action, omni.graph.nodes,
# omni.replicator.core, isaacsim.core.nodes, isaacsim.sensors.physics) all come
# up at app startup; calling set_extension_enabled_immediate on them
# redundantly was observed to trigger a crash inside
# libomni.graph.image.core.so during the first render-product evaluation on
# Isaac 5.1 + RTX 5090, so don't touch them.
_ext_mgr = omni.kit.app.get_app().get_extension_manager()
try:
    _ext_mgr.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
except Exception as exc:  # pragma: no cover
    print(f"[run_ros2] WARN failed to enable isaacsim.ros2.bridge: {exc}")

# With the default (publish_without_verification=false) Isaac 5.1's ROS2 bridge
# only actually emits samples for /clock, /imu, /odom, /tf after a subscriber
# has been discovered on the DDS side, and that handshake can take several
# seconds -- long enough that `ros2 topic hz /clock` often prints
# "does not appear to be published yet" forever. For an MVP / debugging
# workflow we want every publisher to fire as soon as its OG node ticks, so
# flip the setting on at boot. Real-robot runs can revert this.
try:
    import carb  # noqa: E402
    _carb_settings = carb.settings.get_settings()
    _carb_settings.set(
        "/exts/isaacsim.ros2.bridge/publish_without_verification", True
    )
    print("[run_ros2] carb: publish_without_verification=True")
except Exception as exc:  # pragma: no cover
    print(f"[run_ros2] WARN could not set publish_without_verification: {exc}")

import omni.replicator.core as rep  # noqa: E402  (auto-loaded)

# Isaac 5.x prim utilities. Must be imported AFTER Kit boots.
try:
    from isaacsim.core.utils.prims import create_prim  # noqa: E402
except Exception:  # pragma: no cover
    from omni.isaac.core.utils.prims import create_prim  # noqa: E402

# In Isaac Sim 5.1 the old `isaacsim.core.api.articulations.Articulation`
# class was renamed to `SingleArticulation` and moved to `isaacsim.core.prims`;
# `isaacsim.core.api.articulations` now only exports ArticulationGripper /
# ArticulationSubset. CmdVelDriver uses this class to push world poses into
# the PhysX-owned articulation root, so import it at module scope and fail
# fast if it isn't available.
try:
    from isaacsim.core.prims import SingleArticulation  # noqa: E402
except Exception:  # pragma: no cover
    # 4.x fallback — older installs still export Articulation under
    # isaacsim.core.api.articulations. Alias it so CmdVelDriver is unchanged.
    from isaacsim.core.api.articulations import Articulation as SingleArticulation  # noqa: E402

# Our scene library — imports inside require Kit to be already running.
from sim import warehouse_scene as ws  # type: ignore  # noqa: E402


# =============================================================================
# Constants (must match warehouse_scene.build_go2_spawn paths)
# =============================================================================
GO2_ROOT = "/World/Go2"
GO2_BASE = "/World/Go2/base"     # articulation root if Go2 USD composed OK
CAM_PATH = "/World/Go2/base/front_cam"
IMU_PATH = "/World/Go2/base/imu"
ACTION_GRAPH_PATH = "/World/ROS2ActionGraph"


# =============================================================================
# Stage setup
# =============================================================================
def build_scene():
    """Create a fresh World and populate it with the warehouse. Returns the
    live World instance. This replaces the old open_stage(saved_usd) path."""
    print("[run_ros2] Building warehouse scene (fresh stage)...")
    world = ws.build_world()
    ws.build_full_warehouse(world)

    # Let lights / colliders settle before sensors attach.
    try:
        world.reset()
        for _ in range(3):
            simulation_app.update()
    except Exception:
        print("[run_ros2] WARN world.reset()/tick failed (non-fatal):")
        traceback.print_exc()
    print("[run_ros2] Scene built.")
    return world


def _pick_go2_base(stage):
    """Go2 USD may or may not compose a /base child (depends on asset version).
    If /World/Go2/base doesn't exist, fall back to /World/Go2 so downstream
    prims still attach under the robot root."""
    if stage.GetPrimAtPath(GO2_BASE).IsValid():
        return GO2_BASE
    print(f"[run_ros2] {GO2_BASE} not found; attaching sensors to {GO2_ROOT} instead.")
    return GO2_ROOT


def ensure_camera_prim():
    """Create an RGB-D camera under the Go2 base link if missing."""
    stage = omni.usd.get_context().get_stage()
    base_path = _pick_go2_base(stage)
    cam_path = f"{base_path}/front_cam"

    if stage.GetPrimAtPath(cam_path).IsValid():
        print(f"[run_ros2] Camera prim already exists at {cam_path}")
        return cam_path

    # Position: 30 cm forward (+X), 12 cm up (+Z). Orientation in WXYZ:
    # the USD camera natively looks down its -Z axis with image-up = +Y.
    # We want a robot-mounted forward-facing RGB camera with:
    #   * looking direction = base_link +X (forward)
    #   * image-up         = base_link +Z (world up)
    #   * image-right      = base_link -Y (robot's right)
    # That is the composition of two rotations:
    #   R1 = -90° about base_link +Y      → puts USD -Z onto base_link +X
    #   R2 = +90° about base_link +X      → puts USD +Y onto base_link +Z
    # (R2 corrects the 90° image roll we'd see otherwise.)
    # The resulting quaternion (WXYZ) is (0.5, 0.5, -0.5, -0.5).
    # Isaac 5.x create_prim expects orientation as a plain [w,x,y,z]
    # float sequence, NOT a Gf.Quat* (that path calls
    # np.asarray().astype(float32) and Gf.Quatd is not float-castable).
    create_prim(
        prim_path=cam_path,
        prim_type="Camera",
        translation=(0.30, 0.0, 0.12),
        orientation=[0.5, 0.5, -0.5, -0.5],  # WXYZ; -90°@Y then +90°@X
    )
    cam_prim = stage.GetPrimAtPath(cam_path)
    cam = UsdGeom.Camera(cam_prim)
    cam.CreateFocalLengthAttr(18.14756)
    cam.CreateHorizontalApertureAttr(20.955)   # ~60° HFOV
    cam.CreateVerticalApertureAttr(15.2908)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.05, 30.0))
    cam.CreateFocusDistanceAttr(2.0)
    print(f"[run_ros2] Camera created at {cam_path}")
    return cam_path


# Module-level handle so the IMUSensor Python wrapper isn't garbage-collected.
# That matters because IMUSensor.__init__ is what actually registers our prim
# with the C++ `_sensor` interface — losing the reference would let the
# interface drop the sensor on the next GC sweep and IsaacReadIMU would
# silently stop producing data.
_IMU_SENSOR_KEEPALIVE = None


def ensure_imu_prim():
    """Create + REGISTER an IMU sensor on the Go2 base link.

    Two-step dance is required on Isaac Sim 5.1:

    1. `IsaacSensorCreateImuSensor` (via omni.kit.commands) builds the USD
       prim with `IsaacImuSensor` schema and sets `enabled=True`.
    2. `isaacsim.sensors.physics.IMUSensor(prim_path=...)` wraps that prim
       in a Python BaseSensor and, crucially, calls
       `_sensor.acquire_imu_sensor_interface()` which REGISTERS the prim
       with the C++ sensor backend. Without step 2 `IsaacReadIMU` keeps
       returning "no valid sensor reading, is the sensor enabled?" every
       tick and its `outputs:execOut` never fires, so PubIMU never even
       advertises `/imu` on DDS. Step 1 alone is NOT enough.
    """
    global _IMU_SENSOR_KEEPALIVE
    stage = omni.usd.get_context().get_stage()
    base_path = _pick_go2_base(stage)
    imu_path = f"{base_path}/imu"

    if not stage.GetPrimAtPath(imu_path).IsValid():
        # Step 1: build the schema'd prim. Isaac 5.1 dropped `visualize`
        # and renamed the filter kwargs — pass the minimal stable set.
        success, _ = omni.kit.commands.execute(
            "IsaacSensorCreateImuSensor",
            path="/imu",
            parent=base_path,
            sensor_period=1.0 / max(ARGS.imu_hz, 1.0),
            translation=Gf.Vec3d(0.0, 0.0, 0.0),
            orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # identity
        )
        if not success:
            raise SystemExit("[run_ros2] IsaacSensorCreateImuSensor failed; "
                             "see Kit log above.")
        print(f"[run_ros2] IMU prim created at {imu_path}")
    else:
        print(f"[run_ros2] IMU prim already exists at {imu_path}")

    # Step 2: register with the C++ sensor interface. This has to happen
    # AFTER the prim exists and BEFORE the action graph first ticks.
    try:
        from isaacsim.sensors.physics import IMUSensor  # noqa: E402
    except Exception:  # pragma: no cover — 4.x fallback
        from omni.isaac.sensor import IMUSensor  # noqa: E402

    _IMU_SENSOR_KEEPALIVE = IMUSensor(
        prim_path=imu_path,
        name="go2_imu",
        frequency=int(max(ARGS.imu_hz, 1.0)),
        translation=np.array([0.0, 0.0, 0.0], dtype=float),
        orientation=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),  # WXYZ
    )
    print(f"[run_ros2] IMU registered with sensor interface at {imu_path}")
    return imu_path


def make_render_product(cam_path):
    """Build a single render product on the camera; reused by RGB + depth + camera_info."""
    rp = rep.create.render_product(cam_path, resolution=(RGB_W, RGB_H))
    rp_path = rp.path if hasattr(rp, "path") else str(rp)
    print(f"[run_ros2] RenderProduct created at {rp_path}  ({RGB_W}x{RGB_H})")
    return rp_path


# Module-level keepalives for the RTX LiDAR sensor / render product.
# The Replicator pipeline holds the sensor + render product alive via
# its own internal references once the OG helper is wired, but pinning
# them here adds a defensive belt+suspenders against GC.
_LIDAR_PRIM_KEEPALIVE = None
_LIDAR_RP_KEEPALIVE = None


def ensure_lidar_prim():
    """Create an RTX LiDAR sensor under the Go2 base link and a
    dedicated Replicator render product for it.

    Default config is Ouster OS1, 32 ch @ 10 Hz, 1024 horizontal res
    — chosen as a Livox MID-360 stand-in because Isaac Sim 5.1 doesn't
    ship a Livox MID-360 JSON. For SLAM / Nav2 / frontier purposes the
    two are interchangeable; sim2real later only requires swapping the
    LiDAR driver on the real Go2.

    The PointCloud2 publisher itself is NOT created here. It's added
    in build_action_graph() as an `isaacsim.ros2.bridge.ROS2RtxLidarHelper`
    OG node — the same high-level abstraction we use for the cameras.
    Going through the OG helper avoids the time-sequencing pitfall
    we hit when calling rep.writers.get(...) directly + manually
    setting PostProcessDispatchIsaacSimulationGate.step: the gate
    sits BEFORE IsaacCreateRTXLidarScanBuffer in the pipeline, so a
    naive step>1 silently produces empty 0-point clouds because the
    accumulator only sees one render frame at a time. The OG helper
    sets `frameSkipCount` and the gate step in the right order via
    its post_attach hook so the buffer accumulates correctly.

    Returns (lidar_prim_path, render_product_path) tuple, or
    (None, None) if creation was skipped / failed. A failure here is
    non-fatal: the rest of the sim pipeline still comes up, just
    with no /lidar/points.
    """
    global _LIDAR_PRIM_KEEPALIVE, _LIDAR_RP_KEEPALIVE

    if ARGS.no_lidar or not ARGS.lidar_config:
        print("[run_ros2] LiDAR skipped (--no-lidar or empty --lidar-config)")
        return None, None

    stage = omni.usd.get_context().get_stage()
    base_path = _pick_go2_base(stage)
    parent_xform_path = f"{base_path}/lidar"

    # Mounting: 10 cm forward of base origin, 20 cm above. That keeps
    # the LiDAR clear of the Go2 body silhouette (so it doesn't see
    # itself) and roughly matches a Livox MID-360 mount on a real
    # Go2 EDU. Identity orientation: LiDAR's local Z (rotation axis)
    # is already aligned with base_link's +Z (up), which matches
    # the OS1 config's expectation.
    #
    # Decompose the deprecated all-in-one config name (e.g.
    # 'OS1_REV6_32ch10hz1024res') into Isaac 5.x's preferred
    # config + variant pair (config='OS1', variant=full_name).
    # Isaac 5.x's IsaacSensorCreateRtxLidar internally does the
    # same auto-truncation but emits a deprecation warning; calling
    # the new API explicitly is cleaner and forward-compatible.
    cfg_arg = ARGS.lidar_config
    variant_arg = None
    if cfg_arg.startswith("OS") and len(cfg_arg) > 3:
        variant_arg = cfg_arg
        cfg_arg = cfg_arg[:3]   # 'OS1', 'OS0', or 'OS2'

    sensor_prim = None
    if not stage.GetPrimAtPath(parent_xform_path).IsValid():
        try:
            kwargs = dict(
                path="/lidar",
                parent=base_path,
                config=cfg_arg,
                translation=Gf.Vec3d(0.10, 0.0, 0.20),
                orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),  # Gf.Quatd is w,i,j,k
            )
            if variant_arg:
                kwargs["variant"] = variant_arg
            success, sensor_prim = omni.kit.commands.execute(
                "IsaacSensorCreateRtxLidar", **kwargs,
            )
        except Exception as exc:  # pragma: no cover
            print(f"[run_ros2] WARN IsaacSensorCreateRtxLidar threw: {exc}")
            traceback.print_exc()
            return None, None
        if not success or sensor_prim is None:
            print(f"[run_ros2] WARN IsaacSensorCreateRtxLidar returned "
                  f"success={success} prim={sensor_prim}; skipping LiDAR")
            return None, None
        _LIDAR_PRIM_KEEPALIVE = sensor_prim
    else:
        # Re-running without scene rebuild: parent Xform still there,
        # walk down to find the OmniLidar child.
        parent_prim = stage.GetPrimAtPath(parent_xform_path)
        for child in parent_prim.GetChildren():
            if child.GetTypeName() == "OmniLidar":
                sensor_prim = child
                break
        if sensor_prim is None:
            print(f"[run_ros2] WARN existing prim at {parent_xform_path} has "
                  f"no OmniLidar child; skipping LiDAR")
            return None, None

    # CRITICAL: bind the render product to the actual OmniLidar prim,
    # NOT to the parent Xform that 'IsaacSensorCreateRtxLidar' references
    # the OS1.usd into. OgnROS2RtxLidarHelper specifically checks:
    #   prim.GetTypeName() == "OmniLidar"  AND
    #   prim.HasAPI("OmniSensorGenericLidarCoreAPI")
    # If we point the render product at the parent Xform, the helper
    # queries that, sees an Xform (not OmniLidar), and silently warns
    # "Render product not attached to RTX Lidar" — and never publishes
    # a single point. Use sensor_prim.GetPath() returned from the
    # command (it's already auto-resolved to the OmniLidar child by
    # IsaacSensorCreateRtxSensor._add_reference).
    actual_lidar_path = str(sensor_prim.GetPath())
    print(f"[run_ros2] LiDAR ('{ARGS.lidar_config}') OmniLidar prim "
          f"at {actual_lidar_path} (parent Xform at {parent_xform_path})")

    # RTX LiDAR data flows through Replicator render products, NOT
    # through OmniGraph the way our /odom and /imu pipelines do. The
    # render product resolution [1, 1] is intentional and matches the
    # NVIDIA standalone example — RTX LiDAR uses a custom render path
    # that ignores the camera-style WxH input.
    try:
        rp = rep.create.render_product(actual_lidar_path, [1, 1],
                                       name="Go2LidarRP")
    except Exception as exc:  # pragma: no cover
        print(f"[run_ros2] WARN rep.create.render_product on LiDAR failed: {exc}")
        traceback.print_exc()
        return None, None
    _LIDAR_RP_KEEPALIVE = rp

    rp_path = rp.path if hasattr(rp, "path") else str(rp)
    print(f"[run_ros2] LiDAR RenderProduct created at {rp_path}")
    return actual_lidar_path, rp_path


# =============================================================================
# OmniGraph
# =============================================================================
# Map logical node name -> ordered list of candidate full type identifiers.
#
# These identifiers were verified by extracting strings from the compiled .so
# files inside the Isaac Sim 5.1 extensions (see scripts/dev_env.sh for the
# install path). For each row we try the 5.1 (isaacsim.*) name first and fall
# back to the 4.x (omni.isaac.*) name if someone runs this on an older build.
#
# NOTE ON `Ogn` PREFIX: in 5.1, `OgnIsaacRunOneSimulationFrame` is the ONE
# node in isaacsim.core.nodes whose registered identifier still carries the
# `Ogn` prefix — every other identifier is bare (`IsaacComputeOdometry`,
# `IsaacReadSimulationTime`, `IsaacSimulationGate`, ...). Do not strip it.
_NODE_CANDIDATES = {
    "RunOne": (
        # 5.1 — real registered string taken from
        # isaacsim.core.nodes/bin/*.so: `isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame`
        "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame",
        # 4.x fallback
        "omni.isaac.core_nodes.IsaacRunOneSimulationFrame",
    ),
    "SimTime": (
        "isaacsim.core.nodes.IsaacReadSimulationTime",
        "omni.isaac.core_nodes.IsaacReadSimulationTime",
    ),
    "ComputeOdom": (
        "isaacsim.core.nodes.IsaacComputeOdometry",
        "omni.isaac.core_nodes.IsaacComputeOdometry",
    ),
    "ReadIMU": (
        "isaacsim.sensors.physics.IsaacReadIMU",
        "omni.isaac.sensor.IsaacReadIMU",
    ),
    "Ctx": (
        "isaacsim.ros2.bridge.ROS2Context",
        "omni.isaac.ros2_bridge.ROS2Context",
    ),
    "PubClock": (
        "isaacsim.ros2.bridge.ROS2PublishClock",
        "omni.isaac.ros2_bridge.ROS2PublishClock",
    ),
    "CamRGB": (
        "isaacsim.ros2.bridge.ROS2CameraHelper",
        "omni.isaac.ros2_bridge.ROS2CameraHelper",
    ),
    "CamDepth": (
        "isaacsim.ros2.bridge.ROS2CameraHelper",
        "omni.isaac.ros2_bridge.ROS2CameraHelper",
    ),
    "CamInfo": (
        "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
        "omni.isaac.ros2_bridge.ROS2CameraInfoHelper",
    ),
    # Second instance of the same node type — both depth and color are
    # produced from a single render product (one Camera prim under
    # /World/Go2/base/front_cam) so their intrinsics are identical, but
    # ROS REP-104/REP-105 expects /camera/depth/camera_info to be
    # advertised independently from /camera/color/camera_info. Emitting
    # this keeps cv_bridge / image_pipeline / Nav2 / depth_image_proc
    # consumers happy without forcing them to rewrite the depth header.
    "CamInfoDepth": (
        "isaacsim.ros2.bridge.ROS2CameraInfoHelper",
        "omni.isaac.ros2_bridge.ROS2CameraInfoHelper",
    ),
    "PubIMU": (
        "isaacsim.ros2.bridge.ROS2PublishImu",
        "omni.isaac.ros2_bridge.ROS2PublishImu",
    ),
    "PubOdom": (
        "isaacsim.ros2.bridge.ROS2PublishOdometry",
        "omni.isaac.ros2_bridge.ROS2PublishOdometry",
    ),
    "PubRawTF": (
        "isaacsim.ros2.bridge.ROS2PublishRawTransformTree",
        "omni.isaac.ros2_bridge.ROS2PublishRawTransformTree",
    ),
    "SubTwist": (
        "isaacsim.ros2.bridge.ROS2SubscribeTwist",
        "omni.isaac.ros2_bridge.ROS2SubscribeTwist",
    ),
    "LidarROS2": (
        # 5.x — high-level helper that wraps RtxLidarROS2PublishPointCloud(Buffer)
        # writers and the PostProcessDispatchIsaacSimulationGate.step
        # throttle into one node. Avoids the manual sequencing pitfall
        # we hit calling rep.writers + set_node_attributes by hand.
        "isaacsim.ros2.bridge.ROS2RtxLidarHelper",
        "omni.isaac.ros2_bridge.ROS2RtxLidarHelper",
    ),
}


def _create_node(graph_path, name):
    """Create node {graph_path}/{name} by walking the hardcoded candidate
    list for that logical name. Returns the identifier that actually worked."""
    candidates = _NODE_CANDIDATES[name]
    node_path = f"{graph_path}/{name}"

    last_exc = None
    for ident in candidates:
        try:
            og.Controller.create_node(node_path, ident)
            return ident
        except Exception as exc:
            last_exc = exc
            # Clean up any half-created prim before the next attempt.
            try:
                og.Controller.delete_node(node_path)
            except Exception:
                pass

    print(f"[run_ros2] ERROR could not create '{name}'")
    print(f"[run_ros2]   tried: {list(candidates)}")
    raise RuntimeError(
        f"No working node type for '{name}'. "
        f"Tried {list(candidates)}. Last error: {last_exc}"
    )


def build_action_graph(render_product_path, imu_path, chassis_path,
                       lidar_render_product_path=None):
    keys = og.Controller.Keys

    # Frame IDs follow REP-105 conventions for legged robots.
    odom_frame = "odom"
    base_frame = "base_link"
    cam_frame = "camera_link"
    imu_frame = "imu_link"
    lidar_frame = "lidar_link"

    # Step 1: create the graph with a known-good node so the graph exists.
    og.Controller.edit(
        {"graph_path": ACTION_GRAPH_PATH, "evaluator_name": "execution"},
        {keys.CREATE_NODES: [("OnTick", "omni.graph.action.OnPlaybackTick")]},
    )

    # Step 2: create each remaining node. _create_node walks the
    # version-aware hardcoded candidate map above.
    logical_names = ["RunOne", "SimTime", "ComputeOdom", "ReadIMU",
                     "Ctx", "PubClock",
                     "CamRGB", "CamDepth", "CamInfo", "CamInfoDepth",
                     "PubIMU", "PubOdom", "PubRawTF",
                     "SubTwist"]
    if lidar_render_product_path:
        logical_names.append("LidarROS2")

    for logical_name in logical_names:
        ident = _create_node(ACTION_GRAPH_PATH, logical_name)
        print(f"[run_ros2]   created {logical_name:<12} -> {ident}")

    print(f"[run_ros2] All graph nodes created under {ACTION_GRAPH_PATH}.")

    # In omni.graph-1.141.2 (Isaac 5.1) og.Controller.edit no longer auto-
    # prefixes short "Node.port" references with the graph path, so every
    # attribute reference below must be absolute (full Sdf path).
    def A(short):
        return f"{ACTION_GRAPH_PATH}/{short}"

    # EXECUTION WIRING — critical subtlety:
    #
    # `isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame` is documented as
    # "Executes an output execution pulse the FIRST TIME this node is ran".
    # It is a ONE-SHOT gate, not a per-simulation-frame clock. We (wrongly)
    # used to route PubClock/ReadIMU/ComputeOdom/SubTwist through RunOne,
    # which made every one of them fire a single pulse at t=0 and never
    # again -> /clock had one message that FastDDS subscribers always
    # missed (QoS=VOLATILE), /odom & /tf same story, and /imu was never
    # even advertised because ReadIMU's first-ever call happens before
    # physics has populated the IMU buffer so its execOut never fired.
    #
    # The fix: drive those per-frame nodes directly from OnPlaybackTick,
    # which fires on every frame while the timeline is playing. Keep
    # RunOne feeding the camera helpers — their execIn is a one-time
    # render-product binding trigger, so one-shot semantics are exactly
    # what ROS2CameraHelper wants (and that's why cameras were the only
    # topic group that worked in previous runs).
    connect = [
        # One-time setup path: OnTick -> RunOne -> Cam*/Lidar (binds render product)
        (A("OnTick.outputs:tick"), A("RunOne.inputs:execIn")),
        (A("RunOne.outputs:step"), A("CamRGB.inputs:execIn")),
        (A("RunOne.outputs:step"), A("CamDepth.inputs:execIn")),
        (A("RunOne.outputs:step"), A("CamInfo.inputs:execIn")),
        (A("RunOne.outputs:step"), A("CamInfoDepth.inputs:execIn")),

        # Per-frame path: OnTick -> every publisher/reader we want ticking
        # on every simulation frame. PubIMU/PubOdom/PubRawTF still cascade
        # off their respective reader's execOut so they only publish with
        # valid data.
        (A("OnTick.outputs:tick"), A("PubClock.inputs:execIn")),
        (A("OnTick.outputs:tick"), A("ReadIMU.inputs:execIn")),
        (A("OnTick.outputs:tick"), A("ComputeOdom.inputs:execIn")),
        (A("OnTick.outputs:tick"), A("SubTwist.inputs:execIn")),
        # NOTE: PubIMU used to cascade off ReadIMU.outputs:execOut, but that
        # output is documented as "triggers when sensor has data" — which on
        # Isaac 5.1 means the first few frames (and any frame where the IMU
        # interface returns stale) the pulse is skipped, PubIMU never ticks,
        # and /imu/data is never even advertised on DDS. Driving PubIMU
        # directly from OnTick guarantees the topic is published every frame;
        # the data-port connections below still stream fresh values from
        # ReadIMU whenever the sensor produces them.
        (A("OnTick.outputs:tick"), A("PubIMU.inputs:execIn")),
        (A("ComputeOdom.outputs:execOut"), A("PubOdom.inputs:execIn")),
        (A("ComputeOdom.outputs:execOut"), A("PubRawTF.inputs:execIn")),
        (A("Ctx.outputs:context"), A("PubClock.inputs:context")),
        (A("Ctx.outputs:context"), A("CamRGB.inputs:context")),
        (A("Ctx.outputs:context"), A("CamDepth.inputs:context")),
        (A("Ctx.outputs:context"), A("CamInfo.inputs:context")),
        (A("Ctx.outputs:context"), A("CamInfoDepth.inputs:context")),
        (A("Ctx.outputs:context"), A("PubIMU.inputs:context")),
        (A("Ctx.outputs:context"), A("PubOdom.inputs:context")),
        (A("Ctx.outputs:context"), A("PubRawTF.inputs:context")),
        (A("Ctx.outputs:context"), A("SubTwist.inputs:context")),
        (A("SimTime.outputs:simulationTime"), A("PubClock.inputs:timeStamp")),
        (A("SimTime.outputs:simulationTime"), A("PubIMU.inputs:timeStamp")),
        (A("SimTime.outputs:simulationTime"), A("PubOdom.inputs:timeStamp")),
        (A("SimTime.outputs:simulationTime"), A("PubRawTF.inputs:timeStamp")),
        (A("ReadIMU.outputs:angVel"), A("PubIMU.inputs:angularVelocity")),
        (A("ReadIMU.outputs:linAcc"), A("PubIMU.inputs:linearAcceleration")),
        (A("ReadIMU.outputs:orientation"), A("PubIMU.inputs:orientation")),
        (A("ComputeOdom.outputs:linearVelocity"), A("PubOdom.inputs:linearVelocity")),
        (A("ComputeOdom.outputs:angularVelocity"), A("PubOdom.inputs:angularVelocity")),
        (A("ComputeOdom.outputs:orientation"), A("PubOdom.inputs:orientation")),
        (A("ComputeOdom.outputs:position"), A("PubOdom.inputs:position")),
        (A("ComputeOdom.outputs:position"), A("PubRawTF.inputs:translation")),
        (A("ComputeOdom.outputs:orientation"), A("PubRawTF.inputs:rotation")),
    ]
    if lidar_render_product_path:
        # LidarROS2 helper is a one-shot init like the camera helpers.
        connect.extend([
            (A("RunOne.outputs:step"), A("LidarROS2.inputs:execIn")),
            (A("Ctx.outputs:context"), A("LidarROS2.inputs:context")),
        ])

    set_values = [
        (A("CamRGB.inputs:type"), "rgb"),
        (A("CamRGB.inputs:topicName"), "/camera/color/image_raw"),
        (A("CamRGB.inputs:frameId"), cam_frame),
        (A("CamRGB.inputs:renderProductPath"), render_product_path),
        (A("CamRGB.inputs:frameSkipCount"), max(0, ARGS.camera_frame_skip)),

        (A("CamDepth.inputs:type"), "depth"),
        (A("CamDepth.inputs:topicName"), "/camera/depth/image_rect_raw"),
        (A("CamDepth.inputs:frameId"), cam_frame),
        (A("CamDepth.inputs:renderProductPath"), render_product_path),
        (A("CamDepth.inputs:frameSkipCount"), max(0, ARGS.camera_frame_skip)),

        (A("CamInfo.inputs:topicName"), "/camera/color/camera_info"),
        (A("CamInfo.inputs:frameId"), cam_frame),
        (A("CamInfo.inputs:renderProductPath"), render_product_path),
        (A("CamInfo.inputs:frameSkipCount"), max(0, ARGS.camera_frame_skip)),

        # Depth camera_info — same intrinsics as color (single Camera
        # prim → single render product) but advertised on its own topic
        # so REP-104 consumers (depth_image_proc, point_cloud_xyz,
        # cv_bridge users) don't have to remap.
        (A("CamInfoDepth.inputs:topicName"), "/camera/depth/camera_info"),
        (A("CamInfoDepth.inputs:frameId"), cam_frame),
        (A("CamInfoDepth.inputs:renderProductPath"), render_product_path),
        (A("CamInfoDepth.inputs:frameSkipCount"), max(0, ARGS.camera_frame_skip)),

        # Topic name matches REP-145 / the downstream go2_* perception stack
        # which subscribes to /imu/data (NOT /imu).
        (A("PubIMU.inputs:topicName"), "/imu/data"),
        (A("PubIMU.inputs:frameId"), imu_frame),

        # CRITICAL for Isaac 5.1: default (useLatestData=False) makes ReadIMU
        # read from the sensor's internal filtered buffer. That buffer is only
        # populated when the IMU has been registered through the
        # isaacsim.sensors.physics Python wrapper (IMUSensor class). Our
        # lower-level IsaacSensorCreateImuSensor command creates the prim
        # but skips that registration path, so the buffer stays empty and
        # ReadIMU keeps printing "no valid sensor reading, is the sensor
        # enabled?" — which in turn means ReadIMU.execOut never fires,
        # PubIMU never advertises, and /imu never shows up on DDS.
        # Switching useLatestData=True makes ReadIMU query the raw physics
        # step every tick and sidesteps the buffer entirely.
        (A("ReadIMU.inputs:useLatestData"), True),
        (A("ReadIMU.inputs:readGravity"), True),

        (A("PubOdom.inputs:topicName"), "/odom"),
        (A("PubOdom.inputs:odomFrameId"), odom_frame),
        (A("PubOdom.inputs:chassisFrameId"), base_frame),
        (A("PubRawTF.inputs:parentFrameId"), odom_frame),
        (A("PubRawTF.inputs:childFrameId"), base_frame),
        (A("PubRawTF.inputs:topicName"), "/tf"),

        (A("PubClock.inputs:topicName"), "/clock"),

        (A("SubTwist.inputs:topicName"), ARGS.cmd_vel_topic),
    ]

    if lidar_render_product_path:
        # ROS2RtxLidarHelper handles the full Replicator wiring: it
        # picks the RtxLidarROS2PublishPointCloudBuffer writer when
        # fullScan=True (one full rotation per message), and writes
        # PostProcessDispatchIsaacSimulationGate.step = frameSkipCount+1
        # in its post_attach hook so the buffer accumulator gets the
        # right number of render frames before each publish. Setting
        # frameSkipCount = lidar_publish_step - 1 gives us /lidar/points
        # at ~playback_hz / lidar_publish_step Hz.
        gate_step = max(1, int(ARGS.lidar_publish_step))
        set_values.extend([
            (A("LidarROS2.inputs:type"), "point_cloud"),
            (A("LidarROS2.inputs:topicName"), "/lidar/points"),
            (A("LidarROS2.inputs:frameId"), lidar_frame),
            (A("LidarROS2.inputs:renderProductPath"), lidar_render_product_path),
            (A("LidarROS2.inputs:fullScan"), True),
            (A("LidarROS2.inputs:frameSkipCount"), gate_step - 1),
        ])

    # Step 3: apply values + connections on the already-populated graph.
    og.Controller.edit(
        ACTION_GRAPH_PATH,
        {
            keys.SET_VALUES: set_values,
            keys.CONNECT: connect,
        },
    )

    # `target` typed inputs (imuPrim, chassisPrim) are USD relationships, not
    # plain attributes. og.Controller.attribute(...).set([Sdf.Path(...)]) may
    # appear to work in Fabric but does NOT persist to the underlying USD
    # relationship, so any timeline reset / graph re-init wipes them and the
    # downstream IsaacReadIMU / IsaacComputeOdometry nodes silently produce no
    # output (which is exactly what kills /imu, /odom and /tf). Write them
    # through the USD relationship API so they stick.
    stage = omni.usd.get_context().get_stage()
    imu_node_prim = stage.GetPrimAtPath(f"{ACTION_GRAPH_PATH}/ReadIMU")
    odom_node_prim = stage.GetPrimAtPath(f"{ACTION_GRAPH_PATH}/ComputeOdom")
    imu_rel = (imu_node_prim.GetRelationship("inputs:imuPrim")
               or imu_node_prim.CreateRelationship("inputs:imuPrim", custom=False))
    chassis_rel = (odom_node_prim.GetRelationship("inputs:chassisPrim")
                   or odom_node_prim.CreateRelationship("inputs:chassisPrim", custom=False))
    imu_rel.SetTargets([Sdf.Path(imu_path)])
    chassis_rel.SetTargets([Sdf.Path(chassis_path)])
    print(f"[run_ros2] target rels written  "
          f"ReadIMU.imuPrim={imu_path}  ComputeOdom.chassisPrim={chassis_path}")

    print(f"[run_ros2] Action graph built at {ACTION_GRAPH_PATH}.")
    return og.get_graph_by_path(ACTION_GRAPH_PATH)


# =============================================================================
# /cmd_vel driver  (Phase 5: locomotion-backend façade)
# =============================================================================
# The class name `CmdVelDriver` is preserved for external references
# and log-grep continuity. What changed in Phase 5 is that it no
# longer owns the motion logic itself; it is a façade that reads the
# SubTwist OmniGraph node once per physics step and forwards the
# (linear, angular) command to a pluggable LocomotionBackend chosen
# via --locomotion {kinematic, policy}. See sim/locomotion_backends.py.
from sim.locomotion_backends import (     # noqa: E402
    LocomotionBackendConfig,
    PolicyConfig,
    make_backend,
)


class CmdVelDriver:
    """Reads the latest body-frame Twist from the SubTwist OmniGraph
    node each physics tick and forwards it to the selected Phase 5
    LocomotionBackend.

    Phase 0 behaviour (kinematic set_world_pose integration) is
    preserved as the default via `--locomotion kinematic`. A
    `--locomotion policy` scaffold is available but requires an
    external Isaac Lab TorchScript checkpoint; on any error the
    factory silently falls back to `kinematic` and logs a warning.
    """

    def __init__(self, graph_path, articulation_path):
        self._lin_attr = og.Controller.attribute(
            f"{graph_path}/SubTwist.outputs:linearVelocity"
        )
        self._ang_attr = og.Controller.attribute(
            f"{graph_path}/SubTwist.outputs:angularVelocity"
        )

        cfg = LocomotionBackendConfig(
            articulation_path=articulation_path,
            max_lin=float(ARGS.max_lin),
            max_ang=float(ARGS.max_ang),
        )
        policy_cfg = PolicyConfig(
            checkpoint_path=ARGS.policy_checkpoint,
            decimation=int(ARGS.policy_decimation),
        )
        self._backend, warn = make_backend(
            ARGS.locomotion, cfg, policy_cfg=policy_cfg
        )
        if warn:
            print(f"[run_ros2] WARN {warn}")
        print(
            f"[run_ros2] CmdVelDriver → LocomotionBackend='{self._backend.name}' "
            f"articulation='{articulation_path}'"
        )

    def step(self, dt):
        try:
            lin = self._lin_attr.get()
            ang = self._ang_attr.get()
        except Exception:
            return
        self._backend.step(dt, lin, ang)


# =============================================================================
# Main
# =============================================================================
def main():
    if ARGS.diag == "boot-only":
        print("[run_ros2] diag=boot-only — Kit booted, exiting without building scene.")
        simulation_app.close()
        return

    world = build_scene()
    if ARGS.diag == "after-build":
        print("[run_ros2] diag=after-build — scene built, exiting.")
        simulation_app.close()
        return

    cam_path = ensure_camera_prim()
    imu_path = ensure_imu_prim()
    lidar_path, lidar_rp_path = ensure_lidar_prim()
    # chassis = Go2 base link if available, otherwise the root xform.
    chassis_path = _pick_go2_base(omni.usd.get_context().get_stage())
    if ARGS.diag == "after-sensors":
        print(f"[run_ros2] diag=after-sensors — camera+IMU+LiDAR (lidar={lidar_path}) "
              f"created, exiting.")
        simulation_app.close()
        return

    rp_path = make_render_product(cam_path)
    build_action_graph(rp_path, imu_path, chassis_path, lidar_rp_path)
    if ARGS.diag == "after-graph":
        print("[run_ros2] diag=after-graph — graph built, exiting.")
        simulation_app.close()
        return

    # NOTE: do NOT call world.reset() here. build_scene() already reset once;
    # a second reset after the action graph is built was observed to stop the
    # timeline mid-handshake and wipe Fabric state for the freshly-created
    # OmniGraph nodes, which is why /clock/imu/odom/tf stayed silent on the
    # first run. Instead, just make sure the timeline is in Play so that
    # omni.graph.action.OnPlaybackTick keeps firing.
    omni.timeline.get_timeline_interface().play()

    driver = None
    if not ARGS.no_cmd_vel:
        try:
            driver = CmdVelDriver(ACTION_GRAPH_PATH, chassis_path)
            print(f"[run_ros2] /cmd_vel driver attached (articulation). "
                  f"Topic: {ARGS.cmd_vel_topic}")
        except Exception as exc:
            print(f"[run_ros2] WARN /cmd_vel driver disabled: {exc}")
            traceback.print_exc()

    # Prime physics + OmniGraph before the main loop. On Isaac 5.1 the
    # articulation tensor view and the isaacsim.sensors.physics IMU sensor
    # both need a handful of simulation frames before they start returning
    # valid readings; without this, IsaacReadIMU / IsaacComputeOdometry
    # produce their first "no valid sensor reading" (or zero-output) tick
    # and PubIMU / PubRawTF never hit a state where they actually emit on
    # DDS. Stepping the world a few times here closes that gap before any
    # external ros2 CLI gets a chance to bind, and is cheap (~6 physics
    # frames at 60 Hz = 0.1 s wall time).
    _PRIME_FRAMES = 12
    print(f"[run_ros2] Priming {_PRIME_FRAMES} physics frames before main loop...")
    for _ in range(_PRIME_FRAMES):
        world.step(render=True)

    print("[run_ros2] Ready. Publishing ROS 2 topics. Ctrl-C to stop.")
    last_print = time.time()
    while simulation_app.is_running():
        world.step(render=True)
        if driver is not None:
            driver.step(dt=world.get_physics_dt())
        if time.time() - last_print > 5.0:
            last_print = time.time()
            print(f"[run_ros2] alive  sim_time={world.current_time:.2f}s")

    simulation_app.close()


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except Exception:
        traceback.print_exc()
        try:
            simulation_app.close()
        finally:
            sys.exit(1)
