#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Go2 semantic-nav warehouse: reusable scene-building library.
#
# This module contains everything that's needed to programmatically construct
# our 10x10 m enclosed warehouse (floor, walls, Go2, table, person) on top of
# a *fresh* Isaac Sim stage. It has no CLI and does not boot SimulationApp —
# callers must do that BEFORE importing this module, since the imports below
# require Kit to already be running.
#
# Two entry points are used by the rest of the codebase:
#
#   build_full_warehouse(world) -> None
#     Populate an existing isaacsim.core.api.World with the full scene
#     (table + person + walls + Go2; chair is no longer placed by default,
#     see build_full_warehouse for how to re-enable it).
#
#   Individual build_walls() / build_table() / build_chair() /
#   build_person() / build_go2_spawn()
#     For callers that want finer control.
#
# Why a separate library (not just re-open the saved USD)?
#   On Isaac Sim 5.1 + RTX 5090 we hit a hard crash inside
#   libomni.graph.image.core.plugin.so when UsdContext::reopenUsd() runs on
#   the saved warehouse USD. Re-building fresh each run completely sidesteps
#   reopenUsd and has been stable in practice.
# -----------------------------------------------------------------------------
from __future__ import annotations

import math
import sys
import traceback
from pathlib import Path

# Kit-dependent imports. If these fail, the caller forgot to boot SimulationApp
# before importing this module.
import numpy as np
import omni.usd
from pxr import Gf, Sdf, UsdGeom, UsdLux

# Isaac Sim 5.x renamed `omni.isaac.*` to `isaacsim.*`. Try the new name first,
# fall back to 4.x. Same module then works on either install.
try:
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import FixedCuboid
    from isaacsim.core.utils.prims import create_prim, is_prim_path_valid  # noqa: F401
    from isaacsim.core.utils.stage import add_reference_to_stage
except Exception:  # pragma: no cover
    from omni.isaac.core import World
    from omni.isaac.core.objects import FixedCuboid
    from omni.isaac.core.utils.prims import create_prim, is_prim_path_valid  # noqa: F401
    from omni.isaac.core.utils.stage import add_reference_to_stage

# get_assets_root_path moved around between versions; try every known location.
try:
    from isaacsim.storage.native import get_assets_root_path  # Isaac 5.x
except Exception:
    try:
        from omni.isaac.nucleus import get_assets_root_path  # Isaac 4.1
    except Exception:  # pragma: no cover
        from omni.isaac.core.utils.nucleus import get_assets_root_path  # older


# =============================================================================
# Layout constants (meters). Edit these to move things.
# =============================================================================
ROOM_SIZE = 10.0          # 10 m x 10 m floor
FLOOR_Z = 0.0
WALL_HEIGHT = 2.5
WALL_THICK = 0.2
# Warehouse is FULLY ENCLOSED — no doorway. Flip ENCLOSED_ROOM to False and
# set DOORWAY_WIDTH / DOORWAY_CENTER_Y to re-introduce a gap on the +X wall.
ENCLOSED_ROOM = True
DOORWAY_WIDTH = 0.0
DOORWAY_CENTER_Y = 0.0

# Object positions
# Go2's nominal "standing" base-center height is ~0.40 m. Spawning AT that
# height leaves zero clearance between any of the 12 leg link colliders and
# the ground plane — one bad default joint angle in the referenced Go2 USD
# is enough to cause a first-frame penetration, which on Isaac Sim 5.1's
# TGS solver can tunnel the articulation through /World/defaultGroundPlane
# and make Phase 0 validation meaningless. 0.55 m gives a safe 15 cm
# vertical margin; gravity is then disabled on the articulation (see
# run_go2_warehouse_ros2.CmdVelDriver) so the robot stays where we set it
# instead of collapsing under its own weight with no walking policy.
GO2_SPAWN_XYZ = (-4.0, -4.0, 0.55)
GO2_SPAWN_YAW_DEG = 45.0

# Layout for Day 8+ MVP demo (chair removed from default warehouse):
#
#   Table  = NW corner, deep into the camera FOV edge so Go2 has to
#            either rotate-in-place or drive a bit to centre it. From
#            spawn pose (-4,-4) yaw=45°, world-angle ≈ 82°, depth ≈
#            7.6 m. Camera HFOV ≈ 80° = visible range [5°, 85°]; the
#            table sits right at the upper edge so it gets clipped on
#            the very first frame and snaps fully into view as Go2
#            yaws a few degrees.
#   Person = SE-of-centre, comfortably inside the camera FOV centre
#            (world-angle ≈ 25°, depth ≈ 7.2 m). YOLOE locks onto it
#            within 1-2 frames of mapping starting — `go to person`
#            smoke test reaches /target/selected within seconds.
#   Chair  = NOT spawned by default any more. Too small/thin to land
#            in 2D LiDAR + occupancy grid; was visually fine but
#            cluttered the EastRural-themed semantic map. The
#            `build_chair()` function is kept for legacy launches
#            (chair_*.launch.py) — they call it explicitly. Add
#            `_safe("chair", build_chair)` back to build_full_warehouse
#            if you specifically want chair regression coverage.
#
# Day 9 (table-cleanup): pull the table farther into the room interior.
# The previous (-3.0, 3.5) put the south edge of the 1.2×0.6 m tabletop
# only ~1.2 m from the south wall; LiDAR + YOLOE perception from oblique
# angles routinely produced 3-4 ghost table markers because the cluster
# boundary leaked into the wall (no clear free-space halo around the
# table). The new (-2.5, 2.0) leaves ≥ 2.5 m clearance to the west wall
# and ≥ 3.0 m to the south wall, so the table reads as an isolated
# island instead of a wall fragment.
#
# Distance Table↔Person check (room = 10×10, walls at ±5):
#   table  = (-2.5,  2.0)
#   person = ( 2.5, -1.0)
#   sqrt(25 + 9) ≈ 5.83 m  →  ≥ 4.0 m as the spec requires.
TABLE_XYZ = (-2.5, 2.0, 0.0)
# Kept as a constant so chair_*.launch.py + legacy tests still resolve
# the symbol; build_full_warehouse no longer places a chair by default.
CHAIR_XYZ = (3.5, -3.5, 0.0)
# Day 8+ MVP target — a person USD inside Go2 spawn-pose camera FOV so
# perception lights it up immediately. Pulled away from the table to
# ensure nav2's costmap inflation around one obstacle never overlaps
# the other's approach ring.
PERSON_XYZ = (2.5, -1.0, 0.0)
# Yaw of the person USD. Isaac people USDs face +X by default; rotating
# 180° turns the silhouette towards the spawn camera so YOLOE sees the
# canonical COCO-`person` upright frontal pose with helmet + arms
# rather than the back of a head. Adjust if the model orientation
# convention differs from -X facing.
PERSON_YAW_DEG = 180.0
# Asset scales — EastRural ArchVis assets ship at scene-scale that reads as
# "huge" in a meter-unit world; tweak these if either prop still looks off.
TABLE_SCALE = (1.0, 1.0, 1.0)
CHAIR_SCALE = (0.5, 0.5, 0.5)
# Isaac people USDs ship at human scale (~1.8 m tall) so leave at 1.0.
PERSON_SCALE = (1.0, 1.0, 1.0)
# Day 9 hot-fix — replace the legacy full-body cuboid with a
# *constellation* of thin leg-level columns. The previous 0.6 x 0.6 x
# 1.7 m cuboid wrapped the human silhouette so completely that YOLOE
# saw the dim grey proxy first and the COCO-class score for "person"
# collapsed. The new design:
#   * caps the proxy height at ~0.8 m so the head/torso/arms/face
#     remain unobstructed in the camera frustum,
#   * keeps each individual column thin (0.10 m square),
#   * spreads three columns inside a ~0.30 m diameter cluster around
#     the legs/feet so PointCloud2 BFS clustering (default tolerance
#     0.20 m) glues them into one cluster — enough points for
#     `pointcloud_min_cluster_points=5` even when LiDAR sees the
#     person from oblique angles.
# The fallback "person USD missing" path still uses a single full-body
# cuboid (it has nothing else to render).
PERSON_PROXY_RADIUS_M = 0.30        # legacy fallback diameter
PERSON_PROXY_HEIGHT_M = 1.70        # legacy fallback height
# Day 9 leg-level constellation parameters.
PERSON_PROXY_COL_WIDTH_M = 0.10     # square footprint of each column
PERSON_PROXY_COL_HEIGHT_M = 0.80    # tall enough to span LiDAR scan
                                    # height but well below the head
# Three offsets in the person's local XY frame. The triangle is
# centred on the person's feet and sized so the centroids sit on a
# 0.18 m radius — well below the cluster tolerance. They are NOT
# placed in front of the body so the camera silhouette stays clean.
PERSON_PROXY_COL_OFFSETS = (
    (-0.12, -0.04),   # left foot, slightly back
    (+0.12, -0.04),   # right foot, slightly back
    (+0.00, +0.16),   # behind/between feet
)
TABLE_HEIGHT_FALLBACK = 0.75

# Wall colour: light grey [0.85, 0.85, 0.85] historically. With EastRural's
# light-wood Chair/Table USDs the wall+chair edge contrast was so low YOLOE
# couldn't reliably segment the chair from a near-white background. This
# darker industrial blue-grey gives ~0.5 luminance gap to the wood props.
WALL_COLOR = np.array([0.20, 0.30, 0.45])

# Yaw of the EastRural chair USD when placed in the warehouse. The asset's
# native +X axis points "out the back of the chair" — at yaw=180° the chair
# faces -X, which from Go2's spawn-side approach means YOLOE sees mostly the
# back panel (large flat wood surface, low feature density). Yaw=90° rotates
# the chair so its side profile (legs + seat + backrest silhouette) is what
# the camera sees first — these are the visual cues the COCO 'chair' class
# was trained on, and detection confidence jumps noticeably.
CHAIR_YAW_DEG = 90.0


# =============================================================================
# Asset candidates (best-effort — NVIDIA renames these between Isaac versions).
# =============================================================================
#
# Path conventions for ASSET_CANDIDATES entries:
#   * Relative paths (e.g. "Isaac/Robots/Unitree/Go2/go2.usd") are joined
#     under get_assets_root_path() — the Isaac assets root.
#   * Absolute URLs (omniverse://..., http(s)://..., file://..., or "/abs/path")
#     are used as-is. This is how you paste in a path you copied from Isaac
#     Sim's NVIDIA Assets browser via right-click > "Copy URL Link".
#
ASSET_CANDIDATES = {
    "go2":   ["Isaac/Robots/Unitree/Go2/go2.usd"],
    # Table: EastRural dining table to match the chair's set (user-confirmed S3 URL).
    "table": [
        "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Furniture/DiningSets/EastRural/EastRural_Table.usd",
    ],
    # Chair: EastRural_Chair pairs with EastRural_Table (they ship as one DiningSet).
    # The chair ships oversized; CHAIR_SCALE (default 0.5) shrinks it.
    "chair": [
        "http://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Residential/Furniture/DiningSets/EastRural/EastRural_Chair.usd",
    ],
    # Person: Day 8+ MVP target. The user singled out
    # `male_adult_construction_05_new.usd`; Isaac Sim 5.x typically
    # mounts the people pack under "Isaac/People/Characters/<NAME>".
    # We try several layouts so the same scene script works on
    # Isaac 4.1 / 4.5 / 5.x without manual edits. If none resolve,
    # build_person() falls back to a non-photo-realistic primitive
    # body — perception will not flag it as `person` in that case,
    # but the scene still loads cleanly.
    "person": [
        "Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd",
        "Isaac/People/Characters/male_adult_construction_05/male_adult_construction_05.usd",
        "Isaac/Samples/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd",
        "NVIDIA/Assets/Isaac/5.0/Isaac/People/Characters/male_adult_construction_05_new/male_adult_construction_05_new.usd",
        # Generic NVIDIA people pack fallbacks — any one of these
        # gives YOLOE a strong COCO-`person` signal.
        "Isaac/People/Characters/F_Business_02/F_Business_02.usd",
        "Isaac/People/Characters/M_Business_02/M_Business_02.usd",
        "Isaac/People/Characters/original_male_adult_construction_05/original_male_adult_construction_05.usd",
    ],
}


# Where the flattened USD gets saved by build_go2_warehouse.py. Exported here
# so callers that DO want the saved file know the canonical location.
def default_out_path() -> str:
    here = Path(__file__).resolve().parent
    return str(here / "worlds" / "go2_warehouse_10x10.usd")


# =============================================================================
# Helpers
# =============================================================================
def _log(msg: str) -> None:
    print(f"[warehouse] {msg}", flush=True)


def _err(msg: str) -> None:
    print(f"[warehouse][ERR] {msg}", flush=True, file=sys.stderr)


def _safe(step_name: str, fn, *a, **kw):
    """Run a builder step; log full traceback on failure but keep going."""
    try:
        fn(*a, **kw)
        return True
    except Exception:
        _err(f"step {step_name!r} failed, continuing:\n{traceback.format_exc()}")
        return False


def _stage():
    return omni.usd.get_context().get_stage()


def _ensure_xform(prim_path: str) -> None:
    if not is_prim_path_valid(prim_path):
        UsdGeom.Xform.Define(_stage(), prim_path)


def _ensure_parent_xform(prim_path: str) -> None:
    parent = str(Path(prim_path).parent)
    if parent and parent != "/" and not is_prim_path_valid(parent):
        _ensure_parent_xform(parent)
        UsdGeom.Xform.Define(_stage(), parent)


def _set_pose(prim_path: str, t_xyz=(0, 0, 0), yaw_deg: float = 0.0,
              scale=(1.0, 1.0, 1.0)) -> None:
    """Set translate / rotate(Z) / scale on an Xformable prim, reusing any
    *existing* xform ops so precision (double3/float3) and op ordering
    coming from a referenced USD are preserved (Isaac samples use double3)."""
    prim = _stage().GetPrimAtPath(prim_path)
    if not prim.IsValid():
        _err(f"_set_pose: missing prim {prim_path}")
        return
    xf = UsdGeom.Xformable(prim)
    existing = {op.GetOpName(): op for op in xf.GetOrderedXformOps()}

    def _set_vec(op, vec_xyz):
        t = str(op.GetAttr().GetTypeName()).lower()
        if "double" in t:
            op.Set(Gf.Vec3d(*vec_xyz))
        else:
            op.Set(Gf.Vec3f(*vec_xyz))

    if "xformOp:translate" in existing:
        _set_vec(existing["xformOp:translate"], t_xyz)
    else:
        xf.AddTranslateOp().Set(Gf.Vec3d(*t_xyz))

    if "xformOp:rotateZ" in existing:
        existing["xformOp:rotateZ"].Set(float(yaw_deg))
    elif "xformOp:orient" in existing:
        h = math.radians(float(yaw_deg)) * 0.5
        cos_h, sin_h = math.cos(h), math.sin(h)
        attr = existing["xformOp:orient"].GetAttr()
        if attr.GetTypeName() == Sdf.ValueTypeNames.Quatd:
            attr.Set(Gf.Quatd(cos_h, 0.0, 0.0, sin_h))
        else:
            attr.Set(Gf.Quatf(cos_h, 0.0, 0.0, sin_h))
    elif "xformOp:rotateXYZ" in existing:
        t = str(existing["xformOp:rotateXYZ"].GetAttr().GetTypeName()).lower()
        if "double" in t:
            existing["xformOp:rotateXYZ"].Set(Gf.Vec3d(0.0, 0.0, float(yaw_deg)))
        else:
            existing["xformOp:rotateXYZ"].Set(Gf.Vec3f(0.0, 0.0, float(yaw_deg)))
    else:
        xf.AddRotateZOp().Set(float(yaw_deg))

    if "xformOp:scale" in existing:
        _set_vec(existing["xformOp:scale"], scale)
    else:
        xf.AddScaleOp().Set(Gf.Vec3f(*scale))


def _remove_prim(prim_path: str) -> None:
    try:
        _stage().RemovePrim(prim_path)
    except Exception:
        pass


def _is_absolute_asset_url(s: str) -> bool:
    if not s:
        return False
    return (
        s.startswith("omniverse://")
        or s.startswith("http://")
        or s.startswith("https://")
        or s.startswith("file://")
        or s.startswith("/")
    )


def _try_add_reference(prim_path: str, rel_candidates) -> bool:
    """Try each candidate USD; True on success. Candidates may be either
    paths relative to get_assets_root_path() or absolute URLs. A successful
    'referenced' prim must actually compose children — a dead URL still
    creates the prim but its composition is empty, which we reject. When
    *all* candidates fail, the prim is removed from the stage so the caller
    can create a clean fallback primitive at the same path."""
    try:
        root = get_assets_root_path()
    except Exception:
        root = None
    if not root and not any(_is_absolute_asset_url(c) for c in rel_candidates):
        _log("Nucleus asset root unavailable and no absolute URLs — using primitives.")
        return False
    for rel in rel_candidates:
        if _is_absolute_asset_url(rel):
            url = rel
        else:
            if not root:
                _log(f"Skipping relative ref {rel!r}: no Nucleus asset root.")
                continue
            url = f"{root}/{rel}"
        try:
            _ensure_parent_xform(prim_path)
            add_reference_to_stage(usd_path=url, prim_path=prim_path)
            prim = _stage().GetPrimAtPath(prim_path)
            if not (prim.IsValid() and prim.GetTypeName()):
                continue
            children = list(prim.GetChildren())
            if len(children) == 0:
                try:
                    prim.GetReferences().ClearReferences()
                except Exception:
                    pass
                _log(f"ref {rel} composed empty prim; dropping and trying next.")
                continue
            _log(f"Referenced {rel} -> {prim_path}")
            return True
        except Exception as exc:
            _log(f"ref failed for {rel}: {exc}")
            continue
    _remove_prim(prim_path)
    return False


# =============================================================================
# Builders
# =============================================================================
def build_world(
    stage_units_in_meters: float = 1.0,
    physics_dt: float = 1.0 / 200.0,
    rendering_dt: float = 1.0 / 60.0,
) -> World:
    """Create a fresh World + ground plane + dome light. Always call this
    FIRST, before any other build_* function, because it's what actually
    allocates the USD stage that the rest of the builders mutate.

    Why physics_dt=1/200s by default:
        Isaac Lab's stock `Isaac-Velocity-Flat-Unitree-Go2` task trains at
        physics 200 Hz with action decimation=4 (policy 50 Hz). The exported
        TorchScript policy assumes the same. Running PhysX at the Isaac Sim
        default (1/60s = 60 Hz) would put the policy at 15 Hz under the
        same decimation, which is ~3.3x slower than training and visibly
        unstable (Go2 falls over within the first second). Keeping render
        at 60 Hz preserves visual cadence without wasting GPU on shader
        recompiles. Override these via kwargs if you have a checkpoint
        trained with different settings."""
    world = World(
        stage_units_in_meters=stage_units_in_meters,
        physics_dt=physics_dt,
        rendering_dt=rendering_dt,
    )
    world.scene.add_default_ground_plane(z_position=FLOOR_Z)
    # Align ground friction with the IsaacLab Velocity training distribution
    # (velocity_env_cfg.py:53 → static_friction=1.0, dynamic_friction=1.0).
    # The default ground plane ships with whatever PhysicsMaterial Isaac Sim
    # picks for `defaultGroundPlane` — historically friction≈0.5 — which is
    # noticeably slipperier than what the policy was trained on and lets a
    # marginal trot accumulate into a fall over a few seconds.
    try:
        from pxr import UsdPhysics, UsdShade, Sdf as _Sdf  # noqa: F401
        stage = _stage()
        gp_path = "/World/defaultGroundPlane"
        for prim in stage.Traverse():
            p = str(prim.GetPath())
            if not p.startswith(gp_path):
                continue
            if prim.GetTypeName() == "PhysicsMaterial":
                mat = UsdPhysics.MaterialAPI(prim)
                try:
                    mat.CreateStaticFrictionAttr().Set(1.0)
                    mat.CreateDynamicFrictionAttr().Set(1.0)
                    mat.CreateRestitutionAttr().Set(0.0)
                except Exception:
                    pass
    except Exception as exc:
        _log(f"ground friction align failed ({exc}); using default.")
    _ensure_parent_xform("/World/Lights/Dome")
    dome = UsdLux.DomeLight.Define(_stage(), "/World/Lights/Dome")
    try:
        dome.CreateIntensityAttr(1500.0)
    except Exception:
        try:
            dome.GetPrim().CreateAttribute(
                "inputs:intensity", Sdf.ValueTypeNames.Float).Set(1500.0)
        except Exception as exc:
            _log(f"dome intensity set failed ({exc}); keeping default.")
    return world


def build_walls(group_path: str = "/World/Warehouse/Walls") -> None:
    _ensure_parent_xform(group_path)
    _ensure_xform(group_path)
    half = ROOM_SIZE / 2.0
    h = WALL_HEIGHT
    z_mid = h / 2.0 + 0.001

    # North (-Y) and South (+Y): long along X
    for name, y in (("wall_S", +half + WALL_THICK / 2),
                    ("wall_N", -half - WALL_THICK / 2)):
        FixedCuboid(
            prim_path=f"{group_path}/{name}",
            name=name,
            position=[0.0, y, z_mid],
            scale=[ROOM_SIZE + 2 * WALL_THICK, WALL_THICK, h],
            color=WALL_COLOR,
        )

    # West (-X) wall
    FixedCuboid(
        prim_path=f"{group_path}/wall_W",
        name="wall_W",
        position=[-half - WALL_THICK / 2, 0.0, z_mid],
        scale=[WALL_THICK, ROOM_SIZE, h],
        color=WALL_COLOR,
    )

    # East (+X) wall
    if ENCLOSED_ROOM or DOORWAY_WIDTH <= 0.01:
        FixedCuboid(
            prim_path=f"{group_path}/wall_E",
            name="wall_E",
            position=[half + WALL_THICK / 2, 0.0, z_mid],
            scale=[WALL_THICK, ROOM_SIZE, h],
            color=WALL_COLOR,
        )
        _log(f"Walls built. Room is FULLY ENCLOSED, height={h} m")
    else:
        door_half = DOORWAY_WIDTH / 2.0
        south_len = half - (DOORWAY_CENTER_Y + door_half)
        north_len = half + (DOORWAY_CENTER_Y - door_half)
        if south_len > 0.01:
            FixedCuboid(
                prim_path=f"{group_path}/wall_E_south",
                name="wall_E_south",
                position=[half + WALL_THICK / 2,
                          DOORWAY_CENTER_Y + door_half + south_len / 2,
                          z_mid],
                scale=[WALL_THICK, south_len, h],
                color=WALL_COLOR,
            )
        if north_len > 0.01:
            FixedCuboid(
                prim_path=f"{group_path}/wall_E_north",
                name="wall_E_north",
                position=[half + WALL_THICK / 2,
                          DOORWAY_CENTER_Y - door_half - north_len / 2,
                          z_mid],
                scale=[WALL_THICK, north_len, h],
                color=WALL_COLOR,
            )
        _log(f"Walls built. Doorway on +X wall, y=[{DOORWAY_CENTER_Y - door_half:.2f},"
             f" {DOORWAY_CENTER_Y + door_half:.2f}], height={h} m")


def build_go2_spawn(prim_path: str = "/World/Go2") -> None:
    """Reference Go2 USD if available; else create a placeholder so downstream
    nodes can attach a robot later. Root prim path is stable."""
    _ensure_parent_xform(prim_path)
    ok = _try_add_reference(prim_path, ASSET_CANDIDATES["go2"])
    if not ok:
        _ensure_xform(prim_path)
        cap_path = f"{prim_path}/placeholder_body"
        try:
            cap_prim = UsdGeom.Capsule.Define(_stage(), cap_path)
            cap_prim.GetAxisAttr().Set("X")
            cap_prim.GetHeightAttr().Set(0.35)
            cap_prim.GetRadiusAttr().Set(0.12)
            cap_prim.GetDisplayColorAttr().Set([(0.2, 0.6, 1.0)])
        except Exception:
            FixedCuboid(prim_path=cap_path, name="go2_placeholder",
                        position=[0, 0, 0.10], scale=[0.45, 0.22, 0.22],
                        color=np.array([0.2, 0.6, 1.0]))
        _log("Go2 asset not found — created a placeholder under /World/Go2.")
    _set_pose(prim_path, t_xyz=GO2_SPAWN_XYZ, yaw_deg=GO2_SPAWN_YAW_DEG)


def build_table(
    prim_path: str = "/World/Table",
    *,
    add_collision_proxy: bool = True,
    semantic_proxy_visible_to_lidar: bool = True,
) -> None:
    """Spawn the warehouse table.

    Day 9 Task 4 — when ``add_collision_proxy=True`` we drop a low,
    flat cuboid the size of the tabletop *just under* the table
    surface. Real tables ship with thin legs that LiDAR misses; this
    proxy gives the LiDAR + Nav2 costmap a clean rectangular
    silhouette right where the visual table sits. With
    ``semantic_proxy_visible_to_lidar=True`` (default) the proxy is
    dim grey + low-alpha — visually subtle to the operator but
    visible to RTX raycasting / camera depth.
    """
    _ensure_parent_xform(prim_path)
    if _try_add_reference(prim_path, ASSET_CANDIDATES["table"]):
        _set_pose(prim_path, t_xyz=TABLE_XYZ, yaw_deg=0.0, scale=TABLE_SCALE)
        used_usd = True
    else:
        # Day 9+ Phase C — fatter, darker fallback box for stronger
        # YOLO-visible silhouette + 4 explicit leg cuboids so the
        # primitive shape reads as "table" rather than "block". Light
        # wood floor → near-black walnut tabletop gives ≥0.4 luminance
        # gap for YOLO11s prompts ("table" / "desk").
        h = TABLE_HEIGHT_FALLBACK
        # Tabletop slab — slightly thicker (0.08m vs the legacy 0m
        # implicit value) and saturated dark-walnut so the visible
        # edge against the lighter floor is unambiguous.
        top_thickness = 0.08
        FixedCuboid(
            prim_path=prim_path,
            name="table_fallback",
            position=[TABLE_XYZ[0], TABLE_XYZ[1], h - top_thickness / 2.0],
            scale=[1.20, 0.70, top_thickness],
            color=np.array([0.30, 0.18, 0.08]),
        )
        # Four leg cuboids beneath the corners. Real-world dining
        # tables have ~5–7 cm legs; we go slightly thicker (0.10 m) so
        # the depth camera + LiDAR grab them reliably even from
        # oblique angles. Half the table's footprint (1.20 × 0.70)
        # ⇒ legs at ±0.50, ±0.27 with 0.10 m square cross-section.
        leg_path_root = f"{prim_path}/legs"
        _ensure_parent_xform(leg_path_root)
        leg_h = h - top_thickness
        leg_w = 0.10
        leg_dx = 0.55 - leg_w / 2.0
        leg_dy = 0.30 - leg_w / 2.0
        for li, (dx, dy) in enumerate(
            ((+leg_dx, +leg_dy), (-leg_dx, +leg_dy),
             (+leg_dx, -leg_dy), (-leg_dx, -leg_dy))
        ):
            FixedCuboid(
                prim_path=f"{leg_path_root}/leg_{li}",
                name=f"table_fallback_leg_{li}",
                position=[
                    TABLE_XYZ[0] + dx,
                    TABLE_XYZ[1] + dy,
                    leg_h / 2.0,
                ],
                scale=[leg_w, leg_w, leg_h],
                color=np.array([0.25, 0.15, 0.06]),
            )
        _log(
            "Table USD not found — fallback: 1.20x0.70x0.08m walnut tabletop "
            "+ 4 x 0.10x0.10m legs (high-contrast YOLO friendly)."
        )
        used_usd = False

    if add_collision_proxy:
        # Footprint dimensions of the table proxy. Tighter than the
        # visual table so we don't bake costmap inflation into a
        # larger-than-real obstacle. Day 9+ Phase C — bumped from
        # 0.10 m to 0.14 m (slightly thicker tabletop) and the colour
        # darkened + opacity raised so the camera sees a saturated
        # brown stripe across the visual tabletop instead of a near-
        # transparent wash. The 0.14 m depth gives the LiDAR more
        # vertical extent at the table's height while keeping us
        # well below the 0.20 m default Nav2 inflation radius.
        proxy_path = f"{prim_path}/collision_proxy"
        h_top = TABLE_HEIGHT_FALLBACK if not used_usd else 0.75
        proxy_h = 0.14
        FixedCuboid(
            prim_path=proxy_path,
            name="table_collision_proxy",
            position=[
                TABLE_XYZ[0], TABLE_XYZ[1],
                h_top - proxy_h / 2.0,
            ],
            scale=[1.10, 0.55, proxy_h],
            color=np.array([0.30, 0.18, 0.08]),
        )
        if not semantic_proxy_visible_to_lidar:
            try:
                from pxr import UsdGeom as _UsdGeom
                UsdGeom.Imageable(
                    _stage().GetPrimAtPath(proxy_path)
                ).MakeInvisible()
            except Exception:
                pass
            visibility_note = "hidden (perception-only test mode)"
        else:
            try:
                prim = _stage().GetPrimAtPath(proxy_path)
                imageable = UsdGeom.Imageable(prim)
                imageable.MakeVisible()
                gprim = UsdGeom.Gprim(prim)
                gprim.GetDisplayColorAttr().Set([(0.30, 0.18, 0.08)])
                try:
                    # Day 9+ Phase C — was 0.25 (faint wash), now 0.55
                    # (dominant brown stripe) so YOLO can pick the
                    # contrast against the floor reliably.
                    gprim.GetDisplayOpacityAttr().Set([0.55])
                except Exception:
                    pass
            except Exception:
                pass
            visibility_note = (
                "LiDAR-visible + YOLO-friendly (dark walnut, alpha 0.55)"
            )
        _log(
            f"Table at {TABLE_XYZ} with tabletop collision proxy "
            f"(1.10x0.55x{proxy_h:.2f}m at z={h_top - proxy_h/2.0:.2f}m, "
            f"{visibility_note})."
        )


def build_chair(prim_path: str = "/World/Chair") -> None:
    _ensure_parent_xform(prim_path)
    if _try_add_reference(prim_path, ASSET_CANDIDATES["chair"]):
        _set_pose(prim_path, t_xyz=CHAIR_XYZ, yaw_deg=CHAIR_YAW_DEG,
                  scale=CHAIR_SCALE)
        return
    seat_h = 0.45
    FixedCuboid(
        prim_path=prim_path,
        name="chair_fallback",
        position=[CHAIR_XYZ[0], CHAIR_XYZ[1], seat_h / 2.0],
        scale=[0.5, 0.5, seat_h],
        color=np.array([0.15, 0.15, 0.15]),
    )
    _log("Chair USD not found — using 0.5x0.5x0.45 m black cube.")


def build_person(
    prim_path: str = "/World/Person",
    *,
    add_collision_proxy: bool = True,
    semantic_proxy_visible_to_lidar: bool = True,
) -> None:
    """Spawn the MVP-target person USD inside Go2 spawn-camera FOV.

    Notes
    -----
    * The user singled out ``male_adult_construction_05_new.usd``;
      ASSET_CANDIDATES["person"] tries that path first and falls back
      to NVIDIA people-pack defaults that ship in Isaac 4.1+.
    * Isaac people USDs render the body but their collision geometry
      is inconsistent — some packs ship a full mesh, others ship none.
      ``add_collision_proxy`` overlays a vertical FixedCuboid cylinder
      sized to the silhouette so 2D LiDAR sees a stable obstacle and
      nav2's costmap inflation halo treats the person as a fixed
      pillar (matching the semantic-marker centroid). Default True;
      disable via ``add_semantic_collision_proxies:=false`` on the
      builder if a perception-only test wants to walk through the
      person without bumping the costmap.
    """
    _ensure_parent_xform(prim_path)
    body_path = f"{prim_path}/body"
    if _try_add_reference(body_path, ASSET_CANDIDATES["person"]):
        _set_pose(body_path, t_xyz=(0.0, 0.0, 0.0), yaw_deg=PERSON_YAW_DEG,
                  scale=PERSON_SCALE)
        _set_pose(prim_path, t_xyz=PERSON_XYZ, yaw_deg=0.0)
        person_label = "Person USD"
    else:
        # Photo-realism fallback. YOLOE will not classify this as
        # `person` (it's a navy-blue cuboid), but the scene still
        # loads and the costmap inflation halo still makes sense.
        FixedCuboid(
            prim_path=body_path,
            name="person_fallback_body",
            position=[
                PERSON_XYZ[0], PERSON_XYZ[1],
                PERSON_PROXY_HEIGHT_M / 2.0,
            ],
            scale=[
                PERSON_PROXY_RADIUS_M * 2.0,
                PERSON_PROXY_RADIUS_M * 2.0,
                PERSON_PROXY_HEIGHT_M,
            ],
            color=np.array([0.10, 0.20, 0.55]),
        )
        person_label = "person fallback cuboid"
        _log(
            f"Person USD not found — using "
            f"{PERSON_PROXY_RADIUS_M * 2:.2f}m wide x "
            f"{PERSON_PROXY_HEIGHT_M:.2f}m tall navy-blue stand-in. "
            f"YOLOE will NOT detect this as `person`; mount the "
            f"people pack under Isaac/People to enable detection."
        )
    if add_collision_proxy:
        # Day 9 hot-fix — leg-level proxy *constellation* (replaces
        # the legacy full-body cuboid). Goal: the human USD silhouette
        # stays uncovered for RGB/YOLOE while LiDAR + nav2 still see
        # an obstacle right where the semantic marker says it is.
        #
        # Design:
        #   * 3 thin (0.10x0.10 m) columns, each ~0.80 m tall,
        #     arranged in a triangle on a ~0.18 m radius around the
        #     person's feet (PERSON_PROXY_COL_OFFSETS). LiDAR voxel-
        #     grid BFS (cluster_tolerance_m=0.20) glues them into one
        #     cluster. The trio passes
        #     ``pointcloud_min_cluster_points=5`` even from oblique
        #     angles.
        #   * Tops sit at ~0.80 m — below the typical YOLOE-relevant
        #     person torso/arm/face region (~1.0–1.7 m), so the
        #     camera frustum sees the person USD's full upper body.
        #   * Visibility is sensor-conditional via
        #     ``semantic_proxy_visible_to_lidar``:
        #       True  (default) → visible but dim grey + low alpha.
        #       False           → UsdGeom.MakeInvisible (legacy mode,
        #                          --invisible-collision-proxies CLI).
        proxy_paths: list[str] = []
        for i, (dx, dy) in enumerate(PERSON_PROXY_COL_OFFSETS):
            col_path = f"{prim_path}/leg_proxy_{i}"
            FixedCuboid(
                prim_path=col_path,
                name=f"person_leg_proxy_{i}",
                position=[
                    PERSON_XYZ[0] + dx,
                    PERSON_XYZ[1] + dy,
                    PERSON_PROXY_COL_HEIGHT_M / 2.0,
                ],
                scale=[
                    PERSON_PROXY_COL_WIDTH_M,
                    PERSON_PROXY_COL_WIDTH_M,
                    PERSON_PROXY_COL_HEIGHT_M,
                ],
                color=np.array([0.30, 0.30, 0.30]),
            )
            proxy_paths.append(col_path)

        if not semantic_proxy_visible_to_lidar:
            # Legacy path — fully invisible. RTX camera + LiDAR will
            # both miss the columns (use only for perception-only
            # stress tests).
            for p in proxy_paths:
                try:
                    UsdGeom.Imageable(
                        _stage().GetPrimAtPath(p)
                    ).MakeInvisible()
                except Exception:
                    pass
            visibility_note = (
                "hidden (invisible to LiDAR + camera, legacy)"
            )
        else:
            # Subtle but LiDAR-visible. RTX raycasts hit a solid
            # prim regardless of display opacity; RGB blends in dim
            # alpha so YOLOE doesn't latch onto the proxy.
            for p in proxy_paths:
                try:
                    prim = _stage().GetPrimAtPath(p)
                    UsdGeom.Imageable(prim).MakeVisible()
                    gprim = UsdGeom.Gprim(prim)
                    gprim.GetDisplayColorAttr().Set(
                        [(0.30, 0.30, 0.30)]
                    )
                    try:
                        gprim.GetDisplayOpacityAttr().Set([0.10])
                    except Exception:
                        pass
                except Exception:
                    pass
            visibility_note = (
                "LiDAR-visible (3-col constellation, dim+translucent)"
            )
        _log(
            f"{person_label} placed at {PERSON_XYZ} with "
            f"{len(PERSON_PROXY_COL_OFFSETS)}-column leg proxy "
            f"(each {PERSON_PROXY_COL_WIDTH_M:.2f}m x "
            f"{PERSON_PROXY_COL_HEIGHT_M:.2f}m tall, "
            f"{visibility_note})."
        )
    else:
        _log(f"{person_label} placed at {PERSON_XYZ} (no collision proxy).")


def build_full_warehouse(
    world: World,
    *,
    add_semantic_collision_proxies: bool = True,
    semantic_proxy_visible_to_lidar: bool = True,
) -> None:
    """Populate an already-constructed World with every warehouse prop.

    Parameters
    ----------
    world :
        The World previously created by :func:`build_world`.
    add_semantic_collision_proxies :
        When True (default), props that don't reliably ship collision
        geometry (currently: person) get a hidden cylinder/cuboid
        overlay so 2D LiDAR + nav2 inflation see a stable obstacle
        right where the semantic marker says it is. Set False for
        perception-only stress tests where you specifically want the
        robot to be able to walk *through* the person.

    Each builder step is wrapped in :func:`_safe` so a single bad
    asset can't kill the rest.
    """
    _safe("make_groups", lambda: (UsdGeom.Xform.Define(_stage(), "/World/Warehouse"),
                                  UsdGeom.Xform.Define(_stage(), "/World/Warehouse/Walls")))
    _safe("walls", build_walls)
    _safe(
        "table",
        lambda: build_table(
            add_collision_proxy=add_semantic_collision_proxies,
            semantic_proxy_visible_to_lidar=(
                semantic_proxy_visible_to_lidar
            ),
        ),
    )
    # Chair intentionally omitted from the default MVP warehouse.
    _safe(
        "person",
        lambda: build_person(
            add_collision_proxy=add_semantic_collision_proxies,
            semantic_proxy_visible_to_lidar=(
                semantic_proxy_visible_to_lidar
            ),
        ),
    )
    _safe("go2", build_go2_spawn)
