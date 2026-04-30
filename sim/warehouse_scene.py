#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Go2 semantic-nav warehouse: reusable scene-building library.
#
# This module contains everything that's needed to programmatically construct
# our 10x10 m enclosed warehouse (floor, walls, Go2, table, chair) on top of a
# *fresh* Isaac Sim stage. It has no CLI and does not boot SimulationApp —
# callers must do that BEFORE importing this module, since the imports below
# require Kit to already be running.
#
# Two entry points are used by the rest of the codebase:
#
#   build_full_warehouse(world) -> None
#     Populate an existing isaacsim.core.api.World with the full scene.
#
#   Individual build_walls() / build_table() / build_chair() / build_go2_spawn()
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

TABLE_XYZ = (1.5, 1.0, 0.0)
CHAIR_XYZ = (2.7, 1.0, 0.0)
# Asset scales — EastRural ArchVis assets ship at scene-scale that reads as
# "huge" in a meter-unit world; tweak these if either prop still looks off.
TABLE_SCALE = (1.0, 1.0, 1.0)
CHAIR_SCALE = (0.5, 0.5, 0.5)
# Person and Cup intentionally removed for MVP stability:
#   * Person — NVIDIA people USDs were unstable across Isaac 4.1.
#   * Cup    — was floating mid-air because the table's exact top height is
#              asset-dependent; not worth solving for the "go to the chair" demo.
TABLE_HEIGHT_FALLBACK = 0.75


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
def build_world(stage_units_in_meters: float = 1.0) -> World:
    """Create a fresh World + ground plane + dome light. Always call this
    FIRST, before any other build_* function, because it's what actually
    allocates the USD stage that the rest of the builders mutate."""
    world = World(stage_units_in_meters=stage_units_in_meters)
    world.scene.add_default_ground_plane(z_position=FLOOR_Z)
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
            color=np.array([0.85, 0.85, 0.85]),
        )

    # West (-X) wall
    FixedCuboid(
        prim_path=f"{group_path}/wall_W",
        name="wall_W",
        position=[-half - WALL_THICK / 2, 0.0, z_mid],
        scale=[WALL_THICK, ROOM_SIZE, h],
        color=np.array([0.85, 0.85, 0.85]),
    )

    # East (+X) wall
    if ENCLOSED_ROOM or DOORWAY_WIDTH <= 0.01:
        FixedCuboid(
            prim_path=f"{group_path}/wall_E",
            name="wall_E",
            position=[half + WALL_THICK / 2, 0.0, z_mid],
            scale=[WALL_THICK, ROOM_SIZE, h],
            color=np.array([0.85, 0.85, 0.85]),
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
                color=np.array([0.85, 0.85, 0.85]),
            )
        if north_len > 0.01:
            FixedCuboid(
                prim_path=f"{group_path}/wall_E_north",
                name="wall_E_north",
                position=[half + WALL_THICK / 2,
                          DOORWAY_CENTER_Y - door_half - north_len / 2,
                          z_mid],
                scale=[WALL_THICK, north_len, h],
                color=np.array([0.85, 0.85, 0.85]),
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


def build_table(prim_path: str = "/World/Table") -> None:
    _ensure_parent_xform(prim_path)
    if _try_add_reference(prim_path, ASSET_CANDIDATES["table"]):
        _set_pose(prim_path, t_xyz=TABLE_XYZ, yaw_deg=0.0, scale=TABLE_SCALE)
        return
    h = TABLE_HEIGHT_FALLBACK
    FixedCuboid(
        prim_path=prim_path,
        name="table_fallback",
        position=[TABLE_XYZ[0], TABLE_XYZ[1], h / 2.0],
        scale=[1.2, 0.6, h],
        color=np.array([0.55, 0.35, 0.15]),
    )
    _log("Table USD not found — using 1.2x0.6x0.75 m brown box.")


def build_chair(prim_path: str = "/World/Chair") -> None:
    _ensure_parent_xform(prim_path)
    if _try_add_reference(prim_path, ASSET_CANDIDATES["chair"]):
        _set_pose(prim_path, t_xyz=CHAIR_XYZ, yaw_deg=180.0, scale=CHAIR_SCALE)
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


def build_full_warehouse(world: World) -> None:
    """Populate an already-constructed World with every warehouse prop.
    Each step is wrapped in _safe so a single bad asset can't kill the rest."""
    _safe("make_groups", lambda: (UsdGeom.Xform.Define(_stage(), "/World/Warehouse"),
                                  UsdGeom.Xform.Define(_stage(), "/World/Warehouse/Walls")))
    _safe("walls", build_walls)
    _safe("table", build_table)
    _safe("chair", build_chair)
    _safe("go2", build_go2_spawn)
