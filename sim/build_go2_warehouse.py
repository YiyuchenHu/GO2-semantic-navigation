#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# Go2 semantic-nav MVP warehouse builder (Isaac Sim 4.1+).
#
# Thin CLI wrapper around sim/warehouse_scene.py. Boots Kit, builds the full
# scene in a fresh World, and exports a flattened USD so it can be inspected
# via File > Open in the Isaac Sim GUI. Runtime scripts (sim/run_*.py) do NOT
# need the saved USD — they import warehouse_scene directly and build fresh,
# which avoids a known libomni.graph.image.core crash in UsdContext::reopenUsd
# on Isaac 5.1 + RTX 5090.
#
# Scene: 10m x 10m enclosed room (4 solid walls), a Go2 spawn, a table and a
# chair. Person and cup were removed for MVP stability.
#
# Run (Isaac-embedded Python; headless OK):
#   cd /path/to/GO2-semantic-navigation
#   $ISAAC_SIM_ROOT/python.sh sim/build_go2_warehouse.py
#   # or with GUI:
#   $ISAAC_SIM_ROOT/python.sh sim/build_go2_warehouse.py --no-headless
#
# Output USD (saved automatically):
#   <repo>/sim/worlds/go2_warehouse_10x10.usd
# -----------------------------------------------------------------------------
import argparse
import os
import sys
import traceback


def _parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--no-headless", dest="headless", action="store_false",
                   help="Open the GUI while building (default: headless).")
    p.add_argument("--out", default=None,
                   help="Output USD path. Default: sim/worlds/go2_warehouse_10x10.usd")
    p.set_defaults(headless=True)
    args, _ = p.parse_known_args()
    return args


def _boot_simulation_app(headless: bool):
    """Import and instantiate SimulationApp with 5.x -> 4.x -> <4.1 fallback.
    Returns the live SimulationApp so caller can .update()/.close()."""
    try:
        import isaacsim  # noqa: F401  (silences a 4.1 deprecation warning)
    except Exception:
        pass

    SimulationApp = None
    try:
        from isaacsim.simulation_app import SimulationApp as _Sa5  # Isaac 5.x
        SimulationApp = _Sa5
    except Exception:
        try:
            from isaacsim import SimulationApp as _Sa4  # Isaac 4.1
            SimulationApp = _Sa4
        except Exception:  # pragma: no cover
            pass
    if SimulationApp is None:
        from omni.isaac.kit import SimulationApp  # pre-4.1 fallback
    return SimulationApp({"headless": headless})


def main() -> int:
    args = _parse_args()
    simulation_app = _boot_simulation_app(args.headless)

    # Only import after Kit is booted — these pull in omni.usd/pxr/isaacsim.*
    # which will segfault or ImportError without a live SimulationApp.
    from sim import warehouse_scene as ws  # type: ignore  # noqa: E402

    out_path = args.out or ws.default_out_path()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    ws._log(f"Building warehouse. Output: {out_path}")

    code = 0
    try:
        world = None
        try:
            world = ws.build_world()
        except Exception:
            ws._err("FATAL: build_world() failed:\n" + traceback.format_exc())
            return 2

        ws.build_full_warehouse(world)

        # Tick a few frames so lights/colliders warm up before save.
        try:
            world.reset()
            for _ in range(3):
                simulation_app.update()
        except Exception:
            ws._err("world.reset()/tick failed (non-fatal):\n" + traceback.format_exc())

        try:
            ws._stage().GetRootLayer().Export(out_path)
            ws._log(f"Saved USD to {out_path}")
        except Exception:
            ws._err("save failed:\n" + traceback.format_exc())
            code = 3

        ws._log("Done. Open the USD in Isaac Sim GUI (File > Open).")
    finally:
        try:
            simulation_app.close()
        except Exception:
            pass
    return code


if __name__ == "__main__":
    # Ensure `from sim import warehouse_scene as ws` works when launched as
    # `$ISAAC_SIM_ROOT/python.sh sim/build_go2_warehouse.py` from the repo root.
    import os as _os
    import sys as _sys
    _repo_root = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
    if _repo_root not in _sys.path:
        _sys.path.insert(0, _repo_root)

    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception:
        traceback.print_exc()
        sys.exit(1)
