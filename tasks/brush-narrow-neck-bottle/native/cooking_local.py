"""Select only the installed non-UJITSO collision cooking backend after App."""
import json

def configure(trace):
    # Called during yam_arena.scene import, after SimulationApp startup and
    # before make_env -> stage attach/cooking. No simulator advance here.
    import carb
    from omni.physx.bindings import _physx
    key = _physx.SETTING_UJITSO_COLLISION_COOKING
    if key != "/physics/cooking/ujitsoCollisionCooking":
        raise RuntimeError("Unexpected installed cooking setting contract")
    settings = carb.settings.get_settings()
    before = settings.get(key)
    settings.set_bool(key, False)
    actual = settings.get_as_bool(key)
    if actual:
        raise RuntimeError("Collision cooking setting did not persist")
    if trace is None:
        return
    trace.mkdir(parents=True, exist_ok=True)
    (trace / "cooking-backend-setting.json").write_text(json.dumps({
        "setting": key,
        "prior_setting": before,
        "requested_setting": False,
        "verified_setting": actual,
        "stage": "after_app_before_make_env",
        "scope": "collision cooking dispatch backend only",
        "geometry_or_physics_changed": False,
        "cache_deleted": False
    }, indent=2, allow_nan=False) + "\n")
