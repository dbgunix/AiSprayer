import logging
from typing import Any, Dict, Optional
from fastapi import APIRouter, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel

from apps.robot.services.robot_service import robot_service
from core.config import CONFIG_REGISTRY, sprayer_config
from services.log_service import log_service
from services.setting_service import SettingService

logger = logging.getLogger(__name__)

sys_router = APIRouter(prefix="/api/system", tags=["System"])


class SettingsUpdate(BaseModel):
    settings: Dict[str, Any]


class ResetSettingsReq(BaseModel):
    key: Optional[str] = None
    category: Optional[str] = None
    all: Optional[bool] = False


# Build lookup index for defensive validation
REGISTRY_BY_KEY: Dict[str, Dict[str, Any]] = {item["key"]: item for item in CONFIG_REGISTRY}
for item in CONFIG_REGISTRY:
    if "legacy_key" in item:
        REGISTRY_BY_KEY[item["legacy_key"]] = item


def validate_setting_entry(key: str, value: Any) -> Any:
    """Validate and type-coerce a setting entry based on configuration registry."""
    reg = REGISTRY_BY_KEY.get(key)
    if not reg:
        return value

    expected_type = reg.get("type")

    if expected_type == "number":
        try:
            num_val = float(value)
            if isinstance(reg.get("default"), int) and float(value).is_integer():
                num_val = int(value)
        except (ValueError, TypeError):
            raise HTTPException(status_code=400, detail=f"Setting '{key}' must be a valid numeric value.")

        min_v = reg.get("min")
        max_v = reg.get("max")
        if min_v is not None and num_val < min_v:
            raise HTTPException(status_code=400, detail=f"Setting '{key}' value {num_val} is less than minimum allowed ({min_v}).")
        if max_v is not None and num_val > max_v:
            raise HTTPException(status_code=400, detail=f"Setting '{key}' value {num_val} exceeds maximum allowed ({max_v}).")
        return num_val

    elif expected_type == "boolean":
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            if value.lower() in ("true", "1", "yes"):
                return True
            if value.lower() in ("false", "0", "no"):
                return False
        if isinstance(value, (int, float)):
            return bool(value)
        raise HTTPException(status_code=400, detail=f"Setting '{key}' must be a boolean value.")

    elif expected_type == "select":
        allowed_options = [opt["value"] for opt in reg.get("options", [])]
        if allowed_options and value not in allowed_options:
            raise HTTPException(status_code=400, detail=f"Invalid value for '{key}'. Allowed options: {allowed_options}.")
        return value

    elif expected_type == "vector3":
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise HTTPException(status_code=400, detail=f"Setting '{key}' must be a 3-element list [Rx, Ry, Rz].")
        try:
            return [float(v) for v in value]
        except (ValueError, TypeError):
            raise HTTPException(status_code=400, detail=f"Setting '{key}' contains non-numeric vector elements.")

    elif expected_type == "vector6":
        if not isinstance(value, (list, tuple)) or len(value) != 6:
            raise HTTPException(status_code=400, detail=f"Setting '{key}' must be a 6-element list [J1, J2, J3, J4, J5, J6].")
        try:
            return [float(v) for v in value]
        except (ValueError, TypeError):
            raise HTTPException(status_code=400, detail=f"Setting '{key}' contains non-numeric vector elements.")

    elif expected_type == "tags":
        if isinstance(value, str):
            return [t.strip() for t in value.split(",") if t.strip()]
        if isinstance(value, (list, tuple)):
            return [str(t).strip() for t in value if str(t).strip()]
        raise HTTPException(status_code=400, detail=f"Setting '{key}' must be a list of class strings.")

    elif expected_type == "string":
        return str(value).strip()

    return value


@sys_router.get("/config")
def get_config():
    """
    Get full system configuration metadata (with active values, YAML defaults, override status)
    and flat backward-compatible key-value mapping.
    """
    metadata = sprayer_config.get_config_metadata()
    flat_config: Dict[str, Any] = {}
    for item in metadata:
        flat_config[item["key"]] = item["value"]
        if "legacy_key" in item:
            flat_config[item["legacy_key"]] = item["value"]

    return {
        "status": "success",
        "metadata": metadata,
        "config": flat_config,
    }


@sys_router.post("/config")
def update_config(req: SettingsUpdate):
    """
    Update system configurations, persist to SQLite, reload in-memory cache,
    and hot-sync hardware state where appropriate.
    """
    if not req.settings:
        return {"status": "success", "message": "No settings provided to update."}

    srv = SettingService()
    validated_settings: Dict[str, Any] = {}

    # Defensive validation pass
    for k, v in req.settings.items():
        validated_v = validate_setting_entry(k, v)
        validated_settings[k] = validated_v

    # Persistence pass
    robot_speed_updated = False
    for k, v in validated_settings.items():
        srv.set_value(k, v)
        if k in ("robot.global_speed_factor", "global_speed_factor"):
            robot_speed_updated = True

    # Invalidate and reload in-memory cache
    sprayer_config.reload_db_overrides()

    # Hot-sync RobotService
    robot_service.reload_config()
    if robot_speed_updated and robot_service.is_connected():
        try:
            robot_service.set_global_speed_factor(sprayer_config.global_speed_factor)
        except Exception as e:
            logger.warning(f"Failed to hot-sync robot speed: {e}")

    return {
        "status": "success",
        "message": "System configuration updated successfully.",
        "metadata": sprayer_config.get_config_metadata(),
    }


@sys_router.post("/config/reset")
def reset_config(req: ResetSettingsReq):
    """
    Reset one setting, a whole category, or all settings back to YAML baseline defaults.
    """
    srv = SettingService()

    if req.key:
        srv.delete_value(req.key)
        # Also clean legacy alias if present
        reg = REGISTRY_BY_KEY.get(req.key)
        if reg and "legacy_key" in reg:
            srv.delete_value(reg["legacy_key"])
    elif req.category:
        srv.reset_category(req.category)
    else:
        srv.reset_all()

    # Invalidate and reload in-memory cache
    sprayer_config.reload_db_overrides()

    # Hot-sync RobotService
    robot_service.reload_config()
    if robot_service.is_connected():
        try:
            robot_service.set_global_speed_factor(sprayer_config.global_speed_factor)
        except Exception as e:
            logger.warning(f"Failed to hot-sync robot speed after reset: {e}")

    return {
        "status": "success",
        "message": "Configuration successfully reset to YAML defaults.",
        "metadata": sprayer_config.get_config_metadata(),
    }

@sys_router.websocket("/logs/ws")
async def websocket_logs(websocket: WebSocket):
    await log_service.connect(websocket)
    try:
        while True:
            # Keep the connection open and wait for client disconnect
            await websocket.receive_text()
    except WebSocketDisconnect:
        log_service.disconnect(websocket)


