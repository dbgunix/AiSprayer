from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, List
import os
import math
import time
import shutil
import logging
import cv2
import numpy as np
from core.utils.fast_yaml import fast_yaml_load, fast_yaml_dump
from core.config import sprayer_config
from core.hardware.robot.base_driver import RobotPose, is_spraying_on
from apps.camera.services.camera_service import camera_service
from apps.robot.services.robot_service import robot_service
from apps.interactive.sam_service import sam_service
from apps.interactive.reconstruction_service import reconstruction_service
from apps.interactive.manual_path_service import manual_path_service
from apps.interactive.auto_path_service import auto_path_service, AutoPathServiceError
from apps.interactive.path_verification_service import path_verification_service, get_default_poi_tolerance_rpy_deg


router = APIRouter(prefix="/api/interactive", tags=["Interactive"])
logger = logging.getLogger(__name__)

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))
TEMPLATE_GROUP_DIR = os.path.join(PROJECT_ROOT, "data", "template_group")

os.makedirs(TEMPLATE_GROUP_DIR, exist_ok=True)

class CreateTemplateRequest(BaseModel):
    name: str | None = None

@router.get("/templates")
def list_templates():
    if not os.path.exists(TEMPLATE_GROUP_DIR):
        return {"templates": []}
    
    templates = []
    for item in os.listdir(TEMPLATE_GROUP_DIR):
        item_path = os.path.join(TEMPLATE_GROUP_DIR, item)
        if os.path.isdir(item_path):
            templates.append(item)
    
    # Sort by descending order (newest first usually)
    templates.sort(reverse=True)
    return {"templates": templates}

@router.post("/templates")
def create_template(req: CreateTemplateRequest):
    name = req.name
    if not name:
        name = time.strftime("%Y%m%d_%H%M%S")
    
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if os.path.exists(template_path):
        raise HTTPException(status_code=400, detail="Template name already exists")
        
    try:
        os.makedirs(template_path, exist_ok=True)
        logger.info(f"Created new template directory: {template_path}")
        return {"message": "Template created successfully", "name": name}
    except Exception as e:
        logger.error(f"Failed to create template '{name}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/templates/{name}/files")
def list_template_files(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    files = []
    for item in os.listdir(template_path):
        item_path = os.path.join(template_path, item)
        if os.path.isfile(item_path):
            size = os.path.getsize(item_path)
            ctime = os.path.getctime(item_path)
            files.append({"name": item, "size": size, "ctime": ctime})
            
    files.sort(key=lambda x: x["ctime"], reverse=True)
    return {"files": files}

@router.delete("/templates/{name}")
def delete_template(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
    try:
        shutil.rmtree(template_path)
        logger.info(f"Deleted template directory: {template_path}")
        return {"message": f"Template '{name}' deleted successfully"}
    except Exception as e:
        logger.error(f"Failed to delete template '{name}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.delete("/templates/{name}/files/{filename}")
def delete_template_file(name: str, filename: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
    
    file_path = os.path.join(template_path, filename)
    # Prevent path traversal
    if not os.path.abspath(file_path).startswith(os.path.abspath(template_path)):
        raise HTTPException(status_code=400, detail="Invalid filename")
        
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"File '{filename}' not found")
        
    try:
        if os.path.isfile(file_path):
            os.remove(file_path)
            logger.info(f"Deleted template file: {file_path}")
            return {"message": f"File '{filename}' deleted successfully"}
        elif os.path.isdir(file_path):
            shutil.rmtree(file_path)
            logger.info(f"Deleted template directory: {file_path}")
            return {"message": f"Directory '{filename}' deleted successfully"}
    except Exception as e:
        logger.error(f"Failed to delete file '{filename}' in '{name}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

def is_raw_capture_file(filename: str) -> bool:
    """
    Check if a file belongs to original raw photo capture:
    - Color image (e.g. scan.color.jpg, scan.jpg, scan.png)
    - Depth map (e.g. scan.depth.png, scan.depth.npy, scan.depth.raw)
    - Parameters (e.g. scan.params.yaml, scan.info.yaml, scan.params.json)
    Masks, meshes, point clouds, paths, and verification reports are excluded.
    """
    lower = filename.lower()
    # Generated artifacts must be cleaned
    if any(k in lower for k in ["mask", "path", "mesh", "report"]):
        return False
    # Color photo
    if lower in ("scan.jpg", "scan.jpeg", "scan.png"):
        return True
    if "color" in lower and any(lower.endswith(ext) for ext in [".jpg", ".jpeg", ".png", ".bmp", ".webp"]):
        return True
    # Depth photo / data
    if "depth" in lower and any(lower.endswith(ext) for ext in [".png", ".npy", ".raw", ".tiff", ".bin", ".bmp"]):
        return True
    # Parameter files
    if ("param" in lower or lower.startswith("scan.info.")) and any(lower.endswith(ext) for ext in [".yaml", ".yml", ".json"]):
        return True
    return False

@router.post("/templates/{name}/clean")
def clean_template_files(name: str):
    """
    Clean all generated files in the template directory,
    preserving only raw photo capture files: color image, depth map, and parameters.
    """
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")

    deleted_files = []
    kept_files = []
    try:
        for item in sorted(os.listdir(template_path)):
            item_path = os.path.join(template_path, item)
            if is_raw_capture_file(item):
                kept_files.append(item)
            else:
                if os.path.isfile(item_path) or os.path.islink(item_path):
                    os.remove(item_path)
                    deleted_files.append(item)
                    logger.info(f"Cleaned template generated file: {item_path}")
                elif os.path.isdir(item_path):
                    shutil.rmtree(item_path)
                    deleted_files.append(item + "/")
                    logger.info(f"Cleaned template generated directory: {item_path}")

        logger.info(f"Cleaned template '{name}': deleted {len(deleted_files)} items, kept {len(kept_files)} raw files.")
        return {
            "message": f"Template '{name}' cleaned successfully",
            "template": name,
            "deleted": deleted_files,
            "kept": kept_files,
        }
    except Exception as e:
        logger.error(f"Failed to clean template '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/templates/{name}/capture")
def capture_template_data(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    intr_dict = camera_service.get_intrinsics_dict()
    if not intr_dict or not intr_dict.get("intrinsic_matrix"):
        logger.error(f"Capture failed for template '{name}': Camera intrinsics not available.")
        raise HTTPException(status_code=503, detail="Camera intrinsics not available (camera offline)")
        
    try:
        logger.info(f"Starting hardware data capture for template '{name}'...")
        save_res = camera_service.save_frame(
            save_dir=template_path,
            color_filename="scan.color.jpg",
            depth_filename="scan.depth.png",
            color_format="jpg",
            save_color=True,
            save_depth=True,
            save_info_yaml=False
        )
        if not save_res:
            logger.warning(f"Initial frame capture failed for template '{name}', retrying after 0.8s...")
            time.sleep(0.8)
            save_res = camera_service.save_frame(
                save_dir=template_path,
                color_filename="scan.color.jpg",
                depth_filename="scan.depth.png",
                color_format="jpg",
                save_color=True,
                save_depth=True,
                save_info_yaml=False
            )
        if not save_res:
            raise HTTPException(status_code=500, detail="Hardware camera frame capture failed. The camera may be re-initializing, please try again.")

        # 2. 动态保存相机参数元数据 (无 hardcode，直接来自驱动/硬件配置)
        meta = {
            "version": "1.0",
            "template_name": name,
            "timestamp": time.time(),
            "camera_params": {
                "camera_model": intr_dict.get("camera_model", "Orbbec"),
                "intrinsic_matrix": intr_dict.get("intrinsic_matrix", []),
                "distortion_coeffs": intr_dict.get("distortion_coeffs", []),
                "width": intr_dict.get("width", 1280),
                "height": intr_dict.get("height", 800),
                "depth_scale": intr_dict.get("depth_scale", 1.0)
            }
        }
        params_path = os.path.join(template_path, "scan.params.yaml")
        with open(params_path, 'w', encoding='utf-8') as f:
            fast_yaml_dump(meta, f)
        logger.info(f"Saved camera metadata: {params_path}")
            
        logger.info(f"Successfully completed all captures for template '{name}'.")
        return {"message": "Data captured successfully", "template": name}
    except Exception as e:
        logger.error(f"Capture error for template '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

class PredictRequest(BaseModel):
    points: list[list[int]]
    labels: list[int]
    # 可选：wissight 检出的框 (x1, y1, x2, y2)，与点提示合并成一个 mask
    box: Optional[list[float]] = None

class SaveMasksRequest(BaseModel):
    committed_masks: list[dict]

@router.post("/templates/{name}/sam/init")
def init_sam(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    success = sam_service.init_template(template_path, name)
    if not success:
        logger.error(f"Failed to initialize SAM for template '{name}'.")
        raise HTTPException(status_code=500, detail="Failed to initialize SAM for this template")
    return {"message": "SAM initialized successfully"}

@router.post("/templates/{name}/detect")
def detect_objects(name: str):
    """用 wissight 检出衣物，把结果当作交互分割的自动初稿。

    `interactive.detector.sam_refine` 开（默认）时只回框，界面再拿它调 /sam/predict 精修；
    关时回传体里额外带 polygons（wissight 自己的实例 mask）。
    检测器不可用时返回空 detections（而不是 5xx），界面自动回退到纯手动点选。
    """
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")

    try:
        return sam_service.detect(template_path, name)
    except Exception as e:
        logger.error(f"Auto-detection failed for template '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/templates/{name}/sam/predict")
def predict_sam(name: str, req: PredictRequest):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")

    try:
        result = sam_service.predict_action(name, req.points, req.labels, template_path, box=req.box)
        return result
    except Exception as e:
        logger.error(f"Prediction failed for template '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/templates/{name}/sam/save")
def save_sam(name: str, req: SaveMasksRequest):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    try:
        success = sam_service.save_masks(template_path, name, req.committed_masks)
        if not success:
            raise HTTPException(status_code=500, detail="Failed to save masks")
        return {"message": "Masks saved successfully"}
    except PermissionError as e:
        logger.warning(f"Save masks permission error for template '{name}': {e}")
        raise HTTPException(status_code=403, detail=f"Permission denied: {str(e)}")
    except Exception as e:
        logger.warning(f"Save masks failed for template '{name}': {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/templates/{name}/reconstruct")
def reconstruct_surface(name: str):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    t_start = time.perf_counter()
    logger.info(f"⏱️ [3D Reconstruction] Received request for template '{name}' at {time.strftime('%Y-%m-%d %H:%M:%S')}")
    try:
        result = reconstruction_service.reconstruct_surface(template_path, name)
        elapsed_sec = time.perf_counter() - t_start
        logger.info(
            f"✅ [3D Reconstruction] Finished for template '{name}' in {elapsed_sec:.2f}s "
            f"(vertices: {result.get('vertices')}, faces: {result.get('faces')})"
        )
        return result
    except FileNotFoundError as e:
        logger.warning(f"Reconstruction file missing for '{name}': {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except ValueError as e:
        logger.warning(f"Reconstruction validation error for '{name}': {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.warning(f"Reconstruction failed for template '{name}' after {time.perf_counter() - t_start:.2f}s: {e}")
        raise HTTPException(status_code=500, detail=f"Reconstruction error: {str(e)}")


class SamplePointRequest(BaseModel):
    u: int
    v: int

@router.post("/templates/{name}/sample_point")
def sample_point(name: str, req: SamplePointRequest):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")

    try:
        # Standoff (spray target distance) is config-driven only: spraying.spray_dist_mm
        result = manual_path_service.sample_point_pose(name, req.u, req.v, sprayer_config.spray_distance_mm)
        return result
    except Exception as e:
        logger.warning(f"Sample point calculation failed for '{name}' at ({req.u},{req.v}): {e}")
        raise HTTPException(status_code=500, detail=f"Point sampling failed: {str(e)}")

@router.get("/templates/{name}/manual_paths")
def get_manual_paths(name: str, state_type: str = "raw"):
    try:
        return manual_path_service.load_manual_paths(name, state_type=state_type)
    except Exception as e:
        logger.warning(f"Get manual paths failed for template '{name}': {e}")
        raise HTTPException(status_code=500, detail=f"Failed to load manual paths: {str(e)}")


class AutoPathRequest(BaseModel):
    # standoff (spray target distance) is no longer client-overridable; it is read
    # exclusively from config spraying.spray_dist_mm on the backend.
    row_spacing_mm: Optional[float] = None
    point_spacing_mm: Optional[float] = None


@router.post("/templates/{name}/auto_paths")
def generate_auto_paths(name: str, req: AutoPathRequest = AutoPathRequest()):
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
    body = req
    try:
        return auto_path_service.generate_auto_paths(
            name,
            standoff_dist_mm=None,
            row_spacing_mm=body.row_spacing_mm,
            point_spacing_mm=body.point_spacing_mm,
        )
    except AutoPathServiceError as e:
        logger.warning("Auto path generation rejected for '%s': %s", name, e)
        raise HTTPException(status_code=400, detail=str(e))
    except PermissionError as e:
        logger.warning("Auto path permission error for '%s': %s", name, e)
        raise HTTPException(status_code=403, detail=f"Permission denied: {str(e)}")
    except Exception as e:
        logger.warning("Auto path generation failed for '%s': %s", name, e)
        raise HTTPException(status_code=500, detail=f"Auto path generation failed: {str(e)}")


class SaveManualPathsRequest(BaseModel):
    paths: list

@router.post("/templates/{name}/manual_paths")
def save_manual_paths(name: str, req: SaveManualPathsRequest, state_type: str = "raw"):
    try:
        data = {
            "paths": req.paths,
            # Standoff is config-driven only: spraying.spray_dist_mm
            "standoff_distance_mm": sprayer_config.spray_distance_mm
        }
        manual_path_service.save_manual_paths(name, data, state_type=state_type)
        return {"message": "Manual paths saved successfully"}
    except PermissionError as e:
        logger.warning(f"Save manual paths permission error for '{name}': {e}")
        raise HTTPException(status_code=403, detail=f"Permission denied: {str(e)}")
    except Exception as e:
        logger.warning(f"Save manual paths error for '{name}': {e}")
        raise HTTPException(status_code=500, detail=f"Failed to save manual paths: {str(e)}")


@router.get("/templates/{name}/session_data")
def get_session_data(name: str):
    """
    One-shot endpoint: returns all data needed for fully client-side normal computation.
    - depth_flat: base64-encoded float32 array (row-major, h*w values in mm)
    - width, height: depth image dimensions
    - intrinsics: {fx, fy, cx, cy}
    - T_base_camera: 4x4 row-major float list (translation in mm)
    - calib_source: description string
    """
    import base64
    from apps.interactive.reconstruction_service import reconstruction_service

    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")

    # Load depth
    depth_npy = os.path.join(template_path, "scan.depth.npy")
    depth_png = os.path.join(template_path, "scan.depth.png")
    if os.path.exists(depth_npy):
        depth_map = np.load(depth_npy).astype(np.float32)
    elif os.path.exists(depth_png):
        depth_map = cv2.imread(depth_png, cv2.IMREAD_UNCHANGED).astype(np.float32)
    else:
        raise HTTPException(status_code=400, detail="Depth map not found for this template")

    if depth_map is None:
        raise HTTPException(status_code=400, detail="Failed to load depth map")

    h, w = depth_map.shape[:2]

    # Load calibration
    try:
        T_cam_to_base_m, k_matrix, calib_desc = reconstruction_service.get_latest_calibration()
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to load calibration: {str(e)}")

    # Intrinsics
    if k_matrix is not None:
        fx, fy = float(k_matrix[0, 0]), float(k_matrix[1, 1])
        cx, cy = float(k_matrix[0, 2]), float(k_matrix[1, 2])
    else:
        fx, fy = 611.68, 611.69
        cx, cy = float(w) / 2.0, float(h) / 2.0

    # Convert transform: translate meters -> mm
    T_base_camera = T_cam_to_base_m.copy()
    T_base_camera[0:3, 3] *= 1000.0

    # Encode depth map as base64 float32
    depth_bytes = depth_map.flatten().astype(np.float32).tobytes()
    depth_b64 = base64.b64encode(depth_bytes).decode('ascii')

    return {
        "width": w,
        "height": h,
        "depth_flat_b64": depth_b64,
        "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
        "T_base_camera": T_base_camera.flatten().tolist(),
        "calib_source": calib_desc
    }


class PoiConstraintConfig(BaseModel):
    ref_rpy_deg: list[float] | None = None  # e.g. [0.0, 0.0, 0.0]; 显式给出时它就是包络中心
    tolerance_rpy_deg: list[float] = Field(default_factory=get_default_poi_tolerance_rpy_deg)  # [tol_rx, tol_ry, tol_rz]
    # 'config'(配置文件 poi_ref_rpy_deg) | 'home'(Home 正解) | 'live'(机械臂当前 TCP, 本层解析成 ref_rpy) | 'raw'(逐点名义法向)
    # 缺省时沿用配置文件 spraying.poi_anchor_source
    anchor_source: str | None = None


class KinematicsOptions(BaseModel):
    # 不设默认速度/步长：前端通常不传，让 motion_cli 读配置（spraying.velocity / slerp_step_mm）。
    step_size_mm: Optional[float] = None
    linear_velocity_mm_s: Optional[float] = None


class VerifyPathRequest(BaseModel):
    state_type: str = "raw"  # 'raw' | 'auto' | 'poi' | 'auto_poi'
    options: KinematicsOptions = KinematicsOptions()


class OptimizePathRequest(BaseModel):
    mode: str = "poi"
    source: str = "raw"  # 'raw' | 'auto'
    poi_config: PoiConstraintConfig | None = None
    options: KinematicsOptions = KinematicsOptions()


@router.post("/templates/{name}/verify_paths")
def verify_paths(name: str, req: VerifyPathRequest = VerifyPathRequest()):
    try:
        res = path_verification_service.verify_template_paths(
            name, 
            state_type=req.state_type,
            options=req.options.model_dump(exclude_none=True)
        )
        return res
    except FileNotFoundError as e:
        raise HTTPException(status_code=404, detail=str(e))
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Verification error for '{name}': {e}", exc_info=True)
        raise HTTPException(status_code=500, detail=f"Verification failed: {str(e)}")


@router.post("/templates/{name}/optimize_paths")
def optimize_paths(name: str, req: OptimizePathRequest = OptimizePathRequest()):
    try:
        poi_dict = req.poi_config.model_dump() if req.poi_config else None
        if poi_dict and str(poi_dict.get("anchor_source") or "").strip().lower() == "live":
            # live: 以机械臂当前 TCP 姿态作包络中心, 在这里解析成具体 ref_rpy 再下发给优化器
            live_pose, _ = robot_service.get_current_pose()
            if not live_pose or len(live_pose) < 6:
                raise HTTPException(
                    status_code=409,
                    detail="Live robot TCP pose is unavailable. Connect the robot before using anchor_source='live'."
                )
            poi_dict["ref_rpy_deg"] = [round(float(live_pose[3]), 2), round(float(live_pose[4]), 2), round(float(live_pose[5]), 2)]
            poi_dict["anchor_source"] = "config"
        res = path_verification_service.optimize_template_paths(
            name,
            source=req.source,
            poi_config=poi_dict,
            options=req.options.model_dump(exclude_none=True)
        )
        return res
    except HTTPException:
        raise
    except FileNotFoundError as e:
        logger.error(f"❌ [Path Optimization] File not found for '{name}': {e}")
        raise HTTPException(status_code=404, detail=str(e))
    except ValueError as e:
        logger.error(f"❌ [Path Optimization] Invalid request for '{name}': {e}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"❌ [Path Optimization] Error for '{name}': {e}")
        res = path_verification_service.record_optimization_failure(name, source=req.source, error=str(e))
        return res


@router.get("/templates/{name}/verification_report")
def get_verification_report(name: str, state_type: str = "raw"):
    """
    Retrieves saved verification report from unified .path.yaml.
    """
    report = path_verification_service.get_saved_report(name, state_type=state_type)
    if report is not None:
        return report
    raise HTTPException(status_code=404, detail=f"No saved verification report found for '{state_type}'.")


@router.get("/robot/urdf_tool_tcp")
def get_robot_urdf_tcp():
    """Returns TCP offset extracted directly from URDF."""
    return path_verification_service.get_urdf_tcp()


@router.get("/robot/anchor_pose")
def get_robot_anchor_pose(source: str = "home"):
    """
    Returns reference TCP poses for POI constraint configuration:
    1. Current live robot TCP pose (if connected)
    2. Robot Home pose TCP orientation
    """
    from apps.robot.services.robot_service import robot_service
    from core.motion.kinematics import CR5Kinematics

    solver = CR5Kinematics()
    
    # 1. Dobot home joint angles must match DobotDriver.go_home(): JointMovJ([0, 0, -90, -90, -90, 0]).
    # Use controller-frame FK here so POI Home anchor Rx/Ry/Rz matches Dobot TCP pose convention.
    home_deg = [0.0, 0.0, -90.0, -90.0, -90.0, 0.0]
    home_rad = [math.radians(v) for v in home_deg]
    home_xyz, home_rpy_raw = solver.forward_controller(home_rad)
    home_xyz = [round(float(v), 2) for v in home_xyz]
    home_rpy = [round(float(v), 2) for v in home_rpy_raw]

    # 2. Live robot TCP pose if connected
    live_pose, _ = robot_service.get_current_pose()
    live_rpy = None
    live_xyz = None
    if live_pose and len(live_pose) >= 6:
        live_xyz = [round(float(live_pose[0]), 2), round(float(live_pose[1]), 2), round(float(live_pose[2]), 2)]
        live_rpy = [round(float(live_pose[3]), 2), round(float(live_pose[4]), 2), round(float(live_pose[5]), 2)]

    if source not in {"home", "live", "config", "raw"}:
        raise HTTPException(status_code=400, detail="source must be 'home', 'live', 'config' or 'raw'")

    if source == "live":
        if not live_rpy:
            raise HTTPException(status_code=409, detail="Live robot TCP pose is unavailable. Connect the robot before capturing a live POI anchor pose.")
        selected_rpy = live_rpy
        selected_xyz = live_xyz
    elif source == "config":
        cfg_ref = sprayer_config.poi_ref_rpy_deg
        if not cfg_ref:
            raise HTTPException(status_code=409, detail="spraying.poi_ref_rpy_deg is not configured in aisprayer_config.yaml")
        selected_rpy = [round(float(v), 2) for v in cfg_ref]
        selected_xyz = home_xyz
    elif source == "raw":
        # 逐点名义法向锚点: 没有全局参考姿态可言
        selected_rpy = None
        selected_xyz = None
    else:
        selected_rpy = home_rpy
        selected_xyz = home_xyz

    return {
        "source": source,
        "is_connected": robot_service._is_connected,
        "rpy_deg": selected_rpy,
        "xyz_mm": selected_xyz,
        "default_tolerance_rpy_deg": get_default_poi_tolerance_rpy_deg(),
        "default_anchor_source": sprayer_config.poi_anchor_source,
        "default_ref_rpy_deg": sprayer_config.poi_ref_rpy_deg,
        "home_pose": {
            "xyz_mm": home_xyz,
            "rpy_deg": home_rpy,
            "joints_deg": home_deg
        },
        "live_pose": {
            "xyz_mm": live_xyz or home_xyz,
            "rpy_deg": live_rpy or home_rpy
        } if live_pose else None
    }


@router.get("/templates/{name}/summary")
def get_template_summary(name: str):
    """
    Returns complete atomic summary of template data in a single round-trip:
    files, masks, raw/auto/poi paths, unified verification reports, and URDF tool TCP info.
    """
    import time
    t0 = time.time()
    
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    if not os.path.exists(template_path):
        raise HTTPException(status_code=404, detail="Template not found")
        
    # 1. File items (excluding obsolete standalone .report.json)
    files = []
    for item in os.listdir(template_path):
        item_path = os.path.join(template_path, item)
        if os.path.isfile(item_path) and not item.endswith(".report.json") and not item.endswith(".report.yaml"):
            size = os.path.getsize(item_path)
            ctime = os.path.getctime(item_path)
            files.append({"name": item, "size": size, "ctime": ctime})
    files.sort(key=lambda x: x["ctime"], reverse=True)

    file_set = {f["name"] for f in files}
    image_filename = "scan.color.jpg" if "scan.color.jpg" in file_set else ("scan.jpg" if "scan.jpg" in file_set else None)
    has_image = image_filename is not None
    has_depth = "scan.depth.png" in file_set or "scan.depth.npy" in file_set
    has_mesh = "scan.mesh.ply" in file_set or "scan.mesh.stl" in file_set

    t1 = time.time()

    # 2. Masks
    masks_dict = sam_service.get_template_masks(template_path)
    masks_data = masks_dict.get("masks", []) if masks_dict else []

    t2 = time.time()

    # 3. Paths & Verification (Single-pass load for maximum speed)
    raw_paths_data = manual_path_service.load_manual_paths(name, state_type="raw")
    t3 = time.time()
    auto_paths_data = manual_path_service.load_manual_paths(name, state_type="auto")
    t4 = time.time()
    poi_paths_data = manual_path_service.load_manual_paths(name, state_type="poi")
    t5 = time.time()
    auto_poi_paths_data = manual_path_service.load_manual_paths(name, state_type="auto_poi")
    t6 = time.time()

    raw_paths = raw_paths_data.get("paths", [])
    auto_paths = auto_paths_data.get("paths", []) if auto_paths_data.get("paths") else []
    auto_poi_paths = auto_poi_paths_data.get("paths", []) if auto_poi_paths_data.get("paths") else []
    poi_paths = poi_paths_data.get("paths", []) if poi_paths_data.get("paths") else []
    # Standoff is config-driven only (spraying.spray_dist_mm); ignore any value baked
    # into previously-saved path files so config stays the single source of truth.
    standoff = sprayer_config.spray_distance_mm

    raw_report = raw_paths_data.get("verification")
    auto_report = auto_paths_data.get("verification")
    auto_poi_report = auto_poi_paths_data.get("verification")
    poi_report = poi_paths_data.get("verification")

    # 4. URDF TCP
    urdf_tcp = path_verification_service.get_urdf_tcp()
    t7 = time.time()

    logger.info(f"get_template_summary breakdown: init+files={t1-t0:.3f}s, masks={t2-t1:.3f}s, raw={t3-t2:.3f}s, auto={t4-t3:.3f}s, poi={t5-t4:.3f}s, auto_poi={t6-t5:.3f}s, tcp={t7-t6:.3f}s, TOTAL={t7-t0:.3f}s")

    return {
        "template": name,
        "files": files,
        "has_image": has_image,
        "image_filename": image_filename,
        "has_depth": has_depth,
        "has_mesh": has_mesh,
        "masks": masks_data,
        "raw_paths": raw_paths,
        "auto_paths": auto_paths,
        "auto_poi_paths": auto_poi_paths,
        "poi_paths": poi_paths,
        "standoff_distance_mm": standoff,
        "raw_report": raw_report,
        "auto_report": auto_report,
        "auto_poi_report": auto_poi_report,
        "poi_report": poi_report,
        "urdf_tcp": urdf_tcp
    }


# ─── Real Robot Path Waypoint Execution (move_l_segments) ─────────────────────

def _is_point_spraying(pt: dict) -> bool:
    """
    判断航点是否处于喷涂开启状态：
    1. 优先读取显式 spraying 字段 (生成器写入 "on"/"off"，也兼容 bool)
    2. 无该字段时按 is_jump 推断 (is_jump=True 表示空走转移段，不喷涂)
    3. 两者都没有则默认开启
    """
    val = pt.get("spraying")
    if val is None:
        return not bool(pt.get("is_jump", False))
    return is_spraying_on(val)


def _group_points_into_segments(points_data: List[dict]) -> List[dict]:
    """
    按相邻航点的 spraying 状态进行游程合并 (Run-Length Encoding)。
    连续相同 spraying 状态的航点合并为同一段，使 DO 开关只在状态转换边界触发一次。
    输入项格式: {"pose": dict, "spraying": bool}，返回格式: [{"spraying": bool, "poses": [pose_dict, ...]}, ...]
    """
    segments: List[dict] = []
    for item in points_data:
        if segments and segments[-1]["spraying"] == item["spraying"]:
            segments[-1]["poses"].append(item["pose"])
        else:
            segments.append({"spraying": item["spraying"], "poses": [item["pose"]]})
    return segments


class ExecuteYamlPathRequest(BaseModel):
    file_name: str
    path_id: Optional[int] = None # None or 0 means all paths, 1-based index/ID
    # Linear motion parameters (MoveL queue)
    speed_l: Optional[float] = None # mm/s (笛卡尔线速度)
    acc_l: Optional[float] = None   # % (笛卡尔加速度百分比)
    # Joint motion parameters (Go Home & MovJ to start point)
    speed_j: Optional[float] = None # % (关节速度百分比)
    acc_j: Optional[float] = None   # % (关节加速度百分比)
    cp_ratio: Optional[int] = 100
    spray_do_index: Optional[int] = None # 喷涂开关 DO 端口编号 (若未指定则从配置文件读取)
    # Backward compatibility fields
    speed: Optional[float] = None
    acc: Optional[float] = None

@router.get("/templates/{name}/yaml_paths_info")
def get_yaml_paths_info(name: str, file_name: str):
    """
    读取并解析指定 YAML 轨迹文件，返回包含的全部子路径及其点数摘要，供前端交互选择。
    """
    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    file_path = os.path.join(template_path, file_name)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"File '{file_name}' not found in template '{name}'")
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = fast_yaml_load(f) or {}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to parse YAML file: {e}")

    paths = data.get("paths", [])
    path_infos = []
    total_pts = 0
    for idx, p in enumerate(paths):
        pts = p.get("points", [])
        pts_count = len(pts)
        total_pts += pts_count
        path_infos.append({
            "path_id": p.get("path_id", idx + 1),
            "name": p.get("name", f"Path {idx + 1}"),
            "point_count": pts_count
        })

    return {
        "template": name,
        "file_name": file_name,
        "total_paths": len(path_infos),
        "total_points": total_pts,
        "paths": path_infos
    }

@router.post("/templates/{name}/execute_yaml_path")
def execute_yaml_path(name: str, req: ExecuteYamlPathRequest):
    """
    根据选择的路径 (单条或全部)，提取 Waypoints 位姿并通过 robot_service 发送到真实机械臂执行：
    1. 执行前先回到 Home 姿态 (使用关节参数 speed_j, acc_j)
    2. 第 1 条 path 的第 1 个 waypoint 使用 move_j 过去 (使用关节参数 speed_j, acc_j)
    3. 然后通过 robot_service.move_l_segments 连续笛卡尔直线分段执行 (使用线速度参数 speed_l, acc_l, cp_ratio)，
       驱动层按各段 spraying 状态在段边界同步切换喷涂 DO
    """
    if not robot_service.is_connected():
        raise HTTPException(status_code=400, detail="Robot is not connected. Please connect robot first in control panel.")
    if robot_service.is_moving():
        raise HTTPException(status_code=409, detail="Robot is currently moving. Please wait for the current motion to complete.")

    template_path = os.path.join(TEMPLATE_GROUP_DIR, name)
    file_path = os.path.join(template_path, req.file_name)
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail=f"Path file '{req.file_name}' not found in template '{name}'")

    robot_service.broadcast_exec_status(action="Parsing trajectory data & safety check...", stage="preparing")
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = fast_yaml_load(f) or {}
    except Exception as e:
        robot_service.broadcast_exec_status(action=f"Failed to read path file: {e}", stage="error")
        raise HTTPException(status_code=500, detail=f"Failed to read YAML file: {e}")

    raw_paths = data.get("paths", [])
    if not raw_paths:
        raise HTTPException(status_code=400, detail=f"No paths found in {req.file_name}")

    # 严密安全防护：物理机械臂只允许执行校验状态为 PASS 的安全路径，严防执行 FAILED 或未校验路径
    verification = data.get("verification") or {}
    ver_status = str(verification.get("status") or verification.get("summary", {}).get("status") or "").upper()
    if ver_status != "PASS":
        err_msg = (
            f"Cannot execute '{req.file_name}': Path verification status is '{ver_status or 'UNVERIFIED'}'. "
            "Only paths verified with status 'PASS' can be executed on the physical robot."
        )
        logger.error(f"❌ [Robot Execution] {err_msg}")
        robot_service.broadcast_exec_status(action=f"Path kinematics verification failed ({ver_status or 'UNVERIFIED'})", stage="error")
        raise HTTPException(status_code=400, detail=err_msg)

    # 筛选待执行的路径
    target_paths = []
    if req.path_id is not None and req.path_id > 0:
        target_paths = [p for p in raw_paths if p.get("path_id") == req.path_id]
        if not target_paths and len(raw_paths) >= req.path_id:
            target_paths = [raw_paths[req.path_id - 1]]
    else:
        target_paths = raw_paths

    if not target_paths:
        raise HTTPException(status_code=400, detail=f"Selected path #{req.path_id} not found in {req.file_name}")

    # 解析 MoveL (笛卡尔线运动) 与 MoveJ (关节运动) 速度/加速度参数，优先沿用控制面板当前设定值
    curr_speed_l, curr_acc_l, curr_speed_j, curr_acc_j = robot_service.get_speed()
    speed_l = req.speed_l if req.speed_l is not None else (req.speed if req.speed is not None else curr_speed_l)
    acc_l = req.acc_l if req.acc_l is not None else (req.acc if req.acc is not None else curr_acc_l)
    speed_j = req.speed_j if req.speed_j is not None else curr_speed_j
    acc_j = req.acc_j if req.acc_j is not None else curr_acc_j

    # 0. 自动根据配置文件中的 robot_tcp_id 设置机械臂工具坐标系，并读取喷涂 DO 端子编号
    robot_service.broadcast_exec_status(action="Configuring robot tool TCP...", stage="configuring")
    sprayer_config.reload()
    target_tool_id = sprayer_config.robot_tcp_id
    target_tcp_name = sprayer_config.robot_tcp
    target_do_index = req.spray_do_index if req.spray_do_index is not None else sprayer_config.spray_do_index
    logger.info(f"execute_yaml_path: Automatically configuring robot tool to ID={target_tool_id} (TCP={target_tcp_name}), Spray DO index={target_do_index}...")
    tool_ok, tool_err = robot_service.set_tool(target_tool_id)
    if not tool_ok:
        logger.warning(f"execute_yaml_path: Warning when setting robot tool to ID {target_tool_id}: {tool_err}")

    total_executed_pts = 0
    executed_names = []
    has_moved_j_to_start = False

    # 统计全部待执行航点总数，用于进度上报
    total_waypoints_all = sum(
        1 for p in target_paths for pt in p.get("points", []) if pt.get("tcp_pose_base")
    )
    waypoints_done = 0

    try:
        # 1. 在执行 path 之前先回到 Home 姿态 (使用 MoveJ 关节运动参数)
        robot_service.broadcast_exec_status(action="Moving robot to Home position...", stage="moving_home")
        logger.info(f"execute_yaml_path: Moving robot to Home position (speed_j={speed_j}%, acc_j={acc_j}%) before trajectory execution...")
        home_ok, home_err = robot_service.go_home(speed=speed_j, acc=acc_j)
        if not home_ok:
            raise HTTPException(status_code=500, detail=f"Failed to go home before path execution: {home_err}")

        logger.info("execute_yaml_path: Robot has moved to Home position.")

        for p_idx, path in enumerate(target_paths):
            pts = path.get("points", [])
            valid_items = []
            for pt in pts:
                tcp = pt.get("tcp_pose_base")
                if tcp:
                    valid_items.append({
                        "pose": {
                            "x": float(tcp.get("x", 0.0)),
                            "y": float(tcp.get("y", 0.0)),
                            "z": float(tcp.get("z", 0.0)),
                            "rx": float(tcp.get("rx", 0.0)),
                            "ry": float(tcp.get("ry", 0.0)),
                            "rz": float(tcp.get("rz", 0.0)),
                            "is_radians": False, # YAML 中姿态角度为 deg
                        },
                        "spraying": _is_point_spraying(pt),
                    })
            if not valid_items:
                continue

            path_title = path.get("name", f"Path {p_idx + 1}")
            executed_names.append(path_title)

            # 2. 第 1 条有效 path 的第 1 个 waypoint，使用 move_j 快速安全过渡过去 (关节参数，此时 DO 保持关闭)
            just_moved_j = False
            if not has_moved_j_to_start:
                robot_service.broadcast_exec_status(action=f"Transitioning to start waypoint ({path_title})...", stage="moving_start")
                first_pt = valid_items[0]["pose"]
                first_pose = RobotPose.from_dict(first_pt).to_list()
                logger.info(f"execute_yaml_path: MovJ to 1st waypoint of {path_title} -> {first_pt} "
                            f"(speed_j={speed_j}%, acc_j={acc_j}%, tool={target_tool_id})")
                j_ok, j_err = robot_service.move_to_pose_j(first_pose, speed=speed_j, acc=acc_j, tool_num=target_tool_id)
                if not j_ok:
                    raise HTTPException(status_code=500, detail=f"Failed to MovJ to start waypoint of {path_title}: {j_err}")
                has_moved_j_to_start = True
                just_moved_j = True
                waypoints_done += 1
                logger.info(f"execute_yaml_path: Robot has moved to 1st waypoint of {path_title}.")

            # 3. 执行笛卡尔轨迹：已经 MovJ 到达的第 1 个点不再重复下发，后续 path 则从第 1 个点开始全部纳入分段
            exec_items = valid_items[1:] if just_moved_j else valid_items
            if exec_items:
                # 按相邻航点的 spraying 状态游程合并，相同连续状态合并为一段，仅在边界开关一次 DO
                segments = _group_points_into_segments(exec_items)
                logger.info(f"execute_yaml_path: Executing {len(exec_items)} waypoints in {len(segments)} segments on {path_title} "
                            f"(speed_l={speed_l} mm/s, acc_l={acc_l}%, cp={req.cp_ratio}, DO={target_do_index}, tool={target_tool_id})")
                robot_service.broadcast_exec_status(
                    action=f"Executing spray trajectory ({path_title}, pt {waypoints_done + 1}/{total_waypoints_all})...",
                    stage="spraying",
                    progress=waypoints_done / max(total_waypoints_all, 1),
                    current_waypoint=waypoints_done,
                    total_waypoints=total_waypoints_all,
                )
                # move_l_segments 只在最后一段等待轨迹真正执行完成，因此进度按整条 path 粒度上报
                batch_ok, batch_err = robot_service.move_l_segments(
                    segments,
                    speed=speed_l,
                    acc=acc_l,
                    cp_ratio=req.cp_ratio if req.cp_ratio is not None else 100,
                    spray_do_index=target_do_index,
                    tool_num=target_tool_id,
                )
                if not batch_ok:
                    raise HTTPException(status_code=500, detail=f"Execution error on {path_title}: {batch_err}")
                waypoints_done += len(exec_items)

            total_executed_pts += len(valid_items)
            robot_service.broadcast_exec_progress(
                current_waypoint=waypoints_done,
                total_waypoints=total_waypoints_all,
                path_idx=p_idx,
                total_paths=len(target_paths)
            )

        # 4. 轨迹全部执行完毕后回位至 Home
        # 工业安全与工艺防护：在轨迹执行结束、回 Home 动作触发前，强制确保喷涂 DO 关闭 (立即指令毫秒级关断，杜绝回程误喷拉丝)
        logger.info(f"execute_yaml_path: Ensuring spray DO (index {target_do_index}) is OFF before returning to Home...")
        try:
            off_ok, off_err = robot_service.set_do(target_do_index, 0, immediate=True)
            if not off_ok:
                logger.warning(f"execute_yaml_path: Warning when ensuring DO({target_do_index}, 0) before returning Home: {off_err}")
            time.sleep(0.02)
        except Exception as do_err:
            logger.warning(f"execute_yaml_path: Exception when turning off DO before returning Home: {do_err}")

        robot_service.broadcast_exec_status(action="Trajectory complete, returning to Home...", stage="returning_home")
        logger.info(f"execute_yaml_path: Returning robot to Home position after trajectory (speed_j={speed_j}%, acc_j={acc_j}%)...")
        home_ok, home_err = robot_service.go_home(speed=speed_j, acc=acc_j)
        if not home_ok:
            raise HTTPException(status_code=500, detail=f"Failed to go home after path execution: {home_err}")

        robot_service.broadcast_exec_status(action="Spray task executed successfully", stage="done")
    except Exception as e:
        robot_service.broadcast_exec_status(action=f"Execution aborted: {str(e)}", stage="error")
        logger.exception(f"execute_yaml_path: Trajectory execution failed: {e}")
        raise
    finally:
        # 执行完成或异常中断时，强制安全关闭机械臂喷涂 DO (使用立即指令确保关喷)
        logger.info(f"execute_yaml_path: Setting robot spray DO (index {target_do_index}) to OFF (0)...")
        try:
            off_ok, off_err = robot_service.set_do(target_do_index, 0, immediate=True)
            if not off_ok:
                logger.warning(f"execute_yaml_path: Warning when setting DO({target_do_index}, 0): {off_err}")
            time.sleep(0.05)
        except Exception as e:
            logger.error(f"execute_yaml_path: Exception while turning off DO({target_do_index}, 0): {e}")

    return {
        "status": "success",
        "message": f"Successfully executed {len(target_paths)} path(s) ({total_executed_pts} points): {', '.join(executed_names)}",
        "executed_paths": len(target_paths),
        "total_points": total_executed_pts
    }



