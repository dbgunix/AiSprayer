"""3D 喷涂航点与轨迹规划模块 (WaypointPlanner)。

将牛仔裤 2D 掩码分腿打标、网格顶点投影分类、外侧缝拟合、直面之字形 (Zigzag) 切片、
裆部重叠航点去重、1D 法向滤波与连续无奇异 TCP 姿态计算高度内聚封装于一体。
"""

from __future__ import annotations

import logging
from typing import Any, List, Optional, Sequence, Tuple, Union

import cv2
import numpy as np
import trimesh
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R_tool

logger = logging.getLogger(__name__)

LABEL_NONE = 0
LABEL_LEFT = 1
LABEL_RIGHT = 2
LABEL_OVERLAP = 3


class WaypointPlannerError(ValueError):
    """规划失败异常（缺标定、空输入、几何异常、采不到有效航点等）。"""


def split_jeans_mask(
    mask_2d: np.ndarray,
    depth_threshold_ratio: float = 0.1,
    overlap_px: float = 0.0,
) -> List[np.ndarray]:
    """基于轮廓凸缺陷 (Convexity Defects) 分析自动识别裤裆并将 2D 掩码切分为双腿。

    若未识别到显著裤裆（如侧面或单腿工况），直接返回包含单掩码的列表。

    :param mask_2d: HxW 布尔掩码
    :param depth_threshold_ratio: 裤裆凸缺陷最小深度与包围盒高度的比值
    :param overlap_px: 分割中线两侧保留的重叠带像素宽度 (0.0 表示硬切分)
    :return: 包含 1 或 2 个布尔掩码的列表 [mask_left, mask_right] 或 [mask]
    """
    mask_bool = np.asarray(mask_2d, dtype=bool)
    mask_uint8 = mask_bool.astype(np.uint8) * 255
    contours, _ = cv2.findContours(
        mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return [mask_bool]

    c = max(contours, key=cv2.contourArea)
    hull_indices = cv2.convexHull(c, returnPoints=False)
    if hull_indices is None or len(hull_indices) < 3:
        return [mask_bool]

    defects = cv2.convexityDefects(c, hull_indices)
    if defects is None:
        return [mask_bool]

    _x, _y, w, h = cv2.boundingRect(c)
    max_depth = 0.0
    best_defect = None

    for i in range(defects.shape[0]):
        row = defects[i].flatten()
        s, e, f, d = row
        depth = d / 256.0

        start = tuple(c[s][0])
        end = tuple(c[e][0])
        far = tuple(c[f][0])  # 裤裆缺陷最深点

        if depth > max_depth:
            dist_se = float(np.hypot(start[0] - end[0], start[1] - end[1]))
            # 两个裤脚点跨度应相对较宽，且缺陷深度足够
            if dist_se > 0.2 * w and depth > depth_threshold_ratio * h:
                min_x = min(start[0], end[0])
                max_x = max(start[0], end[0])
                if min_x < far[0] < max_x:
                    max_depth = depth
                    best_defect = (start, end, far)

    if best_defect is None:
        logger.info(
            "[Segmentation] No significant crotch found; returning single mask."
        )
        return [mask_bool]

    start, end, far = best_defect
    if start[0] > end[0]:
        start, end = end, start

    p_left = np.array(start, dtype=np.float64)
    p_right = np.array(end, dtype=np.float64)
    p_crotch = np.array(far, dtype=np.float64)

    v_vec = p_right - p_left
    v_norm = float(np.linalg.norm(v_vec))
    if v_norm < 1e-6:
        return [mask_bool]

    # 限制左右裤腿分界向量倾角（最大偏角约 14 度），防止因单侧卷裤脚或断口导致分界线倾斜切入裤管
    max_vy = 0.25 * abs(v_vec[0])
    v_vec[1] = np.clip(v_vec[1], -max_vy, max_vy)
    v_norm = float(np.linalg.norm(v_vec))
    # 防御性二次早退：当 v_vec[0]==0 时 max_vy=0 会将整个 v_vec 归零；当前上游
    # 缺陷跨度过滤 min_x < far.x < max_x 已经排除了该分支，但保留此保护以免未来
    # 放宽检测范围时 signed_distance 变成 NaN/Inf。
    if v_norm < 1e-6:
        return [mask_bool]

    h_img, w_img = mask_bool.shape[:2]
    y_idx, x_idx = np.mgrid[0:h_img, 0:w_img]

    dx = x_idx - p_crotch[0]
    dy = y_idx - p_crotch[1]
    signed_distance = (dx * v_vec[0] + dy * v_vec[1]) / v_norm

    overlap_val = max(float(overlap_px), 0.0)
    mask_left = mask_bool & (signed_distance <= overlap_val)
    mask_right = mask_bool & (signed_distance > -overlap_val)

    # 过滤微小碎屑噪点（Bug 2 修复：保留所有显著连通分量，防止猫须或褶皱处断腿）
    def filter_noise_cc(m: np.ndarray, min_area_ratio: float = 0.05) -> np.ndarray:
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            m.astype(np.uint8), connectivity=8
        )
        if num_labels <= 1:
            return m
        areas = stats[1:, cv2.CC_STAT_AREA]
        max_area = float(np.max(areas))
        threshold = max(50.0, max_area * min_area_ratio)
        valid_labels = [i + 1 for i, a in enumerate(areas) if a >= threshold]
        if not valid_labels:
            return m
        return np.isin(labels, valid_labels)

    mask_left = filter_noise_cc(mask_left)
    mask_right = filter_noise_cc(mask_right)

    logger.info(
        "[Segmentation] Crotch found (depth=%.1fpx); split into 2 legs (overlap=%.1fpx)",
        max_depth,
        overlap_val,
    )
    return [mask_left, mask_right]


class PathNormalSmoother:
    """轨迹法向量 1D 滑动平均平滑器。"""

    def __init__(self, window_size: int = 5):
        self.window_size = max(1, int(window_size))

    def smooth(self, paths: Sequence[dict[str, Any]]) -> List[dict[str, Any]]:
        """对路径样本序列执行 1D 法向量平滑。"""
        if not paths or len(paths) < self.window_size or self.window_size <= 1:
            return [dict(p) for p in paths]

        normals = np.array([p["normal"] for p in paths], dtype=np.float64)
        pad_width = self.window_size // 2
        padded_normals = np.pad(
            normals, ((pad_width, pad_width), (0, 0)), mode="edge"
        )

        smoothed_normals = np.zeros_like(normals)
        kernel = np.ones(self.window_size, dtype=np.float64) / self.window_size

        for i in range(3):
            smoothed_normals[:, i] = np.convolve(
                padded_normals[:, i], kernel, mode="valid"
            )

        norms = np.linalg.norm(smoothed_normals, axis=1, keepdims=True)
        norms[norms < 1e-6] = 1.0
        smoothed_normals = smoothed_normals / norms

        smoothed_paths = []
        for i, p in enumerate(paths):
            new_p = dict(p)
            new_p["normal"] = smoothed_normals[i].tolist()
            smoothed_paths.append(new_p)

        return smoothed_paths


class WaypointPlanner:
    """3D 喷涂航点规划器 (WaypointPlanner)。

    提供基于网格曲面与 2D 掩码的自动路径规划，输出与 scan.auto.path.yaml 同构的字典。
    """

    def __init__(
        self,
        spray_dist_mm: float = 150.0,
        row_spacing_mm: float = 60.0,
        point_spacing_mm: float = 100.0,
        image_size: Tuple[int, int] = (1280, 800),
        camera_intrinsics: Optional[np.ndarray] = None,
        T_camera_to_base: Optional[np.ndarray] = None,
        depth_threshold_ratio: float = 0.1,
        dedup_radius_mm: float = 30.0,
        normal_smooth_window: int = 5,
        mesh_unit: str = "m",
        align_outer_edge: bool = True,
    ):
        self.spray_dist_mm = float(spray_dist_mm)
        self.row_spacing_mm = float(row_spacing_mm)
        self.point_spacing_mm = float(point_spacing_mm)
        self.image_size = (int(image_size[0]), int(image_size[1]))
        self.camera_intrinsics = (
            np.asarray(camera_intrinsics, dtype=np.float64)
            if camera_intrinsics is not None
            else None
        )
        self.T_camera_to_base = (
            np.asarray(T_camera_to_base, dtype=np.float64)
            if T_camera_to_base is not None
            else None
        )
        self.depth_threshold_ratio = float(depth_threshold_ratio)
        self.dedup_radius_mm = float(dedup_radius_mm)
        self.normal_smooth_window = int(normal_smooth_window)
        self.mesh_unit = str(mesh_unit).strip().lower()
        self.align_outer_edge = bool(align_outer_edge)
        if self.mesh_unit not in ("m", "mm"):
            raise WaypointPlannerError(
                f"mesh_unit must be 'm' or 'mm', got {mesh_unit!r}"
            )

    def plan(
        self,
        mesh: trimesh.Trimesh,
        masks: Union[dict, np.ndarray],
        camera_intrinsics: Optional[np.ndarray] = None,
        T_camera_to_base: Optional[np.ndarray] = None,
        image_size: Optional[Tuple[int, int]] = None,
    ) -> dict[str, Any]:
        """规划全套自动喷涂航点。

        :param mesh: 完整表面网格（单位需与 mesh_unit 一致）
        :param masks: 已解析的 scan.masks.yaml 字典，或者直接传入 2D 布尔掩码
        :param camera_intrinsics: 可选覆盖 3x3 相机内参
        :param T_camera_to_base: 可选覆盖 4x4 手眼标定矩阵
        :param image_size: 可选覆盖 (width, height)
        :return: scan.auto.path.yaml 结构的字典
        """
        k = (
            np.asarray(camera_intrinsics, dtype=np.float64)
            if camera_intrinsics is not None
            else self.camera_intrinsics
        )
        t = (
            np.asarray(T_camera_to_base, dtype=np.float64)
            if T_camera_to_base is not None
            else self.T_camera_to_base
        )
        img_sz = image_size if image_size is not None else self.image_size

        if k is None or t is None:
            raise WaypointPlannerError(
                "camera_intrinsics and T_camera_to_base are required; refusing to plan without K/T"
            )
        if k.shape != (3, 3):
            raise WaypointPlannerError(
                f"camera_intrinsics must be 3x3, got shape {k.shape}"
            )
        if t.shape != (4, 4):
            raise WaypointPlannerError(
                f"T_camera_to_base must be 4x4, got shape {t.shape}"
            )

        if (
            mesh is None
            or len(getattr(mesh, "vertices", [])) < 10
            or len(getattr(mesh, "faces", [])) < 1
        ):
            raise WaypointPlannerError("mesh is empty or too small")

        n_faces0 = int(len(mesh.faces))
        verts_m = self._vertices_meters(mesh)

        if isinstance(masks, np.ndarray):
            combined = masks.astype(bool)
        else:
            combined = self._rasterize_masks(masks, img_sz)

        leg_masks = split_jeans_mask(
            combined,
            depth_threshold_ratio=self.depth_threshold_ratio,
            overlap_px=0.0,
        )

        uv, z_ok = self._project_vertices(verts_m, k, t)
        labels = self._label_vertices(uv, z_ok, leg_masks, img_sz)

        full_tree = cKDTree(verts_m)
        vertex_normals = np.asarray(mesh.vertex_normals, dtype=np.float64)
        if vertex_normals.shape[0] != verts_m.shape[0]:
            raise WaypointPlannerError("mesh.vertex_normals size mismatch")

        if len(leg_masks) == 1:
            face_keep = self._faces_for_labels(
                mesh, labels, {LABEL_LEFT, LABEL_RIGHT, LABEL_OVERLAP}
            )
            samples = self._sample_leg_faces(
                mesh, verts_m, face_keep, full_tree, vertex_normals, leg_id=0
            )
            if not samples:
                raise WaypointPlannerError(
                    "no waypoints sampled on the single-leg region"
                )
        else:
            left_faces = self._faces_for_labels(
                mesh, labels, {LABEL_LEFT, LABEL_OVERLAP}
            )
            right_faces = self._faces_for_labels(
                mesh, labels, {LABEL_RIGHT, LABEL_OVERLAP}
            )
            left_pts = self._sample_leg_faces(
                mesh, verts_m, left_faces, full_tree, vertex_normals, leg_id=0
            )
            right_pts = self._sample_leg_faces(
                mesh, verts_m, right_faces, full_tree, vertex_normals, leg_id=1
            )
            if not left_pts and not right_pts:
                raise WaypointPlannerError("no waypoints sampled on either leg")
            samples = self._concat_and_dedup(left_pts, right_pts)

        if int(len(mesh.faces)) != n_faces0:
            raise WaypointPlannerError(
                "mesh faces were mutated during planning; this is a bug"
            )

        smoother = PathNormalSmoother(window_size=self.normal_smooth_window)
        samples = smoother.smooth(samples)
        points = self._samples_to_waypoints(samples, k, t)
        if not points:
            raise WaypointPlannerError(
                "waypoint conversion produced an empty path"
            )

        return {
            "paths": [
                {
                    "path_id": 1,
                    "name": "Auto Path",
                    "points": points,
                }
            ],
            "standoff_distance_mm": float(self.spray_dist_mm),
            "type": "auto",
            "coordinate_frame": "base_link",
        }

    def _vertices_meters(self, mesh: trimesh.Trimesh) -> np.ndarray:
        verts = np.asarray(mesh.vertices, dtype=np.float64)
        if self.mesh_unit == "mm":
            return verts / 1000.0
        return verts.copy()

    @staticmethod
    def _rasterize_masks(masks_data: dict, image_size: Tuple[int, int]) -> np.ndarray:
        if not isinstance(masks_data, dict):
            raise WaypointPlannerError("masks_data must be a parsed dict")
        items = masks_data.get("masks", [])
        if not items:
            raise WaypointPlannerError("no masks defined in masks_data")
        width, height = image_size
        canvas = np.zeros((height, width), dtype=np.uint8)
        n_poly = 0
        for item in items:
            for poly in item.get("polygons", []) or []:
                if poly is None or len(poly) < 3:
                    continue
                pts = np.asarray(poly, dtype=np.int32)
                if pts.ndim != 2 or pts.shape[1] < 2:
                    continue
                cv2.fillPoly(canvas, [pts[:, :2]], 255)
                n_poly += 1
        if n_poly == 0 or int(np.count_nonzero(canvas)) < 50:
            raise WaypointPlannerError("mask area is empty or too small (< 50 px)")
        return canvas > 0

    @staticmethod
    def _project_vertices(
        verts_m: np.ndarray,
        k: np.ndarray,
        t_camera_to_base: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        t_bc = np.linalg.inv(t_camera_to_base)
        r = t_bc[:3, :3]
        t = t_bc[:3, 3]
        p_cam = (r @ verts_m.T).T + t
        z = p_cam[:, 2]
        z_ok = z > 1e-6
        uv = np.full((verts_m.shape[0], 2), -1.0, dtype=np.float64)
        fx, fy = float(k[0, 0]), float(k[1, 1])
        cx, cy = float(k[0, 2]), float(k[1, 2])
        if np.any(z_ok):
            uv[z_ok, 0] = fx * p_cam[z_ok, 0] / z[z_ok] + cx
            uv[z_ok, 1] = fy * p_cam[z_ok, 1] / z[z_ok] + cy
        return uv, z_ok

    @staticmethod
    def _label_vertices(
        uv: np.ndarray,
        z_ok: np.ndarray,
        leg_masks: List[np.ndarray],
        image_size: Tuple[int, int],
    ) -> np.ndarray:
        width, height = image_size
        labels = np.full(uv.shape[0], LABEL_NONE, dtype=np.int32)
        u = np.rint(uv[:, 0]).astype(np.int32)
        v = np.rint(uv[:, 1]).astype(np.int32)
        in_img = z_ok & (u >= 0) & (u < width) & (v >= 0) & (v < height)
        if not np.any(in_img):
            raise WaypointPlannerError(
                "no mesh vertices project into image; check K/T calibration and mesh unit"
            )

        if len(leg_masks) == 1:
            m0 = leg_masks[0]
            hit = np.zeros(uv.shape[0], dtype=bool)
            hit[in_img] = m0[v[in_img], u[in_img]]
            labels[hit] = LABEL_LEFT
            return labels

        left, right = leg_masks[0], leg_masks[1]
        in_l = np.zeros(uv.shape[0], dtype=bool)
        in_r = np.zeros(uv.shape[0], dtype=bool)
        in_l[in_img] = left[v[in_img], u[in_img]]
        in_r[in_img] = right[v[in_img], u[in_img]]
        labels[in_l & ~in_r] = LABEL_LEFT
        labels[in_r & ~in_l] = LABEL_RIGHT
        labels[in_l & in_r] = LABEL_OVERLAP
        if not np.any(labels != LABEL_NONE):
            raise WaypointPlannerError("projected vertices miss both leg masks")
        return labels

    @staticmethod
    def _faces_for_labels(
        mesh: trimesh.Trimesh,
        labels: np.ndarray,
        accept: set[int],
    ) -> np.ndarray:
        faces = np.asarray(mesh.faces, dtype=np.int64)
        accept_mask = np.zeros(labels.shape[0], dtype=bool)
        for lab in accept:
            accept_mask |= labels == lab
        return np.any(accept_mask[faces], axis=1)

    def _sample_leg_faces(
        self,
        mesh: trimesh.Trimesh,
        verts_m: np.ndarray,
        face_keep: np.ndarray,
        full_tree: cKDTree,
        vertex_normals: np.ndarray,
        leg_id: int,
    ) -> List[dict[str, Any]]:
        if not np.any(face_keep):
            logger.warning("leg %d has no faces after labeling", leg_id)
            return []
        faces = np.asarray(mesh.faces)[face_keep]
        leg_mesh = trimesh.Trimesh(vertices=verts_m, faces=faces, process=False)
        used = np.unique(faces.reshape(-1))
        pca_verts = verts_m[used]
        if pca_verts.shape[0] < 10:
            logger.warning(
                "leg %d has too few vertices (%d)", leg_id, pca_verts.shape[0]
            )
            return []
        samples = self._zigzag_sample(
            leg_mesh, pca_verts, full_tree, vertex_normals
        )
        for p in samples:
            p["leg_id"] = leg_id
        return samples

    @staticmethod
    def _pca_main_axis(vertices: np.ndarray) -> np.ndarray:
        centered = vertices - vertices.mean(axis=0)
        cov = np.cov(centered.T)
        _w, vecs = np.linalg.eigh(cov)
        main = np.asarray(vecs[:, 2], dtype=np.float64)
        if main[1] < 0.0:
            main = -main
        n = np.linalg.norm(main)
        if n < 1e-9:
            return np.array([0.0, 1.0, 0.0], dtype=np.float64)
        return main / n

    @staticmethod
    def _transverse_axis(main: np.ndarray) -> np.ndarray:
        trans = np.cross(np.array([1.0, 0.0, 0.0], dtype=np.float64), main)
        n = np.linalg.norm(trans)
        if n < 1e-5:
            return np.array([0.0, 1.0, 0.0], dtype=np.float64)
        return trans / n

    def _fit_outer_edge_axis(
        self, vertices: np.ndarray
    ) -> Tuple[np.ndarray, bool]:
        rough_main = self._pca_main_axis(vertices)
        rough_trans = self._transverse_axis(rough_main)
        v_long = vertices @ rough_main
        v_trans = vertices @ rough_trans
        min_l, max_l = float(v_long.min()), float(v_long.max())
        bins = np.linspace(min_l, max_l, 21)
        left_edge, right_edge = [], []
        for i in range(20):
            mask = (v_long >= bins[i]) & (v_long <= bins[i + 1])
            if not np.any(mask):
                continue
            bt, bl = v_trans[mask], v_long[mask]
            left_edge.append([bt[int(np.argmin(bt))], bl[int(np.argmin(bt))]])
            right_edge.append([bt[int(np.argmax(bt))], bl[int(np.argmax(bt))]])
        left_edge = np.asarray(left_edge, dtype=np.float64)
        right_edge = np.asarray(right_edge, dtype=np.float64)

        def fit_edge(edge_pts: np.ndarray) -> Tuple[np.ndarray, float]:
            if edge_pts.shape[0] < 2:
                return rough_main, float("inf")
            centered = edge_pts - edge_pts.mean(axis=0)
            _u, s, vh = np.linalg.svd(centered, full_matrices=False)
            d_trans, d_long = float(vh[0, 0]), float(vh[0, 1])
            if d_long < 0.0:
                d_trans, d_long = -d_trans, -d_long
            err = float(s[1]) if s.size > 1 else 0.0
            axis = d_trans * rough_trans + d_long * rough_main
            n = np.linalg.norm(axis)
            if n < 1e-9:
                return rough_main, err
            return axis / n, err

        left_axis, left_err = fit_edge(left_edge)
        right_axis, right_err = fit_edge(right_edge)
        if left_err <= right_err:
            logger.info(
                "leg outer-edge: left seam err=%.4f (right=%.4f)",
                left_err,
                right_err,
            )
            return left_axis, True
        logger.info(
            "leg outer-edge: right seam err=%.4f (left=%.4f)",
            right_err,
            left_err,
        )
        return right_axis, False

    def _zigzag_sample(
        self,
        slice_mesh: trimesh.Trimesh,
        pca_verts: np.ndarray,
        full_tree: cKDTree,
        vertex_normals: np.ndarray,
    ) -> List[dict[str, Any]]:
        row_s = self.row_spacing_mm / 1000.0
        pt_s = self.point_spacing_mm / 1000.0
        if self.align_outer_edge:
            edge_axis, is_left = self._fit_outer_edge_axis(pca_verts)
        else:
            edge_axis, is_left = self._pca_main_axis(pca_verts), True

        plane_normal = np.cross(
            np.array([1.0, 0.0, 0.0], dtype=np.float64), edge_axis
        )
        n = np.linalg.norm(plane_normal)
        if n < 1e-8:
            plane_normal = self._transverse_axis(edge_axis)
        else:
            plane_normal = plane_normal / n
        if plane_normal[1] < 0.0:
            plane_normal = -plane_normal

        projections = pca_verts @ plane_normal
        min_proj, max_proj = float(projections.min()), float(projections.max())
        if max_proj - min_proj < row_s * 0.5:
            logger.warning(
                "leg span %.1f mm is smaller than half row spacing",
                (max_proj - min_proj) * 1000.0,
            )
        if is_left:
            slice_projs = np.arange(min_proj + row_s / 2.0, max_proj + 1e-12, row_s)
        else:
            slice_projs = np.arange(max_proj - row_s / 2.0, min_proj - 1e-12, -row_s)

        zigzag: List[dict[str, Any]] = []
        direction_forward = True
        for proj in slice_projs:
            lines = trimesh.intersections.mesh_plane(
                slice_mesh,
                plane_normal=plane_normal,
                plane_origin=plane_normal * float(proj),
            )
            if lines is None or len(lines) == 0:
                continue
            pts = np.unique(
                np.round(
                    np.asarray(lines, dtype=np.float64).reshape(-1, 3),
                    decimals=4,
                ),
                axis=0,
            )
            if pts.shape[0] < 2:
                continue
            order = np.argsort(pts @ edge_axis)
            sorted_pts = pts[order]
            diffs = np.diff(sorted_pts, axis=0)
            dists = np.linalg.norm(diffs, axis=1)
            cum = np.insert(np.cumsum(dists), 0, 0.0)
            total = float(cum[-1])
            if total < pt_s * 0.25:
                continue
            sample_d = np.arange(0.0, total, pt_s)
            if sample_d.size == 0:
                continue
            # 补行末端点: arange 严格 < total 会丢掉每行沿主轴 (edge_axis) 的极值端点,
            # 即 PCA 上下两个顶点所在行够不到真正的边界。显式并入 total;
            # 与末个采样点几乎重合 (< 1/4 点距) 时跳过, 避免生成重复航点。
            if total - float(sample_d[-1]) > pt_s * 0.25:
                sample_d = np.append(sample_d, total)
            sampled = np.zeros((sample_d.size, 3), dtype=np.float64)
            for i in range(3):
                sampled[:, i] = np.interp(sample_d, cum, sorted_pts[:, i])
            if not direction_forward:
                sampled = sampled[::-1]
            _, idx = full_tree.query(sampled)
            normals = vertex_normals[idx]
            for i_pt, (p, nrm) in enumerate(zip(sampled, normals)):
                nlen = float(np.linalg.norm(nrm))
                if nlen < 1e-9:
                    nrm = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                else:
                    nrm = nrm / nlen
                zigzag.append(
                    {
                        "point": p.astype(np.float64).tolist(),
                        "normal": nrm.astype(np.float64).tolist(),
                        "is_jump": bool(len(zigzag) > 0 and i_pt == 0),
                    }
                )
            direction_forward = not direction_forward
        return zigzag

    def _concat_and_dedup(
        self,
        left_pts: Sequence[dict[str, Any]],
        right_pts: Sequence[dict[str, Any]],
    ) -> List[dict[str, Any]]:
        if not right_pts:
            return list(left_pts)
        if not left_pts:
            return list(right_pts)

        radius_m = max(self.dedup_radius_mm, 0.0) / 1000.0
        keep_right = np.ones(len(right_pts), dtype=bool)
        if radius_m > 0.0:
            left_xyz = np.asarray([p["point"] for p in left_pts], dtype=np.float64)
            right_xyz = np.asarray(
                [p["point"] for p in right_pts], dtype=np.float64
            )
            d_lr, _ = cKDTree(left_xyz).query(right_xyz, k=1)
            drop = d_lr <= radius_m
            keep_right[drop] = False
            logger.info(
                "leg-seam dedup: dropped %d / %d right-leg points (radius=%.1f mm)",
                int(np.count_nonzero(drop)),
                len(right_pts),
                self.dedup_radius_mm,
            )

        kept_right = [p for p, k in zip(right_pts, keep_right) if k]
        if kept_right:
            first = dict(kept_right[0])
            first["is_jump"] = True
            kept_right[0] = first
        return list(left_pts) + kept_right

    def _samples_to_waypoints(
        self,
        samples: Sequence[dict[str, Any]],
        k: np.ndarray,
        t_camera_to_base: np.ndarray,
    ) -> List[dict[str, Any]]:
        fx, fy = float(k[0, 0]), float(k[1, 1])
        cx, cy = float(k[0, 2]), float(k[1, 2])
        t_bc = np.linalg.inv(t_camera_to_base)
        r_bc, t_bc_trans = t_bc[:3, :3], t_bc[:3, 3]

        points: List[dict[str, Any]] = []
        prev_x_tool: Optional[np.ndarray] = None

        for i, s in enumerate(samples):
            p_m = np.asarray(s["point"], dtype=np.float64)
            n_base = np.asarray(s["normal"], dtype=np.float64)
            nlen = float(np.linalg.norm(n_base))
            if nlen < 1e-9:
                n_base = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                n_base = n_base / nlen

            p_surf_mm = p_m * 1000.0
            p_tcp_mm = p_surf_mm + self.spray_dist_mm * n_base
            z_tool = -n_base

            is_jump = bool(s.get("is_jump", False))
            # 遇到换步跳跃点时，主动重置参考轴，防止跨扫描线姿态扭曲
            if is_jump:
                prev_x_tool = None

            # 连续标架跟踪法 (Bug 1 修复：增加与 z_tool 的平行共线检测，彻底避免 90 度奇异翻转)
            if prev_x_tool is not None and abs(float(np.dot(z_tool, prev_x_tool))) < 0.90:
                x_ref = prev_x_tool
            else:
                x_ref = (
                    np.array([1.0, 0.0, 0.0], dtype=np.float64)
                    if abs(z_tool[2]) > 0.707
                    else np.array([0.0, 0.0, 1.0], dtype=np.float64)
                )

            y_tool = np.cross(z_tool, x_ref)
            ylen = float(np.linalg.norm(y_tool))
            if ylen < 1e-9:
                y_tool = np.array([0.0, 1.0, 0.0], dtype=np.float64)
            else:
                y_tool = y_tool / ylen

            x_tool = np.cross(y_tool, z_tool)
            xlen = float(np.linalg.norm(x_tool))
            if xlen < 1e-9:
                x_tool = np.array([1.0, 0.0, 0.0], dtype=np.float64)
            else:
                x_tool = x_tool / xlen
            prev_x_tool = x_tool

            euler = R_tool.from_matrix(
                np.column_stack((x_tool, y_tool, z_tool))
            ).as_euler("xyz", degrees=True)

            p_cam_m = r_bc @ p_m + t_bc_trans
            n_cam = r_bc @ n_base
            ncam_len = float(np.linalg.norm(n_cam))
            if ncam_len > 1e-9:
                n_cam = n_cam / ncam_len

            if p_cam_m[2] > 1e-6:
                u = fx * p_cam_m[0] / p_cam_m[2] + cx
                v = fy * p_cam_m[1] / p_cam_m[2] + cy
                p_tcp_cam = p_cam_m + (self.spray_dist_mm / 1000.0) * n_cam
                if p_tcp_cam[2] > 1e-6:
                    u_tcp = fx * p_tcp_cam[0] / p_tcp_cam[2] + cx
                    v_tcp = fy * p_tcp_cam[1] / p_tcp_cam[2] + cy
                    proj = [round(float(u_tcp - u), 1), round(float(v_tcp - v), 1)]
                else:
                    proj = [0.0, 0.0]
            else:
                u, v = 0.0, 0.0
                proj = [0.0, 0.0]

            points.append(
                {
                    "index": i + 1,
                    "pixel": [int(round(u)), int(round(v))],
                    "surface_point_cam_mm": [
                        round(float(x) * 1000.0, 2) for x in p_cam_m
                    ],
                    "surface_point_base_mm": [
                        round(float(x), 2) for x in p_surf_mm
                    ],
                    "surface_normal_base": [
                        round(float(x), 4) for x in n_base
                    ],
                    "surface_normal_cam": [round(float(x), 4) for x in n_cam],
                    "standoff_distance_mm": round(float(self.spray_dist_mm), 1),
                    "tcp_pose_base": {
                        "x": round(float(p_tcp_mm[0]), 2),
                        "y": round(float(p_tcp_mm[1]), 2),
                        "z": round(float(p_tcp_mm[2]), 2),
                        "rx": round(float(euler[0]), 2),
                        "ry": round(float(euler[1]), 2),
                        "rz": round(float(euler[2]), 2),
                    },
                    "normal_2d_proj": proj,
                    "spraying": "off" if is_jump else "on",
                    "is_jump": is_jump,
                    "leg_id": int(s.get("leg_id", 0)),
                }
            )
        return points
