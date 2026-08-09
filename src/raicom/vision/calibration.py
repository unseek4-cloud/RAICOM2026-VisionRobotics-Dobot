# -*- coding: utf-8 -*-
"""EIH 手眼标定读取和像素坐标定位。

本模块只负责坐标计算，不发送任何机器人指令。正式模式坚持“失败关闭”：
标定文件、深度、单位、工作空间中任意一项不可信时直接抛出中文异常，绝不
返回猜测坐标。机器人抓取 Z 严格按赛前实测的台面参考值计算，而不是使用
EIH 变换得到的物理点 Z。
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, isfinite, sin
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np

from raicom.config import Settings
from raicom.types import CameraIntrinsics


class CalibrationError(RuntimeError):
    """标定数据或坐标计算不安全。"""


def _number(value: Any, name: str) -> float:
    """读取有限浮点数，并给出可直接定位配置项的错误。"""
    if isinstance(value, bool):
        raise CalibrationError(f"{name} 必须是数字，不能是布尔值")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise CalibrationError(f"{name} 必须填写有限数字，当前为：{value!r}") from exc
    if not isfinite(result):
        raise CalibrationError(f"{name} 不能是 NaN 或无穷大")
    return result


def _vector(value: Any, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise CalibrationError(f"{name} 必须包含 {length} 个数字")
    return tuple(_number(item, f"{name}[{index}]") for index, item in enumerate(value))


def _axis_rotation(axis: str, angle_degree: float) -> np.ndarray:
    angle = np.deg2rad(angle_degree)
    c, s = cos(angle), sin(angle)
    if axis == "x":
        return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))
    if axis == "y":
        return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))
    if axis == "z":
        return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
    raise CalibrationError(f"未知旋转轴：{axis}")


def _pose_matrix(pose_mm_deg: Sequence[float], order: str) -> np.ndarray:
    """把 [X,Y,Z,Rx,Ry,Rz] 变为 ``基座 <- Tip`` 齐次矩阵。

    配置 ``zyx`` 明确表示 ``Rz @ Ry @ Rx``，与现有 settings.yaml 的说明
    一致。也支持其余不重复的三轴排列，便于现场按控制器文档调整。
    """
    if len(pose_mm_deg) != 6:
        raise CalibrationError("拍照位必须是 [X,Y,Z,Rx,Ry,Rz] 六个值")
    order = str(order).strip().lower()
    if len(order) != 3 or set(order) != {"x", "y", "z"}:
        raise CalibrationError(
            "calibration.pose_rotation_order 必须是 xyz 三轴的不重复排列，例如 zyx"
        )
    x, y, z, rx, ry, rz = (float(v) for v in pose_mm_deg)
    angles = {"x": rx, "y": ry, "z": rz}
    rotation = np.eye(3, dtype=np.float64)
    for axis in order:
        rotation = rotation @ _axis_rotation(axis, angles[axis])
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = (x, y, z)
    return transform


def _read_opencv_matrix(
    path: Path, node_name: str
) -> tuple[np.ndarray, dict[str, str]]:
    try:
        import cv2
    except ImportError as exc:  # pragma: no cover - 环境检查会先提示依赖
        raise CalibrationError("读取 EIH YAML 需要 OpenCV（cv2）") from exc

    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise CalibrationError(f"无法打开 EIH 标定文件：{path}")
    try:
        node = storage.getNode(node_name)
        if node.empty():
            raise CalibrationError(f"标定文件中缺少矩阵节点：{node_name}")
        matrix = node.mat()
        metadata: dict[str, str] = {}
        for key in ("Dimension type", "Calibration type", "CameraName", "Time"):
            item = storage.getNode(key)
            if item.empty():
                continue
            try:
                metadata[key] = str(item.string())
            except Exception:
                # 元数据用于防呆，矩阵节点才是运行必需项；旧版工具若字符串接口
                # 不兼容则保留为空，后续仍由刚体矩阵和现场多点验证兜底。
                continue
    finally:
        storage.release()
    if matrix is None:
        raise CalibrationError(f"标定节点 {node_name} 不是 OpenCV 矩阵")
    return np.asarray(matrix, dtype=np.float64), metadata


def _validate_rigid_transform(matrix: np.ndarray, source: str) -> None:
    if matrix.shape != (4, 4):
        raise CalibrationError(f"{source} 必须是 4×4，当前形状为 {matrix.shape}")
    if not np.isfinite(matrix).all():
        raise CalibrationError(f"{source} 含 NaN 或无穷大")
    if not np.allclose(matrix[3], (0.0, 0.0, 0.0, 1.0), atol=1e-7):
        raise CalibrationError(f"{source} 最后一行必须为 [0,0,0,1]")
    rotation = matrix[:3, :3]
    orthogonality_error = float(np.linalg.norm(rotation.T @ rotation - np.eye(3)))
    determinant = float(np.linalg.det(rotation))
    if orthogonality_error > 1e-3 or abs(determinant - 1.0) > 1e-3:
        raise CalibrationError(
            f"{source} 旋转矩阵非法：det={determinant:.6f}，"
            f"正交误差={orthogonality_error:.3g}"
        )


@dataclass(slots=True)
class CalibrationModel:
    """固定拍照位下的 EIH 坐标计算模型。"""

    table_depth_mm: float
    robot_table_touch_z_mm: float
    press_down_mm: float
    min_object_height_mm: float
    max_object_height_mm: float
    xy_offset_mm: tuple[float, float]
    workspace_mm: dict[str, tuple[float, float]]
    simulation: bool = False
    camera_to_tip_mm: np.ndarray | None = None
    base_to_tip_mm: np.ndarray | None = None

    @classmethod
    def from_settings(
        cls, settings: Settings, simulation: bool = False
    ) -> "CalibrationModel":
        """从集中配置创建模型，并在创建阶段完成单位和矩阵防呆。"""
        if simulation:
            table_depth = _number(
                settings.get(
                    "simulation.table_depth_mm",
                    settings.get("calibration.table_depth_mm", 700.0),
                ),
                "simulation.table_depth_mm",
            )
            table_touch = _number(
                settings.get(
                    "simulation.robot_table_touch_z_mm",
                    settings.get("calibration.robot_table_touch_z_mm", 100.0),
                ),
                "simulation.robot_table_touch_z_mm",
            )
        else:
            table_depth = _number(
                settings.get("calibration.table_depth_mm"),
                "calibration.table_depth_mm",
            )
            table_touch = _number(
                settings.get("calibration.robot_table_touch_z_mm"),
                "calibration.robot_table_touch_z_mm",
            )

        if table_depth <= 0:
            raise CalibrationError("空台面深度必须大于 0 mm")
        press = _number(settings.get("calibration.press_down_mm", 0.0), "press_down_mm")
        if press < 0:
            raise CalibrationError("calibration.press_down_mm 不能为负数")
        min_height = _number(
            settings.get("calibration.min_object_height_mm", 1.0),
            "calibration.min_object_height_mm",
        )
        max_height = _number(
            settings.get("calibration.max_object_height_mm", 120.0),
            "calibration.max_object_height_mm",
        )
        if min_height < 0 or max_height <= min_height:
            raise CalibrationError("工件高度上下限配置不合理")
        xy_offset = _vector(
            settings.get("calibration.xy_offset_mm", [0.0, 0.0]),
            2,
            "calibration.xy_offset_mm",
        )
        workspace = cls._workspace_from_settings(settings, required=not simulation)

        if simulation:
            return cls(
                table_depth_mm=table_depth,
                robot_table_touch_z_mm=table_touch,
                press_down_mm=press,
                min_object_height_mm=min_height,
                max_object_height_mm=max_height,
                xy_offset_mm=(xy_offset[0], xy_offset[1]),
                workspace_mm=workspace,
                simulation=True,
            )

        path = settings.resolve_path("calibration.eih_yaml", required=True)
        if not path.is_file():
            raise CalibrationError(f"EIH 标定文件不存在：{path}")
        node_name = str(settings.get("calibration.transform_node", "CamToTipTransform"))
        camera_to_tip, metadata = _read_opencv_matrix(path, node_name)
        dimension = metadata.get("Dimension type", "").strip().upper()
        calibration_type = metadata.get("Calibration type", "").strip()
        if dimension and dimension != "3D":
            raise CalibrationError(
                f"标定文件 Dimension type={dimension!r}，任务二/三必须使用 3D EIH"
            )
        folded_type = calibration_type.casefold()
        if calibration_type and "eih" not in folded_type and "眼在手" not in calibration_type:
            raise CalibrationError(
                f"标定文件 Calibration type={calibration_type!r}，不是眼在手上 EIH 标定"
            )
        _validate_rigid_transform(camera_to_tip, node_name)

        unit = str(settings.get("calibration.matrix_translation_unit", "m")).lower()
        if unit not in {"m", "mm"}:
            raise CalibrationError("matrix_translation_unit 只能是 m 或 mm")
        camera_to_tip = camera_to_tip.copy()
        if unit == "m":
            camera_to_tip[:3, 3] *= 1000.0
        translation_length = float(np.linalg.norm(camera_to_tip[:3, 3]))
        if not 0.5 <= translation_length <= 2000.0:
            raise CalibrationError(
                "CamToTip 平移长度异常："
                f"{translation_length:.3f} mm；请检查 m/mm 单位设置"
            )
        if bool(settings.get("calibration.invert_cam_to_tip", False)):
            camera_to_tip = np.linalg.inv(camera_to_tip)
            _validate_rigid_transform(camera_to_tip, f"inverse({node_name})")

        photo_pose = _vector(
            settings.get("robot.photo_pose_mm_deg"), 6, "robot.photo_pose_mm_deg"
        )
        order = str(settings.get("calibration.pose_rotation_order", "zyx"))
        base_to_tip = _pose_matrix(photo_pose, order)

        return cls(
            table_depth_mm=table_depth,
            robot_table_touch_z_mm=table_touch,
            press_down_mm=press,
            min_object_height_mm=min_height,
            max_object_height_mm=max_height,
            xy_offset_mm=(xy_offset[0], xy_offset[1]),
            workspace_mm=workspace,
            simulation=False,
            camera_to_tip_mm=camera_to_tip,
            base_to_tip_mm=base_to_tip,
        )

    @staticmethod
    def _workspace_from_settings(
        settings: Settings, *, required: bool
    ) -> dict[str, tuple[float, float]]:
        result: dict[str, tuple[float, float]] = {}
        for axis in ("x", "y", "z"):
            raw = settings.get(f"robot.workspace_mm.{axis}", None)
            try:
                bounds = _vector(raw, 2, f"robot.workspace_mm.{axis}")
            except CalibrationError:
                if required:
                    raise
                continue
            if bounds[0] >= bounds[1]:
                if required:
                    raise CalibrationError(
                        f"robot.workspace_mm.{axis} 下限必须小于上限"
                    )
                continue
            result[axis] = (bounds[0], bounds[1])
        return result

    @staticmethod
    def _camera_point(
        pixel: tuple[int | float, int | float],
        depth_mm: float,
        intrinsics: CameraIntrinsics,
    ) -> tuple[float, float, float]:
        u, v = _number(pixel[0], "pixel.u"), _number(pixel[1], "pixel.v")
        depth = _number(depth_mm, "depth_mm")
        if depth <= 0:
            raise CalibrationError("工件深度必须大于 0 mm")
        if intrinsics.width <= 0 or intrinsics.height <= 0:
            raise CalibrationError("相机图像尺寸无效")
        if not (0 <= u < intrinsics.width and 0 <= v < intrinsics.height):
            raise CalibrationError(
                f"像素 ({u:.1f},{v:.1f}) 超出图像 "
                f"{intrinsics.width}×{intrinsics.height}"
            )
        fx = _number(intrinsics.fx, "intrinsics.fx")
        fy = _number(intrinsics.fy, "intrinsics.fy")
        ppx = _number(intrinsics.ppx, "intrinsics.ppx")
        ppy = _number(intrinsics.ppy, "intrinsics.ppy")
        if fx <= 0 or fy <= 0:
            raise CalibrationError("相机焦距 fx/fy 必须大于 0")

        native = getattr(intrinsics, "native", None)
        if native is not None:
            try:
                import pyrealsense2 as rs

                point = rs.rs2_deproject_pixel_to_point(native, [u, v], depth)
                if len(point) != 3 or not np.isfinite(point).all():
                    raise ValueError("SDK 返回非有限坐标")
                return float(point[0]), float(point[1]), float(point[2])
            except Exception as exc:
                # 真机已有原生内参时不能悄悄退化为无畸变针孔公式，否则边缘区域
                # 会出现难以察觉的系统误差，应让操作者先解决 SDK/内参问题。
                raise CalibrationError(f"RealSense 原生反投影失败：{exc}") from exc

        # 模拟模式和不带原生 SDK 对象的离线验证使用标准针孔模型。
        x = (u - ppx) * depth / fx
        y = (v - ppy) * depth / fy
        return float(x), float(y), depth

    def locate(
        self,
        pixel: tuple[int | float, int | float],
        depth_mm: float,
        intrinsics: CameraIntrinsics,
    ) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        """返回 ``(相机XYZ毫米, 机器人抓取XYZ毫米)``。

        Z 的唯一公式为：
        ``table_robot_touch + table_depth - object_depth - press``。
        深度差超出工件高度范围时立即拒绝，不做钳位和默认值兜底。
        """
        camera_xyz = self._camera_point(pixel, depth_mm, intrinsics)
        object_height = self.table_depth_mm - camera_xyz[2]
        if not self.min_object_height_mm <= object_height <= self.max_object_height_mm:
            raise CalibrationError(
                f"工件高度 {object_height:.2f} mm 超出允许范围 "
                f"[{self.min_object_height_mm:.2f}, {self.max_object_height_mm:.2f}]；"
                "请检查台面深度、取样位置或相机拍照位"
            )
        robot_z = self.robot_table_touch_z_mm + object_height - self.press_down_mm

        if self.simulation:
            # 模拟坐标只用于软件联调：把图像中心映射到机器人工作区附近，
            # 不冒充真实手眼标定结果。
            u, v = float(pixel[0]), float(pixel[1])
            robot_x = 250.0 + (u - intrinsics.ppx) * 0.35 + self.xy_offset_mm[0]
            robot_y = (v - intrinsics.ppy) * 0.35 + self.xy_offset_mm[1]
        else:
            if self.camera_to_tip_mm is None or self.base_to_tip_mm is None:
                raise CalibrationError("真实模式 EIH 变换尚未加载")
            point = np.array((*camera_xyz, 1.0), dtype=np.float64)
            robot_point = self.base_to_tip_mm @ self.camera_to_tip_mm @ point
            if not np.isfinite(robot_point).all() or abs(robot_point[3]) < 1e-9:
                raise CalibrationError("EIH 变换结果无效")
            robot_x = float(robot_point[0] / robot_point[3]) + self.xy_offset_mm[0]
            robot_y = float(robot_point[1] / robot_point[3]) + self.xy_offset_mm[1]

        robot_xyz = (float(robot_x), float(robot_y), float(robot_z))
        self.validate_workspace(robot_xyz)
        return camera_xyz, robot_xyz

    def validate_workspace(self, robot_xyz_mm: Sequence[float]) -> None:
        """校验三轴软件工作空间；模拟配置未填写边界时跳过对应轴。"""
        if len(robot_xyz_mm) != 3:
            raise CalibrationError("机器人坐标必须包含 X、Y、Z 三个值")
        for axis, value in zip(("x", "y", "z"), robot_xyz_mm):
            number = _number(value, f"robot_{axis}_mm")
            bounds = self.workspace_mm.get(axis)
            if bounds is not None and not bounds[0] <= number <= bounds[1]:
                raise CalibrationError(
                    f"机器人 {axis.upper()}={number:.2f} mm 超出软件工作空间 "
                    f"[{bounds[0]:.2f}, {bounds[1]:.2f}]"
                )

    def validate_reference_points(
        self,
        samples: Iterable[
            tuple[
                tuple[int | float, int | float],
                float,
                CameraIntrinsics,
                tuple[float, float],
            ]
        ],
        *,
        max_xy_error_mm: float = 5.0,
        minimum_points: int = 3,
    ) -> list[float]:
        """用至少三个已知点防呆检查矩阵方向、单位和欧拉角约定。

        返回各点 XY 误差。只要点数不足或任一点超差就抛出异常，现场可据此
        决定是否切换 ``invert_cam_to_tip``，但程序不会自动猜方向。
        """
        errors: list[float] = []
        for pixel, depth, intrinsics, expected_xy in samples:
            _, actual = self.locate(pixel, depth, intrinsics)
            expected = _vector(expected_xy, 2, "expected_robot_xy_mm")
            error = float(np.hypot(actual[0] - expected[0], actual[1] - expected[1]))
            errors.append(error)
        if len(errors) < minimum_points:
            raise CalibrationError(f"手眼方向验证至少需要 {minimum_points} 个分散点")
        limit = _number(max_xy_error_mm, "max_xy_error_mm")
        failed = [value for value in errors if value > limit]
        if failed:
            raise CalibrationError(
                f"手眼多点验证失败：最大 XY 误差 {max(errors):.2f} mm，"
                f"允许 {limit:.2f} mm；请检查矩阵方向、m/mm 和姿态旋转顺序"
            )
        return errors


__all__ = ["CalibrationError", "CalibrationModel"]
