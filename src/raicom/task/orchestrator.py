# -*- coding: utf-8 -*-
"""任务一→3D识别抓取的唯一顺序控制状态机。

设计重点：
- 3D识别抓取只加载一套七分类 OBB 模型；
- 每抓一件都等待 Lua 返回 HOME，再在固定拍照位重新取帧，禁止用旧图算新坐标；
- 深度无效、标定异常、坐标越界、ACK 超时均立即拒绝运动；
- 停止后绝不在 finally 中自动发回拍照位等新运动命令。
"""

from __future__ import annotations

import logging
import math
import statistics
import threading
import time
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from ..config import Settings
from ..events import EventBus
from ..interfaces import CalibrationLike, CameraLike, DVSLike, DetectorLike, RobotLike
from ..recognition_region import RecognitionRegionStore
from ..result_store import ResultStore
from ..types import Detection, DirectPlaceTarget, TaskState
from ..vision.yolo_detector import oriented_box_axis


class TaskError(RuntimeError):
    """需要终止当前竞赛流程的业务异常。"""


class TaskStopped(TaskError):
    """用户主动停止或超过比赛限时。"""


class TaskOrchestrator:
    def __init__(
        self,
        settings: Settings,
        bus: EventBus,
        logger: logging.Logger,
        camera: CameraLike,
        calibration: CalibrationLike,
        detectors: Mapping[str, DetectorLike],
        recognition_regions: RecognitionRegionStore,
        dvs: DVSLike,
        robot: RobotLike,
        simulation_world: Any | None = None,
    ) -> None:
        self.settings = settings
        self.bus = bus
        self.log = logger.getChild("task")
        self.camera = camera
        self.calibration = calibration
        self.detectors = dict(detectors)
        self.recognition_regions = recognition_regions
        self.dvs = dvs
        self.robot = robot
        self.simulation_world = simulation_world

        output = settings.get("application.result_log_jsonl", "logs/results.jsonl")
        output_path = Path(str(output))
        if not output_path.is_absolute():
            output_path = settings.project_root / output_path
        self.results = ResultStore(output_path)

        self._run_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._deadline = 0.0
        self.state = TaskState.IDLE

    @property
    def is_running(self) -> bool:
        return self._run_lock.locked()

    def _set_state(self, state: TaskState, detail: str = "") -> None:
        self.state = state
        self.bus.emit("task_state", state.value, detail)
        if detail:
            self.log.info("%s：%s", state.value, detail)
        else:
            self.log.info("状态切换：%s", state.value)

    def request_stop(self) -> None:
        """停止任务状态机，并请求执行端停止；不把它宣传为物理急停。"""
        self._stop_event.set()
        self._set_state(TaskState.STOPPING, "收到停止任务请求")
        try:
            self.robot.request_stop()
        except Exception as exc:
            self.log.warning("执行端停止请求发送失败：%s", exc)

    def _check_continue(self) -> None:
        if self._stop_event.is_set():
            raise TaskStopped("任务已由操作员停止")
        if self._deadline and time.monotonic() >= self._deadline:
            self._stop_event.set()
            raise TaskStopped("已达到 600 秒比赛运行上限")

    def run(self, which: str = "all") -> bool:
        if not self._run_lock.acquire(blocking=False):
            raise TaskError("已有任务正在运行，禁止并发控制机械臂")
        self._stop_event.clear()
        timeout_s = float(self.settings.get("application.competition_timeout_s", 600))
        self._deadline = time.monotonic() + timeout_s
        self.bus.emit("timer_started", timeout_s)

        try:
            sequence = ["task1", "task3"] if which == "all" else [which]
            for task_name in sequence:
                self._check_continue()
                if not bool(self.settings.get(f"tasks.{task_name}.enabled", True)):
                    self.log.info("%s 已在配置中禁用，跳过", task_name)
                    continue
                if task_name == "task1":
                    self.run_task1()
                elif task_name == "task3":
                    self.run_3d_pick_task()
                else:
                    raise TaskError(f"不支持的任务：{task_name}")

            self._check_continue()
            self._set_state(TaskState.COMPLETED, "所选任务已自动完成")
            self.bus.emit("timer_stopped")
            return True
        except TaskStopped as exc:
            self._set_state(TaskState.IDLE, str(exc))
            self.log.warning("流程停止：%s", exc)
            self.bus.emit("timer_stopped")
            return False
        except Exception as exc:
            self._set_state(TaskState.FAILED, str(exc))
            self.log.exception("任务失败：%s", exc)
            self.bus.emit("alarm", str(exc))
            self.bus.emit("timer_stopped")
            return False
        finally:
            # 关键安全约束：停止/异常后不追加任何机械臂运动。
            self._deadline = 0.0
            self._run_lock.release()

    def run_task1(self) -> None:
        self._set_state(TaskState.TASK1, "等待 Dobot Vision Studio 识别结果")
        self._check_continue()
        # 丢弃启动任务前残留的旧结果。DVS 应在收到本次 ``ok`` 后重新输出；
        # 这样不会把上一次调试/断线重发的数据误计为本轮评分结果。
        clear_results = getattr(self.dvs, "clear_results", None)
        if callable(clear_results):
            clear_results()
        try:
            triggered = self.dvs.trigger()
        except Exception as exc:
            raise TaskError(f"向 DVS 发送任务一触发字符串 ok 失败：{exc}") from exc
        if not triggered:
            raise TaskError("未能向 DVS 发送任务一触发字符串 ok；请先确认 DVS 已连接")
        self.log.info("已向 Dobot Vision Studio 发送任务一触发字符串：ok")

        timeout = float(self.settings.get("dvs.task1_timeout_s", 30.0))
        expected_messages = max(1, int(self.settings.get("dvs.expected_results", 1)))
        wait_deadline = time.monotonic() + timeout
        messages: list[dict[str, Any]] = []
        after_seq: int | None = None
        external_sequences: set[str] = set()
        while len(messages) < expected_messages:
            remaining = max(0.05, wait_deadline - time.monotonic())
            incoming = self.dvs.wait_for_result(
                timeout=remaining, after_sequence=after_seq
            )
            self._check_continue()
            if not incoming:
                raise TaskError(
                    f"任务一在 {timeout:.1f}s 内仅收到 {len(messages)}/{expected_messages} 条 DVS 结果"
                )
            item = dict(incoming)
            seq_value = item.get("_seq")
            if isinstance(seq_value, int):
                after_seq = seq_value

            self._validate_dvs_result(item)
            # ``_seq`` 是接收器内部序号；DVS 自己提供的 ``seq`` 用于识别
            # 应用层重发。重复报文只记录日志，不重复计数。
            external_seq = item.get("seq")
            if external_seq is not None:
                normalized_seq = str(external_seq)
                if normalized_seq in external_sequences:
                    self.log.warning("忽略 DVS 重复 seq=%s", external_seq)
                    continue
                external_sequences.add(normalized_seq)
            messages.append(item)

        result: dict[str, Any] = {}
        for item in messages:
            result.update({key: value for key, value in item.items() if not key.startswith("_")})
        if len(messages) > 1:
            result["messages"] = messages

        required = list(self.settings.get("dvs.required_fields", []))
        missing = [name for name in required if name not in result]
        if missing:
            raise TaskError(f"任务一结果缺少现场要求字段：{', '.join(missing)}")

        clean = dict(result)
        self.results.append("task1_result", {"task": "task1", "result": clean})
        self.bus.emit("task1_result", clean)
        self.bus.emit(
            "result_row",
            {
                "任务": "任务一",
                "类别": str(clean.get("object", clean.get("name", "DVS测量"))),
                "颜色": str(clean.get("color", "-")),
                "置信度": str(clean.get("confidence", "-")),
                "像素": "-",
                "深度(mm)": "-",
                "机器人XYZ(mm)": "-",
                "角度/RZ(°)": "-",
                "状态": "识别完成",
            },
        )
        self.log.info("任务一结果：%s", clean)

    @staticmethod
    def _validate_dvs_result(item: Mapping[str, Any]) -> None:
        """校验一条任务一结果，拒绝失败标记和非有限测量值。"""

        task_value = item.get("task")
        if task_value is not None and str(task_value).strip().casefold() not in {
            "task1",
            "任务一",
        }:
            raise TaskError(f"收到非任务一 DVS 结果：task={task_value!r}")

        version = item.get("version")
        if version is not None:
            try:
                version_number = float(version)
            except (TypeError, ValueError) as exc:
                raise TaskError(f"DVS 协议 version 无效：{version!r}") from exc
            if not math.isfinite(version_number) or version_number != 1.0:
                raise TaskError(f"不支持的 DVS 协议 version：{version!r}")

        ok_value = item.get("ok")
        if ok_value is not None:
            if isinstance(ok_value, str):
                ok = ok_value.strip().casefold() in {"1", "true", "yes", "ok", "成功"}
            elif isinstance(ok_value, (int, float)) and not isinstance(ok_value, bool):
                ok = math.isfinite(float(ok_value)) and float(ok_value) != 0.0
            else:
                ok = bool(ok_value)
            if not ok:
                raise TaskError("DVS 返回 ok=false，本次任务一结果不合格")

        for key, value in item.items():
            if isinstance(value, float) and not math.isfinite(value):
                raise TaskError(f"DVS 字段 {key} 为 NaN/Inf，拒绝该结果")

    def run_3d_pick_task(self) -> None:
        task_name = "task3"
        self._set_state(TaskState.TASK3, "检查拍照位并逐件重新识别、抓取、分类放置")
        if task_name not in self.detectors:
            raise TaskError(f"{task_name} 检测模型未加载")
        if not self.robot.is_connected:
            raise TaskError("DobotStudio Pro 执行脚本尚未连接")

        reply = self.robot.is_at_photo()
        if reply.status not in ("done", "ok"):
            code = str(reply.raw.get("code", "")).strip()
            reason = reply.message or reply.status
            if code and code not in reason:
                reason = f"{code}：{reason}"
            raise TaskError(f"无法读取机械臂当前位姿：{reason}")
        if reply.raw.get("at_photo") is not True:
            current_pose = reply.raw.get("current_pose")
            suffix = f"，当前位姿={current_pose}" if current_pose is not None else ""
            raise TaskError(f"机械臂不在固定拍照位{suffix}；请先点击“自动回到拍照位”")
        self.camera.flush()

        maximum = self.settings.task_max_objects(task_name)

        completed = 0
        # 每次抓放后都回到拍照位重新识别；现场目标少于上限时以连续空帧结束，
        # 达到配置上限时则立即正常结束，不再把桌面上的额外工件当成数量错误。
        while completed < maximum:
            self._check_continue()
            candidate = self._acquire_stable_candidate(task_name)
            if candidate is None:
                self.log.info("%s 已连续多帧无目标", task_name)
                break
            det, bundle = candidate

            try:
                depth_mm = self._measure_temporal_depth(bundle, det.bbox)
            except Exception as exc:
                raise TaskError(
                    f"{task_name} {det.class_name} 深度无效，拒绝机械臂运动：{exc}"
                ) from exc
            if not math.isfinite(depth_mm):
                raise TaskError("深度为 NaN/Inf，拒绝机械臂运动")

            try:
                camera_xyz, robot_xyz = self.calibration.locate(
                    det.pixel_center, depth_mm, bundle.intrinsics
                )
            except Exception as exc:
                raise TaskError(f"坐标变换失败，拒绝运动：{exc}") from exc

            det.depth_mm = depth_mm
            det.camera_xyz_mm = tuple(float(v) for v in camera_xyz)
            det.robot_xyz_mm = tuple(float(v) for v in robot_xyz)
            if det.oriented_bbox is None:
                raise TaskError(
                    f"{task_name} {det.class_name} 缺少 OBB 四点框，拒绝猜测抓取 RZ"
                )
            try:
                axis_center, axis_endpoint, _ = oriented_box_axis(det.oriented_bbox)
                # 圆柱体绕 Z 旋转后轮廓不变，抓取保持 0°，放置仍使用配置姿态。
                pick_rz = (
                    0.0
                    if det.shape == "cylinder"
                    else self.calibration.image_axis_to_robot_rz_deg(
                        axis_center,
                        axis_endpoint,
                        depth_mm,
                        bundle.intrinsics,
                    )
                )
            except Exception as exc:
                raise TaskError(f"工件角度换算失败，拒绝运动：{exc}") from exc
            if not math.isfinite(pick_rz) or not -90.0 <= pick_rz < 90.0:
                raise TaskError(f"抓取 RZ={pick_rz!r} 不在最短旋转范围 [-90,90)")
            det.pick_rz_deg = float(pick_rz)
            det.extra["pick_rz_deg"] = float(pick_rz)
            route_key, place_pose = self._resolve_place(det)
            object_height_mm = self.calibration.object_height_mm(depth_mm)
            det.extra["object_height_mm"] = object_height_mm
            det.route_key = route_key
            det.status = "等待机械臂"
            self._emit_detection_row(det)

            self.log.info(
                "%s 抓取目标 %s：像素=%s 深度=%.2fmm 机器人=(%.2f,%.2f,%.2f) "
                "最短RZ=%+.2f° → %s",
                task_name,
                det.class_name,
                det.pixel_center,
                depth_mm,
                *robot_xyz,
                pick_rz,
                route_key,
            )

            self._check_continue()
            target, action_reply, target_result = self._execute_direct_place(
                det,
                robot_xyz,
                place_pose,
                route_key,
                object_height_mm,
            )

            completed += 1
            det.status = "抓放完成并回拍照位"
            self._emit_detection_row(det)
            self.results.append(
                "pick_complete",
                {
                    "task": task_name,
                    "index": completed,
                    "detection": det.to_dict(),
                    "target": target_result,
                    "robot_reply": action_reply.raw,
                },
            )
            if self.simulation_world is not None:
                self.simulation_world.remove(det.object_id)
            self.camera.flush()

        if completed == 0:
            raise TaskError(f"{task_name} 未检测到可分拣工件")
        if completed == maximum:
            self.log.info("%s 已达到配置的分拣上限 %d 件", task_name, maximum)
        self.log.info("%s 完成，共 %d 件（配置上限 %d 件）", task_name, completed, maximum)

    def _execute_direct_place(
        self,
        det: Detection,
        robot_xyz: tuple[float, float, float],
        place_pose: Sequence[float | None],
        route_key: str,
        object_height_mm: float,
    ) -> tuple[DirectPlaceTarget, Any, dict[str, Any]]:
        """吸取 P1 后由 DobotStudio Pro MovJ 直接移动到同 Z 的 P2。"""

        if len(place_pose) != 6:
            raise TaskError(f"{route_key} 放置位姿必须为 [X,Y,Z,Rx,Ry,Rz]")
        place_xy = (float(place_pose[0]), float(place_pose[1]))
        place_orientation = (
            float(place_pose[3]),
            float(place_pose[4]),
            float(place_pose[5]),
        )
        target = DirectPlaceTarget(
            task="task3",
            object_id=det.object_id,
            pick_x_mm=float(robot_xyz[0]),
            pick_y_mm=float(robot_xyz[1]),
            pick_z_mm=float(robot_xyz[2]),
            pick_rz_deg=float(det.pick_rz_deg if det.pick_rz_deg is not None else 0.0),
            place_x_mm=place_xy[0],
            place_y_mm=place_xy[1],
            place_rx_deg=place_orientation[0],
            place_ry_deg=place_orientation[1],
            place_rz_deg=place_orientation[2],
            route_key=route_key,
        )

        place_z = target.pick_z_mm

        det.extra.update(
            {
                "place_release_z_mm": place_z,
                "configured_place_pose_mm_deg": list(place_pose),
                "place_pose_mm_deg": [
                    place_xy[0],
                    place_xy[1],
                    place_z,
                    place_orientation[0],
                    place_orientation[1],
                    place_orientation[2],
                ],
            }
        )
        self.log.info(
            "3D识别抓取直接路径：P1=(%.2f,%.2f,%.2f,%+.1f°) → "
            "P2=(%.2f,%.2f,%.2f,%+.1f,%+.1f,%+.1f°)，P1.Z=P2.Z",
            target.pick_x_mm,
            target.pick_y_mm,
            target.pick_z_mm,
            target.pick_rz_deg,
            target.place_x_mm,
            target.place_y_mm,
            place_z,
            *place_orientation,
        )
        det.status = "执行 P1→P2 同Z直接抓放"
        self._emit_detection_row(det)
        self._check_continue()
        action_reply = self.robot.pick_and_place_direct(target)
        if action_reply.status not in ("done", "home", "ok"):
            code = str(action_reply.raw.get("code", "")).strip()
            phase = str(action_reply.raw.get("phase", "")).strip()
            details = [value for value in (phase, code) if value]
            marker = f"[{' / '.join(details)}] " if details else ""
            raise TaskError(
                f"3D识别抓取直接抓放失败（{action_reply.status}）："
                f"{marker}{action_reply.message}"
            )

        return target, action_reply, {
            "pick": [
                target.pick_x_mm,
                target.pick_y_mm,
                target.pick_z_mm,
                target.pick_rz_deg,
            ],
            "object_height_mm": object_height_mm,
            "configured_place_pose": list(place_pose),
            "place_pose": [
                target.place_x_mm,
                target.place_y_mm,
                place_z,
                target.place_rx_deg,
                target.place_ry_deg,
                target.place_rz_deg,
            ],
            "place_z_mm": place_z,
            "route": route_key,
        }

    def _acquire_stable_candidate(self, task_name: str) -> tuple[Detection, Any] | None:
        detector = self.detectors[task_name]
        timeout_s = float(self.settings.get(f"tasks.{task_name}.detect_timeout_s", 20.0))
        empty_need = int(self.settings.get(f"tasks.{task_name}.empty_confirm_frames", 8))
        stable_need = int(self.settings.get(f"tasks.{task_name}.stable_frames", 3))
        tolerance = float(
            self.settings.get(f"tasks.{task_name}.stable_center_tolerance_px", 12)
        )
        angle_tolerance = float(
            self.settings.get(f"tasks.{task_name}.stable_angle_tolerance_deg", 5.0)
        )
        deadline = min(
            time.monotonic() + timeout_s,
            self._deadline if self._deadline else float("inf"),
        )
        empty_count = 0
        stable_count = 0
        previous: Detection | None = None

        while time.monotonic() < deadline:
            self._check_continue()
            bundle = self.camera.get_frame()
            raw_detections = detector.detect(bundle.color_bgr)
            image_height, image_width = bundle.color_bgr.shape[:2]
            detections = self.recognition_regions.filter(
                task_name,
                raw_detections,
                image_width,
                image_height,
            )
            annotated = detector.annotate(bundle.color_bgr, detections)
            self.bus.emit("frame", annotated)
            self.bus.emit("detections", [d.to_dict() for d in detections])

            if not detections:
                empty_count += 1
                stable_count = 0
                previous = None
                if empty_count >= empty_need:
                    return None
                continue

            empty_count = 0
            selected = self._select_candidate(task_name, detections, bundle)
            if previous is not None and self._same_target(
                previous, selected, tolerance, angle_tolerance
            ):
                stable_count += 1
            else:
                previous = selected
                stable_count = 1
            if stable_count >= stable_need:
                return selected, bundle

        raise TaskError(f"{task_name} 在 {timeout_s:.1f}s 内未获得稳定检测")

    def _measure_temporal_depth(
        self, first_bundle: Any, bbox: tuple[int, int, int, int]
    ) -> float:
        """在机械臂静止的拍照位连续取样，拒绝波动过大的深度。"""
        count = max(1, int(self.settings.get("camera.temporal_depth_samples", 5)))
        limit = float(self.settings.get("camera.max_temporal_depth_spread_mm", 8.0))
        values: list[float] = []
        bundle = first_bundle
        for index in range(count):
            self._check_continue()
            if index > 0:
                bundle = self.camera.get_frame()
            value = float(self.camera.measure_depth_mm(bundle, bbox))
            if math.isfinite(value):
                values.append(value)
        if len(values) < max(1, (count + 1) // 2):
            raise TaskError(f"多帧有效深度不足：{len(values)}/{count}")
        spread = max(values) - min(values)
        if spread > limit:
            raise TaskError(
                f"目标深度多帧波动 {spread:.2f} mm，超过允许 {limit:.2f} mm"
            )
        return float(statistics.median(values))

    def _select_candidate(
        self, task_name: str, detections: list[Detection], bundle: Any
    ) -> Detection:
        order = str(self.settings.get(f"tasks.{task_name}.candidate_order", "left_to_right"))
        if order == "confidence":
            return max(detections, key=lambda d: d.confidence)
        if order == "nearest_center":
            cx = float(bundle.intrinsics.width) / 2.0
            cy = float(bundle.intrinsics.height) / 2.0
            return min(
                detections,
                key=lambda d: (d.pixel_center[0] - cx) ** 2
                + (d.pixel_center[1] - cy) ** 2,
            )
        return min(detections, key=lambda d: (d.pixel_center[0], d.pixel_center[1]))

    @staticmethod
    def _same_target(
        a: Detection, b: Detection, tolerance: float, angle_tolerance: float = 5.0
    ) -> bool:
        if a.class_name != b.class_name:
            return False
        dx = float(a.pixel_center[0] - b.pixel_center[0])
        dy = float(a.pixel_center[1] - b.pixel_center[1])
        if math.hypot(dx, dy) > tolerance:
            return False
        if a.shape == "cylinder" and b.shape == "cylinder":
            return True
        if a.image_angle_deg is None or b.image_angle_deg is None:
            return False
        angle_delta = abs(
            (float(a.image_angle_deg) - float(b.image_angle_deg) + 90.0) % 180.0
            - 90.0
        )
        return angle_delta <= angle_tolerance

    def _resolve_place(self, det: Detection) -> tuple[str, tuple[float | None, ...]]:
        key = (
            "simulation.place_poses_mm_deg"
            if self.simulation_world is not None
            else "robot.place_poses_mm_deg"
        )
        poses = self.settings.get(key)
        if not isinstance(poses, Mapping):
            raise TaskError(f"缺少放置位姿配置：{key}")
        matched_name = next(
            (
                str(name)
                for name in poses
                if str(name).strip().casefold() == det.class_name.strip().casefold()
            ),
            None,
        )
        if matched_name is None:
            raise TaskError(
                f"类别 {det.class_name!r} 没有放置位姿；请在 settings.yaml 的 {key} 中填写"
            )
        pose = poses[matched_name]
        if not isinstance(pose, Sequence) or isinstance(pose, (str, bytes)) or len(pose) != 6:
            raise TaskError(f"{key}.{matched_name} 必须为 [X,Y,Z,Rx,Ry,Rz]")
        if pose[2] is not None:
            raise TaskError(f"{key}.{matched_name}[2] 必须为 null，Z 只能由视觉识别")
        required = (pose[0], pose[1], pose[3], pose[4], pose[5])
        if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            for value in required
        ):
            raise TaskError(f"{key}.{matched_name} 的 X/Y/Rx/Ry/Rz 尚未填写有效数值")
        return matched_name, tuple(
            None if index == 2 else float(value)
            for index, value in enumerate(pose)
        )

    def _emit_detection_row(self, det: Detection) -> None:
        xyz = det.robot_xyz_mm
        xyz_text = "-" if xyz is None else "{:.1f}, {:.1f}, {:.1f}".format(*xyz)
        self.bus.emit(
            "result_row",
            {
                "任务": "3D识别抓取",
                "类别": det.class_name,
                "颜色": det.color,
                "置信度": f"{det.confidence:.3f}",
                "像素": f"{det.pixel_center[0]}, {det.pixel_center[1]}",
                "深度(mm)": "-" if det.depth_mm is None else f"{det.depth_mm:.2f}",
                "高度(mm)": (
                    "-"
                    if not isinstance(det.extra.get("object_height_mm"), (int, float))
                    else f"{float(det.extra['object_height_mm']):.2f}"
                ),
                "机器人XYZ(mm)": xyz_text,
                "角度/RZ(°)": (
                    "-"
                    if det.pick_rz_deg is None
                    else f"{det.image_angle_deg:+.1f} / {det.pick_rz_deg:+.1f}"
                ),
                "状态": det.status,
            },
        )
