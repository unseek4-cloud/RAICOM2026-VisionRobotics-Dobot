# -*- coding: utf-8 -*-
"""任务一→任务二→任务三的唯一顺序控制状态机。

设计重点：
- 所有工件同时在桌面，但每个任务只使用自己的模型与类别过滤规则；
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
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from ..config import Settings, SettingsError
from ..events import EventBus
from ..interfaces import CalibrationLike, CameraLike, DVSLike, DetectorLike, RobotLike
from ..result_store import ResultStore
from ..types import Detection, PickTarget, StackPlaceTarget, TaskState


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
            sequence = ["task1", "task2", "task3"] if which == "all" else [which]
            for task_name in sequence:
                self._check_continue()
                if not bool(self.settings.get(f"tasks.{task_name}.enabled", True)):
                    self.log.info("%s 已在配置中禁用，跳过", task_name)
                    continue
                if task_name == "task1":
                    self.run_task1()
                else:
                    self.run_pick_task(task_name)

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
        # 丢弃启动任务前残留的旧结果。DVS 应在收到本次 TRIGGER 后重新输出；
        # 这样不会把上一次调试/断线重发的数据误计为本轮评分结果。
        clear_results = getattr(self.dvs, "clear_results", None)
        if callable(clear_results):
            clear_results()
        try:
            triggered = self.dvs.trigger()
        except Exception as exc:
            triggered = False
            self.log.warning("DVS 软触发发送失败，将继续等待主动输出：%s", exc)
        if triggered:
            self.log.info("已向 Dobot Vision Studio 发送软触发")

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

    def run_pick_task(self, task_name: str) -> None:
        state = TaskState.TASK2 if task_name == "task2" else TaskState.TASK3
        self._set_state(state, "回拍照位并逐件重新识别、抓取、分类")
        if task_name not in self.detectors:
            raise TaskError(f"{task_name} 检测模型未加载")
        if not self.robot.is_connected:
            raise TaskError("DobotStudio Pro 执行脚本尚未连接")

        reply = self.robot.go_photo()
        if reply.status not in ("done", "home", "ok"):
            code = str(reply.raw.get("code", "")).strip()
            reason = reply.message or reply.status
            if code and code not in reason:
                reason = f"{code}：{reason}"
            raise TaskError(f"机械臂未能到达固定拍照位：{reason}")
        self.camera.flush()

        expected = int(self.settings.get(f"tasks.{task_name}.expected_objects"))
        maximum = int(self.settings.get(f"tasks.{task_name}.max_objects", expected))
        if maximum < expected or maximum < 1:
            raise SettingsError(f"tasks.{task_name}.max_objects 必须 >= expected_objects >= 1")

        completed = 0
        # 达到 expected/max 后仍再做一次“当前任务无目标”确认。这样既满足
        # “抓取直到没有工件”，又能在模型意外检测出额外目标时安全失败，而不是
        # 悄悄把未知件留在桌面或无限抓取。
        while True:
            self._check_continue()
            candidate = self._acquire_stable_candidate(task_name)
            if candidate is None:
                self.log.info("%s 已连续多帧无目标", task_name)
                break
            if completed >= maximum:
                raise TaskError(
                    f"{task_name} 已达到最大允许 {maximum} 件，但仍检测到目标；"
                    "请核对现场数量、类别过滤和模型误检"
                )
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
            route_key, place = self._resolve_place(task_name, det)
            object_height_mm = self.calibration.object_height_mm(depth_mm)
            det.extra["object_height_mm"] = object_height_mm
            det.route_key = route_key
            det.status = "等待机械臂"
            self._emit_detection_row(det)

            self.log.info(
                "%s 抓取目标 %s：像素=%s 深度=%.2fmm 机器人=(%.2f,%.2f,%.2f) → %s",
                task_name,
                det.class_name,
                det.pixel_center,
                depth_mm,
                *robot_xyz,
                route_key,
            )

            self._check_continue()
            if task_name == "task3":
                target, action_reply, target_result = self._execute_task3_dynamic_place(
                    det,
                    robot_xyz,
                    place,
                    route_key,
                    object_height_mm,
                )
            else:
                target = PickTarget(
                    task=task_name,
                    object_id=det.object_id,
                    pick_x_mm=float(robot_xyz[0]),
                    pick_y_mm=float(robot_xyz[1]),
                    pick_z_mm=float(robot_xyz[2]),
                    place_x_mm=float(place["x_mm"]),
                    place_y_mm=float(place["y_mm"]),
                    place_down_mm=float(place["down_mm"]),
                    route_key=route_key,
                )
                action_reply = self.robot.pick_and_place(target)
                if action_reply.status not in ("done", "home", "ok"):
                    raise TaskError(
                        f"机器人抓放失败（{action_reply.status}）：{action_reply.message}"
                    )
                target_result = {
                    "pick": [target.pick_x_mm, target.pick_y_mm, target.pick_z_mm],
                    "place_xy": [target.place_x_mm, target.place_y_mm],
                    "place_down_mm": target.place_down_mm,
                    "route": route_key,
                }

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

        if completed != expected:
            raise TaskError(f"{task_name} 完成 {completed} 件，现场预期为 {expected} 件")
        self.log.info("%s 完成，共 %d 件", task_name, completed)

    def _execute_task3_dynamic_place(
        self,
        det: Detection,
        robot_xyz: tuple[float, float, float],
        place: Mapping[str, Any],
        route_key: str,
        object_height_mm: float,
    ) -> tuple[StackPlaceTarget, Any, dict[str, Any]]:
        """任务三先持件观察目标顶面，再按绝对 Z 叠放。"""

        inspection_z = float(self.settings.get("robot.motion.place_inspection_z_mm"))
        orientation_raw = self.settings.get("robot.motion.orientation_mm_deg")
        if not isinstance(orientation_raw, (list, tuple)) or len(orientation_raw) != 3:
            raise TaskError("robot.motion.orientation_mm_deg 必须包含 3 个姿态值")
        orientation = tuple(float(value) for value in orientation_raw)
        place_xy = (float(place["x_mm"]), float(place["y_mm"]))
        try:
            inspection_pose = self.calibration.placement_inspection_pose(
                place_xy, inspection_z, orientation
            )
        except Exception as exc:
            raise TaskError(f"任务三放置观察位计算失败，拒绝运动：{exc}") from exc

        target = StackPlaceTarget(
            task="task3",
            object_id=det.object_id,
            pick_x_mm=float(robot_xyz[0]),
            pick_y_mm=float(robot_xyz[1]),
            pick_z_mm=float(robot_xyz[2]),
            object_height_mm=float(object_height_mm),
            place_x_mm=place_xy[0],
            place_y_mm=place_xy[1],
            inspection_x_mm=float(inspection_pose[0]),
            inspection_y_mm=float(inspection_pose[1]),
            inspection_z_mm=float(inspection_pose[2]),
            route_key=route_key,
        )

        det.status = "已抓取，前往放置点上方识别当前顶面"
        self._emit_detection_row(det)
        inspect_reply = self.robot.pick_to_inspection(target)
        if inspect_reply.status not in ("done", "ok"):
            raise TaskError(
                f"任务三未能持件到达放置观察位（{inspect_reply.status}）："
                f"{inspect_reply.message}"
            )

        # 此后若视觉失败，Lua 会保持真空和观察位，不擅自猜高度或释放工件。
        self.camera.flush()
        det.status = "吸盘保持中，正在识别放置点顶面高度"
        self._emit_detection_row(det)
        try:
            surface_xyz, surface_depth_mm, valid_points = self._measure_place_surface(
                target, inspection_pose
            )
        except Exception as exc:
            raise TaskError(
                "任务三放置顶面高度识别失败；机械臂仍在观察位保持吸盘，"
                f"禁止自动释放：{exc}"
            ) from exc

        z_up_sign = int(self.settings.get("robot.motion.z_up_sign", 1))
        press_down = float(
            self.settings.get("tasks.task3.placement_vision.press_down_mm", 0.0)
        )
        if (
            not math.isfinite(press_down)
            or press_down < 0
            or press_down >= object_height_mm
        ):
            raise TaskError(
                f"任务三放置下压补偿 {press_down!r} 必须不小于 0 且小于"
                f"当前工件高度 {object_height_mm:.2f} mm；机械臂仍保持吸盘"
            )
        place_z = float(surface_xyz[2]) + z_up_sign * (
            float(object_height_mm) - press_down
        )
        try:
            self.calibration.validate_workspace((place_xy[0], place_xy[1], place_z))
        except Exception as exc:
            raise TaskError(
                "任务三视觉计算的释放 Z 超出工作空间；机械臂仍保持吸盘："
                f"{exc}"
            ) from exc

        descent = z_up_sign * (target.inspection_z_mm - place_z)
        minimum_descent = float(
            self.settings.get(
                "tasks.task3.placement_vision.min_descent_clearance_mm", 20.0
            )
        )
        if not math.isfinite(descent) or descent < minimum_descent:
            raise TaskError(
                f"任务三观察位到释放位仅有 {descent:.2f} mm 安全间距，"
                f"小于 {minimum_descent:.2f} mm；机械臂仍保持吸盘"
            )

        det.extra.update(
            {
                "place_surface_xyz_mm": surface_xyz,
                "place_surface_depth_mm": surface_depth_mm,
                "place_surface_valid_points": valid_points,
                "place_release_z_mm": place_z,
            }
        )
        self.log.info(
            "task3 放置顶面：机器人=(%.2f,%.2f,%.2f)，深度=%.2fmm，"
            "工件高=%.2fmm，释放Z=%.2fmm，有效点=%d",
            *surface_xyz,
            surface_depth_mm,
            object_height_mm,
            place_z,
            valid_points,
        )
        det.status = "顶面高度已识别，按视觉 Z 下放"
        self._emit_detection_row(det)
        self._check_continue()
        action_reply = self.robot.place_from_inspection(
            target, inspect_reply.command_id, place_z
        )
        if action_reply.status not in ("done", "home", "ok"):
            raise TaskError(
                f"任务三动态放置失败（{action_reply.status}）：{action_reply.message}"
            )

        return target, action_reply, {
            "pick": [target.pick_x_mm, target.pick_y_mm, target.pick_z_mm],
            "object_height_mm": object_height_mm,
            "place_xy": [target.place_x_mm, target.place_y_mm],
            "inspection_pose": list(inspection_pose),
            "surface_xyz": list(surface_xyz),
            "surface_depth_mm": surface_depth_mm,
            "surface_valid_points": valid_points,
            "place_z_mm": place_z,
            "route": route_key,
            "hold_id": inspect_reply.command_id,
        }

    def _measure_place_surface(
        self,
        target: StackPlaceTarget,
        inspection_pose: tuple[float, float, float, float, float, float],
    ) -> tuple[tuple[float, float, float], float, int]:
        """在放置观察位进行多帧三维顶面识别和波动检查。"""

        prefix = "tasks.task3.placement_vision"
        count = max(1, int(self.settings.get(f"{prefix}.temporal_samples", 5)))
        spread_limit = float(self.settings.get(f"{prefix}.max_surface_spread_mm", 4.0))
        radius = float(self.settings.get(f"{prefix}.sample_radius_mm", 8.0))
        min_points = int(self.settings.get(f"{prefix}.min_valid_points", 20))
        # 观察高度降低后，已叠放工件的顶面会比抓取区目标更靠近相机；
        # 放置测高使用独立下限，不能误改抓取区的深度安全范围。
        depth_min = float(
            self.settings.get(
                f"{prefix}.depth_min_mm",
                self.settings.get("camera.depth_min_mm", 300.0),
            )
        )
        depth_max = float(self.settings.get("camera.depth_max_mm", 1500.0))
        surfaces: list[tuple[float, float, float]] = []
        depths: list[float] = []
        point_counts: list[int] = []
        errors: list[str] = []

        for _ in range(count):
            self._check_continue()
            bundle = self.camera.get_frame()
            self.bus.emit("frame", bundle.color_bgr)
            try:
                surface, depth_mm, valid_points = (
                    self.calibration.locate_surface_at_robot_xy(
                        bundle,
                        (target.place_x_mm, target.place_y_mm),
                        inspection_pose,
                        depth_min_mm=depth_min,
                        depth_max_mm=depth_max,
                        radius_mm=radius,
                        min_points=min_points,
                    )
                )
            except Exception as exc:
                errors.append(str(exc))
                continue
            surfaces.append(surface)
            depths.append(depth_mm)
            point_counts.append(valid_points)

        required = max(1, (count + 1) // 2)
        if len(surfaces) < required:
            detail = errors[-1] if errors else "无有效三维结果"
            raise TaskError(
                f"放置顶面多帧有效结果不足：{len(surfaces)}/{count}；{detail}"
            )
        z_values = [surface[2] for surface in surfaces]
        spread = max(z_values) - min(z_values)
        if spread > spread_limit:
            raise TaskError(
                f"放置顶面多帧 Z 波动 {spread:.2f} mm，超过允许 {spread_limit:.2f} mm"
            )
        median_surface = tuple(
            float(statistics.median(surface[axis] for surface in surfaces))
            for axis in range(3)
        )
        return (
            median_surface,
            float(statistics.median(depths)),
            int(statistics.median(point_counts)),
        )

    def _acquire_stable_candidate(self, task_name: str) -> tuple[Detection, Any] | None:
        detector = self.detectors[task_name]
        timeout_s = float(self.settings.get(f"tasks.{task_name}.detect_timeout_s", 20.0))
        empty_need = int(self.settings.get(f"tasks.{task_name}.empty_confirm_frames", 8))
        stable_need = int(self.settings.get(f"tasks.{task_name}.stable_frames", 3))
        tolerance = float(
            self.settings.get(f"tasks.{task_name}.stable_center_tolerance_px", 12)
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
            detections = detector.detect(bundle.color_bgr)
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
            if previous is not None and self._same_target(previous, selected, tolerance):
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
    def _same_target(a: Detection, b: Detection, tolerance: float) -> bool:
        if a.class_name != b.class_name:
            return False
        dx = float(a.pixel_center[0] - b.pixel_center[0])
        dy = float(a.pixel_center[1] - b.pixel_center[1])
        return math.hypot(dx, dy) <= tolerance

    def _resolve_place(self, task_name: str, det: Detection) -> tuple[str, Mapping[str, Any]]:
        prefix = (
            "simulation.place_points"
            if self.simulation_world is not None
            else "robot.place_points"
        )
        points = self.settings.get(f"{prefix}.{task_name}")
        if not isinstance(points, Mapping):
            raise TaskError(f"缺少 {task_name} 放置点配置")
        route_by = str(self.settings.get(f"tasks.{task_name}.route_by"))
        candidates: list[str] = []
        if route_by == "color":
            candidates.append(str(det.color).lower())
        elif route_by == "shape":
            candidates.append(str(det.shape).lower())
        elif route_by == "class":
            candidates.append(det.class_name)
        elif route_by == "match_or_class":
            known = self.settings.get("yolo.task3.known_label", None)
            if known:
                candidates.append(
                    "match" if det.class_name.casefold() == str(known).casefold() else "not_match"
                )
            candidates.append(det.class_name)
        candidates.append("default")

        for key in candidates:
            if key not in points:
                continue
            point = points[key]
            if not isinstance(point, Mapping):
                continue
            required_fields = (
                ("x_mm", "y_mm") if task_name == "task3" else ("x_mm", "y_mm", "down_mm")
            )
            if all(
                isinstance(point.get(field), (int, float))
                and not isinstance(point.get(field), bool)
                and math.isfinite(float(point[field]))
                for field in required_fields
            ):
                return key, point
        raise TaskError(
            f"{task_name} 的目标 {det.class_name}/{det.color}/{det.shape} 没有可用放置点；"
            "请在 settings.yaml 填写对应 route_key 或 default"
        )

    def _emit_detection_row(self, det: Detection) -> None:
        xyz = det.robot_xyz_mm
        xyz_text = "-" if xyz is None else "{:.1f}, {:.1f}, {:.1f}".format(*xyz)
        self.bus.emit(
            "result_row",
            {
                "任务": "任务二" if det.task == "task2" else "任务三",
                "类别": det.class_name,
                "颜色": det.color,
                "置信度": f"{det.confidence:.3f}",
                "像素": f"{det.pixel_center[0]}, {det.pixel_center[1]}",
                "深度(mm)": "-" if det.depth_mm is None else f"{det.depth_mm:.2f}",
                "机器人XYZ(mm)": xyz_text,
                "状态": det.status,
            },
        )
