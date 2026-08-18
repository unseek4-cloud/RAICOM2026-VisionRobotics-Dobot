# -*- coding: utf-8 -*-
"""组装真机/模拟组件并管理生命周期。"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from .config import Settings
from .events import EventBus
from .logging_utils import configure_logging
from .recognition_region import RecognitionRegionStore
from .task.orchestrator import TaskOrchestrator


class RuntimeErrorWithContext(RuntimeError):
    pass


class SystemRuntime:
    def __init__(
        self,
        settings: Settings,
        real_mode: bool = False,
        *,
        enable_live_preview: bool = False,
    ):
        self.settings = settings
        self.real_mode = real_mode
        self.bus = EventBus()
        self.recognition_regions = RecognitionRegionStore(settings)
        self.logger = configure_logging(
            settings.project_root,
            self.bus,
            str(settings.get("application.log_level", "INFO")),
        )
        self.log = self.logger.getChild("runtime")
        self.bus.subscribe("dvs_connection", self._on_dvs_connection)
        self.bus.subscribe("robot_connection", self._on_robot_connection)
        self.bus.subscribe("robot_status", self._on_robot_status)
        self.bus.subscribe("task_state", self._on_task_state)
        self.orchestrator: TaskOrchestrator | None = None
        self.camera: Any = None
        self.dvs: Any = None
        self.robot: Any = None
        self.detectors: dict[str, Any] = {}
        self.calibration: Any = None
        self.simulation_world: Any = None
        self._started = False
        self._lock = threading.RLock()
        self._live_preview_enabled = bool(enable_live_preview)
        self._live_preview_task = "task3"
        self._live_preview_state_lock = threading.RLock()
        self._live_preview_stop = threading.Event()
        self._live_preview_thread: threading.Thread | None = None

    @property
    def is_started(self) -> bool:
        return self._started

    def _status(self, key: str, text: str, ok: bool = True) -> None:
        self.bus.emit("component_status", key, text, ok)

    def _on_dvs_connection(self, connected: bool) -> None:
        text = "已连接" if connected else ("等待DVS连接" if self._started else "未连接")
        self._status("dvs", text, bool(connected) or self._started)

    def _on_robot_connection(self, connected: bool) -> None:
        text = "Lua已连接" if connected else ("等待Lua连接" if self._started else "未连接")
        self._status("robot", text, bool(connected) or self._started)

    def _on_robot_status(self, message: Any) -> None:
        """把 Lua 协议状态转成现场界面可直接理解的中文。

        TCP 套接字连上只代表网络可达；只有 ``HELLO/ready`` 才代表
        DobotStudio Pro 中的脚本现场参数校验通过。
        """
        if not isinstance(message, dict):
            return
        status = str(message.get("status", ""))
        command_id = str(message.get("id", ""))
        phase = str(message.get("phase", ""))
        phase_names = {
            "idle": "待机",
            "at_photo": "已在拍照位",
            "away_from_photo": "不在拍照位",
            "move_to_p1": "DobotStudio Pro规划到抓取点P1",
            "vacuum_on": "开启吸盘",
            "move_p1_to_p2_same_z": "DobotStudio Pro同Z规划P1→P2",
            "vacuum_off": "释放吸盘",
            "return_photo": "返回固定拍照位",
        }
        if command_id == "HELLO":
            if status == "ready":
                self._status("robot", "Lua已就绪", True)
            elif status == "config_error":
                detail = str(message.get("message", message.get("code", "未知错误")))
                self._status("robot", f"Lua配置错误：{detail}", False)
                self.bus.emit("alarm", f"DobotStudio Pro Lua 配置未通过：{detail}")
            return
        if status == "running":
            self._status("robot", f"执行中：{phase_names.get(phase, phase or '未知阶段')}", True)
        elif status == "done":
            self._status("robot", phase_names.get(phase, "命令完成"), True)
        elif status in {"error", "rejected", "busy", "stopped"}:
            code = str(message.get("code", status))
            self._status("robot", f"执行错误：{code}", False)

    def _on_task_state(self, state: str, _detail: str = "") -> None:
        if state == "3D识别抓取":
            self.set_live_preview_task("task3")

    def set_live_preview_task(self, task: str) -> None:
        """设置 UI 实时 YOLO 图使用 3D 识别抓取模型。"""

        if task != "task3":
            raise ValueError("实时预览只支持 3D识别抓取（task3）")
        with self._live_preview_state_lock:
            self._live_preview_task = task

    def return_to_photo(self) -> bool:
        """让真机自动回拍照位，并用实机当前位姿复核到位状态。"""

        if not self._started or self.robot is None:
            raise RuntimeErrorWithContext("系统尚未初始化")
        if self.real_mode and not bool(getattr(self.robot, "is_connected", False)):
            raise RuntimeErrorWithContext("DobotStudio Pro Lua 尚未连接")

        self.bus.emit("task_state", "返回拍照位", "正在调用 DobotStudio Pro 运动规划")
        reply = self.robot.go_photo()
        if reply.status not in ("done", "home", "ok"):
            raise RuntimeErrorWithContext(reply.message or f"回拍照位失败：{reply.status}")

        check = self.robot.is_at_photo()
        if check.status not in ("done", "ok"):
            raise RuntimeErrorWithContext(check.message or f"拍照位复核失败：{check.status}")
        if check.raw.get("at_photo") is not True:
            pose = check.raw.get("current_pose", "未知")
            raise RuntimeErrorWithContext(f"运动完成但实机位姿仍不在拍照位：{pose}")

        self.bus.emit("task_state", "待机", "机械臂已自动回到拍照位")
        return True

    def start(self) -> bool:
        with self._lock:
            if self._started:
                return True
            self.bus.emit("task_state", "初始化", "加载配置、通信、视觉与标定组件")
            self.log.info("开始初始化，运行模式：%s", "真机" if self.real_mode else "模拟")
            try:
                self._build_components()

                self.dvs.start()
                self._status(
                    "dvs",
                    "监听中" if self.real_mode else "模拟结果就绪",
                    True,
                )

                self.robot.start()
                self._status(
                    "robot",
                    "监听中，等待Lua" if self.real_mode and not self.robot.is_connected else "已连接",
                    True,
                )

                self.camera.start()
                self._status("camera", "采集正常" if self.real_mode else "模拟画面正常", True)
                latest = self.camera.flush(count=2 if not self.real_mode else None)
                # 初始化完成后先显示一帧原始画面，避免监控面板在任务开始前一直
                # 空白；任务运行期间则由检测器持续发送带标注画面。
                if getattr(latest, "color_bgr", None) is not None:
                    self.bus.emit("frame", latest.color_bgr)

                self.orchestrator = TaskOrchestrator(
                    settings=self.settings,
                    bus=self.bus,
                    logger=self.logger,
                    camera=self.camera,
                    calibration=self.calibration,
                    detectors=self.detectors,
                    recognition_regions=self.recognition_regions,
                    dvs=self.dvs,
                    robot=self.robot,
                    simulation_world=self.simulation_world,
                )
                self._started = True
                self._start_live_preview()
                self._status("runtime", "已初始化", True)
                self.bus.emit("task_state", "待机", "系统已就绪；真机运行前请确认实体急停")
                self.log.info("系统初始化完成")
                return True
            except Exception as exc:
                self.log.exception("系统初始化失败：%s", exc)
                self.bus.emit("alarm", f"系统初始化失败：{exc}")
                self._status("runtime", "初始化失败", False)
                self._stop_components()
                raise RuntimeErrorWithContext(str(exc)) from exc

    def _build_components(self) -> None:
        from .vision.calibration import CalibrationModel
        from .vision.dvs_tcp import DVSReceiver, MockDVSReceiver
        from .vision.realsense_camera import RealSenseCamera
        from .vision.yolo_detector import YoloDetector
        from .robot.lua_bridge import LuaBridgeServer
        from .robot.simulation import MockRobot

        self.calibration = CalibrationModel.from_settings(
            self.settings, simulation=not self.real_mode
        )

        if self.real_mode:
            self.camera = RealSenseCamera(self.settings, self.log.getChild("camera"))
            self.dvs = DVSReceiver(self.settings, self.bus, self.log.getChild("dvs"))
            self.robot = LuaBridgeServer(self.settings, self.bus, self.log.getChild("robot"))
            self.detectors = {
                "task3": YoloDetector(
                    self.settings, "task3", self.log.getChild("yolo.task3")
                ),
            }
        else:
            from .vision.simulation import MockCamera, MockDetector, SimulationWorld

            self.simulation_world = SimulationWorld(self.settings)
            self.camera = MockCamera(
                self.settings, self.simulation_world, self.log.getChild("camera.mock")
            )
            self.dvs = MockDVSReceiver(self.settings, self.bus, self.log.getChild("dvs.mock"))
            self.robot = MockRobot(
                self.settings,
                self.bus,
                self.log.getChild("robot.mock"),
                self.simulation_world,
            )
            self.detectors = {
                "task3": MockDetector(
                    self.settings,
                    "task3",
                    self.simulation_world,
                    self.log.getChild("yolo.mock.task3"),
                ),
            }

        self._status("task3_model", "已加载" if self.real_mode else "模拟模型", True)

    def _start_live_preview(self) -> None:
        if not self._live_preview_enabled:
            return
        if self._live_preview_thread is not None and self._live_preview_thread.is_alive():
            return
        self._live_preview_stop.clear()
        self._live_preview_thread = threading.Thread(
            target=self._live_preview_loop,
            name="ui-live-vision",
            daemon=True,
        )
        self._live_preview_thread.start()

    def _live_preview_loop(self) -> None:
        from .vision.live_display import build_live_vision_frame

        preview_fps = max(
            1.0,
            min(30.0, float(self.settings.get("application.preview_fps", 12.0))),
        )
        interval_s = 1.0 / preview_fps
        depth_min = float(self.settings.get("camera.depth_min_mm", 300.0))
        depth_max = float(self.settings.get("camera.depth_max_mm", 1500.0))
        last_warning = 0.0

        while not self._live_preview_stop.is_set():
            started_at = time.monotonic()
            try:
                with self._live_preview_state_lock:
                    task = self._live_preview_task
                camera = self.camera
                detector = self.detectors.get(task)
                calibration = self.calibration
                if camera is None or detector is None or calibration is None:
                    break
                bundle = camera.get_frame(timeout_ms=1000)
                live_frame = build_live_vision_frame(
                    bundle,
                    task=task,
                    detector=detector,
                    camera=camera,
                    calibration=calibration,
                    recognition_regions=self.recognition_regions,
                    depth_min_mm=depth_min,
                    depth_max_mm=depth_max,
                )
                self.bus.emit("vision_frame", live_frame)
            except Exception as exc:
                if self._live_preview_stop.is_set():
                    break
                now = time.monotonic()
                if now - last_warning >= 3.0:
                    self.log.warning("实时三路视觉刷新失败：%s", exc)
                    last_warning = now
            elapsed = time.monotonic() - started_at
            # 即使 CPU 推理慢于目标帧率，也主动给任务状态机留出取帧/推理窗口。
            self._live_preview_stop.wait(max(0.01, interval_s - elapsed))

    def _stop_live_preview(self) -> None:
        self._live_preview_stop.set()
        thread, self._live_preview_thread = self._live_preview_thread, None
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=10.0)
            if thread.is_alive():
                self.log.warning("实时视觉线程未在 10 秒内退出")

    def stop(self) -> None:
        with self._lock:
            if self.orchestrator is not None and self.orchestrator.is_running:
                self.orchestrator.request_stop()
            self._stop_components()
            self.orchestrator = None
            self._started = False
            self._status("runtime", "已停止", False)

    def _stop_components(self) -> None:
        self._stop_live_preview()
        for component, key in (
            (self.camera, "camera"),
            (self.dvs, "dvs"),
            (self.robot, "robot"),
        ):
            if component is None:
                continue
            try:
                component.stop()
            except Exception as exc:
                self.log.warning("停止组件 %s 时出现异常：%s", key, exc)
        self.camera = None
        self.dvs = None
        self.robot = None
        self.detectors = {}
        self.calibration = None
        self.simulation_world = None


def run_headless(settings: Settings, real_mode: bool, task: str) -> int:
    runtime = SystemRuntime(settings, real_mode=real_mode)
    try:
        runtime.start()
        if real_mode:
            wait_s = 60.0
            runtime.log.info("等待 DobotStudio Pro Lua 连接（最多 %.0fs）", wait_s)
            if not runtime.robot.wait_connected(wait_s):
                raise RuntimeErrorWithContext("DobotStudio Pro Lua 在 60 秒内未连接")
        assert runtime.orchestrator is not None
        return 0 if runtime.orchestrator.run(task) else 1
    except KeyboardInterrupt:
        if runtime.orchestrator is not None:
            runtime.orchestrator.request_stop()
        return 130
    finally:
        runtime.stop()
