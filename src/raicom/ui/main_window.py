# -*- coding: utf-8 -*-
"""比赛现场 PyQt5 中文监控面板。"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from ..config import Settings
from ..environment import collect_environment_report
from ..runtime import SystemRuntime


class UiSignals(QtCore.QObject):
    log = QtCore.pyqtSignal(str)
    frame = QtCore.pyqtSignal(object)
    task_state = QtCore.pyqtSignal(str, str)
    component_status = QtCore.pyqtSignal(str, str, bool)
    result_row = QtCore.pyqtSignal(dict)
    task1_result = QtCore.pyqtSignal(dict)
    alarm = QtCore.pyqtSignal(str)
    timer_started = QtCore.pyqtSignal(float)
    timer_stopped = QtCore.pyqtSignal()
    worker_done = QtCore.pyqtSignal(str, bool, str)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, settings: Settings, real_mode: bool):
        super().__init__()
        self.settings = settings
        self.real_mode = real_mode
        self.runtime = SystemRuntime(settings, real_mode=real_mode)
        self.signals = UiSignals()
        self._busy = False
        self._initialized = False
        self._timer_deadline: float | None = None
        self._completed_counts = {"任务二": 0, "任务三": 0}
        self._alarm_count = 0

        self.setWindowTitle(str(settings.get("application.name")))
        self.resize(1480, 900)
        self.setMinimumSize(1180, 760)
        self.setFont(QtGui.QFont("Microsoft YaHei UI", 10))
        self._build_ui()
        self._wire_events()
        self._apply_style()

        self.clock_timer = QtCore.QTimer(self)
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(200)

    def _build_ui(self) -> None:
        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)

        header = QtWidgets.QHBoxLayout()
        title_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel("睿抗 2026 · 视觉引导柔性分拣系统")
        title.setObjectName("title")
        subtitle = QtWidgets.QLabel("Dobot Vision Studio · YOLO · RealSense D435 · Dobot E6")
        subtitle.setObjectName("subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header.addLayout(title_box)
        header.addStretch(1)

        self.mode_badge = QtWidgets.QLabel("真机模式" if self.real_mode else "安全模拟模式")
        self.mode_badge.setObjectName("realBadge" if self.real_mode else "demoBadge")
        self.mode_badge.setAlignment(QtCore.Qt.AlignCenter)
        self.mode_badge.setMinimumWidth(125)
        self.timer_label = QtWidgets.QLabel("计时 10:00")
        self.timer_label.setObjectName("timer")
        self.timer_label.setMinimumWidth(130)
        self.timer_label.setAlignment(QtCore.Qt.AlignCenter)
        header.addWidget(self.mode_badge)
        header.addWidget(self.timer_label)
        root.addLayout(header)

        upper = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        upper.setChildrenCollapsible(False)
        root.addWidget(upper, 3)

        video_panel = QtWidgets.QFrame()
        video_panel.setObjectName("panel")
        video_layout = QtWidgets.QVBoxLayout(video_panel)
        video_header = QtWidgets.QHBoxLayout()
        video_title = QtWidgets.QLabel("实时视觉画面")
        video_title.setObjectName("sectionTitle")
        self.frame_info = QtWidgets.QLabel("等待相机初始化")
        self.frame_info.setObjectName("muted")
        video_header.addWidget(video_title)
        video_header.addStretch(1)
        video_header.addWidget(self.frame_info)
        video_layout.addLayout(video_header)
        self.video = QtWidgets.QLabel("尚未收到图像")
        self.video.setAlignment(QtCore.Qt.AlignCenter)
        self.video.setMinimumSize(700, 480)
        self.video.setObjectName("video")
        video_layout.addWidget(self.video, 1)
        upper.addWidget(video_panel)

        right = QtWidgets.QWidget()
        right_layout = QtWidgets.QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)

        status_box = QtWidgets.QGroupBox("系统状态")
        status_grid = QtWidgets.QGridLayout(status_box)
        self.status_labels: dict[str, QtWidgets.QLabel] = {}
        names = [
            ("runtime", "主程序"),
            ("dvs", "DVS TCP"),
            ("camera", "D435 相机"),
            ("task2_model", "任务二模型"),
            ("task3_model", "任务三模型"),
            ("robot", "DobotStudio脚本"),
        ]
        for row, (key, text) in enumerate(names):
            label = QtWidgets.QLabel(text)
            value = QtWidgets.QLabel("● 未初始化")
            value.setObjectName("statusOff")
            value.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.status_labels[key] = value
            status_grid.addWidget(label, row, 0)
            status_grid.addWidget(value, row, 1)
        right_layout.addWidget(status_box)

        state_box = QtWidgets.QGroupBox("当前任务")
        state_layout = QtWidgets.QVBoxLayout(state_box)
        self.state_label = QtWidgets.QLabel("待机")
        self.state_label.setObjectName("stateLabel")
        self.state_detail = QtWidgets.QLabel("请先执行系统自检与初始化")
        self.state_detail.setWordWrap(True)
        self.state_detail.setObjectName("muted")
        self.progress_label = QtWidgets.QLabel()
        self.progress_label.setObjectName("muted")
        state_layout.addWidget(self.state_label)
        state_layout.addWidget(self.state_detail)
        state_layout.addWidget(self.progress_label)
        self._refresh_progress()
        right_layout.addWidget(state_box)

        task1_box = QtWidgets.QGroupBox("任务一 · DVS测量/二维码/字符")
        task1_layout = QtWidgets.QVBoxLayout(task1_box)
        self.task1_text = QtWidgets.QPlainTextEdit()
        self.task1_text.setReadOnly(True)
        self.task1_text.setPlaceholderText("等待 Dobot Vision Studio TCP 结果……")
        self.task1_text.setMaximumHeight(125)
        task1_layout.addWidget(self.task1_text)
        right_layout.addWidget(task1_box)

        controls = QtWidgets.QGroupBox("运行控制")
        grid = QtWidgets.QGridLayout(controls)
        self.btn_check = QtWidgets.QPushButton("系统自检")
        self.btn_init = QtWidgets.QPushButton("初始化系统")
        self.btn_task1 = QtWidgets.QPushButton("运行任务一")
        self.btn_task2 = QtWidgets.QPushButton("运行任务二")
        self.btn_task3 = QtWidgets.QPushButton("运行任务三")
        self.btn_all = QtWidgets.QPushButton("全流程自动运行")
        self.btn_stop = QtWidgets.QPushButton("停止任务（非物理急停）")
        self.btn_all.setObjectName("primaryButton")
        self.btn_stop.setObjectName("stopButton")
        grid.addWidget(self.btn_check, 0, 0)
        grid.addWidget(self.btn_init, 0, 1)
        grid.addWidget(self.btn_task1, 1, 0)
        grid.addWidget(self.btn_task2, 1, 1)
        grid.addWidget(self.btn_task3, 2, 0)
        grid.addWidget(self.btn_all, 2, 1)
        grid.addWidget(self.btn_stop, 3, 0, 1, 2)
        right_layout.addWidget(controls)

        safety = QtWidgets.QLabel(
            "⚠ 软件停止不能替代机械臂实体急停。真机首次运行必须低速、空载、有人监护。"
        )
        safety.setObjectName("safety")
        safety.setWordWrap(True)
        right_layout.addWidget(safety)
        right_layout.addStretch(1)
        upper.addWidget(right)
        upper.setStretchFactor(0, 7)
        upper.setStretchFactor(1, 4)

        lower = QtWidgets.QTabWidget()
        root.addWidget(lower, 2)
        self.result_table = QtWidgets.QTableWidget(0, 8)
        self.result_table.setHorizontalHeaderLabels(
            ["任务", "类别", "颜色", "置信度", "像素", "深度(mm)", "机器人XYZ(mm)", "状态"]
        )
        self.result_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.result_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.result_table.setAlternatingRowColors(True)
        self.result_table.verticalHeader().setVisible(False)
        header_view = self.result_table.horizontalHeader()
        header_view.setSectionResizeMode(QtWidgets.QHeaderView.ResizeToContents)
        header_view.setSectionResizeMode(6, QtWidgets.QHeaderView.Stretch)
        header_view.setSectionResizeMode(7, QtWidgets.QHeaderView.Stretch)
        lower.addTab(self.result_table, "识别与抓放结果")

        self.log_text = QtWidgets.QPlainTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumBlockCount(3000)
        self.log_text.setFont(QtGui.QFont("Consolas", 9))
        lower.addTab(self.log_text, "运行日志")

        self._set_task_buttons(False)
        self.btn_stop.setEnabled(False)

    def _wire_events(self) -> None:
        bus = self.runtime.bus
        bus.subscribe("log", self.signals.log.emit)
        bus.subscribe("frame", self.signals.frame.emit)
        bus.subscribe("task_state", self.signals.task_state.emit)
        bus.subscribe("component_status", self.signals.component_status.emit)
        bus.subscribe("result_row", self.signals.result_row.emit)
        bus.subscribe("task1_result", self.signals.task1_result.emit)
        bus.subscribe("alarm", self.signals.alarm.emit)
        bus.subscribe("timer_started", self.signals.timer_started.emit)
        bus.subscribe("timer_stopped", self.signals.timer_stopped.emit)

        self.signals.log.connect(self._append_log)
        self.signals.frame.connect(self._show_frame)
        self.signals.task_state.connect(self._show_state)
        self.signals.component_status.connect(self._show_component)
        self.signals.result_row.connect(self._add_result_row)
        self.signals.task1_result.connect(self._show_task1_result)
        self.signals.alarm.connect(self._show_alarm)
        self.signals.timer_started.connect(self._start_timer)
        self.signals.timer_stopped.connect(self._stop_timer)
        self.signals.worker_done.connect(self._worker_finished)

        self.btn_check.clicked.connect(self._check_environment)
        self.btn_init.clicked.connect(self._initialize)
        self.btn_task1.clicked.connect(lambda: self._run_task("task1"))
        self.btn_task2.clicked.connect(lambda: self._run_task("task2"))
        self.btn_task3.clicked.connect(lambda: self._run_task("task3"))
        self.btn_all.clicked.connect(lambda: self._run_task("all"))
        self.btn_stop.clicked.connect(self._stop_task)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget { background: #f3f6fa; color: #1c2733; }
            QLabel#title { font-size: 25px; font-weight: 700; color: #102a43; }
            QLabel#subtitle, QLabel#muted { color: #6b7c8f; }
            QLabel#demoBadge { background:#d9f2e6; color:#176b45; border-radius:12px; padding:7px 12px; font-weight:700; }
            QLabel#realBadge { background:#ffe6d9; color:#a33b16; border-radius:12px; padding:7px 12px; font-weight:700; }
            QLabel#timer { background:#102a43; color:white; border-radius:8px; padding:7px 12px; font-size:17px; font-weight:700; }
            QFrame#panel, QGroupBox { background:white; border:1px solid #d9e2ec; border-radius:8px; }
            QGroupBox { margin-top:10px; padding-top:10px; font-weight:700; }
            QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; }
            QLabel#sectionTitle { font-size:16px; font-weight:700; }
            QLabel#video { background:#0c141d; color:#8fa3b7; border:1px solid #263746; border-radius:5px; }
            QLabel#stateLabel { font-size:22px; font-weight:700; color:#0b63ce; }
            QLabel#statusOn { color:#16784b; font-weight:700; }
            QLabel#statusOff { color:#8b99a8; }
            QLabel#statusBad { color:#c0362c; font-weight:700; }
            QLabel#safety { color:#9b2c2c; background:#fff4f2; border:1px solid #ffc9c2; border-radius:6px; padding:8px; }
            QPushButton { background:white; border:1px solid #bcccdc; border-radius:6px; padding:8px 10px; font-weight:600; }
            QPushButton:hover { background:#eaf2fb; border-color:#5b9bd5; }
            QPushButton:disabled { color:#9aa8b5; background:#eef2f5; }
            QPushButton#primaryButton { background:#087f5b; color:white; border-color:#087f5b; }
            QPushButton#primaryButton:hover { background:#066a4b; }
            QPushButton#stopButton { background:#c92a2a; color:white; border-color:#c92a2a; }
            QPushButton#stopButton:hover { background:#a61e1e; }
            QPlainTextEdit, QTableWidget { background:white; border:1px solid #d9e2ec; border-radius:5px; }
            QHeaderView::section { background:#e8eef5; color:#334e68; padding:6px; border:0; border-right:1px solid #d9e2ec; font-weight:700; }
            QTabWidget::pane { border:1px solid #d9e2ec; background:white; }
            QTabBar::tab { background:#e8eef5; padding:8px 16px; }
            QTabBar::tab:selected { background:white; color:#0b63ce; font-weight:700; }
            """
        )

    def _set_task_buttons(self, enabled: bool) -> None:
        for button in (self.btn_task1, self.btn_task2, self.btn_task3, self.btn_all):
            button.setEnabled(enabled and not self._busy)

    def _run_background(self, name: str, function: Callable[[], Any]) -> None:
        if self._busy:
            QtWidgets.QMessageBox.information(self, "正在运行", "请等待当前操作完成。")
            return
        self._busy = True
        self.btn_init.setEnabled(False)
        self.btn_check.setEnabled(False)
        self._set_task_buttons(False)
        self.btn_stop.setEnabled(name.startswith("task:"))

        def worker() -> None:
            ok = False
            message = ""
            try:
                result = function()
                ok = result is not False
            except Exception as exc:
                message = str(exc)
            self.signals.worker_done.emit(name, ok, message)

        threading.Thread(target=worker, name=f"ui-{name}", daemon=True).start()

    def _check_environment(self) -> None:
        ok, lines = collect_environment_report(self.settings, self.real_mode)
        dialog = QtWidgets.QMessageBox(self)
        dialog.setWindowTitle("系统自检")
        dialog.setIcon(QtWidgets.QMessageBox.Information if ok else QtWidgets.QMessageBox.Warning)
        dialog.setText("自检通过" if ok else "自检发现待处理项")
        dialog.setDetailedText("\n".join(lines))
        dialog.exec_()

    def _initialize(self) -> None:
        self._run_background("initialize", self.runtime.start)

    def _run_task(self, task: str) -> None:
        if not self._initialized:
            QtWidgets.QMessageBox.warning(self, "尚未初始化", "请先点击“初始化系统”。")
            return
        if task == "all":
            self._completed_counts = {"任务二": 0, "任务三": 0}
            self._alarm_count = 0
            self.result_table.setRowCount(0)
            self.task1_text.clear()
            self._refresh_progress()
        self._run_background(f"task:{task}", lambda: self.runtime.orchestrator.run(task))

    def _stop_task(self) -> None:
        if self.runtime.orchestrator is not None:
            self.runtime.orchestrator.request_stop()
        self.btn_stop.setEnabled(False)

    @QtCore.pyqtSlot(str)
    def _append_log(self, text: str) -> None:
        self.log_text.appendPlainText(text)
        bar = self.log_text.verticalScrollBar()
        bar.setValue(bar.maximum())

    @QtCore.pyqtSlot(object)
    def _show_frame(self, bgr: object) -> None:
        if not isinstance(bgr, np.ndarray) or bgr.ndim != 3:
            return
        if bgr.shape[2] == 3:
            rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            image = QtGui.QImage(
                rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0], QtGui.QImage.Format_RGB888
            ).copy()
        else:
            return
        pixmap = QtGui.QPixmap.fromImage(image).scaled(
            self.video.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
        )
        self.video.setPixmap(pixmap)
        self.frame_info.setText(f"{bgr.shape[1]}×{bgr.shape[0]} · {time.strftime('%H:%M:%S')}")

    @QtCore.pyqtSlot(str, str)
    def _show_state(self, state: str, detail: str) -> None:
        self.state_label.setText(state)
        self.state_detail.setText(detail or "-")

    @QtCore.pyqtSlot(str, str, bool)
    def _show_component(self, key: str, text: str, ok: bool) -> None:
        label = self.status_labels.get(key)
        if label is None:
            return
        label.setText(("● " if ok else "● ") + text)
        label.setObjectName("statusOn" if ok else "statusBad")
        label.style().unpolish(label)
        label.style().polish(label)

    @QtCore.pyqtSlot(dict)
    def _add_result_row(self, row: dict) -> None:
        labels = ["任务", "类别", "颜色", "置信度", "像素", "深度(mm)", "机器人XYZ(mm)", "状态"]
        index = self.result_table.rowCount()
        self.result_table.insertRow(index)
        for column, key in enumerate(labels):
            item = QtWidgets.QTableWidgetItem(str(row.get(key, "-")))
            if column in (0, 2, 3, 4, 5):
                item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.result_table.setItem(index, column, item)
        self.result_table.scrollToBottom()
        task_name = str(row.get("任务", ""))
        status = str(row.get("状态", ""))
        if task_name in self._completed_counts and "抓放完成" in status:
            self._completed_counts[task_name] += 1
            self._refresh_progress()

    def _refresh_progress(self) -> None:
        expected2 = int(self.settings.get("tasks.task2.expected_objects", 0))
        expected3 = int(self.settings.get("tasks.task3.expected_objects", 0))
        self.progress_label.setText(
            f"抓放进度：任务二 {self._completed_counts['任务二']}/{expected2}  ·  "
            f"任务三 {self._completed_counts['任务三']}/{expected3}  ·  "
            f"异常 {self._alarm_count}"
        )

    @QtCore.pyqtSlot(dict)
    def _show_task1_result(self, result: dict) -> None:
        self.task1_text.setPlainText(
            "\n".join(f"{key}：{value}" for key, value in result.items() if not str(key).startswith("_"))
        )

    @QtCore.pyqtSlot(str)
    def _show_alarm(self, text: str) -> None:
        self._alarm_count += 1
        self._refresh_progress()
        QtWidgets.QMessageBox.critical(self, "任务异常", text)

    @QtCore.pyqtSlot(float)
    def _start_timer(self, seconds: float) -> None:
        self._timer_deadline = time.monotonic() + seconds

    @QtCore.pyqtSlot()
    def _stop_timer(self) -> None:
        self._timer_deadline = None

    def _update_clock(self) -> None:
        if self._timer_deadline is None:
            self.timer_label.setText("计时 10:00")
            return
        remain = max(0, int(self._timer_deadline - time.monotonic() + 0.999))
        self.timer_label.setText(f"剩余 {remain // 60:02d}:{remain % 60:02d}")
        if remain <= 60:
            self.timer_label.setStyleSheet("background:#a61e1e;color:white;")
        else:
            self.timer_label.setStyleSheet("")

    @QtCore.pyqtSlot(str, bool, str)
    def _worker_finished(self, name: str, ok: bool, message: str) -> None:
        self._busy = False
        self.btn_check.setEnabled(True)
        self.btn_init.setEnabled(not self._initialized)
        self.btn_stop.setEnabled(False)
        if name == "initialize" and ok:
            self._initialized = True
            self.status_labels["runtime"].setText("● 已初始化")
            self.status_labels["runtime"].setObjectName("statusOn")
        self._set_task_buttons(self._initialized)
        if message:
            QtWidgets.QMessageBox.critical(self, "操作失败", message)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        if self.runtime.orchestrator is not None and self.runtime.orchestrator.is_running:
            answer = QtWidgets.QMessageBox.question(
                self,
                "任务仍在运行",
                "任务仍在运行。是否发送停止请求并关闭界面？\n"
                "请同时准备使用实体急停。",
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
            self.runtime.orchestrator.request_stop()
            QtWidgets.QMessageBox.information(
                self,
                "已发送停止请求",
                "普通停止不会抢断当前机器人动作。\n"
                "为保持 TCP 监控和结果回执，窗口将暂时保持打开；"
                "请等待当前动作结束后再关闭。\n"
                "若存在碰撞或人员危险，请立即使用实体急停。",
            )
            event.ignore()
            return
        try:
            self.runtime.stop()
        finally:
            event.accept()


def run_gui(settings: Settings, real_mode: bool = False) -> int:
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setApplicationName(str(settings.get("application.name")))
    app.setFont(QtGui.QFont("Microsoft YaHei UI", 10))
    window = MainWindow(settings, real_mode)
    window.show()
    return int(app.exec_())
