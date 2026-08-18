# 2026 睿抗机器视觉（越疆）视觉引导分拣系统

本项目面向 2026 睿抗机器人开发者大赛 CAIM 工创赛道“视觉系统创新”赛项，使用 Python 作为主控，组合 Dobot Vision Studio、YOLO、Intel RealSense D435、PyQt5 和 DobotStudio Pro，实现任务一和“3D识别抓取”的识别、监控、坐标计算和吸盘抓放。

> **当前定制版本：**任务二已删除；原任务三更名为“3D识别抓取”，识别大圆柱、正方体、梯形、长方体、圆柱、六棱柱和平行四边形。七类放置位姿统一配置为 `[X,Y,Z,Rx,Ry,Rz]`，其中 Z 为 `null`，运行时直接复用当前抓取点 P1 的 Z。程序不再设置或校验软件工作空间边界。

> **设备说明：**软件工作空间限位已按需求移除；机器人控制器自身的运动可达性、实体急停和现场防护不由本程序替代。

## 1. 赛题要求、现场任务书和本项目方案的关系

### 1.1 优先级

比赛现场应按以下优先级执行：

1. 现场最新发布的任务书、评分标准和裁判指令；
2. 赛前睿抗官网或公众号发布的技术更新；
3. 本项目所依据的赛项规则文件；
4. 本 README、默认配置和示例图。

赛项规则中的图 1、图 2、图 3 均为示意或参考。图中的红、蓝、绿、橙色工件，尺寸标记 `a/b/c`、标签样式和摆放位置都不能当成现场固定参数。现场任务书若与本项目默认值不同，必须修改 `config/settings.yaml`，不能为了沿用默认配置而忽略现场要求。

### 1.2 规则明确要求的内容

- 评分验证时，所有工件全程同时出现在初始识别作业区。
- 当前定制程序固定按“任务一 → 3D识别抓取”完成。
- 检测过程中要在交互界面实时显示识别结果。
- 评分运行由裁判宣布开始并计时。队伍可启动程序，之后任务必须自动完成，不允许人工干预。
- 自动运行时间不得超过 10 分钟，即 `600 s`。
- 现场不提供云端网络，模型和依赖必须在本机离线可用。
- 硬件部署和基础连接调试限时 30 分钟。
- 程序设计及软硬件联调不超过 150 分钟。
- 需要完成 2D 视觉、开源 3D 视觉、机器人标定转换、模型训练部署、PC 监控界面和系统通信联调。
- 系统硬件装配 10 分，现场运行 80 分；职业素养与安全最多扣 10 分。每项任务的细分分值、尺寸/坐标容差和误抓扣分以现场评分表为准。

### 1.3 当前软件任务边界

任务一保留 Dobot Vision Studio 的测量、二维码和字符结果接收功能，不执行机器人动作。任务二的运行、配置、界面、采集和训练入口均已删除。3D识别抓取使用 YOLO-OBB、D435 深度和 EIH 标定计算 P1，并按工件类别读取 P2 的 X/Y/Rx/Ry/Rz。

### 1.4 本项目采用、但不是赛题强制的运动流程

用户提出并在本项目中参数化的流程是：

1. 点击真机 UI 的“自动回到拍照位”，或由 Lua 执行 `go_photo`；
2. 点击“运行3D识别抓取”时读取实际 TCP 位姿，只有在拍照位误差内才开始识别；
3. 识别并计算抓取点 P1 `[X抓取,Y抓取,Z,Rx抓取,Ry抓取,Rz抓取]`；
4. DobotStudio Pro 使用 `MovJ` 到 P1，开启吸盘；
5. 读取配置的放置点 P2 `[X放置,Y放置,Z,Rx放置,Ry放置,Rz放置]`，其中 `P2.Z=P1.Z`；
6. DobotStudio Pro 使用自身运动规划由 P1 直接到 P2，释放后回拍照位；
7. Python 收到 `done(at_photo)` 后才识别下一件。

## 2. 系统组成与数据流

```text
Dobot Vision Studio ──TCP──┐
                           │
D435 ──彩色/对齐深度── YOLO/深度/EIH ── Python 主控 ──TCP JSONL── DobotStudio Pro Lua ── E6/吸盘
                           │
                           └── PyQt5 中文监控面板 + JSONL/文本日志
```

- 任务一：Dobot Vision Studio 负责 2D 测量与二维码/字符识别，结果通过 TCP 发给 Python。
- 3D识别抓取：Python 使用 YOLO-OBB 识别七类工件，D435/EIH 换算抓取 XYZ 与最短 RZ，并透传各类配置的放置姿态。
- 机器人控制：Python 不直接和 DobotStudio Pro 同时争夺机械臂控制权。Python 只向 DobotStudio Pro 中运行的 Lua 执行脚本发送抓放命令，Lua 独占实际运动和吸盘 IO。

更完整的层次架构、状态机、异常处理和验收矩阵见 [系统设计说明](docs/系统设计说明.md)。

## 3. 目录说明

```text
RAICOM-Project/
├─ main.py                         # 统一启动入口
├─ config/
│  ├─ settings.yaml                # 现场参数中心；真机前必须逐项填写
│  └─ calibration/
│     ├─ CaliMatrixData.example.yaml
│     └─ CaliMatrixData.yaml       # 现场标定后放入，不随示例生成
├─ dobotstudio/
│  └─ raicom_e6_executor.lua       # DobotStudio Pro 机器人执行脚本
├─ models/
│  └─ task3.pt                     # 3D识别抓取七分类现场模型
├─ src/raicom/                     # 配置、视觉、通信、调度和 UI
├─ tools/check_environment.py      # 环境、端口和真机参数自检
├─ tools/capture_yolo_dataset.py   # D435 RGB 拍照和 train/val/test 一键划分工具
├─ tools/train_yolo.py             # 强制使用本地基础权重的3D识别抓取离线训练入口
├─ tests/                           # DVS、标定、仿真视觉、运行时/UI与协议测试
├─ logs/                           # 中文运行日志和结果 JSONL
├─ requirements.txt
└─ docs/系统设计说明.md
```

示例标定文件只用于验证解析逻辑，绝不能用于比赛现场抓取。模型目录也不会放置虚假的比赛权重。

## 4. raicom-e6 环境与依赖

### 4.1 必须先激活指定环境

在 PowerShell 中执行：

```powershell
conda activate D:\anaconda\envs\raicom-e6
Set-Location D:\CAIM\RAICOM-Project
python -c "import sys; print(sys.executable); print(sys.version)"
```

输出解释器应位于 `D:\anaconda\envs\raicom-e6`。不要在未激活环境时直接双击 `python.exe`；Conda 环境的 DLL 搜索路径可能尚未正确建立。

### 4.2 当前截图中已经存在的关键包

用户截图已经显示：

- `torch 2.3.1+cu121`、`torchvision 0.18.1+cu121`、`torchaudio 2.3.1+cu121`；
- `ultralytics 8.4.50`；
- `pyrealsense2 2.57.7.10387`；
- `numpy 1.26.4`；
- `opencv-python 4.9.0.80` 与 `opencv-contrib-python 4.9.0.80`；
- `PyYAML 6.0.3`、`Pillow 12.1.1` 等。

不要为了运行本项目重新安装或升级 Torch。现有 Torch 是 CUDA 12.1 构建，盲目执行普通 PyPI 的重装命令可能把它替换成 CPU 版或不匹配版本。`requirements.txt` 特意没有固定或重复安装 Torch。

`PyQt5_sip` 不等于 `PyQt5`。前者只是绑定支持包，`pip list` 中只有 `PyQt5_sip` 不能证明界面库可用。以实际导入为准：

```powershell
python -c "import PyQt5; from PyQt5 import QtCore; print(QtCore.PYQT_VERSION_STR, QtCore.QT_VERSION_STR)"
```

只有该命令失败时，才安装缺少的 PyQt5：

```powershell
python -m pip install PyQt5==5.15.11
```

如果现有环境已经能够联合导入全部依赖，就不要在比赛前随意卸载 OpenCV、Torch 或 CUDA 相关包。当前同时安装的两个 OpenCV 发行包版本一致且可导入，赛前不要为了“清理环境”冒险修改；以后重建新环境时再只保留 `opencv-contrib-python`。

赛项规则把 Pandas、Matplotlib 和 PCLPy 列为建议技术栈，其中 PCLPy 明确为可选。本项目主运行链不依赖它们；若环境自检和项目测试通过，不需要为了“凑齐建议列表”在比赛前临时安装。

### 4.3 自检

```powershell
python main.py --check
python -c "import torch; print(torch.__version__, torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

真机参数填写后再执行：

```powershell
python main.py --real --check
```

`--real --check` 通过只表示配置结构和文件存在性通过。程序已移除软件工作空间边界；设备可达性和实体防护由机器人系统及现场负责。

## 5. 第一次运行：只使用模拟模式

```powershell
conda activate D:\anaconda\envs\raicom-e6
Set-Location D:\CAIM\RAICOM-Project
python main.py --demo
```

预期行为：

- 启动中文 PyQt5 监控面板；
- 不连接真实 D435、DVS、机械臂和吸盘；
- 生成模拟任务一结果、模拟图像/深度和模拟机器人应答；
- 按任务一 → 3D识别抓取顺序演示；
- 日志写入 `logs/`。

无界面联调可使用：

```powershell
python main.py --demo --headless --task all
python main.py --demo --headless --task task3
```

只有模拟全流程、停止逻辑和日志均正确后，才能继续通信联调。

运行自动测试：

```powershell
python -B -m unittest discover -s tests -v
```

`-B` 避免生成新的 `.pyc`。测试通过只证明软件逻辑样例通过，不能替代 D435 枚举、EIH 多点误差、吸盘 IO 和 E6 低速真机验收。

## 6. `config/settings.yaml` 逐项说明

所有距离统一使用毫米 `mm`，角度统一使用度 `degree`。配置中的 `null` 是故意的安全占位值，不是推荐值；关键项仍为 `null` 时，`--real` 应拒绝启动。

### 6.1 `application`

| 字段 | 含义 | 现场要求 |
|---|---|---|
| `name` | UI 和日志中的系统名称 | 可保留 |
| `competition_timeout_s` | 自动评分总超时 | 规则上限为 `600`，不得增大 |
| `log_level` | 日志级别 | 正式比赛建议 `INFO` |
| `result_log_jsonl` | 结构化结果日志 | 保持在本地磁盘 |

### 6.2 `network.dvs`

Python 为 TCP 服务端，Dobot Vision Studio 流程作为客户端连接。

| 字段 | 含义 |
|---|---|
| `listen_host` | `0.0.0.0` 表示监听本机所有网卡；DVS 中不能填这个地址 |
| `port` | 默认自定义应用端口 `6001`，可修改，但两端必须一致 |
| `accept_timeout_s` | 等待连接的轮询超时，用于响应停止 |
| `receive_timeout_s` | 单次接收超时，不代表整项任务超时 |
| `max_line_bytes` | 单条换行帧最大长度，默认 `65536`，超长应拒绝 |

DVS 与 Python 在同一电脑时，DVS 目标地址可填 `127.0.0.1`；不同设备时填运行 Python 的电脑有线网卡实际 IPv4。

### 6.3 `network.robot_bridge`

Python 为 TCP 服务端，DobotStudio Pro Lua 为客户端。

- 默认端口 `2006` 是本项目自定义应用端口，不是越疆控制器外部 TCP-IP V4 的 `29999/30004` 等官方端口。
- 正式方案由 DobotStudio Pro Lua 独占机器人控制，不能同时运行另一套 Python V4 运动控制程序。
- `command_timeout_s` 是单次抓放最长等待；超时后禁止自动补发一个新命令，以免重复抓取。
- `heartbeat_s`：空闲链路自动 `ping/pong` 间隔。只有没有在途命令时才发送；机械臂运动期间绝不插入心跳报文。收到心跳只证明通信会话存在，不证明机器人点位、吸盘或安全状态可用。
- 两端 `max_line_bytes`、端口和换行协议必须一致。

### 6.4 `dvs`

| 字段 | 含义 |
|---|---|
| `encoding` | 首选 `utf-8` |
| `fallback_encoding` | 兼容部分 DVS 环境的 `gb18030` |
| `terminator` | 每条结果以 `\n` 结束 |
| `delimiter` | KV/CSV 兼容格式分隔符，默认逗号 |
| `trigger_text` | 任务一开始时 Python 向 DVS 原样发送的触发文本，固定为 `ok` |
| `task1_timeout_s` | 任务一等待完整结果的超时 |
| `expected_results` | 现场任务书规定需返回多件结果时修改 |
| `required_fields` | 现场必须出现的字段名，如 `a`、`b`、`c`、`qr` |

`required_fields: []` 只表示收到一条非空、可解析的结果即可，不表示现场无需检查尺寸或二维码。拿到任务书后应改成实际字段。

### 6.5 `camera`

- `serial`：有多台 RealSense 时必须填 D435 序列号；只有一台时可以先留空，但比赛建议固定。
- `width/height/fps`：RGB 彩色流，D435 主程序默认使用最大 `1920×1080@30`。改变分辨率后要重新核对运行时内参和检测精度。
- `depth_width/depth_height/depth_fps`：双目深度流，默认 `1280×720@30`；D435 的 RGB 和深度最大分辨率不同，不能共用一组尺寸，程序会把深度对齐到 RGB 坐标。
- `align_depth_to_color: true`：深度像素与 YOLO 彩色框对应的前提。
- `warmup_frames`：相机启动后丢弃不稳定帧。
- `flush_frames_after_motion`：机械臂回拍照位后丢弃残留旧帧，防止使用运动前图像。
- `depth_min_mm/depth_max_mm`：有效深度范围。D435 规则参数给出的最小深度约 `300 mm`，现场拍照位必须给最高工件留余量。
- `depth_patch_px`：目标中心邻域中值窗口，必须为奇数。若工件有孔、反光或框中心落在背景，应改为检测框内缩区域或分割掩膜，而不是盲目增大窗口。
- `temporal_depth_samples`：连续多帧深度中值，增加稳定性但会增加延迟。
- `max_temporal_depth_spread_mm`：多帧最大允许极差。超过该值说明目标、相机或深度正在波动，程序应拒绝抓取。
- `color_exposure/depth_exposure`：`null` 表示自动曝光；固定曝光必须现场实测。

### 6.6 `yolo`

- `device: cuda:0`：GPU 不可用时程序可回退 CPU 并报警。回退 CPU 不是重装 Torch 的理由。
- `image_size`、`confidence`、`iou`、`max_detections`：应通过现场验证集调整，不能只看单帧效果。
- `use_hsv_color_fallback`：模型类别不含颜色时使用 HSV 兜底。HSV 只能在目标掩膜或内缩区域统计，整框背景可能造成错误颜色。
- `chinese_font`：检测图叠加中文类别时使用的字体文件。比赛前确认该路径存在并可离线读取。
- `task3.model`：正式运行前必须存在且为 OBB 模型。
- `include_class_keywords/exclude_class_keywords`：所有工件同时出现时，用于阻止当前任务抓走后续任务工件。现场类别名确定后必须填写或验证。
- `known_label`：任务三如果按“已知/差异”分类可填写已知类别；若现场任务直接按多个类别分拣则保持空并按模型类别路由。

### 6.7 `calibration`

- `eih_yaml`：现场 `opencvCalibration.exe` 生成的 EIH YAML 路径。
- `transform_node`：示例为 `CamToTipTransform`。
- `invert_cam_to_tip`：只有多点验证证明矩阵方向相反后才能修改。
- `matrix_translation_unit`：用户示例矩阵平移量约 `0.02~0.12`，应按米读取并转为毫米；现场文件仍须核对。
- `pose_rotation_order`：当前约定 `zyx`，必须与 DobotStudio Pro 返回姿态的含义一致。
- `table_depth_mm`：固定拍照位、检测/放置共用空台面时的深度参考。
- `robot_table_touch_z_mm`：吸盘刚好贴到检测/放置共用台面时机器人 TCP 的 Z。
- `press_down_mm`：在计算出的工件表面基础上向下的补偿。首次验证保持 `0`。
- `xy_offset_mm`：多点验证后的系统性 XY 补偿；不能用它掩盖旋转、镜像或矩阵方向错误。
- `min_object_height_mm/max_object_height_mm`：视觉高度有效性范围，不是机器人工作空间限位。

### 6.8 `robot`

- `user_coordinate_index`：E6 用户坐标系编号，必须与拍照位、示教点和手眼标定采用的坐标系一致。
- `tool_coordinate_index`：吸盘 TCP/工具坐标系编号，必须与 EIH 中 Tip 定义一致。
- `photo_pose_mm_deg`：固定拍照位 `[X,Y,Z,Rx,Ry,Rz]`，必须与 EIH 标定和空台深度采集使用同一工具、用户坐标系和姿态。
- `motion.orientation_mm_deg`：名称保留了历史后缀，实际内容是抓取姿态 `[Rx,Ry,Rz]`。
- `photo_pose_tolerance_mm/deg`：只用于判断当前 TCP 是否已到拍照位。
- `place_poses_mm_deg`：七类工件的 `[X,Y,Z,Rx,Ry,Rz]`；配置中的 Z 必须为 `null`，执行时令 `P2.Z=P1.Z`。
- `travel_speed_percent/pick_speed_percent/acceleration_percent`：首次真机建议降至 `5~10%`，确认后逐步提高。
- `settle_ms`：运动到拍照位后的机械稳定等待。
- `vacuum.api`：必须现场确认填 `ToolDO` 或 `DO`。用户参考脚本中的 `SetIODO` 不适用于本项目核对过的 E6/V4.5 Lua 接口，禁止照抄。
- `vacuum.io_index`：吸盘数字输出通道，必须现场点动确认。
- `on_value/off_value`：吸盘极性。极性相反时交换，不能靠猜。
- `suction_wait_ms/release_wait_ms`：建立/释放真空的等待时间。
- `feedback_di_index`：只有吸盘带真空压力开关且已接到数字输入时才填写；`null` 表示没有独立吸附反馈。
- `feedback_ok_level/feedback_timeout_ms`：压力开关有效电平和吸附等待上限。不能读取 DO 输出状态来冒充“已经吸住”。
- `lua.pc_server_ip`：运行 Python 的电脑有线网卡 IPv4，不是机器人控制器 IP。
- `lua.pc_server_port`：必须与 `network.robot_bridge.port` 一致。
- `lua.reconnect_delay_ms`：Lua 断线后再次尝试连接 Python 的等待时间；重连只恢复通信，不自动重做不确定的运动。
真机还必须打开 `dobotstudio/raicom_e6_executor.lua`，把 Lua 顶部 `CFG` 中的 PC IP/端口、坐标系、拍照位及容差、抓取姿态、速度/加速度和吸盘参数逐项填写成与 `settings.yaml` 一致。软件工作空间边界及自定义接近/抬升/回撤参数已删除。

### 6.9 `tasks`

- `task1/task3.enabled`：当前全流程为任务一后执行 3D识别抓取。
- `recognition_region`：3D识别抓取的识别框，格式为归一化坐标 `[左, 上, 右, 下]`。
- `max_objects`：3D识别抓取的最大分拣数量，当前默认 7。
- `candidate_order`：任务内部的工程顺序，可选从左到右、置信度或距图像中心。赛题未规定内部先后；修改不会改变任务一至三的大顺序。
- `route_by`：当前按类别映射到 `place_poses_mm_deg`。
- `detect_timeout_s`：当前任务等待可用检测的上限。
- `stable_frames/stable_center_tolerance_px`：连续稳定帧判据，防止机械振动或瞬时误检。
- `empty_confirm_frames`：连续多帧无当前任务目标才允许判断为空。一帧漏检不能结束任务。
### 6.10 `simulation`

只用于演示和自动测试。模拟放置姿态同样令 P2.Z 复用 P1.Z。

## 7. Dobot Vision Studio 任务一 TCP 配置

### 7.1 连接角色

1. 先启动 Python，使其监听 `network.dvs.listen_host:network.dvs.port`；
2. 在 Dobot Vision Studio 流程末端配置 TCP 客户端；
3. 目标 IP 填 Python 电脑的实际地址，同机可用 `127.0.0.1`；
4. 编码优先 UTF-8，确实无法配置时可使用 GB18030；
5. 每条结果必须以换行 `\n` 结束。

TCP 没有“消息边界”。一次 `recv` 可能只有半条消息，也可能包含多条消息。接收器按换行累积拆帧，断线时丢弃残留半包，单行超过 `65536` 字节直接拒绝。

每次开始任务一（单独运行或全流程）时，主控会先清除上轮未消费结果，再向已连接的 DVS 原样发送 UTF-8 字符串 `ok`（不附加换行）。DVS 收到 `ok` 后开始采集和计算，并把本轮结果发送回 Python；若触发未发送成功，任务会立即报错，不会在未触发的情况下空等。正式 JSON 应满足 `version=1`、`task=task1`、`ok=true`，数字必须为有限值，并用外部 `seq` 去重；否则只显示错误而不计任务完成。

### 7.2 推荐 JSON 单行格式

推荐在 DVS 中发送：

```json
{"version":1,"seq":1,"task":"task1","ok":true,"a":72.30,"b":145.10,"c":8.50,"unit":"mm","qr":"RAICOM2026","text":"E6"}
```

实际网络数据末尾必须再有一个换行字符。建议字段：

- `version`：协议版本，当前为 `1`；
- `seq`：本次运行内递增序号，用于排查重复结果；
- `task`：固定 `task1`；
- `ok`：DVS 流程是否完成质量判断；
- `a/b/c`：示例测量字段，现场任务书要求什么就改成什么；
- `unit`：建议明确 `mm`；
- `qr/text`：二维码和字符识别结果。

不要把图 2 的 `a/b/c` 当成永久必填字段。现场任务若只要求两项尺寸，或字段名称不同，应同步修改 DVS 发送内容和 `dvs.required_fields`。

### 7.3 兼容 KV 格式

无法方便构造 JSON 时，可发送：

```text
TASK1,version=1,seq=1,ok=1,a=72.30,b=145.10,c=8.50,unit=mm,qr=RAICOM2026,text=E6
```

接收器还兼容纯 CSV，例如 `72.30,145.10,8.50\n`，字段会映射为 `val0/val1/val2`。纯 CSV 缺少字段名和单位，容易在现场改题时混淆，只建议用于最初连通测试，不建议正式评分使用。

## 8. Python 与 DobotStudio Pro Lua 的 JSONL 协议

### 8.1 基本约定

- Python 主程序：TCP Server；
- `dobotstudio/raicom_e6_executor.lua`：TCP Client；
- 默认端口：`2006`，可配置；
- 编码：UTF-8；
- 帧格式：NDJSON/JSONL，一行一个扁平 JSON，末尾 `\n`；
- 数值单位：位置 `mm`，角度 `degree`；
- 禁止 `NaN`、`Infinity` 和区域小数逗号；
- Lua 必须使用累积缓冲处理半包和粘包；
- 任意时刻只允许一个抓放命令在途。

Python 发给 Lua 的命令公共字段为 `v`、`id`、`cmd`。`id` 必须唯一；Lua 按 `id` 幂等处理，不能重复执行已完成的抓取。Lua 主动发送的启动 `HELLO` 是状态通知，没有 `cmd` 字段。

Lua 连接成功且本地 `CFG` 校验通过后发送：

```json
{"v":1,"id":"HELLO","status":"ready","phase":"idle","model":"Dobot-E6"}
```

3D识别抓取命令示例（必须压成一行；数值仅用于展示字段）：

```json
{"v":1,"id":"DIRECT-example","cmd":"pick_place_direct","task":"task3","object_id":"task3-001","route_key":"正方体","pick_x":120.2,"pick_y":-85.4,"pick_z":132.0,"pick_rx":180.0,"pick_ry":0.0,"pick_rz":-25.0,"place_x":250.0,"place_y":120.0,"place_z":132.0,"place_rx":180.0,"place_ry":0.0,"place_rz":15.0,"photo_x":160.0,"photo_y":-60.0,"photo_z":430.0,"photo_rx":180.0,"photo_ry":0.0,"photo_rz":0.0,"photo_tolerance_mm":2.0,"photo_tolerance_deg":2.0,"user":0,"tool":0,"travel_v":20,"pick_v":10,"accel":20,"settle_ms":300,"vacuum_api":"ToolDO","vacuum_io":1,"vacuum_on_level":1,"vacuum_off_level":0,"vacuum_suction_wait_ms":700,"vacuum_release_wait_ms":400,"vacuum_feedback_enabled":false}
```

字段分组：

- 目标与路由：`task/object_id/route_key`；
- 抓取六维位姿：`pick_x/y/z/rx/ry/rz`；
- 放置六维位姿：`place_x/y/z/rx/ry/rz`，且 `place_z` 必须等于 `pick_z`；
- 拍照六维位姿：`photo_x/y/z/rx/ry/rz`；
- 坐标系与运动：`user/tool/travel_v/pick_v/accel/settle_ms`；
- 吸盘复核：`vacuum_api/vacuum_io/vacuum_on_level/vacuum_off_level`、建立/释放等待和可选 DI 反馈字段。

这些字段由 Python 根据 `settings.yaml` 生成；Lua 核对坐标系、拍照位、到位容差和吸盘等现场参数。Lua 不再维护软件工作空间边界。

其他命令：

- `ping`：Python 在空闲且没有在途命令时自动发送，Lua 返回 `pong`；
- `go_photo`：回拍照位；
- `check_photo`：调用 `GetPose(user,tool)` 读取实际 TCP 位姿并返回 `at_photo`；
- `pick_place_direct`：Lua 在运动前再次检查拍照位，然后用 DobotStudio Pro 的 `MovJ` 到 P1、吸取、同 Z 直接到 P2、释放并回拍照位；
- `stop_after_current`：当前命令执行完成后停止后续任务，不抢断正在执行的运动，不改变当前吸盘输出，也不等同实体急停。如果机械臂正在持件或处于异常保压状态，禁止依靠软件停止命令盲目断真空，必须隔离人员、防止工件坠落，再按现场安全操作流程人工处置。

`go_photo`、`check_photo` 和 `pick_place_direct` 都携带同一份拍照位及容差上下文。协议没有软件 `emergency` 命令，真正紧急情况必须使用实体急停。

Lua 对同一 `id` 返回：

- `accepted`：格式和现场参数校验通过；
- `running`：执行中，带 `phase`；
- `done`：命令目标阶段已完成；直接抓放成功终态为 `at_photo`；
- `error`：带 `code`、`message`、`recoverable`；
- `pong`：`ping` 的诊断响应；
- `config_error`：Lua 启动时本地 `CFG` 未填写完整，不能运动。

典型阶段为：

```text
move_to_p1 → vacuum_on → move_p1_to_p2_same_z → vacuum_off →
return_photo → done(at_photo, holding_part=false)
```

Python 必须等待 `done` 才能拍下一帧或发送下一件。Lua 默认在内存保留最近 32 个终态 ID：同一 ID 和完全相同报文只重放终态，不重复运动；同一 ID 搭配不同报文返回冲突错误。控制器或 Lua 工程重启会清空该缓存，因此重启后的未决命令必须人工复核，不能视为仍具备幂等保护。

超时或断线时禁止用新 `id` 自动重发相同抓取，否则可能发生重复运动。先检查机械臂实际位置、吸盘和日志，再由人工在安全状态下决定恢复策略。

E6/V4.5 Lua 脚本使用 `TCPCreate/TCPStart/TCPRead/TCPWrite/TCPDestroy` 建立通信，用 `GetPose` 检查拍照位，并以点位 table 调用 `CheckMovJ/MovJ`，由 DobotStudio Pro 进行路径规划。

`Dobot Vision Studio 4.1.2` 是视觉软件版本，不代表 `DobotStudio Pro` 也是 4.1.2。比赛前要在 DobotStudio Pro 中单独核对软件、控制器和 Lua API 版本，并以 E6 当前版本的函数帮助为准。

### 8.2 在 DobotStudio Pro 中配置执行脚本

1. 用 DobotStudio Pro 连接 E6，先确认机器人、控制器和脚本指令版本；
2. 打开或建立机器人项目，把 `dobotstudio/raicom_e6_executor.lua` 作为主执行脚本；
3. 只编辑文件顶部 `CFG` 现场区，不在比赛现场随意改协议解析和运动函数；
4. 按下表与 `settings.yaml` 逐项核对；
5. 保存并做脚本语法检查；
6. 先启动 Python TCP 服务，再运行 Lua；
7. 查看 DobotStudio Pro 日志中的 `[RAICOM-E6]` 信息。只有收到 `HELLO ready` 才表示 Lua 静态配置通过；`config_error` 必须先修复。

| Python 配置 | Lua `CFG` | 一致性要求 |
|---|---|---|
| `robot.lua.pc_server_ip` | `python_ip` | 完全相同，填 PC 有线网卡 IP |
| `network.robot_bridge.port` / `robot.lua.pc_server_port` | `python_port` | 三处相同 |
| `robot.user_coordinate_index` | `user_index` | 完全相同 |
| `robot.tool_coordinate_index` | `tool_index` | 完全相同 |
| `robot.photo_pose_mm_deg` | `photo_pose` | 六维完全相同 |
| `robot.photo_pose_tolerance_mm/deg` | `photo_tolerance_mm/deg` | 完全相同 |
| `robot.motion.orientation_mm_deg` | `pick_orientation` | 三维完全相同 |
| Python 速度/加速度/稳定等待 | `travel_v/pick_v/acceleration/settle_ms` | 使用 Dobot API 合法百分比，等待值一致 |
| 吸盘 API、IO、极性、等待、反馈 | `vacuum` | 硬件字段和等待必须匹配 |

Lua 独有的 `keep_on_after_pick_error` 控制“抓起后发生运动异常时是否保持真空”，默认保持，以免工件立即坠落；此时脚本返回不可恢复错误，由操作员在实体安全措施下处理。`recent_id_limit` 控制内存幂等缓存数量，不能替代断电后的人工核对。

## 9. D435 深度、抓取 Z 与 EIH 标定

### 9.1 用户要求的 Z 公式

固定拍照位、相机向下观察且机器人 Z 正方向向上时：

```text
物体高度_mm = 空台面参考深度 z_table_mm - 物体顶面深度 z_surface_mm
抓取Z_mm = 吸盘贴台面时机器人Z z_touch_mm + 物体高度_mm - 下压补偿 press_down_mm
```

这正是用户提出的“`z-table` 减去工件深度，再加吸盘刚好贴台面时的机械臂高度”。代码和文档中的“工件中心深度”指检测框中心附近的**可见顶面深度**，不是物体几何中心到相机的距离。

该公式成立需要同时满足：

- 拍照位和姿态与采集 `z_table` 时完全一致；
- D435 深度已对齐到彩色图；
- 深度值已按 RealSense `depth_scale` 转换为毫米；
- 目标中心区域确实落在物体顶面；
- 机器人 Z 正方向、工具坐标系、吸盘长度没有改变；
- 台面在工作区域内近似平行，或已使用按像素/平面变化的台面参考。

如果台面有明显倾斜，单个 `table_depth_mm` 不够，应采集空台参考深度图或拟合台面平面，使用目标像素处的 `D_table(u,v)-D_top(u,v)`。

### 9.2 P1/P2 共用 Z

检测区与放置区使用同一台面高度，同一工件吸取和放下的高度不变：

```text
P1.Z = 抓取Z
P2.Z = P1.Z
```

放置区不再执行第二次视觉测高，也不再生成观察位、接近点、抬升点或回撤点。P1/P2 的 X/Y 和姿态可以不同，Z 始终相同，运动路径交给 DobotStudio Pro 的 `MovJ` 规划。

### 9.3 现场 EIH 文件

使用 `opencvCalibration.exe` 重新完成 Eye-In-Hand 标定后，把最终 YAML 复制为：

```text
config/calibration/CaliMatrixData.yaml
```

用户上传的 `CaliMatrixData20220530144550.yaml` 是解析示例。其 `CamToTipTransform` 结构合法，但属于 2022 年另一套相机/安装关系，不能用于比赛机械臂。

该示例的 RGB 内参约为 `fx=610.44, fy=611.82, cx=314.24, cy=235.35`，与 640×480 图像量级相符；`CamToTipTransform` 平移按米换算后约为 `[-26.73, -123.03, 32.63] mm`，旋转矩阵也具备合法刚体结构。这些检查只能证明“文件可解析、数值像一个标定结果”，不能证明它适用于当前 D435、镜头分辨率、吸盘 TCP 或 E6 安装。

典型坐标链为：

```text
彩色像素(u,v) + 对齐深度d
  → RealSense运行时内参反投影 P_camera
  → CamToTipTransform 得到 P_tip
  → 固定拍照位 T_base_tip 得到 P_base
  → 工作空间/高度/一致性校验
  → 抓取目标
```

即暂按：

```text
P_base = T_base_tip × T_tip_camera × P_camera
```

`CamToTipTransform` 的字段名不能替代实测证明。正式抓取前至少用 3 个分布在工作区不同位置的已知点验证：

1. X/Y 方向是否正确，是否镜像或交换；
2. 平移单位是米还是毫米；
3. 是否需要 `invert_cam_to_tip`；
4. Dobot 姿态角顺序是否为当前 `zyx` 约定；
5. 平均和最大误差是否满足现场评分容差。

若出现随位置变化的旋转误差，禁止用一个 `xy_offset_mm` 硬补；应修正矩阵方向、姿态转换、坐标系或重新标定。

## 10. YOLO 数据采集、训练和部署

### 10.1 现场策略

- 比赛开始会发模型工件和现场任务书；在 150 分钟联调阶段完成补拍、标注、训练、验证和部署。
- 现场无云，基础权重、Ultralytics 包、字体、标注工具和训练脚本必须提前缓存。
- 优先使用较小检测模型和迁移学习，先保证稳定识别和实时性，再增加 epoch。
- 3D识别抓取模型应覆盖七类工件及现场光照、角度和遮挡变化。
- 所有工件同时出现，训练类别、任务过滤和 ROI 必须共同防止跨任务误抓。
- 必须使用 OBB 四点标注和 OBB 基础权重；普通水平框没有可信角度，真机初始化会拒绝 `task=detect` 权重。

### 10.2 D435 拍照和数据集目录

```text
datasets/task3/
├─ photo                         # D435 拍摄的 1280×720 JPG 原图及待划分同名 TXT 标签
├─ images/train
├─ images/val
├─ images/test
├─ labels/train
├─ labels/val
├─ labels/test
└─ data.yaml
```

启动拍照界面：

```powershell
python tools/capture_yolo_dataset.py
```

界面固定使用 D435 的 RGB `1280×720@30 FPS` 彩色流。拍摄 3D识别抓取数据后，
按一次空格键（或点击“拍照”）保存一张 `.jpg` 到对应的 `photo` 目录。用 YOLO
标注工具完成四点旋转框后，应把每张图片的同名 `.txt` 标签也放在 `photo` 中，例如
`task3_001.jpg` 对应 `task3_001.txt`。点击“一键划分当前任务”后，工具按固定随机
种子把图片和已有标签复制到 `train : val : test = 70% : 20% : 10%`；原始
`photo` 文件不会删除。少于 10 张图片或缺少标签时界面会要求再次确认。

每个非空标签行必须是 Ultralytics OBB 的 9 列格式：

```text
class x1 y1 x2 y2 x3 y3 x4 y4
```

四个角点均按图像宽高归一化到 `[0,1]`。旧的 `class cx cy w h` 5 列标签必须
重新标注；训练脚本会在启动前拒绝它，不能把水平框中心尺寸冒充工件方向。

多台 RealSense 同时连接时，可指定 D435 序列号：

```powershell
python tools/capture_yolo_dataset.py --serial 你的D435序列号
```

也可以不启动界面，重新划分指定任务：

```powershell
python tools/capture_yolo_dataset.py --split-only task3
```

`data.yaml` 示例：

```yaml
path: D:/CAIM/RAICOM-Project/datasets/task3
train: images/train
val: images/val
test: images/test
names:
  0: 大圆柱
  1: 正方体
  2: 梯形
  3: 长方体
  4: 圆柱
  5: 六棱柱
  6: 平行四边形
```

类别名必须与 `place_poses_mm_deg` 的七个键一致。项目训练入口强制要求 `--base` 指向已经存在的本地 OBB 基础权重。

3D识别抓取训练：

```powershell

python tools/train_yolo.py `
  --task task3 `
  --data datasets/task3/data.yaml `
  --base tools/offline_weights/yolo11n-obb.pt `
  --epochs 150 `
  --imgsz 640 `
  --batch 8 `
  --device 0

```

脚本默认把训练 checkpoint 写到项目和 Git 工作区之外的 `D:/RAICOM-YOLO-Runs`，正式模型复制回 `models/task3.pt`。保存权重时先写临时文件再原子替换，并对 Windows 文件临时占用自动重试。训练完成后使用 `data.yaml` 的 `test` 集评估最佳权重。若程序出现下载行为，说明 `--base` 指向的本地权重不完整，应立即停止并检查离线文件。

从旧训练目录中的 `last.pt` 恢复时，脚本会保留 checkpoint 内的轮次、优化器和目标总轮数，并把后续权重迁移到新的 D 盘独立目录：

```powershell
python tools/train_yolo.py --task task3 --data datasets/task3/data.yaml --resume runs/task3_field-2/weights/last.pt --device 0
```

部署前必须用未参与训练的图像验证误检、漏检、相似图案、旋转、遮挡、反光和颜色变化，不能只确认训练集图片。

运行时从 OBB 长轴取无向角，经当前 EIH 矩阵换算为机器人 XY 平面方向，并归一化到
`[-90°,90°)`，因此抓取使用达到同一工件方向的最小 RZ。吸取后机械臂先垂直抬升，
在安全高度保持 XYZ 原地转到 `RZ=0°`，然后才水平转运并放置。`RZ+` 按机器人
XY 右手系为逆时针，`RZ-` 为顺时针；圆柱体没有有效朝向，固定使用 `RZ=0°`。

## 11. 真机启动与调试顺序

### 11.1 永远按层次推进

1. **机械安全：**确认实体急停、机械臂固定、吸盘安装、线缆无干涉、工作区清空。
2. **环境自检：**激活 raicom-e6，执行 `python main.py --check`。
3. **网络：**给 PC、视觉设备和机器人设置同网段静态 IP；确认端口未占用和防火墙放行。
4. **D435：**使用 RealSense Viewer 验证 USB 3、彩色/深度流、序列号和深度尺度，关闭 Viewer 后再启动本项目。
5. **DVS：**只测试 TCP 连通和任务一结果，不启动机器人。
6. **Lua：**核对 DobotStudio Pro/E6 Lua 版本和 `CFG`；先确认 `HELLO/ready`。在工作区清空、低速和全部点位已校验后才测试 `go_photo`，不得先发抓放。
7. **标定：**放置现场 YAML，验证矩阵结构、单位、方向和至少 3 个已知点。
8. **台面 Z：**固定拍照位采集空台深度；示教并记录吸盘刚贴台面时的 TCP Z。
9. **落点：**示教七类 P2 的 X/Y/Rx/Ry/Rz；Z 保持 `null`。
10. **高位空跑：**吸盘关闭，把所有 Z 整体提高，速度/加速度设为 `5~10%`。
11. **单点抓放：**只放一个软质测试件，验证吸盘 IO、下降方向和释放。
12. **单任务：**单独测试 3D识别抓取。
13. **完整流程：**所有工件同时出现，按任务一 → 3D识别抓取连续运行并计时。

### 11.2 启动次序

推荐顺序：

1. 启动 Python 服务和 UI；
2. 确认 UI 显示 DVS/Lua 监听端口；
3. 启动 DVS 流程并连接；
4. 在 DobotStudio Pro 中打开并运行 `raicom_e6_executor.lua`；
5. 等待 Lua `hello/ready`；
6. 点击或命令启动任务；
7. 裁判正式计时时只执行一次启动，后续不操作界面。

正式模式命令：

```powershell
python main.py --real --check
python main.py --real
```

真机必要参数不完整时，先填写配置后再运行。

## 12. PyQt5 监控面板应观察什么

正式运行时至少确认：

- 当前模式明确显示“模拟”或“真机”；
- 当前任务和总体顺序；
- 600 秒倒计时；
- DVS、D435、YOLO、Lua/机器人连接状态；
- 同步实时更新的 RGB 彩色图、Depth 伪彩色图和 YOLO 识别图；
- YOLO 检测框中的类别、颜色、置信度及按“空台深度 − 工件表面深度”计算的物体高度；
- “3D识别抓取”的绿色识别区域是否只覆盖待抓取区。在画面内按住鼠标左键拖拽即可重画；程序只接受中心点位于框内的检测，框外已放置工件不会再次进入稳定判断和抓取流程；
- 深度值、相机坐标、机器人坐标、目标落点；
- 机器人阶段，如到 P1、吸取、同 Z 到 P2、释放、回拍照位；
- 成功数、失败数、分拣数量上限；
- 中文日志和明确错误原因。

“停止任务/停止运动”按钮只向软件状态机和 Lua 发停止请求，不得标注成“实体急停”。任务正在运行时，点击窗口关闭也只会发送该请求并暂时保持窗口和 TCP 连接，不会在动作中途强行断线。发生碰撞风险、人员进入工作区或运动失控时，应使用机械臂实体急停。

## 13. 现场 30 / 150 / 10 分钟安排建议

### 13.1 硬件部署 30 分钟

- 0～5 分钟：清点相机、镜头、光源、D435、机器人、吸盘、线缆；
- 5～15 分钟：机械安装，检查无错装、漏装、松动；
- 15～23 分钟：连接 IO、通信、电源，布线整齐，无短接、漏接、错接、松动；
- 23～27 分钟：静态 IP、USB 3、设备枚举；
- 27～30 分钟：拍照留证，自检并等待裁判检查。

### 13.2 程序与联调 150 分钟

- 尽快读取现场任务书并填写“现场必填清单”；
- 先让任务一 DVS 结果和 UI 跑通；
- 采集七类工件数据并训练 OBB 模型；
- 完成 EIH、空台深度、贴台 Z、拍照位和落点示教；
- 按“通信 → 高位空跑 → 单点抓放 → 单任务 → 全流程”逐层验收；
- 至少预留最后 20～30 分钟做完整计时、重启和断线测试。

### 13.3 正式运行 10 分钟

- 裁判摆放全部工件后，不再调整工件；
- 启动一次程序；
- 自动按任务一 → 3D识别抓取运行；
- 禁止人工点击跳过、修改坐标、重新摆件或手动选择目标；
- 任何恢复操作是否允许，必须先听从裁判。

## 14. 常见故障排查

### 14.1 中文乱码

- 所有 `.py/.yaml/.jsonl/.lua/.md` 保存为 UTF-8；
- PowerShell 可先执行 `chcp 65001`；
- UI 字体使用微软雅黑等中文字体；
- DVS 若只能输出本地编码，保留 `fallback_encoding: gb18030`；
- 不要把已经乱码的日志再次另存覆盖原文件。

### 14.2 `ModuleNotFoundError: PyQt5`

先确认解释器路径，再运行 PyQt5 导入命令。`PyQt5_sip` 存在仍可能缺少 `PyQt5`。只在导入失败时安装 `PyQt5==5.15.11`。

### 14.3 CUDA 不可用

```powershell
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.version.cuda)"
```

先检查是否激活 raicom-e6、显卡驱动和系统重启状态。不要先重装 Torch。程序可临时回退 CPU，但必须重新验证是否能在 10 分钟内完成。

### 14.4 找不到 D435

- 使用 USB 3 端口和合格线缆；
- 关闭 RealSense Viewer 和其他占用相机的软件；
- 检查 `camera.serial`；
- 重新插拔并查看设备管理器；
- 当前电脑若未枚举到设备，不能把“已安装 pyrealsense2”当成硬件验证通过。

### 14.5 深度为 0、跳变或落在背景

- 确认深度对齐到彩色；
- 检查目标是否小于最小测距、表面是否反光/黑色；
- 查看内缩 ROI，而不是只看框中心一个像素；
- 使用邻域中值、时间中值和离群值剔除；
- 深度质量不合格时拒绝抓取，禁止用固定猜测 Z 继续运动。

### 14.6 EIH 后 XY 镜像、旋转或偏差随位置变化

- 检查 `CamToTipTransform` 方向和 `invert_cam_to_tip`；
- 检查米/毫米转换；
- 检查拍照位是否与标定一致；
- 检查 `[Rx,Ry,Rz]` 含义和旋转顺序；
- 检查使用的是对齐后彩色流运行时内参；
- 重新做多点标定，不要用单一 XY 偏移掩盖旋转误差。

### 14.7 抓取 Z 过高或撞桌

- 确认 `z_table` 和工件顶面深度都为毫米；
- 确认 `z_surface` 取在顶面而不是桌面；
- 确认机器人 Z 正方向；
- 检查吸盘工具坐标和长度是否改变；
- 首次把 `press_down_mm` 设为 `0`；
- 用已知高度量块验证公式，再逐步增加下压补偿。

### 14.8 DVS 连接不上或结果不完整

- Python 必须先监听；
- DVS 是客户端，目标不能填 `0.0.0.0`；
- 检查两端端口、IP、网段和 Windows 防火墙；
- 每条消息必须有换行；
- JSON 必须使用英文冒号/逗号和双引号；
- 检查 `required_fields` 是否仍是旧任务字段；
- 断线重连后接收器会清除半包，DVS 应重新发送完整一行。

### 14.9 Lua 已连接但没有 `done`

- 查看最后一个 `running.phase`；
- 在 DobotStudio Pro 检查运动报警、`CheckMovJ/CheckMovL` 返回和 `Wait` 阶段；
- 检查吸盘 API 已明确选择 `ToolDO` 或 `DO`，不能使用参考文件的 `SetIODO`；
- 检查用户/工具坐标系、拍照位和 P1/P2 点位；
- 不要用新 `id` 盲目重发；
- 若位置不安全，使用实体急停并按裁判要求恢复。

### 14.10 模型漏检、跨任务误抓

- 确认加载的是现场 `task3.pt`；
- 检查类别名称和 `include/exclude` 过滤；
- 降低置信度前先看误检数据；
- 每抓一件回拍照位重拍，禁止继续使用抓放前的旧帧；
- 当前任务“无目标”必须连续多帧确认；达到 `max_objects` 上限后应正常结束，不再继续抓取；

## 15. 比赛前和现场必须填写清单

以下任一关键项不确定时，不允许真机抓取：

- [ ] 现场任务书版本、评分表版本和裁判补充说明；
- [ ] 任务一指定道具、测量字段、单位、容差、二维码/字符规则；
- [ ] 七类工件的类别名和任务内优先级；
- [ ] 每个类别对应的落料点和摆放要求；
- [ ] Python 电脑有线网卡 IP、DVS 端口、Lua 端口和防火墙；
- [ ] D435 序列号、分辨率、帧率、深度尺度和曝光；
- [ ] 现场 EIH YAML 路径、矩阵节点、方向、平移单位和姿态顺序；
- [ ] 固定拍照位 `[X,Y,Z,Rx,Ry,Rz]`；
- [ ] 抓取区空台面深度 `table_depth_mm`；
- [ ] 吸盘刚贴抓取台面时机器人 `robot_table_touch_z_mm`；
- [ ] 已知高度量块的 Z 公式验证结果；
- [ ] 抓取姿态与拍照位到位容差；
- [ ] 吸盘 `ToolDO/DO` 选择、IO 通道、开启/关闭极性、建立真空时间和可选压力开关反馈；
- [ ] 七类 P2 的 `X/Y/Rx/Ry/Rz`，并确认配置 Z 为 `null`；
- [ ] 实测每类 P1.Z 与 P2.Z 相同的直接抓放；
- [ ] 速度、加速度、到位等待和单命令超时；
- [ ] YOLO 权重、类别映射、置信度、NMS 和任务过滤；
- [ ] 深度 ROI、无效深度判据、合理高度范围；
- [ ] 无工件/任务完成判据；
- [ ] Python↔DVS 和 Python↔Lua 实际报文样例；
- [ ] 日志磁盘空间、离线字体、基础权重和所有依赖；
- [ ] 实体急停、人员隔离、长发/服装、电气和线缆安全。

## 16. 设计报告需要覆盖的内容

赛题要求提供设计报告。本项目的 [系统设计说明](docs/系统设计说明.md) 可作为技术底稿，但现场报告仍应补充真实照片、实测数据和最终参数：

- 硬件层、算法层、应用层的系统总体架构图；
- 核心部件型号/参数、算法库和选型依据；
- 通信拓扑、协议、数据方向、交互频率和接口字段；
- 从启动到任务完成的工艺流程、触发条件和异常处理；
- 图像采集、预处理、目标检测、深度、坐标转换和运动规划完整链路；
- 标定误差、深度误差、重复定位、推理帧率和全流程耗时；
- 断线、无效深度、越界、漏检、机器人报警和停止策略。

## 17. 安全与职业素养

- 电气装调按安全规范操作；
- 禁止错装、漏装、松动，禁止短接、漏接、错接和接线松动；
- 线缆整齐，不能进入机械臂运动区域；
- 按规则穿长衣/T恤/比赛服和平底绝缘鞋或旅游鞋，禁止拖鞋、高跟鞋、赤脚、无袖背心和短裤；
- 头发不遮眉且露耳，长发必须盘起、扎起或置于安全帽下；
- 比赛结束后工具归位并清扫现场。

任何软件功能都不能替代实体急停、控制器安全限制和现场监护。
