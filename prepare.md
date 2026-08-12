# 赛前参数填写与全流程运行手册

> 适用项目：`D:\CAIM\RAICOM-Project`  
> 机械臂：越疆 Dobot Magician E6  
> 主程序环境：`D:\anaconda\envs\HKtest`  
> 距离单位统一为 `mm`，角度单位统一为 `degree`。

本文档只告诉你“哪里要填、怎么量、按什么顺序测试和运行”。赛题规则、协议原理和系统设计细节请另看 `README.md` 和 `docs/系统设计说明.md`。

## 1. 先看结论：需要你处理的文件

赛前和现场主要需要处理下列位置。

| 位置 | 你要做什么 | 是否必须 |
|---|---|---|
| `config/settings.yaml` | 填写视觉、标定、网络、机械臂、吸盘、落料点和任务参数 | 必须 |
| `dobotstudio/raicom_e6_executor.lua` 顶部 `CFG` | 填写与 `settings.yaml` 一致的 E6 执行参数 | 必须 |
| `models/task2.pt` | 放入任务二 YOLO 权重 | 必须 |
| `models/task3.pt` | 放入任务三 YOLO 权重 | 必须 |
| `config/calibration/CaliMatrixData.yaml` | 放入现场 `opencvCalibration.exe` 生成的 EIH 标定文件 | 必须 |
| Dobot Vision Studio 4.1.2 工程 | 配置任务一识别流程和 TCP 结果输出 | 必须 |
| `config/calibration/validation_points.csv` | 填入至少 3 个已知点，验证 EIH 矩阵方向和误差 | 必须验证，文件名可自定 |
| Windows 有线网卡和防火墙 | 设定静态 IP，允许 DVS 端口和 Lua 端口 | 必须 |

不要把 `config/calibration/CaliMatrixData.example.yaml` 用于真机。它只是你上传的 2022 年样例数据的解析示例，不对应比赛现场的相机安装关系。

## 2. 填写前的安全规则

1. 不确定的机械臂数值必须保持 `null`，不得为了通过检查而填假数。
2. `settings.yaml` 里的一组机械臂参数，在 Lua `CFG` 中也有一组。两处必须核对，协议会在不一致时拒绝运动。
3. 本文中出现的 IP、位姿和落料点只是格式说明，禁止照抄到真机。
4. 首次真机调试使用 `5%~10%` 速度，清空工作区，确保实体急停在手边。
5. 软件“停止任务”不是实体急停，也不会抢断已开始的动作。
6. 只有在 DobotStudio Pro 中完成脚本语法检查、低速单步和空载轨迹验证后，才能放工件。

## 3. 第一处：填写 `config/settings.yaml`

### 3.1 比赛时间和日志

| 配置键 | 怎么填 |
|---|---|
| `application.competition_timeout_s` | 默认 `600`，即10分钟；只有现场评分规则明确改变才修改 |
| `application.log_level` | 调试可用 `DEBUG`，正式比赛建议 `INFO` |
| `application.result_log_jsonl` | 建议保持 `logs/results.jsonl` |

### 3.2 DVS TCP 网络

| 配置键 | 怎么填 |
|---|---|
| `network.dvs.listen_host` | 保持 `0.0.0.0`，表示 Python 监听电脑所有网卡；不要填 DVS 设备 IP |
| `network.dvs.port` | 默认 `6001`，可修改为未占用端口；DVS 工程中必须填相同端口 |
| `dvs.encoding` | DVS 能发 UTF-8 时保持 `utf-8` |
| `dvs.fallback_encoding` | DVS 只能发中文本地编码时保持 `gb18030` 兜底 |
| `dvs.trigger_text` | Python 向 DVS 发的软触发文本；必须与 DVS 工程约定一致 |
| `dvs.task1_timeout_s` | 任务一最长等待时间，要大于 DVS 完整处理耗时 |
| `dvs.expected_results` | 现场任务一需要接收几条结果；一条结果包含全部字段时填 `1` |
| `dvs.required_fields` | 按现场任务书填写必须字段，例如 `["a", "b", "c", "qr", "text"]` |

DVS 正式输出推荐一行 JSON，并在末尾加换行 `\n`：

```json
{"version":1,"seq":1,"task":"task1","ok":true,"a":72.30,"b":145.10,"c":8.50,"unit":"mm","qr":"RAICOM2026","text":"E6"}
```

`a/b/c` 只是格式示例。现场任务书要求什么尺寸、二维码或字符字段，就同时修改 DVS 发送内容和 `required_fields`。

### 3.3 D435 相机

| 配置键 | 怎么填 |
|---|---|
| `camera.serial` | 只连一台 D435 可保持 `null`；多台 RealSense 时必须填目标序列号 |
| `camera.width/height/fps` | 必须与标定、训练和现场运行规格一致；默认 `640×480@30` |
| `camera.align_depth_to_color` | 真机抓取必须保持 `true` |
| `camera.depth_min_mm/depth_max_mm` | 按相机到桌面的实际范围设定，不要设得过宽 |
| `camera.depth_patch_px` | 检测框中心取样区，必须是大于等于3的奇数；小工件可降到5或7 |
| `camera.temporal_depth_samples` | 每次抓取的深度帧数，默认5 |
| `camera.max_temporal_depth_spread_mm` | 多帧深度极差上限，超过就拒绝抓取 |
| `camera.color_exposure/depth_exposure` | `null` 表示自动曝光；只有现场自动曝光明显波动时才换成实测固定值 |

查看 D435 序列号可执行：

```powershell
python -c "import pyrealsense2 as rs; c=rs.context(); print([(d.get_info(rs.camera_info.name),d.get_info(rs.camera_info.serial_number)) for d in c.devices])"
```

### 3.4 YOLO 模型与类别

| 配置键 | 怎么填 |
|---|---|
| `yolo.device` | RTX 4060 保持 `cuda:0`；CUDA 不可用时程序会回退 CPU |
| `yolo.confidence` | 用现场验证集确定；先保持 `0.70`，不要为了解决漏检盲目降得很低 |
| `yolo.iou` | NMS 阈值，先保持 `0.45` |
| `yolo.task2.model` | 保持 `models/task2.pt`，并把实际权重放到该位置 |
| `yolo.task3.model` | 保持 `models/task3.pt`，并把实际权重放到该位置 |
| `include_class_keywords` | 模型还会检测其他任务工件时，填本任务允许的类别关键词 |
| `exclude_class_keywords` | 填必须排除的类别关键词 |
| `yolo.task3.known_label` | 任务三若按“已知图案/非已知图案”分类，填模型中已知图案的完整类别名；否则保持 `null` |

任务二如果使用 `route_by: color`，模型类别名最好直接包含 `red/blue/green/yellow` 或中文颜色名。类别名没有颜色时，程序才会尝试 HSV 颜色兜底。现场颜色不在当前四色范围时，优先把颜色写入模型类别名并改用 `route_by: class`，不要临近比赛随意改 HSV 阈值。

训练命令，`--base` 必须是本地已经存在的离线基础权重：

```powershell
python tools/train_yolo.py --task task2 --data datasets/task2/data.yaml --base offline_weights/yolo11n.pt --epochs 40 --imgsz 640 --batch 8 --device 0
python tools/train_yolo.py --task task3 --data datasets/task3/data.yaml --base offline_weights/yolo11n.pt --epochs 40 --imgsz 640 --batch 8 --device 0
```

### 3.5 EIH 标定文件和坐标变换

| 配置键 | 怎么填 |
|---|---|
| `calibration.eih_yaml` | 建议保持 `config/calibration/CaliMatrixData.yaml` |
| `calibration.transform_node` | 你的文件节点名为 `CamToTipTransform` 时保持不变 |
| `calibration.invert_cam_to_tip` | 先填 `false`；至少3个已知点显示矩阵方向相反时才改为 `true` |
| `calibration.matrix_translation_unit` | 根据现场 YAML 平移向量单位填 `m` 或 `mm`；不能仅根据旧文件猜测 |
| `calibration.pose_rotation_order` | 当前为 `zyx`，必须通过多点验证确认 |
| `calibration.table_depth_mm` | 机械臂在固定拍照位，抓取区无工件时的 D435 深度中值；只用于计算被抓工件高度 |
| `calibration.robot_table_touch_z_mm` | 抓取区无工件，同一用户/工具坐标系下，吸盘刚贴抓取台面时的 TCP Z；不用于任务三放置台面 |
| `calibration.press_down_mm` | 正值表示在计算表面的基础上再向下压；首次保持 `0.0` |
| `calibration.xy_offset_mm` | 只能修正多点上近似相同的小平移偏差；镜像、旋转或误差随位置变化时禁止用它硬补 |
| `min_object_height_mm/max_object_height_mm` | 按现场工件最小/最大合理高度填写，用于拒绝桌面误检和异常深度 |

现场重新标定后，复制文件：

```powershell
Copy-Item "D:\现场标定输出\CaliMatrixData时间戳.yaml" ".\config\calibration\CaliMatrixData.yaml"
```

测量空台面深度：

```powershell
python tools/measure_table_depth.py --frames 60
```

如果中央 80×80 不是实际抓取区，传入空白桌面 ROI：

```powershell
python tools/measure_table_depth.py --frames 60 --roi X Y W H
```

把输出的 `z-table 中值` 填入 `calibration.table_depth_mm`。采集期间机械臂必须已在最终拍照位且保持静止。

验证工具复制一份表格后填写：

```powershell
Copy-Item .\config\calibration\validation_points.example.csv .\config\calibration\validation_points.csv
python tools/validate_handeye.py .\config\calibration\validation_points.csv --max-error-mm 5
```

CSV 每行格式是：

```text
u,v,depth_mm,expected_robot_x_mm,expected_robot_y_mm
```

至少取 3 个分散点，建议取左上、右上、左下、右下和中心共 5 点。`--max-error-mm` 要按现场评分容差设定，不要为了“让测试通过”而随意设得很大。

本项目实际使用的 Z 公式是：

```text
工件高度 = 空台面深度 - 工件可见顶面深度
抓取Z = 吸盘贴台时机械臂Z + 工件高度 - 下压补偿
```

用已知高度的软质量块验证公式后，才能测试吸取。

### 3.6 E6 坐标系、拍照位和工作空间

| 配置键 | 怎么填 |
|---|---|
| `robot.user_coordinate_index` | DobotStudio Pro 中实际使用的用户坐标系编号 `0~9` |
| `robot.tool_coordinate_index` | 已设定好吸盘 TCP 的工具坐标系编号 `0~9` |
| `robot.photo_pose_mm_deg` | 机械臂固定拍照位 `[X,Y,Z,Rx,Ry,Rz]`，必须与 EIH 标定和采集 `z-table` 时一致 |
| `robot.workspace_mm.x/y/z` | 在同一用户坐标系下，通过低速示教确认的软件安全边界 |
| `robot.motion.orientation_mm_deg` | 吸盘竖直向下时的 `[Rx,Ry,Rz]` |
| `robot.motion.z_up_sign` | 机械臂当前用户坐标的 Z 正方向向上填 `1`，相反填 `-1` |

工作空间必须包含下列所有实际点：

- 固定拍照位；
- 每个工件的抓取点和抓取上方点；
- 吸取后的垂直抬升点；
- 每个落料点的高位转运点、放置点和释放回撤点。

边界不能照抄 E6 理论工作半径，必须结合实际安装、桌面、相机、吸盘、线缆和障碍物设置得更保守。

### 3.7 运动参数

| 配置键 | 含义与填法 |
|---|---|
| `robot.motion.approach_mm` | 抓取点上方的接近距离 |
| `robot.motion.pick_lift_mm` | 吸取后 X/Y 不变，只沿 Z 抬升的距离 |
| `robot.motion.release_retract_mm` | 释放后 X/Y 不变，只沿 Z 回撤的距离 |
| `travel_speed_percent` | 前往抓取上方、水平转运和回拍照位的速度比例 |
| `pick_speed_percent` | 下降、抬升和回撤的直线运动速度比例 |
| `acceleration_percent` | 加速度比例 |
| `settle_ms` | 到位后的稳定等待时间 |

第一次真机调试，将 `travel_speed_percent` 和 `pick_speed_percent` 都降到 `5~10`，并在 Lua `CFG.motion` 中同步修改。速度调高前必须重新完整验证轨迹和停车距离。

### 3.8 吸盘 IO

| 配置键 | 怎么填 |
|---|---|
| `robot.vacuum.api` | 吸盘接在末端输出填 `"ToolDO"`，接在底座/控制箱输出填 `"DO"` |
| `robot.vacuum.io_index` | `ToolDO` 只能填1或2；`DO` 按 E6 实际接线填1~16 |
| `on_value/off_value` | 在 DobotStudio Pro 监视/点动界面实测极性；两者必须分别为0和1且不能相同 |
| `suction_wait_ms` | 打开吸盘后建立真空的等待时间 |
| `release_wait_ms` | 关闭吸盘后释放工件的等待时间 |
| `feedback_di_index` | 只有真的真空压力开关反馈才填 DI 号；没有保持 `null` |
| `feedback_ok_level` | 压力开关表示“吸住”时的电平 |
| `feedback_timeout_ms` | 等待真空反馈的超时时间 |

不能用 DO 输出状态代替“真空已建立”的压力反馈。没有压力开关时，依靠合理的 `suction_wait_ms` 和现场重复试验确认。

### 3.9 Python 与 Lua 的网络

| 配置键 | 怎么填 |
|---|---|
| `network.robot_bridge.listen_host` | 保持 `0.0.0.0` |
| `network.robot_bridge.port` | 默认 `2006`，不得与29999/30004/30005/30006等控制器端口冲突 |
| `robot.lua.pc_server_ip` | 填运行 Python 的比赛电脑有线网卡 IPv4，不是 E6 IP，也不能填 `127.0.0.1` |
| `robot.lua.pc_server_port` | 必须与 `network.robot_bridge.port` 完全一致 |
| `command_timeout_s` | 一次回拍照位或抓放动作的最长等待时间，必须大于低速完整动作耗时 |

查看电脑当前 IPv4：

```powershell
ipconfig
Get-NetIPAddress -AddressFamily IPv4 | Where-Object {$_.IPAddress -notlike "127.*"} | Format-Table InterfaceAlias,IPAddress,PrefixLength
```

例如 E6 是 `192.168.5.1`，电脑可以设为同网段且未被占用的地址，但必须以现场网络为准，不得直接照抄示例。

网络检查：

```powershell
$e6Ip = "把这里改成E6的实际IPv4"
$pythonPcIp = "把这里改成Python电脑的实际IPv4"
ping $e6Ip
Test-NetConnection $pythonPcIp -Port 2006
```

`Test-NetConnection` 只有在 Python 已经监听2006端口时才会成功。如果 Windows 防火墙拦截，在有管理员权限的前提下为当前 Python 或 TCP 6001/2006 添加入站规则；不要关闭整个系统防火墙。

### 3.10 任务二/三落料点

`robot.place_points` 是真机落料点。每个任务的 `default` 必须填写；需要真正分类时，还必须填实际会命中的每个路由点。

| 字段 | 含义 |
|---|---|
| `x_mm/y_mm` | 分类区放置点的机械臂 X/Y；任务二和任务三都必填 |
| `down_mm` | 仅任务二使用：保持吸取后的高位 Z 移到目标 XY 后的固定下降距离 |

任务三不再配置 `down_mm`。机械臂持件到放置点观察位后，通过 EIH 和实时深度点云识别当前台面/堆顶 Z；随后先在观察 XY 垂直下降到 `释放Z + release_retract_mm`，保持该低位 Z 只移动 XY 到放置点，再垂直下降释放，避免把无逆解的放置 XY 高位当作终点。多个路由使用同一 XY 时，首件放在台面，后续件自动叠到上一件顶面；抓取台面和放置台面不要求同高。

同时现场确认 `robot.motion.place_inspection_z_mm`：当前按实测限制为 `410 mm`。吸取后先在原抓取 XY 抬升 `pick_lift_mm`，保持该低位 Z 只移动到观察 XY，再在该观察 XY 垂直升到 `410 mm`；禁止在任意抓取 XY 直接升到 410。在这个观察位、相同姿态和放置 XY 下，空台记录 `tasks.task3.placement_vision.place_table_depth_mm`，低速示教吸盘刚接触空放置台面时记录 `place_table_touch_z_mm`；实验室值分别为 `388/90 mm`，比赛现场只修改这两项。程序再用实时顶面深度自动计算首件和后续堆叠高度。放置测高的 `depth_min_mm=250` 与抓取区 `camera.depth_min_mm=300` 相互独立。任务三视觉失败时程序会保持吸盘并停止，必须按现场安全流程人工处置。

路由对应关系：

- `tasks.task2.route_by: color`：优先查找 `red/blue/green/yellow`，没有可用点时使用 `default`。
- `tasks.task2.route_by: shape`：查找 `cube/cylinder`，需要你在 `place_points.task2` 中增加同名键。
- `tasks.task2.route_by: class`：键名必须与 YOLO 模型类别名完全一致。
- `tasks.task3.route_by: match_or_class`：填了 `known_label` 时使用 `match/not_match`，否则先查找模型类别名，最后使用 `default`。

每个落料点都要用当前用户坐标系和吸盘 TCP 低速示教，并检查放置、回撤和回拍照位路径不碰撞。

### 3.11 任务数量、选择顺序和完成条件

| 配置键 | 怎么填 |
|---|---|
| `tasks.task1.enabled` | 正式比赛保持 `true` |
| `tasks.task1.robot_action` | 保持 `none`；公布赛题只明确任务一识别和输出，现场任务书若新增机械臂动作，必须再修改程序 |
| `expected_objects` | 现场应该成功处理的工件数 |
| `max_objects` | 允许处理的最大工件数，必须大于等于 `expected_objects` |
| `candidate_order` | `left_to_right`、`confidence` 或 `nearest_center`，按现场任务内顺序要求填 |
| `route_by` | 按现场分类规则填 `color/shape/class/match_or_class` |
| `detect_timeout_s` | 本任务获取稳定检测的最长时间 |
| `empty_confirm_frames` | 连续多少帧无本任务目标才认定无工件 |
| `stable_frames` | 连续多少帧匹配同一目标才进入深度计算 |
| `stable_center_tolerance_px` | 多帧目标中心可允许的像素偏差 |

公布规则下任务二和任务三默认各2件，但必须以比赛当天任务书为准。所有工件全程同时出现，因此模型关键词和类别必须能防止任务二误抓任务三工件，反之亦然。

## 4. 第二处：填写 DobotStudio Pro Lua 脚本

打开：

```text
D:\CAIM\RAICOM-Project\dobotstudio\raicom_e6_executor.lua
```

只修改脚本顶部的 `local CFG = {...}` 现场参数区，不要改后面的协议解析和运动状态机。

### 4.1 Python 与 Lua 必须核对的字段

| `settings.yaml` | Lua `CFG` | 要求 |
|---|---|---|
| `robot.lua.pc_server_ip` | `python_ip` | 完全相同，都是 Python 电脑有线网卡 IP |
| `network.robot_bridge.port` | `python_port` | 完全相同 |
| `robot.user_coordinate_index` | `user_index` | 完全相同 |
| `robot.tool_coordinate_index` | `tool_index` | 完全相同 |
| `robot.photo_pose_mm_deg` | `photo_pose.x/y/z/rx/ry/rz` | 六个值完全相同 |
| `robot.motion.orientation_mm_deg` | `pick_orientation.rx/ry/rz` | 三个值完全相同 |
| `robot.workspace_mm` | `workspace` | 六个边界完全相同；每条运动命令都会交叉校验 |
| `approach_mm` | `motion.approach_mm` | 完全相同 |
| `pick_lift_mm` | `motion.pick_lift_mm` | 完全相同 |
| `release_retract_mm` | `motion.release_retract_mm` | 完全相同 |
| `z_up_sign` | `motion.z_up_sign` | 完全相同 |
| Python 旅行/抓取速度 | `motion.travel_v/pick_v` | 建议完全相同，Lua 绝不能设得更高 |
| `acceleration_percent` | `motion.acceleration` | 完全相同 |
| `settle_ms` | `motion.settle_ms` | 完全相同 |
| `robot.vacuum.*` | `vacuum.*` | API、IO、极性、等待和反馈完全相同 |

Lua 还有两个独有安全参数：

- `vacuum.keep_on_after_pick_error = true`：抓起工件后如果运动异常，默认保持真空，避免工件立即掉落。保持 `true`，异常后由人员在隔离和防坠落条件下处理。
- `recent_id_limit = 32`：记忆最近命令 ID，防止断线后重复抓取。控制器或 Lua 工程重启会清空该内存缓存，重启后未决命令必须人工复核。

### 4.2 Lua 上线前的检查

1. 确认 DobotStudio Pro 版本和 E6 控制器固件支持脚本中的现代 API。
2. 在 DobotStudio Pro 中执行“脚本检查”，确认不存在函数签名或语法错误。
3. 检查 `TCPCreate/TCPStart/TCPRead/TCPWrite/TCPDestroy`。
4. 检查 `MovJ/MovL/CheckMovJ/CheckMovL/Wait`。
5. 检查吸盘使用 `ToolDO` 还是 `DO`。
6. 脚本连接 Python 后，DobotStudio Pro 日志必须显示已连接，Python UI 必须显示“Lua已就绪”。
7. 出现 `config_error` 时禁止运动，先核对 `CFG`。

## 5. 第三处：配置 Dobot Vision Studio 4.1.2

1. Python 是 TCP 服务端，DVS 是 TCP 客户端。
2. DVS 的目标 IP 填 Python 电脑的实际 IPv4；DVS 与 Python 在同一台电脑时可填 `127.0.0.1`。
3. DVS 目标端口填 `network.dvs.port`，默认6001。
4. 优先使用 UTF-8。
5. 每条结果必须以 `\n` 结束。
6. 流程应支持收到 `TRIGGER\n` 后重新采集和计算，然后发送本轮新结果。
7. 推荐发送 `version/task/ok/seq`，便于拒绝错任务、失败结果和重复报文。
8. 量纲字段必须统一，尺寸推荐明确发送 `unit: "mm"`。

实际连通前可先用项目的本机测试验证拆包/粘包逻辑：

```powershell
python -B -m unittest discover -s tests -p "test_dvs_tcp.py" -v
```

## 6. 完整命令清单

### 6.1 每次打开 PowerShell 先执行

```powershell
conda activate D:\anaconda\envs\HKtest
Set-Location D:\CAIM\RAICOM-Project
chcp 65001
python -c "import sys; print(sys.executable); print(sys.version)"
```

解释器必须位于 `D:\anaconda\envs\HKtest`。未激活 Conda 环境时不要直接双击该环境的 `python.exe`，否则可能因 DLL 搜索路径不完整而无输出退出。

### 6.2 不连任何真实硬件：环境检查

```powershell
python main.py --demo --check
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'CPU')"
```

如果缺包，才执行：

```powershell
python -m pip install -r requirements.txt
```

现有 HKtest 中 Torch 是 CUDA 12.1 构建，不要为了安装本项目重装 Torch。

### 6.3 不连任何真实硬件：图形化模拟

```powershell
python main.py --demo
```

界面打开后：

1. 确认右上角显示“安全模拟模式”。
2. 点击“系统自检”。
3. 点击“初始化系统”。
4. 点击“全流程自动运行”。
5. 确认按任务一→任务二→任务三完成，任务二和三各2件，最后确认无当前任务目标。

模拟模式不会连接 D435、DVS、E6 或吸盘。

### 6.4 不连任何真实硬件：无界面全流程

```powershell
python main.py --demo --headless --task all
```

分任务模拟：

```powershell
python main.py --demo --headless --task task1
python main.py --demo --headless --task task2
python main.py --demo --headless --task task3
```

### 6.5 不连任何真实硬件：自动测试

```powershell
python -B -m unittest discover -s tests -v
python tools/test_robot_bridge_loopback.py
```

预期单元测试为 `11 tests ... OK`，桥接测试最后显示 `PASS`。这两条命令只使用本机模拟 TCP，不连真实机械臂。

### 6.6 只连 D435，不运行机械臂

1. D435 使用 USB 3 线和 USB 3 端口。
2. 先用 RealSense Viewer 确认彩色/深度流，然后完全关闭 Viewer，避免占用相机。
3. 查看设备枚举。

```powershell
python -c "import pyrealsense2 as rs; print('RealSense设备数=',len(rs.context().devices))"
```

4. 当机械臂已由 DobotStudio Pro 人工低速移到最终拍照位并停止后，清空桌面，执行：

```powershell
python tools/measure_table_depth.py --frames 60
```

该工具只读相机，不连接也不控制机械臂。

### 6.7 真机参数检查，但不启动硬件

所有文件和参数填完后，先执行：

```powershell
python main.py --real --check
```

这条命令只检查依赖、文件、端口和必填参数，不启动相机服务、Lua TCP 服务和机械臂运动。

只要出现一条“未填写”、“文件不存在”或“端口冲突”，就先修正，不要进入下一步。

### 6.8 真实 DVS 任务一联调，不运行机械臂

前提：真机必填参数已经完整，D435 可以取帧，两个 YOLO 模型可以加载。此阶段不运行 Lua 脚本。

```powershell
python main.py --real
```

在 UI 中：

1. 点击“初始化系统”；初始化只启动相机、模型和 TCP 监听，不会自动运动机械臂。
2. 启动 DVS 工程，确认 UI 显示 DVS 已连接。
3. 只点击“运行任务一”。
4. 确认尺寸、二维码、字符和单位都正确显示。
5. 不得点击任务二、任务三或全流程。

未运行 Lua 时 UI 显示“等待Lua连接”是正常现象。

### 6.9 连接 E6 后的第一步：只测语法、IO 和网络

1. 检查机械臂安装、负载、吸盘、线缆、实体急停和工作区隔离。
2. 在 DobotStudio Pro 中确认用户坐标系和工具坐标系。
3. 使用 DobotStudio Pro 监视/点动功能，人工低速测试吸盘开关和极性。
4. 不放工件，使用 DobotStudio Pro 人工低速验证拍照位、抓取区、落料区和回撤轨迹。
5. 对 `raicom_e6_executor.lua` 执行脚本检查。
6. 启动 Python，点击“初始化系统”，但不运行任务。

```powershell
python main.py --real
```

7. 在 DobotStudio Pro 中运行 Lua 脚本。Lua 只建立 TCP 连接并等待命令，启动本身不会让机械臂移动。
8. UI 必须显示“Lua已就绪”，不能只看到套接字“已连接”。
9. 如果 UI 显示 Lua 配置错误，停止脚本并核对两处配置，不要尝试运行任务。

### 6.10 第二步：低速空载和固定拍照位

当前项目不提供跳过视觉检查的“随意坐标手动发送”按钮，这是为了避免误输入坐标直接运动。

因此低速空载应在 DobotStudio Pro 内完成：

1. 把速度和加速度设为 `5%~10%`。
2. 使用示教/单步确认固定拍照位可达。
3. 按预计的接近→下降→抬升→水平转运→放置→回撤→拍照位顺序单步检查。
4. 此时工作区不放工件，吸盘保持关闭。
5. 任何 `CheckMovJ/CheckMovL` 错误、奇异姿态、线缆干涉或接近边界的轨迹都必须先处理。

### 6.11 第三步：单件低速抓放

1. 只放1个软质测试件。
2. 如果测任务二，临时设置：

```yaml
tasks:
  task2:
    expected_objects: 1
    max_objects: 1
```

3. 确认该工件的类别、路由键和落料点已填好。
4. 在 UI 中只点击“运行任务二”。
5. 一只手保持可以随时按实体急停，人员不得进入运动区。
6. 确认动作实际顺序：回拍照位→重新取帧→识别→下降吸取→原XY抬升→恒Z水平转运→下降→释放→回撤→回拍照位。
7. 确认释放后程序重新取帧，而不是继续使用抓取前的旧图。
8. 测试完成后把 `expected_objects/max_objects` 恢复为现场正式数量。

如果第一件的 Z 还没有用已知高度量块验证，不得直接增加 `press_down_mm`。

### 6.12 第四步：任务二和任务三分别运行

在 UI 中分别点击：

1. “运行任务二”；
2. 重新摆放与复核后，点击“运行任务三”。

每个任务都必须验证：

- 实际数量与 `expected_objects` 一致；
- 只识别本任务工件，不误抓另一任务工件；
- 类别、颜色、置信度、深度和机械臂坐标在 UI 中正确显示；
- 路由键命中正确落料点；
- 最后连续多帧无本任务目标才完成；
- 耗时在10分钟以内。

### 6.13 第五步：所有工件同时出现的全流程

```powershell
python main.py --real
```

1. 点击“初始化系统”。
2. 启动 DVS 工程并确认已连接。
3. 在 DobotStudio Pro 中运行 Lua，确认“Lua已就绪”。
4. 所有任务的工件同时摆在初始区。
5. 点击“全流程自动运行”，之后不再操作界面。
6. 确认固定顺序为任务一→任务二→任务三。
7. 确认任务二结束时不会因桌面仍有任务三工件而误报失败。
8. 使用秒表或 UI 倒计时确认完整运行不超过600秒。

裁判计时时只允许一次启动动作的具体解释必须提前询问裁判：

- 如果允许在计时前打开并初始化系统，计时开始后只点击一次“全流程自动运行”。
- 如果要求计时后才能启动应用，应在比赛前与裁判确认“启动程序”是否允许随后点击初始化和全流程。不要在正式计时时临时猜测。

命令行全自动方式为：

```powershell
python main.py --real --headless --task all
```

该方式会立即初始化，等待 Lua 最多60秒，然后自动执行全流程，但不显示 PyQt5 监控面板。赛题要求交互界面和实时结果显示，所以正式评分优先使用上面的图形界面方式，除非裁判明确允许无界面运行。

## 7. 程序启动顺序速查

### 7.1 无硬件模拟

```text
激活 HKtest → python main.py --demo → 初始化系统 → 全流程自动运行
```

### 7.2 真机联调

```text
检查实体急停/工作区
  → 激活 HKtest
  → python main.py --real --check
  → python main.py --real
  → UI初始化
  → DVS连接
  → DobotStudio Pro运行Lua
  → 确认Lua已就绪
  → 单任务低速
  → 所有工件全流程
```

### 7.3 正式评分前最后一次

```text
核对现场任务书
  → 恢复正式工件数量
  → 确认模型和路由是最终版
  → 确认标定/拍照位/吸盘TCP未变
  → 清空旧日志前先保存调试记录
  → 重启后重做自检和连通检查
  → 计时演练
```

## 8. 日志和故障定位命令

实时查看主日志：

```powershell
Get-Content .\logs\raicom.log -Encoding UTF8 -Tail 100 -Wait
```

查看最后结果：

```powershell
Get-Content .\logs\results.jsonl -Encoding UTF8 -Tail 20
```

查看端口占用：

```powershell
Get-NetTCPConnection -LocalPort 6001,2006 -ErrorAction SilentlyContinue | Format-Table LocalAddress,LocalPort,State,OwningProcess
```

查看占用这两个端口的对应进程：

```powershell
Get-NetTCPConnection -LocalPort 6001,2006 -ErrorAction SilentlyContinue |
    Select-Object -ExpandProperty OwningProcess -Unique |
    ForEach-Object { Get-Process -Id $_ }
```

任务失败时优先看：

1. UI 最后的机械臂中文阶段；
2. `logs/raicom.log` 最后100行；
3. `logs/results.jsonl` 是否已记录某件抓放完成；
4. DobotStudio Pro 中 `[RAICOM-E6]` 日志和机械臂报警；
5. DVS 实际发送的完整原始报文。

断线、命令超时或机械臂异常后，禁止改用新 ID 盲目重发同一次抓取。先确认机械臂位置、是否持件、吸盘状态和日志，再按现场安全流程人工恢复。

## 9. 比赛前最终勾选表

### 9.1 环境和文件

- [ ] `conda activate D:\anaconda\envs\HKtest` 后解释器路径正确。
- [ ] `python main.py --demo --check` 通过。
- [ ] 11项单元测试和 Lua 桥接回环通过。
- [ ] `models/task2.pt` 和 `models/task3.pt` 是现场最终权重。
- [ ] `CaliMatrixData.yaml` 是现场重新标定结果，不是 `.example.yaml`。
- [ ] 离线保存了基础权重、中文字体、安装包和驱动。

### 9.2 现场任务书

- [ ] 任务一测量字段、单位、容差、二维码和字符要求已同步到 DVS 和 `required_fields`。
- [ ] 任务二/三的实际数量已写入 `expected_objects/max_objects`。
- [ ] 类别名、颜色、形状、已知图案和任务过滤已确认。
- [ ] 任务内选择顺序和分类路由已确认。
- [ ] 所有实际路由都有已示教的落料点。

### 9.3 D435 和标定

- [ ] D435 能被枚举，RealSense Viewer 已关闭。
- [ ] 分辨率、帧率和标定时一致。
- [ ] 固定拍照位未变。
- [ ] `table_depth_mm` 是最终拍照位的空台深度。
- [ ] `robot_table_touch_z_mm` 是同一工具/用户坐标系下的贴台 Z。
- [ ] 用已知高度量块验证了 Z 公式。
- [ ] 至少3个分散点的 EIH XY 误差通过现场容差。
- [ ] 矩阵方向、m/mm 单位和 `zyx` 姿态顺序已实测确认。

### 9.4 E6、吸盘和网络

- [ ] 用户坐标系、工具坐标系、吸盘 TCP 和负载已设置。
- [ ] 拍照位、抓取姿态和工作空间已低速验证。
- [ ] 接近、抬升、放置下降和释放回撤距离已验证。
- [ ] `ToolDO/DO`、IO号、极性、吸取/释放等待已实测。
- [ ] Python 和 Lua 的所有对应字段已逐项核对。
- [ ] Python电脑IP、DVS端口6001、Lua端口2006和防火墙已验证。
- [ ] DobotStudio Pro 中 Lua 脚本检查通过，UI 显示“Lua已就绪”。
- [ ] 实体急停、人员隔离、线缆和防坠落措施已确认。

### 9.5 最终运行

- [ ] `python main.py --real --check` 无任何未通过项。
- [ ] 单件低速抓放通过。
- [ ] 任务二单任务通过。
- [ ] 任务三单任务通过。
- [ ] 所有工件同时出现的1→2→3全流程通过。
- [ ] 完整流程在600秒以内。
- [ ] 已与裁判确认计时时允许的唯一启动方式。

## 10. 最容易填错的十个地方

1. `robot.lua.pc_server_ip` 填的是 Python 电脑 IP，不是 E6 IP。
2. `listen_host` 保持 `0.0.0.0`，DVS/Lua 的目标 IP 才填 Python 电脑 IP。
3. `CaliMatrixData.example.yaml` 不能用于真机。
4. `CamToTipTransform` 名字不能单独证明矩阵方向，必须多点验证。
5. `table_depth_mm` 必须在最终拍照位、抓取区空台面时测量，不能拿它计算任务三放置高度。
6. 贴台 Z、拍照位、EIH 和真机运行必须使用同一吸盘 TCP 和坐标系。
7. `down_mm` 只属于任务二；任务三使用目标上方实时深度识别得到的绝对释放 Z。
8. 任务路由键必须与 YOLO 类别/颜色/形状名对应，否则会落到 `default`。
9. Python 和 Lua 两处速度、坐标系、姿态、吸盘参数必须同步。
10. 软件停止不是实体急停；运动中异常时不要通过关闭窗口或拔网线处理。
