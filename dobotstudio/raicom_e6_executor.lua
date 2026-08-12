--[[
===============================================================================
2026 睿抗机器人开发者大赛 · Dobot Magician E6 执行脚本

适用接口：DobotStudio Pro V4.5 脚本指令
通信方向：本脚本是 TCP 客户端，Python 主程序是 TCP 服务端
协议格式：UTF-8 NDJSON（一行一个扁平 JSON 对象，以 \n 结束）

重要安全说明：
1. 本脚本中的“停止后续任务”不是物理急停，不能替代 E6 实体急停开关。
2. 所有标有“★现场必填”的参数必须在低速、无工件条件下逐项确认。
3. 脚本故意不使用参考文件中的 SetIODO、TCPConnect、TCPRecv、TCPSend，
   也不使用未在 E6/V4.5 指令表中确认的 Sync()。
4. 正式使用的 API 为 TCPCreate/TCPStart/TCPRead/TCPWrite/TCPDestroy、
   MovJ/MovL、CheckMovJ/CheckMovL、Wait、DO/ToolDO。
5. 指令签名按 DobotStudio Pro V4.5 手册编写；比赛前仍须在实际安装版本中使用
   “脚本检查/低速单步”确认。特别要点：Wait(check_str, timeout_ms) 返回布尔值，
   E6 末端 DI/DO 各 2 路、底座 DI/DO 各 16 路，但实际接线与输出极性必须实测。
===============================================================================
--]]

local CFG = {
    -- ----------------------------------------------------------------------
    -- ★现场必填：比赛电脑有线网卡 IP。不是机器人 IP，也不能填 127.0.0.1。
    -- E6 默认 LAN1 常为 192.168.5.1，电脑可设同网段未占用地址，例如 .10。
    -- ----------------------------------------------------------------------
    python_ip = "192.168.5.100",
    python_port = 2006,
    connect_timeout_s = 5,
    write_timeout_s = 5,
    reconnect_delay_ms = 2000,
    max_line_bytes = 65536,

    -- ★现场必填：必须与 EIH 标定、示教点和吸盘 TCP 使用的坐标系一致。
    user_index = 0,
    tool_index = 0,

    -- ★现场必填：固定拍照位 [X,Y,Z,Rx,Ry,Rz]，不能用 Home() 代替。
    photo_pose = {
        x = 160.0, y = -60.0, z = 430.0,
        rx = 180.0, ry = 0, rz = 0,
    },

    -- ★现场必填：吸盘竖直抓取时的姿态。
    pick_orientation = {rx = 180.0, ry = 0, rz = 0},

    -- ★现场必填：按当前用户坐标系低速示教得到的软件工作空间。
    workspace = {
        x_min = 155.0, x_max = 290.0,
        y_min = -150.0, y_max = 135.0,
        z_min = 100.0, z_max = 435.0,
    },

    -- 运动参数。下列初值与 settings.yaml 一致；首次真机应在两处同步调低后再单步。
    motion = {
        approach_mm = 40.0,       -- 抓取点上方的接近距离
        pick_lift_mm = 80.0,      -- 吸取后保持 X/Y 不变的抬升距离
        release_retract_mm = 80.0,-- 释放后保持放置 X/Y 不变的回撤距离
        -- 现场实测 Z=417 已不可达：原抓取 XY 只升至 410，随后保持 Z=410 仅移动 XY。
        place_inspection_z_mm = 410.0,
        z_up_sign = 1,            -- 标准用户坐标 Z 向上填 1；若现场相反填 -1
        travel_v = 20,            -- 运动速度安全上限比例 (0,100]
        pick_v = 10,              -- 直线接近/离开速度安全上限比例 (0,100]
        acceleration = 20,        -- 加速度比例 (0,100]
        settle_ms = 300,          -- 到位后短暂稳定时间
    },

    -- ----------------------------------------------------------------------
    -- ★现场必填：吸盘输出。
    -- api 只能填 "ToolDO"（末端输出）或 "DO"（底座输出）。
    -- E6 末端 ToolDO 只有 1/2；底座 DO 为 1~16。
    -- 必须点动确认 on_level/off_level 极性。
    -- ----------------------------------------------------------------------
    vacuum = {
        api = "ToolDO",
        io_index = 1,
        on_level = 1,
        off_level = 0,
        suction_wait_ms = 700,
        release_wait_ms = 400,

        -- 只有安装真空压力开关时才启用；不能用 DO 状态冒充“已经吸住”。
        feedback_enabled = false,
        feedback_di_index = nil,
        feedback_ok_level = 1,
        feedback_timeout_ms = 1500,

        -- 抓起后发生运动异常时保持真空，防止工件立即坠落，由操作员处理。
        keep_on_after_pick_error = true,
    },

    -- Lua 内存中保留的最近终态 ID。断线后 Python 以同一 ID 重发时不会重复抓取。
    -- 控制器重启或重新启动工程会清空该缓存，比赛程序应视为需要人工复核。
    recent_id_limit = 32,
}

local PROTOCOL_VERSION = 1
local recent_by_id = {}
local recent_order = {}
-- 任务三分成“持件观察”和“按视觉 Z 释放”两条命令。两条命令之间必须在
-- Lua 内保留持件事务，禁止回拍照位或开始另一件抓取。
local active_hold = nil

local function log(message)
    Log("[RAICOM-E6] " .. tostring(message))
end

local function is_finite(value)
    return type(value) == "number"
        and value == value
        and value ~= math.huge
        and value ~= -math.huge
end

local function is_integer(value)
    return is_finite(value) and value == math.floor(value)
end

local function level_constant(level)
    if level == 1 then
        return ON
    end
    return OFF
end

local function json_escape(value)
    local text = tostring(value)
    text = text:gsub("\\", "\\\\")
    text = text:gsub('"', '\\"')
    text = text:gsub("\b", "\\b")
    text = text:gsub("\f", "\\f")
    text = text:gsub("\n", "\\n")
    text = text:gsub("\r", "\\r")
    text = text:gsub("\t", "\\t")
    text = text:gsub("[%z\1-\31]", function(char)
        return string.format("\\u%04x", string.byte(char))
    end)
    return '"' .. text .. '"'
end

local function json_value(value)
    local value_type = type(value)
    if value_type == "string" then
        return json_escape(value)
    elseif value_type == "number" then
        if not is_finite(value) then
            error("JSON cannot encode NaN/Inf")
        end
        return string.format("%.10g", value)
    elseif value_type == "boolean" then
        return value and "true" or "false"
    elseif value == nil then
        return "null"
    end
    error("unsupported JSON type: " .. value_type)
end

local function json_encode_object(fields)
    local parts = {}
    for key, value in pairs(fields) do
        if value ~= nil then
            parts[#parts + 1] = json_escape(key) .. ":" .. json_value(value)
        end
    end
    return "{" .. table.concat(parts, ",") .. "}"
end

-- 协议只接收 Python 生成的扁平 JSON。id/cmd 均限制为安全 ASCII，因而不需要
-- 在控制器上实现完整 JSON AST；数值仍交给 tonumber 严格转换并检查 finite。
local function json_get_string(line, key)
    local pattern = '"' .. key .. '"%s*:%s*"([^"\\]*)"'
    return line:match(pattern)
end

local function json_get_number(line, key)
    local pattern = '"' .. key .. '"%s*:%s*([^,%}%s]+)'
    local token = line:match(pattern)
    if token == nil then
        return nil
    end
    local value = tonumber(token)
    if not is_finite(value) then
        return nil
    end
    return value
end

local function json_get_boolean(line, key)
    local pattern = '"' .. key .. '"%s*:%s*([%a]+)'
    local token = line:match(pattern)
    if token == "true" then
        return true
    elseif token == "false" then
        return false
    end
    return nil
end

local function write_json(socket, fields)
    local ok, encoded = pcall(json_encode_object, fields)
    if not ok then
        log("JSON encode failed: " .. tostring(encoded))
        return false
    end
    local err = TCPWrite(socket, encoded .. "\n", CFG.write_timeout_s)
    if err ~= 0 then
        log("TCPWrite failed, err=" .. tostring(err))
        return false
    end
    return true
end

local function send_status(socket, command_id, status, extra)
    local fields = {
        v = PROTOCOL_VERSION,
        id = command_id,
        status = status,
    }
    if extra ~= nil then
        for key, value in pairs(extra) do
            fields[key] = value
        end
    end
    return write_json(socket, fields), fields
end

local function remember_terminal(command_id, command, request_line, fields)
    if recent_by_id[command_id] == nil then
        recent_order[#recent_order + 1] = command_id
    end
    recent_by_id[command_id] = {
        command = command,
        request_line = request_line,
        terminal = fields,
    }
    while #recent_order > CFG.recent_id_limit do
        local oldest = table.remove(recent_order, 1)
        recent_by_id[oldest] = nil
    end
end

local function valid_command_id(command_id)
    return type(command_id) == "string"
        and #command_id >= 1
        and #command_id <= 64
        and command_id:match("^[A-Za-z0-9_.:%-]+$") ~= nil
end

local function validate_static_config()
    if type(CFG.python_ip) ~= "string" or CFG.python_ip == "" then
        return false, "python_ip is empty"
    end
    if not is_integer(CFG.python_port) or CFG.python_port < 1024 or CFG.python_port > 59999 then
        return false, "python_port is invalid"
    end
    if not is_integer(CFG.max_line_bytes) or CFG.max_line_bytes < 256 then
        return false, "max_line_bytes must be at least 256"
    end
    if not is_integer(CFG.recent_id_limit) or CFG.recent_id_limit < 1 then
        return false, "recent_id_limit must be positive"
    end
    if not is_integer(CFG.user_index) or CFG.user_index < 0 or CFG.user_index > 9 then
        return false, "user_index must be 0..9"
    end
    if not is_integer(CFG.tool_index) or CFG.tool_index < 0 or CFG.tool_index > 9 then
        return false, "tool_index must be 0..9"
    end

    local photo = CFG.photo_pose
    for _, key in ipairs({"x", "y", "z", "rx", "ry", "rz"}) do
        if not is_finite(photo[key]) then
            return false, "photo_pose." .. key .. " is not configured"
        end
    end
    local orientation = CFG.pick_orientation
    for _, key in ipairs({"rx", "ry", "rz"}) do
        if not is_finite(orientation[key]) then
            return false, "pick_orientation." .. key .. " is not configured"
        end
    end

    local ws = CFG.workspace
    for _, axis in ipairs({"x", "y", "z"}) do
        local minimum = ws[axis .. "_min"]
        local maximum = ws[axis .. "_max"]
        if not is_finite(minimum) or not is_finite(maximum) or minimum >= maximum then
            return false, "workspace " .. axis .. " bounds are invalid"
        end
    end
    if photo.x < ws.x_min or photo.x > ws.x_max
        or photo.y < ws.y_min or photo.y > ws.y_max
        or photo.z < ws.z_min or photo.z > ws.z_max then
        return false, "photo_pose is outside workspace"
    end

    local motion = CFG.motion
    if not is_finite(motion.approach_mm) or motion.approach_mm <= 0 then
        return false, "motion.approach_mm must be positive"
    end
    if not is_finite(motion.pick_lift_mm) or motion.pick_lift_mm <= 0 then
        return false, "motion.pick_lift_mm must be positive"
    end
    if not is_finite(motion.release_retract_mm) or motion.release_retract_mm <= 0 then
        return false, "motion.release_retract_mm must be positive"
    end
    if not is_finite(motion.place_inspection_z_mm)
        or motion.place_inspection_z_mm < CFG.workspace.z_min
        or motion.place_inspection_z_mm > CFG.workspace.z_max then
        return false, "motion.place_inspection_z_mm is outside workspace"
    end
    if motion.z_up_sign ~= 1 and motion.z_up_sign ~= -1 then
        return false, "motion.z_up_sign must be 1 or -1"
    end
    for _, key in ipairs({"travel_v", "pick_v", "acceleration"}) do
        local value = motion[key]
        if not is_finite(value) or value <= 0 or value > 100 then
            return false, "motion." .. key .. " must be in (0,100]"
        end
    end
    if not is_finite(motion.settle_ms) or motion.settle_ms < 0 then
        return false, "motion.settle_ms must be non-negative"
    end

    local vacuum = CFG.vacuum
    if vacuum.api ~= "ToolDO" and vacuum.api ~= "DO" then
        return false, "vacuum.api must be ToolDO or DO"
    end
    if not is_integer(vacuum.io_index) then
        return false, "vacuum.io_index is not configured"
    end
    if vacuum.api == "ToolDO" and (vacuum.io_index < 1 or vacuum.io_index > 2) then
        return false, "ToolDO index must be 1 or 2"
    end
    if vacuum.api == "DO" and (vacuum.io_index < 1 or vacuum.io_index > 16) then
        return false, "E6 base DO index must be 1..16"
    end
    if (vacuum.on_level ~= 0 and vacuum.on_level ~= 1)
        or (vacuum.off_level ~= 0 and vacuum.off_level ~= 1)
        or vacuum.on_level == vacuum.off_level then
        return false, "vacuum output levels are invalid"
    end
    if not is_finite(vacuum.suction_wait_ms) or vacuum.suction_wait_ms < 0
        or not is_finite(vacuum.release_wait_ms) or vacuum.release_wait_ms < 0 then
        return false, "vacuum wait time is invalid"
    end
    if vacuum.feedback_enabled then
        if not is_integer(vacuum.feedback_di_index) or vacuum.feedback_di_index < 1 then
            return false, "vacuum feedback DI is invalid"
        end
        if vacuum.api == "ToolDO" and vacuum.feedback_di_index > 2 then
            return false, "E6 ToolDI index must be 1 or 2"
        end
        if vacuum.api == "DO" and vacuum.feedback_di_index > 16 then
            return false, "E6 base DI index must be 1..16"
        end
        if vacuum.feedback_ok_level ~= 0 and vacuum.feedback_ok_level ~= 1 then
            return false, "vacuum feedback level is invalid"
        end
        if not is_finite(vacuum.feedback_timeout_ms) or vacuum.feedback_timeout_ms <= 0 then
            return false, "vacuum feedback timeout is invalid"
        end
    end
    return true, ""
end

local function make_pose(x, y, z, rx, ry, rz)
    return {pose = {x, y, z, rx, ry, rz}}
end

local function point_in_workspace(x, y, z)
    local ws = CFG.workspace
    return is_finite(x) and is_finite(y) and is_finite(z)
        and x >= ws.x_min and x <= ws.x_max
        and y >= ws.y_min and y <= ws.y_max
        and z >= ws.z_min and z <= ws.z_max
end

local function pose_in_workspace(pose)
    return point_in_workspace(pose.x, pose.y, pose.z)
end

local function checked_movj(pose, velocity)
    if not pose_in_workspace(pose) then
        return false, "OUT_OF_WORKSPACE"
    end
    local point = make_pose(pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz)
    local options = {
        user = CFG.user_index,
        tool = CFG.tool_index,
        a = CFG.motion.acceleration,
        v = velocity,
        cp = 0,
    }
    local status = CheckMovJ(point, options)
    if status ~= 0 then
        return false, "CHECK_MOVJ_" .. tostring(status)
    end
    MovJ(point, options)
    -- V4.5 Wait 会在上一条命令完成后再开始等待，因此这里同时承担到位屏障。
    Wait(1)
    return true, ""
end

local function checked_movl(pose, velocity)
    if not pose_in_workspace(pose) then
        return false, "OUT_OF_WORKSPACE"
    end
    local point = make_pose(pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz)
    local options = {
        user = CFG.user_index,
        tool = CFG.tool_index,
        a = CFG.motion.acceleration,
        v = velocity,
        cp = 0,
    }
    local status = CheckMovL(point, options)
    if status ~= 0 then
        return false, "CHECK_MOVL_" .. tostring(status)
    end
    MovL(point, options)
    Wait(1)
    return true, ""
end

local function set_vacuum(enabled)
    local vacuum = CFG.vacuum
    local level = enabled and vacuum.on_level or vacuum.off_level
    if vacuum.api == "ToolDO" then
        ToolDO(vacuum.io_index, level_constant(level))
    else
        DO(vacuum.io_index, level_constant(level))
    end
    Wait(enabled and vacuum.suction_wait_ms or vacuum.release_wait_ms)

    if enabled and vacuum.feedback_enabled then
        local expression
        if vacuum.api == "ToolDO" then
            expression = "ToolDI(" .. tostring(vacuum.feedback_di_index) .. ") == "
                .. tostring(vacuum.feedback_ok_level)
        else
            expression = "DI(" .. tostring(vacuum.feedback_di_index) .. ") == "
                .. tostring(vacuum.feedback_ok_level)
        end
        if not Wait(expression, vacuum.feedback_timeout_ms) then
            return false, "VACUUM_TIMEOUT"
        end
    end
    return true, ""
end

local function photo_pose()
    return {
        x = CFG.photo_pose.x,
        y = CFG.photo_pose.y,
        z = CFG.photo_pose.z,
        rx = CFG.photo_pose.rx,
        ry = CFG.photo_pose.ry,
        rz = CFG.photo_pose.rz,
    }
end

local function almost_equal(left, right)
    return is_finite(left) and is_finite(right) and math.abs(left - right) <= 0.0001
end

-- Python 会把 settings.yaml 中已通过安全校验的现场参数显式放入每条运动命令；
-- Lua 再与本脚本 CFG 比较。两处不一致时拒绝动作，避免改错文件后静默运行。
local function validate_request_context(line)
    local user = json_get_number(line, "user")
    local tool = json_get_number(line, "tool")
    if not is_integer(user) or user ~= CFG.user_index then
        return false, "CONFIG_MISMATCH_USER"
    end
    if not is_integer(tool) or tool ~= CFG.tool_index then
        return false, "CONFIG_MISMATCH_TOOL"
    end

    local photo = CFG.photo_pose
    local photo_fields = {
        photo_x = photo.x, photo_y = photo.y, photo_z = photo.z,
        photo_rx = photo.rx, photo_ry = photo.ry, photo_rz = photo.rz,
    }
    for key, expected in pairs(photo_fields) do
        if not almost_equal(json_get_number(line, key), expected) then
            return false, "CONFIG_MISMATCH_" .. string.upper(key)
        end
    end

    local ws = CFG.workspace
    local workspace_fields = {
        workspace_x_min = ws.x_min, workspace_x_max = ws.x_max,
        workspace_y_min = ws.y_min, workspace_y_max = ws.y_max,
        workspace_z_min = ws.z_min, workspace_z_max = ws.z_max,
    }
    for key, expected in pairs(workspace_fields) do
        if not almost_equal(json_get_number(line, key), expected) then
            return false, "CONFIG_MISMATCH_" .. string.upper(key)
        end
    end

    local travel_v = json_get_number(line, "travel_v")
    local pick_v = json_get_number(line, "pick_v")
    local accel = json_get_number(line, "accel")
    local settle_ms = json_get_number(line, "settle_ms")
    if not is_finite(travel_v) or travel_v <= 0 or travel_v > 100 then
        return false, "BAD_TRAVEL_V"
    end
    if not is_finite(pick_v) or pick_v <= 0 or pick_v > 100 then
        return false, "BAD_PICK_V"
    end
    if not is_finite(accel) or accel <= 0 or accel > 100 then
        return false, "BAD_ACCEL"
    end
    if not is_finite(settle_ms) or settle_ms < 0 then
        return false, "BAD_SETTLE_MS"
    end
    -- Lua CFG 是最终安全上限；Python 可以请求更慢，但不能请求更快。
    if travel_v > CFG.motion.travel_v
        or pick_v > CFG.motion.pick_v
        or accel > CFG.motion.acceleration then
        return false, "PYTHON_MOTION_EXCEEDS_LUA_LIMIT"
    end
    if not almost_equal(settle_ms, CFG.motion.settle_ms) then
        return false, "CONFIG_MISMATCH_SETTLE_MS"
    end

    local vacuum = CFG.vacuum
    if json_get_string(line, "vacuum_api") ~= vacuum.api then
        return false, "CONFIG_MISMATCH_VACUUM_API"
    end
    if json_get_number(line, "vacuum_io") ~= vacuum.io_index then
        return false, "CONFIG_MISMATCH_VACUUM_IO"
    end
    if json_get_number(line, "vacuum_on_level") ~= vacuum.on_level
        or json_get_number(line, "vacuum_off_level") ~= vacuum.off_level then
        return false, "CONFIG_MISMATCH_VACUUM_LEVEL"
    end
    if not almost_equal(
        json_get_number(line, "vacuum_suction_wait_ms"), vacuum.suction_wait_ms
    ) or not almost_equal(
        json_get_number(line, "vacuum_release_wait_ms"), vacuum.release_wait_ms
    ) then
        return false, "CONFIG_MISMATCH_VACUUM_WAIT"
    end

    local feedback_enabled = json_get_boolean(line, "vacuum_feedback_enabled")
    if feedback_enabled ~= vacuum.feedback_enabled then
        return false, "CONFIG_MISMATCH_VACUUM_FEEDBACK"
    end
    if feedback_enabled then
        if json_get_number(line, "vacuum_feedback_di") ~= vacuum.feedback_di_index
            or json_get_number(line, "vacuum_feedback_level") ~= vacuum.feedback_ok_level
            or not almost_equal(
                json_get_number(line, "vacuum_feedback_timeout_ms"),
                vacuum.feedback_timeout_ms
            ) then
            return false, "CONFIG_MISMATCH_VACUUM_FEEDBACK"
        end
    end
    return true, ""
end

local function go_photo(socket, command_id)
    send_status(socket, command_id, "running", {phase = "return_photo"})
    local ok, code = checked_movj(photo_pose(), CFG.motion.travel_v)
    if not ok then
        return false, code
    end
    Wait(CFG.motion.settle_ms)
    return true, ""
end

local function parse_pick_job(line)
    local job = {
        pick_x = json_get_number(line, "pick_x"),
        pick_y = json_get_number(line, "pick_y"),
        pick_z = json_get_number(line, "pick_z"),
        pick_rx = json_get_number(line, "pick_rx"),
        pick_ry = json_get_number(line, "pick_ry"),
        pick_rz = json_get_number(line, "pick_rz"),
        place_x = json_get_number(line, "place_x"),
        place_y = json_get_number(line, "place_y"),
        place_z = json_get_number(line, "place_z"),
        place_rx = json_get_number(line, "place_rx"),
        place_ry = json_get_number(line, "place_ry"),
        place_rz = json_get_number(line, "place_rz"),
        approach_z = json_get_number(line, "approach_z"),
        transfer_z = json_get_number(line, "transfer_z"),
        retract_z = json_get_number(line, "retract_z"),
        place_down_mm = json_get_number(line, "place_down_mm"),
    }
    for _, key in ipairs({
        "pick_x", "pick_y", "pick_z", "pick_rx", "pick_ry", "pick_rz",
        "place_x", "place_y", "place_z", "place_rx", "place_ry", "place_rz",
        "approach_z", "transfer_z", "retract_z", "place_down_mm",
    }) do
        local value = job[key]
        if not is_finite(value) then
            return nil, "BAD_NUMBER_" .. key
        end
    end
    if job.place_down_mm <= 0 then
        return nil, "BAD_PLACE_DOWN"
    end

    local orientation = CFG.pick_orientation
    if not almost_equal(job.pick_rx, orientation.rx)
        or not almost_equal(job.pick_ry, orientation.ry)
        or not almost_equal(job.pick_rz, orientation.rz)
        or not almost_equal(job.place_rx, orientation.rx)
        or not almost_equal(job.place_ry, orientation.ry)
        or not almost_equal(job.place_rz, orientation.rz) then
        return nil, "CONFIG_MISMATCH_ORIENTATION"
    end

    local sign = CFG.motion.z_up_sign
    local expected_approach = job.pick_z + sign * CFG.motion.approach_mm
    local expected_transfer = job.pick_z + sign * CFG.motion.pick_lift_mm
    local expected_place = job.transfer_z - sign * job.place_down_mm
    local expected_retract = job.place_z + sign * CFG.motion.release_retract_mm
    if not almost_equal(job.approach_z, expected_approach) then
        return nil, "CONFIG_MISMATCH_APPROACH_Z"
    end
    if not almost_equal(job.transfer_z, expected_transfer) then
        return nil, "CONFIG_MISMATCH_TRANSFER_Z"
    end
    if not almost_equal(job.place_z, expected_place) then
        return nil, "CONFIG_MISMATCH_PLACE_Z"
    end
    if not almost_equal(job.retract_z, expected_retract) then
        return nil, "CONFIG_MISMATCH_RETRACT_Z"
    end
    return job, ""
end

local function build_job_poses(job)
    local poses = {
        pick_approach = {
            x = job.pick_x, y = job.pick_y, z = job.approach_z,
            rx = job.pick_rx, ry = job.pick_ry, rz = job.pick_rz,
        },
        pick = {
            x = job.pick_x, y = job.pick_y, z = job.pick_z,
            rx = job.pick_rx, ry = job.pick_ry, rz = job.pick_rz,
        },
        pick_lift = {
            x = job.pick_x, y = job.pick_y, z = job.transfer_z,
            rx = job.pick_rx, ry = job.pick_ry, rz = job.pick_rz,
        },
        place_transfer = {
            x = job.place_x, y = job.place_y, z = job.transfer_z,
            rx = job.place_rx, ry = job.place_ry, rz = job.place_rz,
        },
        place = {
            x = job.place_x, y = job.place_y, z = job.place_z,
            rx = job.place_rx, ry = job.place_ry, rz = job.place_rz,
        },
        place_retract = {
            x = job.place_x, y = job.place_y, z = job.retract_z,
            rx = job.place_rx, ry = job.place_ry, rz = job.place_rz,
        },
    }
    for name, pose in pairs(poses) do
        if not pose_in_workspace(pose) then
            return nil, "OUT_OF_WORKSPACE_" .. name
        end
    end
    return poses, ""
end

local function pick_and_place(socket, command_id, line)
    local job, parse_error = parse_pick_job(line)
    if job == nil then
        return false, parse_error, false
    end
    local poses, pose_error = build_job_poses(job)
    if poses == nil then
        return false, pose_error, false
    end

    local holding_part = false
    local ok
    local code

    send_status(socket, command_id, "running", {phase = "above_pick"})
    ok, code = checked_movj(poses.pick_approach, CFG.motion.travel_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "descend_pick"})
    ok, code = checked_movl(poses.pick, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "vacuum_on"})
    ok, code = set_vacuum(true)
    if not ok then
        set_vacuum(false)
        -- 真空未建立时不携带工件，尝试沿原路径退出。
        checked_movl(poses.pick_approach, CFG.motion.pick_v)
        go_photo(socket, command_id)
        return false, code, false
    end
    holding_part = true

    -- 用户要求：吸住后 X/Y 不变，仅抬高 Z。
    send_status(socket, command_id, "running", {phase = "lift_pick"})
    ok, code = checked_movl(poses.pick_lift, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    -- 用户要求：保持抬升后的 Z 不变，只将 X/Y 移到放置点。
    send_status(socket, command_id, "running", {phase = "transfer_xy"})
    -- 必须使用直线运动；MovJ 只能保证终点 Z 相同，关节插补途中并不保持恒定 Z。
    ok, code = checked_movl(poses.place_transfer, CFG.motion.travel_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "descend_place"})
    ok, code = checked_movl(poses.place, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "vacuum_off"})
    ok, code = set_vacuum(false)
    if not ok then return false, code, holding_part end
    holding_part = false

    send_status(socket, command_id, "running", {phase = "retract_place"})
    ok, code = checked_movl(poses.place_retract, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    ok, code = go_photo(socket, command_id)
    if not ok then return false, code, holding_part end
    return true, "", holding_part
end

local function parse_stack_pick_job(line)
    local job = {
        pick_x = json_get_number(line, "pick_x"),
        pick_y = json_get_number(line, "pick_y"),
        pick_z = json_get_number(line, "pick_z"),
        object_height_mm = json_get_number(line, "object_height_mm"),
        place_x = json_get_number(line, "place_x"),
        place_y = json_get_number(line, "place_y"),
        inspection_x = json_get_number(line, "inspection_x"),
        inspection_y = json_get_number(line, "inspection_y"),
        inspection_z = json_get_number(line, "inspection_z"),
        pick_rx = json_get_number(line, "pick_rx"),
        pick_ry = json_get_number(line, "pick_ry"),
        pick_rz = json_get_number(line, "pick_rz"),
        inspection_rx = json_get_number(line, "inspection_rx"),
        inspection_ry = json_get_number(line, "inspection_ry"),
        inspection_rz = json_get_number(line, "inspection_rz"),
        approach_z = json_get_number(line, "approach_z"),
        lift_z = json_get_number(line, "lift_z"),
    }
    for _, key in ipairs({
        "pick_x", "pick_y", "pick_z", "object_height_mm", "place_x", "place_y",
        "inspection_x", "inspection_y", "inspection_z",
        "pick_rx", "pick_ry", "pick_rz",
        "inspection_rx", "inspection_ry", "inspection_rz", "approach_z", "lift_z",
    }) do
        local value = job[key]
        if not is_finite(value) then
            return nil, "BAD_NUMBER_" .. key
        end
    end
    if job.object_height_mm <= 0 then
        return nil, "BAD_OBJECT_HEIGHT"
    end
    if not almost_equal(job.inspection_z, CFG.motion.place_inspection_z_mm) then
        return nil, "CONFIG_MISMATCH_PLACE_INSPECTION_Z"
    end

    local orientation = CFG.pick_orientation
    for _, prefix in ipairs({"pick", "inspection"}) do
        if not almost_equal(job[prefix .. "_rx"], orientation.rx)
            or not almost_equal(job[prefix .. "_ry"], orientation.ry)
            or not almost_equal(job[prefix .. "_rz"], orientation.rz) then
            return nil, "CONFIG_MISMATCH_ORIENTATION"
        end
    end
    local sign = CFG.motion.z_up_sign
    if not almost_equal(job.approach_z, job.pick_z + sign * CFG.motion.approach_mm) then
        return nil, "CONFIG_MISMATCH_APPROACH_Z"
    end
    if not almost_equal(job.lift_z, job.pick_z + sign * CFG.motion.pick_lift_mm) then
        return nil, "CONFIG_MISMATCH_LIFT_Z"
    end
    return job, ""
end

local function build_stack_pick_poses(job)
    local poses = {
        pick_approach = {
            x = job.pick_x, y = job.pick_y, z = job.approach_z,
            rx = job.pick_rx, ry = job.pick_ry, rz = job.pick_rz,
        },
        pick = {
            x = job.pick_x, y = job.pick_y, z = job.pick_z,
            rx = job.pick_rx, ry = job.pick_ry, rz = job.pick_rz,
        },
        pick_lift = {
            x = job.pick_x, y = job.pick_y, z = job.lift_z,
            rx = job.pick_rx, ry = job.pick_ry, rz = job.pick_rz,
        },
        inspection_raise = {
            x = job.pick_x, y = job.pick_y, z = job.inspection_z,
            rx = job.inspection_rx, ry = job.inspection_ry, rz = job.inspection_rz,
        },
        inspection = {
            x = job.inspection_x, y = job.inspection_y, z = job.inspection_z,
            rx = job.inspection_rx, ry = job.inspection_ry, rz = job.inspection_rz,
        },
    }
    for name, pose in pairs(poses) do
        if not pose_in_workspace(pose) then
            return nil, "OUT_OF_WORKSPACE_" .. name
        end
    end
    return poses, ""
end

local function pick_to_inspection(socket, command_id, job, poses)
    local holding_part = false
    local ok
    local code

    send_status(socket, command_id, "running", {phase = "above_pick"})
    ok, code = checked_movj(poses.pick_approach, CFG.motion.travel_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "descend_pick"})
    ok, code = checked_movl(poses.pick, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "vacuum_on"})
    ok, code = set_vacuum(true)
    if not ok then
        set_vacuum(false)
        checked_movl(poses.pick_approach, CFG.motion.pick_v)
        go_photo(socket, command_id)
        return false, code, false
    end
    holding_part = true
    active_hold = {
        hold_id = command_id,
        ready = false,
        place_x = job.place_x,
        place_y = job.place_y,
        inspection_x = job.inspection_x,
        inspection_y = job.inspection_y,
        inspection_z = job.inspection_z,
    }

    send_status(socket, command_id, "running", {phase = "lift_pick"})
    ok, code = checked_movl(poses.pick_lift, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    -- 先在原抓取 XY 竖直升到现场确认可达的观察高度 410，再保持 Z 不变
    -- 只水平移动到相机位于放置点上方的观察 XY；工件随吸盘横向让开。
    send_status(socket, command_id, "running", {phase = "raise_inspection"})
    ok, code = checked_movl(poses.inspection_raise, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "at_place_inspection"})
    ok, code = checked_movl(poses.inspection, CFG.motion.travel_v)
    if not ok then return false, code, holding_part end
    Wait(CFG.motion.settle_ms)
    active_hold.ready = true
    return true, "", holding_part
end

local function parse_stack_release_job(line)
    local job = {
        place_x = json_get_number(line, "place_x"),
        place_y = json_get_number(line, "place_y"),
        place_z = json_get_number(line, "place_z"),
        inspection_x = json_get_number(line, "inspection_x"),
        inspection_y = json_get_number(line, "inspection_y"),
        inspection_z = json_get_number(line, "inspection_z"),
        place_rx = json_get_number(line, "place_rx"),
        place_ry = json_get_number(line, "place_ry"),
        place_rz = json_get_number(line, "place_rz"),
        inspection_rx = json_get_number(line, "inspection_rx"),
        inspection_ry = json_get_number(line, "inspection_ry"),
        inspection_rz = json_get_number(line, "inspection_rz"),
        retract_z = json_get_number(line, "retract_z"),
    }
    for _, key in ipairs({
        "place_x", "place_y", "place_z", "inspection_x", "inspection_y", "inspection_z",
        "place_rx", "place_ry", "place_rz",
        "inspection_rx", "inspection_ry", "inspection_rz", "retract_z",
    }) do
        local value = job[key]
        if not is_finite(value) then
            return nil, "BAD_NUMBER_" .. key
        end
    end
    local hold_id = json_get_string(line, "hold_id")
    if not valid_command_id(hold_id) then
        return nil, "BAD_HOLD_ID"
    end
    job.hold_id = hold_id
    if active_hold == nil or active_hold.hold_id ~= hold_id then
        return nil, "HOLD_ID_MISMATCH"
    end
    if not active_hold.ready then
        return nil, "HOLD_NOT_AT_INSPECTION"
    end
    for _, key in ipairs({"place_x", "place_y", "inspection_x", "inspection_y", "inspection_z"}) do
        if not almost_equal(job[key], active_hold[key]) then
            return nil, "HOLD_TARGET_MISMATCH_" .. string.upper(key)
        end
    end
    if not almost_equal(job.inspection_z, CFG.motion.place_inspection_z_mm) then
        return nil, "CONFIG_MISMATCH_PLACE_INSPECTION_Z"
    end

    local orientation = CFG.pick_orientation
    for _, prefix in ipairs({"place", "inspection"}) do
        if not almost_equal(job[prefix .. "_rx"], orientation.rx)
            or not almost_equal(job[prefix .. "_ry"], orientation.ry)
            or not almost_equal(job[prefix .. "_rz"], orientation.rz) then
            return nil, "CONFIG_MISMATCH_ORIENTATION"
        end
    end
    local expected_retract = job.place_z
        + CFG.motion.z_up_sign * CFG.motion.release_retract_mm
    if not almost_equal(job.retract_z, expected_retract) then
        return nil, "CONFIG_MISMATCH_RETRACT_Z"
    end
    return job, ""
end

local function build_stack_release_poses(job)
    local poses = {
        inspection = {
            x = job.inspection_x, y = job.inspection_y, z = job.inspection_z,
            rx = job.inspection_rx, ry = job.inspection_ry, rz = job.inspection_rz,
        },
        place_transfer = {
            x = job.place_x, y = job.place_y, z = job.inspection_z,
            rx = job.place_rx, ry = job.place_ry, rz = job.place_rz,
        },
        place = {
            x = job.place_x, y = job.place_y, z = job.place_z,
            rx = job.place_rx, ry = job.place_ry, rz = job.place_rz,
        },
        place_retract = {
            x = job.place_x, y = job.place_y, z = job.retract_z,
            rx = job.place_rx, ry = job.place_ry, rz = job.place_rz,
        },
    }
    for name, pose in pairs(poses) do
        if not pose_in_workspace(pose) then
            return nil, "OUT_OF_WORKSPACE_" .. name
        end
    end
    return poses, ""
end

local function place_from_inspection(socket, command_id, poses)
    local holding_part = true
    local ok
    local code

    -- 一旦第二阶段开始运动，就不允许把失败后的未知位置当作观察位重试。
    active_hold.ready = false
    send_status(socket, command_id, "running", {phase = "transfer_to_place"})
    ok, code = checked_movl(poses.place_transfer, CFG.motion.travel_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "descend_place_visual_z"})
    ok, code = checked_movl(poses.place, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    send_status(socket, command_id, "running", {phase = "vacuum_off"})
    ok, code = set_vacuum(false)
    if not ok then return false, code, holding_part end
    holding_part = false
    active_hold = nil

    send_status(socket, command_id, "running", {phase = "retract_place"})
    ok, code = checked_movl(poses.place_retract, CFG.motion.pick_v)
    if not ok then return false, code, holding_part end

    ok, code = go_photo(socket, command_id)
    if not ok then return false, code, holding_part end
    return true, "", holding_part
end

local function cache_and_send_terminal(socket, command_id, command, line, status, extra)
    local fields = {
        v = PROTOCOL_VERSION,
        id = command_id,
        status = status,
    }
    if extra ~= nil then
        for key, value in pairs(extra) do
            fields[key] = value
        end
    end
    -- 必须先写入幂等缓存，再尝试发网络；这样动作完成后即使正好断线，重连
    -- 收到相同 ID 也只返回缓存终态，不会再次抓取。
    remember_terminal(command_id, command, line, fields)
    return write_json(socket, fields)
end

local function handle_line(socket, line)
    if #line == 0 then
        return true
    end
    if #line > CFG.max_line_bytes then
        log("protocol line is too long")
        return false
    end
    if line:sub(1, 1) ~= "{" or line:sub(-1) ~= "}" then
        log("invalid JSON object boundary")
        return true
    end

    local version = json_get_number(line, "v")
    local command_id = json_get_string(line, "id")
    local command = json_get_string(line, "cmd")
    if version ~= PROTOCOL_VERSION then
        if valid_command_id(command_id) then
            send_status(socket, command_id, "error", {
                code = "BAD_VERSION", message = "protocol version must be 1", recoverable = true,
            })
        end
        return true
    end
    if not valid_command_id(command_id) then
        log("invalid or missing command id")
        return true
    end
    if command == nil or command:match("^[a-z_]+$") == nil then
        send_status(socket, command_id, "error", {
            code = "BAD_COMMAND", message = "invalid command", recoverable = true,
        })
        return true
    end

    local cached = recent_by_id[command_id]
    if cached ~= nil then
        if cached.command ~= command or cached.request_line ~= line then
            send_status(socket, command_id, "error", {
                code = "DUPLICATE_ID_CONFLICT",
                message = "same id used with different payload",
                recoverable = false,
            })
            return true
        end
        send_status(socket, command_id, "accepted", {duplicate = true})
        return write_json(socket, cached.terminal)
    end

    if command == "ping" then
        send_status(socket, command_id, "pong", {
            state = active_hold ~= nil and "holding_task3_part" or "idle",
        })
        return true
    end

    if command == "stop_after_current" then
        -- 普通停止请求：主线程不会抢断正在执行的运动；它只会在当前命令完成
        -- 后读到这里。不追加运动，也不改变吸盘输出；运动异常后贸然断真空可能掉件。
        send_status(socket, command_id, "accepted", {cmd = command})
        return cache_and_send_terminal(socket, command_id, command, line, "done", {
            phase = active_hold ~= nil and "at_place_inspection" or "idle",
            holding_part = active_hold ~= nil,
        })
    end

    local config_ok, config_error = validate_static_config()
    if not config_ok then
        send_status(socket, command_id, "error", {
            code = "CONFIG_INVALID", message = config_error, recoverable = false,
        })
        return true
    end

    if active_hold ~= nil and command ~= "place_from_inspection" then
        send_status(socket, command_id, "error", {
            code = "HOLD_ACTIVE",
            message = "a task3 part is held; only place_from_inspection is allowed",
            recoverable = false,
            holding_part = true,
        })
        return true
    end

    if command == "go_photo" then
        local request_ok, request_error = validate_request_context(line)
        if not request_ok then
            send_status(socket, command_id, "error", {
                code = request_error, message = "Python/Lua config mismatch", recoverable = false,
            })
            return true
        end
        send_status(socket, command_id, "accepted", {cmd = command})
        local ok, code = go_photo(socket, command_id)
        if ok then
            return cache_and_send_terminal(
                socket, command_id, command, line, "done", {phase = "at_photo"}
            )
        end
        return cache_and_send_terminal(socket, command_id, command, line, "error", {
            code = code, message = "failed to reach photo pose", recoverable = false,
        })
    elseif command == "pick_to_inspection" then
        local request_ok, request_error = validate_request_context(line)
        if not request_ok then
            send_status(socket, command_id, "error", {
                code = request_error, message = "Python/Lua config mismatch", recoverable = false,
            })
            return true
        end
        local job, parse_error = parse_stack_pick_job(line)
        if job == nil then
            send_status(socket, command_id, "error", {
                code = parse_error, message = "invalid task3 inspection fields", recoverable = true,
            })
            return true
        end
        local poses, pose_error = build_stack_pick_poses(job)
        if poses == nil then
            send_status(socket, command_id, "error", {
                code = pose_error, message = "task3 inspection pose outside workspace", recoverable = false,
            })
            return true
        end

        send_status(socket, command_id, "accepted", {cmd = command})
        local ok, code, holding_part = pick_to_inspection(socket, command_id, job, poses)
        if ok then
            return cache_and_send_terminal(socket, command_id, command, line, "done", {
                phase = "at_place_inspection", holding_part = true,
            })
        end
        if holding_part and not CFG.vacuum.keep_on_after_pick_error then
            set_vacuum(false)
            active_hold = nil
            holding_part = false
        end
        return cache_and_send_terminal(socket, command_id, command, line, "error", {
            code = code,
            message = holding_part and "motion failed while holding task3 part"
                or "task3 pick/inspection failed",
            recoverable = false,
            holding_part = holding_part,
        })
    elseif command == "place_from_inspection" then
        local request_ok, request_error = validate_request_context(line)
        if not request_ok then
            send_status(socket, command_id, "error", {
                code = request_error, message = "Python/Lua config mismatch", recoverable = false,
            })
            return true
        end
        local job, parse_error = parse_stack_release_job(line)
        if job == nil then
            send_status(socket, command_id, "error", {
                code = parse_error, message = "invalid task3 visual placement fields", recoverable = false,
            })
            return true
        end
        local poses, pose_error = build_stack_release_poses(job)
        if poses == nil then
            send_status(socket, command_id, "error", {
                code = pose_error, message = "task3 visual placement pose outside workspace", recoverable = false,
            })
            return true
        end

        send_status(socket, command_id, "accepted", {cmd = command})
        local ok, code, holding_part = place_from_inspection(socket, command_id, poses)
        if ok then
            return cache_and_send_terminal(
                socket, command_id, command, line, "done", {phase = "at_photo", holding_part = false}
            )
        end
        if holding_part and not CFG.vacuum.keep_on_after_pick_error then
            set_vacuum(false)
            active_hold = nil
            holding_part = false
        end
        return cache_and_send_terminal(socket, command_id, command, line, "error", {
            code = code,
            message = holding_part and "visual placement failed while holding task3 part"
                or "visual placement failed after release",
            recoverable = false,
            holding_part = holding_part,
        })
    elseif command == "pick_place" then
        local request_ok, request_error = validate_request_context(line)
        if not request_ok then
            send_status(socket, command_id, "error", {
                code = request_error, message = "Python/Lua config mismatch", recoverable = false,
            })
            return true
        end
        -- 数值和全部派生点在 accepted 前校验，非法数据绝不会启动运动。
        local job, parse_error = parse_pick_job(line)
        if job == nil then
            send_status(socket, command_id, "error", {
                code = parse_error, message = "invalid pick/place fields", recoverable = true,
            })
            return true
        end
        local poses, pose_error = build_job_poses(job)
        if poses == nil then
            send_status(socket, command_id, "error", {
                code = pose_error, message = "derived pose outside workspace", recoverable = false,
            })
            return true
        end

        send_status(socket, command_id, "accepted", {cmd = command})
        local ok, code, holding_part = pick_and_place(socket, command_id, line)
        if ok then
            return cache_and_send_terminal(
                socket, command_id, command, line, "done", {phase = "at_photo"}
            )
        end
        if holding_part and not CFG.vacuum.keep_on_after_pick_error then
            set_vacuum(false)
        end
        return cache_and_send_terminal(socket, command_id, command, line, "error", {
            code = code,
            message = holding_part and "motion failed while holding part" or "pick/place failed",
            recoverable = false,
            holding_part = holding_part,
        })
    end

    send_status(socket, command_id, "error", {
        code = "UNKNOWN_COMMAND", message = "unsupported command", recoverable = true,
    })
    return true
end

local function communication_session(socket)
    local config_ok, config_error = validate_static_config()
    if config_ok then
        write_json(socket, {
            v = PROTOCOL_VERSION,
            id = "HELLO",
            status = "ready",
            phase = "idle",
            model = "Dobot-E6",
        })
    else
        write_json(socket, {
            v = PROTOCOL_VERSION,
            id = "HELLO",
            status = "config_error",
            code = "CONFIG_INVALID",
            message = config_error,
        })
    end

    local buffer = ""
    while true do
        -- timeout=0 为官方定义的阻塞读取。连接断开后 TCPRead 返回非零并进入重连。
        local err, chunk = TCPRead(socket, 0, "string")
        if err ~= 0 then
            log("TCPRead failed, err=" .. tostring(err))
            return false
        end
        if type(chunk) ~= "string" then
            log("TCPRead returned non-string data")
            return false
        end
        buffer = buffer .. chunk

        while true do
            local newline = buffer:find("\n", 1, true)
            if newline == nil then
                break
            end
            local line = buffer:sub(1, newline - 1)
            buffer = buffer:sub(newline + 1)
            if line:sub(-1) == "\r" then
                line = line:sub(1, -2)
            end
            local ok, keep_connection = pcall(handle_line, socket, line)
            if not ok then
                log("command handler exception: " .. tostring(keep_connection))
                -- API 异常可能发生在机械臂已经运动之后。把该 ID 记为不可恢复终态，
                -- 重连后绝不能因为 Python 重发同一 ID 而再次执行一次抓取。
                local command_id = json_get_string(line, "id")
                local command = json_get_string(line, "cmd")
                if valid_command_id(command_id) and command ~= nil
                    and command ~= "ping" and recent_by_id[command_id] == nil then
                    local fields = {
                        v = PROTOCOL_VERSION,
                        id = command_id,
                        status = "error",
                        code = "LUA_HANDLER_EXCEPTION",
                        message = tostring(keep_connection),
                        recoverable = false,
                        holding_part = "unknown",
                    }
                    remember_terminal(command_id, command, line, fields)
                    write_json(socket, fields)
                end
                return false
            end
            if not keep_connection then
                return false
            end
        end

        if #buffer > CFG.max_line_bytes then
            log("unfinished NDJSON line exceeds max_line_bytes")
            return false
        end
    end
end

local function main()
    log("RAICOM E6 executor started")
    while true do
        local create_err, socket = TCPCreate(false, CFG.python_ip, CFG.python_port)
        if create_err ~= 0 then
            log("TCPCreate failed, err=" .. tostring(create_err))
        else
            local start_err = TCPStart(socket, CFG.connect_timeout_s)
            if start_err == 0 then
                log("connected to Python " .. CFG.python_ip .. ":" .. tostring(CFG.python_port))
                communication_session(socket)
            else
                log("TCPStart failed, err=" .. tostring(start_err))
            end
            TCPDestroy(socket)
        end
        log("reconnect after " .. tostring(CFG.reconnect_delay_ms) .. " ms")
        Wait(CFG.reconnect_delay_ms)
    end
end

main()
