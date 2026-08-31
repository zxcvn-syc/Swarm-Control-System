(() => {
  "use strict";

  const STATE_LABELS = {
    LOCKED: "已锁定",
    MANUAL_READY: "手动封控待命",
    AUTO_READY: "自动封控待命",
    ACTIVE: "封控执行中",
    FAULT: "故障闭锁",
    EMERGENCY_HOLD: "紧急保持",
  };
  const MODE_LABELS = { NONE: "未启用", MANUAL: "手动", AUTO: "自动" };
  const REASONS = {
    operator_token_not_configured: "未配置操作员控制令牌，当前为只读模式。",
    remote_control_not_explicitly_allowed: "远程控制未获明确允许，当前为只读模式。",
    safety_status_unavailable: "尚未收到监督器安全状态。",
    safety_status_stale: "监督器安全状态已过期，拒绝执行控制。",
    control_service_unavailable: "封控控制服务不可用。",
    control_service_timeout: "封控控制服务响应超时。",
    control_service_error: "封控控制服务出现错误。",
    operator_token_invalid: "控制令牌无效。",
    operator_id_required: "请填写操作员编号。",
    ground_confirmation_required: "复位前必须确认地面安全状态。",
    unsupported_command: "不支持的封控命令。",
    control_ready: "控制链路已就绪。",
    startup_locked: "监督器启动后默认锁定。",
    invalid_enclosure_command: "封控指令格式无效。",
    future_enclosure_command: "封控指令时间戳超前。",
    stale_enclosure_command: "封控指令已过期。",
    replayed_enclosure_command: "检测到重复封控指令。",
    unknown_control_command: "未知控制命令。",
    control_session_mismatch: "控制请求不属于当前监督器会话。",
    control_request_expired: "控制请求已过期。",
    operator_emergency_hold: "操作员发起紧急保持。",
    reset_requires_fault_or_emergency_hold: "仅故障闭锁或紧急保持状态允许复位。",
    operator_reset_locked: "已完成地面确认，监督器回到锁定状态。",
    emergency_hold_requires_reset: "紧急保持必须先完成确认复位。",
    operator_disabled: "操作员已停止封控。",
    fault_or_emergency_hold_requires_reset: "故障或紧急保持状态必须先确认复位。",
    operator_enabled_manual: "操作员已启用手动封控，等待新的封控指令。",
    operator_enabled_auto: "操作员已启用自动封控，等待稳定目标锁定和新指令。",
    unsupported_control_command: "不支持的控制命令。",
    enclosure_command_timeout: "封控指令心跳超时。",
    target_lock_lost: "自动封控所需的目标锁定已丢失。",
    containment_command_active: "封控指令正在通过安全门。",
    drone_states_timeout: "平台状态超时。",
    no_available_platform: "没有可用的封控平台。",
    mavros_state_timeout: "MAVROS 状态超时。",
    mavros_disconnected: "MAVROS 已断开。",
    flight_safety_fault: "飞行安全监督器进入故障闭锁。",
    invalid_ground_confirmation: "地面确认字段无效。",
    enclosure_command_timeout: "封控指令心跳超时。",
    target_timeout: "目标状态超时。",
    mavros_state_timeout: "MAVROS 状态超时。",
    control_request_replayed: "控制请求已被使用，已拒绝重放。",
    pilot_commands_disabled: "飞手命令默认关闭。",
    pilot_audit_log_not_configured: "未配置飞手命令审计日志，已拒绝控制。",
    pilot_control_ready: "飞手命令链路已就绪，每次操作仍需确认短语。",
    pilot_action_required: "请选择飞手命令。",
    pilot_action_unsupported: "不支持该飞手命令。",
    pilot_confirmation_required: "请输入命令确认短语。",
    pilot_confirmation_mismatch: "确认短语不匹配，命令未发送。",
    pilot_command_in_progress: "上一条飞手命令仍在执行。",
    pilot_mavros_state_unavailable: "尚未收到飞控状态。",
    pilot_mavros_state_stale: "飞控状态已过期。",
    pilot_mavros_disconnected: "MAVROS 未连接飞控。",
    pilot_vehicle_already_armed: "飞行器已解锁。",
    pilot_vehicle_already_disarmed: "飞行器已上锁。",
    pilot_arm_offboard_rejected: "禁止在 OFFBOARD 状态下请求解锁。",
    pilot_safety_gate_not_locked: "仅安全门锁定并保持时允许请求解锁。",
    pilot_ground_confirmation_required: "上锁前必须确认飞行器已在地面。",
    pilot_offboard_requires_armed: "进入 OFFBOARD 前飞行器必须已由飞手解锁。",
    pilot_offboard_safety_gate_inactive: "进入 OFFBOARD 需要目标锁定、封控安全门激活且保持解除。",
    pilot_mode_not_configured: "该飞行模式未配置。",
    pilot_arm_service_unavailable: "MAVROS 解锁服务不可用。",
    pilot_mode_service_unavailable: "MAVROS 模式服务不可用。",
    pilot_command_timeout: "飞控服务响应超时。",
    pilot_command_service_error: "飞控服务调用失败。",
    pilot_command_rejected: "飞控拒绝了该请求。",
    pilot_command_sent: "请求已发送；请等待飞控状态实际更新后再进入下一步。",
    pilot_audit_log_unavailable: "审计日志不可写，已拒绝发送命令。",
  };
  const STATUS_POLL_MS = 750;
  const MAX_EVENTS = 16;

  const element = (id) => document.getElementById(id);
  const ui = {
    dot: element("status-dot"),
    connection: element("status-connection"),
    clock: element("local-time"),
    video: element("video-stream"),
    videoEmpty: element("video-empty"),
    videoMeta: element("video-meta"),
    videoOverlay: element("video-overlay"),
    overlayTarget: element("overlay-target"),
    overlayFrame: element("overlay-frame"),
    stateName: element("state-heading"),
    stateReason: element("state-reason"),
    targetLock: element("target-lock"),
    activationMode: element("activation-mode"),
    holdState: element("hold-state"),
    mavrosState: element("mavros-state"),
    platformState: element("platform-state"),
    commandState: element("command-state"),
    form: element("control-form"),
    operatorId: element("operator-id"),
    token: element("operator-token"),
    resetConfirmed: element("ground-confirmed"),
    resetButton: element("reset-button"),
    enableButton: element("enable-button"),
    message: element("control-message"),
    eventLog: element("event-log"),
    clearEvents: element("clear-events"),
    pilotAvailability: element("pilot-availability"),
    pilotFcuState: element("pilot-fcu-state"),
    pilotArmState: element("pilot-arm-state"),
    pilotModeState: element("pilot-mode-state"),
    pilotActions: element("pilot-actions"),
    pilotMessage: element("pilot-control-message"),
    pilotDialog: element("pilot-confirmation"),
    pilotDialogForm: element("pilot-confirmation-form"),
    pilotDialogDescription: element("pilot-confirmation-description"),
    pilotDialogInput: element("pilot-confirmation-input"),
    pilotDialogCancel: element("pilot-confirmation-cancel"),
  };
  let latestStatus = null;
  let events = [];
  let previousStateKey = "";
  let commandPending = false;
  let pilotCommandPending = false;
  let pendingPilotAction = null;
  let streamErrored = false;

  const ageText = (seconds) => {
    if (typeof seconds !== "number" || !Number.isFinite(seconds)) return "未知";
    if (seconds < 1) return "刚刚";
    return `${seconds.toFixed(seconds < 10 ? 1 : 0)} 秒前`;
  };
  const yesNo = (value, yes, no) => (value ? yes : no);
  const reasonText = (reason) => REASONS[reason] || reason || "未提供原因。";
  const timeText = (date = new Date()) => date.toLocaleTimeString("zh-CN", { hour12: false });
  const modeFromForm = () => document.querySelector('input[name="activation-mode"]:checked').value;
  const commandLabel = (command) => ({
    enable_manual: "开启手动封控",
    enable_auto: "开启自动封控",
    disable: "停止封控",
    emergency_hold: "紧急保持",
    reset_fault: "地面确认后复位",
  }[command] || command);
  const pilotActionLabel = (action) => ({
    arm: "请求解锁",
    disarm: "地面确认后上锁",
    position: "切换 POSCTL",
    altitude: "切换 ALTCTL",
    offboard: "请求 OFFBOARD",
  }[action] || action);

  function setClass(node, state) {
    node.classList.remove("good", "warn", "bad");
    if (state) node.classList.add(state);
  }

  function setStatusCell(node, label, state) {
    node.textContent = label;
    setClass(node, state);
  }

  function setMessage(text, type = "") {
    ui.message.textContent = text;
    ui.message.classList.remove("error", "success");
    if (type) ui.message.classList.add(type);
  }

  function setPilotMessage(text, type = "") {
    ui.pilotMessage.textContent = text;
    ui.pilotMessage.classList.remove("error", "success");
    if (type) ui.pilotMessage.classList.add(type);
  }

  function renderEvents() {
    ui.eventLog.replaceChildren();
    if (events.length === 0) {
      const item = document.createElement("li");
      item.className = "event-empty";
      item.textContent = "尚未收到安全状态或操作结果。";
      ui.eventLog.append(item);
      return;
    }
    events.forEach((event) => {
      const item = document.createElement("li");
      const timestamp = document.createElement("time");
      timestamp.dateTime = event.iso;
      timestamp.textContent = event.time;
      const message = document.createElement("span");
      message.textContent = event.message;
      item.append(timestamp, message);
      ui.eventLog.append(item);
    });
  }

  function addEvent(message) {
    const now = new Date();
    events = [{ iso: now.toISOString(), time: timeText(now), message }, ...events].slice(0, MAX_EVENTS);
    renderEvents();
  }

  function renderStatus(status) {
    latestStatus = status;
    const available = Boolean(status.available);
    const state = String(status.state_name || "LOCKED");
    const stateLabel = available ? (STATE_LABELS[state] || state) : "等待安全状态";
    const isFault = state === "FAULT" || state === "EMERGENCY_HOLD";
    const isWarning = state === "LOCKED" || state === "MANUAL_READY" || state === "AUTO_READY";
    const stateClass = isFault ? "fault" : (isWarning ? "warning" : "");
    const fresh = typeof status.status_age_seconds === "number" && status.status_age_seconds <= 3;

    ui.stateName.textContent = stateLabel;
    ui.stateName.classList.remove("warning", "fault");
    if (stateClass) ui.stateName.classList.add(stateClass);
    ui.stateReason.textContent = available ? reasonText(status.reason) : "监督器尚未发布状态。";
    ui.dot.classList.remove("online", "fault");
    if (!available || !fresh || isFault) ui.dot.classList.add("fault");
    else ui.dot.classList.add("online");
    ui.connection.textContent = available ? `安全状态 ${ageText(status.status_age_seconds)}` : "等待安全状态";

    const targetText = status.target_locked ? `已锁定 #${status.locked_target_id}` : "未锁定";
    setStatusCell(ui.targetLock, targetText, status.target_locked ? "good" : "warn");
    setStatusCell(
      ui.activationMode,
      MODE_LABELS[status.activation_mode_name] || "未知",
      status.activation_mode_name === "NONE" ? "warn" : "good",
    );
    setStatusCell(ui.holdState, yesNo(status.hold_requested, "保持中", "已解除"), status.hold_requested ? "warn" : "good");
    const mavrosHealthy = Boolean(status.mavros_connected && status.mavros_fresh);
    setStatusCell(ui.mavrosState, yesNo(mavrosHealthy, "已连接", "异常或离线"), mavrosHealthy ? "good" : "bad");
    setStatusCell(
      ui.platformState,
      yesNo(status.drone_states_fresh, "新鲜", "已过期"),
      status.drone_states_fresh ? "good" : "bad",
    );
    setStatusCell(ui.commandState, yesNo(status.command_fresh, "新鲜", "已过期"), status.command_fresh ? "good" : "warn");

    const pilotMavros = status.mavros || {};
    const pilotFresh = Boolean(pilotMavros.available)
      && typeof pilotMavros.age_seconds === "number"
      && pilotMavros.age_seconds <= 1;
    setStatusCell(
      ui.pilotFcuState,
      yesNo(pilotFresh && pilotMavros.connected, "已连接", "离线或过期"),
      pilotFresh && pilotMavros.connected ? "good" : "bad",
    );
    setStatusCell(ui.pilotArmState, yesNo(pilotMavros.armed, "已解锁", "已上锁"), pilotMavros.armed ? "warn" : "good");
    setStatusCell(ui.pilotModeState, pilotMavros.mode || "未知", pilotMavros.mode === "OFFBOARD" ? "warn" : "good");
    ui.pilotAvailability.textContent = status.pilot_control_available ? "人工控制已启用" : "人工控制未启用";
    setClass(ui.pilotAvailability, status.pilot_control_available ? "good" : "warn");

    const video = status.video || {};
    const hasVideo = Boolean(video.available);
    ui.videoMeta.textContent = hasVideo ? `视频帧 ${ageText(video.age_seconds)}` : "等待视频流";
    ui.videoOverlay.hidden = !hasVideo;
    ui.overlayTarget.textContent = status.target_locked ? `锁定目标 #${status.locked_target_id}` : "未锁定目标";
    ui.overlayFrame.textContent = hasVideo ? `帧 ${video.sequence || 0}` : "实时帧";
    ui.videoEmpty.hidden = hasVideo && !streamErrored;

    const stateKey = `${available}:${state}:${status.reason}:${status.session_id || 0}`;
    if (available && stateKey !== previousStateKey) {
      addEvent(`${stateLabel}：${reasonText(status.reason)}`);
      previousStateKey = stateKey;
    }
    updateControlAvailability(status);
    updatePilotAvailability(status);
  }

  function updateControlAvailability(status = latestStatus) {
    const canControl = Boolean(status && status.control_available && status.available && !commandPending);
    ui.form.querySelectorAll("button").forEach((button) => { button.disabled = !canControl; });
    ui.resetButton.disabled = !canControl || !ui.resetConfirmed.checked;
    const selectedMode = modeFromForm();
    ui.enableButton.textContent = selectedMode === "auto" ? "开启自动封控" : "开启手动封控";
    if (status && !status.control_available) {
      setMessage(reasonText(status.control_reason), "error");
    } else if (!status || !status.available) {
      setMessage("等待监督器安全状态，暂不允许执行控制。", "error");
    } else if (!commandPending) {
      setMessage("控制链路已就绪。", "success");
    }
  }

  function updatePilotAvailability(status = latestStatus) {
    const canPilotControl = Boolean(status && status.pilot_control_available && !pilotCommandPending);
    ui.pilotActions.querySelectorAll("button").forEach((button) => { button.disabled = !canPilotControl; });
    if (!status || !status.pilot_control_available) {
      setPilotMessage(reasonText(status && status.pilot_control_reason), "error");
    } else if (!pilotCommandPending) {
      setPilotMessage("每条飞手命令均需输入确认短语，方向与姿态由 RC 控制。", "success");
    }
  }

  async function submitCommand(command) {
    if (commandPending) return;
    if (!latestStatus || !latestStatus.control_available || !latestStatus.available) {
      setMessage("控制链路未就绪。", "error");
      return;
    }
    const operatorId = ui.operatorId.value.trim();
    if (!operatorId) {
      ui.operatorId.focus();
      setMessage("请填写操作员编号。", "error");
      return;
    }
    if (!ui.token.value) {
      ui.token.focus();
      setMessage("请填写控制令牌。", "error");
      return;
    }
    if (command === "reset_fault" && !ui.resetConfirmed.checked) {
      setMessage("复位前必须确认地面安全状态。", "error");
      return;
    }
    if (command === "emergency_hold" && !window.confirm("紧急保持会立即锁存封控保持请求。确认执行？")) return;

    commandPending = true;
    updateControlAvailability();
    setMessage(`正在请求${commandLabel(command)}...`);
    try {
      const response = await fetch("/api/control", {
        method: "POST",
        headers: { "Content-Type": "application/json", "X-Flight-Safety-Token": ui.token.value },
        body: JSON.stringify({ command, operator_id: operatorId, ground_confirmed: ui.resetConfirmed.checked }),
      });
      const result = await response.json().catch(() => ({}));
      const outcome = result.accepted ? "已接受" : "被拒绝";
      const detail = reasonText(result.reason);
      setMessage(`${commandLabel(command)}${outcome}：${detail}`, result.accepted ? "success" : "error");
      addEvent(`${commandLabel(command)}${outcome}：${detail}`);
    } catch (_error) {
      setMessage("无法连接本地控制服务。", "error");
      addEvent(`${commandLabel(command)}失败：无法连接本地控制服务。`);
    } finally {
      commandPending = false;
      updateControlAvailability();
    }
  }

  function openPilotConfirmation(action) {
    if (!latestStatus || !latestStatus.pilot_control_available) {
      setPilotMessage("飞手命令链路未就绪。", "error");
      return;
    }
    const operatorId = ui.operatorId.value.trim();
    if (!operatorId) {
      ui.operatorId.focus();
      setPilotMessage("请先填写操作员编号。", "error");
      return;
    }
    if (!ui.token.value) {
      ui.token.focus();
      setPilotMessage("请先填写控制令牌。", "error");
      return;
    }
    if (action === "disarm" && !ui.resetConfirmed.checked) {
      setPilotMessage("上锁前必须勾选地面安全确认。", "error");
      return;
    }
    const phrase = latestStatus.pilot_actions && latestStatus.pilot_actions[action];
    if (!phrase) {
      setPilotMessage("该飞手命令未配置确认短语。", "error");
      return;
    }
    pendingPilotAction = action;
    ui.pilotDialogDescription.textContent = `${pilotActionLabel(action)}将通过 MAVROS 发给 PX4。输入 ${phrase} 后才会发送请求。`;
    ui.pilotDialogInput.value = "";
    ui.pilotDialogInput.placeholder = phrase;
    ui.pilotDialog.showModal();
    ui.pilotDialogInput.focus();
  }

  async function submitPilotCommand() {
    const action = pendingPilotAction;
    if (!action || pilotCommandPending) return;
    const confirmation = ui.pilotDialogInput.value.trim();
    const expected = latestStatus && latestStatus.pilot_actions && latestStatus.pilot_actions[action];
    if (confirmation !== expected) {
      ui.pilotDialogInput.focus();
      setPilotMessage("确认短语不匹配，命令未发送。", "error");
      return;
    }
    pilotCommandPending = true;
    updatePilotAvailability();
    setPilotMessage(`正在请求${pilotActionLabel(action)}...`);
    try {
      const response = await fetch("/api/pilot-control", {
        method: "POST",
        headers: { "Content-Type": "application/json", "X-Flight-Safety-Token": ui.token.value },
        body: JSON.stringify({
          action,
          confirmation,
          operator_id: ui.operatorId.value.trim(),
          ground_confirmed: ui.resetConfirmed.checked,
        }),
      });
      const result = await response.json().catch(() => ({}));
      const outcome = result.accepted ? "请求已接受" : "请求被拒绝";
      const detail = reasonText(result.reason);
      setPilotMessage(`${pilotActionLabel(action)}${outcome}：${detail}`, result.accepted ? "success" : "error");
      addEvent(`${pilotActionLabel(action)}${outcome}：${detail}`);
    } catch (_error) {
      setPilotMessage("无法连接本地飞手控制服务。", "error");
      addEvent(`${pilotActionLabel(action)}失败：无法连接本地飞手控制服务。`);
    } finally {
      pilotCommandPending = false;
      pendingPilotAction = null;
      updatePilotAvailability();
    }
  }

  async function refreshStatus() {
    try {
      const response = await fetch("/api/status", { cache: "no-store" });
      if (!response.ok) throw new Error("status unavailable");
      renderStatus(await response.json());
    } catch (_error) {
      latestStatus = null;
      ui.dot.classList.remove("online");
      ui.dot.classList.add("fault");
      ui.connection.textContent = "状态接口不可达";
      ui.stateName.textContent = "状态接口不可达";
      ui.stateName.className = "state-name fault";
      ui.stateReason.textContent = "请确认本地仪表板进程仍在运行。";
      updateControlAvailability(null);
      updatePilotAvailability(null);
    }
  }

  function updateClock() { ui.clock.textContent = timeText(); }
  ui.video.addEventListener("load", () => {
    streamErrored = false;
    if (latestStatus && latestStatus.video && latestStatus.video.available) ui.videoEmpty.hidden = true;
  });
  ui.video.addEventListener("error", () => {
    streamErrored = true;
    ui.videoEmpty.hidden = false;
  });
  ui.form.addEventListener("submit", (event) => {
    event.preventDefault();
    submitCommand(modeFromForm() === "auto" ? "enable_auto" : "enable_manual");
  });
  ui.form.querySelectorAll("input[name='activation-mode']").forEach((input) => {
    input.addEventListener("change", () => updateControlAvailability());
  });
  ui.form.querySelectorAll("button[data-command]").forEach((button) => {
    button.addEventListener("click", () => submitCommand(button.dataset.command));
  });
  ui.pilotActions.querySelectorAll("button[data-pilot-action]").forEach((button) => {
    button.addEventListener("click", () => openPilotConfirmation(button.dataset.pilotAction));
  });
  ui.pilotDialogForm.addEventListener("submit", (event) => {
    event.preventDefault();
    const action = pendingPilotAction;
    const expected = latestStatus && latestStatus.pilot_actions && latestStatus.pilot_actions[action];
    if (ui.pilotDialogInput.value.trim() !== expected) {
      ui.pilotDialogInput.focus();
      return;
    }
    ui.pilotDialog.close();
    submitPilotCommand();
  });
  ui.pilotDialogCancel.addEventListener("click", () => {
    pendingPilotAction = null;
    ui.pilotDialog.close();
  });
  ui.pilotDialog.addEventListener("cancel", () => { pendingPilotAction = null; });
  ui.resetConfirmed.addEventListener("change", () => {
    updateControlAvailability();
    updatePilotAvailability();
  });
  ui.clearEvents.addEventListener("click", () => { events = []; renderEvents(); });

  renderEvents();
  updateClock();
  updateControlAvailability(null);
  updatePilotAvailability(null);
  refreshStatus();
  window.setInterval(updateClock, 1000);
  window.setInterval(refreshStatus, STATUS_POLL_MS);
})();
