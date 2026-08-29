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
  };
  let latestStatus = null;
  let events = [];
  let previousStateKey = "";
  let commandPending = false;
  let streamLoaded = false;

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
    setStatusCell(ui.activationMode, MODE_LABELS[status.activation_mode_name] || "未知", status.activation_mode_name === "NONE" ? "warn" : "good");
    setStatusCell(ui.holdState, yesNo(status.hold_requested, "保持中", "已解除"), status.hold_requested ? "warn" : "good");
    const mavrosHealthy = Boolean(status.mavros_connected && status.mavros_fresh);
    setStatusCell(ui.mavrosState, yesNo(mavrosHealthy, "已连接", "异常或离线"), mavrosHealthy ? "good" : "bad");
    setStatusCell(ui.platformState, yesNo(status.drone_states_fresh, "新鲜", "已过期"), status.drone_states_fresh ? "good" : "bad");
    setStatusCell(ui.commandState, yesNo(status.command_fresh, "新鲜", "已过期"), status.command_fresh ? "good" : "warn");

    const video = status.video || {};
    const hasVideo = Boolean(video.available);
    ui.videoMeta.textContent = hasVideo ? `视频帧 ${ageText(video.age_seconds)}` : "等待视频流";
    ui.videoOverlay.hidden = !hasVideo;
    ui.overlayTarget.textContent = status.target_locked ? `锁定目标 #${status.locked_target_id}` : "未锁定目标";
    ui.overlayFrame.textContent = hasVideo ? `帧 ${video.sequence || 0}` : "实时帧";
    ui.videoEmpty.hidden = hasVideo && streamLoaded;

    const stateKey = `${available}:${state}:${status.reason}:${status.session_id || 0}`;
    if (available && stateKey !== previousStateKey) {
      addEvent(`${stateLabel}：${reasonText(status.reason)}`);
      previousStateKey = stateKey;
    }
    updateControlAvailability(status);
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
    }
  }

  function updateClock() { ui.clock.textContent = timeText(); }
  ui.video.addEventListener("load", () => {
    streamLoaded = true;
    if (latestStatus && latestStatus.video && latestStatus.video.available) ui.videoEmpty.hidden = true;
  });
  ui.video.addEventListener("error", () => {
    streamLoaded = false;
    ui.videoEmpty.hidden = false;
  });
  ui.form.addEventListener("submit", (event) => {
    event.preventDefault();
    submitCommand(modeFromForm() === "auto" ? "enable_auto" : "enable_manual");
  });
  ui.form.querySelectorAll("input[name='activation-mode']").forEach((input) => input.addEventListener("change", () => updateControlAvailability()));
  ui.form.querySelectorAll("button[data-command]").forEach((button) => button.addEventListener("click", () => submitCommand(button.dataset.command)));
  ui.resetConfirmed.addEventListener("change", () => updateControlAvailability());
  ui.clearEvents.addEventListener("click", () => { events = []; renderEvents(); });

  renderEvents();
  updateClock();
  updateControlAvailability(null);
  refreshStatus();
  window.setInterval(updateClock, 1000);
  window.setInterval(refreshStatus, STATUS_POLL_MS);
})();
