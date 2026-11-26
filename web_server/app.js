const get = (id) => document.getElementById(id);
const status = get("status");
const feedback = get("feedback");
const form = get("models");
const seg = get("segmentation");
const pose = get("pose");
const loadButton = get("load");
const logs = get("logs");
let cursor = 0;
let available = [];
let serverLoading = true;
let submitting = false;
let waitingForReload = false;
let online = false;

function updateLoadButton() {
  loadButton.disabled = !online || serverLoading || submitting || !seg.value || !pose.value || seg.value === pose.value;
  loadButton.textContent = serverLoading || submitting ? "Đang nạp…" : "Nạp model";
}

function renderOptions(select, names, active, defaultName) {
  const old = select.value;
  select.replaceChildren();
  if (!names.length) {
    select.add(new Option("Chưa có tệp model", ""));
    return;
  }
  for (const name of names) select.add(new Option(name, name));
  select.value = [old, active, defaultName + ".pt", defaultName + ".onnx", names[0]]
    .find((name) => names.includes(name));
}

function renderEntries(entries, reset) {
  if (reset) logs.replaceChildren();
  for (const entry of entries) {
    const row = document.createElement("li");
    row.dataset.level = entry.level;
    const time = document.createElement("time");
    time.dateTime = entry.time;
    time.textContent = new Date(entry.time).toLocaleTimeString("vi-VN");
    const level = document.createElement("span");
    level.className = "log-level";
    level.textContent = entry.level;
    const message = document.createElement("span");
    message.className = "log-message";
    message.textContent = entry.message;
    row.append(time, level, message);
    logs.append(row);
  }
  while (logs.children.length > 500) logs.firstElementChild.remove();
  get("empty").hidden = logs.children.length > 0;
  if (get("follow").checked) logs.scrollTop = logs.scrollHeight;
}

function showFeedback(message, error = false) {
  feedback.textContent = message;
  feedback.dataset.error = String(error);
}

async function refresh() {
  try {
    const response = await fetch(`/dashboard/api/state?after=${cursor}`, { cache: "no-store", signal: AbortSignal.timeout(5000) });
    if (response.status === 401) {
      window.location.replace("/dashboard/login");
      return;
    }
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const state = await response.json();
    online = true;
    serverLoading = state.loading;
    status.textContent = state.loading ? "Đang nạp model" : state.model_ready ? "Model sẵn sàng" : "Chưa có model";
    status.dataset.state = state.loading ? "waiting" : state.model_ready ? "ready" : "error";
    get("active-seg").textContent = state.segmentation || "—";
    get("active-pose").textContent = state.pose || "—";
    if (!seg.options.length || available.join("\n") !== state.available.join("\n")) {
      available = state.available;
      renderOptions(seg, available, state.segmentation, "best_yolo11_seg");
      renderOptions(pose, available, state.pose, "best_yolo11_pose");
    }
    if (waitingForReload && !state.loading) {
      waitingForReload = false;
      showFeedback(state.model_error || "Đã nạp model.", Boolean(state.model_error));
    } else if (state.model_error && !submitting) {
      showFeedback(state.model_error, true);
    }
    renderEntries(state.entries, state.reset);
    cursor = state.cursor;
  } catch {
    online = false;
    serverLoading = false;
    status.textContent = "Mất kết nối";
    status.dataset.state = "error";
  } finally {
    updateLoadButton();
  }
}

form.addEventListener("submit", async (event) => {
  event.preventDefault();
  if (loadButton.disabled) return;
  submitting = true;
  updateLoadButton();
  showFeedback("Đang gửi yêu cầu nạp model…");
  try {
    const response = await fetch("/dashboard/api/models", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ segmentation: seg.value, pose: pose.value }),
      signal: AbortSignal.timeout(5000),
    });
    if (response.status === 401) {
      window.location.replace("/dashboard/login");
      return;
    }
    const result = await response.json();
    if (!response.ok) throw new Error(result.message);
    serverLoading = true;
    waitingForReload = true;
    showFeedback(result.message);
  } catch (error) {
    showFeedback(error.message || "Không gửi được yêu cầu.", true);
  } finally {
    submitting = false;
    updateLoadButton();
  }
});

for (const select of [seg, pose]) select.addEventListener("change", updateLoadButton);
get("clear").addEventListener("click", () => {
  logs.replaceChildren();
  get("empty").hidden = false;
  get("empty").textContent = "Đang chờ log mới.";
});

get("logout").addEventListener("click", async () => {
  const button = get("logout");
  button.disabled = true;
  try {
    const response = await fetch("/dashboard/api/logout", { method: "POST" });
    if (!response.ok) throw new Error("Đăng xuất thất bại. Hãy thử lại.");
    window.location.replace("/dashboard/login");
  } catch (error) {
    showFeedback(error.message || "Không kết nối được AI Server.", true);
    button.disabled = false;
  }
});

async function poll() {
  await refresh();
  setTimeout(poll, 1200);
}
poll();
