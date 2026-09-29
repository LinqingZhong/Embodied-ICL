"use strict";
const $ = (id) => document.getElementById(id);
let status = null,
  connected = false,
  lastResponse = 0,
  lastFrame = 0;
let snapshotBusy = false;
let layoutKey = "";
const rows = new Map(),
  pending = new Map();
let sending = false,
  commandChain = Promise.resolve(),
  connectionGeneration = 0;
function error(message) {
  $("error").textContent = message;
  $("error").hidden = false;
  clearTimeout(error.timer);
  error.timer = setTimeout(() => ($("error").hidden = true), 6000);
}
async function api(path, body) {
  const options = { signal: AbortSignal.timeout(4000) };
  if (body !== undefined)
    Object.assign(options, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    });
  const response = await fetch(path, options);
  const data = await response.json();
  if (!response.ok) throw new Error(data.detail || "请求失败");
  return data;
}
function command(body) {
  const generation = connectionGeneration;
  const execute = async () => {
    if (
      !connected ||
      status?.phase !== "ready" ||
      generation !== connectionGeneration
    )
      return;
    try {
      await api("/api/command", body);
    } catch (e) {
      error(e.message);
    }
  };
  commandChain = commandChain.then(execute, execute);
  return commandChain;
}
async function drainTargets() {
  if (sending) return;
  sending = true;
  try {
    while (pending.size && connected) {
      const [joint, value] = pending.entries().next().value;
      pending.delete(joint);
      await command({ action: "joint", joint, value });
    }
  } finally {
    sending = false;
  }
}
function queueTarget(index, value) {
  if (!connected) return;
  pending.set(index, value);
  drainTargets();
}
function buildJoints(joints) {
  let lastGroup = null;
  for (const joint of joints) {
    if (rows.has(joint.index)) continue;
    if (joint.group !== lastGroup) {
      const title = document.createElement("h3");
      title.textContent = joint.group;
      $("joints").append(title);
      lastGroup = joint.group;
    }
    const row = document.createElement("div");
    row.className = "joint";
    row.innerHTML =
      '<div class="joint-label"><label></label><output></output></div><div class="joint-input"><button type="button">−</button><input type="range" step="any"><button type="button">+</button></div><div class="joint-actual"></div>';
    const input = row.querySelector("input"),
      label = row.querySelector("label");
    input.id = `joint-${joint.index}`;
    label.htmlFor = input.id;
    label.textContent = joint.name;
    input.min = joint.min;
    input.max = joint.max;
    input.value = joint.target;
    const output = row.querySelector("output");
    const buttons = row.querySelectorAll("button");
    buttons[0].setAttribute("aria-label", `${joint.name} 减小目标`);
    buttons[1].setAttribute("aria-label", `${joint.name} 增大目标`);
    const send = (value) => {
      value = Math.min(Number(input.max), Math.max(Number(input.min), value));
      input.value = value;
      output.textContent = `${value.toFixed(3)} ${joint.unit}`;
      row.lastEdit = performance.now();
      queueTarget(joint.index, value);
    };
    input.addEventListener("pointerdown", () => (row.editing = true));
    input.addEventListener("keydown", () => (row.editing = true));
    input.addEventListener("keyup", () => (row.editing = false));
    input.addEventListener("input", () => send(Number(input.value)));
    buttons[0].onclick = () => send(Number(input.value) - joint.step);
    buttons[1].onclick = () => send(Number(input.value) + joint.step);
    $("joints").append(row);
    rows.set(joint.index, { row, input, output });
  }
}
function controls() {
  const ready = connected && status?.phase === "ready";
  document
    .querySelectorAll(".control-panel button,.control-panel input")
    .forEach((el) => (el.disabled = !ready));
  const manual = ready && status?.manual_control !== false && !status?.active_task;
  document.querySelectorAll("#joints button,#joints input,#grippers button")
    .forEach((el) => (el.disabled = !manual));
  if (!manual) pending.clear();
  $("step").disabled = !ready || !status.paused;
  $("snapshot").disabled = !ready || !status.sequence || snapshotBusy;
  $("pause").textContent = status?.paused ? "继续" : "暂停";
}
function refreshImages(data) {
  for (const { name: camera } of data.cameras) {
    const img = $(`${camera}-view`),
      health = img.health;
    if (
      !data.sequence ||
      health.busy ||
      (!health.failed && health.sequence === data.sequence)
    )
      continue;
    health.busy = true;
    fetch(`/api/camera/${camera}.jpg?frame=${data.sequence}`, {
      signal: AbortSignal.timeout(2500),
    })
      .then((response) => {
        if (!response.ok) throw new Error("Camera unavailable");
        return response.blob();
      })
      .then((blob) => {
        if (health.url) URL.revokeObjectURL(health.url);
        health.url = URL.createObjectURL(blob);
        health.sequence = data.sequence;
        img.src = health.url;
      })
      .catch(() => {
        health.busy = false;
        health.failed = true;
      });
  }
}
async function poll() {
  try {
    const data = await api("/api/status");
    lastResponse = performance.now();
    connected = true;
    if (data.sequence !== status?.sequence) lastFrame = performance.now();
    ensureLayout(data);
    status = data;
    $("connection").textContent =
      data.phase === "ready"
        ? "● 已连接"
        : "● " + (data.phase === "error" ? "仿真异常" : "初始化中");
    $("connection").className =
      "badge " + (data.phase === "error" ? "offline" : "online");
    $("sim-time").textContent = `${data.sim_time.toFixed(2)} s`;
    $("mode").textContent =
      data.phase !== "ready"
        ? "初始化 / 异常"
        : data.paused
          ? "已暂停"
          : "运行中";
    $("gripper-width").textContent =
      data.gripper_width == null
        ? data.grippers
            .map((g) => `${g.label} ${(g.actual[0] ?? 0).toFixed(2)} ${g.unit}`)
            .join(" / ") || "—"
        : `${(data.gripper_width * 1000).toFixed(1)} mm`;
    $("message").textContent = data.active_task
      ? `程序控制中：${data.active_task}`
      : data.manual_control === false ? "观测模式：通过 RobotClient 下发动作" : data.message;
    buildJoints(data.joints);
    for (const joint of data.joints) {
      const { row, input, output } = rows.get(joint.index);
      if (
        performance.now() - (row.lastEdit || -Infinity) > 700 &&
        !row.editing &&
        !pending.has(joint.index)
      )
        input.value = joint.target;
      output.textContent = `${Number(input.value).toFixed(3)} ${joint.unit}`;
      row.querySelector(".joint-actual").textContent =
        `实际 ${joint.actual.toFixed(3)}  ·  已施加 ${joint.applied.toFixed(3)} ${joint.unit}`;
    }
    refreshImages(data);
  } catch (e) {
    connected = false;
    pending.clear();
    connectionGeneration++;
    $("connection").textContent = "● 连接中断";
    $("connection").className = "badge offline";
    $("message").textContent =
      "无法连接服务器。正在重试；请检查仿真进程和 SSH 隧道。";
  }
  controls();
  setTimeout(poll, 100);
}
function ensureLayout(data) {
  const key = JSON.stringify([
    data.robot,
    data.joints.map((j) => j.name),
    data.cameras,
    data.grippers.map((g) => g.name),
  ]);
  if (key === layoutKey) return;
  layoutKey = key;
  connectionGeneration++;
  pending.clear();
  rows.clear();
  $("joints").replaceChildren();
  $("robot-name").textContent = data.robot;
  document.querySelectorAll("#camera-list img").forEach((img) => {
    if (img.health?.url) URL.revokeObjectURL(img.health.url);
  });
  $("camera-list").replaceChildren();
  for (const camera of data.cameras) {
    const article = document.createElement("article");
    article.className = "camera";
    article.innerHTML =
      '<div class="section-title"><h2></h2></div><div class="image-wrap"><img><div class="camera-placeholder">等待第一帧真实渲染画面</div></div>';
    article.querySelector("h2").textContent = camera.label;
    const img = article.querySelector("img");
    img.id = `${camera.name}-view`;
    img.alt = camera.label;
    img.health = {
      busy: false,
      failed: true,
      sequence: 0,
      received: 0,
      url: null,
    };
    img.onload = () => {
      Object.assign(img.health, {
        busy: false,
        failed: false,
        received: performance.now(),
      });
      img.parentElement.classList.add("ready");
    };
    img.onerror = () => {
      img.health.busy = false;
      img.health.failed = true;
    };
    $("camera-list").append(article);
  }
  $("grippers").replaceChildren();
  data.grippers.forEach((gripper, index) => {
    const section = document.createElement("div");
    section.className = "gripper";
    const title = document.createElement("h3");
    title.textContent = gripper.label;
    section.append(title);
    for (const [open, label] of [
      [true, "张开"],
      [false, "闭合"],
    ]) {
      const button = document.createElement("button");
      button.textContent = label;
      button.id =
        index === 0
          ? open
            ? "open"
            : "close"
          : `${gripper.name}-${open ? "open" : "close"}`;
      button.onclick = () =>
        command({ action: "gripper", gripper: gripper.name, open });
      section.append(button);
    }
    $("grippers").append(section);
  });
}
window.addEventListener("pointerup", () =>
  rows.forEach(({ row }) => (row.editing = false)),
);
window.addEventListener("pointercancel", () =>
  rows.forEach(({ row }) => (row.editing = false)),
);
setInterval(() => {
  const staleConnection = performance.now() - lastResponse > 2500;
  if (staleConnection && connected) {
    connected = false;
    pending.clear();
    connectionGeneration++;
    controls();
  }
  const age =
    status?.frame_age == null
      ? null
      : status.frame_age + (performance.now() - lastResponse) / 1000;
  $("frame-age").textContent = age == null ? "—" : `${age.toFixed(1)} s`;
  const stale =
    !connected || status?.phase !== "ready" || !lastFrame || age > 3;
  document.querySelectorAll(".image-wrap").forEach((el) => {
    const health = el.querySelector("img").health;
    el.classList.toggle(
      "stale",
      stale || health.failed || performance.now() - health.received > 3000,
    );
    el.querySelector(".camera-placeholder").textContent = status?.sequence
      ? "画面已过期，等待新帧"
      : "等待第一帧真实渲染画面";
  });
}, 250);
$("pause").onclick = () =>
  command({ action: status.paused ? "resume" : "pause" });
$("step").onclick = () => command({ action: "step" });
$("reset").onclick = () => {
  pending.clear();
  command({ action: "reset" });
};
$("snapshot").onclick = async () => {
  snapshotBusy = true;
  controls();
  try {
    const result = await api("/api/snapshot", {});
    const link = document.createElement("a");
    link.href = result.download;
    link.download = result.filename;
    document.body.append(link);
    link.click();
    link.remove();
    $("message").textContent = `快照已保存：${result.filename}`;
  } catch (e) {
    error(e.message);
  } finally {
    snapshotBusy = false;
    controls();
  }
};
window.addEventListener("blur", () => {
  pending.clear();
  rows.forEach(({ row }) => (row.editing = false));
});
poll();
