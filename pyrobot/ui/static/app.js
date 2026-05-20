const stateBox = document.getElementById("state-box");
const poseLine = document.getElementById("pose-line");
const conn = document.getElementById("conn");
const gcodeForm = document.getElementById("gcode-form");
const gcodeInput = document.getElementById("gcode-input");
const gcodeResult = document.getElementById("gcode-result");
const canvas = document.getElementById("arm-canvas");
const ctx = canvas.getContext("2d");

async function fetchState() {
  const r = await fetch("/api/state");
  if (!r.ok) throw new Error(r.statusText);
  return r.json();
}

function drawArm(data) {
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const link = data.link_length_mm || 250;
  const q = data.joints_deg || { a: 90, b: 90, c: 0, d: 0 };
  const pose = data.pose_mm || { x: 250, y: 0, z: 250 };
  const home = data.home_mm || { x: 250, y: 0, z: 250 };

  const scale = 0.55;
  const bx = 50;
  const by = h - 40;

  const toScreen = (xMm, zMm) => ({
    x: bx + xMm * scale,
    y: by - zMm * scale,
  });

  const base = toScreen(0, 0);
  const end = toScreen(pose.x, pose.z);
  const homePt = toScreen(home.x, home.z);

  const aRad = ((q.a - (180 - q.b) / 2) * Math.PI) / 180;
  const elbow = {
    x: bx + link * scale * Math.sin(aRad),
    y: by - link * scale * Math.cos(aRad),
  };

  ctx.strokeStyle = "#2d3a4f";
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(base.x, base.y);
  ctx.lineTo(homePt.x, homePt.y);
  ctx.stroke();
  ctx.setLineDash([]);

  ctx.lineWidth = 5;
  ctx.lineCap = "round";
  ctx.strokeStyle = "#5a6a85";
  ctx.beginPath();
  ctx.moveTo(base.x, base.y);
  ctx.lineTo(elbow.x, elbow.y);
  ctx.stroke();

  ctx.strokeStyle = "#3d8bfd";
  ctx.beginPath();
  ctx.moveTo(elbow.x, elbow.y);
  ctx.lineTo(end.x, end.y);
  ctx.stroke();

  const r = 7;
  for (const [pt, color] of [
    [base, "#8b9cb3"],
    [elbow, "#f39c12"],
    [end, "#2ecc71"],
    [homePt, "#8b9cb366"],
  ]) {
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, r, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.fillStyle = "#8b9cb3";
  ctx.font = "11px system-ui";
  ctx.fillText("база", base.x + 10, base.y);
  ctx.fillText("home", homePt.x + 8, homePt.y - 8);
  ctx.fillStyle = "#2ecc71";
  ctx.fillText("TCP", end.x + 8, end.y);
}

function updateUi(data) {
  const m = data.motion || {};
  stateBox.textContent = JSON.stringify(data, null, 2);
  poseLine.textContent =
    `TCP: X=${data.pose_mm.x.toFixed(1)} Y=${data.pose_mm.y.toFixed(1)} Z=${data.pose_mm.z.toFixed(1)} mm · ` +
    `A=${data.joints_deg.a.toFixed(2)}° B=${data.joints_deg.b.toFixed(2)}° C=${data.joints_deg.c.toFixed(2)}°`;

  const fc = m.fault_code || 0;
  conn.textContent = fc ? `FAULT ${fc}` : m.in_motion ? "MOVING" : "OK";
  conn.className = "badge " + (fc ? "fault" : "ok");

  drawArm(data);
}

async function poll() {
  try {
    const data = await fetchState();
    updateUi(data);
  } catch (e) {
    conn.textContent = "OFFLINE";
    conn.className = "badge fault";
    stateBox.textContent = String(e);
  }
}

async function post(url, body) {
  const opts = body
    ? { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) }
    : { method: "POST" };
  const r = await fetch(url, opts);
  const text = await r.text();
  let json;
  try {
    json = JSON.parse(text);
  } catch {
    json = { raw: text };
  }
  if (!r.ok) throw new Error(json.detail || json.raw || r.statusText);
  return json;
}

gcodeForm.addEventListener("submit", async (ev) => {
  ev.preventDefault();
  const line = gcodeInput.value.trim();
  if (!line) return;
  gcodeResult.textContent = "…";
  try {
    const res = await post("/api/gcode", { line });
    gcodeResult.textContent = JSON.stringify(res, null, 2);
    await poll();
  } catch (e) {
    gcodeResult.textContent = String(e);
  }
});

document.getElementById("btn-home").onclick = () => {
  gcodeInput.value = "G28";
  gcodeForm.requestSubmit();
};

document.getElementById("btn-reset").onclick = async () => {
  gcodeResult.textContent = await post("/api/reset-fault").then((r) => JSON.stringify(r, null, 2));
  await poll();
};

document.getElementById("btn-estop").onclick = async () => {
  gcodeResult.textContent = await post("/api/estop").then((r) => JSON.stringify(r, null, 2));
  await poll();
};

setInterval(poll, 400);
poll();
