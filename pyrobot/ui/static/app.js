const stateBox = document.getElementById("state-box");
const poseLine = document.getElementById("pose-line");
const conn = document.getElementById("conn");
const gcodeForm = document.getElementById("gcode-form");
const gcodeInput = document.getElementById("gcode-input");
const gcodeResult = document.getElementById("gcode-result");
const canvas = document.getElementById("arm-canvas");
const ctx = canvas.getContext("2d");
const voiceStatus = document.getElementById("voice-status");
const yoloLine = document.getElementById("yolo-line");

const STREAMS = ["rgb", "depth", "mask", "track"];

/** Encoder A=90° at home → link along +X (to the right on screen). */
const ARM_A_OFFSET_RAD = Math.PI / 2;

let jogStepMm = 10;
let voiceFeedMmMin = 800;
let lastState = null;
let voiceListening = false;
let speechRec = null;
let gestureBusy = false;

const FEED_STORAGE_KEY = "vitalv3_voice_feed_mm_min";

async function fetchState() {
  const r = await fetch("/api/state");
  if (!r.ok) throw new Error(r.statusText);
  return r.json();
}

function tipAt(ox, oy, lenPx, angRad) {
  return {
    x: ox + lenPx * Math.cos(angRad),
    y: oy - lenPx * Math.sin(angRad),
  };
}

function encoderToScreenRad(aDeg, bDeg) {
  const aRad = (aDeg * Math.PI) / 180;
  const bRad = (bDeg * Math.PI) / 180;
  const screenA = ARM_A_OFFSET_RAD - aRad;
  return { screenA, screenB: screenA + bRad };
}

function drawAngleArc(cx, cy, r, startRad, endRad, color) {
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.arc(cx, cy, r, -startRad, -endRad, endRad < startRad);
  ctx.stroke();
}

function drawArm(data) {
  const w = canvas.width;
  const h = canvas.height;
  ctx.clearRect(0, 0, w, h);

  const link = data.link_length_mm || 250;
  const q = data.joints_deg || { a: 90, b: 90, c: 0, d: 0 };
  const scale = 0.55;
  const bx = 40;
  const by = h - 48;
  const lenPx = link * scale;

  const { screenA, screenB } = encoderToScreenRad(q.a, q.b);

  const base = { x: bx, y: by };
  const elbow = tipAt(base.x, base.y, lenPx, screenA);
  const end = tipAt(elbow.x, elbow.y, lenPx, screenB);

  ctx.strokeStyle = "#2d3a4f";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(16, by);
  ctx.lineTo(w - 16, by);
  ctx.stroke();

  ctx.fillStyle = "#5a6a85";
  ctx.font = "10px system-ui";
  ctx.fillText("+X →", w - 52, by + 14);

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

  drawAngleArc(base.x, base.y, 28, 0, screenA, "#f39c12");
  drawAngleArc(elbow.x, elbow.y, 22, screenA, screenB, "#3d8bfd");

  const r = 7;
  for (const [pt, color] of [
    [base, "#8b9cb3"],
    [elbow, "#f39c12"],
    [end, "#2ecc71"],
  ]) {
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(pt.x, pt.y, r, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.fillStyle = "#8b9cb3";
  ctx.font = "11px system-ui";
  ctx.fillText("база", base.x + 10, base.y + 4);
  ctx.fillStyle = "#f39c12";
  ctx.fillText(`A=${q.a.toFixed(1)}°`, base.x + 34, base.y - 18);
  ctx.fillStyle = "#3d8bfd";
  ctx.fillText(`B=${q.b.toFixed(1)}°`, elbow.x + 10, elbow.y - 12);
  ctx.fillStyle = "#2ecc71";
  ctx.fillText("TCP", end.x + 8, end.y);
}

function refreshStreams() {
  const t = Date.now();
  for (const name of STREAMS) {
    const img = document.getElementById(`stream-${name}`);
    if (img) img.src = `/api/frame/${name}?t=${t}`;
  }
}

function formatYoloLine(vis) {
  const dets = vis?.detections || [];
  if (!dets.length) return "YOLO: —";
  const parts = dets
    .slice(0, 6)
    .map((d) => `${d.class_name} ${(d.confidence * 100).toFixed(0)}%`);
  const more = dets.length > 6 ? ` (+${dets.length - 6})` : "";
  return `YOLO: ${parts.join(", ")}${more}`;
}

function updateUi(data) {
  lastState = data;
  if (data.ui?.jog_step_mm != null) jogStepMm = data.ui.jog_step_mm;
  if (data.ui?.voice_feed_mm_min != null && !localStorage.getItem(FEED_STORAGE_KEY)) {
    voiceFeedMmMin = data.ui.voice_feed_mm_min;
    syncFeedSlider();
  }

  const m = data.motion || {};
  const vis = data.vision || {};
  stateBox.textContent = JSON.stringify(data, null, 2);
  poseLine.textContent =
    `TCP: X=${data.pose_mm.x.toFixed(1)} Y=${data.pose_mm.y.toFixed(1)} Z=${data.pose_mm.z.toFixed(1)} mm · ` +
    `A=${data.joints_deg.a.toFixed(2)}° B=${data.joints_deg.b.toFixed(2)}° C=${data.joints_deg.c.toFixed(2)}°`;

  const tofEl = document.getElementById("tof-line");
  if (tofEl) {
    const tof = vis.tof_distance_mm;
    tofEl.textContent = tof != null ? `ToF Z≈${tof.toFixed(0)} mm` : "ToF: —";
  }
  if (yoloLine) yoloLine.textContent = formatYoloLine(vis);

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

const VOICE_DIRECTIONS = [
  { words: ["вперёд", "вперед", "forward", "впереди"], delta: (mm) => [mm, 0, 0] },
  { words: ["назад", "back", "backward"], delta: (mm) => [-mm, 0, 0] },
  { words: ["вправо", "right"], delta: (mm) => [0, mm, 0] },
  { words: ["влево", "left"], delta: (mm) => [0, -mm, 0] },
  { words: ["вверх", "up", "выше"], delta: (mm) => [0, 0, mm] },
  { words: ["вниз", "down", "ниже"], delta: (mm) => [0, 0, -mm] },
];

function normalizeVoicePhrase(phrase) {
  return phrase
    .trim()
    .toLowerCase()
    .replace(/,/g, ".")
    .replace(/\s+/g, " ");
}

/** Extract mm from «вперед 30», «30 вперед» or use default step. */
function parseVoiceAmount(text, defaultMm) {
  const after = text.match(/\s(\d+(?:\.\d+)?)\s*$/);
  if (after) return parseFloat(after[1]);
  const before = text.match(/^(\d+(?:\.\d+)?)\s+/);
  if (before) return parseFloat(before[1]);
  return defaultMm;
}

/** Map phrase → jog delta [dx, dy, dz] in mm, or special token. */
function voiceToJog(phrase) {
  const t = normalizeVoicePhrase(phrase);
  if (/^(домой|home|главная)$/.test(t)) return "home";
  if (/^(стоп|stop|эстоп|estop)$/.test(t)) return "estop";

  for (const dir of VOICE_DIRECTIONS) {
    for (const word of dir.words) {
      const reAfter = new RegExp(`^${word}(?:\\s+(\\d+(?:\\.\\d+)?))?$`, "i");
      const reBefore = new RegExp(`^(\\d+(?:\\.\\d+)?)\\s+${word}$`, "i");
      if (reAfter.test(t) || reBefore.test(t)) {
        const mm = parseVoiceAmount(t, jogStepMm);
        if (!(mm > 0)) return null;
        return dir.delta(mm);
      }
    }
  }
  return null;
}

function parseVoiceIntent(phrase) {
  const t = normalizeVoicePhrase(phrase);

  const see = t.match(/^(?:ты\s+)?видишь\s+(.+)$/);
  if (see) return { kind: "see", object: see[1].trim() };

  if (/^(привет|здравствуй|hello|hi)$/.test(t)) return { kind: "gesture", gesture: "hello" };
  if (/^(пока|до свидания|bye|goodbye)$/.test(t)) return { kind: "gesture", gesture: "bye" };
  if (/^(да|yes|ага|конечно)$/.test(t)) return { kind: "gesture", gesture: "nod" };
  if (/^(нет|no|неа)$/.test(t)) return { kind: "gesture", gesture: "shake" };
  if (/^(танцуй|dance|потанцуй)$/.test(t)) return { kind: "gesture", gesture: "dance" };
  if (/^(спасибо|thanks|thank you)$/.test(t)) return { kind: "gesture", gesture: "thanks" };
  if (/^(подтверди|кивая|кивни)$/.test(t)) return { kind: "gesture", gesture: "nod" };

  const jog = voiceToJog(phrase);
  if (jog === "home") return { kind: "home" };
  if (jog === "estop") return { kind: "estop" };
  if (Array.isArray(jog)) return { kind: "jog", delta: jog };
  return null;
}

/** Russian spoken name → COCO / YOLO class_name (English). */
const YOLO_RU_ALIASES = {
  чашка: "cup",
  чашку: "cup",
  кружка: "cup",
  кружку: "cup",
  человек: "person",
  человека: "person",
  людей: "person",
  бутылка: "bottle",
  бутылку: "bottle",
  телефон: "cell phone",
  стул: "chair",
  ноутбук: "laptop",
  книга: "book",
  книгу: "book",
  собака: "dog",
  кошка: "cat",
  кот: "cat",
  ножницы: "scissors",
  ножниц: "scissors",
  ножница: "scissors",
  ножницыми: "scissors",
  вилка: "fork",
  вилку: "fork",
  ложка: "spoon",
  ложку: "spoon",
  нож: "knife",
  ножа: "knife",
  яблоко: "apple",
  банан: "banana",
  апельсин: "orange",
  машина: "car",
  автомобиль: "car",
  велосипед: "bicycle",
  самолёт: "airplane",
  самолет: "airplane",
  птица: "bird",
  лошадь: "horse",
  корова: "cow",
  слон: "elephant",
  мишка: "teddy bear",
  медведь: "teddy bear",
  часы: "clock",
  ваза: "vase",
  раковина: "sink",
  холодильник: "refrigerator",
  телевизор: "tv",
  телик: "tv",
  пицца: "pizza",
  торшер: "tie",
  галстук: "tie",
  рюкзак: "backpack",
  зонт: "umbrella",
  зонтик: "umbrella",
  очки: "eyeglasses",
  клавиатура: "keyboard",
  мышь: "mouse",
  мышка: "mouse",
  пульт: "remote",
  диван: "couch",
  кровать: "bed",
  стол: "dining table",
  туалет: "toilet",
  растение: "potted plant",
  цветок: "potted plant",
  лампа: "traffic light",
  светофор: "traffic light",
};

function normalizeSeeObject(name) {
  return name
    .toLowerCase()
    .trim()
    .replace(/^[«"'"]+|[»"'".?!,]+$/g, "")
    .trim();
}

function yoloQueryTerms(name) {
  const raw = normalizeSeeObject(name);
  const terms = new Set([raw]);
  const en = YOLO_RU_ALIASES[raw];
  if (en) {
    terms.add(en);
    for (const w of en.split(/\s+/)) terms.add(w);
  }
  return [...terms];
}

function yoloClassMatchesTerms(cls, terms) {
  const c = cls.toLowerCase();
  const clsWords = c.split(/\s+/);
  for (const needle of terms) {
    if (!needle) continue;
    if (c === needle) return true;
    if (c.includes(needle) || needle.includes(c)) return true;
    const nw = needle.split(/\s+/);
    if (nw.length > 1 && nw.every((w) => clsWords.includes(w))) return true;
    if (nw.length === 1 && clsWords.some((w) => w === nw || w.startsWith(nw) || nw.startsWith(w))) {
      return true;
    }
  }
  return false;
}

function yoloSeesObject(name) {
  const terms = yoloQueryTerms(name);
  const dets = lastState?.vision?.detections || [];
  return dets.some((d) => yoloClassMatchesTerms(d.class_name || "", terms));
}

function yoloVisibleClassList() {
  const dets = lastState?.vision?.detections || [];
  return [...new Set(dets.map((d) => d.class_name).filter(Boolean))];
}

async function ensurePose() {
  if (lastState?.pose_mm) return lastState.pose_mm;
  const data = await fetchState();
  lastState = data;
  return data.pose_mm;
}

function buildG1Line(pose, dx, dy, dz, feed) {
  const f = Math.round(feed);
  const x = pose.x + dx;
  const y = pose.y + dy;
  const z = pose.z + dz;
  return `G1 X${x.toFixed(1)} Y${y.toFixed(1)} Z${z.toFixed(1)} F${f}`;
}

async function sendG1Relative(dx, dy, dz, feed = voiceFeedMmMin) {
  const pose = await ensurePose();
  const line = buildG1Line(pose, dx, dy, dz, feed);
  gcodeInput.value = line;
  const res = await post("/api/gcode", { line });
  await poll();
  return res;
}

async function runGestureNod(feed) {
  const dz = 16;
  for (const d of [dz, -dz, dz, -dz, 0]) {
    await sendG1Relative(0, 0, d, feed);
  }
}

async function runGestureShake(feed) {
  const dy = 20;
  for (const d of [dy, -2 * dy, 2 * dy, -dy, 0]) {
    await sendG1Relative(0, d, 0, feed);
  }
}

async function runGestureHello(feed) {
  await runGestureNod(feed);
}

async function runGestureBye(feed) {
  await runGestureShake(feed);
}

async function runGestureDance(feed) {
  await runGestureShake(feed);
  await runGestureNod(feed);
}

async function runGesture(intent, feed) {
  gestureBusy = true;
  try {
    switch (intent.gesture) {
      case "nod":
        await runGestureNod(feed);
        break;
      case "shake":
        await runGestureShake(feed);
        break;
      case "hello":
        await runGestureHello(feed);
        break;
      case "bye":
        await runGestureBye(feed);
        break;
      case "dance":
        await runGestureDance(feed);
        break;
      case "thanks":
        await runGestureNod(feed);
        break;
      default:
        break;
    }
  } finally {
    gestureBusy = false;
  }
}

async function runSeeObject(objectName, feed) {
  const data = await fetchState();
  lastState = data;

  const seen = yoloSeesObject(objectName);
  const visible = yoloVisibleClassList();
  const mapped = YOLO_RU_ALIASES[normalizeSeeObject(objectName)];

  if (seen) {
    voiceStatus.textContent = `Вижу «${objectName}»${mapped ? ` (${mapped})` : ""} — киваю`;
    await runGestureNod(feed);
  } else {
    const hint = visible.length ? ` Сейчас YOLO: ${visible.join(", ")}.` : " YOLO пусто.";
    voiceStatus.textContent = `Не вижу «${objectName}»${mapped ? ` (ищу ${mapped})` : ""} — мотаю головой.${hint}`;
    await runGestureShake(feed);
  }
}

async function runVoiceCommand(phrase) {
  if (gestureBusy) {
    voiceStatus.textContent = "Жест выполняется…";
    return;
  }

  const intent = parseVoiceIntent(phrase);
  if (intent == null) {
    voiceStatus.textContent = `Не понял: «${phrase}»`;
    return;
  }

  const feed = voiceFeedMmMin;

  if (intent.kind === "home") {
    gcodeInput.value = "G28";
    gcodeForm.requestSubmit();
    voiceStatus.textContent = "G28 Home";
    return;
  }
  if (intent.kind === "estop") {
    await post("/api/estop");
    voiceStatus.textContent = "E-STOP";
    await poll();
    return;
  }
  if (intent.kind === "see") {
    gcodeResult.textContent = "…";
    try {
      await runSeeObject(intent.object, feed);
      gcodeResult.textContent = voiceStatus.textContent;
    } catch (e) {
      gcodeResult.textContent = String(e);
    }
    return;
  }
  if (intent.kind === "gesture") {
    gcodeResult.textContent = "…";
    voiceStatus.textContent = `Жест: ${intent.gesture}`;
    try {
      await runGesture(intent, feed);
      gcodeResult.textContent = "OK";
    } catch (e) {
      gcodeResult.textContent = String(e);
    }
    return;
  }

  const [dx, dy, dz] = intent.delta;
  const pose = await ensurePose();
  const line = buildG1Line(pose, dx, dy, dz, feed);
  gcodeInput.value = line;
  voiceStatus.textContent = `→ ${line}`;
  gcodeResult.textContent = "…";
  try {
    const res = await post("/api/gcode", { line });
    gcodeResult.textContent = JSON.stringify(res, null, 2);
    await poll();
  } catch (e) {
    gcodeResult.textContent = String(e);
  }
}

function syncFeedSlider() {
  const slider = document.getElementById("feed-slider");
  const label = document.getElementById("feed-value");
  if (!slider || !label) return;
  slider.value = String(Math.round(voiceFeedMmMin));
  label.textContent = String(Math.round(voiceFeedMmMin));
}

function initFeedSlider() {
  const stored = localStorage.getItem(FEED_STORAGE_KEY);
  if (stored) voiceFeedMmMin = parseFloat(stored) || voiceFeedMmMin;

  const slider = document.getElementById("feed-slider");
  const label = document.getElementById("feed-value");
  if (!slider || !label) return;

  syncFeedSlider();
  slider.addEventListener("input", () => {
    voiceFeedMmMin = parseFloat(slider.value);
    label.textContent = String(Math.round(voiceFeedMmMin));
    localStorage.setItem(FEED_STORAGE_KEY, String(voiceFeedMmMin));
  });
}

function initVoice() {
  const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
  if (!SR) {
    voiceStatus.textContent = "Голос недоступен (нужен Chrome / Safari)";
    document.getElementById("btn-voice")?.setAttribute("disabled", "true");
    return;
  }
  speechRec = new SR();
  speechRec.lang = "ru-RU";
  speechRec.interimResults = false;
  speechRec.maxAlternatives = 1;

  speechRec.onresult = (ev) => {
    const text = ev.results[0][0].transcript;
    voiceStatus.textContent = `Слышу: ${text}`;
    runVoiceCommand(text);
  };
  speechRec.onerror = (ev) => {
    voiceStatus.textContent = `Ошибка: ${ev.error}`;
    voiceListening = false;
    document.getElementById("btn-voice")?.classList.remove("active");
  };
  speechRec.onend = () => {
    voiceListening = false;
    document.getElementById("btn-voice")?.classList.remove("active");
    if (voiceStatus.textContent.startsWith("Слушаю")) {
      voiceStatus.textContent = "Готов";
    }
  };
}

function toggleVoice() {
  if (!speechRec) return;
  if (voiceListening) {
    speechRec.stop();
    return;
  }
  voiceListening = true;
  voiceStatus.textContent = "Слушаю… (например: «вперед 20», «влево 15»)";
  document.getElementById("btn-voice")?.classList.add("active");
  try {
    speechRec.start();
  } catch {
    speechRec.stop();
    setTimeout(() => speechRec.start(), 200);
  }
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

document.getElementById("btn-zero-enc").onclick = async () => {
  const ok = confirm(
    "Zero encoders\n\n" +
      "Поставьте манипулятор в позу HOME (A≈90°, B≈90°), затем OK.\n\n" +
      "Будет AT+ZERO на энкодерах A/B и запись offset в encoders_offsets.json."
  );
  if (!ok) return;
  gcodeResult.textContent = "Zero encoders…";
  try {
    const res = await post("/api/zero-encoders");
    gcodeResult.textContent = JSON.stringify(res, null, 2);
    await poll();
  } catch (e) {
    gcodeResult.textContent = String(e);
  }
};

document.getElementById("btn-reset").onclick = async () => {
  gcodeResult.textContent = await post("/api/reset-fault").then((r) => JSON.stringify(r, null, 2));
  await poll();
};

document.getElementById("btn-estop").onclick = async () => {
  gcodeResult.textContent = await post("/api/estop").then((r) => JSON.stringify(r, null, 2));
  await poll();
};

document.getElementById("btn-voice")?.addEventListener("click", toggleVoice);

const voiceEmuForm = document.getElementById("voice-emulator-form");
const voiceEmuInput = document.getElementById("voice-emulator-input");
if (voiceEmuForm && voiceEmuInput) {
  voiceEmuForm.addEventListener("submit", (ev) => {
    ev.preventDefault();
    const text = voiceEmuInput.value.trim();
    if (!text) return;
    voiceStatus.textContent = `Эмулятор: «${text}»`;
    runVoiceCommand(text);
  });
}

initVoice();
initFeedSlider();

setInterval(poll, 400);
setInterval(refreshStreams, 500);
poll();
refreshStreams();
