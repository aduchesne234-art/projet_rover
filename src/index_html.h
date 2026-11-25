// index_html.h - the phone control page, served by the ESP32 at http://192.168.4.1
#pragma once
#include <Arduino.h>

static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Rover</title>
<style>
  * { box-sizing: border-box; -webkit-user-select: none; user-select: none; }
  html, body { margin: 0; height: 100%; background: #14171c; color: #e8eaed;
               font-family: system-ui, sans-serif; overflow: hidden; }
  body { display: flex; flex-direction: column; align-items: center; gap: 10px; padding: 10px; }
  .row { display: flex; gap: 8px; align-items: center; justify-content: center; width: 100%; max-width: 420px; }
  #conn { width: 12px; height: 12px; border-radius: 50%; background: #d33; }
  #conn.on { background: #3c6; }
  #state { font-size: 14px; opacity: .8; flex: 1; text-align: right; }
  #camBox { width: 100%; max-width: 420px; height: 28vh; background: #000; border-radius: 8px;
            display: flex; align-items: center; justify-content: center; font-size: 13px; opacity: .95; }
  #cam { max-width: 100%; max-height: 100%; object-fit: contain; }
  button { flex: 1; padding: 12px 0; font-size: 16px; border: 0; border-radius: 10px;
           background: #2a2f38; color: inherit; }
  button.active { background: #2e7d4f; }
  #stop { background: #b3261e; font-weight: 700; font-size: 19px; }
  #pad { position: relative; width: min(60vw, 32vh, 280px); aspect-ratio: 1; border-radius: 50%;
         background: #20252d; border: 2px solid #343a45; touch-action: none; }
  #pad.disabled { opacity: .35; pointer-events: none; }
  #knob { position: absolute; left: 50%; top: 50%; width: 22%; height: 22%; margin: -11% 0 0 -11%;
          border-radius: 50%; background: #5b8def; pointer-events: none; }
  #oFront, #oWheels { flex: 1; text-align: center; padding: 8px 0; border-radius: 8px; background: #20252d; font-size: 15px; }
  #oFront.hit { background: #b3261e; }
  input[type=range] { flex: 1; }
</style>
</head>
<body>
  <div class="row"><div id="conn"></div><span>Rover</span><span id="state">connecting...</span></div>
  <div id="camBox"><img id="cam" alt=""><span id="camMsg"></span></div>
  <div class="row">
    <button id="bManual" class="active">Manual</button>
    <button id="bAuto">Auto</button>
    <button id="bCam">Camera</button>
  </div>
  <div class="row"><div id="oFront">Front: --</div><div id="oWheels">Wheels: --</div></div>
  <div id="pad"><div id="knob"></div></div>
  <div class="row"><span>Speed</span><input id="speed" type="range" min="20" max="100" value="70"></div>
  <div class="row"><button id="stop">STOP</button></div>

<script>
const $ = id => document.getElementById(id);
let mode = "manual", wantMode = null, jx = 0, jy = 0, busy = false;

// ---------- camera (MJPEG stream on port 81) ----------
let camOn = true;
function setCam(on) {
  camOn = on;
  $("bCam").classList.toggle("active", on);
  $("camMsg").textContent = on ? "" : "camera off";
  $("cam").style.display = on ? "" : "none";
  $("cam").src = on ? "http://" + location.hostname + ":81/stream?" + Date.now() : "";
}
$("cam").onerror = () => { if (camOn) { $("cam").style.display = "none"; $("camMsg").textContent = "camera unavailable"; } };
$("cam").onload = () => { $("camMsg").textContent = ""; $("cam").style.display = ""; };
$("bCam").onclick = () => setCam(!camOn);

// ---------- joystick ----------
const pad = $("pad"), knob = $("knob");
let active = false;
function move(e) {
  const r = pad.getBoundingClientRect(), max = r.width / 2;
  let dx = e.clientX - (r.left + max), dy = e.clientY - (r.top + max);
  const d = Math.hypot(dx, dy);
  if (d > max) { dx *= max / d; dy *= max / d; }
  knob.style.transform = `translate(${dx}px, ${dy}px)`;
  jx = dx / max; jy = dy / max;
}
function release() { active = false; jx = jy = 0; knob.style.transform = "translate(0,0)"; }
pad.addEventListener("pointerdown", e => { active = true; pad.setPointerCapture(e.pointerId); move(e); });
pad.addEventListener("pointermove", e => { if (active) move(e); });
pad.addEventListener("pointerup", release);
pad.addEventListener("pointercancel", release);

// joystick -> left/right motor values (arcade mixing)
function mix() {
  const f = -jy, t = jx;                 // up = forward, right = turn right
  let l = f + t, r = f - t;
  const m = Math.max(1, Math.abs(l), Math.abs(r));
  const k = $("speed").value / 100;
  return { l: +(l / m * k).toFixed(2), r: +(r / m * k).toFixed(2) };
}

// ---------- control loop: one small HTTP request every 60 ms ----------
// In manual mode it carries the joystick values; the rover stops if they stop arriving.
// The reply is the rover status.
async function tick() {
  if (busy) return;
  busy = true;
  const q = [];
  if (mode === "manual") { const m = mix(); q.push("l=" + m.l, "r=" + m.r); }
  const sent = wantMode;
  if (sent) q.push("mode=" + sent);
  const ctl = new AbortController();
  const to = setTimeout(() => ctl.abort(), 400);
  try {
    const res = await fetch("/api?" + q.join("&"), { signal: ctl.signal, cache: "no-store" });
    const s = await res.json();
    if (wantMode === sent) wantMode = null;
    mode = s.mode;
    $("conn").classList.add("on");
    $("state").textContent = s.mode + " / " + s.state + (s.imu ? "" : " (no gyro)");
    $("bManual").classList.toggle("active", mode === "manual");
    $("bAuto").classList.toggle("active", mode === "auto");
    $("pad").classList.toggle("disabled", mode === "auto");
    $("oFront").textContent = "Front: " + (s.distance < 0 ? "> 200 cm" : s.distance + " cm");
    $("oFront").classList.toggle("hit", s.front);
    $("oWheels").textContent = "L " + s.rpmL + " | R " + s.rpmR + " rpm";
  } catch (e) {
    $("conn").classList.remove("on");
    $("state").textContent = "disconnected";
  } finally {
    clearTimeout(to);
    busy = false;
  }
}
setInterval(tick, 60);

// ---------- buttons ----------
$("bManual").onclick = () => { wantMode = "manual"; };
$("bAuto").onclick = () => { release(); wantMode = "auto"; };
$("stop").onclick = () => { release(); wantMode = "manual"; };

setCam(true);
</script>
</body>
</html>
)rawliteral";
