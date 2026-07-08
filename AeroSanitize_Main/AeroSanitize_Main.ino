#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "DHT.h"

// ── PIN MAP ──────────────────────────────────────────────────────────────────
#define RELAY_PIN   14   // UV-C lamp relay        HIGH = lamp ON
#define RADAR_PIN    5   // Radar OUT              HIGH = motion
#define DOOR_PIN    12   // Reed switch            INPUT_PULLUP, LOW = closed
#define DHT_PIN     15   // DHT22 ambient temp     optional, not in safety logic
#define DHTTYPE     DHT22

// ── TIMING CONSTANTS ─────────────────────────────────────────────────────────
const unsigned long UVC_DURATION_MS     = 60000UL; // UV-C cycle          60 s
const unsigned long SCAN_DURATION_MS    = 10000UL; // Pre-ignition scan   10 s
const unsigned long AUTO_START_DELAY_MS = 10000UL; // Room clear → auto   10 s
const unsigned long SENSOR_INTERVAL_MS  =   300UL; // Re-read sensors    0.3 s

// ── OBJECTS ──────────────────────────────────────────────────────────────────
WebServer        server(80);
Adafruit_AMG88xx amg;
DHT              dht(DHT_PIN, DHTTYPE);

// ── STATE MACHINE ─────────────────────────────────────────────────────────────
enum SystemState { STANDBY, SCANNING, UVC_ACTIVE, UNSAFE };
SystemState currentState = STANDBY;

// ── SENSOR GLOBALS ────────────────────────────────────────────────────────────
float currentThermal = 0.0f;  // Hottest AMG8833 pixel (°C)
bool  radarMotion    = false;  // true = motion detected
bool  doorOpen       = false;  // true = door is open
bool  systemIsSafe   = false;  // composite: all three sensors must pass

// ── COMPUTED STATE (read by handleData, written by runStateMachine) ────────────
unsigned long countdownLeft  = 0;   // ms left in UV-C cycle
unsigned long scanTimeLeft   = 0;   // ms left in safety scan
unsigned long autoStartInMs  = 0;   // ms until auto-start (0 = not counting)

// ── INTERNAL TIMERS ───────────────────────────────────────────────────────────
unsigned long uvcStartTime   = 0;
unsigned long scanStartTime  = 0;
unsigned long lastSensorRead = 0;
unsigned long roomSafeStart  = 0;   // when room first became continuously safe
bool          roomWasSafe    = false;

// ─────────────────────────────────────────────────────────────────────────────
// DASHBOARD HTML (stored in flash via PROGMEM, not eaten from RAM)
// ─────────────────────────────────────────────────────────────────────────────
const char DASHBOARD_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#090d12">
<title>Aero-Sanitize AI</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#090d12;--sf:#101820;--sf2:#0b1219;--bd:#1a2636;
  --t1:#d8e8f2;--t2:#556c7e;--t3:#2e404f;
  --g:#00d98b;--y:#f5a422;--r:#ff3d54;--b:#38b2ff;
  --rr:13px;
  --mono:ui-monospace,"Cascadia Mono","Consolas",monospace
}
html{background:var(--bg)}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
  background:var(--bg);color:var(--t1);min-height:100svh;
  -webkit-font-smoothing:antialiased;overscroll-behavior:none}
.pg{max-width:420px;margin:0 auto;padding:16px 14px calc(20px + env(safe-area-inset-bottom))}

/* ── HEADER ── */
.hdr{display:flex;align-items:center;justify-content:space-between;margin-bottom:14px}
.brand{display:flex;align-items:center;gap:7px;font-size:10.5px;font-weight:800;
  letter-spacing:.18em;color:var(--t2);text-transform:uppercase}
.bdot{width:7px;height:7px;border-radius:50%;background:var(--b);
  box-shadow:0 0 7px var(--b);flex:none}
.pill{display:flex;align-items:center;gap:5px;background:var(--sf);
  border:1px solid var(--bd);border-radius:99px;padding:4px 10px;
  font-family:var(--mono);font-size:9px;font-weight:700;letter-spacing:.06em;color:var(--t2)}
.pdot{width:5px;height:5px;border-radius:50%;background:var(--g);
  box-shadow:0 0 5px var(--g);animation:blink 2s ease infinite;flex:none}
.pill.off .pdot{background:var(--y);box-shadow:0 0 5px var(--y);animation-duration:.65s}

/* ── STATUS CARD ── */
.sc{background:var(--sf);border:1px solid var(--bd);border-radius:var(--rr);
  padding:15px;margin-bottom:11px;transition:border-color .25s;
  position:relative;overflow:hidden}
.sc::before{content:"";position:absolute;left:0;top:0;bottom:0;width:3px;
  background:var(--t3);transition:background .25s}
.sc.s-ready::before{background:var(--g)}
.sc.s-scan::before{background:var(--y)}
.sc.s-active::before,.sc.s-unsafe::before{background:var(--r)}
.sc.s-active{animation:aP 2s ease infinite}
.sc.s-unsafe{animation:aP 1s ease infinite}
.si{display:flex;align-items:flex-start;gap:12px}
.sic{width:38px;height:38px;border-radius:10px;background:var(--sf2);
  display:flex;align-items:center;justify-content:center;flex:none;transition:background .25s}
.sc.s-ready .sic{background:rgba(0,217,139,.13)}
.sc.s-scan  .sic{background:rgba(245,164,34,.13)}
.sc.s-active .sic,.sc.s-unsafe .sic{background:rgba(255,61,84,.13)}
.sic svg{width:19px;height:19px;color:var(--t3);transition:color .25s}
.sc.s-ready  .sic svg{color:var(--g)}
.sc.s-scan   .sic svg{color:var(--y)}
.sc.s-active .sic svg,.sc.s-unsafe .sic svg{color:var(--r)}
.stx{flex:1;min-width:0}
.stitle{font-size:13px;font-weight:700;letter-spacing:.01em}
.ssub{font-size:11.5px;color:var(--t2);margin-top:3px;line-height:1.4}

/* ── AUTO-START PROGRESS BAR ── */
.asw{margin-top:11px;padding-top:11px;border-top:1px solid var(--bd);display:none}
.asw.vis{display:block}
.aslb{font-size:9px;font-weight:800;letter-spacing:.12em;color:var(--t2);
  text-transform:uppercase;margin-bottom:5px;
  display:flex;justify-content:space-between;align-items:center}
.aslb span{font-family:var(--mono);color:var(--g);font-size:10px}
.asbar{height:3px;background:var(--bd);border-radius:99px;overflow:hidden}
.asfill{height:100%;background:var(--g);border-radius:99px;width:0;transition:width .25s linear}

/* ── SENSOR CARDS ── */
.sr{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:11px}
.scard{background:var(--sf);border:1px solid var(--bd);border-radius:var(--rr);
  padding:11px 7px 9px;text-align:center;transition:border-color .25s,background .25s}
.scard.al{border-color:rgba(255,61,84,.45);background:rgba(255,61,84,.05)}
.slbl{display:block;font-size:8.5px;font-weight:800;letter-spacing:.14em;
  color:var(--t3);text-transform:uppercase;margin-bottom:7px}
.sv{display:block;font-family:var(--mono);font-size:11.5px;font-weight:700;
  color:var(--t1);margin-top:6px;transition:color .25s}
.scard.al .sv{color:var(--r)}

/* radar glyph */
.rr{width:34px;height:34px;border-radius:50%;border:1px solid var(--bd);
  background:var(--sf2);position:relative;overflow:hidden;margin:0 auto}
.rsw{position:absolute;inset:0;
  background:conic-gradient(from 0deg,transparent 0deg,var(--b) 22deg,transparent 44deg);
  animation:spin 2.8s linear infinite;opacity:.5}
.scard.al .rsw{
  background:conic-gradient(from 0deg,transparent 0deg,var(--r) 28deg,transparent 56deg);
  opacity:.95;animation-duration:.85s}
.rc{position:absolute;top:50%;left:50%;width:4px;height:4px;
  margin:-2px;border-radius:50%;background:var(--t3)}

/* thermal bar */
.tb{width:10px;height:34px;border-radius:5px;background:var(--sf2);
  border:1px solid var(--bd);position:relative;overflow:hidden;margin:0 auto}
.tf{position:absolute;bottom:0;left:0;right:0;height:0%;
  background:linear-gradient(to top,var(--r),var(--y) 50%,var(--g));transition:height .5s ease}
.tline{position:absolute;bottom:72%;left:-2px;right:-2px;height:1px;
  background:var(--y);opacity:.7}

/* door glyph */
.dframe{width:28px;height:36px;margin:0 auto;border:1.5px solid var(--bd);
  border-radius:3px;background:var(--sf2);position:relative;overflow:hidden}
.dleaf{position:absolute;top:3px;left:3px;right:3px;bottom:3px;background:var(--t3);
  border-radius:2px;transform-origin:left center;
  transition:transform .45s cubic-bezier(.3,.7,.4,1),background .25s}
.dleaf.op{transform:scaleX(.12);background:var(--r)}
.dknob{position:absolute;right:4px;top:50%;width:3px;height:3px;
  margin-top:-1.5px;border-radius:50%;background:var(--sf2)}

/* ── TIMER RING ── */
.tmrsec{background:var(--sf);border:1px solid var(--bd);border-radius:var(--rr);
  padding:20px 16px;margin-bottom:11px;
  display:none;flex-direction:column;align-items:center;transition:border-color .25s}
.tmrsec.s-scan{border-color:rgba(245,164,34,.3)}
.tmrsec.s-active{border-color:rgba(255,61,84,.3)}
.rw{position:relative;width:148px;height:148px}
.rsvg{width:100%;height:100%;transform:rotate(-90deg)}
.rtrack{fill:none;stroke:var(--bd);stroke-width:5}
.rarc{fill:none;stroke:var(--b);stroke-width:5;stroke-linecap:round;
  stroke-dasharray:301.6;stroke-dashoffset:301.6;
  transition:stroke-dashoffset .5s linear,stroke .25s}
.tmrsec.s-scan   .rarc{stroke:var(--y)}
.tmrsec.s-active .rarc{stroke:var(--r)}
.rc2{position:absolute;inset:0;display:flex;flex-direction:column;
  align-items:center;justify-content:center}
.rtime{font-family:var(--mono);font-size:28px;font-weight:700}
.rlbl{font-size:9px;font-weight:800;letter-spacing:.14em;
  color:var(--t2);margin-top:5px;text-transform:uppercase}

/* ── BUTTONS ── */
.br{display:flex;flex-direction:column;gap:8px;margin-bottom:12px}
.btn{display:flex;align-items:center;justify-content:center;gap:8px;padding:15px;
  border:none;border-radius:var(--rr);font-family:inherit;font-size:14px;
  font-weight:700;letter-spacing:.03em;cursor:pointer;
  transition:opacity .15s,transform .1s;-webkit-tap-highlight-color:transparent}
.btn:active{transform:scale(.97)}
.btn:disabled{opacity:.3;cursor:not-allowed;transform:none}
.bstart{background:var(--g);color:#012b1b}
.bstart:not(:disabled):hover{opacity:.88}
.bstop{background:var(--r);color:#fff}
.bstop:hover{opacity:.88}

.ft{text-align:center;font-family:var(--mono);font-size:9px;
  color:var(--t3);letter-spacing:.04em}

/* ── ANIMATIONS ── */
@keyframes blink{0%,100%{opacity:1}50%{opacity:.25}}
@keyframes spin{to{transform:rotate(360deg)}}
@keyframes aP{0%,100%{border-color:rgba(255,61,84,.2)}50%{border-color:rgba(255,61,84,.65)}}
@media(prefers-reduced-motion:reduce){
  *,*::before,*::after{animation-duration:.001ms!important;transition-duration:.001ms!important}
}
</style>
</head>
<body>
<div class="pg">

<header class="hdr">
  <div class="brand">
    <span class="bdot"></span>AERO-SANITIZE
    <span style="color:var(--t3);margin-left:4px;font-weight:500">AI</span>
  </div>
  <div class="pill" id="pill">
    <span class="pdot"></span><span id="pillTxt">LIVE</span>
  </div>
</header>

<section class="sc" id="sc" role="status" aria-live="polite">
  <div class="si">
    <div class="sic">
      <svg id="sico" viewBox="0 0 24 24" fill="none" stroke="currentColor"
           stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round">
        <path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/>
      </svg>
    </div>
    <div class="stx">
      <div class="stitle" id="stitle">CONNECTING&#8230;</div>
      <div class="ssub"   id="ssub">Establishing link to controller</div>
    </div>
  </div>
  <div class="asw" id="asw">
    <div class="aslb">AUTO-START IN<span id="astime">--</span></div>
    <div class="asbar"><div class="asfill" id="asf"></div></div>
  </div>
</section>

<div class="sr">
  <div class="scard" id="rcrd">
    <span class="slbl">Radar</span>
    <div class="rr"><div class="rsw"></div><div class="rc"></div></div>
    <span class="sv" id="rv">--</span>
  </div>
  <div class="scard" id="tcrd">
    <span class="slbl">Thermal</span>
    <div class="tb"><div class="tf" id="tf"></div><div class="tline"></div></div>
    <span class="sv" id="tv">--&#176;C</span>
  </div>
  <div class="scard" id="dcrd">
    <span class="slbl">Door</span>
    <div class="dframe">
      <div class="dleaf" id="dl"><div class="dknob"></div></div>
    </div>
    <span class="sv" id="dv">--</span>
  </div>
</div>

<section class="tmrsec" id="tmrsec">
  <div class="rw">
    <svg viewBox="0 0 100 100" class="rsvg">
      <circle class="rtrack" cx="50" cy="50" r="48"/>
      <circle class="rarc"   cx="50" cy="50" r="48" id="rarc"/>
    </svg>
    <div class="rc2">
      <div class="rtime" id="rtime">--:--</div>
      <div class="rlbl"  id="rlbl">STANDBY</div>
    </div>
  </div>
</section>

<div class="br">
  <button class="btn bstart" id="bs" onclick="cmd('/start')" disabled>
    &#9654; START UV-C CYCLE
  </button>
  <button class="btn bstop" onclick="cmd('/stop')">
    &#9632; EMERGENCY STOP
  </button>
</div>

<div class="ft" id="ft">--</div>
</div>

<script>
(function(){
'use strict';

// ── CONSTANTS ──
var CIRC = 301.6;           // 2 * PI * 48  (ring r=48)
var FAIL_MAX = 3;           // consecutive failures before "OFFLINE"

// ── STATE ──
var fails = 0, fetching = false, lastState = '';

// ── HELPERS ──
function $(id){ return document.getElementById(id); }
function pad(n){ return n < 10 ? '0' + n : '' + n; }
function clamp(v,a,b){ return v < a ? a : v > b ? b : v; }
function ms2mmss(ms){
  var s = Math.ceil(ms / 1000);
  return pad(Math.floor(s / 60)) + ':' + pad(s % 60);
}

// SVG icon paths per state
var ICONS = {
  ready:  '<path d="M12 22s8-4 8-10V5l-8-3-8 3v7c0 6 8 10 8 10z"/>',
  scan:   '<circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/>',
  active: '<path d="M13 2L3 14h9l-1 8 10-12h-9l1-8z"/>',
  unsafe: '<path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/>'
};

function setConn(ok){
  $('pill').className = 'pill' + (ok ? '' : ' off');
  $('pillTxt').textContent = ok ? 'LIVE' : 'OFFLINE';
}
function setCard(cls, ico, title, sub){
  $('sc').className    = 'sc ' + cls;
  $('sico').innerHTML  = ICONS[ico] || ICONS.ready;
  $('stitle').textContent = title;
  $('ssub').textContent   = sub;
}
function showTimer(show, cls){
  var t = $('tmrsec');
  t.style.display = show ? 'flex' : 'none';
  t.className = 'tmrsec ' + (cls || '');
}

// ── COMMAND (POST with no body) ──
window.cmd = async function(url){
  try{ await fetch(url, { method: 'POST' }); }
  catch(e){ /* silent — UI will reflect result on next poll */ }
};

// ── POLL (with AbortController timeout + fetch-guard) ──
async function poll(){
  if(fetching) return;          // never stack two requests
  fetching = true;

  var ctrl = typeof AbortController !== 'undefined' ? new AbortController() : null;
  var tid  = ctrl ? setTimeout(function(){ ctrl.abort(); }, 900) : null;

  try{
    var res = await fetch('/data', ctrl ? { signal: ctrl.signal } : {});
    if(tid) clearTimeout(tid);
    if(!res.ok) throw new Error('HTTP ' + res.status);
    var d = await res.json();
    fails = 0;
    setConn(true);
    render(d);
  } catch(e){
    if(tid) clearTimeout(tid);
    if(++fails >= FAIL_MAX){
      setConn(false);
      setCard('', 'ready', 'CONNECTION LOST', 'Retrying link to controller\u2026');
    }
  } finally {
    fetching = false;
  }
}

// ── RENDER ──
function render(d){
  var st   = d.state || 'STANDBY';
  var safe = d.isSafe !== false;
  var uvc  = d.uvcActive === true;
  var scan = (st === 'SCANNING');

  // STATUS CARD — skip re-render if state hasn't changed (reduces DOM work)
  if(st !== lastState){
    lastState = st;
    if(st === 'UNSAFE'){
      setCard('s-unsafe','unsafe','UNSAFE \u2014 DO NOT ENTER','Hazard detected \u00b7 Lamp is OFF');
    } else if(uvc){
      setCard('s-active','active','UV-C ACTIVE \u2014 STAY OUT','Sterilization cycle in progress');
    } else if(scan){
      setCard('s-scan','scan','SCANNING ROOM\u2026','Confirming absence of personnel');
    } else {
      setCard(safe ? 's-ready' : '', 'ready',
        safe ? 'SYSTEM READY' : 'STANDBY \u2014 ROOM OCCUPIED',
        safe ? 'All sensors nominal \u00b7 Room clear' : 'Waiting for room to clear');
    }
  } else if(st === 'STANDBY'){
    // Safe flag can flip without a state-label change
    $('sc').className     = 'sc ' + (safe ? 's-ready' : '');
    $('stitle').textContent = safe ? 'SYSTEM READY' : 'STANDBY \u2014 ROOM OCCUPIED';
    $('ssub').textContent   = safe ? 'All sensors nominal \u00b7 Room clear' : 'Waiting for room to clear';
  }

  // AUTO-START BAR
  var asw = $('asw');
  if(st === 'STANDBY' && safe && d.autoStartIn > 0 && d.autoStartTotal > 0){
    asw.className = 'asw vis';
    var pct = 100 - (d.autoStartIn / d.autoStartTotal * 100);
    $('asf').style.width = clamp(pct, 0, 100) + '%';
    $('astime').textContent = Math.ceil(d.autoStartIn / 1000) + 's';
  } else {
    asw.className = 'asw';
  }

  // RADAR
  var mot = (d.radar === 'MOTION');
  $('rcrd').className   = 'scard' + (mot ? ' al' : '');
  $('rv').textContent   = d.radar || '--';

  // THERMAL
  var tmp = parseFloat(d.thermal);
  var hot = !isNaN(tmp) && tmp > 34.0;
  $('tcrd').className = 'scard' + (hot ? ' al' : '');
  $('tv').innerHTML   = isNaN(tmp) ? '--&#176;C' : tmp.toFixed(1) + '&#176;C';
  $('tf').style.height = isNaN(tmp) ? '0%' : clamp((tmp-18)/(40-18)*100, 0, 100) + '%';

  // DOOR
  var op = (d.door === 'OPEN');
  $('dcrd').className = 'scard' + (op ? ' al' : '');
  $('dv').textContent = d.door || '--';
  var dl = $('dl');
  op ? dl.classList.add('op') : dl.classList.remove('op');

  // TIMER RING — shows during SCANNING (scan countdown) or UV-C (cycle countdown)
  if(scan || uvc){
    var total = scan ? (d.scanTotal || 10000) : (d.total || 60000);
    var left  = scan ? (d.scanTimer || 0)     : (d.timer || 0);
    showTimer(true, scan ? 's-scan' : 's-active');
    $('rtime').textContent = ms2mmss(left);
    $('rlbl').textContent  = scan ? 'SAFETY SCAN' : 'UV-C CYCLE';
    $('rarc').style.strokeDashoffset = CIRC * (1 - clamp(left / total, 0, 1));
  } else {
    showTimer(false);
    $('rarc').style.strokeDashoffset = CIRC; // reset for next use
  }

  // START BUTTON — only STANDBY + safe (matches C++ handleStart guard exactly)
  $('bs').disabled = !(st === 'STANDBY' && safe);

  // FOOTER
  $('ft').textContent = 'Updated ' + new Date().toLocaleTimeString();
}

// ── KICK OFF ──
setInterval(poll, 1000);
poll();

})();
</script>
</body>
</html>
)=====";


// ─────────────────────────────────────────────────────────────────────────────
// SENSOR READING  (called every SENSOR_INTERVAL_MS — not every loop)
// ─────────────────────────────────────────────────────────────────────────────
void readSensors() {
  // Radar: digital HIGH = movement
  radarMotion = (digitalRead(RADAR_PIN) == HIGH);

  // Door: INPUT_PULLUP → LOW = magnet present (closed), HIGH = open
  doorOpen = (digitalRead(DOOR_PIN) == HIGH);

  // AMG8833: scan all 64 pixels, keep the hottest one
  float pixels[64];
  amg.readPixels(pixels);
  float maxT = 0.0f;
  for (int i = 0; i < 64; i++) {
    if (pixels[i] > maxT) maxT = pixels[i];
  }
  currentThermal = maxT;
}

// Safety gate: ALL three conditions must pass simultaneously
bool isRoomSafe() {
  if (doorOpen)               return false;  // door must be shut
  if (radarMotion)            return false;  // zero motion allowed
  if (currentThermal > 34.0f) return false;  // no body heat signature
  return true;
}

// Human-readable state label sent to the dashboard
String stateLabel() {
  switch (currentState) {
    case SCANNING:   return "SCANNING";
    case UVC_ACTIVE: return "ACTIVE";
    case UNSAFE:     return "UNSAFE";
    default:         return "STANDBY";
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// STATE MACHINE
// ─────────────────────────────────────────────────────────────────────────────
void runStateMachine() {
  unsigned long now = millis();

  switch (currentState) {

    // ── STANDBY: idle. Auto-starts after AUTO_START_DELAY_MS of clear room ──
    case STANDBY:
      digitalWrite(RELAY_PIN, LOW);   // always ensure lamp off in standby
      countdownLeft = 0;
      scanTimeLeft  = 0;

      if (systemIsSafe) {
        if (!roomWasSafe) {
          // Room JUST cleared — begin the auto-start countdown
          roomWasSafe   = true;
          roomSafeStart = now;
        }
        unsigned long safeElapsed = now - roomSafeStart;
        if (safeElapsed >= AUTO_START_DELAY_MS) {
          // 10 consecutive seconds of safety → automatically begin scan
          currentState  = SCANNING;
          scanStartTime = now;
          roomWasSafe   = false;
          autoStartInMs = 0;
          Serial.println("[AUTO] 10s clear confirmed → starting safety scan.");
        } else {
          autoStartInMs = AUTO_START_DELAY_MS - safeElapsed;
        }
      } else {
        // Something moved / door opened → reset the countdown
        roomWasSafe   = false;
        autoStartInMs = 0;
      }
      break;

    // ── SCANNING: 10-second pre-ignition confirmation ──
    case SCANNING:
      autoStartInMs = 0;
      countdownLeft = 0;

      if (!systemIsSafe) {
        // Violation during scan → abort immediately
        currentState = UNSAFE;
        scanTimeLeft = 0;
        Serial.println("[SCAN] Safety violation → UNSAFE.");
      } else {
        unsigned long elapsed = now - scanStartTime;
        if (elapsed >= SCAN_DURATION_MS) {
          // Scan passed — ignite the UV-C lamp
          currentState  = UVC_ACTIVE;
          uvcStartTime  = now;
          scanTimeLeft  = 0;
          digitalWrite(RELAY_PIN, HIGH);
          Serial.println("[SCAN] PASS → UV-C LAMP ON.");
        } else {
          scanTimeLeft = SCAN_DURATION_MS - elapsed;
        }
      }
      break;

    // ── UVC_ACTIVE: lamp is on, count down 60 seconds ──
    case UVC_ACTIVE:
      autoStartInMs = 0;
      scanTimeLeft  = 0;

      if (!systemIsSafe) {
        // Intrusion during sterilization → kill lamp immediately
        currentState  = UNSAFE;
        countdownLeft = 0;
        digitalWrite(RELAY_PIN, LOW);
        Serial.println("[SAFETY] Intrusion during cycle → LAMP OFF → UNSAFE.");
      } else {
        unsigned long elapsed = now - uvcStartTime;
        if (elapsed >= UVC_DURATION_MS) {
          // Cycle complete — return to standby
          currentState  = STANDBY;
          countdownLeft = 0;
          roomWasSafe   = false;  // fresh countdown after cycle completes
          digitalWrite(RELAY_PIN, LOW);
          Serial.println("[DONE] UV-C cycle complete → STANDBY.");
        } else {
          countdownLeft = UVC_DURATION_MS - elapsed;
        }
      }
      break;

    // ── UNSAFE: lamp killed, wait for ALL sensors to clear, then auto-recover ──
    case UNSAFE:
      digitalWrite(RELAY_PIN, LOW);
      countdownLeft = 0;
      scanTimeLeft  = 0;
      autoStartInMs = 0;

      if (systemIsSafe) {
        // Room is clear again → auto-recover without needing Emergency Stop
        currentState = STANDBY;
        roomWasSafe  = false;  // fresh auto-start countdown after recovery
        Serial.println("[UNSAFE CLEARED] Sensors nominal → STANDBY.");
      }
      break;
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// WEB HANDLERS
// ─────────────────────────────────────────────────────────────────────────────
void handleRoot() {
  // Cache HTML for 10 minutes — it never changes at runtime
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send_P(200, PSTR("text/html"), DASHBOARD_HTML);
}

void handleData() {
  // Never cache live sensor data
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection",    "close");   // releases socket quickly

  String json = "{";
  json += "\"thermal\":"        + String(currentThermal, 1)                              + ",";
  json += "\"radar\":\""        + String(radarMotion ? "MOTION" : "CLEAR")               + "\",";
  json += "\"door\":\""         + String(doorOpen    ? "OPEN"   : "CLOSED")              + "\",";
  json += "\"isSafe\":"         + String(systemIsSafe ? "true" : "false")                + ",";
  json += "\"uvcActive\":"      + String(currentState == UVC_ACTIVE ? "true" : "false")  + ",";
  json += "\"state\":\""        + stateLabel()                                            + "\",";
  json += "\"timer\":"          + String(countdownLeft)                                   + ",";
  json += "\"total\":"          + String(UVC_DURATION_MS)                                 + ",";
  json += "\"scanTimer\":"      + String(scanTimeLeft)                                    + ",";
  json += "\"scanTotal\":"      + String(SCAN_DURATION_MS)                                + ",";
  json += "\"autoStartIn\":"    + String(autoStartInMs)                                   + ",";
  json += "\"autoStartTotal\":" + String(AUTO_START_DELAY_MS);
  json += "}";

  server.send(200, "application/json", json);
}

void handleStart() {
  if (currentState == STANDBY && systemIsSafe) {
    currentState  = SCANNING;
    scanStartTime = millis();
    roomWasSafe   = false;
    autoStartInMs = 0;
    Serial.println("[CMD] Manual START → safety scan begun.");
  } else {
    Serial.println("[CMD] START rejected — not STANDBY or room not safe.");
  }
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  currentState  = STANDBY;
  roomWasSafe   = false;
  autoStartInMs = 0;
  countdownLeft = 0;
  scanTimeLeft  = 0;
  digitalWrite(RELAY_PIN, LOW);
  Serial.println("[CMD] EMERGENCY STOP → STANDBY.");
  server.send(200, "text/plain", "OK");
}


// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Aero-Sanitize AI starting...");

  // Output — lamp off at boot
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Inputs
  pinMode(RADAR_PIN, INPUT);
  pinMode(DOOR_PIN,  INPUT_PULLUP);

  // I2C at 400 kHz (Fast Mode) — cuts AMG8833 read from ~20 ms to ~5 ms
  Wire.begin();
  Wire.setClock(400000);

  // AMG8833 thermal camera — halts system if missing (critical safety sensor)
  if (!amg.begin()) {
    Serial.println("[ERROR] AMG8833 not found! Check SDA/SCL wiring. HALTED.");
    while (true) { delay(1000); }
  }
  Serial.println("[OK] AMG8833 @ 400 kHz.");

  // DHT22 ambient temperature (optional — not part of safety logic)
  dht.begin();
  Serial.println("[OK] DHT22 initialized.");

  // SoftAP — ESP32 becomes its own router, no internet needed
  WiFi.softAP("AeroSanitize-AI", NULL);
  Serial.print("[WIFI] AP live. Dashboard → http://");
  Serial.println(WiFi.softAPIP());

  // Routes
  server.on("/",      HTTP_GET,  handleRoot);
  server.on("/data",  HTTP_GET,  handleData);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/stop",  HTTP_POST, handleStop);
  server.begin();

  Serial.println("[READY] STANDBY. Auto-start fires after 10s of clear room.");
}


// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();             // ① Network FIRST (catches incoming requests)

  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readSensors();                   // ② I2C read (~5 ms at 400 kHz) — only every 300 ms
    systemIsSafe = isRoomSafe();
  }

  runStateMachine();                 // ③ Fast state logic (no I2C, no delay)

  server.handleClient();             // ④ Network AGAIN — catches requests queued during sensor read
}