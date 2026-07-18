#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "DHT.h"
#include <RTClib.h>                                  // Adafruit RTC library (DS3231)
#include <idriss_mokdadi-project-1_inferencing.h>     // Edge Impulse TinyML model

// ── PIN MAP ──────────────────────────────────────────────────────────────────
#define RELAY_PIN   14   // UV-C lamp relay
#define RADAR_PIN    5   // Radar OUT              HIGH = motion
#define DOOR_PIN    12   // Reed switch            INPUT_PULLUP, LOW = closed
#define DHT_PIN     15   // DHT22 humidity         fed to the AI dose model
#define LDR_PIN     34   // LDR                    fed to the AI dose model + bulb-health check
#define DHTTYPE     DHT22

// SDA = GPIO 8, SCL = GPIO 9 on ESP32-S3 — shared by the AMG8833 AND the RTC.

// ── RELAY POLARITY ───────────────────────────────────────────────────────────
// Your relay module is active-LOW: pulling the pin LOW energizes the coil and
// turns the lamp ON; HIGH is the idle/off state. That's the inverse of the
// intuitive HIGH=ON assumption, which is exactly the bug you were seeing
// (lamp on at STANDBY, off during the cycle). Everywhere the code wants the
// lamp on/off, it now goes through these two constants — if you ever swap to
// an active-HIGH relay board, flip these two lines and nothing else changes.
#define RELAY_ACTIVE_LEVEL LOW
#define RELAY_IDLE_LEVEL    HIGH

// ── TIMING CONSTANTS ─────────────────────────────────────────────────────────
const unsigned long SCAN_DURATION_MS    = 10000UL; // Mandatory pre-ignition scan (10s)
const unsigned long AUTO_START_DELAY_MS = 10000UL; // Room clear -> auto-arm (10s)
const unsigned long SENSOR_INTERVAL_MS  =   300UL; // Re-read sensors every 300ms
const unsigned long AI_TRIGGER_DELAY_MS =  4000UL; // Lamp warm-up before the LDR/AI read is taken

// ── SAFETY / AI THRESHOLDS ───────────────────────────────────────────────────
const int   LDR_FAULT_THRESHOLD    = 550;   // below this, the bulb is considered dead/degraded
const float MAX_PREDICTED_MINUTES  = 60.0;  // sanity cap on the AI-predicted dose

// ── DUAL-CORE AI SHARED STATE ────────────────────────────────────────────────
volatile int   shared_ldr         = 0;
volatile float shared_humidity    = 0.0;
volatile float predicted_minutes  = 0.0;
volatile bool  ai_needs_to_run    = false;
volatile bool  ai_finished        = false;
TaskHandle_t   AI_Task;

// ── NURSE'S LOG (RTC-TIMESTAMPED, FIFO) ──────────────────────────────────────
struct SanitizationLog {
  String status;        // COMPLETED / ABORTED / FAILED
  float  durationMins;  // AI-predicted dose for that cycle
  String timestamp;      // "14:35:10"
  String dateStamp;      // "16/07/2026"
  String reason;         // e.g. "Radar Intrusion", "Bulb degraded"
};
#define MAX_LOGS 8
SanitizationLog cycleLogs[MAX_LOGS];
int logCount = 0;
unsigned long totalCyclesRun = 0;

// ── OBJECTS ──────────────────────────────────────────────────────────────────
WebServer        server(80);
Adafruit_AMG88xx amg;
DHT              dht(DHT_PIN, DHTTYPE);
RTC_DS3231       rtc;   // swap for RTC_DS1307 if that's your physical module

// ── STATE MACHINE ─────────────────────────────────────────────────────────────
enum SystemState { STANDBY, SCANNING, WARMUP, UVC_ACTIVE, UNSAFE };
SystemState currentState = STANDBY;

// ── SENSOR GLOBALS ────────────────────────────────────────────────────────────
float currentThermal = 0.0f;
bool  radarMotion    = false;
bool  doorOpen        = false;
bool  systemIsSafe    = false;

// ── COMPUTED STATE (read by handleData, written by runStateMachine) ────────────
unsigned long countdownLeft          = 0;   // ms left in the UV-C cycle
unsigned long scanTimeLeft           = 0;   // ms left in the pre-ignition scan
unsigned long warmupTimeLeft         = 0;   // ms left before the lamp is warm enough to read
unsigned long autoStartInMs          = 0;   // ms until auto-start (0 = not counting)
unsigned long calculatedUvcDurationMs = 0;  // AI-predicted cycle length, set at ignition

// ── AI TRIGGER TRACKING (one shot per cycle, fired once the lamp has warmed up) ─
// NOTE: this fires during WARMUP — i.e. AFTER the 10s pre-ignition safety scan
// has already passed and the lamp has been switched on. It never fires while
// the room is still being checked for occupancy.
bool aiTriggered = false;   // guards against re-firing the AI mid-warmup
int  capturedLdr = 0;       // LDR value frozen at the moment the AI was triggered —
                             // this is the value used for the bulb-health gate later,
                             // NOT the live shared_ldr (which keeps drifting as the
                             // lamp continues warming up / room conditions change).

// ── FAULT TRACKING ────────────────────────────────────────────────────────────
String lastBreachReason      = "";
bool   unsafeIsHardwareFault = false;  // true = bulb/dose fault (needs a human), false = room re-entered
String logMessage            = "System booted. Awaiting cycle initiation.";

// ── INTERNAL TIMERS ───────────────────────────────────────────────────────────
unsigned long uvcStartTime   = 0;
unsigned long scanStartTime  = 0;
unsigned long warmupStartTime = 0;
unsigned long lastSensorRead = 0;
unsigned long roomSafeStart  = 0;
bool          roomWasSafe    = false;


// ─────────────────────────────────────────────────────────────────────────────
// DASHBOARD HTML (flash-resident via PROGMEM)
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

/* ── LIVE LOG TICKER ── */
.cl{margin:0 0 11px;padding:8px 11px;background:var(--sf2);border:1px solid var(--bd);
  border-left:3px solid var(--b);border-radius:8px;font-family:var(--mono);
  font-size:10.5px;color:var(--t2);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}

/* ── SENSOR CARDS ── */
.sr{display:grid;grid-template-columns:repeat(auto-fit,minmax(84px,1fr));gap:8px;margin-bottom:11px}
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

/* humidity bar (informational — feeds the AI, not a safety cutoff) */
.hb{width:10px;height:34px;border-radius:5px;background:var(--sf2);
  border:1px solid var(--bd);position:relative;overflow:hidden;margin:0 auto}
.hf{position:absolute;bottom:0;left:0;right:0;height:0%;background:var(--b);opacity:.7;transition:height .5s ease}

/* door glyph */
.dframe{width:28px;height:36px;margin:0 auto;border:1.5px solid var(--bd);
  border-radius:3px;background:var(--sf2);position:relative;overflow:hidden}
.dleaf{position:absolute;top:3px;left:3px;right:3px;bottom:3px;background:var(--t3);
  border-radius:2px;transform-origin:left center;
  transition:transform .45s cubic-bezier(.3,.7,.4,1),background .25s}
.dleaf.op{transform:scaleX(.12);background:var(--r)}
.dknob{position:absolute;right:4px;top:50%;width:3px;height:3px;
  margin-top:-1.5px;border-radius:50%;background:var(--sf2)}

/* ── AI DOSE ADVISOR ── */
.aic{background:var(--sf);border:1px solid var(--bd);border-radius:var(--rr);
  padding:14px 15px;margin-bottom:11px;transition:border-color .25s}
.aich{display:flex;align-items:center;justify-content:space-between;margin-bottom:9px}
.ailbl{font-size:9px;font-weight:800;letter-spacing:.14em;color:var(--t2);text-transform:uppercase}
.chip{font-family:var(--mono);font-size:9px;font-weight:800;letter-spacing:.05em;
  padding:3px 9px;border-radius:99px;text-transform:uppercase;white-space:nowrap;border:1px solid transparent}
.chip-ok{background:rgba(0,217,139,.13);color:var(--g)}
.chip-bad{background:rgba(255,61,84,.13);color:var(--r)}
.chip-neutral{background:var(--sf2);color:var(--t2);border-color:var(--bd)}
.aibody{display:flex;align-items:baseline;justify-content:space-between;gap:10px}
.aival{font-family:var(--mono);font-size:21px;font-weight:700}
.aival.think{color:var(--y);animation:blink 1.1s ease infinite}
.aildr{font-size:9.5px;color:var(--t3);font-family:var(--mono);white-space:nowrap}
.aisub{margin-top:7px;font-size:11px;color:var(--t2)}

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
.btn.busy{opacity:.55;pointer-events:none}
.bstart{background:var(--g);color:#012b1b}
.bstart:not(:disabled):hover{opacity:.88}
.bstop{background:var(--r);color:#fff}
.bstop:hover{opacity:.88}

/* ── HISTORY LOG ── */
.hc{background:var(--sf);border:1px solid var(--bd);border-radius:var(--rr);
  padding:13px 13px 6px;margin-bottom:11px}
.hclbl{font-size:9px;font-weight:800;letter-spacing:.13em;color:var(--t2);
  text-transform:uppercase;margin-bottom:9px}
table.htbl{width:100%;border-collapse:collapse;font-size:11px}
.htbl th{text-align:left;font-size:8.5px;letter-spacing:.06em;text-transform:uppercase;
  color:var(--t3);padding:0 5px 7px;font-weight:700}
.htbl td{padding:8px 5px;border-top:1px solid var(--sf2);color:var(--t2);vertical-align:top}
.htime{font-family:var(--mono);font-size:9.5px;color:var(--t3);white-space:nowrap}
.hempty{text-align:center;color:var(--t3);padding:16px 0}

/* ── STATS + FOOTER ── */
.stats{display:flex;justify-content:center;gap:14px;margin-bottom:6px;
  font-family:var(--mono);font-size:9.5px;color:var(--t3)}
.stats b{color:var(--t2)}
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

<div class="cl" id="cl">&gt; Awaiting first telemetry frame&#8230;</div>

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
  <div class="scard" id="hcrd">
    <span class="slbl">Humidity</span>
    <div class="hb"><div class="hf" id="hf"></div></div>
    <span class="sv" id="hv">--%</span>
  </div>
</div>

<section class="aic" id="aic">
  <div class="aich">
    <span class="ailbl">AI Dose Advisor</span>
    <span class="chip chip-neutral" id="bulbChip">BULB --</span>
  </div>
  <div class="aibody">
    <div class="aival" id="aival">--</div>
    <div class="aildr">LDR <span id="ldrv">--</span></div>
  </div>
  <div class="aisub" id="aisub">Awaiting next cycle</div>
</section>

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
  <button class="btn bstart" id="bs" onclick="cmd('/start', this)" disabled>
    <span id="bsIcon">&#9654;</span>&nbsp;<span id="bsTxt">START UV-C CYCLE</span>
  </button>
  <button class="btn bstop" onclick="cmd('/stop', this)">
    &#9632; EMERGENCY STOP
  </button>
</div>

<section class="hc">
  <div class="hclbl">Nurse's Log &mdash; Last Cycles (RTC Timestamps)</div>
  <table class="htbl">
    <thead><tr><th>Status</th><th>Dose</th><th>When</th><th>Reason</th></tr></thead>
    <tbody id="hbody"><tr><td colspan="4" class="hempty">No cycles recorded yet.</td></tr></tbody>
  </table>
</section>

<div class="stats" id="stats"><span>CYCLES <b id="stCycles">--</b></span><span>CLIENTS <b id="stClients">--</b></span><span>UP <b id="stUptime">--</b></span></div>
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
function fmtUptime(sec){
  sec = Math.floor(sec);
  var h = Math.floor(sec/3600), m = Math.floor((sec%3600)/60), s = sec%60;
  return h > 0 ? (h+'h'+pad(m)+'m') : (m+'m'+pad(s)+'s');
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

// ── COMMAND (POST, then refresh immediately instead of waiting for the next tick) ──
window.cmd = function(url, btn){
  if(btn){ btn.classList.add('busy'); }
  fetch(url, { method: 'POST' })
    .catch(function(){ /* UI reflects the real result on next poll regardless */ })
    .then(function(){ return poll(); })
    .then(function(){ if(btn){ btn.classList.remove('busy'); } });
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
  var warm = (st === 'WARMUP');
  var aiReady = d.aiReady === true;

  // STATUS CARD
  if(st !== lastState || st === 'UNSAFE'){
    lastState = st;
    if(st === 'UNSAFE'){
      var reason = d.breachReason || 'Unknown fault';
      if(d.hardwareFault){
        setCard('s-unsafe','unsafe','HARDWARE FAULT \u2014 CYCLE HALTED','Cause: ' + reason + ' \u00b7 Needs attention');
      } else {
        setCard('s-unsafe','unsafe','UNSAFE \u2014 CYCLE INTERRUPTED','Cause: ' + reason + ' \u00b7 Lamp is OFF');
      }
    } else if(uvc){
      setCard('s-active','active','UV-C ACTIVE \u2014 STAY OUT','Sterilizing for the AI-predicted duration');
    } else if(warm){
      setCard('s-scan','scan', aiReady ? 'CYCLE STARTING \u2014 DOSE CONFIRMED' : 'LAMP WARMING UP\u2026',
        aiReady ? 'AI dose scored \u00b7 beginning full cycle' : 'Lamp is on \u00b7 reading bulb intensity for the AI');
    } else if(scan){
      setCard('s-scan','scan','SCANNING ROOM\u2026', 'Confirming room is clear before ignition \u00b7 lamp off');
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

  // LOG TICKER
  $('cl').textContent = '> ' + (d.log || '');

  // RADAR
  var mot = (d.radar === 'MOTION');
  $('rcrd').className   = 'scard' + (mot ? ' al' : '');
  $('rv').textContent   = d.radar || '--';

  // THERMAL (alert threshold fixed at 34.0C, matches isRoomSafe)
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

  // HUMIDITY (informational — feeds the AI advisor)
  var hum = parseFloat(d.humidity);
  var humOk = !isNaN(hum);
  $('hv').textContent = humOk ? hum.toFixed(1) + '%' : '--%';
  $('hf').style.height = (humOk ? clamp(hum,0,100) : 0) + '%';

  // AI DOSE ADVISOR + BULB HEALTH
  var ldr = parseInt(d.ldr, 10);
  var ldrOk = !isNaN(ldr);
  var thresh = (typeof d.ldrThreshold === 'number') ? d.ldrThreshold : 550;
  var bulbOk = ldrOk && ldr >= thresh;
  $('ldrv').textContent = ldrOk ? ldr : '--';
  $('bulbChip').textContent = ldrOk ? (bulbOk ? 'BULB OK' : 'BULB DEGRADED') : 'BULB --';
  $('bulbChip').className = 'chip ' + (!ldrOk ? 'chip-neutral' : (bulbOk ? 'chip-ok' : 'chip-bad'));

  var predicted = parseFloat(d.predictedMinutes);
  var predictedOk = !isNaN(predicted) && predicted > 0;
  var aivalEl = $('aival');
  if(warm && !aiReady){
    aivalEl.textContent = 'CALCULATING\u2026';
    aivalEl.className = 'aival think';
    $('aisub').textContent = 'Lamp warming up before the TinyML dose read';
  } else if(predictedOk){
    aivalEl.textContent = predicted.toFixed(1) + ' MIN';
    aivalEl.className = 'aival';
    $('aisub').textContent = uvc ? 'Running the full AI-predicted dose' : 'Last predicted sanitization dose';
  } else {
    aivalEl.textContent = '--';
    aivalEl.className = 'aival';
    $('aisub').textContent = scan ? 'Waiting for pre-scan to finish' : 'Awaiting next cycle';
  }

  // TIMER RING — shows during SCANNING (room check), WARMUP (lamp warm-up +
  // AI read), or UV-C (AI-predicted countdown)
  if(scan || warm || uvc){
    var total, left, label;
    if(uvc){
      total = d.total || 60000;
      left  = d.timer || 0;
      label = 'UV-C CYCLE';
    } else if(warm){
      total = d.warmupTotal || 4000;
      left  = d.warmupTimer || 0;
      label = (left === 0 && !aiReady) ? 'AI CALCULATING' : 'LAMP WARM-UP';
    } else {
      total = d.scanTotal || 10000;
      left  = d.scanTimer || 0;
      label = 'SAFETY SCAN';
    }
    showTimer(true, uvc ? 's-active' : 's-scan');
    $('rtime').textContent = ms2mmss(left);
    $('rlbl').textContent = label;
    $('rarc').style.strokeDashoffset = CIRC * (1 - clamp(left / total, 0, 1));
  } else {
    showTimer(false);
    $('rarc').style.strokeDashoffset = CIRC; // reset for next use
  }

  // HISTORY TABLE
  var hist = d.history || [];
  var rows = '';
  if(hist.length === 0){
    rows = '<tr><td colspan="4" class="hempty">No cycles recorded yet.</td></tr>';
  } else {
    hist.forEach(function(h){
      var cls = h.status === 'COMPLETED' ? 'chip-ok' : ((h.status === 'ABORTED' || h.status === 'FAILED') ? 'chip-bad' : 'chip-neutral');
      rows += '<tr>';
      rows += '<td><span class="chip ' + cls + '">' + h.status + '</span></td>';
      rows += '<td>' + h.duration + ' min</td>';
      rows += '<td class="htime">' + h.date + '<br>' + h.time + '</td>';
      rows += '<td>' + h.reason + '</td>';
      rows += '</tr>';
    });
  }
  $('hbody').innerHTML = rows;

  // START BUTTON — valid from STANDBY+safe, or to acknowledge/retry from UNSAFE
  var canStart = (st === 'STANDBY' && safe) || st === 'UNSAFE';
  $('bs').disabled = !canStart;
  $('bsTxt').textContent = (st === 'UNSAFE') ? 'ACKNOWLEDGE & RETRY' : 'START UV-C CYCLE';

  // STATS + FOOTER
  if(typeof d.cycleCount === 'number') $('stCycles').textContent = d.cycleCount;
  if(typeof d.clients === 'number')    $('stClients').textContent = d.clients;
  if(typeof d.uptimeSec === 'number')  $('stUptime').textContent = fmtUptime(d.uptimeSec);
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
// LOGGING
// ─────────────────────────────────────────────────────────────────────────────
void addLog(String status, float durationMins, String reason) {
  DateTime now = rtc.now();

  char timeBuffer[16];
  sprintf(timeBuffer, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  char dateBuffer[16];
  sprintf(dateBuffer, "%02d/%02d/%04d", now.day(), now.month(), now.year());

  SanitizationLog entry = { status, durationMins, String(timeBuffer), String(dateBuffer), reason };

  if (logCount < MAX_LOGS) {
    cycleLogs[logCount] = entry;
    logCount++;
  } else {
    for (int i = 1; i < MAX_LOGS; i++) cycleLogs[i - 1] = cycleLogs[i];
    cycleLogs[MAX_LOGS - 1] = entry;
  }
  totalCyclesRun++;
}


// ─────────────────────────────────────────────────────────────────────────────
// CORE 0 TASK — TinyML dose inference, kept off the safety-critical Core 1 loop
// ─────────────────────────────────────────────────────────────────────────────
void runAILogic(void * parameter) {
  for (;;) {
    if (ai_needs_to_run) {
      Serial.println("[CORE 0] Triggered. Running TinyML regression inference...");

      float features[2] = { (float)shared_ldr, shared_humidity };
      signal_t features_signal;
      numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &features_signal);

      ei_impulse_result_t result = { 0 };
      run_classifier(&features_signal, &result, false);

      predicted_minutes = result.classification[0].value;
      ai_needs_to_run    = false;
      ai_finished        = true;

      Serial.print("[CORE 0] Inference complete. Predicted dose: ");
      Serial.print(predicted_minutes);
      Serial.println(" min.");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// SENSORS
// ─────────────────────────────────────────────────────────────────────────────
void readSensors() {
  radarMotion = (digitalRead(RADAR_PIN) == HIGH);
  doorOpen    = (digitalRead(DOOR_PIN)  == HIGH);   // INPUT_PULLUP: HIGH = open

  float pixels[64];
  amg.readPixels(pixels);
  float maxT = 0.0f;
  for (int i = 0; i < 64; i++) if (pixels[i] > maxT) maxT = pixels[i];
  currentThermal = maxT;

  shared_ldr = analogRead(LDR_PIN);

  float h = dht.readHumidity();
  if (!isnan(h)) shared_humidity = h;   // DHT22 reads intermittently fail; keep last good value
}

bool isRoomSafe() {
  if (doorOpen)               return false;
  if (radarMotion)            return false;
  if (currentThermal > 34.0f) return false;
  return true;
}

// Identifies WHICH sensor tripped an unsafe reading, in the same priority
// order as isRoomSafe() above, so every log entry can say exactly why a
// cycle failed instead of just "unsafe".
String unsafeSensorReason() {
  if (doorOpen)               return "door sensor (door opened)";
  if (radarMotion)            return "radar sensor (motion detected)";
  if (currentThermal > 34.0f) return "thermal sensor (room " + String(currentThermal, 1) + "C > 34.0C)";
  return "unknown sensor";
}

String stateLabel() {
  switch (currentState) {
    case SCANNING:   return "SCANNING";
    case WARMUP:     return "WARMUP";
    case UVC_ACTIVE: return "ACTIVE";
    case UNSAFE:     return "UNSAFE";
    default:         return "STANDBY";
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// STATE MACHINE
// ─────────────────────────────────────────────────────────────────────────────
void enterScanning(unsigned long now) {
  currentState      = SCANNING;
  scanStartTime      = now;
  roomWasSafe         = false;
  autoStartInMs       = 0;

  // Pure safety pre-check: lamp stays OFF for this entire 10s window. The AI
  // does NOT run here and the lamp does NOT turn on here — that only happens
  // once this scan passes and the actual cycle begins (see enterWarmup()).
  ai_needs_to_run     = false;
  ai_finished         = false;
  aiTriggered         = false;
  predicted_minutes   = 0.0;
  digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);

  Serial.println("[SCAN] Pre-ignition safety scan begun (lamp OFF, confirming room is clear).");
}

void enterWarmup(unsigned long now) {
  currentState       = WARMUP;
  warmupStartTime      = now;
  aiTriggered           = false;
  ai_needs_to_run       = false;
  ai_finished           = false;
  predicted_minutes     = 0.0;

  // The safety scan just passed — the cycle begins NOW. Lamp ON immediately
  // so it has AI_TRIGGER_DELAY_MS to reach full intensity before the LDR/AI
  // read (see the WARMUP case below).
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LEVEL);

  Serial.println("[WARMUP] Pre-scan passed -> cycle begins. Lamp ON, warming up for the AI dose read.");
}

void runStateMachine() {
  unsigned long now = millis();

  switch (currentState) {

    // ── STANDBY: idle, monitoring. Auto-arms after AUTO_START_DELAY_MS clear ──
    case STANDBY:
      digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
      countdownLeft = 0;
      scanTimeLeft  = 0;

      if (systemIsSafe) {
        if (!roomWasSafe) {
          roomWasSafe   = true;
          roomSafeStart = now;
        }
        unsigned long safeElapsed = now - roomSafeStart;
        if (safeElapsed >= AUTO_START_DELAY_MS) {
          enterScanning(now);
        } else {
          autoStartInMs = AUTO_START_DELAY_MS - safeElapsed;
        }
      } else {
        // Ordinary occupancy — not an alarm, just reset the arm countdown.
        roomWasSafe   = false;
        autoStartInMs = 0;
      }
      break;

    // ── SCANNING: pure pre-ignition safety check. Lamp stays OFF the entire
    //              10s window — no AI, no LDR read here. The cycle itself
    //              (lamp + AI dose read) only begins once this passes. ──
    case SCANNING:
      autoStartInMs = 0;
      countdownLeft = 0;
      warmupTimeLeft = 0;

      if (!systemIsSafe) {
        // Benign interruption (someone still in the room) — NOT an alarm,
        // the lamp was never on. Log exactly which sensor caused the abort.
        String reason = unsafeSensorReason();
        addLog("FAILED", 0.0, "During pre-scan: " + reason);
        currentState = STANDBY;
        scanTimeLeft = 0;
        roomWasSafe  = false;
        Serial.println("[SCAN] Aborted during pre-scan (" + reason + ") -> STANDBY.");
        break;
      }

      {
        unsigned long elapsed = now - scanStartTime;
        if (elapsed < SCAN_DURATION_MS) {
          scanTimeLeft = SCAN_DURATION_MS - elapsed;
          break;
        }

        // Pre-ignition safety scan passed for the full window — the cycle
        // now begins: lamp turns on and warm-up/AI scoring starts.
        scanTimeLeft = 0;
        enterWarmup(now);
      }
      break;

    // ── WARMUP: the cycle has begun — lamp is ON. After AI_TRIGGER_DELAY_MS
    //            (lamp at full intensity) the LDR is sampled once and the AI
    //            dose advisor is triggered exactly once per cycle. ──
    case WARMUP:
      autoStartInMs = 0;
      countdownLeft = 0;
      scanTimeLeft  = 0;

      if (!systemIsSafe) {
        // The lamp is already live here, so a breach is a real safety event
        // (same severity as an intrusion during the active cycle), not a
        // benign pre-scan interruption.
        String reason = unsafeSensorReason();
        unsafeIsHardwareFault = false;
        lastBreachReason        = "During warm-up: " + reason;
        logMessage                = "SAFETY OVERRIDE: intrusion during lamp warm-up!";
        addLog("ABORTED", 0.0, lastBreachReason);
        currentState = UNSAFE;
        warmupTimeLeft = 0;
        digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
        Serial.println("[SAFETY] Intrusion during warm-up (" + reason + ") -> LAMP OFF -> UNSAFE.");
        break;
      }

      {
        unsigned long elapsed = now - warmupStartTime;

        if (!aiTriggered) {
          if (elapsed < AI_TRIGGER_DELAY_MS) {
            warmupTimeLeft = AI_TRIGGER_DELAY_MS - elapsed;
            break;
          }
          // Lamp has had AI_TRIGGER_DELAY_MS to reach full brightness — the
          // LDR reading now actually reflects the bulb, not a cold read.
          aiTriggered     = true;
          capturedLdr      = shared_ldr;   // freeze the bulb-health reading for the gate below
          ai_needs_to_run  = true;
          warmupTimeLeft   = 0;
          Serial.print("[WARMUP] Lamp warmed up (");
          Serial.print(elapsed);
          Serial.print("ms) -> AI dose advisor triggered. LDR=");
          Serial.println(capturedLdr);
          break;   // give Core 0 a tick to pick up the inference request
        }

        if (!ai_finished) {
          warmupTimeLeft = 0;   // waiting on the advisor
          break;
        }

        // AI has an answer — run the two safety gates.
        if (capturedLdr < LDR_FAULT_THRESHOLD) {
          lastBreachReason      = "Bulb degraded (LDR " + String(capturedLdr) + " < " + String(LDR_FAULT_THRESHOLD) + ")";
          unsafeIsHardwareFault = true;
          logMessage             = "HARDWARE FAULT: bulb degradation detected. Replace bulb.";
          addLog("FAILED", predicted_minutes, lastBreachReason);
          currentState = UNSAFE;
          digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);   // lamp was on for the read — shut it off on fault
          Serial.println("[FAULT] Bulb degradation detected -> UNSAFE.");
        } else if (predicted_minutes > MAX_PREDICTED_MINUTES) {
          lastBreachReason      = "Predicted dose out of bounds (" + String(predicted_minutes, 1) + " min)";
          unsafeIsHardwareFault = true;
          logMessage             = "OPERATIONAL FAULT: attenuation too high. Cycle aborted.";
          addLog("FAILED", predicted_minutes, lastBreachReason);
          currentState = UNSAFE;
          digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
          Serial.println("[FAULT] Dose prediction out of bounds -> UNSAFE.");
        } else {
          calculatedUvcDurationMs = (unsigned long)(predicted_minutes * 60.0 * 1000.0);
          logMessage               = "UV-C dose confirmed. Sanitizing room.";
          currentState              = UVC_ACTIVE;
          uvcStartTime               = now;
          // Lamp is already ON (it's been running since warm-up began) — no
          // relay write needed here.
          Serial.println("[WARMUP] PASS -> continuing UV-C cycle at AI-predicted dose.");
        }
      }
      break;

    // ── UVC_ACTIVE: lamp on, counting down the AI-predicted duration ──
    case UVC_ACTIVE:
      autoStartInMs = 0;
      scanTimeLeft  = 0;
      warmupTimeLeft = 0;

      if (!systemIsSafe) {
        // A genuine safety event: the lamp was on and someone/something entered.
        String reason = unsafeSensorReason();
        unsafeIsHardwareFault = false;
        lastBreachReason        = "During active cycle: " + reason;
        logMessage                = "SAFETY OVERRIDE: intrusion during active cycle!";
        addLog("ABORTED", predicted_minutes, lastBreachReason);
        currentState = UNSAFE;
        countdownLeft = 0;
        digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
        Serial.println("[SAFETY] Intrusion during cycle (" + reason + ") -> LAMP OFF -> UNSAFE.");
      } else {
        unsigned long elapsed = now - uvcStartTime;
        if (elapsed >= calculatedUvcDurationMs) {
          logMessage   = "Sanitization complete! Room safe to enter.";
          addLog("COMPLETED", predicted_minutes, "Pathogens destroyed. Room certified.");
          currentState = STANDBY;
          countdownLeft = 0;
          roomWasSafe   = false;   // fresh 10s clear required before the next auto-arm
          digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
          Serial.println("[DONE] UV-C cycle complete -> STANDBY.");
        } else {
          countdownLeft = calculatedUvcDurationMs - elapsed;
        }
      }
      break;

    // ── UNSAFE: lamp forced off. Room-safety events auto-clear; hardware
    //            faults stay latched until acknowledged via Start or Stop. ──
    case UNSAFE:
      digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
      countdownLeft = 0;
      scanTimeLeft  = 0;
      warmupTimeLeft = 0;
      autoStartInMs = 0;

      if (!unsafeIsHardwareFault && systemIsSafe) {
        currentState = STANDBY;
        roomWasSafe  = false;
        Serial.println("[UNSAFE CLEARED] Sensors nominal -> STANDBY.");
      }
      break;
  }
}


// ─────────────────────────────────────────────────────────────────────────────
// WEB HANDLERS
// ─────────────────────────────────────────────────────────────────────────────
void handleRoot() {
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send_P(200, PSTR("text/html"), DASHBOARD_HTML);
}

void handleData() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");

  String json = "{";
  json += "\"thermal\":"        + String(currentThermal, 1)                             + ",";
  json += "\"radar\":\""        + String(radarMotion ? "MOTION" : "CLEAR")               + "\",";
  json += "\"door\":\""         + String(doorOpen    ? "OPEN"   : "CLOSED")              + "\",";
  json += "\"humidity\":"       + String(shared_humidity, 1)                             + ",";
  json += "\"ldr\":"            + String(shared_ldr)                                     + ",";
  json += "\"ldrThreshold\":"   + String(LDR_FAULT_THRESHOLD)                            + ",";
  json += "\"aiReady\":"        + String(ai_finished ? "true" : "false")                 + ",";
  json += "\"predictedMinutes\":" + String(predicted_minutes, 1)                          + ",";
  json += "\"isSafe\":"         + String(systemIsSafe ? "true" : "false")                + ",";
  json += "\"uvcActive\":"      + String(currentState == UVC_ACTIVE ? "true" : "false")  + ",";
  json += "\"state\":\""        + stateLabel()                                            + "\",";
  json += "\"breachReason\":\"" + lastBreachReason                                        + "\",";
  json += "\"hardwareFault\":"  + String(unsafeIsHardwareFault ? "true" : "false")        + ",";
  json += "\"log\":\""          + logMessage                                              + "\",";
  json += "\"timer\":"          + String(countdownLeft)                                   + ",";
  json += "\"total\":"          + String(calculatedUvcDurationMs)                         + ",";
  json += "\"scanTimer\":"      + String(scanTimeLeft)                                    + ",";
  json += "\"scanTotal\":"      + String(SCAN_DURATION_MS)                                + ",";
  json += "\"warmupTimer\":"    + String(warmupTimeLeft)                                  + ",";
  json += "\"warmupTotal\":"    + String(AI_TRIGGER_DELAY_MS)                             + ",";
  json += "\"autoStartIn\":"    + String(autoStartInMs)                                   + ",";
  json += "\"autoStartTotal\":" + String(AUTO_START_DELAY_MS)                             + ",";
  json += "\"cycleCount\":"     + String(totalCyclesRun)                                  + ",";
  json += "\"clients\":"        + String(WiFi.softAPgetStationNum())                      + ",";
  json += "\"uptimeSec\":"      + String(millis() / 1000)                                 + ",";

  json += "\"history\": [";
  for (int i = logCount - 1; i >= 0; i--) {
    json += "{";
    json += "\"status\":\""   + cycleLogs[i].status                          + "\",";
    json += "\"duration\":\"" + String(cycleLogs[i].durationMins, 1)         + "\",";
    json += "\"time\":\""     + cycleLogs[i].timestamp                       + "\",";
    json += "\"date\":\""     + cycleLogs[i].dateStamp                       + "\",";
    json += "\"reason\":\""   + cycleLogs[i].reason                          + "\"";
    json += "}";
    if (i > 0) json += ",";
  }
  json += "]";
  json += "}";

  server.send(200, "application/json", json);
}

void handleStart() {
  if ((currentState == STANDBY && systemIsSafe) || currentState == UNSAFE) {
    unsafeIsHardwareFault = false;
    enterScanning(millis());
    logMessage = "Manual start — lamp powering on for room sweep + AI advisor.";
    Serial.println("[CMD] Manual START.");
  } else {
    Serial.println("[CMD] START rejected — not STANDBY-safe or UNSAFE.");
  }
  server.send(200, "text/plain", "OK");
}

void handleStop() {
  if (currentState == UVC_ACTIVE || currentState == SCANNING || currentState == WARMUP) {
    addLog("ABORTED", predicted_minutes, "Manual operator stop");
  }
  currentState           = STANDBY;
  roomWasSafe             = false;
  autoStartInMs           = 0;
  countdownLeft           = 0;
  scanTimeLeft             = 0;
  warmupTimeLeft           = 0;
  unsafeIsHardwareFault   = false;
  logMessage               = "Emergency stop requested from dashboard.";
  digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
  Serial.println("[CMD] EMERGENCY STOP -> STANDBY.");
  server.send(200, "text/plain", "OK");
}


// ─────────────────────────────────────────────────────────────────────────────
// SETUP
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Aero-Sanitize AI starting...");

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);   // lamp OFF at boot, regardless of polarity

  pinMode(RADAR_PIN, INPUT);
  pinMode(DOOR_PIN,  INPUT_PULLUP);

  Wire.begin();
  Wire.setClock(400000);   // Fast Mode — cuts AMG8833 read time noticeably

  if (!rtc.begin()) {
    Serial.println("[ERROR] RTC module not found! Check I2C wiring.");
    logMessage = "WARNING: RTC clock not found. Log timestamps may be wrong.";
  } else {
    Serial.println("[OK] RTC initialized.");
    if (rtc.lostPower()) {
      Serial.println("[RTC] Battery was disconnected — calibrating to compile time.");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  }

  if (!amg.begin()) {
    Serial.println("[ERROR] AMG8833 not found! Check SDA/SCL wiring. HALTED.");
    while (true) { delay(1000); }
  }
  Serial.println("[OK] AMG8833 thermal camera ready.");

  dht.begin();
  Serial.println("[OK] DHT22 humidity sensor ready.");

  xTaskCreatePinnedToCore(runAILogic, "AI_Task", 10000, NULL, 1, &AI_Task, 0);
  Serial.println("[OK] AI advisor task spawned on Core 0.");

  WiFi.softAP("AeroSanitize-AI", NULL);
  Serial.print("[WIFI] AP live. Dashboard -> http://");
  Serial.println(WiFi.softAPIP());

  server.on("/",      HTTP_GET,  handleRoot);
  server.on("/data",  HTTP_GET,  handleData);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/stop",  HTTP_POST, handleStop);
  server.begin();

  Serial.println("[READY] STANDBY. Auto-arms after 10s of a clear room.");
}


// ─────────────────────────────────────────────────────────────────────────────
// LOOP
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();   // network first — catches incoming requests promptly

  unsigned long now = millis();
  if (now - lastSensorRead >= SENSOR_INTERVAL_MS) {
    lastSensorRead = now;
    readSensors();
    systemIsSafe = isRoomSafe();
  }

  runStateMachine();

  server.handleClient();   // network again — catches anything queued during the sensor read
}
