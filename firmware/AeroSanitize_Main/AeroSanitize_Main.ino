#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_AMG88xx.h>
#include "DHT.h"
#include <RTClib.h>                                  // Adafruit RTC library (DS3231)
#include <Preferences.h>                              // NVS-backed persistent cycle log
#include <esp_now.h>                                  // ESP-NOW receiver for remote room units
#include <idriss_mokdadi-project-1_inferencing.h>     // Edge Impulse TinyML model
#include "secrets.h"                                  // ESP-NOW keys + allowed remote MACs (copy secrets.example.h)

// ── PIN MAP ──────────────────────────────────────────────────────────────────
#define RELAY_PIN   14   // UV-C lamp relay
#define RADAR_PIN    5   // Radar OUT              HIGH = motion
#define DOOR_PIN    12   // Reed switch            INPUT_PULLUP, LOW = closed
#define DHT_PIN     15   // DHT22 humidity+temp    fed to the AI dose model + thermal baseline
#define LDR_PIN      4   // LDR (ADC1_CH3)         fed to the AI dose model + bulb-health check
// NOTE: this MUST be an ADC1-capable pin (GPIO1–10 on the ESP32-S3). GPIO34
// is a valid ADC1 pin on the *original* ESP32 but has no ADC hardware behind
// it at all on the S3 — analogRead() on a non-ADC pin silently returns 0,
// which is exactly the "always reads 0" symptom. GPIO4 is free here (I2C
// has 8/9, radar has 5), but rewire the physical LDR divider to match
// whatever pin you actually pick.
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
const int   LDR_ADC_MAX            = 4095;  // 12-bit ADC full-scale, used to invert the raw LDR reading

// ── THERMAL OCCUPANCY DETECTION (ambient-relative, not absolute) ─────────────
// A fixed "> 34.0C" cutoff false-triggers in a room that's simply hot. Instead,
// occupancy is judged by how far the AMG8833's hottest pixel rises ABOVE the
// DHT22's live ambient air reading. A person's skin-surface reading normally
// sits several degrees above whatever the room air currently is, even when
// the room itself is warm — so the delta stays a meaningful signal even on
// a hot day, whereas the absolute pixel value does not.
const float THERMAL_DELTA_THRESHOLD = 3.5f;  // 🔶 TUNE THIS: degrees above ambient that counts as "human present"
const float THERMAL_ABS_CEILING     = 42.0f; // hard safety cap regardless of ambient (e.g. overheat/fire), independent of occupancy logic

// ── DHT22 RELIABILITY TRACKING ────────────────────────────────────────────────
// The DHT22 datasheet requires >=2s between reads; the old code re-read it
// every 300ms on the same cadence as the other sensors. That either returns a
// cached/stale value or NaN depending on the library, and gets worse once the
// relay/UV-C ballast starts switching and injects noise onto the single-wire
// line. The old "if NaN, keep last good value" logic then silently hides a
// PERSISTENT failure — which is exactly why humidity looked frozen forever
// once a cycle started. This tracks real failures instead of masking them.
const unsigned long DHT_MIN_INTERVAL_MS = 2200UL; // respect the sensor's own refresh spec
const int           DHT_FAIL_LIMIT      = 5;      // consecutive failures before flagging a real fault
unsigned long lastDhtReadAttempt   = 0;
int           dhtConsecutiveFails  = 0;
bool          dhtFaulted           = false; // surfaced to dashboard/log instead of hidden
unsigned long dhtFaultSinceMs      = 0;     // 0 = not currently faulted; else millis() when the fault began
const unsigned long DHT_REINIT_AFTER_MS = 30000UL; // if stuck faulted this long, re-init the sensor instead of waiting for a power cycle

// ── HEAP DIAGNOSTICS ──────────────────────────────────────────────────────────
// The dashboard polls /data every second, forever. Building that response with
// Arduino String concatenation (the old approach) allocates and frees dozens
// of small heap blocks per request — over hours that fragments the heap badly,
// and once an allocation somewhere fails, previously-fine subsystems (DHT
// reads included) can start failing permanently with no obvious cause. This
// logs free heap periodically so a slow fragmentation leak is visible before
// it becomes a hard failure.
unsigned long lastHeapLog = 0;
const unsigned long HEAP_LOG_INTERVAL_MS = 30000UL;

// ── DUAL-CORE AI SHARED STATE ────────────────────────────────────────────────
volatile int   shared_ldr         = 0;
volatile float shared_humidity    = 0.0;
volatile float predicted_minutes  = 0.0;
volatile bool  ai_needs_to_run    = false;
// Feature snapshot fed to the model — frozen atomically at the moment the AI
// is triggered (same instant as capturedLdr below), instead of reading the
// live, still-drifting shared_ldr/shared_humidity from inside the async Core 0
// task. Two features that are each frozen at slightly different times can
// look like "the same input" cycle after cycle even when the room genuinely
// changed — this guarantees both features come from one consistent instant.
volatile int   ai_feature_ldr       = 0;
volatile float ai_feature_humidity  = 0.0;
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

// ── PERSISTENT LOG STORAGE (NVS ring buffer — survives reboot) ───────────────
// The RAM array above is wiped on every power cycle/reboot, so the "nurse's
// log" never actually lasted longer than the current session. This mirrors
// every addLog() call into flash (NVS) as a FIXED-SIZE BINARY STRUCT instead
// of Strings — the same reasoning as the handleData() heap fix: Strings here
// would mean ever-growing heap churn on every cycle, forever.
//
// Capacity is a hard cap, not a growing log: once MAX_PERSISTED_LOGS is
// reached, each new entry overwrites the OLDEST slot (a true ring buffer),
// so flash usage is flat for the life of the unit, however many cycles it
// ever runs. Sizing math (so you can safely raise/lower this):
//   sizeof(PersistedLogEntry) = 4(time) + 4(duration) + 1(status) + 40(reason) = 49B
//   NVS stores blobs in 32B pages, so each 49B entry actually costs
//   ceil(49/32)+1 = 3 pages * 32B = 96B on flash.
//   100 entries -> ~9.6KB, well inside the default ~20-24KB NVS partition
//   (check Tools > Partition Scheme in the IDE if you ever raise this a lot).
#define MAX_PERSISTED_LOGS 100
#define REASON_MAX_LEN      40
#define LOG_NS              "aerolog"   // NVS namespace, <=15 chars

struct __attribute__((packed)) PersistedLogEntry {
  uint32_t unixTime;                // RTC epoch seconds at log time
  float    durationMins;
  uint8_t  status;                  // 0=COMPLETED 1=ABORTED 2=FAILED 3=unknown
  char     reason[REASON_MAX_LEN];  // truncated, always null-terminated
};

Preferences logPrefs;
uint16_t    persistHead  = 0;  // next NVS slot index to write
uint16_t    persistCount = 0;  // entries used so far, caps at MAX_PERSISTED_LOGS

// ── BUSY HOURS TRACKING ────────────────────────────────────────────────────
// Cumulative minutes-occupied-per-hour-of-day, lifetime, for the dashboard's
// Busy Hours chart. Bucketed by hour (0-23), NOT by calendar day — this
// answers "which part of the day does this room tend to be in use", which is
// what actually helps a nurse pick a low-traffic UV-C slot; a per-day log
// would just be the existing Nurse's Log.
//
// Kept as minutes, not milliseconds or seconds: at 300ms ticks accumulated
// over a multi-year deployment, a millisecond or second counter can overflow
// uint32_t; minutes cannot realistically overflow in this device's lifetime.
// Storage: 24 * 4 bytes = 96 bytes in NVS — negligible next to the log.
//
// The RTC is only queried once a minute (OCC_HOUR_CHECK_INTERVAL_MS), not on
// every 300ms sensor tick, and NVS is only written every 15 minutes AND only
// if something actually changed (busyHoursDirty) — this keeps the feature's
// added cost on the Core 1 loop to plain integer math almost all the time.
uint32_t      occupiedMinutesByHour[24] = {0};
unsigned long occupiedMsAccumulator     = 0;    // sub-minute carry
int           cachedHourOfDay           = -1;   // -1 = not yet read from RTC
unsigned long lastHourCheckMs           = 0;
bool          busyHoursDirty            = false;
unsigned long lastBusyFlushMs           = 0;
const unsigned long OCC_HOUR_CHECK_INTERVAL_MS   = 60UL * 1000UL;        // re-check RTC hour every 1 min
const unsigned long BUSY_HOURS_FLUSH_INTERVAL_MS = 15UL * 60UL * 1000UL; // flush to NVS every 15 min

// ── ESP-NOW FLEET RECEIVER ─────────────────────────────────────────────────
// Receives ENCRYPTED beacons from standalone remote room units (see the
// companion AeroSanitize_Remote.ino). Each remote is just an LDR + ESP32
// reading light level and shouting it out — this is a DISPLAY-ONLY fleet
// status feature. Nothing received here ever reaches isRoomSafe() or the
// relay: same principle as the LDR being kept out of the local 3-sensor
// interlock (it's a diagnostic signal, not a presence vote), just extended
// to remote units too. If a beacon stopped arriving entirely, the worst
// case is a stale/offline row in a dashboard table — never a false "safe".
//
// Both sketches must broadcast/listen on the SAME WiFi channel — ESP-NOW
// doesn't hop channels to find peers. The main unit pins its softAP to
// ESPNOW_CHANNEL below; the remote sketch hardcodes the same number.
#define ESPNOW_CHANNEL     6
#define MAX_FLEET_REMOTES  4
#define FLEET_STALE_MS     8000UL   // no beacon in this long -> shown OFFLINE

struct __attribute__((packed)) RemoteBeacon {
  char     roomName[16];    // null-terminated, e.g. "Room 2"
  uint16_t ldr;             // raw ADC reading, same convention as the local LDR
  uint32_t seq;             // increments per send, lets the hub notice drops
  uint32_t remoteUptimeMs;  // informational only — never used for any timing logic
};

struct FleetSlot {
  uint8_t       mac[6];
  RemoteBeacon  lastBeacon;
  unsigned long lastSeenMs;
  bool          everSeen;
};

FleetSlot fleet[MAX_FLEET_REMOTES];

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
float ambientTemp    = 25.0f;  // DHT22 ambient air temp — sane default until first valid read
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
<meta name="theme-color" content="#080b10">
<title>Aero-Sanitize AI</title>
<style>
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#080b10;--panel:#12181f;--well:#0b1116;--line:#212b37;
  --ink:#eaf0f5;--ink-dim:#93a1b0;--ink-faint:#5c6d81;
  --go:#2fd383;--caution:#f0a83c;--stop:#ef4b5f;--info:#4c9eff;
  --radius:10px;
  --mono:ui-monospace,"Cascadia Mono","Consolas",monospace
}
html{background:var(--bg)}
body{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
  background:var(--bg);color:var(--ink);min-height:100svh;
  -webkit-font-smoothing:antialiased;overscroll-behavior:none}
.pg{max-width:440px;margin:0 auto;padding:14px 14px calc(26px + env(safe-area-inset-bottom))}
.sheet{background:var(--panel);border:1px solid var(--line);border-radius:var(--radius);overflow:hidden}

/* One divider language for every section boundary in the sheet. */
.zone{padding:16px;border-top:1px solid var(--line)}
.panel-head{display:flex;align-items:baseline;justify-content:space-between;gap:10px;margin-bottom:11px}
.panel-head>span:first-child{font-size:13.5px;font-weight:600;color:var(--ink)}
.panel-head-meta{font-size:11px;color:var(--ink-faint);white-space:nowrap;overflow:hidden;text-overflow:ellipsis}

/* ── MASTHEAD ── */
.hdr{display:flex;align-items:center;justify-content:space-between;padding:14px 16px}
.brand{font-size:14px;color:var(--ink)}
.brand b{font-weight:600}
.brand .unit{color:var(--ink-faint);font-weight:400;margin-left:5px}
.pill{display:flex;align-items:center;gap:6px;padding:4px 10px 4px 8px;
  border:1px solid var(--line);border-radius:99px;font-size:11px;font-weight:600;color:var(--ink-dim)}
.pdot{width:6px;height:6px;border-radius:50%;background:var(--go);
  animation:blink 2s ease infinite;flex:none}
.pill.off .pdot{background:var(--caution);animation-duration:.65s}

/* ── HERO STATUS — the one bold moment on the page ── */
.sc{padding:20px 16px;background:var(--well);border-top:1px solid var(--line);
  border-left:3px solid var(--ink-faint);transition:border-color .3s}
.sc.s-ready{border-left-color:var(--go)}
.sc.s-scan{border-left-color:var(--caution)}
.sc.s-active,.sc.s-unsafe{animation:aP 2s ease infinite}
.sc.s-unsafe{animation-duration:1s}
.si{display:flex;align-items:flex-start;gap:13px}
.sic{width:40px;height:40px;border-radius:10px;background:rgba(255,255,255,.04);
  display:flex;align-items:center;justify-content:center;flex:none;transition:background .3s}
.sc.s-ready .sic{background:rgba(47,211,131,.14)}
.sc.s-scan  .sic{background:rgba(240,168,60,.14)}
.sc.s-active .sic,.sc.s-unsafe .sic{background:rgba(239,75,95,.14)}
.sic svg{width:20px;height:20px;color:var(--ink-faint);transition:color .3s}
.sc.s-ready  .sic svg{color:var(--go)}
.sc.s-scan   .sic svg{color:var(--caution)}
.sc.s-active .sic svg,.sc.s-unsafe .sic svg{color:var(--stop)}
.stx{flex:1;min-width:0;padding-top:2px}
.stitle{font-size:20px;font-weight:700;letter-spacing:-.01em;line-height:1.2}
.ssub{font-size:13px;color:var(--ink-dim);margin-top:5px;line-height:1.5}

.asw{margin-top:14px;padding-top:14px;border-top:1px solid var(--line);display:none}
.asw.vis{display:block}
.aslb{font-size:12px;color:var(--ink-dim);margin-bottom:6px;
  display:flex;justify-content:space-between;align-items:center}
.aslb span{font-family:var(--mono);color:var(--go);font-size:12px;font-weight:600}
.asbar{height:3px;background:var(--line);border-radius:99px;overflow:hidden}
.asfill{height:100%;background:var(--go);border-radius:99px;width:0;transition:width .25s linear}

/* ── LIVE LOG TICKER ── */
.cl{padding:10px 16px;background:var(--well);border-top:1px solid var(--line);
  font-family:var(--mono);font-size:11px;color:var(--ink-dim);
  white-space:nowrap;overflow:hidden;text-overflow:ellipsis}

/* ── SENSOR READOUTS — one instrument list, not five repeated cards ── */
.scard{display:flex;align-items:center;gap:11px;padding:9px 0;
  border-top:1px solid var(--line);transition:background .25s}
.scard:first-child{border-top:none}
.scard.al{background:rgba(239,75,95,.07);margin:0 -16px;padding:9px 16px;border-top-color:transparent}
.iico{flex:none;width:24px;height:22px;display:flex;align-items:center;justify-content:center}
.slbl{flex:1;font-size:13px;color:var(--ink-dim)}
.sv{font-family:var(--mono);font-size:13px;font-weight:600;color:var(--ink);transition:color .25s}
.scard.al .sv{color:var(--stop)}

/* radar glyph */
.rr{width:22px;height:22px;border-radius:50%;border:1px solid var(--line);
  background:var(--well);position:relative;overflow:hidden}
.rsw{position:absolute;inset:0;
  background:conic-gradient(from 0deg,transparent 0deg,var(--info) 26deg,transparent 52deg);
  animation:spin 2.8s linear infinite;opacity:.55}
.scard.al .rsw{background:conic-gradient(from 0deg,transparent 0deg,var(--stop) 32deg,transparent 64deg);
  opacity:1;animation-duration:.85s}
.rc{position:absolute;top:50%;left:50%;width:3px;height:3px;margin:-1.5px;
  border-radius:50%;background:var(--ink-faint)}

/* thermal / ambient bar */
.tb{width:7px;height:22px;border-radius:3px;background:var(--well);
  border:1px solid var(--line);position:relative;overflow:hidden}
.tf{position:absolute;bottom:0;left:0;right:0;height:0%;
  background:linear-gradient(to top,var(--stop),var(--caution) 50%,var(--go));transition:height .5s ease}
.tline{position:absolute;bottom:72%;left:-2px;right:-2px;height:1px;background:var(--caution);opacity:.8}

/* humidity bar */
.hb{width:7px;height:22px;border-radius:3px;background:var(--well);
  border:1px solid var(--line);position:relative;overflow:hidden}
.hf{position:absolute;bottom:0;left:0;right:0;height:0%;background:var(--info);opacity:.8;transition:height .5s ease}

/* door glyph */
.dframe{width:18px;height:24px;border:1.5px solid var(--line);
  border-radius:3px;background:var(--well);position:relative;overflow:hidden}
.dleaf{position:absolute;top:2px;left:2px;right:2px;bottom:2px;background:var(--ink-faint);
  border-radius:2px;transform-origin:left center;
  transition:transform .45s cubic-bezier(.3,.7,.4,1),background .25s}
.dleaf.op{transform:scaleX(.14);background:var(--stop)}
.dknob{position:absolute;right:3px;top:50%;width:2px;height:2px;
  margin-top:-1px;border-radius:50%;background:var(--well)}

/* ── AI DOSE ADVISOR ── */
.chip{font-size:11px;font-weight:600;padding:3px 10px;border-radius:99px;
  white-space:nowrap;border:1px solid transparent}
.chip-ok{background:rgba(47,211,131,.14);color:var(--go)}
.chip-bad{background:rgba(239,75,95,.14);color:var(--stop)}
.chip-neutral{background:transparent;color:var(--ink-dim);border-color:var(--line)}
.dose-row{display:flex;align-items:baseline;justify-content:space-between;gap:10px}
.aival{font-family:var(--mono);font-size:24px;font-weight:700;letter-spacing:-.01em}
.aival.think{color:var(--caution);animation:blink 1.1s ease infinite}
.dose-meta{font-size:11.5px;color:var(--ink-faint);font-family:var(--mono);white-space:nowrap}
.dose-sub{margin-top:8px;font-size:12.5px;color:var(--ink-dim)}

/* ── TIMER RING ── */
.tmrsec{display:none;flex-direction:column;align-items:center}
.rw{position:relative;width:138px;height:138px;margin:2px auto 0}
.rsvg{width:100%;height:100%;transform:rotate(-90deg)}
.rtrack{fill:none;stroke:var(--line);stroke-width:4}
.rarc{fill:none;stroke:var(--info);stroke-width:4;stroke-linecap:round;
  stroke-dasharray:301.6;stroke-dashoffset:301.6;
  transition:stroke-dashoffset .5s linear,stroke .25s}
.tmrsec.s-scan   .rarc{stroke:var(--caution)}
.tmrsec.s-active .rarc{stroke:var(--stop)}
.rc2{position:absolute;inset:0;display:flex;flex-direction:column;
  align-items:center;justify-content:center}
.rtime{font-family:var(--mono);font-size:27px;font-weight:700}
.rlbl{font-size:12px;color:var(--ink-dim);margin-top:5px}

/* ── ACTIONS ── */
.br{display:flex;flex-direction:column;gap:9px;padding:16px}
.btn{display:flex;align-items:center;justify-content:center;gap:8px;padding:14px;
  border:none;border-radius:var(--radius);font-family:inherit;font-size:14.5px;
  font-weight:600;cursor:pointer;transition:opacity .15s,transform .1s,background .15s;
  -webkit-tap-highlight-color:transparent}
.btn:active{transform:scale(.98)}
.btn:disabled{opacity:.32;cursor:not-allowed;transform:none}
.btn.busy{opacity:.55;pointer-events:none}
.bstart{background:var(--go);color:#08301d}
.bstart:not(:disabled):hover{opacity:.9}
.bstop{background:transparent;color:var(--stop);border:1.5px solid var(--stop);
  font-weight:700;letter-spacing:.02em}
.bstop:hover{background:rgba(239,75,95,.09)}

/* ── NURSE'S LOG ── */
table.htbl{width:100%;border-collapse:collapse;font-size:12px}
.htbl th{text-align:left;font-size:11px;font-weight:600;color:var(--ink-faint);padding:0 6px 8px}
.htbl td{padding:8px 6px;border-top:1px solid var(--line);color:var(--ink-dim);vertical-align:top}
.htbl .chip{text-transform:capitalize}
.htime{font-family:var(--mono);font-size:10.5px;color:var(--ink-faint);white-space:nowrap}
.hempty{text-align:center;color:var(--ink-faint);padding:16px 0}

/* ── BUSY HOURS CHART ── */
.bhrow{display:flex;align-items:flex-end;gap:2px;height:52px}
.bhbar{flex:1;min-width:0;background:var(--well);border-radius:1px;height:4%;
  transition:height .4s ease,background .25s}
.bhbar.now{box-shadow:0 0 0 1px var(--info) inset}
.bhticks{display:flex;justify-content:space-between;margin-top:7px;
  font-family:var(--mono);font-size:9.5px;color:var(--ink-faint)}

/* ── FLEET STATUS ── */
.fsrow{display:flex;align-items:center;gap:9px;padding:8px 0;border-top:1px solid var(--line);font-size:13px}
.fsrow:first-child{border-top:none}
.fsdot{width:7px;height:7px;border-radius:50%;flex-shrink:0;background:var(--ink-faint)}
.fsdot.on{background:var(--go)}
.fsname{flex:1;color:var(--ink)}
.fsldr{font-family:var(--mono);font-size:11px;color:var(--ink-faint)}
.fsage{font-family:var(--mono);font-size:10.5px;color:var(--ink-faint);min-width:44px;text-align:right}
.fsempty{text-align:center;color:var(--ink-faint);padding:14px 0;font-size:12px}

/* ── STATS + FOOTER ── */
.stats{display:flex;justify-content:center;gap:16px;padding:12px 16px 4px;
  font-family:var(--mono);font-size:11px;color:var(--ink-faint)}
.stats b{color:var(--ink-dim);font-weight:600}
.ft{text-align:center;font-family:var(--mono);font-size:10.5px;
  color:var(--ink-faint);padding:2px 16px 14px}

/* ── ANIMATIONS ── */
@keyframes blink{0%,100%{opacity:1}50%{opacity:.3}}
@keyframes spin{to{transform:rotate(360deg)}}
@keyframes aP{0%,100%{border-left-color:rgba(239,75,95,.4)}50%{border-left-color:rgba(239,75,95,1)}}
@media(prefers-reduced-motion:reduce){
  *,*::before,*::after{animation-duration:.001ms!important;transition-duration:.001ms!important}
}
</style>
</head>
<body>
<div class="pg">
<div class="sheet">

<header class="hdr">
  <div class="brand"><b>Aero-Sanitize</b><span class="unit">AI</span></div>
  <div class="pill" id="pill">
    <span class="pdot"></span><span id="pillTxt">Live</span>
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
      <div class="stitle" id="stitle">Connecting&#8230;</div>
      <div class="ssub"   id="ssub">Establishing link to the controller.</div>
    </div>
  </div>
  <div class="asw" id="asw">
    <div class="aslb"><span>Auto-starting in</span><span id="astime">--</span></div>
    <div class="asbar"><div class="asfill" id="asf"></div></div>
  </div>
</section>

<div class="cl" id="cl">&gt; Awaiting first telemetry frame&#8230;</div>

<section class="zone">
  <div class="panel-head"><span>Sensors</span></div>
  <div id="rcrd" class="scard">
    <span class="iico"><div class="rr"><div class="rsw"></div><div class="rc"></div></div></span>
    <span class="slbl">Radar</span>
    <span class="sv" id="rv">--</span>
  </div>
  <div id="tcrd" class="scard">
    <span class="iico"><div class="tb"><div class="tf" id="tf"></div><div class="tline"></div></div></span>
    <span class="slbl">Thermal &#916;</span>
    <span class="sv" id="tv">--&#176;C</span>
  </div>
  <div id="dcrd" class="scard">
    <span class="iico"><div class="dframe"><div class="dleaf" id="dl"><div class="dknob"></div></div></div></span>
    <span class="slbl">Door</span>
    <span class="sv" id="dv">--</span>
  </div>
  <div id="hcrd" class="scard">
    <span class="iico"><div class="hb"><div class="hf" id="hf"></div></div></span>
    <span class="slbl">Humidity</span>
    <span class="sv" id="hv">--%</span>
  </div>
  <div id="acrd" class="scard">
    <span class="iico"><div class="tb"><div class="tf" id="af" style="background:var(--info)"></div><div class="tline" style="display:none"></div></div></span>
    <span class="slbl">Ambient</span>
    <span class="sv" id="av">--&#176;C</span>
  </div>
</section>

<section class="zone">
  <div class="panel-head"><span>AI dose advisor</span><span class="chip chip-neutral" id="bulbChip">Bulb &mdash;</span></div>
  <div class="dose-row">
    <div class="aival" id="aival">--</div>
    <div class="dose-meta">LDR <span id="ldrv">--</span></div>
  </div>
  <div class="dose-sub" id="aisub">Awaiting next cycle.</div>
</section>

<section class="tmrsec" id="tmrsec">
  <div class="rw">
    <svg viewBox="0 0 100 100" class="rsvg">
      <circle class="rtrack" cx="50" cy="50" r="48"/>
      <circle class="rarc"   cx="50" cy="50" r="48" id="rarc"/>
    </svg>
    <div class="rc2">
      <div class="rtime" id="rtime">--:--</div>
      <div class="rlbl"  id="rlbl">Standby</div>
    </div>
  </div>
</section>

<div class="br">
  <button class="btn bstart" id="bs" onclick="cmd('/start', this)" disabled>
    <span id="bsIcon">&#9654;</span><span id="bsTxt">Start UV-C cycle</span>
  </button>
  <button class="btn bstop" onclick="cmd('/stop', this)">
    <span>&#9632;</span><span>EMERGENCY STOP</span>
  </button>
</div>

<section class="zone">
  <div class="panel-head"><span>Nurse's log</span><span class="panel-head-meta">RTC timestamps</span></div>
  <table class="htbl">
    <thead><tr><th>Status</th><th>Dose</th><th>When</th><th>Reason</th></tr></thead>
    <tbody id="hbody"><tr><td colspan="4" class="hempty">No cycles recorded yet.</td></tr></tbody>
  </table>
</section>

<section class="zone">
  <div class="panel-head"><span>Busy hours</span><span class="panel-head-meta" id="bhsub">No occupancy data yet</span></div>
  <div class="bhrow" id="bhrow"></div>
  <div class="bhticks"><span>12a</span><span>6a</span><span>12p</span><span>6p</span><span>11p</span></div>
</section>

<section class="zone">
  <div class="panel-head"><span>Fleet status</span><span class="panel-head-meta">Remote room units</span></div>
  <div id="fsbody"><div class="fsempty">No remote units detected yet.</div></div>
</section>

<div class="stats" id="stats"><span>Cycles <b id="stCycles">--</b></span><span>Clients <b id="stClients">--</b></span><span>Up <b id="stUptime">--</b></span><span>Heap <b id="stHeap">--</b></span></div>
<div class="ft" id="ft">--</div>

</div>
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
function hourLabel(h){
  return h === 0 ? '12a' : h < 12 ? h+'a' : h === 12 ? '12p' : (h-12)+'p';
}
// Draws the 24-bar Busy Hours chart. `mins` is the device's lifetime
// minutes-occupied-per-hour array from /data — bar height/opacity are both
// scaled relative to that array's own max, so the chart is always readable
// regardless of how long the unit has been deployed. The "now" outline uses
// the BROWSER's local clock purely as a visual cue — it has no bearing on
// any safety logic, which only ever runs off the device's own RTC.
function renderBusyHours(mins){
  var row = $('bhrow');
  if(!row) return;
  if(!mins || mins.length !== 24){ row.innerHTML = ''; return; }

  var max = 0, total = 0;
  for(var i=0;i<24;i++){ if(mins[i] > max) max = mins[i]; total += mins[i]; }

  var curHour = new Date().getHours();
  var html = '';
  for(var h=0; h<24; h++){
    var v = mins[h];
    var pct = max > 0 ? Math.max(4, Math.round(v/max*100)) : 4;
    var alpha = max > 0 ? (0.16 + 0.74*(v/max)) : 0.12;
    var cls = 'bhbar' + (h === curHour ? ' now' : '');
    html += '<div class="'+cls+'" style="height:'+pct+'%;background:rgba(76,158,255,'+alpha.toFixed(2)+')"'
          + ' title="'+hourLabel(h)+'\u2013'+hourLabel((h+1)%24)+': '+v+' min occupied (lifetime)"></div>';
  }
  row.innerHTML = html;
  $('bhsub').textContent = total > 0
    ? (Math.round(total/60*10)/10) + 'h occupied, lifetime'
    : 'No occupancy data yet';
}
// Renders the Fleet Status list from /data's `fleet` array. Purely
// informational — a remote showing OFFLINE never affects this unit's own
// safety state, same as the rest of the ESP-NOW feature.
function renderFleet(fleet){
  var body = $('fsbody');
  if(!body) return;
  if(!fleet || fleet.length === 0){
    body.innerHTML = '<div class="fsempty">No remote units detected yet.</div>';
    return;
  }
  var html = '';
  fleet.forEach(function(f){
    html += '<div class="fsrow">';
    html += '<span class="fsdot' + (f.online ? ' on' : '') + '"></span>';
    html += '<span class="fsname">' + esc(f.name) + '</span>';
    html += '<span class="fsldr">LDR ' + f.ldr + '</span>';
    html += '<span class="fsage">' + (f.online ? (f.ageSec + 's ago') : 'Offline') + '</span>';
    html += '</div>';
  });
  body.innerHTML = html;
}
function esc(s){
  var d = document.createElement('div');
  d.textContent = s == null ? '' : s;
  return d.innerHTML;
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
  $('pillTxt').textContent = ok ? 'Live' : 'Offline';
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
      setCard('', 'ready', 'Connection lost', 'Retrying the link to the controller\u2026');
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
        setCard('s-unsafe','unsafe','Hardware fault','Cause: ' + reason + '. Needs attention before retrying.');
      } else {
        setCard('s-unsafe','unsafe','Cycle interrupted','Cause: ' + reason + '. Lamp is off.');
      }
    } else if(uvc){
      setCard('s-active','active','UV-C active','Stay out \u2014 running for the AI-predicted duration.');
    } else if(warm){
      setCard('s-scan','scan', aiReady ? 'Starting the cycle' : 'Lamp warming up\u2026',
        aiReady ? 'Dose confirmed. Beginning the full run.' : 'Reading bulb intensity for the AI dose model.');
    } else if(scan){
      setCard('s-scan','scan','Checking the room\u2026', 'Confirming it\u2019s clear before the lamp ignites.');
    } else {
      setCard(safe ? 's-ready' : '', 'ready',
        safe ? 'Room clear' : 'Room occupied',
        safe ? 'All sensors nominal. Ready to start a cycle.' : 'Waiting for the room to clear before arming.');
    }
  } else if(st === 'STANDBY'){
    // Safe flag can flip without a state-label change
    $('sc').className     = 'sc ' + (safe ? 's-ready' : '');
    $('stitle').textContent = safe ? 'Room clear' : 'Room occupied';
    $('ssub').textContent   = safe ? 'All sensors nominal. Ready to start a cycle.' : 'Waiting for the room to clear before arming.';
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

  // THERMAL — ambient-relative occupancy check (fixes false positives in a hot room)
  var tmp = parseFloat(d.thermal);
  var ambient = parseFloat(d.ambientTemp);
  var delta = parseFloat(d.thermalDelta);
  var deltaThresh = (typeof d.thermalDeltaThreshold === 'number') ? d.thermalDeltaThreshold : 3.5;
  var hot = !isNaN(delta) && delta > deltaThresh;
  $('tcrd').className = 'scard' + (hot ? ' al' : '');
  $('tv').innerHTML   = isNaN(tmp) ? '--&#176;C' : tmp.toFixed(1) + '&#176;C';
  $('tf').style.height = isNaN(tmp) ? '0%' : clamp((tmp-18)/(40-18)*100, 0, 100) + '%';

  // AMBIENT (DHT22 room air temperature — the new thermal baseline)
  $('av').innerHTML = isNaN(ambient) ? '--&#176;C' : ambient.toFixed(1) + '&#176;C';
  $('af').style.height = isNaN(ambient) ? '0%' : clamp((ambient-18)/(40-18)*100, 0, 100) + '%';

  // DOOR
  var op = (d.door === 'OPEN');
  $('dcrd').className = 'scard' + (op ? ' al' : '');
  $('dv').textContent = d.door || '--';
  var dl = $('dl');
  op ? dl.classList.add('op') : dl.classList.remove('op');

  // HUMIDITY (informational — feeds the AI advisor)
  var hum = parseFloat(d.humidity);
  var humOk = !isNaN(hum);
  var dhtFault = d.dhtFault === true;
  $('hcrd').className = 'scard' + (dhtFault ? ' al' : '');
  $('hv').textContent = dhtFault ? 'STALE' : (humOk ? hum.toFixed(1) + '%' : '--%');
  $('hf').style.height = (humOk ? clamp(hum,0,100) : 0) + '%';

  // AI DOSE ADVISOR + BULB HEALTH
  var ldr = parseInt(d.ldr, 10);
  var ldrOk = !isNaN(ldr);
  var thresh = (typeof d.ldrThreshold === 'number') ? d.ldrThreshold : 550;
  var bulbOk = ldrOk && ldr >= thresh;
  $('ldrv').textContent = ldrOk ? ldr : '--';
  $('bulbChip').textContent = ldrOk ? (bulbOk ? 'Bulb OK' : 'Bulb degraded') : 'Bulb \u2014';
  $('bulbChip').className = 'chip ' + (!ldrOk ? 'chip-neutral' : (bulbOk ? 'chip-ok' : 'chip-bad'));

  var predicted = parseFloat(d.predictedMinutes);
  var predictedOk = !isNaN(predicted) && predicted > 0;
  var aivalEl = $('aival');
  if(warm && !aiReady){
    aivalEl.textContent = 'Calculating\u2026';
    aivalEl.className = 'aival think';
    $('aisub').textContent = 'Lamp warming up before the TinyML dose read.';
  } else if(predictedOk){
    aivalEl.textContent = predicted.toFixed(1) + ' min';
    aivalEl.className = 'aival';
    $('aisub').textContent = uvc ? 'Running the full AI-predicted dose.' : 'Last predicted sanitization dose.';
  } else {
    aivalEl.textContent = '--';
    aivalEl.className = 'aival';
    $('aisub').textContent = scan ? 'Waiting for the pre-scan to finish.' : 'Awaiting next cycle.';
  }

  // TIMER RING — shows during SCANNING (room check), WARMUP (lamp warm-up +
  // AI read), or UV-C (AI-predicted countdown)
  if(scan || warm || uvc){
    var total, left, label;
    if(uvc){
      total = d.total || 60000;
      left  = d.timer || 0;
      label = 'UV-C cycle';
    } else if(warm){
      total = d.warmupTotal || 4000;
      left  = d.warmupTimer || 0;
      label = (left === 0 && !aiReady) ? 'Calculating dose' : 'Warming up';
    } else {
      total = d.scanTotal || 10000;
      left  = d.scanTimer || 0;
      label = 'Safety scan';
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

  // BUSY HOURS CHART
  renderBusyHours(d.busyHours);

  // FLEET STATUS
  renderFleet(d.fleet);

  // START BUTTON — valid from STANDBY+safe, or to acknowledge/retry from UNSAFE
  var canStart = (st === 'STANDBY' && safe) || st === 'UNSAFE';
  $('bs').disabled = !canStart;
  $('bsTxt').textContent = (st === 'UNSAFE') ? 'Acknowledge & retry' : 'Start UV-C cycle';

  // STATS + FOOTER
  if(typeof d.cycleCount === 'number') $('stCycles').textContent = d.cycleCount;
  if(typeof d.clients === 'number')    $('stClients').textContent = d.clients;
  if(typeof d.uptimeSec === 'number')  $('stUptime').textContent = fmtUptime(d.uptimeSec);
  if(typeof d.freeHeap === 'number')   $('stHeap').textContent = Math.round(d.freeHeap / 1024) + 'KB';
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
uint8_t statusToCode(const String &status) {
  if (status == "COMPLETED") return 0;
  if (status == "ABORTED")   return 1;
  if (status == "FAILED")    return 2;
  return 3;
}

String codeToStatus(uint8_t code) {
  switch (code) {
    case 0: return "COMPLETED";
    case 1: return "ABORTED";
    case 2: return "FAILED";
    default: return "UNKNOWN";
  }
}

// Writes one entry into the NVS ring buffer at persistHead, then advances
// the head with wraparound. This IS the "delete old logs once full" behavior
// — once persistCount hits MAX_PERSISTED_LOGS, every new write's target slot
// is by definition the oldest surviving entry, so it's silently replaced.
// Only ~3 small NVS writes happen per call (one entry blob + two counters),
// and this only runs at cycle end/abort — never on the 300ms sensor loop.
void persistLogEntry(const PersistedLogEntry &entry) {
  char key[8];
  snprintf(key, sizeof(key), "l%u", persistHead);
  logPrefs.putBytes(key, &entry, sizeof(entry));

  persistHead = (persistHead + 1) % MAX_PERSISTED_LOGS;
  if (persistCount < MAX_PERSISTED_LOGS) persistCount++;

  logPrefs.putUShort("head",  persistHead);
  logPrefs.putUShort("count", persistCount);
}

// Called once at boot: opens the NVS namespace, restores the lifetime cycle
// counter, and replays the most recent MAX_LOGS entries back into the RAM
// display buffer so the dashboard's Nurse's Log isn't empty after a reboot.
// (Older entries beyond MAX_LOGS still exist in NVS; they're just not shown
// on the small dashboard table — see the note at the end of this function.)
void loadPersistedLogs() {
  if (!logPrefs.begin(LOG_NS, false)) {
    Serial.println("[NVS] WARNING: could not open log namespace — cycle history will not persist this boot.");
    return;
  }

  persistHead    = logPrefs.getUShort("head", 0);
  persistCount   = logPrefs.getUShort("count", 0);
  totalCyclesRun = logPrefs.getULong("total", 0);

  int toLoad = min((int)persistCount, MAX_LOGS);
  logCount = 0;

  // Walk backwards from the most-recently-written slot so cycleLogs[] ends
  // up oldest-first, matching what addLog() would have produced live.
  for (int i = toLoad - 1; i >= 0; i--) {
    int slot = ((int)persistHead - 1 - i + MAX_PERSISTED_LOGS) % MAX_PERSISTED_LOGS;
    char key[8];
    snprintf(key, sizeof(key), "l%d", slot);

    PersistedLogEntry entry;
    size_t got = logPrefs.getBytes(key, &entry, sizeof(entry));
    if (got != sizeof(entry)) continue;   // shouldn't happen, but never trust flash blindly

    DateTime dt(entry.unixTime);
    char timeBuffer[16];
    sprintf(timeBuffer, "%02d:%02d:%02d", dt.hour(), dt.minute(), dt.second());
    char dateBuffer[16];
    sprintf(dateBuffer, "%02d/%02d/%04d", dt.day(), dt.month(), dt.year());

    entry.reason[REASON_MAX_LEN - 1] = '\0';  // guarantee termination regardless of flash contents
    cycleLogs[logCount] = { codeToStatus(entry.status), entry.durationMins,
                             String(timeBuffer), String(dateBuffer), String(entry.reason) };
    logCount++;
  }

  Serial.print("[NVS] Restored ");
  Serial.print(logCount);
  Serial.print(" of ");
  Serial.print(persistCount);
  Serial.print(" persisted cycle logs (lifetime total: ");
  Serial.print(totalCyclesRun);
  Serial.println(" cycles).");
}

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

  // Mirror into the NVS ring buffer so this cycle survives a reboot.
  PersistedLogEntry pEntry;
  pEntry.unixTime     = now.unixtime();
  pEntry.durationMins = durationMins;
  pEntry.status       = statusToCode(status);
  strncpy(pEntry.reason, reason.c_str(), REASON_MAX_LEN - 1);
  pEntry.reason[REASON_MAX_LEN - 1] = '\0';
  persistLogEntry(pEntry);

  totalCyclesRun++;
  logPrefs.putULong("total", totalCyclesRun);
}


// ─────────────────────────────────────────────────────────────────────────────
// BUSY HOURS TRACKING
// ─────────────────────────────────────────────────────────────────────────────
void loadBusyHours() {
  size_t got = logPrefs.getBytes("occMins", occupiedMinutesByHour, sizeof(occupiedMinutesByHour));
  if (got != sizeof(occupiedMinutesByHour)) {
    memset(occupiedMinutesByHour, 0, sizeof(occupiedMinutesByHour));
    Serial.println("[NVS] No prior busy-hours data — starting fresh.");
  } else {
    Serial.println("[NVS] Restored busy-hours history.");
  }
}

// Called once per sensor tick (every SENSOR_INTERVAL_MS) from loop(). Cheap
// integer math on every call; the only I2C traffic is the RTC hour lookup,
// which is throttled to once a minute regardless of how often this is called.
void updateBusyHours(bool occupied) {
  unsigned long now = millis();

  if (cachedHourOfDay < 0 || now - lastHourCheckMs >= OCC_HOUR_CHECK_INTERVAL_MS) {
    lastHourCheckMs   = now;
    cachedHourOfDay   = rtc.now().hour();
  }

  if (!occupied || cachedHourOfDay < 0 || cachedHourOfDay > 23) return;

  occupiedMsAccumulator += SENSOR_INTERVAL_MS;
  while (occupiedMsAccumulator >= 60000UL) {
    occupiedMsAccumulator -= 60000UL;
    occupiedMinutesByHour[cachedHourOfDay]++;
    busyHoursDirty = true;
  }
}

// Writes the whole 24-slot array as one small blob, at most every
// BUSY_HOURS_FLUSH_INTERVAL_MS, and only when something actually changed.
// A power loss can drop up to ~15 minutes of the freshest data — an
// acceptable trade for not hammering flash with a write every tick.
void flushBusyHoursIfDue(unsigned long now) {
  if (!busyHoursDirty) return;
  if (now - lastBusyFlushMs < BUSY_HOURS_FLUSH_INTERVAL_MS) return;

  lastBusyFlushMs = now;
  logPrefs.putBytes("occMins", occupiedMinutesByHour, sizeof(occupiedMinutesByHour));
  busyHoursDirty = false;
}


// ─────────────────────────────────────────────────────────────────────────────
// ESP-NOW FLEET RECEIVER
// ─────────────────────────────────────────────────────────────────────────────
// Finds an existing slot for this MAC, or claims a free one, or — if all
// MAX_FLEET_REMOTES slots are already taken by OTHER units — drops the
// beacon. Dropping is fine here: this is a display feature with a small
// fixed table, not a safety-relevant list that must never lose data.
void upsertFleetSlot(const uint8_t *mac, const RemoteBeacon &beacon) {
  int freeSlot = -1;
  for (int i = 0; i < MAX_FLEET_REMOTES; i++) {
    if (fleet[i].everSeen && memcmp(fleet[i].mac, mac, 6) == 0) {
      fleet[i].lastBeacon = beacon;
      fleet[i].lastSeenMs = millis();
      return;
    }
    if (!fleet[i].everSeen && freeSlot < 0) freeSlot = i;
  }
  if (freeSlot >= 0) {
    memcpy(fleet[freeSlot].mac, mac, 6);
    fleet[freeSlot].lastBeacon = beacon;
    fleet[freeSlot].lastSeenMs = millis();
    fleet[freeSlot].everSeen   = true;
  }
}

// ── ENCRYPTED PEERS ────────────────────────────────────────────────────────
// ESP-NOW encryption only works between registered peers: each remote's MAC
// is added here with the shared LMK, and the PMK is set once. Broadcast
// packets can't be encrypted, so remotes send unicast to this unit's softAP
// MAC. As a second layer, onEspNowRecv() also drops anything whose source MAC
// isn't in KNOWN_REMOTES. ESP32 allows only a handful of encrypted peers
// (6 by default), which is why the list is capped at MAX_FLEET_REMOTES.
static const size_t KNOWN_REMOTE_COUNT = sizeof(KNOWN_REMOTES) / sizeof(KNOWN_REMOTES[0]);
static_assert(sizeof(ESPNOW_PMK) - 1 == 16, "ESPNOW_PMK must be exactly 16 characters");
static_assert(sizeof(ESPNOW_LMK) - 1 == 16, "ESPNOW_LMK must be exactly 16 characters");
static_assert(sizeof(KNOWN_REMOTES) / sizeof(KNOWN_REMOTES[0]) <= MAX_FLEET_REMOTES,
              "KNOWN_REMOTES has more entries than MAX_FLEET_REMOTES");

bool isKnownRemote(const uint8_t *mac) {
  for (size_t i = 0; i < KNOWN_REMOTE_COUNT; i++) {
    if (memcmp(KNOWN_REMOTES[i], mac, 6) == 0) return true;
  }
  return false;
}

void registerEncryptedRemotes() {
  esp_now_set_pmk((const uint8_t *)ESPNOW_PMK);
  for (size_t i = 0; i < KNOWN_REMOTE_COUNT; i++) {
    esp_now_peer_info_t peer = {};
    memcpy(peer.peer_addr, KNOWN_REMOTES[i], 6);
    peer.channel = ESPNOW_CHANNEL;
    peer.ifidx   = WIFI_IF_AP;          // this unit runs as a softAP
    peer.encrypt = true;
    memcpy(peer.lmk, ESPNOW_LMK, 16);
    esp_err_t r = esp_now_add_peer(&peer);
    Serial.print("[ESP-NOW] Remote ");
    Serial.print(i);
    Serial.println(r == ESP_OK ? " registered (encrypted)." : " FAILED to register. Check KNOWN_REMOTES.");
  }
}

// NOTE ON ARDUINO-ESP32 CORE VERSIONS:
// This callback signature matches current cores (3.x), which pass an
// esp_now_recv_info_t* carrying the sender's MAC. Older cores (2.x and
// earlier) use a different signature instead:
//     void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len)
// If the IDE reports a signature mismatch against esp_now_register_recv_cb()
// on compile, switch to that older form and replace `info->src_addr` below
// with `mac` — the function body itself doesn't need to change.
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!isKnownRemote(info->src_addr)) return;   // only registered remotes
  if (len != sizeof(RemoteBeacon)) return;      // ignore anything that isn't our exact struct
  RemoteBeacon incoming;
  memcpy(&incoming, data, sizeof(incoming));
  incoming.roomName[sizeof(incoming.roomName) - 1] = '\0';  // guarantee termination regardless of sender
  upsertFleetSlot(info->src_addr, incoming);
}

// The room name comes from another device over the air — not free-text user
// input, but still external data, and a corrupted packet could contain a
// stray quote/backslash that would break the /data JSON for every field
// after it. This swaps anything JSON-unsafe for '_' rather than fully
// escaping it; good enough for a short room label.
void sanitizeForJson(char *dst, const char *src, size_t dstSize) {
  size_t i = 0;
  for (; i < dstSize - 1 && src[i] != '\0'; i++) {
    char c = src[i];
    dst[i] = (c == '"' || c == '\\' || (unsigned char)c < 0x20) ? '_' : c;
  }
  dst[i] = '\0';
}


// ─────────────────────────────────────────────────────────────────────────────
// CORE 0 TASK — TinyML dose inference, kept off the safety-critical Core 1 loop
// ─────────────────────────────────────────────────────────────────────────────
void runAILogic(void * parameter) {
  for (;;) {
    if (ai_needs_to_run) {
      Serial.println("[CORE 0] Triggered. Running TinyML regression inference...");

      float features[2] = { (float)ai_feature_ldr, ai_feature_humidity };
      signal_t features_signal;
      numpy::signal_from_buffer(features, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &features_signal);

      ei_impulse_result_t result = { 0 };
      run_classifier(&features_signal, &result, false);

      predicted_minutes = result.classification[0].value;
      ai_needs_to_run    = false;
      ai_finished        = true;

      // Print the actual feature vector, not just the output — if LDR/humidity
      // are barely changing cycle to cycle, the prediction won't either, and
      // this line is how you confirm whether that's a sensor problem or the
      // model genuinely seeing the same room conditions.
      Serial.print("[CORE 0] Inference complete. Features: LDR=");
      Serial.print(ai_feature_ldr);
      Serial.print(" Humidity=");
      Serial.print(ai_feature_humidity);
      Serial.print(" -> Predicted dose: ");
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

  // Your LDR wiring reads HIGH when it's dark and LOW when it's bright — the
  // opposite of what the AI model was trained on (high=bright, low=dark) and
  // the opposite of what the bulb-health threshold expects. Invert once here
  // so every downstream consumer (dashboard, AI features, LDR_FAULT_THRESHOLD
  // check) sees high=bright, low=dark, without having to know about the
  // physical wiring quirk.
  int rawLdr = analogRead(LDR_PIN);
  shared_ldr = LDR_ADC_MAX - rawLdr;

  // DHT22: read on its OWN cadence (>=2.2s), decoupled from the 300ms loop
  // that everything else uses. Reading it faster than its spec is what made
  // it look "frozen" — the library either hands back a cached value or NaN,
  // and the previous code just silently kept the last good value forever on
  // NaN with no visibility into whether it was actually still working.
  unsigned long nowDht = millis();
  if (nowDht - lastDhtReadAttempt >= DHT_MIN_INTERVAL_MS) {
    lastDhtReadAttempt = nowDht;

    float h = dht.readHumidity();
    float t = dht.readTemperature();
    bool hOk = !isnan(h);
    bool tOk = !isnan(t);

    if (hOk) shared_humidity = h;
    if (tOk) ambientTemp     = t;

    if (hOk || tOk) {
      dhtConsecutiveFails = 0;
      dhtFaulted          = false;
      dhtFaultSinceMs     = 0;
    } else {
      dhtConsecutiveFails++;
      if (dhtConsecutiveFails >= DHT_FAIL_LIMIT && !dhtFaulted) {
        dhtFaulted      = true;
        dhtFaultSinceMs = nowDht;
        Serial.println("[WARNING] DHT22: 5+ consecutive failed reads. Likely EMI from the "
                        "relay/UV-C ballast coupling into the single-wire line, or a wiring/"
                        "pull-up issue. Humidity and ambient temp are now STALE, not live.");
      }
    }

    // Self-heal: DHT bit-bang libraries can wedge internally after enough
    // failed reads — a known ESP32 quirk, often triggered or worsened by
    // WiFi's interrupt bursts stealing the microsecond-precise timing DHT
    // reads need. If it's been stuck faulted this long, re-run begin() to
    // reset the library's internal state instead of needing a physical
    // power cycle to recover.
    if (dhtFaulted && dhtFaultSinceMs != 0 && (nowDht - dhtFaultSinceMs >= DHT_REINIT_AFTER_MS)) {
      Serial.println("[RECOVERY] DHT22 stuck faulted 30s+ -> re-initializing sensor object.");
      dht.begin();
      dhtFaultSinceMs = nowDht; // if this attempt doesn't clear it, we'll retry again in another 30s
    }
  }
}

bool isRoomSafe() {
  if (doorOpen)    return false;
  if (radarMotion) return false;

  // Ambient-relative thermal check: judge occupancy by how far the hottest
  // AMG8833 pixel rises ABOVE the current DHT22 room-air reading, instead of
  // a fixed absolute cutoff. This is what fixes false positives in a room
  // that's simply warm/hot — a person's skin-surface reading still sits
  // meaningfully above ambient even when ambient itself is high.
  float delta = currentThermal - ambientTemp;
  if (delta > THERMAL_DELTA_THRESHOLD) return false;

  // Independent absolute safety ceiling (overheat/fire backstop), regardless
  // of ambient — this is NOT the occupancy check, just a hard cap.
  if (currentThermal > THERMAL_ABS_CEILING) return false;

  return true;
}

// Identifies WHICH sensor tripped an unsafe reading, in the same priority
// order as isRoomSafe() above, so every log entry can say exactly why a
// cycle failed instead of just "unsafe".
String unsafeSensorReason() {
  if (doorOpen)    return "door sensor (door opened)";
  if (radarMotion) return "radar sensor (motion detected)";

  float delta = currentThermal - ambientTemp;
  if (delta > THERMAL_DELTA_THRESHOLD) {
    return "thermal sensor (" + String(delta, 1) + "C above ambient " + String(ambientTemp, 1) +
           "C, limit " + String(THERMAL_DELTA_THRESHOLD, 1) + "C)";
  }
  if (currentThermal > THERMAL_ABS_CEILING) {
    return "thermal sensor (" + String(currentThermal, 1) + "C exceeds absolute ceiling " +
           String(THERMAL_ABS_CEILING, 1) + "C)";
  }
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
        // Relay OFF first, logging after: addLog() now does an RTC read plus
        // an NVS flash write, both slower than a digitalWrite(). On a real
        // intrusion the lamp needs to die before anything else runs.
        currentState = UNSAFE;
        warmupTimeLeft = 0;
        digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
        addLog("ABORTED", 0.0, lastBreachReason);
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
          capturedLdr      = shared_ldr;        // freeze the bulb-health reading for the gate below
          ai_feature_ldr      = capturedLdr;       // same frozen instant, fed to the AI
          ai_feature_humidity = shared_humidity;   // frozen alongside it — not read separately later by Core 0
          ai_needs_to_run  = true;
          warmupTimeLeft   = 0;
          Serial.print("[WARMUP] Lamp warmed up (");
          Serial.print(elapsed);
          Serial.print("ms) -> AI dose advisor triggered. LDR=");
          Serial.print(capturedLdr);
          Serial.print(" Humidity=");
          Serial.println(ai_feature_humidity);
          if (dhtFaulted) {
            Serial.println("[WARNING] DHT22 is currently flagged FAULTED — the humidity feature "
                            "above is a stale cached value, not a live reading. Predictions will "
                            "look repetitive until this clears.");
          }
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
        // Relay OFF first, logging after — see the matching note in the
        // WARMUP branch above. This is the highest-stakes case of the two:
        // someone entering mid-sanitization, lamp live.
        currentState = UNSAFE;
        countdownLeft = 0;
        digitalWrite(RELAY_PIN, RELAY_IDLE_LEVEL);
        addLog("ABORTED", predicted_minutes, lastBreachReason);
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

  // Built with a fixed buffer + snprintf instead of Arduino String
  // concatenation. This handler runs on EVERY dashboard poll (once a second,
  // forever) — the old String-based version allocated and freed dozens of
  // small heap blocks per request, which fragments the heap over hours of
  // continuous operation. That's the most likely reason the whole system
  // (DHT22 included) can degrade or lock up permanently after "some cycles"
  // rather than recovering. `static` keeps this off the stack.
  // 3000 -> 3400: fits up to MAX_FLEET_REMOTES fleet entries (~80B each) added below.
  static char json[3400];
  int len = 0;

  len += snprintf(json + len, sizeof(json) - len,
    "{\"thermal\":%.1f,\"ambientTemp\":%.1f,\"thermalDelta\":%.1f,\"thermalDeltaThreshold\":%.1f,"
    "\"radar\":\"%s\",\"door\":\"%s\",\"humidity\":%.1f,\"dhtFault\":%s,\"dhtFailCount\":%d,"
    "\"ldr\":%d,\"ldrThreshold\":%d,\"aiReady\":%s,\"predictedMinutes\":%.1f,"
    "\"isSafe\":%s,\"uvcActive\":%s,\"state\":\"%s\",\"breachReason\":\"%s\",\"hardwareFault\":%s,"
    "\"log\":\"%s\",\"timer\":%lu,\"total\":%lu,\"scanTimer\":%lu,\"scanTotal\":%lu,"
    "\"warmupTimer\":%lu,\"warmupTotal\":%lu,\"autoStartIn\":%lu,\"autoStartTotal\":%lu,"
    "\"cycleCount\":%lu,\"clients\":%d,\"uptimeSec\":%lu,\"freeHeap\":%u,",
    currentThermal, ambientTemp, currentThermal - ambientTemp, THERMAL_DELTA_THRESHOLD,
    radarMotion ? "MOTION" : "CLEAR", doorOpen ? "OPEN" : "CLOSED", shared_humidity,
    dhtFaulted ? "true" : "false", dhtConsecutiveFails,
    shared_ldr, LDR_FAULT_THRESHOLD, ai_finished ? "true" : "false", predicted_minutes,
    systemIsSafe ? "true" : "false", (currentState == UVC_ACTIVE) ? "true" : "false",
    stateLabel().c_str(), lastBreachReason.c_str(), unsafeIsHardwareFault ? "true" : "false",
    logMessage.c_str(), countdownLeft, calculatedUvcDurationMs, scanTimeLeft, SCAN_DURATION_MS,
    warmupTimeLeft, AI_TRIGGER_DELAY_MS, autoStartInMs, AUTO_START_DELAY_MS,
    totalCyclesRun, WiFi.softAPgetStationNum(), (unsigned long)(millis() / 1000),
    (unsigned int)ESP.getFreeHeap());

  // Busy Hours: 24 lifetime minutes-occupied buckets, index 0 = midnight-1am.
  len += snprintf(json + len, sizeof(json) - len, "\"busyHours\":[");
  for (int h = 0; h < 24 && len < (int)sizeof(json) - 200; h++) {
    len += snprintf(json + len, sizeof(json) - len,
      "%lu%s", (unsigned long)occupiedMinutesByHour[h], (h < 23) ? "," : "");
  }
  len += snprintf(json + len, sizeof(json) - len, "],");

  // Fleet Status: remote room units heard over ESP-NOW. Display-only — see
  // the note at the top of the ESP-NOW FLEET RECEIVER section.
  len += snprintf(json + len, sizeof(json) - len, "\"fleet\":[");
  bool firstFleet = true;
  for (int i = 0; i < MAX_FLEET_REMOTES && len < (int)sizeof(json) - 200; i++) {
    if (!fleet[i].everSeen) continue;
    unsigned long ageMs = millis() - fleet[i].lastSeenMs;
    bool online = ageMs < FLEET_STALE_MS;
    char safeName[16];
    sanitizeForJson(safeName, fleet[i].lastBeacon.roomName, sizeof(safeName));
    len += snprintf(json + len, sizeof(json) - len,
      "%s{\"name\":\"%s\",\"ldr\":%u,\"online\":%s,\"ageSec\":%lu}",
      firstFleet ? "" : ",", safeName, fleet[i].lastBeacon.ldr,
      online ? "true" : "false", ageMs / 1000);
    firstFleet = false;
  }
  len += snprintf(json + len, sizeof(json) - len, "],");

  len += snprintf(json + len, sizeof(json) - len, "\"history\": [");
  for (int i = logCount - 1; i >= 0 && len < (int)sizeof(json) - 200; i--) {
    len += snprintf(json + len, sizeof(json) - len,
      "{\"status\":\"%s\",\"duration\":\"%.1f\",\"time\":\"%s\",\"date\":\"%s\",\"reason\":\"%s\"}%s",
      cycleLogs[i].status.c_str(), cycleLogs[i].durationMins, cycleLogs[i].timestamp.c_str(),
      cycleLogs[i].dateStamp.c_str(), cycleLogs[i].reason.c_str(), (i > 0) ? "," : "");
  }
  len += snprintf(json + len, sizeof(json) - len, "]}");

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

  loadPersistedLogs();  // restore Nurse's Log + lifetime cycle count from NVS
  loadBusyHours();      // restore the Busy Hours chart data from NVS

  dht.begin();
  Serial.println("[OK] DHT22 humidity + ambient temperature sensor ready.");

  xTaskCreatePinnedToCore(runAILogic, "AI_Task", 10000, NULL, 1, &AI_Task, 0);
  Serial.println("[OK] AI advisor task spawned on Core 0.");

  // Channel pinned explicitly (was NULL/auto before) so the remote unit(s)
  // can hardcode the same channel and find this unit on it. Pairing is done
  // by MAC address and the shared keys in secrets.h. See ESPNOW_CHANNEL above.
  WiFi.softAP("AeroSanitize-AI", NULL, ESPNOW_CHANNEL);
  Serial.print("[WIFI] AP live on channel ");
  Serial.print(ESPNOW_CHANNEL);
  Serial.print(". Dashboard -> http://");
  Serial.println(WiFi.softAPIP());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] WARNING: init failed — Fleet Status will show no remote units. Dashboard/safety logic are unaffected.");
  } else {
    Serial.print("[ESP-NOW] Hub softAP MAC (put this in each remote's HUB_MAC): ");
    Serial.println(WiFi.softAPmacAddress());
    registerEncryptedRemotes();
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.println("[OK] ESP-NOW fleet receiver listening (encrypted).");
  }

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
    updateBusyHours(!systemIsSafe);   // same tick as the safety read — reuses systemIsSafe, no extra sensor work
  }

  runStateMachine();
  flushBusyHoursIfDue(now);   // cheap check every loop; actual NVS write throttled inside

  if (now - lastHeapLog >= HEAP_LOG_INTERVAL_MS) {
    lastHeapLog = now;
    Serial.print("[HEAP] Free: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes. A steady downward trend over hours points to heap "
                    "fragmentation as the cause of any long-run lockups.");
  }

  server.handleClient();   // network again — catches anything queued during the sensor read
}
