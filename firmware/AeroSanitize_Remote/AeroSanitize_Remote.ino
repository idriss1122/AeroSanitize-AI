// ═══════════════════════════════════════════════════════════════════════════
//  AERO-SANITIZE AI — REMOTE ROOM UNIT
// ═══════════════════════════════════════════════════════════════════════════
//  A minimal, standalone ESP32 + LDR that sends its light-level reading to
//  the AeroSanitize hub over encrypted ESP-NOW, for the hub's Fleet Status
//  dashboard card. This sketch has NO relay, NO safety logic, and makes NO
//  decisions of its own. A bug or crash here can, at worst, make one
//  dashboard row go stale. It can never affect whether a UV-C lamp turns on.
//
//  The onboard LED is deliberately never touched: the pin is not configured
//  and nothing is written to it, so it stays dark. (A hardware power LED
//  wired straight to 3V3 can't be turned off in software.)
//
//  SETUP:
//   1. Copy secrets.example.h to secrets.h in this folder and fill it in:
//      the same PMK and LMK as the hub, plus the hub's softAP MAC address
//      (the hub prints it on Serial at boot).
//   2. Wire the LDR as a voltage divider into an ADC1 pin (LDR_PIN).
//        - Classic ESP32: ADC1 = GPIO32-39. GPIO34 is a safe default.
//        - ESP32-S3: ADC1 = GPIO1-10, change LDR_PIN accordingly.
//   3. Set ROOM_NAME. ESPNOW_CHANNEL must equal the hub's value (6).
//   4. Flash, open Serial at 115200. The unit prints its own MAC on boot.
//      Put that MAC into KNOWN_REMOTES in the HUB's secrets.h and reflash
//      the hub, otherwise the hub will ignore this unit.
// ═══════════════════════════════════════════════════════════════════════════

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "secrets.h"   // ESPNOW_PMK, ESPNOW_LMK, HUB_MAC. Never commit this file.

// ── CONFIG ────────────────────────────────────────────────────────────────
#define ROOM_NAME        "Room 2"   // shown on the hub's Fleet Status card (15 chars max)
#define ESPNOW_CHANNEL   6          // MUST match AeroSanitize_Main.ino
#define LDR_PIN          4         // ADC1 pin. On an ESP32-S3 use GPIO 1-10
#define SEND_INTERVAL_MS 2000UL     // how often to send a reading
#define LDR_INVERT       0          // 1 = send (4095 - raw) so higher = brighter, like the hub's own LDR

static_assert(sizeof(ESPNOW_PMK) - 1 == 16, "ESPNOW_PMK must be exactly 16 characters");
static_assert(sizeof(ESPNOW_LMK) - 1 == 16, "ESPNOW_LMK must be exactly 16 characters");

// ── WIRE FORMAT: must be byte-for-byte identical to the copy of this
//    struct in AeroSanitize_Main.ino. If you change one, change both. ──────
struct __attribute__((packed)) RemoteBeacon {
  char     roomName[16];
  uint16_t ldr;
  uint32_t seq;
  uint32_t remoteUptimeMs;
};

uint32_t sendSeq = 0;
unsigned long lastSendMs = 0;

// Delivery report, for Serial debugging only.
// This signature matches recent Arduino-ESP32 3.x cores (wifi_tx_info_t*).
// On an older core use: void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status)
void onEspNowSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS
    ? "[ESP-NOW] Delivered (hub ACKed)."
    : "[ESP-NOW] Send failed. Check the hub MAC (must be its softAP MAC), channel and keys.");
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] AeroSanitize Remote Room Unit starting...");

  // STA mode, never joins a network. ESP-NOW rides on the radio only.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  Serial.print("[INFO] This unit's MAC (add it to KNOWN_REMOTES on the hub): ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed. Halting.");
    while (true) { delay(1000); }
  }

  // Encryption: PMK protects the key exchange, the per-peer LMK encrypts the
  // frames. Broadcast peers can't be encrypted, so this is a unicast link to
  // the hub's MAC.
  esp_now_set_pmk((const uint8_t *)ESPNOW_PMK);
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, HUB_MAC, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = true;
  memcpy(peer.lmk, ESPNOW_LMK, 16);
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[ERROR] Failed to register the encrypted hub peer. Halting.");
    while (true) { delay(1000); }
  }

  Serial.print("[OK] Sending encrypted as \"");
  Serial.print(ROOM_NAME);
  Serial.print("\" on channel ");
  Serial.println(ESPNOW_CHANNEL);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSendMs < SEND_INTERVAL_MS) return;
  lastSendMs = now;

  int raw = analogRead(LDR_PIN);
#if LDR_INVERT
  raw = 4095 - raw;
#endif

  RemoteBeacon beacon;
  memset(&beacon, 0, sizeof(beacon));
  strncpy(beacon.roomName, ROOM_NAME, sizeof(beacon.roomName) - 1);
  beacon.ldr            = (uint16_t)raw;
  beacon.seq            = sendSeq++;
  beacon.remoteUptimeMs = now;

  esp_err_t result = esp_now_send(HUB_MAC, (uint8_t *)&beacon, sizeof(beacon));

  Serial.print("[TX] seq=");
  Serial.print(beacon.seq);
  Serial.print(" ldr=");
  Serial.print(beacon.ldr);
  Serial.println(result == ESP_OK ? " queued OK" : " queue FAILED");
}
