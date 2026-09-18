#pragma once
// Copy this file to secrets.h (same folder) and fill it in. secrets.h is
// gitignored, so your real keys never reach GitHub.

// Both keys must be EXACTLY 16 characters and identical on the hub and every
// remote. Generate your own random ones.
#define ESPNOW_PMK "CHANGE_ME_PMK_16"
#define ESPNOW_LMK "CHANGE_ME_LMK_16"

// The HUB's softAP MAC address (printed on the hub's Serial monitor at boot
// as "[ESP-NOW] Hub softAP MAC"). Not the hub's station MAC.
static const uint8_t HUB_MAC[6] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
