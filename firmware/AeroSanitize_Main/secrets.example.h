#pragma once
// Copy this file to secrets.h (same folder) and fill it in. secrets.h is
// gitignored, so your real keys never reach GitHub.

// Both keys must be EXACTLY 16 characters and identical on the hub and every
// remote. Generate your own random ones.
#define ESPNOW_PMK "CHANGE_ME_PMK_16"
#define ESPNOW_LMK "CHANGE_ME_LMK_16"

// MAC addresses of the remote units allowed to report to this hub, as printed
// by each remote on Serial at boot ("This unit's MAC"). Up to
// MAX_FLEET_REMOTES (4) entries. Add one line per remote.
static const uint8_t KNOWN_REMOTES[][6] = {
  { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },   // Room 2
};
