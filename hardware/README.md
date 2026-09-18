# Hardware

## Bill of materials, main unit

| Qty | Part | Role |
|---|---|---|
| 1 | ESP32-S3 dev board | Controller, Wi-Fi access point, ESP-NOW receiver |
| 1 | AMG8833 8×8 thermal sensor breakout | Thermal guard, I²C address 0x69 |
| 1 | CDM324 Doppler radar module | Radar guard |
| 1 | LM358 dual op-amp | Radar conditioning: amplifier stage and comparator stage |
| 1 | Potentiometer | Comparator threshold (radar sensitivity) |
| 3 | 330 Ω resistors | 5 V to 3.3 V divider on the radar output |
| 2 | MC-38 magnetic reed switch and magnet | Door closed sensing |
| 1 | DHT22 | Ambient temperature and humidity |
| 1 | DS3231 RTC module with coin cell | Timestamps for the log and busy hours, I²C address 0x68 |
| 1 | LDR and a fixed resistor | Bulb-health check, wired as a voltage divider |
| 1 | 5 V relay module, active LOW | Switches the lamp |
| 1 | 220 V blue LED bulb and holder | Demo stand-in for the UV-C tube |
| 1 | 5 V supply | Powers the board and modules |

## Bill of materials, remote room unit

| Qty | Part |
|---|---|
| 1 | ESP32 dev board |
| 1 | LDR and a fixed resistor (divider) |

## Wiring summary

| Signal | ESP32-S3 GPIO |
|---|---|
| Relay IN | 14 |
| Radar OUT (after divider) | 5 |
| Door reed switches | 12 (to GND, internal pull-up) |
| DHT22 data | 15 |
| LDR divider midpoint | 4 |
| SDA | 8 |
| SCL | 9 |

The two reed switches are wired in series into the single door input, so opening either door reads as OPEN.

## Build notes

**Analog pins.** Use ADC1 pins only (GPIO 1 to 10 on the S3). ADC2 doesn't work reliably while Wi-Fi is on.

**I²C.** The AMG8833 and the DS3231 share SDA and SCL. They can only coexist because they sit on different addresses: keep the AMG8833 on 0x69, since the DS3231 owns 0x68. The bus runs at 400 kHz to keep the thermal read short.

**Level shifting.** The radar comparator swings to 5 V and the ESP32-S3 GPIOs are not 5 V tolerant, so the divider on the radar output is required, not optional.

**Radar tuning.** Set the potentiometer once the unit is mounted and the room is in its normal state. Too sensitive and ventilation or a curtain will trip it. Too dull and it stops seeing breathing.

**Thermal threshold.** After mounting, look at the thermal delta on the dashboard with the room empty and then with a person sitting still, and adjust `THERMAL_DELTA_THRESHOLD` if needed.

**Noise.** The DHT22 uses a timing-sensitive single-wire protocol and is the first thing to misbehave when the relay switches. Keep its wire short, use a pull-up, and route it away from the relay and mains wiring. The firmware flags repeated read failures as STALE on the dashboard.

**Mains.** The relay contacts carry 220 V. Enclose them, keep them apart from the low-voltage side, and have someone qualified check the wiring. Use the LED bulb until you have a proper reason to use anything else.

## Photos

Add wiring diagrams and build photos to `docs/images/` and reference them from the main README.
