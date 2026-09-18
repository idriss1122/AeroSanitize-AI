# Design decisions

Why the system looks the way it does, including the versions we tried and dropped.

## Safety is plain boolean logic

The lamp is controlled by `isRoomSafe()`, which is an AND of the door, radar and thermal checks. The relay does not depend on a model output, a network message or anything else that can be uncertain.

We considered letting a neural network judge occupancy from the sensor data and dropped the idea. A classifier can be right 99% of the time, and the 1% here is a person under a UV-C lamp. The TinyML model is limited to recommending a run time, capped at 60 minutes, while the guards keep running.

## Why not PIR

PIR sensors respond to changes in infrared. A patient who is anesthetized, sedated or asleep does not change anything, and the sensor reports an empty room. That is the failure this project exists to avoid. Doppler radar picks up breathing-scale movement, and the thermal array sees body heat whether or not anything is moving.

## Three guards, and the LDR is not one of them

An early draft counted the LDR as a fourth presence vote. We removed it. A dark room is not an empty room, so light level says nothing about occupancy. The LDR is now purely an equipment check: after warm-up, if the reading next to the lamp is below the threshold the bulb is considered dead or degraded, the lamp is shut off, and the unit latches a hardware fault.

## Thermal check is relative to the room

A fixed "above 34 °C means a person" rule fires constantly in a hot room. The firmware compares the hottest thermal pixel to the DHT22 room air reading and trips when the gap exceeds a threshold (3.5 °C by default). A separate absolute ceiling at 42 °C stays in place as a backstop for overheating or fire, independent of the occupancy logic.

## Relay off first, log second

When a guard trips while the lamp is on, the code sets the relay to its idle level before doing anything else. The log write involves an I²C RTC read and a flash write, both slower than a GPIO write, so they happen after the lamp is already off.

## Two states for "unsafe"

An intrusion is a room-safety event: once the room is clear again the unit returns to standby by itself. A bulb or dose fault is a hardware problem that needs a person, so it stays latched until an operator acknowledges it with Start or clears it with Stop.

## No physical start button, no cloud

The operator starts a cycle from the dashboard so they can be outside the room when it begins. The unit runs its own Wi-Fi access point instead of joining hospital Wi-Fi, so it works in clinics without IT infrastructure and doesn't depend on anyone else's network.

## Remote units are display-only

The fleet feature is kept away from safety on purpose. A remote unit only reports a light level, and nothing it sends is read by `isRoomSafe()` or the relay code. If a remote crashes or a packet is lost, the worst outcome is a stale row in a table.

## Core layout

The Arduino `loop()` runs on Core 1 and holds the sensor read, the safety check, the state machine, the relay writes and the web server polling. The TinyML inference runs as its own task on Core 0 so the heavy computation never stalls the loop. There are no `delay()` calls in the runtime path, only `millis()` timers and a `vTaskDelay` in the inference task.

## Radar signal chain

The CDM324's output is at microvolt level. Stage 1 is an AC-coupled non-inverting LM358 amplifier with roughly 100× gain; stage 2 is an LM358 comparator with a potentiometer for the threshold. The comparator's 5 V output goes through a resistor divider to about 3.3 V before it reaches the ESP32-S3.

## ADC bank

All analog reads use ADC1 (GPIO 1 to 10 on the S3). ADC2 is shared with the Wi-Fi radio and gives invalid reads or lockups once Wi-Fi is active, which for this device is always.

## Demo lamp

The demo switches a 220 V blue LED bulb through the same relay a UV-C tube would use. The logic under test is identical and nobody needs protective gear at a demo table.

## Heap and memory

The `/data` endpoint is polled every second, indefinitely. Building that JSON with Arduino `String` concatenation allocated and freed dozens of small blocks per request and fragmented the heap over hours. It now writes into one static buffer with `snprintf`, and free heap is logged every 30 seconds so a slow leak shows up in the Serial output before it turns into a lockup.
