# PinDex

**Smart Component & Sensor Identifier**, an ESP32 instrument for electronics
labs. The goal is to let students insert a component or sensor module, have
it identified, and get its pinout.

> **Status: v0.2 (Phase 2).** Adds DHT11-compatible detection, A/B/C pinout
> identification and a live temperature/humidity display.
> **Powered probing is disabled by default** (`PINDEX_POWERED_PROBING_ENABLED 0`).
> With the default build the probes stay high impedance and the display
> shows *Probing disabled*. Nothing here identifies arbitrary components,
> and inserting arbitrary components is not safe.

## Project files

| File | Purpose |
|---|---|
| `PinDex.ino` | Sketch entry point: startup order, screen updates, Serial heartbeat and the `r` retry command. |
| `config.h` | All pins, timings, thresholds, failure limits and the powered-probing setting. |
| `display_manager.h/.cpp` | VSPI + SSD1306 setup, splash, idle, status and DHT11 result screens. |
| `probe_engine.h/.cpp` | Owns the probe GPIOs: precheck, power sequencing, DHT capture, scan/confirm/monitor state machine. |
| `dht11.h/.cpp` | Pure DHT11 frame decoding (timing → bits → bytes → values); no GPIO, host-testable. |
| `tests/` | Host-side tests with a mock Arduino layer. Not compiled by the Arduino IDE. |
| `README.md` | This document. |
| `.gitignore` | Keeps build output and editor/OS files out of version control. |

## Hardware

- Board: **original ESP32** (e.g. ESP32-WROOM-32 DevKit, 30- or 38-pin).
  ESP32-C3, S2 and S3 are not supported.
- Display: **7-pin SPI SSD1306 OLED, 128×64**, on hardware VSPI. No MISO wire.

GPIO numbers are the board's **GPIO labels**, not physical header positions.

| Connection | ESP32 connection |
|---|---|
| OLED D0 / CLK / SCK | GPIO 18 |
| OLED D1 / MOSI / DATA | GPIO 23 |
| OLED RES / Reset | GPIO 4 |
| OLED DC / Data-Command | GPIO 2 |
| OLED CS / Chip Select | GPIO 5 |
| OLED VCC | 3.3V |
| OLED GND | GND |
| Probe A | GPIO 25 |
| Probe B | GPIO 26 |
| Probe C | GPIO 27 |

Probes A, B and C are **generic test nodes**. They are not fixed VCC, GND or
signal terminals.

**Strapping pins:** GPIO 2 and GPIO 5 affect how the ESP32 boots. Anything
attached to them must not force incompatible levels during reset (GPIO 2
must not be held high, GPIO 5 must not be held low). The OLED inputs are
fine. On some DevKits the on-board LED is on GPIO 2 and may flicker while
the display updates; that is normal.

## Powered probing: prerequisites

Three bare GPIOs are **not** a protected component tester. Trying all six
power permutations deliberately reverse-biases the part and drives its DATA
pin as a supply in some of them. Before setting
`PINDEX_POWERED_PROBING_ENABLED` to `1`, the probes must go through a
**reviewed interface** that provides:

- suitable current limiting on every node, for every permutation;
- protection against wrong polarity and wrong pin assignment for the
  intended parts, including what an onboard module resistor does in each
  permutation;
- defined fault handling, for example shorted nodes or a charged capacitor;
- the DATA pull-up arrangement (see the DHT11 profile below).

Series resistors alone are not comprehensive protection. Enabling the
setting is a deployment step, **not** a safety mechanism. The firmware
cannot detect a missing or faulty interface.

The selected VCC and GND are ordinary GPIO outputs at nominal 3.3V logic
levels, not regulated supply rails; the voltage sags under load. Never
"fix" a 5V-only module by applying 5V to ESP32 GPIOs.

## Permutation scan

Each permutation assigns the three roles; the scan order is fixed:

| Index | VCC | DATA | GND |
|---|---|---|---|
| 1 | A | B | C |
| 2 | C | B | A |
| 3 | B | A | C |
| 4 | B | C | A |
| 5 | A | C | B |
| 6 | C | A | B |

For every attempt the engine runs the precheck, then powers up in this
order: GND driven LOW, VCC driven HIGH, DATA as `INPUT_PULLUP`. On the
ESP32 core 3.x, `digitalWrite()` before `pinMode(OUTPUT)` sets the output
latch first, so a pin never briefly drives the opposite level.

Before moving to another permutation the engine tears down in this order:
DATA pull-up/drive removed, VCC released, then all nodes set to plain
`INPUT`. It then waits `PROBE_POWER_OFF_MS` (300 ms). That timed wait does
not prove a capacitor has discharged, and the firmware never shorts nodes
to discharge them.

## Pre-power check (diagnostic heuristic only)

Before every power-up, each node in turn is set to `INPUT_PULLUP` while the
other two are `INPUT_PULLDOWN`. After a short settle time, all three nodes
are sampled four times, then everything returns to plain `INPUT`.

- A pulled-down node reading HIGH → `SuspiciousCoupling`.
- Any node changing between samples → `InconsistentReadings`.
- Either result resets the probes, cancels the scan and **latches a fault**
  (*Socket rejected*). Send `r` in the Serial Monitor to clear it; this
  reruns the whole precheck and scan. It never overrides the
  powered-probing setting.
- Otherwise the result is `NoObviousCoupling`. That is **not** a pass and
  does not mean "safe to power".

Limits: the ESP32's internal pull-up and pull-down are both weak (tens of
kΩ) and roughly equal. Even a dead short between two nodes forms a
mid-rail divider that may read either way, and a short across all three
nodes reads LOW. No resistance or continuity is measured. The check can
also **reject genuine parts**: a module's DATA pull-up resistor or a chip's
ESD diodes can couple a pulled-up node to a pulled-down one. Characterise
this on the protected interface before relying on it. The external
protection remains necessary either way.

## Supported DHT11 profile

DHT sensors use their own single-wire protocol; it is not Dallas/Maxim
1-Wire. PinDex captures it in custom code with no DHT library.

- **Start:** DATA held LOW for 20 ms (open-drain), then released; DATA is
  never driven HIGH.
- **Response:** about 80 µs LOW + 80 µs HIGH acknowledgement, then 40 bits,
  MSB first. Each bit is about 50 µs LOW followed by about 27 µs HIGH (0) or
  about 70 µs HIGH (1); the threshold is 50 µs.
- **Timeouts:** 120 µs per phase and 6500 µs for the whole response. A
  stuck line, a truncated frame or no sensor always ends the capture.
- **Encoding accepted:**
  - byte 0 = humidity (integer %RH)
  - byte 1 = 0
  - byte 2 = temperature (integer °C)
  - byte 3 = temperature tenths, 0–9, sign bit clear
  - byte 4 = (byte 0 + byte 1 + byte 2 + byte 3) mod 256

  Any other layout is rejected as `unsupported-encoding`. That includes
  DHT22/AM2302 frames and negative temperatures flagged with the sign bit.
- **Plausibility window:** 5–95 %RH and 0–50 °C (configurable). The
  checksum alone is not trusted: an all-zero frame has a valid checksum but
  is rejected.
- **Supply:** many DHT11 datasheets list 3.3–5.5V. Confirm that your exact
  module works from about 3.3V (less under load) before testing.
- **Pull-up:** the internal pull-up is weak. DHT11 needs a DATA pull-up,
  typically 4.7–5.1 kΩ, to the sensor's own VCC. Many 3-pin modules include
  one on the board. Because the mapping is arbitrary, do not hard-wire a
  resistor between socket nodes. The protected interface must provide the
  pull-up for whichever node becomes DATA, and must account for a module's
  onboard resistor in every permutation.

The result is reported as **DHT11-compatible**. The protocol has no model
ID, so PinDex cannot prove the part is a DHT11, and it does not identify
every DHT-family sensor.

## Behaviour

**Default build (probing disabled):** splash, then
*v0.2 Ready / Probing disabled*. The probes stay plain `INPUT` and the
Serial heartbeat continues once per second.

**Powered build:**

1. **Scan:** the precheck runs, then permutation 1 is powered. The engine
   waits 1100 ms, sends the start pulse and captures the reply. A failed
   permutation is torn down before the next one. The screen shows
   *Scanning n/6*.
2. **No result:** if all six permutations fail, the screen shows
   *No Component / Unknown*, the probes are reset and the scan repeats
   after 5 s.
3. **Candidate:** on the first valid frame the mapping is saved and stays
   powered, and scanning stops (*DHT11 candidate / Confirming...*).
4. **Confirmation:** another frame is requested 2100 ms later. Two
   consecutive valid frames publish the result. The readings do not have
   to match.
5. **Live display:** the result screen is redrawn only when something
   shown changes:

   ```text
   DHT11 Temp & Hum
   T: 25*C  |  H: 60%
   A:VCC | B:DAT | C:GND
   LIVE
   ```

   The mapping line always lists A, B, C in that order. Readings repeat
   every 2100 ms.
6. **Failures:** any capture or validation failure immediately tears down
   power. The reading is marked stale (inverted *STALE - fail n/3* bar) and
   the failure counter increments. Only the saved mapping is retried, after
   a 1000 ms cooldown, a full precheck and a fresh 1100 ms stabilization.
   Start requests stay at least 2000 ms apart.
7. **Removal:** there is no removal sensor. Removal is **inferred** from
   three consecutive failed scheduled reads. The result is then cleared,
   the probes return to high impedance, *Insert Component...* is shown and
   discovery restarts after 5 s. "Immediate" means right after the software
   detects the error, not at the instant of extraction.
8. **Precheck fault:** latches at once, without waiting for three failures.

The splash is drawn once at boot. When it expires the screen follows the
probe status, so the splash can never overwrite a result or a fault.
Serial prints one line per status change, e.g.
`probe: detected [A:VCC | B:DAT | C:GND] T=25.0C H=60%`, plus the heartbeat
`heartbeat uptime=12s display=ok probe=detected`.

### Timing and latency

| Event | Approximate time |
|---|---|
| One failed permutation (precheck + 1100 ms + start + 300 ms off) | 1.46 s |
| Full empty-socket scan | 8.4 s, then 5 s idle |
| First valid frame at permutation *k* | 1.16 s + (*k* − 1) × 1.46 s |
| Confirmed result | the above + 2.1 s (≈ 3.3 s for #1, ≈ 10.5 s for #6) |
| Removal inferred after extraction | ≈ 4.3–6.5 s |

The per-permutation 1100 ms stabilization dominates scan time.

**Blocking:** all millisecond waits are cooperative. The only busy wait is
the DHT capture. It masks interrupts on the running core (a FreeRTOS
critical section that restores the previous state on every exit) for at
most 6.5 ms, typically about 4–5 ms per frame. The 20 ms start pulse,
stabilization, retries, Serial output and display updates never run inside
it.

## Software requirements

- [Arduino IDE](https://www.arduino.cc/en/software)
- **esp32** board package by Espressif Systems (core 3.x)
- **Adafruit SSD1306** library
- **Adafruit GFX Library**
- Accept the required dependencies when prompted, including
  **Adafruit BusIO**.

No DHT or OneWire library is used.

## Build and upload (Arduino IDE)

1. **Tools → Board → Boards Manager**, search `esp32`, install
   *esp32 by Espressif Systems*.
2. **Tools → Manage Libraries**, install *Adafruit SSD1306* and
   *Adafruit GFX Library*. Click **Install All** when asked about
   dependencies.
3. Keep all sketch files together in a folder named `PinDex/`. Arduino
   requires the folder name to match `PinDex.ino`.
4. Open `PinDex.ino`.
5. **Tools → Board → esp32**, select your board. Use **ESP32 Dev Module**
   for a generic ESP32-WROOM-32 DevKit.
6. **Tools → Port**, select the port of the connected board.
7. Click **Verify**, then **Upload**.
8. Open **Tools → Serial Monitor** at **115200** baud.

To enable powered probing, only once the prerequisites above are met, set
`#define PINDEX_POWERED_PROBING_ENABLED 1` in `config.h` and upload again.

## Host tests

`tests/test_pindex.cpp` exercises the decoder and the probe state machine
against `tests/mock/Arduino.h`, a simulated GPIO, clock and DHT11. Faults
and shorts are simulated only. The tests need a host C++17 compiler (not
the ESP32 toolchain). From the `PinDex` folder:

```text
g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=1 tests/test_pindex.cpp probe_engine.cpp dht11.cpp -o build/test_enabled
g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=0 tests/test_pindex.cpp probe_engine.cpp dht11.cpp -o build/test_disabled
```

Run both binaries; each exits non-zero on any failure.

## Controlled hardware validation

Do these in order, at a bench, with the module's datasheet at hand. Stop at
the first unexpected result.

1. **Baseline:** keep the default (disabled) build and an empty socket.
   Confirm *Probing disabled*, the heartbeat, and that the probe nodes are
   high impedance.
2. **Interface:** build the protected interface described above and have
   it reviewed. Confirm the chosen DHT11 module is specified for about 3.3V.
3. **Empty socket, powered build:** enable the setting, connect the probes
   only through the interface and leave the socket empty. Expect precheck
   `no-obvious-coupling`, *Scanning 1/6 … 6/6*, *No Component / Unknown*,
   and a rescan after 5 s.

   With a scope or logic analyser on the interface outputs, verify:
   - GND comes up before VCC, with no opposite-level glitch as either
     turns on;
   - DATA is released first at teardown;
   - the off-time between permutations is at least 300 ms;
   - the start pulse is 20 ms;
   - nothing is driven while idle or in fault.
4. **Known-good module, permutation 1:** insert the module with A = VCC,
   B = DATA, C = GND. Expect a confirmed result in about 3.3 s showing
   `A:VCC | B:DAT | C:GND`, with readings close to a reference
   thermometer/hygrometer.
5. **Other orientations:** test these only if the interface is designed to
   tolerate reverse polarity and DATA-as-supply for that module.
6. **Removal:** pull the module while it is being monitored. Expect
   *STALE - fail 1/3*, then *2/3*, then *Insert Component...* within about
   6.5 s, then a rescan.

**Never test shorts, reversed devices, powered circuits or charged
capacitors on bare GPIOs.** Fault handling is covered by the simulated host
tests instead.

## Troubleshooting

- **Upload fails / no port:** check the USB cable carries data, select the
  correct port, and confirm the board is set to an original ESP32 (e.g.
  ESP32 Dev Module). Some boards need the **BOOT** button held while
  uploading begins.
- **Blank OLED but heartbeat says `display=ok`:** re-check every wire
  against the table, especially CLK/MOSI swapped or DC/CS swapped, and
  confirm VCC is on 3.3V.
- **SPI vs I2C:** this firmware needs the **7-pin SPI** module
  (GND, VCC, D0, D1, RES, DC, CS). 4-pin I2C modules (GND, VCC, SCL, SDA)
  will not work with this wiring.
- **Garbled Serial output:** set the Serial Monitor to 115200 baud.
- **Probing disabled:** expected with the default build; see *Powered
  probing: prerequisites*.
- **Socket rejected:** the precheck saw coupling or unstable readings.
  Remove the part, check the interface, then send `r`. A module's onboard
  pull-up can cause this (see the precheck limits).
- **No Component / Unknown with a DHT11 inserted:** check the module
  supports about 3.3V and has a DATA pull-up. The Serial `err=` field shows
  why each attempt failed (e.g. `no-response`, `checksum`,
  `unsupported-encoding` for a DHT22).

## Scope and safety limits

- In the default build every probe pin is a plain input with internal pull
  resistors disabled (high impedance). This only means the firmware does
  not drive the probes. It is **not** protection against short circuits,
  overvoltage, ESD or charged capacitors.
- Powered probing drives probe GPIOs and must only be enabled behind the
  protected interface. The weak-pull precheck does not make the socket
  safe to power.
- Do not connect powered circuits or charged capacitors to the probes, and
  never apply 5V to them.
- "Ready" means the firmware is running. Detection is limited to
  DHT11-compatible sensors and requires powered probing.
