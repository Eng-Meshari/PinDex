# PinDex

**Smart Component & Sensor Identifier**, an ESP32 instrument for electronics
labs. The goal is to let students insert a component or sensor module, have
it identified, and get its pinout.

> **Status: v0.2 + Phase 3.2.** Two drivers sit behind a generic probe
> engine:
> - DHT11-compatible detection with A/B/C pinout and live telemetry
>   (powered);
> - weak-pull passive measurement of resistive parts, contacts and
>   switch-like behaviour, plus an optional user-guided LDR mode.
>
> Two settings in `config.h` control probing:
> - **`PINDEX_POWERED_PROBING_ENABLED`** drives probe GPIOs as VCC/GND, so
>   use it only behind the protected interface described below.
> - **`PINDEX_PASSIVE_PROBING_ENABLED`** uses an internal pull-up and an
>   open-drain LOW sink only, and is for unpowered parts.
>
> With both set to `0` the probes stay high impedance and the display shows
> *Probing disabled*. **Check both values before every upload.** Nothing
> here identifies arbitrary components, and inserting arbitrary components
> is not safe.

## Project files

| File | Purpose |
|---|---|
| `PinDex.ino` | Sketch entry point: startup order, screen updates, Serial heartbeat and Serial commands (`r`, `c`, driver commands). |
| `config.h` | All pins, timings, thresholds, failure limits and both probing settings. |
| `display_manager.h/.cpp` | VSPI + SSD1306 setup; renders the engine's status snapshot (splash, idle, status, result screens). |
| `probe_engine.h/.cpp` | Generic orchestrator: owns the probe GPIOs, precheck routing, power sequencing, capability-based attempt scheduling, confirmation, monitoring and removal. Knows no concrete sensor. |
| `sensor_driver.h` | The abstract `SensorDriver` contract: operation status, `DriverRequirements` (incl. topology and excitation capabilities) and `ProbeContext`. |
| `src/drivers/dht11_driver.h/.cpp` | `Dht11Driver`: DHT start pulse, bounded capture, validation and telemetry formatting. |
| `src/drivers/passive_driver.h/.cpp` | `PassiveDriver`: weak-pull two-terminal measurement, classification, switch debouncing, guided LDR mode. |
| `src/drivers/passive_adc.h/.cpp` | The only ADC code: ESP-IDF oneshot ADC with pull-up control and error checking. |
| `src/drivers/driver_registry.h/.cpp` | The static driver table (priority order) and its read-only API. |
| `dht11.h/.cpp` | Pure DHT11 frame decoding (timing → bits → bytes → values); no GPIO, host-testable. |
| `tests/` | Host-side tests with a mock Arduino layer, fake drivers, a resistor-network simulator, a simulated ADC and a simulated DHT11. Not compiled by the Arduino IDE. |
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

Within each permutation, the registered powered drivers are tried in
registry order (permutation-major). Every driver attempt, even on an
unchanged permutation, gets its own precheck and power cycle. A permutation
is **skipped** if the precheck saw coupling on any path from its VCC node
(see the next section).

The engine powers up in this order: GND driven LOW, VCC driven HIGH, DATA
as `INPUT_PULLUP`. On the ESP32 core 3.x, `digitalWrite()` before
`pinMode(OUTPUT)` sets the output latch first, so a pin never briefly
drives the opposite level. This preparation runs only for drivers whose
capabilities say `ThreeWirePowered + ProtectedActive`; it never runs for a
passive driver.

After a failed attempt the engine tears down in this order: DATA
pull-up/drive removed, VCC released, then all nodes set to plain `INPUT`.
Only then does it reset the driver. It then waits `PROBE_POWER_OFF_MS`
(300 ms). That timed wait does not prove a capacitor has discharged, and
the firmware never shorts nodes to discharge them.

## Precheck routing (diagnostic heuristic only)

Before every attempt, each node in turn is set to `INPUT_PULLUP` while the
other two are `INPUT_PULLDOWN`. After a short settle time, all three nodes
are sampled four times, then everything returns to plain `INPUT`.

The precheck **observes**; it never grants safety. A closed button, a wire
and a few-kΩ resistor can all look the same to it, and a finite resistance
does not make powered probing safe either. So the result routes attempts
instead of latching a fault:

| Observation | Weak-pull passive measurement | Powered (VCC/GND) probing |
|---|---|---|
| No pulled-down node read HIGH (`NoObviousCoupling`) | Runs | Runs (as before, behind the protected interface) |
| A pulled-down node read HIGH: that pair is coupled (`CouplingObserved`) | Runs; the ADC tells contact from resistance | **Blocked across that path**: a permutation is skipped if its VCC node is coupled to either other node |
| Readings changed between samples (`InconsistentReadings`) | Retried up to `PRECHECK_MAX_RETRIES` times, then **unknown**: nothing runs this cycle | Same: no safety is inferred |
| ADC or configuration failure (passive driver) | Measurement stops, pins reset, the attempt fails | No fallback permission |

DATA–GND coupling is tolerated for powered attempts. GND only sinks and
DATA is only pulled up or sunk, so no opposing strong drives meet across
it. A coupling observed during a retry of a saved powered mapping makes that
retry fail (`blocked-by-precheck`), so the mapping is not powered again.

Limits: the ESP32's internal pull-up and pull-down are both weak (tens of
kΩ) and roughly equal. Even a dead short between two nodes forms a
mid-rail divider that may read either way, and a short across all three
nodes reads LOW. LOW readings never prove there is no short. A module's
DATA pull-up resistor or a chip's ESD diodes can also look like coupling,
which blocks the matching powered permutations. The external protection
remains necessary either way.

The latched *Socket rejected* fault and the `r` retry still exist, but only
for internal faults. Coupling no longer latches it.

## Supported DHT11 profile

Implemented by `Dht11Driver` (`src/drivers/dht11_driver.cpp`). DHT sensors
use their own single-wire protocol; it is not Dallas/Maxim 1-Wire. PinDex
captures it in custom code with no DHT library.

- **Driver requirements:** 1100 ms power stabilization, at least 2000 ms
  between acquisitions (including retries), a 2100 ms sample interval, and
  a 500 ms operation timeout.

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

## Passive components and switches

Implemented by `PassiveDriver` (`src/drivers/passive_driver.cpp`). Its
capabilities are `TwoTerminal + WeakPullOnly`. The engine never powers
anything for it; it measures the three pairs **A–B, B–C, A–C** once per
scan cycle, after the powered attempts.

**Only connect unpowered parts.** The weak pull-up limits the current the
firmware sources (about 70 µA into a short), but the open-drain LOW sink
is an ordinary GPIO. Nothing in software protects it from an external
voltage source, a charged capacitor or ESD.

### How a pair is measured

For each orientation of the pair:

1. Both terminals are set to plain `INPUT`. The third node is never
   touched and stays plain `INPUT`.
2. The sense terminal is routed to the ADC and its internal pull-up is
   enabled (about 45 kΩ nominal).
3. **Pull-up check:** with the other terminal still released, the sense
   node must read ≥ 2450 mV. Otherwise the result is *Measurement
   unavailable*, the pins are reset, and no value is reported.
4. The other terminal becomes a LOW sink. Its latch is set LOW first, then
   it switches to `OUTPUT_OPEN_DRAIN`, which cannot drive HIGH.
5. After settling, four samples (about 2 ms apart, one per loop pass) give
   the divider voltage.
6. Both terminals return to plain `INPUT`. The orientation is reversed and
   the measurement repeated.

A pair is accepted only if:
- both windows are stable (spread ≤ 80 mV);
- the two orientations agree within max(60 mV, 15 %);
- the voltage is below the open threshold.

Asymmetric results (a diode junction, an unpowered module, an odd network)
and noisy results are not interpreted. All three pairs are measured before
anything is published:
- exactly one conducting pair → candidate;
- several conducting pairs (e.g. three-node coupling, a potentiometer) →
  *No Component / Unknown* with `err=ambiguous-pairs`;
- none → no component.

A candidate is then re-measured from a fresh precheck and confirmed with a
new sample.

### ESP32 ADC details (checked, but not on hardware)

- GPIO 25/26/27 are **ADC2** channels on the original ESP32. ADC2 conflicts
  (e.g. with Wi-Fi, which PinDex does not use) surface as measurement
  errors.
- The installed ESP-IDF channel setup turns the pull-up **off**:
  `adc_oneshot_config_channel()` → `gpio_config_as_analog()` →
  `gpio_pullup_dis()`. `pinMode(INPUT_PULLUP)` followed by `analogRead()`
  therefore cannot keep a divider.
- The adapter (`passive_adc.cpp`):
  - configures the channel first, then calls `gpio_pullup_en()`, which on
    this pad routes to `rtc_gpio_pullup_en()`;
  - uses ESP-IDF `adc_oneshot_read()` and checks every error code;
  - restores the pin with pulls off and `pinMode(INPUT)`.
- Arduino `analogRead()` is not used. It discards read errors, so a failed
  conversion would look like a valid 0 mV, and using it here would create
  two owners of one ADC unit.
- No ADC work runs inside the DHT critical section, and no undocumented
  registers or core files are touched.
- Millivolts come from ESP-IDF line-fitting calibration. Without eFuse
  data it uses `default_vref` 1100 mV, or a nominal 3100 mV full scale if
  no scheme is available.
- **Still unverified:** whether the pull-up is electrically effective while
  the pad is sampled. Step 3 above checks this on every measurement and
  refuses to report a value if it fails.

### Resistance and signal estimates

- **Formula:** `R~ ≈ 45 kΩ × V / (3300 mV − V)`. This assumes the LOW sink
  is at 0 V, and uses the nominal pull-up and rail values. Real internal
  pull-ups vary widely between parts and with temperature, so every value
  is labelled `R~` and is an approximation, not a measurement.
- **Useful window:** about 150–2450 mV, which is roughly 2.1 kΩ–130 kΩ.
  Outside it the display shows a bound, not a value:
  - `R~ < 2.1 kOhm` (contact / low R);
  - `R~ > 130 kOhm` (open-like or very high R).

  Lower resistances (e.g. 1 kΩ) therefore read as *Contact / Low R*.
- **`Signal: XX%`** is a relative conduction index,
  `(3300 − V) / 3300`: 0 % is open, 100 % is a contact. It is not a
  physical unit.

### Classification (conservative)

| Evidence | Shown as |
|---|---|
| Stable, symmetric, finite R | **Resistive Component**, `R~ …`, `Signal: …%` |
| That R then changes by ≥ 150 mV (filtered) for 5 samples | **Variable Resistance**, `Sig …% Possible LDR`. It could equally be a potentiometer, thermistor or moving contact. |
| User-guided calibration accepted (below) | **Photoresistor (LDR)**, `Light: XX%`, `R~ …` |
| Stable near-zero connection | **Contact / Low R**, `State: CLOSED` |
| A contact that then shows a debounced OPEN/CLOSED transition | **Push Button / Switch**, `State: OPEN` / `State: CLOSED` |

- A constant wire never becomes a button.
- Inserting or pulling a wire also produces a transition, so "switch" means
  switch-like behaviour was observed, not that the part is a button.
- An initially open button looks like an empty socket and is not
  identified.
- The 5528 (or any other) LDR model is never claimed.

**Mapping line:** `Pins: A & B, C:---` (the two terminals, then the unused
node). Example screens, with illustrative values only:

```text
Resistive Component      Push Button / Switch     Photoresistor (LDR)
R~ 12.4 kOhm             State: OPEN              Light: 68%
Signal: 42%              Pins: A & C, B:---       R~ 8.2 kOhm
Pins: A & B, C:---       LIVE                     Pins: B & C, A:---
LIVE                                              LIVE
```

### Switches, open readings and removal

- **Switches and contacts:**
  - OPEN is valid telemetry, never a failure, and never counts toward
    removal.
  - The debounce needs 2 consecutive samples, and only clearly closed
    (≤ 150 mV) or clearly open (≥ 2450 mV) readings count (hysteresis).
  - An open switch cannot be told apart from a removed one using these
    nodes alone. Send **`c`** to clear the result and rescan.
- **Resistive parts:** an open-like reading may be a very high resistance
  or a removal.
  - It is shown as `R~ > 130 kOhm` / `Open or removed?` for up to 50
    samples (about 5 s).
  - After that it counts as a failed acquisition, and the normal retry and
    three-failure removal policy applies.
- **ADC failures and invalid configuration** are real errors. They tear
  down immediately and use the same retry policy. If a whole cycle fails
  this way, the screen shows *Measurement unavailable*.

### Guided LDR mode (Serial commands)

Send these in the Serial Monitor while a resistive part is detected:

1. **`l`**: start (or cancel). The screen shows *LDR cal: cover it /
   then send 'd'*.
2. Cover the part, then send **`d`**. Ten samples (about 1 s) form the
   dark reference.
3. Light it, then send **`b`**. Ten samples form the bright reference.

The calibration is accepted only if both reference windows are steady
(spread ≤ 120 mV) and the bright reading is at least 300 mV below the dark
one.
- **Accepted:** the part is then shown as *Photoresistor (LDR)* with
  `Light: XX%`. That is a relative index between your two references,
  clamped to 0–100 %. It is **not lux** and not an absolute ambient light
  level.
- **Too little change:** shows `Cal failed: no change`, and no percentage
  is invented.
- **Unsteady window:** asks you to redo that step.

Clearing (`c`), removal, or any re-probe of the pair discards the
calibration.

## Behaviour

**With both probing settings `0`:** splash, then
*v0.2 Ready / Probing disabled*. The probes stay plain `INPUT`, no driver
ever runs, and the Serial heartbeat continues once per second.

**Scan cycle:** one cycle runs these attempts in order, each preceded by
the precheck:
1. every powered (three-wire) attempt, permutation-major, for drivers in
   registry order;
2. the three passive pairs, once.

Powered attempts are skipped when powered probing is off or the precheck
blocks that permutation; passive attempts are skipped when passive probing
is off. The screen shows *Scanning n/m* (m = 9 with both drivers). A
powered match ends the cycle before any passive attempt.

**With powered probing enabled**, the DHT11 driver behaves as follows:

1. **Scan:** the precheck runs, then permutation 1 is powered. The engine
   waits 1100 ms, then `probe()` sends the start pulse and captures the
   reply. A failed attempt is torn down before the next one.
2. **No result:** if every attempt fails, the screen shows
   *No Component / Unknown* (or *Measurement unavailable*), the probes are
   reset and the scan repeats after 5 s.
3. **Candidate:** on the first valid frame (a successful `probe()`), the
   driver and mapping are saved and stay powered, and scanning stops. The
   screen shows the driver name and *Confirming...*.
4. **Confirmation:** a new acquisition (`readSample()`) is requested
   2100 ms later. Two consecutive valid acquisitions publish the result.
   The readings do not have to match.
5. **Live display:** the result screen is redrawn only when something
   shown changes:

   ```text
   DHT11 Temp & Hum
   T: 25*C  |  H: 60%
   A:VCC | B:DAT | C:GND
   LIVE
   ```

   Line 1 is the driver's `getName()`, line 2 its telemetry, and line 3
   the engine's mapping. The mapping line always lists A, B, C in that
   order. Readings repeat every 2100 ms.
6. **Failures:** any capture or validation failure immediately tears down
   power and resets the driver. The reading is marked stale (inverted
   *STALE - fail n/3* bar) and the failure counter increments. Only the
   saved driver and mapping are retried, after a 1000 ms cooldown, a full
   precheck and a fresh 1100 ms stabilization. Acquisitions stay at least
   2000 ms apart. A retry re-runs `probe()`: success clears the failure
   counter, but the telemetry stays marked *STALE* until the next
   `readSample()` publishes a new value, about 2.1 s later.
7. **Removal:** there is no removal sensor. Removal is **inferred** from
   three consecutive failed scheduled reads. The result is then cleared,
   the probes return to high impedance, *Insert Component...* is shown and
   discovery restarts after 5 s. "Immediate" means right after the software
   detects the error, not at the instant of extraction.
8. **Precheck during a retry:** if it now shows coupling on the saved
   mapping's VCC path, that retry fails without powering anything.

**Serial commands:** `r` clears a latched internal fault; `c` clears the
current result and rescans. Any other letter goes to the detected driver
(the passive driver uses `l`, `d` and `b`).

The splash is drawn once at boot. When it expires the screen follows the
probe status, so the splash can never overwrite a result or a fault.
Serial prints the registered drivers at boot and one line per status
change, e.g.
`probe: detected DHT11 Temp & Hum [A:VCC | B:DAT | C:GND] "T: 25*C  |  H: 60%"`,
plus the heartbeat `heartbeat uptime=12s display=ok probe=detected`.

### Timing and latency

| Event | Approximate time |
|---|---|
| One failed permutation (precheck + 1100 ms + start + 300 ms off) | 1.46 s |
| Full empty-socket scan | 8.4 s, then 5 s idle |
| First valid frame at permutation *k* | 1.16 s + (*k* − 1) × 1.46 s |
| Confirmed result | the above + 2.1 s (≈ 3.3 s for #1, ≈ 10.5 s for #6) |
| Removal inferred after extraction | ≈ 4.3–6.5 s |

The per-permutation 1100 ms stabilization dominates scan time. Each extra
registered driver adds its own attempt (precheck, stabilization, operation,
power-off) to every permutation.

The passive pairs add about 1.1 s per cycle: three pairs of about 0.37 s
each (precheck, two measurement windows, 300 ms off). A passive result is
confirmed about 0.5 s after its cycle ends. Passive monitoring samples
every 100 ms, so switch transitions show within about 0.2–0.3 s.

**Blocking:** all millisecond waits are cooperative. The only busy wait is
the DHT capture inside `Dht11Driver`. It masks interrupts on the running core (a FreeRTOS
critical section that restores the previous state on every exit) for at
most 6.5 ms, typically about 4–5 ms per frame. The 20 ms start pulse,
stabilization, retries, Serial output and display updates never run inside
it.

## Sensor driver architecture

The probe engine is a generic orchestrator. Each sensor protocol is a
`SensorDriver` (`sensor_driver.h`) listed in a fixed registry. The engine
contains no sensor names, decoding, type checks or sensor-specific
branches. The display renders only the engine's `ProbeStatus` snapshot, so
redrawing never triggers an acquisition.

### Ownership

| Engine (`probe_engine.cpp`) | Driver (`src/drivers/…`) |
|---|---|
| Probing permissions and the weak-pull precheck routing | Signalling on the nodes handed to it (DHT start pulse; passive pull-up + open-drain sink) |
| Which permutations/pairs are attempted, in which order, by which driver | Handshake, bounded capture or ADC windows, decoding and validation |
| Preparation by capability: VCC/GND power sequencing only for `ProtectedActive`; nothing for `WeakPullOnly` | Its own sample storage, classification and telemetry formatting |
| Stabilization scheduling, teardown to high impedance | Its own critical sections (the DHT capture only) |
| Confirmation counting, retry limit, removal inference | Optional user commands (e.g. guided LDR) |
| Candidate/active driver and mapping; the result snapshot | — |

A driver never powers VCC/GND, picks other nodes, draws on the OLED or
changes engine state. On every terminal failure the engine first makes the
pins safe, then calls the driver's `reset()`.

### Lifecycle and contract

- `probe(const ProbeContext &)` and `readSample(line1, line2, maxLen)` are
  **cooperative**. A `ProbeContext` holds named GPIOs for one topology
  only: `powered.{vcc,data,gnd}` or `pair.{first,second,unused}`. A passive
  terminal is never passed as "VCC". A driver given the wrong topology
  fails without touching any pin.
  - The first call starts an operation and later calls advance it.
    Arguments must not change while it is pending.
  - `true` means a newly completed success. On `false`, check
    `getOperationStatus()`: `Pending` is not a failure.
  - Once an operation has completed, the next call starts a new
    acquisition. A completed sample is never reported twice.
  - The engine never overlaps the two operations; a driver fails an
    overlapping call.
- `probe()` succeeds only after a genuine acquisition. `readSample()`
  needs a prior successful `probe()` and performs a new acquisition every
  time. Confirmation counts the probe plus new samples
  (`PROBE_CONFIRM_FRAMES` = 2), never cached text.
- **Buffers:**
  - `maxLen` is the capacity of each buffer, including the terminator.
  - Null pointers or zero capacity are rejected without being touched.
  - Valid buffers are always terminated, and filled only on a completed
    success.
  - Drivers never keep the caller's pointers.
- `reset()` is idempotent and cancels any pending operation. It may only
  make a pin safer (e.g. release a held line to plain `INPUT`); it never
  energises one.
- `getRequirements()` returns fixed `DriverRequirements`:
  - **topology**: `ThreeWirePowered` (six permutations) or `TwoTerminal`
    (three pairs);
  - **excitation**: `ProtectedActive` or `WeakPullOnly`. Only the two
    matching combinations are supported; any other combination is never
    attempted.
  - power stabilization (0 for passive);
  - minimum acquisition interval;
  - sample interval (raised to the minimum if it is lower);
  - operation timeout, after which the engine fails an operation that is
    still `Pending`.
- **Attempt order:** within a cycle, `ThreeWirePowered` attempts come
  before `TwoTerminal` ones. Registry order applies within a topology.
  Two-terminal successes are judged over all pairs before one becomes the
  candidate.
- `getName()` returns a static string used as the result title. It may
  change as a driver's classification evolves, and the engine re-reads it
  on every sample.
- **Optional hooks:**
  - `getFailureReason()` feeds the Serial `err=` field;
  - `failureIsMeasurementError()` marks ADC-type failures (shown as
    *Measurement unavailable*);
  - `handleUserCommand(char)` receives Serial commands while the driver is
    detected.

### Adding a driver

1. Implement `class MyDriver final : public SensorDriver` in
   `src/drivers/my_driver.h/.cpp`. Arduino compiles `src/` recursively.
   Use relative includes such as `"../../sensor_driver.h"`.
2. Declare its topology and excitation in `getRequirements()`. They decide
   how the engine prepares and schedules it.
3. In `src/drivers/driver_registry.cpp`, include its header, add
   `MyDriver myDriver;` beside the others, and add `&myDriver` to
   `registeredDrivers[]`. Earlier entries are tried first within the same
   topology. Drivers are static, never allocated.

No engine or display change is needed. Add tests with the mock in
`tests/` before trying the driver on hardware.

### Topology limits

Two topologies exist on the three-node socket:
- **`ThreeWirePowered`:** VCC/DATA/GND over six permutations, powered
  GND-first from GPIOs. Suits three-pin, single-data-line sensors that
  tolerate that policy.
- **`TwoTerminal`:** a terminal pair plus an unused node, with weak-pull
  excitation only. Suits unpowered resistive parts, contacts and switches;
  it is never powered with the DHT policy.

Not supported:
- **I2C parts:** these commonly need separate SDA and SCL lines plus
  power, which this socket cannot provide.
- **Parts needing other excitation**, e.g. a current source, AC, or
  polarity-aware diode tests.

Such families need a new topology or excitation value and a matching
engine policy, not branches inside the state machine. No such driver is
implemented.

### Phase 3.2 changes to existing behaviour

- Precheck coupling no longer latches *Socket rejected*. It blocks powered
  drive across the coupled path and routes the socket to passive
  measurement.
- Inconsistent precheck readings are retried twice, then the cycle ends as
  unknown. They no longer latch a fault.
- `PINDEX_POWERED_PROBING_ENABLED 0` alone no longer keeps the probes
  untouched, because passive probing has its own setting. Set both to `0`
  for the old fully-disabled behaviour.
- *Scanning n/m* counts all attempts in a cycle (6 powered + 3 pairs).

### What the Phase 3.1 refactor preserved

Unchanged by Phase 3.1:
- the powered-probing setting and the weak-pull precheck (changed in 3.2,
  see above);
- GND-first power-up and DATA-first teardown;
- the 300 ms off-time between every attempt;
- the DHT timing: 1100 ms stabilization, 20 ms start pulse, 120 µs and
  6500 µs capture limits, 2100 ms reads, 2000 ms minimum spacing;
- two-frame confirmation, three-failure removal and the 5 s rescan;
- the FreeRTOS critical section, which restores interrupt state on every
  exit.

Visible differences:
- the confirming screen shows the driver name;
- Serial prints the driver's telemetry text instead of tenth-degree values;
- after a recovered failure, telemetry stays *STALE* until the next sample.

## Software requirements

- [Arduino IDE](https://www.arduino.cc/en/software)
- **esp32** board package by Espressif Systems (core 3.x)
- **Adafruit SSD1306** library
- **Adafruit GFX Library**
- Accept the required dependencies when prompted, including
  **Adafruit BusIO**.

No DHT or OneWire library is used.

### Software versions this was checked against

This code was compiled (not run on hardware) with:
- Arduino-ESP32 core **3.3.12**, bundling ESP-IDF **5.5.5**;
- the core's `esp-x32` 2601 toolchain (xtensa GCC 14.2.0);
- `arduino-cli` 1.5.1 (bundled with Arduino IDE 2);
- Adafruit SSD1306 2.5.17, Adafruit GFX 1.12.6 and Adafruit BusIO 1.17.4;
- FQBN `esp32:esp32:esp32` (original ESP32).

The ADC and pull-up findings above come from that core's sources and
binaries. Other 3.x releases may differ, so recheck after upgrading.

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
Passive probing is controlled separately by
`#define PINDEX_PASSIVE_PROBING_ENABLED` (1 = on, unpowered parts only).

## Host tests

`tests/test_pindex.cpp` exercises the probe engine and both driver
contracts against these simulations:
- `tests/mock/Arduino.h`: simulated GPIO and clock;
- a resistor-network solver, which drives both the precheck's digital reads
  and the simulated ADC;
- a simulated DHT11;
- fake drivers that exercise the architecture.

The test file itself defines the registry functions and the passive ADC
adapter, so it is linked **without** `src/drivers/driver_registry.cpp` and
`src/drivers/passive_adc.cpp`. Shorts, faults and parts are simulated only.
The tests need a host C++17 compiler (not the ESP32 toolchain). From the
`PinDex` folder:

```text
SRC="tests/test_pindex.cpp probe_engine.cpp dht11.cpp src/drivers/dht11_driver.cpp src/drivers/passive_driver.cpp"
g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=1 -DPINDEX_PASSIVE_PROBING_ENABLED=1 $SRC -o build/test_all
g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=0 -DPINDEX_PASSIVE_PROBING_ENABLED=1 $SRC -o build/test_passive_only
g++ -std=c++17 -Wall -Wextra -Itests/mock -I. -DPINDEX_POWERED_PROBING_ENABLED=0 -DPINDEX_PASSIVE_PROBING_ENABLED=0 $SRC -o build/test_disabled
```

Run all three binaries; each exits non-zero on any failure.

## Controlled hardware validation

Do these in order, at a bench, with the module's datasheet at hand. Stop at
the first unexpected result.

1. **Baseline:** set both `PINDEX_POWERED_PROBING_ENABLED` and
   `PINDEX_PASSIVE_PROBING_ENABLED` to `0` and use an empty socket.
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

### Passive driver bench steps

Use passive probing only (`PINDEX_POWERED_PROBING_ENABLED 0`,
`PINDEX_PASSIVE_PROBING_ENABLED 1`) and unpowered parts only.

7. **Pull-up check:** with an empty socket, expect *No Component /
   Unknown* (`err=open-circuit`), **not** *Measurement unavailable*.
   *Measurement unavailable* with `err=pullup-unverified` means the pull-up
   is not effective during ADC sampling on your board. In that case passive
   measurement is unavailable; do not add hardware to force it.
8. **Known resistors:** insert 4.7 kΩ, 10 kΩ, 47 kΩ and 100 kΩ (±1 %)
   across A–B in turn.
   - Note each `R~` value next to a multimeter reading. The nominal 45 kΩ
     pull-up means an error of tens of percent is expected.
     `PASSIVE_PULLUP_NOMINAL_OHMS` can be tuned to your board, but values
     stay approximate.
   - Repeat on B–C and A–C, and check the `Pins:` line.
   - A 1 kΩ part should read *Contact / Low R*, and 1 MΩ should read as no
     component.
   - Each part should stay *Resistive Component* for minutes, never
     becoming variable or LDR.
9. **Wire:** a short wire across A–B should read *Contact / Low R*,
   `State: CLOSED`, and stay that way. It should not become a button.
10. **Push button:** insert a normally-open button and hold it closed
    during the scan.
    - Expect *Contact / Low R*. On release, after about 0.2–0.3 s, expect
      *Push Button / Switch* with `State: OPEN`.
    - Press and release a few times and check that each change is shown
      once and that bounce does not flicker the state.
    - Leave it released for over a minute: it must stay detected. Then send
      `c` and confirm the open button is no longer identified.
11. **LDR:** insert an LDR and expect *Resistive Component* (or *Variable
    Resistance* once the light changes).
    - Send `l`, cover the LDR, send `d`, wait about 1 s.
    - Light it, send `b`, wait about 1 s.
    - Expect *Photoresistor (LDR)* with `Light: XX%` tracking your hand.
      Repeat with too little change to see `Cal failed: no change`.
12. **Handoff:** after the passive tests, re-enable powered probing behind
    the interface and repeat step 4. The DHT11 must still be detected,
    which shows the pins return to usable digital inputs.
13. **ADC2:** do not enable Wi-Fi in this firmware. If a future build does,
    expect passive *Measurement unavailable* during Wi-Fi activity.

**Never test shorts, reversed devices, powered circuits or charged
capacitors on bare GPIOs.** Fault handling is covered by the simulated host
tests instead. The passive LOW sink is not protected against external
sources.

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
- **Probing disabled:** expected when both probing settings are `0`.
- **No Component / Unknown with a DHT11 inserted:**
  - check the module supports about 3.3V and has a DATA pull-up;
  - the Serial `err=` field shows why each attempt failed (e.g.
    `no-response`, `checksum`, `unsupported-encoding` for a DHT22);
  - `coupled=` in the Serial log means the precheck blocked some
    permutations; a module's onboard pull-up can cause this.
- **`err=precheck-inconsistent`:** the precheck readings kept changing, so
  nothing was attempted. Check for loose contacts.
- **`err=ambiguous-pairs`:** more than one pair conducts (a three-terminal
  part or a network). This is not supported as a passive result.
- **Measurement unavailable:** the passive ADC path failed (`err=` shows
  `adc-unavailable`, `adc-config-failed`, `adc-read-failed` or
  `pullup-unverified`).
- **A switch stays "OPEN" after you removed it:** expected. An open switch
  and an empty socket look the same, so send `c` to clear.

## Scope and safety limits

- With both probing settings `0`, every probe pin is a plain input with
  internal pull resistors disabled (high impedance). This only means the
  firmware does not drive the probes. It is **not** protection against
  short circuits, overvoltage, ESD or charged capacitors.
- Powered probing drives probe GPIOs and must only be enabled behind the
  protected interface. The weak-pull precheck does not make the socket
  safe to power.
- Passive probing never drives a probe HIGH, but its open-drain LOW sink
  is an unprotected GPIO. Use it only with unpowered parts.
- Do not connect powered circuits or charged capacitors to the probes, and
  never apply 5V to them.
- "Ready" means the firmware is running. Detection is limited to:
  - DHT11-compatible sensors (powered probing);
  - approximate two-terminal resistive and contact behaviour (passive
    probing).

  Neither proves what a part is.
