# PinDex

**Smart Component & Sensor Identifier**, an ESP32 instrument for electronics
labs. The goal is to let students insert a component or sensor module, have
it identified, and get its pinout.

> **Status: v0.2 + Phase 4.** A four-node socket (A, B, C, D) and three
> drivers sit behind a generic probe engine:
> - DHT11-compatible detection with pinout and live telemetry (three-wire,
>   powered from GPIOs);
> - HC-SR04-compatible ultrasonic distance (four-wire). Fully implemented
>   and tested in simulation, but **not attempted on this board**: it needs
>   a protected four-wire power/routing interface that does not exist yet.
>   The firmware reports it as *hardware required* (see
>   [HC-SR04 ultrasonic](#hc-sr04-ultrasonic-four-wire));
> - weak-pull passive measurement of resistive parts, contacts and
>   switch-like behaviour, plus an optional user-guided LDR mode.
>
> Two settings in `config.h` control probing:
> - **`PINDEX_POWERED_PROBING_ENABLED`** drives probe GPIOs as VCC/GND, so
>   use it only behind the protected interface described below. It never
>   enables four-wire probing on its own.
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
| `probe_engine.h/.cpp` | Generic orchestrator: owns the node table and probe GPIOs, the mapping tables, precheck routing, power sequencing (GPIO or four-wire backend), capability-based attempt scheduling, confirmation, monitoring and removal. Knows no concrete sensor. |
| `four_wire_backend.h/.cpp` | The protected four-wire power/routing interface seam. The shipped implementation reports **no capabilities** and refuses every connection, because no such hardware exists yet. |
| `sensor_driver.h` | The abstract `SensorDriver` contract: operation status, `DriverRequirements` (topology, excitation, backend capabilities), `ProbeContext`. |
| `src/drivers/dht11_driver.h/.cpp` | `Dht11Driver`: DHT start pulse, bounded capture, validation and telemetry formatting. |
| `src/drivers/ultrasonic_driver.h/.cpp` | `UltrasonicDriver`: HC-SR04 trigger pulse, interrupt-timestamped echo capture, validation, confirmation and telemetry. |
| `src/drivers/passive_driver.h/.cpp` | `PassiveDriver`: weak-pull two-terminal measurement, classification, switch debouncing, guided LDR mode. |
| `src/drivers/passive_adc.h/.cpp` | The only ADC code: ESP-IDF oneshot ADC (both ADC units) with pull-up control and error checking. |
| `src/drivers/driver_registry.h/.cpp` | The static driver table (priority order) and its read-only API. |
| `dht11.h/.cpp` | Pure DHT11 frame decoding (timing → bits → bytes → values); no GPIO, host-testable. |
| `tests/` | Host-side tests with a mock Arduino layer (incl. interrupts), fake drivers, a resistor-network simulator, a simulated ADC, a simulated DHT11, a fake protected four-wire backend and a simulated HC-SR04. Not compiled by the Arduino IDE. |
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
| Probe D | GPIO 33 |

Probes A, B, C and D are **generic test nodes**. They are not fixed VCC,
GND or signal terminals.

**Socket wiring:** build the external socket in the order **A, B, C, D**
and wire each position to the GPIO in the table. GPIO 25, 26, 27 and 33
are **not** in that order, or next to each other, on every DevKit header
(30- and 38-pin boards differ). Follow the GPIO labels printed on your
board, not header positions. The node table in `probe_engine.cpp`
(`PROBE_NODE_GPIO`) is the only place the firmware maps nodes to GPIOs.

GPIO 25/26/27 are ADC2 channels; GPIO 33 is ADC1 channel 5. GPIO 33 is
also the ESP32's 32 kHz crystal pin (XTAL_32K_N); a board that fits that
crystal cannot use it as a probe.

**Strapping pins:** GPIO 2 and GPIO 5 affect how the ESP32 boots. Anything
attached to them must not force incompatible levels during reset (GPIO 2
must not be held high, GPIO 5 must not be held low). The OLED inputs are
fine. On some DevKits the on-board LED is on GPIO 2 and may flicker while
the display updates; that is normal.

## Powered probing: prerequisites

Four bare GPIOs are **not** a protected component tester. Trying all 24
three-wire mappings deliberately reverse-biases the part and drives its
DATA pin as a supply in some of them. Before setting
`PINDEX_POWERED_PROBING_ENABLED` to `1`, the probes must go through a
**reviewed interface** that provides:

- suitable current limiting on every node, for every mapping;
- protection against wrong polarity and wrong pin assignment for the
  intended parts, including what an onboard module resistor does in each
  mapping;
- defined fault handling, for example shorted nodes or a charged capacitor;
- the DATA pull-up arrangement (see the DHT11 profile below).

Series resistors alone are not comprehensive protection. Enabling the
setting is a deployment step, **not** a safety mechanism. The firmware
cannot detect a missing or faulty interface.

The selected VCC and GND are ordinary GPIO outputs at nominal 3.3V logic
levels, not regulated supply rails; the voltage sags under load. Never
"fix" a 5V-only module by applying 5V to ESP32 GPIOs.

That GPIO arrangement is the **three-wire** path only. **Four-wire**
(ultrasonic) attempts never use a GPIO as a supply. They need a separate
protected four-wire backend that reports its capabilities in firmware; see
[HC-SR04 ultrasonic](#hc-sr04-ultrasonic-four-wire). No such backend is
fitted, so no four-wire attempt runs, whatever this setting says.

## Mapping scan

Three topologies are enumerated over the four nodes. Counts and order are
fixed:

| Topology | Mappings | Order |
|---|---|---|
| Three-wire (VCC, DATA, GND) | 24: every ordered choice of 3 of the 4 nodes; the fourth stays plain `INPUT` | The original six A/B/C permutations first, unchanged, then the same six patterns on ABD, ACD, BCD |
| Four-wire (VCC, SIG1, SIG2, GND) | 24: every ordering of all 4 nodes | Lexicographic by node: ABCD, ABDC, ACBD, … DCBA |
| Two-terminal pair | 6: each unordered pair; the other two stay plain `INPUT` | A–B, B–C, A–C (the original three), then A–D, B–D, C–D |

A pair's two measurement orientations (which terminal senses) happen inside
one pair attempt; the reversed pair is not another physical pair.

The six three-wire patterns, applied to a node set *x < y < z* (ABC, then
ABD, ACD, BCD):

| Pattern | VCC | DATA | GND | On ABC (entries 1–6) |
|---|---|---|---|---|
| 1 | x | y | z | A B C |
| 2 | z | y | x | C B A |
| 3 | y | x | z | B A C |
| 4 | y | z | x | B C A |
| 5 | x | z | y | A C B |
| 6 | z | x | y | C A B |

So entry 7 is VCC A, DATA B, GND D; entry 9 is VCC B, DATA A, GND D;
entry 24 is VCC D, DATA B, GND C.

Within each mapping, the registered drivers of that topology are tried in
registry order (mapping-major). Every driver attempt, even on an
unchanged mapping, gets its own precheck and power cycle. A mapping is
**skipped** if the precheck saw coupling on a path it would drive (see the
next section). A listed mapping is only a candidate: it is energised only
if the backend and the safety policy permit it.

**Three-wire power-up:** GND driven LOW, VCC driven HIGH, DATA as
`INPUT_PULLUP`; the fourth node stays plain `INPUT`. On the ESP32 core
3.x, `digitalWrite()` before `pinMode(OUTPUT)` sets the output latch
first, so a pin never briefly drives the opposite level. This preparation
runs only for `ThreeWirePowered + ProtectedActive` drivers; it never runs
for a passive or four-wire driver.

**Four-wire power-up:** every probe GPIO plain `INPUT` and the backend
switched off (break), then the backend asked to route this mapping
(make). If it refuses, the attempt fails as `backend-refused` and nothing
is energised.

**Teardown** after a failed attempt (and before any change of mapping or
topology): three-wire DATA pull-up/drive removed, then VCC released; then
all four nodes set to plain `INPUT`; then the four-wire backend's supply
and routes switched off (signals are released before the supply, so a
part is never back-fed from a GPIO). Only then is the driver reset. The
engine then waits `PROBE_POWER_OFF_MS` (300 ms). That timed wait does not
prove a capacitor has discharged, and the firmware never shorts nodes to
discharge them.

## Precheck routing (diagnostic heuristic only)

Before every attempt, each of the four nodes in turn is set to
`INPUT_PULLUP` while the other three are `INPUT_PULLDOWN`. After a short
settle time, all four nodes are sampled four times, then everything
returns to plain `INPUT` (and the four-wire backend is off). This covers
all six pairs: A–B, A–C, A–D, B–C, B–D, C–D (`coupled=` in the Serial log
lists them).

The precheck **observes**; it never grants safety. A closed button, a wire
and a few-kΩ resistor can all look the same to it, and a finite resistance
does not make powered probing safe either. So the result routes attempts
instead of latching a fault:

| Observation | Weak-pull passive measurement | Three-wire (GPIO) powered probing | Four-wire (backend) probing |
|---|---|---|---|
| No pulled-down node read HIGH (`NoObviousCoupling`) | Runs | Runs (behind the protected interface) | Runs **only** if the backend reports the required capabilities; the precheck grants nothing |
| A pulled-down node read HIGH: that pair is coupled (`CouplingObserved`) | Runs; the ADC tells contact from resistance | **Blocked across that path**: a mapping is skipped if its VCC node is coupled to any other node | **Every mapping blocked** (see below) |
| Readings changed between samples (`InconsistentReadings`) | Retried up to `PRECHECK_MAX_RETRIES` times, then **unknown**: nothing runs this cycle | Same: no safety is inferred | Same |
| ADC or configuration failure (passive driver) | Measurement stops, pins reset, the attempt fails | No fallback permission | No fallback permission |

Three-wire DATA–GND coupling is tolerated, as before. GND only sinks and
DATA is only pulled up or sunk, so no opposing strong drives meet across
it. A four-wire mapping drives or is driven on three of its four nodes:
VCC is supplied, SIG1 (TRIG) is driven by the ESP32 and SIG2 (ECHO) by
the sensor. Coupling on any of them means a low-impedance supply path or
output contention, and GND is the only undriven role, so **any observed
coupling vetoes every four-wire mapping**. A coupling observed during a
retry of a saved powered mapping makes that retry fail
(`blocked-by-precheck`), so the mapping is not powered again.

Observations are never reused: every attempt, including each retry, runs
a fresh precheck with all nodes reset first.

Limits: the ESP32's internal pull-up and pull-down are both weak (tens of
kΩ) and roughly equal. Even a dead short between two nodes forms a
mid-rail divider that may read either way, and a short across three or
more nodes reads LOW. LOW readings never prove there is no short. A
module's DATA pull-up resistor or a chip's ESD diodes can also look like
coupling, which blocks the matching powered mappings. The external
protection remains necessary either way.

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
  onboard resistor in every mapping.
- **Mappings:** all 24 three-wire mappings, the original A/B/C six first,
  so a DHT11 can sit on any three socket positions. The node it does not
  use stays plain `INPUT`.

The result is reported as **DHT11-compatible**. The protocol has no model
ID, so PinDex cannot prove the part is a DHT11, and it does not identify
every DHT-family sensor.

## HC-SR04 ultrasonic (four-wire)

Implemented by `UltrasonicDriver` (`src/drivers/ultrasonic_driver.cpp`),
registered between the DHT11 and passive drivers. Capabilities:
`FourWirePowered + ProtectedActive`, SIG1 = **TRIG**, SIG2 = **ECHO**,
plus the backend capability `kBackendSupply5V`.

### Status on this board: hardware required

**Real ultrasonic probing is not enabled.** The shipped
`four_wire_backend.cpp` reports no capabilities and refuses every
connection, because no protected four-wire interface exists. So:

- the engine never schedules a four-wire attempt (the scan has 24 + 6 = 30
  attempts, not 54);
- the boot log prints `HC-SR04 Ultrasonic: hardware required (no
  protected four-wire backend); not attempted.` and every status line
  carries `hw-required=[HC-SR04 Ultrasonic]`;
- the DHT11 and passive scans run exactly as they would without it; this
  is never a scan fault;
- no setting enables it. `PINDEX_POWERED_PROBING_ENABLED` does not, and
  must not be made to.

The protocol, timing, validation and engine integration are complete and
covered by the host tests against a fake protected backend and a
simulated HC-SR04.

### Why the bare-GPIO arrangement cannot be used

The standard HC-SR04 needs a **5 V supply** (about 15 mA) and drives a
**5 V ECHO** signal. The three-wire DHT path drives a GPIO HIGH as a
nominal 3.3 V "supply", which is below the module's rating, and an ECHO
pin wired straight to a GPIO would put 5 V on an ESP32 input. A divider
on one assumed ECHO terminal does not solve it either: until the scan
finds the mapping, the real ECHO could be any of the four nodes. Current
limiting alone does not protect against every wrong assignment.

A 3.3 V-compatible ultrasonic module is a different part and is not
substituted silently; it would need its own driver and requirements.

### What the four-wire backend must provide

Before any FourWirePowered attempt the engine requires the backend to
report all of these (`sensor_driver.h`):

| Capability | Meaning |
|---|---|
| `kBackendSupplySwitching` | Supply routed to whichever node is VCC, and switched off on command |
| `kBackendGroundRouting` | Return path routed to whichever node is GND, and only that node |
| `kBackendSignalLevels` | No node can put more than 3.3 V on its ESP32 GPIO, in any mapping |
| `kBackendAssignmentProtection` | Every wrong candidate mapping (24 of them) is tolerated without damage |
| `kBackendBreakBeforeMake` | All routes open before a new mapping closes |
| `kBackendSupply5V` (this driver) | 5 V, current-limited, enough for the sensor |

The backend is addressed by socket node (A–D), not GPIO number. A real
routing circuit will need its own control pins, which this firmware does
not define; the engine keeps every probe GPIO plain `INPUT` while the
backend powers a mapping, and the driver uses only the TRIG and ECHO
GPIOs. Reporting a capability is a hardware design decision; never make
the shipped backend report one to "try it out".

### Protocol and timing

All values are in `config.h`.

| Setting | Value | Note |
|---|---|---|
| `ULTRASONIC_POWER_STABILIZE_MS` | 200 ms | Backend power-on to first trigger. An **engineering choice, not a datasheet figure**: validate it on the actual module. |
| `ULTRASONIC_TRIGGER_PULSE_US` | 10 µs | TRIG HIGH width (minimum) |
| `ULTRASONIC_ECHO_DEADLINE_US` | 25 000 µs | One budget from the trigger reference to the ECHO falling edge |
| `ULTRASONIC_MIN_INTERVAL_MS` | 70 ms | Trigger to trigger, always, including retries |
| `ULTRASONIC_SAMPLE_INTERVAL_MS` | 100 ms | Monitoring cadence |
| `ULTRASONIC_CONFIRM_SAMPLES` | 2 | Consecutive fresh valid echoes per `probe()` |
| Accepted range | 2.0–300.0 cm | Inclusive: 116–17 400 µs |

One measurement:

1. Pending until 70 ms after the previous trigger (the engine has already
   waited the stabilization).
2. ECHO must read LOW, otherwise the measurement fails as
   `echo-stuck-high` and no trigger is sent.
3. The capture is cleared and armed. TRIG is already LOW: it was latched
   LOW before its output turned on.
4. TRIG goes HIGH for at least 10 µs (a bounded busy wait, no critical
   section), then LOW. The **trigger reference** is the instant just
   before TRIG returns LOW.
5. A `CHANGE` interrupt on ECHO timestamps the first rising edge and the
   falling edge after it with `micros()`. The ISR records only that, under
   a brief spinlock; it never formats, logs, allocates or draws.
6. The driver returns Pending until the capture completes, then validates
   it.

`distanceCm = echoDurationUs / 58.0f`. A measurement is accepted only if
both edges came after the trigger reference, in order, with the falling
edge within 25 ms of the reference, and the distance in range. The
deadline is judged on the falling edge's own timestamp, so an on-time
echo is accepted even if the loop processes it much later (e.g. after an
OLED transfer). All arithmetic is unsigned, so a `micros()` rollover
(every ~71.6 min) is harmless.

Failure reasons (`err=` in the Serial log): `echo-stuck-high`, `no-echo`
(never rose: no echo, or ECHO stuck LOW), `echo-timeout` (rose but did not
fall within the budget), `pre-trigger-echo` (an edge before the trigger
reference, e.g. ECHO coupled to TRIG or a stale edge), `malformed-echo`
(edge order wrong, e.g. a glitch), `out-of-range`.

Edges belong to the transaction armed in step 3; anything outside that
window is ignored or rejected. `reset()` (and every failure) disarms the
capture, **detaches** the interrupt and returns TRIG to plain `INPUT`. On
this core `pinMode()` keeps a pin's interrupt type, so detaching is what
stops it; no ultrasonic interrupt is left attached when the same pins are
later used for the DHT capture or passive ADC.

### Confirmation, monitoring and loss of response

- `probe()` succeeds only after **two consecutive fresh** valid echoes on
  the same mapping, at least 70 ms apart. The engine then publishes after
  its usual confirmation (one more fresh sample), so a published result
  rests on three genuine echoes. No sample is ever reused, readings need
  not be identical (the target may move), and a new mapping or a reset
  starts confirmation from zero.
- **Monitoring** keeps the mapping powered and takes a fresh measurement
  every 100 ms. Telemetry is updated only after validation:

  ```text
  HC-SR04 Ultrasonic
  Dist: 123.4 cm
  Echo: 7157 us
  A:VCC B:TRG
  C:ECH D:GND
  LIVE
  ```

- **A missing echo** may mean no suitable target, a target out of range,
  an acoustic problem, or removal. PinDex cannot tell which. A failed
  measurement marks the reading *STALE*, tears the mapping down (signals,
  then backend supply and routes), and retries the saved mapping after the
  normal cooldown with a fresh precheck and stabilization. After
  `PROBE_MAX_CONSECUTIVE_FAILURES` (3) the result is cleared and discovery
  resumes: a **loss of response**, not proof the module was pulled out.
- Two valid echoes show **HC-SR04-compatible** behaviour (TRIG/ECHO timing
  on a 5 V module), not a specific model. The display name says
  `HC-SR04 Ultrasonic` for brevity.

## Passive components and switches

Implemented by `PassiveDriver` (`src/drivers/passive_driver.cpp`). Its
capabilities are `TwoTerminal + WeakPullOnly`. The engine never powers
anything for it; it measures the six pairs **A–B, B–C, A–C, A–D, B–D,
C–D** once per scan cycle, after the active (three- and four-wire)
attempts, so a passive fallback never hides an active protocol.

**Only connect unpowered parts.** The weak pull-up limits the current the
firmware sources (about 70 µA into a short), but the open-drain LOW sink
is an ordinary GPIO. Nothing in software protects it from an external
voltage source, a charged capacitor or ESD.

### How a pair is measured

For each orientation of the pair:

1. Both terminals are set to plain `INPUT`. The other two nodes are never
   touched and stay plain `INPUT`.
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
and noisy results are not interpreted. All six pairs are measured before
anything is published:
- exactly one conducting pair → candidate;
- several conducting pairs (e.g. three-node coupling, a potentiometer) →
  *No Component / Unknown* with `err=ambiguous-pairs`;
- none → no component.

A candidate is then re-measured from a fresh precheck and confirmed with a
new sample.

### ESP32 ADC details (checked, but not on hardware)

- GPIO 25/26/27 are **ADC2** channels on the original ESP32; GPIO 33
  (node D) is **ADC1** channel 5. The adapter looks each pin's unit up
  with `adc_oneshot_io_to_channel()` and claims and calibrates each unit
  separately on first use, so pairs with and without D both work. ADC2
  conflicts (e.g. with Wi-Fi, which PinDex does not use) surface as
  measurement errors.
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

**Mapping lines:** `Pins: A & B` (the two terminals), then the unused
nodes, `C:--- D:---`. Example screens, with illustrative values only:

```text
Resistive Component      Push Button / Switch     Photoresistor (LDR)
R~ 12.4 kOhm             State: OPEN              Light: 68%
Signal: 42%              Pins: A & C              R~ 8.2 kOhm
Pins: A & B              B:--- D:---              Pins: C & D
C:--- D:---              LIVE                     A:--- B:---
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
1. every three-wire attempt (24 mappings), mapping-major, for drivers in
   registry order;
2. every four-wire attempt (24 mappings), only for drivers whose backend
   capabilities are present (none on this board);
3. the six passive pairs, once.

Powered attempts are skipped when powered probing is off or the precheck
blocks that mapping; four-wire attempts are left out of the cycle
entirely without the backend capabilities; passive attempts are skipped
when passive probing is off. The screen shows *Scanning n/m*: m = 30 as
shipped (24 + 6), 54 with a capable four-wire backend. An active match
ends the cycle before any passive attempt.

**With powered probing enabled**, the DHT11 driver behaves as follows:

1. **Scan:** the precheck runs, then three-wire mapping 1 is powered. The
   engine waits 1100 ms, then `probe()` sends the start pulse and captures
   the reply. A failed attempt is torn down before the next one.
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
   A:VCC B:DAT
   C:GND D:---
   LIVE
   ```

   Line 1 is the driver's `getName()`, then its telemetry, then the
   engine's mapping, always in socket order A, B, C, D, with every role
   generated from the actual mapping and unused nodes as `---`. The
   renderer measures the one-line form (`A:VCC B:DAT C:GND D:---`, 138 px
   at the size-1 font, wider than the 128 px panel) and splits it into two
   halves when it does not fit, which with four nodes is always. Readings
   repeat every 2100 ms.
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

The ultrasonic driver, where a backend permits it, follows the same
framework with its own timing (see
[HC-SR04 ultrasonic](#hc-sr04-ultrasonic-four-wire)); the DHT11 failure
handling is unchanged by it.

**Serial commands:** `r` clears a latched internal fault; `c` clears the
current result and rescans. Any other letter goes to the detected driver
(the passive driver uses `l`, `d` and `b`).

The splash is drawn once at boot. When it expires the screen follows the
probe status, so the splash can never overwrite a result or a fault.
Serial prints the registered drivers at boot (and which need hardware)
and one line per status change, e.g.
`probe: detected DHT11 Temp & Hum [A:VCC B:DAT C:GND D:---] "T: 25*C  |  H: 60%" hw-required=[HC-SR04 Ultrasonic]`,
plus the heartbeat `heartbeat uptime=12s display=ok probe=detected`.

### Timing and latency

| Event | Approximate time |
|---|---|
| One failed three-wire mapping (44 ms precheck + 1100 ms + start + 300 ms off) | 1.47 s |
| Full empty-socket scan, as shipped (24 three-wire + 6 pairs) | ≈ 37.5 s, then 5 s idle |
| First valid DHT frame at three-wire entry *k* | 1.17 s + (*k* − 1) × 1.47 s |
| Confirmed DHT result | the above + 2.1 s (≈ 3.3 s for entry 1, ≈ 10.6 s for entry 6, ≈ 37 s for entry 24) |
| Removal inferred after extraction | ≈ 4.3–6.5 s |
| One failed four-wire mapping (with a backend: precheck + 200 ms + 25 ms deadline + 300 ms off) | ≈ 0.57 s |
| Extra scan time with a capable four-wire backend | ≈ 13.7 s for all 24 |

**Phase 4 latency cost:** the D node quadruples the three-wire mappings
(6 → 24), so an empty-socket cycle grew from about 8.4 s to about 37.5 s.
The per-mapping 1100 ms DHT stabilization dominates. Each extra registered
driver adds its own attempt (precheck, stabilization, operation,
power-off) to every mapping of its topology.

The passive pairs add about 2.3 s per cycle: six pairs of about 0.38 s
each (precheck, two measurement windows, 300 ms off). A passive result is
confirmed about 0.5 s after its cycle ends. Passive monitoring samples
every 100 ms, so switch transitions show within about 0.2–0.3 s.

**Blocking:** all millisecond waits are cooperative. There are two busy
waits:
- the DHT capture inside `Dht11Driver`. It masks interrupts on the
  running core (a FreeRTOS critical section that restores the previous
  state on every exit) for at most 6.5 ms, typically about 4–5 ms per
  frame. The 20 ms start pulse, stabilization, retries, Serial output and
  display updates never run inside it;
- the ultrasonic trigger pulse, about 10 µs, without a critical section.
  The echo itself is captured by interrupt; no wait or lock is held for
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
| Probing permissions, backend capability checks and the weak-pull precheck routing | Signalling on the nodes handed to it (DHT start pulse; ultrasonic TRIG/ECHO; passive pull-up + open-drain sink) |
| Which mappings/pairs are attempted, in which order, by which driver | Handshake, bounded capture, echo interrupt or ADC windows, decoding and validation |
| Preparation by capability: GPIO VCC/GND sequencing for three-wire, backend routing for four-wire, nothing for `WeakPullOnly` | Its own sample storage, classification, confirmation evidence and telemetry formatting |
| Stabilization scheduling, teardown to high impedance and backend off | Its own critical sections and interrupts (DHT capture; ultrasonic capture lock and ECHO interrupt) |
| Confirmation counting, retry limit, removal inference | Optional user commands (e.g. guided LDR) |
| Candidate/active driver and mapping; the result snapshot | — |

A driver never powers VCC/GND, picks other nodes, draws on the OLED or
changes engine state. On every terminal failure the engine first makes the
pins safe (and switches the backend off), then calls the driver's
`reset()`.

### Lifecycle and contract

- `probe(const ProbeContext &)` and `readSample(line1, line2, maxLen)` are
  **cooperative**. A `ProbeContext` holds named GPIOs for one topology
  only: `powered.{vcc,data,gnd}`, `fourWire.{vcc,sig1,sig2,gnd}` or
  `pair.{first,second,unused[2]}`. A passive terminal is never passed as
  "VCC". A driver given the wrong topology fails without touching any
  pin.
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
  - **topology**: `ThreeWirePowered` (24 mappings), `FourWirePowered`
    (24 mappings) or `TwoTerminal` (six pairs);
  - **excitation**: `ProtectedActive` (both powered topologies) or
    `WeakPullOnly` (two-terminal). Any other combination is never
    attempted.
  - power stabilization (0 for passive);
  - minimum acquisition interval;
  - sample interval (raised to the minimum if it is lower);
  - operation timeout, after which the engine fails an operation that is
    still `Pending`;
  - **backendCapabilities** (four-wire): `kBackend*` bits the sensor needs
    on top of `kBackendFourWireBaseline`. The engine runs a four-wire
    driver only if `fourWireBackendCapabilities()` reports all of them;
    otherwise the driver is left out of the scan and reported in
    `ProbeStatus::hardwareRequired`;
  - **signalLabel** (four-wire): display names for SIG1/SIG2, e.g.
    `TRG`/`ECH` (default `S1`/`S2`).
- **Attempt order:** within a cycle, `ThreeWirePowered`, then
  `FourWirePowered`, then `TwoTerminal`. Registry order applies within a
  topology. Two-terminal successes are judged over all pairs before one
  becomes the candidate.
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
   topology. Drivers are static, never allocated. The current order is
   `Dht11Driver`, `UltrasonicDriver`, `PassiveDriver`, which matches the
   engine's topology order.

No engine or display change is needed. Add tests with the mock in
`tests/` before trying the driver on hardware.

### Topology limits

Three topologies exist on the four-node socket:
- **`ThreeWirePowered`:** VCC/DATA/GND over 24 mappings, powered
  GND-first from GPIOs; the fourth node stays plain `INPUT`. Suits
  three-pin, single-data-line sensors that tolerate that policy.
- **`FourWirePowered`:** VCC/SIG1/SIG2/GND over 24 mappings, powered only
  by a protected backend that reports its capabilities. Suits four-pin,
  two-signal modules such as the HC-SR04.
- **`TwoTerminal`:** a terminal pair plus two unused nodes, with
  weak-pull excitation only. Suits unpowered resistive parts, contacts
  and switches; it is never powered.

Not supported:
- **I2C parts:** four-pin I2C modules match the four-wire shape, but
  would need open-drain bus handling and a driver; none is implemented.
- **Parts needing other excitation**, e.g. a current source, AC, or
  polarity-aware diode tests.

Such families need a new topology or excitation value and a matching
engine policy, not branches inside the state machine. No such driver is
implemented.

### Phase 4 changes to existing behaviour

- The socket has a fourth node, D (GPIO 33). Three-wire mappings grew
  from 6 to 24 (the original six first, in their old order) and pairs
  from 3 to 6; the empty-socket cycle grew from about 8.4 s to 37.5 s.
- The precheck pulls all four nodes and reports six pairs; `coupled=`
  lists pair names instead of the fixed `AB BC AC`.
- The mapping text is two halves in A–D order (`A:VCC B:DAT` /
  `C:GND D:---`, `Pins: A & B` / `C:--- D:---`) instead of
  `A:VCC | B:DAT | C:GND` or `Pins: A & B, C:---`.
- `resetTestPins()` also switches the four-wire backend off.
- `ProbeStatus::scanIndex/scanTotal` are 16-bit; new fields
  `signalLabel` and `hardwareRequired`.
- DHT11 timing, validation, confirmation and failure handling are
  unchanged.

### Phase 3.2 changes to existing behaviour

- Precheck coupling no longer latches *Socket rejected*. It blocks powered
  drive across the coupled path and routes the socket to passive
  measurement.
- Inconsistent precheck readings are retried twice, then the cycle ends as
  unknown. They no longer latch a fault.
- `PINDEX_POWERED_PROBING_ENABLED 0` alone no longer keeps the probes
  untouched, because passive probing has its own setting. Set both to `0`
  for the old fully-disabled behaviour.
- *Scanning n/m* counts all attempts in a cycle (then 6 powered + 3
  pairs; since Phase 4, 24 + 6 as shipped).

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

`tests/test_pindex.cpp` exercises the probe engine and all three driver
contracts against these simulations:
- `tests/mock/Arduino.h`: simulated GPIO, clock, critical sections and
  pin interrupts (`attachInterruptArg`/`detachInterrupt`; tests deliver
  edges at their own instants);
- a resistor-network solver, which drives both the precheck's digital reads
  and the simulated ADC;
- a simulated DHT11 and a simulated HC-SR04 (TRIG/ECHO timing, stuck
  levels, missing echoes, TRIG coupling);
- a **fake** protected four-wire backend that records every connection and
  flags any made without a break, with a GPIO driven, with an interrupt
  attached or without the capabilities. It exists only in the tests; the
  shipped backend provides nothing;
- fake drivers that exercise the architecture.

Coverage added in Phase 4 includes: the 24/24/6 mapping tables
(uniqueness, roles, original DHT order first), all six precheck pairs,
unused nodes staying high impedance, the four-wire veto, teardown
switching the backend off after the GPIOs, "hardware required" with no or
partial capabilities while DHT11 and passive scans continue, echo
conversion and range boundaries, missing/stuck/malformed/late/pre-trigger
echoes, the 25 ms budget, on-time echoes processed late, `micros()`
rollover, trigger spacing, fresh confirmation samples, cancellation and
late interrupts, mapping changes, four-node mapping text widths, and the
passive ADC and DHT protocol working on the same pins after ultrasonic
teardown.

The test file itself defines the registry functions, the passive ADC
adapter and the four-wire backend, so it is linked **without**
`src/drivers/driver_registry.cpp`, `src/drivers/passive_adc.cpp` and
`four_wire_backend.cpp`. Shorts, faults and parts are simulated only.
The tests need a host C++17 compiler (not the ESP32 toolchain). From the
`PinDex` folder:

```text
SRC="tests/test_pindex.cpp probe_engine.cpp dht11.cpp src/drivers/dht11_driver.cpp src/drivers/passive_driver.cpp src/drivers/ultrasonic_driver.cpp"
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
   `no-obvious-coupling`, *Scanning 1/30 … 30/30* (about 37 s),
   *No Component / Unknown*, the boot line about the ultrasonic needing
   hardware, and a rescan after 5 s.

   With a scope or logic analyser on the interface outputs, verify:
   - GND comes up before VCC, with no opposite-level glitch as either
     turns on;
   - DATA is released first at teardown;
   - the off-time between mappings is at least 300 ms;
   - the start pulse is 20 ms;
   - D stays high impedance during mappings 1–6;
   - nothing is driven while idle or in fault.
4. **Known-good module, mapping 1:** insert the module with A = VCC,
   B = DATA, C = GND (D empty). Expect a confirmed result in about 3.3 s
   showing `A:VCC B:DAT` / `C:GND D:---`, with readings close to a
   reference thermometer/hygrometer.
5. **Other orientations and D:** test these only if the interface is
   designed to tolerate reverse polarity and DATA-as-supply for that
   module. With A = DATA, B = VCC, D = GND (entry 9) expect a result after
   about 15 s showing `A:DAT B:VCC` / `C:--- D:GND`.
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
   - Repeat on B–C, A–C, A–D, B–D and C–D, and check the `Pins:` lines.
     The pairs with D sense on GPIO 33 (ADC1); expect values comparable
     to the ADC2 pairs.
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

### Ultrasonic bench steps (only after a protected four-wire backend exists)

Do not start these on the current board: there is no backend, and the
standard HC-SR04 must never be powered from, or have its ECHO wired to, a
bare ESP32 GPIO.

14. **Backend first:** design and review the routing interface against the
    capability table in
    [HC-SR04 ultrasonic](#hc-sr04-ultrasonic-four-wire), implement
    `four_wire_backend.cpp` for it (its own control pins, honest
    capability bits), and verify with a meter and scope, with no sensor
    fitted, that for every one of the 24 mappings: 5 V appears only on the
    VCC node, ground only on the GND node, no probe GPIO ever sees more
    than 3.3 V, and every route opens before the next mapping closes.
15. **Empty socket:** expect *Scanning n/54*, no `hw-required`, every
    four-wire attempt ending `no-echo`, and the supply off between
    attempts and while idle.
16. **Known-good HC-SR04, mapping 1:** A = VCC, B = TRIG, C = ECHO, D =
    GND, a flat target about 50 cm away. Expect *HC-SR04 Ultrasonic*,
    `Dist:` within a few cm of a tape measure, `Echo:` ≈ 58 µs per cm,
    and `A:VCC B:TRG` / `C:ECH D:GND`.
    - Scope TRIG: ≥ 10 µs pulses at least 70 ms apart (about 100 ms while
      monitoring).
    - Validate `ULTRASONIC_POWER_STABILIZE_MS` (200 ms): from supply-on to
      the first trigger, the first echoes must already be correct. If
      not, raise it.
17. **Range edges:** targets at about 3 cm and 2.5 m should be accepted;
    closer than 2 cm or with no target the reading goes *STALE*, then the
    result is cleared after three failures (loss of response).
18. **Other mappings:** rotate the module through a few of the 24 mappings
    the backend is rated for and confirm each is found and labelled.
19. **Handoff:** after removing the module, repeat steps 4 and 8; the
    DHT11 and passive results must be unchanged.

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
    mappings; a module's onboard pull-up can cause this.
- **`hw-required=[HC-SR04 Ultrasonic]`:** expected on this board. The
  ultrasonic driver needs a protected four-wire backend that is not
  fitted, so it is skipped; other detection is unaffected.
- **An HC-SR04 is inserted but shows *No Component / Unknown*:** expected
  without the backend. Remove it; do not power it from the probe GPIOs.
- **Ultrasonic `err=no-echo` / `echo-timeout` (with a backend):** no
  target in range, a soft or angled target, or the module was removed.
  These cannot be told apart.
- **`err=precheck-inconsistent`:** the precheck readings kept changing, so
  nothing was attempted. Check for loose contacts.
- **`err=ambiguous-pairs`:** more than one of the six pairs conducts (a
  three-terminal part or a network). This is not supported as a passive
  result.
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
- Four-wire (ultrasonic) probing runs only on a backend that reports the
  required capabilities; the shipped one reports none. A 5 V module such
  as the standard HC-SR04 must never be powered from, or have its ECHO
  wired to, a bare probe GPIO.
- Passive probing never drives a probe HIGH, but its open-drain LOW sink
  is an unprotected GPIO. Use it only with unpowered parts.
- Do not connect powered circuits or charged capacitors to the probes, and
  never apply 5V to them.
- "Ready" means the firmware is running. Detection is limited to:
  - DHT11-compatible sensors (powered probing);
  - HC-SR04-compatible ultrasonic modules, only with a protected
    four-wire backend (not fitted);
  - approximate two-terminal resistive and contact behaviour (passive
    probing).

  Neither proves what a part is.
