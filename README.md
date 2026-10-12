# PinDex

**Smart Component & Sensor Identifier**, an ESP32 instrument for electronics
labs. The goal is to let students insert a component or sensor module, have
it identified, and get its pinout.

> **Status: v0.2 + Phase 5.** A four-node socket (A, B, C, D) and four
> drivers sit behind a generic probe engine, in this priority order:
> - DHT11-compatible detection with pinout and live telemetry (three-wire,
>   powered from GPIOs);
> - an I2C address scanner (four-wire, 3.3 V). Fully implemented and tested
>   in simulation, but **not attempted as shipped**: it needs a protected
>   3.3 V four-wire interface that does not exist yet, or the explicitly
>   enabled *experimental* GPIO power profile (off by default). See
>   [I2C scanner](#i2c-scanner-four-wire);
> - HC-SR04-compatible ultrasonic distance (four-wire, 5 V). Fully
>   implemented and tested in simulation, but **not attempted on this
>   board**: it needs a protected 5 V interface that does not exist yet
>   (see [HC-SR04 ultrasonic](#hc-sr04-ultrasonic-four-wire));
> - weak-pull passive measurement of resistive parts, contacts and
>   switch-like behaviour, plus an optional user-guided LDR mode.
>
> Skipped drivers are reported as *hardware required*. Three settings in
> `config.h` control probing:
> - **`PINDEX_POWERED_PROBING_ENABLED`** drives probe GPIOs as VCC/GND, so
>   use it only behind the protected interface described below. It never
>   enables four-wire probing on its own, and `0` also disables the I2C
>   GPIO profile.
> - **`PINDEX_I2C_GPIO_POWER_ENABLED`** (default `0`) lets the I2C scanner
>   power modules straight from probe GPIOs. Experimental and **not
>   protection**; read the I2C section before enabling it.
> - **`PINDEX_PASSIVE_PROBING_ENABLED`** uses an internal pull-up and an
>   open-drain LOW sink only, and is for unpowered parts.
>
> With powered and passive probing both `0` the probes stay high impedance
> and the display shows *Probing disabled*. **Check all three values before
> every upload.** Nothing here identifies arbitrary components, and
> inserting arbitrary components is not safe.

## Project files

| File | Purpose |
|---|---|
| `PinDex.ino` | Sketch entry point: startup order, screen updates, Serial heartbeat and Serial commands (`r`, `c`, driver commands). |
| `config.h` | All pins, timings, thresholds, failure limits and the three probing settings. |
| `display_manager.h/.cpp` | VSPI + SSD1306 setup; renders the engine's status snapshot (splash, idle, status, result screens). |
| `probe_engine.h/.cpp` | Generic orchestrator: owns the node table and probe GPIOs, the mapping tables, precheck routing, power sequencing (GPIO or four-wire backend), capability-based attempt scheduling, confirmation, monitoring and removal. Knows no concrete sensor. |
| `four_wire_backend.h/.cpp` | The protected four-wire power/routing interface seam. The shipped implementation reports **no capabilities** and refuses every connection, because no such hardware exists yet. |
| `sensor_driver.h` | The abstract `SensorDriver` contract: operation status, `DriverRequirements` (topology, excitation, supply profile, open-drain signals), `ProbeContext`, optional release/diagnostics hooks. |
| `src/drivers/dht11_driver.h/.cpp` | `Dht11Driver`: DHT start pulse, bounded capture, validation and telemetry formatting. |
| `src/drivers/ultrasonic_driver.h/.cpp` | `UltrasonicDriver`: HC-SR04 trigger pulse, interrupt-timestamped echo capture, validation, confirmation and telemetry. |
| `src/drivers/i2c_driver.h/.cpp` | `I2cDriver`: incremental 0x08–0x77 address sweeps, two-sweep confirmation, round-robin liveness checks, address hints. |
| `src/drivers/i2c_bus.h/.cpp` | The only I2C controller code: exclusive owner of `Wire1`, return-code mapping, idle-line checks, release. |
| `src/drivers/passive_driver.h/.cpp` | `PassiveDriver`: weak-pull two-terminal measurement, classification, switch debouncing, guided LDR mode. |
| `src/drivers/weak_pull_meter.h/.cpp` | One weak-pull divider measurement of a pair in one orientation (pull-up verified, open-drain sink), shared by `PassiveDriver` and the engine's pull-up classification. |
| `src/drivers/passive_adc.h/.cpp` | The only ADC code: ESP-IDF oneshot ADC (both ADC units) with pull-up control and error checking. |
| `src/drivers/driver_registry.h/.cpp` | The static driver table (priority order) and its read-only API. |
| `dht11.h/.cpp` | Pure DHT11 frame decoding (timing → bits → bytes → values); no GPIO, host-testable. |
| `tests/` | Host-side tests with a mock Arduino layer (incl. interrupts), fake drivers, a resistor-network simulator, a simulated ADC, a simulated DHT11, a fake protected four-wire backend, a simulated HC-SR04 and a simulated I2C bus and module. Not compiled by the Arduino IDE. |
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

That GPIO arrangement is the **three-wire** path. **Four-wire** attempts
are powered according to the driver's **supply profile**
(`DriverRequirements::supply`):

| Profile | Used by | Powered by |
|---|---|---|
| `GpioLogic` | DHT11 (three-wire) | GPIO VCC HIGH / GND LOW, as above |
| `Backend5V` | HC-SR04 | The protected four-wire backend's switched 5 V **only**. Never GPIOs. |
| `Supply3V3` | I2C scanner | The protected backend's switched 3.3 V if it reports it; otherwise **only** the experimental GPIO profile, and only when `PINDEX_I2C_GPIO_POWER_ENABLED` is `1` |
| `None` | Passive | Nothing |

No such backend is fitted and the GPIO profile is off by default, so as
shipped no four-wire attempt runs, whatever `PINDEX_POWERED_PROBING_ENABLED`
says. A profile is never granted at run time, e.g. because an address
hint suggests a low-current part.

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
if its supply profile and the safety policy permit it.

**GPIO power-up** (three-wire, and the experimental four-wire I2C
profile): every node plain `INPUT` and the backend off first, then GND
driven LOW, then VCC driven HIGH; three-wire DATA as `INPUT_PULLUP`; the
other nodes stay plain `INPUT`. On the ESP32 core 3.x, `digitalWrite()`
before `pinMode(OUTPUT)` sets the output latch first, so a pin never
briefly drives the opposite level.

**Backend power-up** (four-wire): every probe GPIO plain `INPUT` and the
backend switched off (break), then the backend asked to route this
mapping with the profile's supply (make). If it refuses, the attempt fails
as `backend-refused` and nothing is energised.

**Teardown** after a failed attempt (and before any change of mapping or
topology), in this order:
1. the driver's `reset()`, while the attempt is still powered: it stops
   its protocol and releases what it holds (DHT start pulse, ultrasonic
   TRIG and ECHO interrupt, I2C controller with its pulls);
2. DATA/SIG1/SIG2 to plain `INPUT`, then VCC released;
3. all four nodes plain `INPUT`, then the four-wire backend's supply and
   routes switched off (signals are released before the supply, so a part
   is never back-fed from a GPIO).

If a driver reports it could not release a peripheral
(`resourcesReleased()` false, e.g. an I2C controller that would not
stop), power still goes off, the engine latches *Socket rejected* with
`err=release-failed`, and nothing else starts until `r`. The engine then
waits `PROBE_POWER_OFF_MS` (300 ms) before the next attempt. That timed
wait does not prove a capacitor has discharged, and the firmware never
shorts nodes to discharge them.

## Precheck routing (diagnostic heuristic only)

Before every attempt, each of the four nodes in turn is set to
`INPUT_PULLUP` while the other three are `INPUT_PULLDOWN`. The pulls are
applied once per node; after `PRECHECK_SETTLE_MS` (20 ms) all four nodes
are sampled, then three more times `PRECHECK_SAMPLE_INTERVAL_MS` (5 ms)
apart, at most one sample per loop pass (about 140 ms for the whole
precheck). Then everything returns to plain `INPUT` (and the four-wire
backend is off). Both waits are provisional and not yet validated on
hardware. They give charge left by the previous attempt (e.g. a breakout's
capacitors) time to bleed through the pulls, but no wait proves a
capacitor has discharged. This covers
all six pairs: A–B, A–C, A–D, B–C, B–D, C–D (`coupled=` in the Serial log
lists them).

The precheck **observes**; it never grants safety. A closed button, a wire
and a few-kΩ resistor can all look the same to it, and a finite resistance
does not make powered probing safe either. So the result routes attempts
instead of latching a fault:

| Observation | Weak-pull passive measurement | Three-wire (GPIO) powered probing | Four-wire probing |
|---|---|---|---|
| No pulled-down node read HIGH (`NoObviousCoupling`) | Runs | Runs (behind the protected interface) | Runs **only** with an eligible supply profile; the precheck grants nothing |
| A pulled-down node read HIGH: that pair is coupled (`CouplingObserved`) | Runs; the ADC tells contact from resistance | **Blocked across that path**: a mapping is skipped if its VCC node is coupled to any other node | I2C: as three-wire, except the measured [pull-up exception](#narrow-open-drain-pull-up-exception). Ultrasonic: **every mapping blocked** (see below) |
| Readings changed between samples (`InconsistentReadings`) | One pass plus up to `PRECHECK_MAX_RETRIES` (2) retries, then **unknown**: only that attempt is skipped (never run), and discovery continues with the next one | Same: that mapping is never powered | Same |
| ADC or configuration failure (passive driver) | Measurement stops, pins reset, the attempt fails | No fallback permission | No fallback permission |

The rule underneath: **a node the attempt drives must show no coupling to
any other node.** VCC is always driven, so a coupled VCC path (to GND, or
into a line the ESP32 sinks) is a known or suspected low-impedance supply
path and vetoes the mapping, whatever its resistance might be. Signals
count as driven unless the driver declares them open-drain
(`DriverRequirements::openDrainSignals`):

- Three-wire DHT DATA and I2C SDA/SCL are open-drain (only released or
  sunk) and GND only sinks, so couplings among them meet no opposing
  strong drive and are tolerated. Only a coupled VCC vetoes.
- Ultrasonic TRIG is push-pull and ECHO is sensor-driven, and GND is the
  only undriven role, so **any observed coupling vetoes every ultrasonic
  mapping**.

A coupling observed during a retry of a saved powered mapping makes that
retry fail (`blocked-by-precheck`), so the mapping is not powered again.

Observations are never reused: every attempt, including each retry, runs
a fresh precheck with all nodes reset first.

Limits: the ESP32's internal pull-up and pull-down are both weak (tens of
kΩ) and roughly equal. Even a dead short between two nodes forms a
mid-rail divider that may read either way, and a short across three or
more nodes reads LOW. LOW readings never prove there is no short. A
module's DATA pull-up resistor, an I2C breakout's SDA/SCL pull-ups to its
VCC pin, or a chip's ESD diodes can also look like coupling, which blocks
the matching powered mappings (often the right one). A digital reading is
never converted into permission. The one narrow, measured exception is
described next. The external protection remains necessary either way.

### Narrow open-drain pull-up exception

An I2C breakout's own SDA/SCL pull-ups connect its VCC pin to both signal
lines. If the digital precheck flags those paths, the rule above would
veto the breakout's correct mapping. The engine (`decideCoupling()` in
`probe_engine.cpp`) therefore makes one exception, in this order:

1. **Permissions first.** The candidate must already be runnable:
   - an enabled topology;
   - an eligible supply (protected 3.3 V backend, or the experimental GPIO
     profile when it is enabled). The exception never enables a disabled
     profile.
2. **Capabilities, never a name.** It applies only to a
   `FourWirePowered + ProtectedActive + Supply3V3` candidate with
   open-drain signals (today: the I2C scanner).
3. **Rail.** Digital VCC–GND coupling is always vetoed, as before. A
   coupled SIG–GND pair also rules the exception out, because pull-ups do
   not explain it.
4. **Measure.** With every supply off and every other node plain INPUT,
   four pairs are measured in both orientations: VCC–GND, VCC–SDA,
   VCC–SCL and SDA–SCL. The measurement is the passive driver's
   weak-pull meter: the internal pull-up is verified, the sink is
   open-drain LOW only, and nothing is ever driven HIGH. Pins are restored
   after each orientation and on cancellation. Evidence belongs to that one
   precheck pass, so every attempt and every retry measures again.
5. **Classify and decide.** The mapping is allowed only if all of these
   hold:
   - VCC–GND reads open, or finite and well above a short. The rail is
     measured because the pull-ups can mask a rail short from the digital
     test.
   - VCC–SDA and VCC–SCL are both stable, symmetric and inside the pull-up
     band.
   - SDA–SCL matches the two pull-ups in series, within
     `PULLUP_SERIES_TOLERANCE_PCT`.

   Then the usual power-up and I2C scan run unchanged.

| Measured | Decision |
|---|---|
| Digital VCC–GND coupling | Vetoed, never measured |
| VCC–GND contact-band in either direction, unstable or directional | Blocked (`VCC-GND low impedance` / `evidence ambiguous`) |
| VCC–SDA/SCL inside 170–900 mV, stable, symmetric | Eligible |
| VCC–SDA/SCL contact band (≤ 150 mV) in either direction | Blocked (`low impedance`) |
| VCC–SDA/SCL unstable, directional, open-like, or outside the band | Blocked (`ambiguous` / `outside pull-up range`) |
| SDA–SCL contact band | Blocked (`SDA-SCL low impedance`) |
| SDA–SCL finite but not the series sum (another path) | Blocked (`not explained by pull-ups`) |
| Any ADC failure or unverified pull-up | Blocked (`ADC measurement unavailable`) |
| Digital coupling with nothing measured | Never a pass |

The bands in `config.h` (`PULLUP_COMPATIBLE_MIN/MAX_MV`) are meant to cover
4.7–10 kΩ pull-ups. They were derived **on paper**, assuming a 30–67 kΩ
internal pull-up (nominally 45 kΩ, not a precision reference) and about
±40 mV ADC error. They have **not** been validated on hardware.
Narrowing them only makes the policy stricter.

Several real-world cases are blocked, which is the conservative outcome:
- pull-ups below about 2.5 kΩ;
- pull-ups behind an on-board regulator, where the path is directional;
- large decoupling capacitance that is still charging;
- a VCC–GND path that is directional.

Readings through an unpowered module's internal circuitry are
approximate, never resistor values.

Passing only means the policy did not veto the attempt. It does not
certify the wiring as safe. Open-drain signalling does not prevent
contention with a supply short; only the measured resistance limits that
current. Outside the exception, SDA–SCL coupling with an uncoupled VCC is
still allowed exactly as before: both lines are open-drain, and the scan
fails on a shorted bus.

Serial shows each blocked evaluation (at most one per
`PRECHECK_LOG_INTERVAL_MS`) and every allowed one. This example is for
illustration; the values are not from a measurement:

```text
precheck: A:VCC B:GND C:SDA D:SCL (I2C Device)
precheck: A-B VCC-GND not-coupled -> open-like 3100/3100 mV
precheck: A-C VCC-SDA coupled -> pullup-compatible 313/314 mV
precheck: A-D VCC-SCL coupled -> pullup-compatible 313/313 mV
precheck: C-D SDA-SCL coupled -> pullup-compatible 571/571 mV
candidate: allowed; open-drain pull-up exception
```

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

## I2C scanner (four-wire)

Implemented by `I2cDriver` (`src/drivers/i2c_driver.cpp`) with its bus
adapter `src/drivers/i2c_bus.cpp`, registered after the DHT11 driver and
before the ultrasonic driver. Capabilities: `FourWirePowered +
ProtectedActive`, supply profile `Supply3V3`, open-drain signals, SIG1 =
**SDA**, SIG2 = **SCL**. It reports which 7-bit addresses answer, the
working VCC/SDA/SCL/GND mapping and a **hint** per address. It does not
identify models.

### Status as shipped: hardware required

**Physical I2C scanning is not enabled.** No protected 3.3 V four-wire
backend exists (the shipped `four_wire_backend.cpp` reports nothing), and
`PINDEX_I2C_GPIO_POWER_ENABLED` is `0`. The engine leaves the driver out of
the scan, the boot log prints `I2C Device: hardware required (no eligible
supply profile); not attempted.`, and every status line carries
`hw-required=[I2C Device]`. DHT11 and passive scans are unaffected. The
protocol, lifecycle and engine integration are complete and covered by
simulated host tests.

### Supply: three separate questions

Being able to talk 3.3 V I2C logic, accepting the supply's voltage **and
current** as a whole module, and tolerating wrong power/signal assignments
while 24 mappings are tried are three different properties. A breakout
can have the first and lack the others: regulators, power LEDs, LCD
backlights, pull-ups, inrush into capacitors and attached loads all add
up, and many breakouts draw more than a GPIO can source.

- **Preferred:** a protected backend that reports
  `kBackendFourWireBaseline | kBackendSupply3V3` (switched, current-limited
  3.3 V routed to any node, ground routing, ESP32-safe signal paths,
  wrong-assignment tolerance, break-before-make). It is used whenever it
  is present, even if the GPIO profile is enabled. A backend that offers
  only 5 V does **not** serve the I2C scanner. A backend offering both
  will also power the ultrasonic attempts at 5 V on the same socket; a
  3.3 V-only module must tolerate that, or the backend must prevent it.
- **Experimental:** `#define PINDEX_I2C_GPIO_POWER_ENABLED 1` powers each
  candidate mapping straight from probe GPIOs: GND driven LOW, then VCC
  driven HIGH, with the same latch-first sequence as the DHT path. This is
  a nominal 3.3 V **logic level**, not a regulated rail, and it is **not
  protection**: software limits no fault current and prevents no
  reverse-polarity damage, and wrong mappings put a GPIO HIGH on the
  module's GND, SDA or SCL pin. Every precheck veto still applies,
  `PINDEX_POWERED_PROBING_ENABLED 0` still disables it, and it is never
  granted to the 5 V ultrasonic driver. Enable it only for a specific
  module whose current draw, inrush and tolerance of every wrong
  assignment you have evaluated, behind the reviewed interface. Never
  enable it because an address hint suggests a low-current part. It does
  not make arbitrary insertion safe.

### Pull-ups and voltage

- SDA and SCL need pull-ups to the **3.3 V domain** that powers the
  module: most breakouts carry them (to their own VCC), a protected
  backend may provide them. The ESP32's internal pull-ups (about 45 kΩ,
  enabled by the core while the controller runs) are too weak to
  guarantee reliable 100 kHz operation.
- **Never** pull SDA or SCL up to 5 V: the probe pins are direct ESP32
  inputs and are not 5 V-tolerant.
- The controller drives SDA and SCL open-drain only; neither is ever
  push-pull HIGH.
- Breakout pull-ups between VCC and SDA/SCL may be seen by the precheck
  as VCC coupling. Such a mapping is then powered only if weak-pull ADC
  measurements show a supported pull-up network (see
  [Narrow open-drain pull-up exception](#narrow-open-drain-pull-up-exception));
  otherwise it stays vetoed.

### Bus ownership

The adapter exclusively owns the existing global `Wire1` (I2C controller
1). Nothing else in PinDex uses `Wire` or `Wire1`: the OLED is hardware
SPI, and the Adafruit SSD1306 SPI constructor leaves its `TwoWire`
pointer null. `Wire` is left alone because libraries default to it. No
second `TwoWire` object is created for either controller. If the
controller is already running (started elsewhere) the attempt fails as
`bus-conflict` and the controller is left untouched; it is never ended by
PinDex.

### Scanning

After the engine has powered the mapping and waited
`I2C_POWER_STABILIZE_MS` (100 ms, an engineering value to validate per
module), `probe()`:

1. starts the controller with `Wire1.begin(sda, scl, 100000UL)` (checked;
   failure is `bus-init-failed`, partial state released) and
   `Wire1.setTimeOut(25)`;
2. requires SDA and SCL to read released HIGH before every transaction
   and before any result is accepted, allowing `I2C_IDLE_SETTLE_MS`
   (10 ms) for a LOW line to release; otherwise `bus-stuck-low`;
3. sweeps **0x08–0x77 inclusive (112 addresses)** in order, **one
   address per call** (one per `loop()` pass), Pending in between:
   `beginTransmission(address)` then `endTransmission(true)`, no payload
   byte, always ending in STOP;
4. records ACKs in a 128-bit bitmap (never truncated), skips NACKs, and
   ends the attempt at once on a timeout (`bus-timeout`) or any other
   error (`bus-error`): a faulted bus gets no further transactions;
5. bounds the whole sweep by `I2C_SWEEP_DEADLINE_MS` (1 s, rollover-safe;
   `sweep-deadline`).

Each transaction is **synchronous**: it blocks for the address byte plus
the core's overhead, bounded by the 25 ms timeout (`Wire1.setTimeOut`)
plus implementation overhead. That is a configured bound, not a measured
hard real-time guarantee. No critical section is held around Wire calls.

`endTransmission()` codes on Arduino-ESP32 3.3.12 for an address-only
write (`i2c_master_probe()`): `0` ACK, `2` NACK, `5` timeout, `4` other
error. The core also returns `2` when the bus is not initialised, so the
adapter only probes while `i2cIsInit()` confirms its own controller runs;
only a genuine `2` counts as an ordinary miss.

A finished sweep is judged on its own: **no ACK** = no responding device
on this mapping (`no-ack`); **every address ACKing** = not a plausible bus
(`all-addresses-ack`). Any other set, one address or many, is a
candidate: a breakout may hold several devices, and one device may answer
at several addresses. A second, independently executed sweep must return
the **same set** to confirm; if it differs, a third is compared with the
second, and after `I2C_MAX_SWEEPS` (3) without two consecutive matching
sweeps the attempt fails as `unstable-addresses`. Nothing is reused from
a cache.

### Address hints are not identities

| Address | Hint shown |
|---|---|
| `0x23` | `BH1750?` |
| `0x27` | `PCF8574 / LCD?` |
| `0x3C`, `0x3D` | `SSD1306 / OLED?` |
| `0x3F` | `PCF8574A / LCD?` |
| `0x48` | `ADS1115 / Other?` |
| `0x50`–`0x57` | `AT24Cxx / EEPROM?` |
| `0x68` | `MPU6050 / DS3231?` |
| `0x69` | `MPU6050 / IMU?` |
| `0x76`, `0x77` | `BMP280 / BME280?` |
| any other | `Unknown model` (still a valid discovery) |

These are common candidates, not an exhaustive or exclusive mapping. An
address never identifies a model; a PCF8574-family answer does not prove
an LCD is attached; several ACKing addresses do not prove several
devices. The confirmed name is the generic **I2C Device**. Working
communication establishes a working candidate mapping; it does not prove
the module is powered through its intended supply pins rather than an
unintended path (e.g. through its I/O protection diodes). No register
read or write, reset, configuration or EEPROM write is ever sent. An
address-only write is the least intrusive probe, but it is not guaranteed
to be side-effect-free for every device.

### Monitoring and presentation

`readSample()` is a **liveness check**, not a measurement: one
address-only transaction to the next confirmed address (round-robin),
accepted once the lines are idle again. It runs every
`I2C_SAMPLE_INTERVAL_MS` (2 s) and also rotates the address presented, so
each shown `ACK` comes from that address's own fresh transaction. No
temperature, acceleration, pressure or other value is ever produced.

```text
I2C Device                I2C Device
Addr: 0x68 (ACK)          Addr 2/3: 0x3C ACK
MPU6050 / DS3231?         SSD1306 / OLED?
A:VCC B:GND               A:VCC B:GND
C:SDA D:SCL               C:SDA D:SCL
LIVE                      LIVE
```

The OLED renders the snapshot only: redraws and rotation never start a
bus transaction. Every confirmed address is printed once on detection and
on demand with the Serial command **`i`**, e.g.
`diagnostics: I2C ACK (3): 0x3C 0x68 0x76` (from cached results; no bus
activity).

### Failures, teardown and recovery

- **Teardown** (failed sweep, rejected confirmation, partial start,
  cancellation, mapping change, liveness failure, driver switch, reset):
  stop issuing transactions, `Wire1.end()` (result checked), pulls off
  and SDA/SCL back to plain `INPUT`; only then does the engine remove the
  supply and restore all four nodes. An ordinary NACK needs none of this;
  the sweep continues.
- **A controller that will not stop** (`end()` fails): the driver reports
  it, the engine still removes power (it never keeps the supply on to
  retry cleanup), latches *Socket rejected* with `err=release-failed`, and
  starts nothing else until `r`. Each later reset tries the stop once more.
- **Loss of response** while monitoring (`address-lost`, `bus-timeout`,
  `bus-error`, `bus-stuck-low`): the reading turns *STALE*, the mapping is
  torn down and retried with fresh power, stabilization and a full
  two-sweep confirmation. After `PROBE_MAX_CONSECUTIVE_FAILURES` (3) the
  result, address list and hints are cleared and discovery resumes. This
  is inferred loss of response, not proof of removal.
- Addresses, hints and confirmation state belong to one session and are
  cleared whenever the mapping or session changes.

### Latency (with an eligible supply)

One empty four-wire I2C attempt takes about 0.66 s (140 ms precheck +
100 ms stabilization + one ~112-pass sweep + 300 ms off). A module on
four-wire mapping *k* is confirmed about (*k* − 1) × 0.66 s + 0.5 s after
the three-wire attempts, and its first liveness sample (and the result
screen) follows 2 s later. In the simulation each pass is 1 ms; on
hardware a pass is usually faster, so sweeps are shorter.

## HC-SR04 ultrasonic (four-wire)

Implemented by `UltrasonicDriver` (`src/drivers/ultrasonic_driver.cpp`),
registered after the I2C scanner and before the passive driver.
Capabilities: `FourWirePowered + ProtectedActive`, supply profile
`Backend5V` (never GPIOs), SIG1 = **TRIG**, SIG2 = **ECHO**.

### Status on this board: hardware required

**Real ultrasonic probing is not enabled.** The shipped
`four_wire_backend.cpp` reports no capabilities and refuses every
connection, because no protected four-wire interface exists. So:

- the engine never schedules an ultrasonic attempt (as shipped the scan
  has 24 + 6 = 30 attempts);
- the boot log prints `HC-SR04 Ultrasonic: hardware required (no
  eligible supply profile); not attempted.` and every status line
  carries `hw-required=[HC-SR04 Ultrasonic]`;
- the DHT11 and passive scans run exactly as they would without it; this
  is never a scan fault;
- no setting enables it. `PINDEX_POWERED_PROBING_ENABLED` does not, and
  must not be made to; the experimental I2C GPIO profile never applies to
  it.

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

**With powered and passive probing both `0`:** splash, then
*v0.2 Ready / Probing disabled*. The probes stay plain `INPUT`, no driver
ever runs, and the Serial heartbeat continues once per second.

**Scan cycle:** one cycle runs these attempts in order, each preceded by
the precheck:
1. every three-wire attempt (24 mappings), mapping-major, for drivers in
   registry order;
2. every four-wire attempt (24 mappings), mapping-major, I2C before
   ultrasonic, only for drivers with an eligible supply profile (none as
   shipped);
3. the six passive pairs, once.

Powered attempts are skipped when powered probing is off or the precheck
blocks that mapping; four-wire drivers without an eligible supply are
left out of the cycle entirely; passive attempts are skipped
when passive probing is off. The screen shows *Scanning n/m*: m = 30 as
shipped (24 + 6), plus 24 for each four-wire driver with an eligible supply
(54 with one, 78 with both). An active match
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

The I2C scanner and the ultrasonic driver, where a supply profile permits
them, follow the same framework with their own timing (see
[I2C scanner](#i2c-scanner-four-wire) and
[HC-SR04 ultrasonic](#hc-sr04-ultrasonic-four-wire)); the DHT11 failure
handling is unchanged by them.

**Serial commands:** `r` clears a latched internal fault; `c` clears the
current result and rescans; `i` prints the detected driver's diagnostics
(the I2C scanner's full address list). Any other letter goes to the
detected driver (the passive driver uses `l`, `d` and `b`).

The splash is drawn once at boot. When it expires the screen follows the
probe status, so the splash can never overwrite a result or a fault.
Serial prints the registered drivers at boot (and which need hardware)
and one line per status change, e.g.
`probe: detected DHT11 Temp & Hum [A:VCC B:DAT C:GND D:---] "T: 25*C  |  H: 60%" hw-required=[I2C Device] hw-required=[HC-SR04 Ultrasonic]`,
plus the heartbeat `heartbeat uptime=12s display=ok probe=detected`.

### Timing and latency

| Event | Approximate time |
|---|---|
| One failed three-wire mapping (140 ms precheck + 1100 ms + start + 300 ms off) | 1.56 s |
| Full empty-socket scan, as shipped (24 three-wire + 6 pairs) | ≈ 40 s, then 5 s idle |
| First valid DHT frame at three-wire entry *k* | 1.26 s + (*k* − 1) × 1.56 s |
| Confirmed DHT result | the above + 2.1 s (≈ 3.4 s for entry 1, ≈ 11.2 s for entry 6, ≈ 39 s for entry 24) |
| Removal inferred after extraction | ≈ 4.5–6.6 s |
| One failed ultrasonic mapping (with a backend: precheck + 200 ms + 25 ms deadline + 300 ms off) | ≈ 0.67 s |
| One empty I2C mapping (with an eligible supply: precheck + 100 ms + one sweep + 300 ms off) | ≈ 0.66 s |
| Extra scan time per four-wire driver with an eligible supply | ≈ 15.8–16.0 s for all 24 |

**Precheck pacing cost:** the 20 ms / 5 ms precheck waits (previously
5 ms / 2 ms) add about 0.1 s to every attempt, ≈ 5 s over a 54-attempt
cycle. An attempt skipped as inconclusive takes about 75 ms (three
passes, each stopped at its first changed sample).

**Phase 4 latency cost:** the D node quadruples the three-wire mappings
(6 → 24), so an empty-socket cycle grew from about 8.4 s to about 37.5 s.
The per-mapping 1100 ms DHT stabilization dominates. Each extra registered
driver adds its own attempt (precheck, stabilization, operation,
power-off) to every mapping of its topology.

The passive pairs add about 2.9 s per cycle: six pairs of about 0.48 s
each (precheck, two measurement windows, 300 ms off). A passive result is
confirmed about 0.6 s after its cycle ends. Passive monitoring samples
every 100 ms, so switch transitions show within about 0.2–0.3 s.

**Blocking:** all millisecond waits are cooperative. These calls block
briefly:
- the DHT capture inside `Dht11Driver`. It masks interrupts on the
  running core (a FreeRTOS critical section that restores the previous
  state on every exit) for at most 6.5 ms, typically about 4–5 ms per
  frame. The 20 ms start pulse, stabilization, retries, Serial output and
  display updates never run inside it;
- the ultrasonic trigger pulse, about 10 µs, without a critical section.
  The echo itself is captured by interrupt; no wait or lock is held for
  it;
- one synchronous I2C address transaction per `loop()` pass while the
  I2C scanner runs, bounded by its 25 ms timeout plus core overhead, with
  no critical section.

## Sensor driver architecture

The probe engine is a generic orchestrator. Each sensor protocol is a
`SensorDriver` (`sensor_driver.h`) listed in a fixed registry. The engine
contains no sensor names, decoding, type checks or sensor-specific
branches. The display renders only the engine's `ProbeStatus` snapshot, so
redrawing never triggers an acquisition.

### Ownership

| Engine (`probe_engine.cpp`) | Driver (`src/drivers/…`) |
|---|---|
| Probing permissions, supply-profile eligibility and the weak-pull precheck routing | Signalling on the nodes handed to it (DHT start pulse; ultrasonic TRIG/ECHO; I2C controller on SDA/SCL; passive pull-up + open-drain sink) |
| Which mappings/pairs are attempted, in which order, by which driver | Handshake, bounded capture, echo interrupt, bus transactions or ADC windows, decoding and validation |
| Preparation by supply profile: GPIO VCC/GND sequencing, backend routing with the profile's voltage, or nothing for `WeakPullOnly` | Its own sample storage, classification, confirmation evidence and telemetry formatting |
| Stabilization scheduling, teardown to high impedance and backend off | Its own critical sections, interrupts and peripherals (DHT capture; ultrasonic capture lock and ECHO interrupt; the I2C controller) |
| Confirmation counting, retry limit, removal inference | Optional user commands (e.g. guided LDR) and diagnostics |
| Candidate/active driver and mapping; the result snapshot | — |

A driver never powers VCC/GND, picks other nodes, draws on the OLED or
changes engine state. On every terminal failure the engine first calls
the driver's `reset()` while the attempt is still powered, so the driver
releases its lines, interrupts and peripherals before the supply goes;
then it removes power, restores every node to plain `INPUT` and switches
the backend off. (Before Phase 5 the pins were made safe first and the
driver reset afterwards.)

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
  make a pin safer (e.g. release a held line to plain `INPUT`, detach an
  interrupt, stop a bus controller it started); it never energises one.
- `getRequirements()` returns fixed `DriverRequirements`:
  - **topology**: `ThreeWirePowered` (24 mappings), `FourWirePowered`
    (24 mappings) or `TwoTerminal` (six pairs);
  - **excitation**: `ProtectedActive` (both powered topologies) or
    `WeakPullOnly` (two-terminal).
  - power stabilization (0 for passive);
  - minimum acquisition interval;
  - sample interval (raised to the minimum if it is lower);
  - operation timeout, after which the engine fails an operation that is
    still `Pending`;
  - **supply**: `GpioLogic` (three-wire), `Backend5V` or `Supply3V3`
    (four-wire), `None` (two-terminal); any other combination is never
    attempted. A four-wire driver runs only if
    `fourWireBackendCapabilities()` reports `kBackendFourWireBaseline`
    plus the profile's supply bit, or, for `Supply3V3` alone, the
    experimental GPIO profile is enabled. Otherwise it is left out of the
    scan and flagged in `ProbeStatus::hardwareRequiredMask`;
  - **openDrainSignals**: the signal/DATA nodes are only released or sunk;
    if false, any coupling on a signal vetoes the mapping;
  - **signalLabel** (four-wire): display names for SIG1/SIG2, e.g.
    `TRG`/`ECH`, `SDA`/`SCL` (default `S1`/`S2`).
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
    detected;
  - `resourcesReleased()` reports a peripheral that `reset()` could not
    release; the engine then removes power and latches a fault;
  - `formatDiagnostics()` returns text from cached results only (Serial
    `i`), never touching a bus or pin.

### Adding a driver

1. Implement `class MyDriver final : public SensorDriver` in
   `src/drivers/my_driver.h/.cpp`. Arduino compiles `src/` recursively.
   Use relative includes such as `"../../sensor_driver.h"`.
2. Declare its topology, excitation and supply profile in
   `getRequirements()`. They decide how the engine prepares and schedules
   it.
3. In `src/drivers/driver_registry.cpp`, include its header, add
   `MyDriver myDriver;` beside the others, and add `&myDriver` to
   `registeredDrivers[]`. Earlier entries are tried first within the same
   topology, among drivers with an eligible supply. Drivers are static,
   never allocated. The current order is `Dht11Driver`, `I2cDriver`,
   `UltrasonicDriver`, `PassiveDriver`, which matches the engine's
   topology order.

No engine or display change is needed. Add tests with the mock in
`tests/` before trying the driver on hardware.

### Topology limits

Three topologies exist on the four-node socket:
- **`ThreeWirePowered`:** VCC/DATA/GND over 24 mappings, powered
  GND-first from GPIOs; the fourth node stays plain `INPUT`. Suits
  three-pin, single-data-line sensors that tolerate that policy.
- **`FourWirePowered`:** VCC/SIG1/SIG2/GND over 24 mappings, powered by a
  protected backend that reports its capabilities (or, for the 3.3 V I2C
  profile only, the experimental GPIO path). Suits four-pin, two-signal
  modules such as the HC-SR04 and 3.3 V I2C breakouts.
- **`TwoTerminal`:** a terminal pair plus two unused nodes, with
  weak-pull excitation only. Suits unpowered resistive parts, contacts
  and switches; it is never powered.

Not supported:
- **I2C model identification** or reading I2C sensor values: the scanner
  reports addresses and hints only.
- **Parts needing other excitation**, e.g. a current source, AC, or
  polarity-aware diode tests.

Such families need a new topology or excitation value and a matching
engine policy, not branches inside the state machine. No such driver is
implemented.

### Phase 5 changes to existing behaviour

- **Teardown order:** the engine now resets the driver first (while the
  attempt is still powered), then removes power and restores the pins.
  For DHT11, ultrasonic and passive attempts the pin sequence is
  effectively unchanged; it lets the I2C driver stop its controller
  before the supply goes.
- A driver that cannot release a peripheral latches *Socket rejected*
  with `err=release-failed` after power-off.
- `DriverRequirements::backendCapabilities` is replaced by an explicit
  `supply` profile plus `openDrainSignals`; `fourWireBackendConnect()`
  now names the supply voltage it is asked for.
- `ProbeStatus::hardwareRequired` (the first skipped driver's name) is
  replaced by `hardwareRequiredMask`, so every skipped driver is
  reported.
- Serial command `i` prints the detected driver's diagnostics.
- With an eligible supply, each I2C mapping adds about 0.56 s to a cycle;
  as shipped nothing changes (the scanner is skipped).

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
  `signalLabel` and `hardwareRequired` (a mask since Phase 5).
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

`tests/test_pindex.cpp` exercises the probe engine and all four driver
contracts against these simulations:
- `tests/mock/Arduino.h`: simulated GPIO, clock, critical sections and
  pin interrupts (`attachInterruptArg`/`detachInterrupt`; tests deliver
  edges at their own instants);
- a resistor-network solver, which drives both the precheck's digital reads
  and the simulated ADC;
- a simulated DHT11 and a simulated HC-SR04 (TRIG/ECHO timing, stuck
  levels, missing echoes, TRIG coupling);
- a **fake** protected four-wire backend that records every connection and
  its supply voltage, and flags any made without a break, with a GPIO
  driven, with an interrupt attached or without the capabilities. It
  exists only in the tests; the shipped backend provides nothing;
- a **simulated I2C bus and module** in place of `i2c_bus.cpp`: the
  module answers only while powered on its own VCC/GND with the
  controller on its own SDA/SCL; it can hold lines LOW, time out, fail a
  transaction, change its address set between sweeps, refuse a start or
  a stop, and flags two transactions in one loop pass, a transaction in a
  critical section, or supply removed while the controller still runs;
- fake drivers that exercise the architecture.

Coverage added for the pull-up exception includes:
- the classifier: 4.7k and 10k, internal pull-up spread, near-zero in
  either direction, noise, directional readings, open, out of band;
- the series check: two pull-ups versus another path, a short, or open;
- the policy on all 24 mappings, which evaluate their own physical pairs
  (only VCC A / GND B qualifies);
- every veto reason, including "measured" pairs with nothing classified;
- capability combinations: 5 V, push-pull, DHT11, and a fake driver with
  the I2C capabilities;
- through the engine:
  - a mocked 0x3C reaching the snapshot with the right roles, and SDA–SCL
    and VCC–SDA/SCL/GND shorts;
  - a rail problem the digital test missed;
  - ADC failure and unavailability, noise and asymmetry;
  - re-measurement on every retry, and cancellation mid-measurement;
  - the GPIO profile, and the disabled profile never measuring;
  - DHT11 and passive detection afterwards;
  - two global guards: no sensing ever starts with a supply on, and
    nothing goes push-pull while a pad is sensed.

The simulator can weaken its pull-downs (`net.pulldownOhms`) so the
digital precheck flags a pull-up network. With equal 45 kΩ pulls it reads
LOW, so whether real hardware flags one is itself unverified.

Precheck pacing and inconclusive skips: no sample before the settle time,
four sets per pull configuration at the sample interval, one set (never
a burst) after a stalled loop, pacing across a `millis()` rollover, one
pass plus exactly `PRECHECK_MAX_RETRIES` retries, a transient that
settles on a retry, a skipped candidate that is never powered followed
by a later one that is found with fresh observations and retry budget,
every candidate skipped in bounded time, a skipped last candidate ending
the cycle normally, clearing mid-retry, the VCC-GND veto after a skip,
and a saved candidate's inconsistent retry counting as a failed
acquisition.

Coverage added in Phase 5 includes: I2C role text on all 24 mappings,
the supply profiles (3.3 V vs 5 V backend, partial capabilities, the GPIO
profile off and on, the powered setting overriding it), the precheck
veto (a VCC-GND short is never powered for I2C), exactly 0x08–0x77, one
transaction per pass, NACK vs timeout/bus error, no/one/two/eleven ACKs,
all-ACK rejection, stuck SDA/SCL before and during a sweep, two-sweep
confirmation, unstable sets and the bounded retry, hints and unknown
addresses, liveness rotation with no bus traffic from the UI, loss of
response and recovery, cancellation and the sweep deadline across a
`millis()` rollover, the controller stopped before the supply (every
teardown), controller conflicts, start and stop failures, repeated
resets, I2C-to-DHT/passive/ultrasonic handoff, small buffers, I2C-before-
ultrasonic priority, and the teardown order for every driver (no reset
after power-off).

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
adapter, the four-wire backend and the I2C bus adapter, so it is linked
**without** `src/drivers/driver_registry.cpp`,
`src/drivers/passive_adc.cpp`, `src/drivers/i2c_bus.cpp` and
`four_wire_backend.cpp`. Shorts, faults, buses and parts are simulated
only; the real `i2c_bus.cpp` (the `Wire1` calls) is checked by compiling
it and against the core sources, not by these tests. The tests need a
host C++17 compiler (not the ESP32 toolchain). From the `PinDex` folder:

```text
SRC="tests/test_pindex.cpp probe_engine.cpp dht11.cpp src/drivers/dht11_driver.cpp src/drivers/passive_driver.cpp src/drivers/ultrasonic_driver.cpp src/drivers/i2c_driver.cpp src/drivers/weak_pull_meter.cpp"
F="-std=c++17 -Wall -Wextra -Itests/mock -I."
g++ $F -DPINDEX_POWERED_PROBING_ENABLED=1 -DPINDEX_PASSIVE_PROBING_ENABLED=1 -DPINDEX_I2C_GPIO_POWER_ENABLED=0 $SRC -o build/test_all
g++ $F -DPINDEX_POWERED_PROBING_ENABLED=1 -DPINDEX_PASSIVE_PROBING_ENABLED=1 -DPINDEX_I2C_GPIO_POWER_ENABLED=1 $SRC -o build/test_i2c_gpio
g++ $F -DPINDEX_POWERED_PROBING_ENABLED=0 -DPINDEX_PASSIVE_PROBING_ENABLED=1 -DPINDEX_I2C_GPIO_POWER_ENABLED=1 $SRC -o build/test_passive_only
g++ $F -DPINDEX_POWERED_PROBING_ENABLED=0 -DPINDEX_PASSIVE_PROBING_ENABLED=0 -DPINDEX_I2C_GPIO_POWER_ENABLED=0 $SRC -o build/test_disabled
```

Run all four binaries; each exits non-zero on any failure.

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
15. **Empty socket:** expect *Scanning n/54* (n/78 if the I2C scanner is also eligible), no `hw-required=[HC-SR04 Ultrasonic]`, every
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

### I2C scanner bench steps

Nothing here has been run on hardware. Do these only with an eligible
supply: a reviewed protected 3.3 V backend, or, for one evaluated module
at a time, the experimental GPIO profile. Never with a module that draws
more than the supply can give, has 5 V pull-ups, or is powered from
elsewhere.

20. **Default build:** with `PINDEX_I2C_GPIO_POWER_ENABLED 0` and no
    backend, confirm the boot line `I2C Device: hardware required ...`,
    `hw-required=[I2C Device]` on status lines, *Scanning n/30*, and with
    a meter that no probe ever goes HIGH during the four-wire part of the
    cycle (there is none).
21. **Evaluate the module first:** its supply current including LEDs,
    backlight and inrush, its regulator, its pull-ups (to its own VCC,
    never 5 V), and what each of the 24 wrong assignments would do to it.
    If any of these is unknown, stop.
22. **Empty socket with an eligible supply:** expect 24 I2C attempts per
    cycle, each `err=no-ack`. With a logic analyser on the SDA/SCL nodes
    confirm: only address-only writes to 0x08–0x77, each ending in STOP,
    100 kHz, nothing driven HIGH (open-drain), the controller stopped and
    both lines released **before** VCC is removed, and the supply off
    between attempts.
23. **Known module, mapping 4:** A = VCC, B = GND, C = SDA, D = SCL (for
    example a BME280 or MPU6050 breakout). Expect *I2C Device*,
    `Addr: 0x76 (ACK)` (or its real address), the matching hint ending in
    `?`, and `A:VCC B:GND` / `C:SDA D:SCL`; `i` prints the full address
    list. Compare with a known-good I2C scanner on the same module. If the
    precheck reports `coupled=AC AD` or similar and the mapping is
    skipped, the breakout's pull-ups are being seen as VCC coupling; that
    veto is intended.
24. **Multi-address module:** e.g. an RTC board with an EEPROM (0x50 and
    0x68): expect `Addr 1/2`, then `Addr 2/2` 2 s later, with no extra bus
    traffic between liveness checks.
25. **Stability:** leave it monitored for minutes; then pull it out. Expect
    *STALE - fail n/3*, retries with fresh power, then the result cleared.
26. **Faulted bus:** hold SDA LOW through a resistor of a few kΩ to the
    module's GND: expect `bus-stuck-low` and no further transactions on
    that attempt.
27. **Validate `I2C_POWER_STABILIZE_MS`** (100 ms) for the module: the
    first sweep after power-on must already return the full address set.
28. **Handoff:** repeat steps 4 and 8; DHT11 and passive results must be
    unchanged.

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
- **Probing disabled:** expected when powered and passive probing are both `0`.
- **No Component / Unknown with a DHT11 inserted:**
  - check the module supports about 3.3V and has a DATA pull-up;
  - the Serial `err=` field shows why each attempt failed (e.g.
    `no-response`, `checksum`, `unsupported-encoding` for a DHT22);
  - `coupled=` in the Serial log means the precheck blocked some
    mappings; a module's onboard pull-up can cause this.
- **`hw-required=[HC-SR04 Ultrasonic]`:** expected on this board. The
  ultrasonic driver needs a protected four-wire backend that is not
  fitted, so it is skipped; other detection is unaffected.
- **`hw-required=[I2C Device]`:** expected as shipped: no protected 3.3 V
  backend and the experimental GPIO profile is off. Other detection is
  unaffected.
- **I2C `err=no-ack` on every mapping (with an eligible supply):** no
  pull-ups (the internal ones are too weak), the module unpowered or
  needing more current than the supply gives, or its VCC mapping vetoed
  by the precheck (`coupled=`).
- **I2C `err=bus-stuck-low` / `bus-timeout`:** a line held LOW, missing
  pull-ups, or a device stretching the clock past 25 ms.
- **I2C `err=unstable-addresses`:** the address set changed between
  sweeps (loose contact, marginal supply or pull-ups).
- **`err=bus-conflict`:** the I2C controller (`Wire1`) is already in use
  elsewhere in the firmware; PinDex does not touch it.
- **Socket rejected with `err=release-failed`:** a driver could not
  release its peripheral (e.g. the I2C controller would not stop). Power
  was removed; send `r` to retry.
- **An HC-SR04 is inserted but shows *No Component / Unknown*:** expected
  without the backend. Remove it; do not power it from the probe GPIOs.
- **Ultrasonic `err=no-echo` / `echo-timeout` (with a backend):** no
  target in range, a soft or angled target, or the module was removed.
  These cannot be told apart.
- **`err=precheck-inconsistent`:** the precheck readings of one attempt
  kept changing through all three passes, so that attempt was skipped
  without power, logged as
  `precheck: attempt 9/54 inconsistent; retries exhausted; skipping`, and
  discovery went on with the next attempt. The status line counts the
  skips (`inconclusive=1/54`); a skipped attempt was never tested, so a
  final *No Component / Unknown* with skips is not proof that the socket
  is empty. Causes: loose contacts, or charge still decaying from the
  previous powered attempt (a breakout's capacitors). If the skips always
  land on the same attempts, the settle times in `config.h` may need
  tuning on that hardware.
- **`err=ambiguous-pairs`:** more than one of the six pairs conducts (a
  three-terminal part or a network). This is not supported as a passive
  result.
- **Measurement unavailable:** the passive ADC path failed (`err=` shows
  `adc-unavailable`, `adc-config-failed`, `adc-read-failed` or
  `pullup-unverified`).
- **A switch stays "OPEN" after you removed it:** expected. An open switch
  and an empty socket look the same, so send `c` to clear.

## Scope and safety limits

- With powered and passive probing both `0`, every probe pin is a plain input with
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
- I2C scanning runs only on a protected 3.3 V backend or, if explicitly
  enabled, the experimental GPIO profile, which is **not protection** and
  does not make arbitrary insertion safe. Never pull SDA/SCL up to 5 V.
- Passive probing never drives a probe HIGH, but its open-drain LOW sink
  is an unprotected GPIO. Use it only with unpowered parts.
- Do not connect powered circuits or charged capacitors to the probes, and
  never apply 5V to them.
- "Ready" means the firmware is running. Detection is limited to:
  - DHT11-compatible sensors (powered probing);
  - responding I2C addresses and their mapping, with tentative hints,
    only with an eligible 3.3 V supply (not enabled as shipped);
  - HC-SR04-compatible ultrasonic modules, only with a protected
    four-wire backend (not fitted);
  - approximate two-terminal resistive and contact behaviour (passive
    probing).

  None of these proves what a part is.
