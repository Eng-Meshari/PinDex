# PinDex

**Smart Component & Sensor Identifier**, an ESP32 instrument for electronics
labs. The goal is to let students insert an unknown component or sensor
module, have it identified, and get its pinout.

> **Status: v0.1 is a firmware scaffold.** It brings up the OLED, runs the
> non-blocking main loop and keeps the probes safe and inactive. It does
> **not** identify components or sensors yet.

## Project files

| File | Purpose |
|---|---|
| `PinDex.ino` | Sketch entry point. Startup order, splash/idle timing, Serial heartbeat. |
| `config.h` | All pin assignments, timings, the baud rate and the version string. |
| `display_manager.h/.cpp` | VSPI + SSD1306 setup, splash screen and idle screen. |
| `probe_engine.h/.cpp` | Probe pin safety (high impedance) and the inactive Idle/Fault state machine. |
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
signal terminals; a component can be inserted in any orientation.

**Strapping pins:** GPIO 2 and GPIO 5 affect how the ESP32 boots. Anything
attached to them must not force incompatible levels during reset (GPIO 2
must not be held high, GPIO 5 must not be held low). The OLED inputs are
fine. On some DevKits the on-board LED is on GPIO 2 and may flicker while
the display updates; that is normal.

## Software requirements

- [Arduino IDE](https://www.arduino.cc/en/software)
- **esp32** board package by Espressif Systems
- **Adafruit SSD1306** library
- **Adafruit GFX Library**
- Accept the required dependencies when prompted, including
  **Adafruit BusIO**.

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

## Expected behaviour

1. Serial prints a startup line with the version, noting that
   identification is not implemented.
2. The OLED shows a **PinDex** splash for about 1.5 s.
3. It then switches once to the idle screen:

   ```text
   ====================
          PinDex
       v0.1 Ready
   Insert Component...
   ====================
   ```

4. Serial prints a heartbeat about once per second, e.g.
   `heartbeat uptime=12s display=ok`.

"Ready" means the scaffold firmware is running. It does **not** mean
component identification is available.

If display initialisation fails, Serial prints one error and the heartbeat
continues with `display=failed`. A successful initialisation does not prove
a panel is connected: the SPI link is write-only, so the ESP32 cannot detect
a missing display.

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

## Scope and safety limits

- In v0.1 every probe pin is a plain input with internal pull resistors
  disabled (high impedance). This only means the firmware does not drive
  the probes. It is **not** protection against short circuits, overvoltage,
  ESD or charged capacitors.
- For initial bring-up, **leave the probe slot empty**.
- Do not connect powered circuits or charged capacitors to the probes, and
  never apply 5V to them.
- Active identification requires a designed protection and current-limiting
  front end before it is implemented.
