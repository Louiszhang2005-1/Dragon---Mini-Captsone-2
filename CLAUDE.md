# Dragon Robot — MEC2105 Projet intégrateur 2 (Polytechnique Montréal, Fall 2026)

Context file for Claude. Read this before changing code or wiring.

## Project
- Team course project: a dragon-themed robot that will be donated to Le Phare
  (pediatric palliative care) for children with limited fine motor control.
  Controls must stay simple, robust and safe.
- Constraints: mass ≤ 1.5 kg, budget ≤ 400 $.
- Mandatory competition actions: **drive the course, wing flapping, bubble
  expulsion, egg laying/opening**. Format: inspection → 2 attempts → best score.
- Team language: French and English (user is bilingual; code comments in English).

## Toolchain
- VS Code + PlatformIO. Board `esp32dev`, framework Arduino, `monitor_speed = 115200`.
- All code is in `src/main.cpp` (single file, functions defined before use,
  no Arduino prototype generation — keep it that way or add prototypes).
- Must compile on ESP32 Arduino core **2.x and 3.x**. PWM goes through the
  `pwmAttach()` / `pwmWrite()` helpers, which switch API with
  `ESP_ARDUINO_VERSION_MAJOR`. Never call `ledcSetup`/`ledcAttach` directly.
- No external libraries (servos are driven with LEDC directly, not ESP32Servo).
- Project lives in `Code/RP2_Dragon_Remote_PlatformIO/RP2_Dragon_Remote/`.
  `build_dir` points to `%USERPROFILE%/.pio-build/` (outside OneDrive: sync locks break builds).
- The dongle is an ESP32-C3 (USB `303A:1001`). Never upload to it. The robot's
  ESP32 DevKit shows up as a CP210x/CH340 USB device instead.

## Hardware
One ESP32 DevKit on the robot runs everything. No second microcontroller.

| Function | Part | ESP32 pins |
|---|---|---|
| Left wheels (front + rear in parallel on AO1/AO2, joined on breadboard) | Driver module 1, channel A (DRV8833-style, no PWM pin) | AIN1 19 / AIN2 18 · STBY 21 |
| Right wheels (front, rear) | Driver module 2 | FR: IN1 26 / IN2 27 · RR: IN1 33 / IN2 32 · STBY 25 |
| Left wing | MG90S servo | 13 |
| Right wing | MG90S servo (mirrored) | 14 |
| Egg | MG90S servo | 16 |
| Bubbles ("soap") | DC motor from an EagleStone 4×AA bubble gun, on driver module 1 channel B (BO1/BO2). No MOSFET available, so a driver channel is used instead. | BIN1 22 / BIN2 23 |

LEDC channels (core 2.x): wheel motors 0–1 and 4–7, bubble motor 2–3
(all 20 kHz, 8-bit), servos 8–10 (50 Hz, 14-bit). Channels share timers in pairs,
so do not put a different frequency on the partner of a used channel.

### Wireless remote (course "carte manette")
- Course-supplied set of 2 pre-programmed boards: handheld controller + dongle.
  **We do not program them.** Only the robot ESP32 gets our code.
- Dongle outputs: pressed = 0 V (LOW), released = 3.3 V (pulled up).
  Radio lost → all outputs return to released → robot stops (fail-safe).
- 5 buttons used:

| Dongle out | ESP32 pin | Button |
|---|---|---|
| 1 | 4 | Move left |
| 2 | 5 | Move right |
| 3 | 15 | Egg |
| 4 | 34 | Wings |
| 5 | 35 | Soap / bubbles |

GPIO 34/35 are input-only with no internal pull-up: they rely on the dongle's
pull-up; add 10 kΩ to 3V3 if testing with the dongle unplugged.
Avoid GPIO 12 (strapping pin, boot fails if pulled high), 0, 1, 3.

### Power
- 2× 1S LiPo 3.7 V 2000 mAh (with protection PCB), wired **in series = 7.4 V**
  (8.4 V full, ~6 V empty), main on/off switch.
- 7.4 V → both motor drivers' VM, and the bubble motor (through the MOSFET).
- 7.4 V → 5 V step-down (buck) converter ≥ 3 A → servos + ESP32 5V/VIN.
- All grounds common. Never power servos from the ESP32 3V3 pin.
- Wheel motors and bubble motor are ~6 V parts, so the code caps them:
  `MOTOR_MAX_DUTY = 180`, `SOAP_POWER_PCT = 75`.
- Cells have no balance lead: charge each cell separately with a 1S charger.

## Behaviour (current code)
- **Drive:** both move buttons = forward; left only = spin left (left wheels
  back, right wheels forward); right only = spin right. Front and rear wheels on
  a side always get the same command. `TURN_INNER_SPEED = 0` gives a wide arc
  instead of a spin. Speeds ramp over `RAMP_TIME_MS`. `COMBO_DELAY_MS` stops a
  brief turn when pressing both buttons. No reverse on the remote (only
  phone/serial have reverse).
- **Wings:** start down at rest (servo 90°), go 45° up (left 135°, right 45°,
  mirrored), back down, **3 cycles**, non-blocking state machine.
- **Egg:** each press toggles open (90°) / closed (0°), slow (60°/s).
- **Bubbles:** each press runs the motor `SOAP_RUN_MS` (1.5 s) with a 1 s
  cooldown. `SOAP_HOLD_TO_RUN = true` switches to run-while-held (max 5 s).
- Everything is non-blocking (loop every 5 ms); no `delay()` in the main loop.
  Keep it that way so driving still works while mechanisms run.

## Test interfaces (no buttons needed)
- **Serial Monitor 115200:** `f`/`b`/`l`/`r` drive 1 s, `x` stop,
  `e` egg, `w` wings, `s` bubbles, `?` status.
- **Wi-Fi phone page:** AP `DragonRobot`, password in `src/secrets.h` (gitignored; template `secrets.example.h`), http://192.168.4.1.
  Hold arrows to drive (auto-stop after 400 ms without a command), buttons for
  egg/wings/bubbles. `ENABLE_WIFI_CONTROL = false` disables it.
- **Live tuning (phone page, "Tuning" panel):** `params[]` table in main.cpp
  lists every tunable variable (min/max/step). Changes apply at once;
  "Save to robot" stores them in NVS (`Preferences`, namespace `dragon`) and
  they override the code defaults at every boot, even with Wi-Fi off and after
  re-upload. "Defaults" clears flash. `LOAD_SAVED_PARAMS = false` ignores flash.
  Serial `p` prints all params (`*` = differs from code). To make a new setting
  tunable: make it non-const and add a row to `params[]`.
- **Jumper test:** touching a button pin to GND = a press.

## Tuning checklist
- Wheel spins backward → `INVERT_<WHEEL> = true`. Wheel faster/slower → `TRIM_<WHEEL>`.
- Wing goes down instead of up → flip the sign of its 45 in `WING_*_FLAP`.
- Fit the servo horn after power-up, so 90° = wing down (rest).
- Servo buzzing at end of travel → narrow `SERVO_MIN_US` / `SERVO_MAX_US`.
- ESP32 resets when motors/servos start → power or ground problem, not code.

## Open items
- Egg and bubble mechanisms not yet physically tested; angles/timings are first guesses.
- Wheel motor rated voltage not confirmed (cap assumes ~6 V).
- Arcade buttons not received yet; testing via USB serial for now.

## Working rules for Claude
- Keep all user-tunable values in the settings blocks at the top of the file.
- Keep fail-safe: if nothing is pressed or the link is lost, motors stop.
- Don't add libraries or a second board unless asked.
- After changes, make sure it still builds (`pio run`).
