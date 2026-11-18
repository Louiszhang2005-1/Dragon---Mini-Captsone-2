#include <Arduino.h>

/*
  Dragon Robot — Remote-Controlled Main Sketch (ESP32 + wireless controller dongle)
  ---------------------------------------------------------------------------------
  One ESP32 runs everything: 4 drive motors, 2 wing servos, egg servo,
  and the DC motor from the soap gun.

  CONTROLLER (MEC2105 "carte manette")
    The dongle has 8 outputs (we use 5). For each output:
      pressed  -> 0 V   (LOW)
      released -> 3.3 V (HIGH, pulled up)
      radio lost -> every output returns to "released" -> robot stops = fail-safe
    Wire: dongle outputs -> ESP32 pins below, dongle GND -> ESP32 GND,
          dongle power   -> ESP32 3V3 (or 5V/VIN).

  BUTTONS (4 direction buttons + 1 action button)
    FORWARD held -> drive forward      BACK held  -> drive backward
    LEFT    held -> turn left          RIGHT held -> turn right
    (LEFT + RIGHT together also = forward)   nothing held -> stop
    ACTION button, counted taps (TAP_WINDOW_MS between taps):
      1 tap  -> egg opens / closes
      2 taps -> wings flap 3 cycles
      3 taps -> bubbles (soap-gun motor runs SOAP_RUN_MS)

  WIRING SUMMARY
    Drive (two DRV8833-style driver modules on the breadboard):
      On each module AIN1<->BIN1 and AIN2<->BIN2 are bridged, so both
      channels get the same command (more current per side).
      STBY of both modules is tied to ESP32 3V3 (always enabled).
      DRV 1 (left wheels):  AIN1 32, AIN2 33
                            AO1/AO2 = front-left, BO1/BO2 = rear-left
      DRV 2 (right wheels): AIN1 25, AIN2 26
                            AO1/AO2 = front-right, BO1/BO2 = rear-right
      VM -> battery + (after switch), GND -> common GND
    Servo signal wires:
      Left wing 13, Right wing 14, Egg 16
    Soap-gun motor (batteries removed from the gun, trigger switch bypassed):
      needs its own driver: a 3rd DRV8833 with AIN1 22, AIN2 23,
      STBY -> 3V3, VM -> battery +, GND -> common, motor on AO1/AO2.
      Nothing goes to the ESP32 directly.
      If it spins the wrong way (no bubbles), swap the 2 wires or set
      SOAP_REVERSE = true.
    Dongle outputs:
      OUT1 -> 4 (left),  OUT2 -> 5 (right), OUT3 -> 15 (forward),
      OUT4 -> 34 (back), OUT5 -> 35 (action: 1/2/3 taps)
      NOTE: pins 34/35 have no internal pull-up. The dongle provides one, so
      this is fine when it is plugged in. If they trigger by themselves with
      the dongle unplugged, add a 10 kΩ resistor from each pin to 3V3.

  POWER — 2x 1S LiPo (3.7 V 2000 mAh) wired in SERIES = 7.4 V (8.4 V full)
    Cell 1 (-) -> GND.  Cell 1 (+) -> Cell 2 (-).  Cell 2 (+) = 7.4 V rail.
    Add a main ON/OFF switch on the 7.4 V wire.
    7.4 V rail -> VM of every motor driver module (DRV8833 max 10.8 V: OK)
               -> 5 V step-down (buck) converter, >= 3 A
                    -> servo red wires (+5 V) AND ESP32 "5V/VIN" pin
    ALL grounds tied together: battery -, buck GND, ESP32 GND, driver GNDs,
    servo brown wires, dongle GND.
    The code caps motor power (MOTOR_MAX_DUTY, SOAP_POWER_PCT) so the
    6 V-rated wheel and bubble motors don't get the full 8.4 V.
    Never power MG90S servos from the ESP32 3V3 pin.
    CHARGING: separate the two cells and charge each with a 1S LiPo charger.
    Reconnect in series only when both are full (they have no balance lead).

  CONTROL WITHOUT THE BUTTONS
  1) PHONE (Wi-Fi): the robot creates its own Wi-Fi network.
       Connect your phone to  "DragonRobot"  (password: see src/secrets.h),
       open  http://192.168.4.1  in the browser.
       Hold the arrows to drive (robot stops when you let go or if Wi-Fi
       drops), tap Egg / Wings / Soap. Works at the same time as the dongle.
       Set ENABLE_WIFI_CONTROL = false to turn it off for the competition.
       "Tuning" panel (bottom of the page): change speeds, trims, angles,
       timings live. "Save to robot" keeps them in flash across reboots;
       "Defaults" goes back to the values in this file.
  2) SERIAL MONITOR (115200 baud, send one letter):
       f = forward 1 s, b = back 1 s, l = left 1 s, r = right 1 s, x = stop
       e = egg, w = wings, s = soap, ? = print status, p = print parameters
  3) DONGLE WITHOUT ARCADE BUTTONS: touch a jumper wire from a button
     connector on the controller board to its GND pin = a button press.

  No extra library needed (servos use the ESP32 LEDC hardware directly).
  Works on ESP32 Arduino core 2.x and 3.x. Board: "ESP32 Dev Module".
*/

// ===================== WI-FI TEST CONTROL =====================
const bool  ENABLE_WIFI_CONTROL = true;
const char* WIFI_NAME     = "DragonRobot";
// Password lives in src/secrets.h (not in git). Copy secrets.example.h to
// secrets.h to set it; without secrets.h the example placeholder is used.
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#warning "src/secrets.h missing: using placeholder Wi-Fi password from secrets.example.h"
#endif
const char* WIFI_PASSWORD = SECRET_WIFI_PASSWORD;
const unsigned long WEB_MOVE_TIMEOUT_MS = 400;  // stop if phone goes quiet

// Live tuning: most settings below can be changed from the phone page
// ("Tuning" panel). "Save" stores them in the ESP32 flash; they then
// REPLACE the values written below at every boot, even after re-uploading
// the code. Press "Defaults" on the page (or set this to false) to go back
// to the values in this file.
const bool LOAD_SAVED_PARAMS = true;

// ======================= DRIVE SETTINGS =======================
// Speed convention: -255 ... 0 ... +255  (+ = forward)
// Speeds are on the 0..255 scale, then limited by MOTOR_MAX_DUTY below.
int DRIVE_SPEED      = 255;   // both move buttons held -> straight
int TURN_OUTER_SPEED = 255;   // wheels on the outside of the turn
int TURN_INNER_SPEED = -255;  // inside wheels: -255 = spin in place, 0 = wide arc
int MOTOR_MAX_DUTY = 165;         // Battery is now 9.3 V: 165/255 × 9.3 V ≈ 6 V
                                 // max on the ~6 V wheel motors.
                                 // Formula: 255 × 6 / battery voltage.
                                 // (3.7 V battery -> 255, 7.4 V pair -> 180)
unsigned long RAMP_TIME_MS   = 300;        // time to go 0 -> full (0 = instant)
unsigned long COMBO_DELAY_MS = 80;         // grace time so pressing both buttons
                                           // doesn't cause a short turn first

// ======================= ACTION BUTTON (taps) =================
// Taps on the action button are counted. When no new tap comes within
// TAP_WINDOW_MS (and the button is released), the action runs:
// 1 tap = egg, 2 taps = wings, 3 taps (or more) = bubbles.
// Longer window = easier for slow tappers, but every action starts later.
unsigned long TAP_WINDOW_MS = 500;

// Calibration. Each driver module drives both wheels of its side with the
// same command, so trim / invert are per side. To flip only ONE wheel,
// swap that wheel's 2 motor wires on the breadboard.
float TRIM_LEFT   = 1.00;   // both left wheels  (DRV 1)
float TRIM_RIGHT  = 1.00;   // both right wheels (DRV 2)
bool  INVERT_LEFT  = false;
bool  INVERT_RIGHT = false;

// ======================= WING SETTINGS ========================
// Angles are servo angles (0..180). REST = start position, FLAP = flapped.
// The two servos are mirrored, so they move in opposite directions.
// NOTE: "LEFT"/"RIGHT" are the code names; they are swapped relative to
// the robot's physical sides.
// LEFT:  120 -> 8   -> 120
// RIGHT:  60 -> 180 -> 60  (mirror of LEFT)
float WING_LEFT_REST   = 120;
float WING_LEFT_FLAP   = 8;
float WING_RIGHT_REST  = 180 - 120;  // = 60
float WING_RIGHT_FLAP  = 180;
int   WING_CYCLES      = 3;
float WING_SPEED_DPS   = 110;  // degrees/second (MG90S max ≈ 500–600)
unsigned long WING_PAUSE_MS = 100;  // pause at top and bottom of each flap

// ======================= EGG SETTINGS =========================
// Assumed: one servo opens the egg. Each press toggles open/closed.
float EGG_CLOSED_ANGLE = 0;
float EGG_OPEN_ANGLE   = 70;
float EGG_SPEED_DPS    = 60;   // slow, smooth opening

// ======================= SOAP SETTINGS ========================
// The DC motor taken out of the soap gun, driven by a 3rd DRV8833
// module (AIN1 22 / AIN2 23, motor on AO1/AO2).
// Default: each press runs the motor for SOAP_RUN_MS (one dose).
bool          SOAP_REVERSE     = false; // true = spin the other way (no bubbles?)
unsigned long SOAP_RUN_MS      = 1500;  // dose length — tune by testing
bool          SOAP_HOLD_TO_RUN = false; // true = runs as long as button is held
unsigned long SOAP_MAX_MS      = 5000;  // safety limit in hold mode
unsigned long SOAP_COOLDOWN_MS = 1000;  // min. time between two doses
int           SOAP_POWER_PCT   = 65;    // 65% of 9.3 V ≈ 6 V = the gun's 4xAA.
                                        // Formula: 100 × 6 / battery voltage.

// ======================= SERVO PULSE ==========================
// MG90S: about 500 µs = 0°, 2400 µs = 180°. Narrow this if a servo buzzes
// at the ends of its travel.
int SERVO_MIN_US = 500;
int SERVO_MAX_US = 2400;
// ==============================================================


// ========================= PINS ===============================
const uint8_t PIN_BTN_MOVE_LEFT  = 4;
const uint8_t PIN_BTN_MOVE_RIGHT = 5;
const uint8_t PIN_BTN_FORWARD    = 15;
const uint8_t PIN_BTN_BACK       = 34;
const uint8_t PIN_BTN_ACTION     = 35;   // 1 / 2 / 3 taps

const uint8_t PIN_SERVO_WING_L = 13;
const uint8_t PIN_SERVO_WING_R = 14;
const uint8_t PIN_SERVO_EGG    = 16;
// Wheel drivers (STBY of both modules is wired to 3V3, not to a GPIO)
const uint8_t PIN_LEFT_IN1     = 32;   // DRV 1 AIN1 (bridged to BIN1)
const uint8_t PIN_LEFT_IN2     = 33;   // DRV 1 AIN2 (bridged to BIN2)
const uint8_t PIN_RIGHT_IN1    = 25;   // DRV 2 AIN1 (bridged to BIN1)
const uint8_t PIN_RIGHT_IN2    = 26;   // DRV 2 AIN2 (bridged to BIN2)

// Bubble motor = 3rd DRV8833 module (AIN1 / AIN2 -> AO1 / AO2)
const uint8_t PIN_SOAP_IN1     = 22;
const uint8_t PIN_SOAP_IN2     = 23;
const uint8_t SOAP_PWM_CH1     = 2;    // LEDC channels (core 2.x only),
const uint8_t SOAP_PWM_CH2     = 3;    // same 20 kHz as the wheel motors

const uint32_t MOTOR_PWM_FREQ = 20000;  // 20 kHz, no motor whine
const uint8_t  MOTOR_PWM_BITS = 8;      // duty 0..255
const uint32_t SERVO_PWM_FREQ = 50;     // standard servo frame (20 ms)
const uint8_t  SERVO_PWM_BITS = 14;
// ==============================================================


// --- PWM helpers: ESP32 Arduino core 2.x and 3.x ---
void pwmAttach(uint8_t pin, uint8_t ch, uint32_t freq, uint8_t bits) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, freq, bits);
#else
  ledcSetup(ch, freq, bits);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}


// ======================== BUTTONS =============================
struct Button {
  const char* name;
  uint8_t pin;
  bool stable;            // true = pressed (debounced)
  bool lastRaw;
  unsigned long lastChange;
  bool justPressed;       // true for one loop on a new press
};

enum { BTN_MOVE_LEFT, BTN_MOVE_RIGHT, BTN_FORWARD, BTN_BACK, BTN_ACTION, NUM_BTN };

Button buttons[NUM_BTN] = {
  {"Move Left",  PIN_BTN_MOVE_LEFT,  false, false, 0, false},
  {"Move Right", PIN_BTN_MOVE_RIGHT, false, false, 0, false},
  {"Forward",    PIN_BTN_FORWARD,    false, false, 0, false},
  {"Back",       PIN_BTN_BACK,       false, false, 0, false},
  {"Action",     PIN_BTN_ACTION,     false, false, 0, false},
};

const unsigned long DEBOUNCE_MS = 25;

void initButtons() {
  for (int i = 0; i < NUM_BTN; i++) {
    // GPIO 34-39 are input-only and have no internal pull-up.
    if (buttons[i].pin >= 34) pinMode(buttons[i].pin, INPUT);
    else                      pinMode(buttons[i].pin, INPUT_PULLUP);
  }
}

void updateButtons(unsigned long now) {
  for (int i = 0; i < NUM_BTN; i++) {
    Button &b = buttons[i];
    b.justPressed = false;
    bool raw = (digitalRead(b.pin) == LOW);   // LOW = pressed
    if (raw != b.lastRaw) { b.lastRaw = raw; b.lastChange = now; }
    if (raw != b.stable && now - b.lastChange >= DEBOUNCE_MS) {
      b.stable = raw;
      if (raw) b.justPressed = true;
      Serial.printf("[btn] %-10s %s\n", b.name, raw ? "PRESSED" : "released");
    }
  }
}


// ======================== SERVOS ==============================
struct ServoOut {
  const char* name;
  uint8_t pin;
  uint8_t ch;       // LEDC channel (core 2.x only); 8-11 so motors keep 0-7
  float current;    // degrees
  float target;     // degrees
  float speedDps;   // degrees per second
};

enum { SV_WING_L, SV_WING_R, SV_EGG, NUM_SERVO };

ServoOut servos[NUM_SERVO] = {
  {"Wing L", PIN_SERVO_WING_L,  8, WING_LEFT_REST,     WING_LEFT_REST,     WING_SPEED_DPS},
  {"Wing R", PIN_SERVO_WING_R,  9, WING_RIGHT_REST,    WING_RIGHT_REST,    WING_SPEED_DPS},
  {"Egg",    PIN_SERVO_EGG,    10, EGG_CLOSED_ANGLE, EGG_CLOSED_ANGLE, EGG_SPEED_DPS},
};

void servoWriteAngle(ServoOut &s, float angle) {
  angle = constrain(angle, 0.0f, 180.0f);
  float us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180.0f;
  uint32_t maxDuty = (1UL << SERVO_PWM_BITS) - 1;
  uint32_t duty = (uint32_t)(us * maxDuty / (1000000.0f / SERVO_PWM_FREQ));
  pwmWrite(s.pin, s.ch, duty);
}

void initServos() {
  for (int i = 0; i < NUM_SERVO; i++) {
    pwmAttach(servos[i].pin, servos[i].ch, SERVO_PWM_FREQ, SERVO_PWM_BITS);
    servoWriteAngle(servos[i], servos[i].current);
  }
}

bool servoAtTarget(const ServoOut &s) { return s.current == s.target; }

// Move every servo a small step toward its target (smooth, non-blocking)
void updateServos(float dt) {
  for (int i = 0; i < NUM_SERVO; i++) {
    ServoOut &s = servos[i];
    if (servoAtTarget(s)) continue;
    float step = s.speedDps * dt;
    float diff = s.target - s.current;
    if (fabs(diff) <= step) s.current = s.target;
    else                    s.current += (diff > 0 ? step : -step);
    servoWriteAngle(s, s.current);
  }
}


// ===================== WINGS (state machine) ==================
enum WingState { WING_IDLE, WING_GOING_OUT, WING_PAUSE_OUT, WING_GOING_BACK, WING_PAUSE_REST };
WingState wingState = WING_IDLE;
int wingCycle = 0;
unsigned long wingTimer = 0;

void wingsStart() {
  if (wingState != WING_IDLE) return;   // ignore presses while flapping
  wingCycle = 0;
  servos[SV_WING_L].target = WING_LEFT_FLAP;
  servos[SV_WING_R].target = WING_RIGHT_FLAP;
  wingState = WING_GOING_OUT;
  Serial.println("[wings] flapping");
}

void updateWings(unsigned long now) {
  bool bothThere = servoAtTarget(servos[SV_WING_L]) && servoAtTarget(servos[SV_WING_R]);
  switch (wingState) {
    case WING_IDLE: break;
    case WING_GOING_OUT:
      if (bothThere) { wingTimer = now; wingState = WING_PAUSE_OUT; }
      break;
    case WING_PAUSE_OUT:
      if (now - wingTimer >= WING_PAUSE_MS) {
        servos[SV_WING_L].target = WING_LEFT_REST;
        servos[SV_WING_R].target = WING_RIGHT_REST;
        wingState = WING_GOING_BACK;
      }
      break;
    case WING_GOING_BACK:
      if (bothThere) {
        wingCycle++;
        Serial.printf("[wings] cycle %d/%d\n", wingCycle, WING_CYCLES);
        if (wingCycle >= WING_CYCLES) { wingState = WING_IDLE; Serial.println("[wings] done"); }
        else { wingTimer = now; wingState = WING_PAUSE_REST; }
      }
      break;
    case WING_PAUSE_REST:
      if (now - wingTimer >= WING_PAUSE_MS) {
        servos[SV_WING_L].target = WING_LEFT_FLAP;
        servos[SV_WING_R].target = WING_RIGHT_FLAP;
        wingState = WING_GOING_OUT;
      }
      break;
  }
}


// ======================== EGG =================================
bool eggOpen = false;

void eggToggle() {
  eggOpen = !eggOpen;
  servos[SV_EGG].target = eggOpen ? EGG_OPEN_ANGLE : EGG_CLOSED_ANGLE;
  Serial.printf("[egg] %s\n", eggOpen ? "opening" : "closing");
}


// ================ SOAP (motor from the soap gun) ==============
// ESP32 22/23 -> 3rd driver module AIN1/AIN2 -> AO1/AO2 -> gun's DC motor.
bool soapRunning = false;
unsigned long soapStartedAt = 0;
unsigned long soapLastEnded = 0;

void soapMotor(bool on) {
  if (!on) {                                   // off = coast (both LOW)
    pwmWrite(PIN_SOAP_IN1, SOAP_PWM_CH1, 0);
    pwmWrite(PIN_SOAP_IN2, SOAP_PWM_CH2, 0);
    return;
  }
  // Same slow-decay drive as the wheels: one input HIGH, PWM on the other
  uint32_t duty = (uint32_t)(255 * constrain(SOAP_POWER_PCT, 0, 100) / 100);
  if (!SOAP_REVERSE) {
    pwmWrite(PIN_SOAP_IN1, SOAP_PWM_CH1, 255);
    pwmWrite(PIN_SOAP_IN2, SOAP_PWM_CH2, 255 - duty);
  } else {
    pwmWrite(PIN_SOAP_IN1, SOAP_PWM_CH1, 255 - duty);
    pwmWrite(PIN_SOAP_IN2, SOAP_PWM_CH2, 255);
  }
}

void initSoap() {
  pwmAttach(PIN_SOAP_IN1, SOAP_PWM_CH1, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  pwmAttach(PIN_SOAP_IN2, SOAP_PWM_CH2, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  soapMotor(false);
}

void soapStart() {
  if (soapRunning) return;
  if (millis() - soapLastEnded < SOAP_COOLDOWN_MS && soapLastEnded != 0) return;
  soapRunning = true;
  soapStartedAt = millis();
  soapMotor(true);
  Serial.println("[soap] motor ON");
}

void updateSoap(unsigned long now) {
  if (!soapRunning) return;
  bool stop;
  if (SOAP_HOLD_TO_RUN) {
    // Runs while the button is held (or SOAP_RUN_MS for a serial 's' test),
    // but never longer than SOAP_MAX_MS.
    bool held = buttons[BTN_ACTION].stable;
    stop = (!held && now - soapStartedAt >= SOAP_RUN_MS) || (now - soapStartedAt >= SOAP_MAX_MS);
  } else {
    stop = (now - soapStartedAt >= SOAP_RUN_MS);   // one fixed dose per press
  }
  if (stop) {
    soapMotor(false);
    soapRunning = false;
    soapLastEnded = now;
    Serial.println("[soap] motor OFF");
  }
}


// ================== ACTION BUTTON (tap counter) ===============
// Counts taps on the action button. The action runs once the button is
// released and no new tap arrived for TAP_WINDOW_MS.
int tapCount = 0;
unsigned long lastTapAt = 0;

void runTapAction(int taps) {
  Serial.printf("[taps] %d -> ", taps);
  if (taps == 1)      { Serial.println("egg");     eggToggle(); }
  else if (taps == 2) { Serial.println("wings");   wingsStart(); }
  else                { Serial.println("bubbles"); soapStart(); }  // 3 or more
}

void updateTaps(unsigned long now) {
  Button &b = buttons[BTN_ACTION];
  if (b.justPressed) { tapCount++; lastTapAt = now; }
  if (tapCount > 0 && !b.stable && now - lastTapAt >= TAP_WINDOW_MS) {
    runTapAction(tapCount);
    tapCount = 0;
  }
}


// ======================== DRIVE ===============================
struct Motor {
  const char* name;
  uint8_t in1, in2;
  uint8_t ch1, ch2;   // LEDC channels (core 2.x only)
  float trim;
  bool  invert;
};

// One entry per side: each drives a whole DRV8833 (A and B bridged).
// LEDC channels 2/3 are used by the bubble motor.
enum { M_LEFT, M_RIGHT, NUM_MOTORS };
Motor motors[NUM_MOTORS] = {
  {"Left",  PIN_LEFT_IN1,  PIN_LEFT_IN2,  0, 1, 1.0, false},  // DRV 1
  {"Right", PIN_RIGHT_IN1, PIN_RIGHT_IN2, 4, 5, 1.0, false},  // DRV 2
};

// Slow-decay drive, same as RP2_Motor_code. value -255..255, 0 = brake.
void setMotor(Motor &m, int value) {
  value = (int)(value * m.trim);
  value = constrain(value, -MOTOR_MAX_DUTY, MOTOR_MAX_DUTY);
  if (m.invert) value = -value;
  if (value > 0) {
    pwmWrite(m.in1, m.ch1, 255);
    pwmWrite(m.in2, m.ch2, 255 - value);
  } else if (value < 0) {
    pwmWrite(m.in1, m.ch1, 255 + value);
    pwmWrite(m.in2, m.ch2, 255);
  } else {
    pwmWrite(m.in1, m.ch1, 255);
    pwmWrite(m.in2, m.ch2, 255);
  }
}

void coastMotor(const Motor &m) {
  pwmWrite(m.in1, m.ch1, 0);
  pwmWrite(m.in2, m.ch2, 0);
}

void initMotors() {
  // trim / invert are copied into motors[] by syncParams()
  for (int i = 0; i < NUM_MOTORS; i++) {
    pwmAttach(motors[i].in1, motors[i].ch1, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    pwmAttach(motors[i].in2, motors[i].ch2, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    coastMotor(motors[i]);
  }
  // STBY is hard-wired to 3V3 on both modules: nothing to do here.
}

float curLeft = 0, curRight = 0;          // current (ramped) side speeds
int   tgtLeft = 0, tgtRight = 0;          // requested side speeds
int   lastMoveCombo = 0;                  // 0 none, 1 left, 2 right, 3 both
unsigned long moveComboSince = 0;

// Serial test: simulated movement
char simMove = 0;
unsigned long simUntil = 0;

float approach(float cur, float tgt, float maxStep) {
  if (fabs(tgt - cur) <= maxStep) return tgt;
  return cur + (tgt > cur ? maxStep : -maxStep);
}

void updateDrive(unsigned long now, float dt) {
  bool L = buttons[BTN_MOVE_LEFT].stable;
  bool R = buttons[BTN_MOVE_RIGHT].stable;
  bool F = buttons[BTN_FORWARD].stable;
  bool back = buttons[BTN_BACK].stable;

  if (simMove && now < simUntil) {
    L = (simMove == 'f' || simMove == 'l');
    R = (simMove == 'f' || simMove == 'r');
    F = false;
    back = (simMove == 'b');
  } else simMove = 0;

  // Forward + back together cancel out -> stop (safe default)
  int combo;
  if (F && back)  combo = 0;
  else if (F)     combo = 3;
  else if (back)  combo = 4;
  else            combo = (L ? 1 : 0) | (R ? 2 : 0);
  if (combo != lastMoveCombo) { lastMoveCombo = combo; moveComboSince = now; }

  // A single button just pressed might be the start of a two-button press:
  // wait COMBO_DELAY_MS before turning, and keep the previous command meanwhile.
  bool waitingForCombo = (combo == 1 || combo == 2) && (now - moveComboSince < COMBO_DELAY_MS);

  if (!waitingForCombo) {
    switch (combo) {
      case 3: tgtLeft = DRIVE_SPEED;      tgtRight = DRIVE_SPEED;      break; // forward
      case 4: tgtLeft = -DRIVE_SPEED;     tgtRight = -DRIVE_SPEED;     break; // back
      case 1: tgtLeft = TURN_INNER_SPEED; tgtRight = TURN_OUTER_SPEED; break; // turn left
      case 2: tgtLeft = TURN_OUTER_SPEED; tgtRight = TURN_INNER_SPEED; break; // turn right
      default: tgtLeft = 0;               tgtRight = 0;                break; // stop
    }
  }

  float maxStep = (RAMP_TIME_MS == 0) ? 1000.0f : 255.0f * (dt * 1000.0f) / RAMP_TIME_MS;
  curLeft  = approach(curLeft,  tgtLeft,  maxStep);
  curRight = approach(curRight, tgtRight, maxStep);

  setMotor(motors[M_LEFT],  (int)curLeft);    // both left wheels
  setMotor(motors[M_RIGHT], (int)curRight);   // both right wheels
}


// ================== LIVE-TUNABLE PARAMETERS ===================
// Table of every setting the phone page can change. To add one: make the
// variable non-const in the settings block, then add a line here
// (key max 15 chars, used for flash storage).
#include <Preferences.h>

enum ParamType { P_INT, P_ULONG, P_FLOAT, P_BOOL };

struct Param {
  const char* key;     // short id (flash key + web id)
  const char* label;   // shown on the phone
  const char* group;
  ParamType   type;
  void*       ptr;
  float mn, mx, step;
  float def;           // value from the code, captured at boot
};

Param params[] = {
  // key       label                      group     type     variable              min   max    step
  {"drv",    "Forward speed",            "Drive", P_INT,   &DRIVE_SPEED,         0,    255,   1},
  {"tOut",   "Turn outer speed",         "Drive", P_INT,   &TURN_OUTER_SPEED,   -255,  255,   1},
  {"tIn",    "Turn inner (-255 = spin)", "Drive", P_INT,   &TURN_INNER_SPEED,   -255,  255,   1},
  {"maxDuty","Motor max duty (180 = 6 V)","Drive",P_INT,   &MOTOR_MAX_DUTY,      0,    255,   1},
  {"ramp",   "Ramp time (ms)",           "Drive", P_ULONG, &RAMP_TIME_MS,        0,    2000,  10},
  {"combo",  "Combo delay (ms)",         "Drive", P_ULONG, &COMBO_DELAY_MS,      0,    300,   5},
  {"tapWin", "Tap window (ms)",          "Action",P_ULONG, &TAP_WINDOW_MS,       200,  1500,  50},
  {"trL",    "Trim left wheels",         "Wheels",P_FLOAT, &TRIM_LEFT,           0,    1.5,   0.01},
  {"trR",    "Trim right wheels",        "Wheels",P_FLOAT, &TRIM_RIGHT,          0,    1.5,   0.01},
  {"invL",   "Invert left wheels",       "Wheels",P_BOOL,  &INVERT_LEFT,         0,    1,     1},
  {"invR",   "Invert right wheels",      "Wheels",P_BOOL,  &INVERT_RIGHT,        0,    1,     1},
  {"wLr",    "Left wing rest (deg)",     "Wings", P_FLOAT, &WING_LEFT_REST,      0,    180,   1},
  {"wLf",    "Left wing up (deg)",       "Wings", P_FLOAT, &WING_LEFT_FLAP,      0,    180,   1},
  {"wRr",    "Right wing rest (deg)",    "Wings", P_FLOAT, &WING_RIGHT_REST,     0,    180,   1},
  {"wRf",    "Right wing up (deg)",      "Wings", P_FLOAT, &WING_RIGHT_FLAP,     0,    180,   1},
  {"wCyc",   "Flap cycles",              "Wings", P_INT,   &WING_CYCLES,         1,    10,    1},
  {"wSpd",   "Wing speed (deg/s)",       "Wings", P_FLOAT, &WING_SPEED_DPS,      10,   600,   10},
  {"wPau",   "Pause top/bottom (ms)",    "Wings", P_ULONG, &WING_PAUSE_MS,       0,    1000,  10},
  {"eCl",    "Egg closed (deg)",         "Egg",   P_FLOAT, &EGG_CLOSED_ANGLE,    0,    180,   1},
  {"eOp",    "Egg open (deg)",           "Egg",   P_FLOAT, &EGG_OPEN_ANGLE,      0,    180,   1},
  {"eSpd",   "Egg speed (deg/s)",        "Egg",   P_FLOAT, &EGG_SPEED_DPS,       10,   600,   5},
  {"sRun",   "Dose length (ms)",         "Bubbles",P_ULONG,&SOAP_RUN_MS,         100,  5000,  100},
  {"sHold",  "Run while held",           "Bubbles",P_BOOL, &SOAP_HOLD_TO_RUN,    0,    1,     1},
  {"sMax",   "Max run when held (ms)",   "Bubbles",P_ULONG,&SOAP_MAX_MS,         500,  10000, 100},
  {"sCool",  "Cooldown (ms)",            "Bubbles",P_ULONG,&SOAP_COOLDOWN_MS,    0,    5000,  100},
  {"sPow",   "Motor power (%)",          "Bubbles",P_INT,  &SOAP_POWER_PCT,      0,    100,   1},
  {"sRev",   "Reverse direction",        "Bubbles",P_BOOL, &SOAP_REVERSE,        0,    1,     1},
  {"svMin",  "Servo pulse at 0 deg (us)","Servos",P_INT,   &SERVO_MIN_US,        400,  1500,  10},
  {"svMax",  "Servo pulse at 180 deg (us)","Servos",P_INT, &SERVO_MAX_US,        1500, 2600,  10},
};
const int NUM_PARAMS = sizeof(params) / sizeof(params[0]);

Preferences prefs;
const char* PREFS_NS = "dragon";

float paramGet(const Param &p) {
  switch (p.type) {
    case P_INT:   return *(int*)p.ptr;
    case P_ULONG: return *(unsigned long*)p.ptr;
    case P_FLOAT: return *(float*)p.ptr;
    case P_BOOL:  return *(bool*)p.ptr ? 1 : 0;
  }
  return 0;
}

void paramSet(Param &p, float v) {
  v = constrain(v, p.mn, p.mx);
  switch (p.type) {
    case P_INT:   *(int*)p.ptr = lroundf(v); break;
    case P_ULONG: *(unsigned long*)p.ptr = (unsigned long)lroundf(v); break;
    case P_FLOAT: *(float*)p.ptr = v; break;
    case P_BOOL:  *(bool*)p.ptr = (v >= 0.5f); break;
  }
}

Param* findParam(const String &key) {
  for (int i = 0; i < NUM_PARAMS; i++)
    if (key == params[i].key) return &params[i];
  return nullptr;
}

// Copy the setting variables into the structs that use them.
// Safe to call any time; never starts a motor.
void syncParams() {
  if (SERVO_MIN_US >= SERVO_MAX_US - 100) SERVO_MIN_US = SERVO_MAX_US - 100;
  motors[M_LEFT].trim  = TRIM_LEFT;  motors[M_LEFT].invert  = INVERT_LEFT;
  motors[M_RIGHT].trim = TRIM_RIGHT; motors[M_RIGHT].invert = INVERT_RIGHT;
  servos[SV_WING_L].speedDps = WING_SPEED_DPS;
  servos[SV_WING_R].speedDps = WING_SPEED_DPS;
  servos[SV_EGG].speedDps    = EGG_SPEED_DPS;
  if (wingState == WING_IDLE) {        // new rest angle -> wings move there
    servos[SV_WING_L].target = WING_LEFT_REST;
    servos[SV_WING_R].target = WING_RIGHT_REST;
  }
  servos[SV_EGG].target = eggOpen ? EGG_OPEN_ANGLE : EGG_CLOSED_ANGLE;
}

// Called after a change from the phone, once the hardware is running.
void applyParamsLive() {
  syncParams();
  for (int i = 0; i < NUM_SERVO; i++) servoWriteAngle(servos[i], servos[i].current);
  if (soapRunning) soapMotor(true);    // new power % takes effect now
}

void loadParams() {
  for (int i = 0; i < NUM_PARAMS; i++) params[i].def = paramGet(params[i]);
  if (!LOAD_SAVED_PARAMS) return;
  int n = 0;
  prefs.begin(PREFS_NS, true);
  for (int i = 0; i < NUM_PARAMS; i++)
    if (prefs.isKey(params[i].key)) { paramSet(params[i], prefs.getFloat(params[i].key)); n++; }
  prefs.end();
  if (n) Serial.printf("[params] %d saved value(s) loaded from flash\n", n);
}

void saveParams() {
  prefs.begin(PREFS_NS, false);
  for (int i = 0; i < NUM_PARAMS; i++) prefs.putFloat(params[i].key, paramGet(params[i]));
  prefs.end();
  Serial.println("[params] saved to flash");
}

void resetParams() {
  prefs.begin(PREFS_NS, false);
  prefs.clear();
  prefs.end();
  for (int i = 0; i < NUM_PARAMS; i++) paramSet(params[i], params[i].def);
  Serial.println("[params] back to code defaults (flash cleared)");
}

void printParams() {
  Serial.println("---- parameters (* = changed from code) ----");
  for (int i = 0; i < NUM_PARAMS; i++) {
    float v = paramGet(params[i]);
    Serial.printf("%c %-8s %-28s %g\n", v != params[i].def ? '*' : ' ',
                  params[i].key, params[i].label, v);
  }
}


// ===================== SERIAL TEST MODE =======================
void printStatus() {
  Serial.println("---- status ----");
  for (int i = 0; i < NUM_BTN; i++)
    Serial.printf("%-10s (GPIO %2d): %s\n", buttons[i].name, buttons[i].pin,
                  buttons[i].stable ? "PRESSED" : "released");
  Serial.printf("Drive L/R: %d / %d\n", (int)curLeft, (int)curRight);
  Serial.printf("Soap motor: %s\n", soapRunning ? "ON" : "off");
  for (int i = 0; i < NUM_SERVO; i++)
    Serial.printf("%-7s servo: %.0f deg\n", servos[i].name, servos[i].current);
}

void handleSerial(unsigned long now) {
  while (Serial.available()) {
    char c = tolower(Serial.read());
    switch (c) {
      case 'f': case 'b': case 'l': case 'r': simMove = c; simUntil = now + 1000; break;
      case 'x': simMove = 0; break;
      case 'e': eggToggle();  break;
      case 'w': wingsStart(); break;
      case 's': soapStart();  break;
      case '?': printStatus(); break;
      case 'p': printParams(); break;
      default: break;
    }
  }
}


// ===================== WI-FI PHONE CONTROL ====================
#include <WiFi.h>
#include <WebServer.h>

WebServer web(80);

const char PAGE[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Dragon Robot</title><style>
body{font-family:sans-serif;background:#111;color:#eee;text-align:center;margin:0;padding:12px;
 -webkit-user-select:none;user-select:none;touch-action:manipulation}
h2{margin:6px 0 14px}
.pad{display:grid;grid-template-columns:repeat(3,90px);gap:10px;justify-content:center}
button{font-size:22px;height:80px;border:0;border-radius:14px;background:#2d6cdf;color:#fff}
button:active,button.on{background:#f39c12}
.act{display:grid;grid-template-columns:1fr;gap:12px;max-width:290px;margin:22px auto}
.act button{background:#27ae60}
#st{margin-top:12px;font-size:14px;color:#aaa;white-space:pre}
details{max-width:420px;margin:20px auto;text-align:left;background:#1c1c1c;border-radius:12px;padding:8px 12px}
summary{font-size:20px;padding:6px 0;cursor:pointer}
h3{margin:14px 0 4px;color:#f39c12;font-size:16px}
.row{display:flex;align-items:center;gap:8px;margin:8px 0;font-size:14px}
.row span{flex:1}
.row span.chg{color:#f39c12}
.row input[type=range]{flex:1.2}
.row input[type=number]{width:68px;font-size:15px;background:#222;color:#eee;border:1px solid #444;border-radius:6px;padding:4px}
.row input[type=checkbox]{width:26px;height:26px}
input{-webkit-user-select:auto;user-select:auto}
.tb{display:flex;gap:10px;margin:14px 0 6px}
.tb button{flex:1;height:48px;font-size:16px}
#msg{font-size:14px;color:#aaa;min-height:18px}
</style></head><body>
<h2>Dragon Robot</h2>
<div class="pad">
 <span></span><button data-m="f">&#9650;</button><span></span>
 <button data-m="l">&#9664;</button><button id="stop">&#9632;</button><button data-m="r">&#9654;</button>
 <span></span><button data-m="b">&#9660;</button><span></span>
</div>
<div class="act">
 <button onclick="cmd('e')">Egg (open / close)</button>
 <button onclick="cmd('w')">Wings (3 flaps)</button>
 <button onclick="cmd('s')">Soap / bubbles</button>
</div>
<div id="st">...</div>
<details id="tune"><summary>Tuning</summary>
<div id="pp">loading...</div>
<div class="tb"><button onclick="saveP()">Save to robot</button><button onclick="resetP()" style="background:#c0392b">Defaults</button></div>
<div id="msg">Changes apply at once. Save keeps them after reboot.</div>
</details>
<script>
const $=id=>document.getElementById(id);
let P=[];
function msg(t){$('msg').textContent=t;}
function loadP(){fetch('/params').then(r=>r.json()).then(a=>{P=a;drawP();}).catch(()=>$('pp').textContent='not connected');}
function drawP(){let h='',g='';P.forEach((p,i)=>{
 if(p.g!=g){g=p.g;h+='<h3>'+g+'</h3>';}
 h+='<div class="row"><span id="l'+i+'">'+p.n+'</span>';
 if(p.t==3)h+='<input type="checkbox" id="c'+i+'" onchange="setP('+i+',this.checked?1:0)">';
 else{const a=' min="'+p.mn+'" max="'+p.mx+'" step="'+p.s+'"';
  h+='<input type="range" id="r'+i+'"'+a+' oninput="$(\'n'+i+'\').value=this.value" onchange="setP('+i+',this.value)">'
   +'<input type="number" id="n'+i+'"'+a+' onchange="setP('+i+',this.value)">';}
 h+='</div>';});
 $('pp').innerHTML=h;P.forEach((p,i)=>showP(i));}
function showP(i){const p=P[i];
 if(p.t==3)$('c'+i).checked=p.v>0.5;else{$('r'+i).value=p.v;$('n'+i).value=p.v;}
 $('l'+i).className=(Math.abs(p.v-p.d)>1e-4)?'chg':'';}
function setP(i,v){fetch('/set?k='+P[i].k+'&v='+v).then(r=>r.text()).then(t=>{P[i].v=parseFloat(t);showP(i);msg('Applied (not saved yet)');}).catch(()=>{showP(i);msg('Failed - not connected');});}
function saveP(){fetch('/save').then(()=>msg('Saved to robot flash')).catch(()=>msg('Save failed'));}
function resetP(){if(!confirm('Reset all to code defaults and clear saved values?'))return;
 fetch('/reset').then(()=>{loadP();msg('Back to code defaults');}).catch(()=>msg('Reset failed'));}
$('tune').addEventListener('toggle',()=>{if($('tune').open)loadP();});

function cmd(c){return fetch('/cmd?c='+c).catch(()=>{});}
let timer=null;
function start(b){stop();b.classList.add('on');const m=b.dataset.m;cmd(m);timer=setInterval(()=>cmd(m),150);}
function stop(){if(timer){clearInterval(timer);timer=null;}
 document.querySelectorAll('.on').forEach(e=>e.classList.remove('on'));cmd('x');}
document.querySelectorAll('[data-m]').forEach(b=>{
 b.addEventListener('pointerdown',e=>{e.preventDefault();start(b);});
 ['pointerup','pointerleave','pointercancel'].forEach(ev=>b.addEventListener(ev,()=>{if(timer)stop();}));
});
document.getElementById('stop').onclick=stop;
document.addEventListener('visibilitychange',()=>{if(document.hidden)stop();});
setInterval(()=>fetch('/status').then(r=>r.text()).then(t=>st.textContent=t).catch(()=>st.textContent='not connected'),700);
</script></body></html>)HTML";

void handleRoot() { web.send_P(200, "text/html", PAGE); }

void handleCmd() {
  String c = web.arg("c");
  unsigned long now = millis();
  if (c.length() == 1) {
    switch (c[0]) {
      case 'f': case 'b': case 'l': case 'r':
        simMove = c[0]; simUntil = now + WEB_MOVE_TIMEOUT_MS; break;
      case 'x': simMove = 0; break;
      case 'e': eggToggle();  break;
      case 'w': wingsStart(); break;
      case 's': soapStart();  break;
    }
  }
  web.send(200, "text/plain", "ok");
}

void handleStatus() {
  char buf[160];
  snprintf(buf, sizeof(buf), "Drive L/R: %d / %d\nEgg: %s   Wings: %s   Soap: %s",
           (int)curLeft, (int)curRight, eggOpen ? "open" : "closed",
           wingState == WING_IDLE ? "idle" : "flapping", soapRunning ? "ON" : "off");
  web.send(200, "text/plain", buf);
}

// List of parameters as JSON: [{k,n,g,t,v,mn,mx,s,d}, ...]
void handleParams() {
  String j = "[";
  for (int i = 0; i < NUM_PARAMS; i++) {
    const Param &p = params[i];
    if (i) j += ',';
    j += "{\"k\":\"";  j += p.key;
    j += "\",\"n\":\""; j += p.label;
    j += "\",\"g\":\""; j += p.group;
    j += "\",\"t\":";   j += (int)p.type;
    j += ",\"v\":";     j += String(paramGet(p), 3);
    j += ",\"mn\":";    j += String(p.mn, 3);
    j += ",\"mx\":";    j += String(p.mx, 3);
    j += ",\"s\":";     j += String(p.step, 3);
    j += ",\"d\":";     j += String(p.def, 3);
    j += '}';
  }
  j += ']';
  web.send(200, "application/json", j);
}

// /set?k=<key>&v=<value>  -> replies with the value actually applied
void handleSet() {
  Param* p = findParam(web.arg("k"));
  if (!p || !web.hasArg("v")) { web.send(404, "text/plain", "unknown"); return; }
  paramSet(*p, web.arg("v").toFloat());
  applyParamsLive();
  float v = paramGet(*p);
  Serial.printf("[params] %s = %g\n", p->key, v);
  web.send(200, "text/plain", String(v, 3));
}

void handleSave()  { saveParams(); web.send(200, "text/plain", "saved"); }
void handleReset() { resetParams(); applyParamsLive(); web.send(200, "text/plain", "reset"); }

void initWifi() {
  if (!ENABLE_WIFI_CONTROL) return;
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_NAME, WIFI_PASSWORD);
  WiFi.setSleep(false);
  web.on("/", handleRoot);
  web.on("/cmd", handleCmd);
  web.on("/status", handleStatus);
  web.on("/params", handleParams);
  web.on("/set", handleSet);
  web.on("/save", handleSave);
  web.on("/reset", handleReset);
  web.begin();
  Serial.printf("Wi-Fi \"%s\" ready -> open http://%s\n", WIFI_NAME,
                WiFi.softAPIP().toString().c_str());
}


// ========================= MAIN ===============================
unsigned long lastTick = 0;

void setup() {
  Serial.begin(115200);
  delay(200);
  loadParams();                        // saved phone tuning (if any)
  syncParams();
  for (int i = 0; i < NUM_SERVO; i++)  // start servos at rest, no motion at boot
    servos[i].current = servos[i].target;
  initSoap();      // bubble motor inputs LOW as early as possible
  initMotors();
  initServos();
  initButtons();
  initWifi();
  Serial.println("==============================");
  Serial.println("Dragon robot ready.");
  Serial.println("Buttons: forward / back / left / right. Action: 1 tap egg, 2 wings, 3 bubbles.");
  Serial.println("Serial test: f b l r x e w s ?   p = parameters");
  lastTick = millis();
}

void loop() {
  if (ENABLE_WIFI_CONTROL) web.handleClient();
  unsigned long now = millis();
  if (now - lastTick < 5) return;         // run every 5 ms
  float dt = (now - lastTick) / 1000.0f;
  lastTick = now;

  handleSerial(now);
  updateButtons(now);

  updateTaps(now);
  updateDrive(now, dt);
  updateWings(now);
  updateSoap(now);
  updateServos(dt);
}
