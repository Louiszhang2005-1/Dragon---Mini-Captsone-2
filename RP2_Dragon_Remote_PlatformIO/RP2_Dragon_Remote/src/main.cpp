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

  BUTTONS
    MOVE LEFT  + MOVE RIGHT held together -> drive forward
    MOVE LEFT  alone                      -> turn left
    MOVE RIGHT alone                      -> turn right
    nothing held                          -> stop
    EGG   (press) -> egg opens; press again -> egg closes
    WINGS (press) -> wings flap 3 cycles (down -> 45° up -> down)
    SOAP  (press) -> soap-gun motor runs for SOAP_RUN_MS (one dose)

  WIRING SUMMARY
    Drive (unchanged from RP2_Motor_code):
      Module 1 (left):  AIN1 19, AIN2 18, BIN1 22, BIN2 23, STBY 21
      Module 2 (right): AIN1 26, AIN2 27, BIN1 33, BIN2 32, STBY 25
    Servo signal wires:
      Left wing 13, Right wing 14, Egg 16
    Soap-gun motor (batteries removed from the gun, trigger switch bypassed):
      GPIO 17 -> 220 Ω -> MOSFET gate, 100 kΩ gate -> GND
      (logic-level N-MOSFET e.g. AO3400 / IRLZ44N, or a ready-made
       "MOSFET switch module" such as D4184: SIG=17, GND=GND)
      Motor + -> 7.4 V rail    |  Motor - -> MOSFET drain  |  source -> GND
      Diode across the motor (1N5819/1N4007), stripe/cathode to motor +
    Dongle outputs:
      OUT1 -> 4 (move left), OUT2 -> 5 (move right), OUT3 -> 15 (egg),
      OUT4 -> 34 (wings),    OUT5 -> 35 (soap)
      NOTE: pins 34/35 have no internal pull-up. The dongle provides one, so
      this is fine when it is plugged in. If they trigger by themselves with
      the dongle unplugged, add a 10 kΩ resistor from each pin to 3V3.

  POWER — 2x 1S LiPo (3.7 V 2000 mAh) wired in SERIES = 7.4 V (8.4 V full)
    Cell 1 (-) -> GND.  Cell 1 (+) -> Cell 2 (-).  Cell 2 (+) = 7.4 V rail.
    Add a main ON/OFF switch on the 7.4 V wire.
    7.4 V rail -> VM of both motor driver modules (DRV8833 max 10.8 V: OK)
               -> soap-gun motor + (switched by the MOSFET on its - side)
               -> 5 V step-down (buck) converter, >= 3 A
                    -> servo red wires (+5 V) AND ESP32 "5V/VIN" pin
    ALL grounds tied together: battery -, buck GND, ESP32 GND, driver GNDs,
    MOSFET source, servo brown wires, dongle GND.
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
int MOTOR_MAX_DUTY = 180;         // hard cap: 180/255 of 8.4 V ≈ 6 V max on
                                 // the wheel motors. Raise (max 255) if too slow
                                 // and your motors are rated for more.
unsigned long RAMP_TIME_MS   = 300;        // time to go 0 -> full (0 = instant)
unsigned long COMBO_DELAY_MS = 80;         // grace time so pressing both buttons
                                           // doesn't cause a short turn first

// Calibration (same meaning as in RP2_Motor_code)
float TRIM_FRONT_LEFT  = 1.00;
float TRIM_FRONT_RIGHT = 1.00;
float TRIM_REAR_LEFT   = 1.00;
float TRIM_REAR_RIGHT  = 1.00;
bool INVERT_FRONT_LEFT  = false;
bool INVERT_FRONT_RIGHT = false;
bool INVERT_REAR_LEFT   = false;
bool INVERT_REAR_RIGHT  = false;

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
float EGG_OPEN_ANGLE   = 85;
float EGG_SPEED_DPS    = 60;   // slow, smooth opening

// ======================= SOAP SETTINGS ========================
// The DC motor taken out of the soap gun, switched by a MOSFET.
// Default: each press runs the motor for SOAP_RUN_MS (one dose).
unsigned long SOAP_RUN_MS      = 1500;  // dose length — tune by testing
bool          SOAP_HOLD_TO_RUN = false; // true = runs as long as button is held
unsigned long SOAP_MAX_MS      = 5000;  // safety limit in hold mode
unsigned long SOAP_COOLDOWN_MS = 1000;  // min. time between two doses
int           SOAP_POWER_PCT   = 75;    // 75% of 8.4 V ≈ 6 V = the gun's 4xAA

// ======================= SERVO PULSE ==========================
// MG90S: about 500 µs = 0°, 2400 µs = 180°. Narrow this if a servo buzzes
// at the ends of its travel.
int SERVO_MIN_US = 500;
int SERVO_MAX_US = 2400;
// ==============================================================


// ========================= PINS ===============================
const uint8_t PIN_BTN_MOVE_LEFT  = 4;
const uint8_t PIN_BTN_MOVE_RIGHT = 5;
const uint8_t PIN_BTN_EGG        = 15;
const uint8_t PIN_BTN_WINGS      = 34;
const uint8_t PIN_BTN_SOAP       = 35;

const uint8_t PIN_SERVO_WING_L = 13;
const uint8_t PIN_SERVO_WING_R = 14;
const uint8_t PIN_SERVO_EGG    = 16;
const uint8_t PIN_SOAP_MOTOR   = 17;   // -> MOSFET gate (via 220 Ω)
const uint8_t SOAP_PWM_CH      = 12;   // LEDC channel (core 2.x only)

const uint8_t STBY_LEFT  = 21;
const uint8_t STBY_RIGHT = 25;

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

enum { BTN_MOVE_LEFT, BTN_MOVE_RIGHT, BTN_EGG, BTN_WINGS, BTN_SOAP, NUM_BTN };

Button buttons[NUM_BTN] = {
  {"Move Left",  PIN_BTN_MOVE_LEFT,  false, false, 0, false},
  {"Move Right", PIN_BTN_MOVE_RIGHT, false, false, 0, false},
  {"Egg",        PIN_BTN_EGG,        false, false, 0, false},
  {"Wings",      PIN_BTN_WINGS,      false, false, 0, false},
  {"Soap",       PIN_BTN_SOAP,       false, false, 0, false},
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
// GPIO -> MOSFET switch -> the gun's original DC motor.
bool soapRunning = false;
unsigned long soapStartedAt = 0;
unsigned long soapLastEnded = 0;

void soapMotor(bool on) {
  uint32_t duty = on ? (uint32_t)(255 * SOAP_POWER_PCT / 100) : 0;
  pwmWrite(PIN_SOAP_MOTOR, SOAP_PWM_CH, duty);
}

void initSoap() {
  pwmAttach(PIN_SOAP_MOTOR, SOAP_PWM_CH, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
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
    bool held = buttons[BTN_SOAP].stable;
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


// ======================== DRIVE ===============================
struct Motor {
  const char* name;
  uint8_t in1, in2;
  uint8_t ch1, ch2;   // LEDC channels (core 2.x only)
  float trim;
  bool  invert;
};

Motor motors[4] = {
  {"Front Left",  19, 18, 0, 1, 1.0, false},  // Module 1, A
  {"Rear Left",   22, 23, 2, 3, 1.0, false},  // Module 1, B
  {"Front Right", 26, 27, 4, 5, 1.0, false},  // Module 2, A
  {"Rear Right",  33, 32, 6, 7, 1.0, false},  // Module 2, B
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
  for (int i = 0; i < 4; i++) {
    pwmAttach(motors[i].in1, motors[i].ch1, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    pwmAttach(motors[i].in2, motors[i].ch2, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
    coastMotor(motors[i]);
  }
  pinMode(STBY_LEFT, OUTPUT);
  pinMode(STBY_RIGHT, OUTPUT);
  digitalWrite(STBY_LEFT, HIGH);
  digitalWrite(STBY_RIGHT, HIGH);
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

  bool back = false;
  if (simMove && now < simUntil) {
    L = (simMove == 'f' || simMove == 'l');
    R = (simMove == 'f' || simMove == 'r');
    back = (simMove == 'b');
  } else simMove = 0;

  int combo = back ? 4 : ((L ? 1 : 0) | (R ? 2 : 0));
  if (combo != lastMoveCombo) { lastMoveCombo = combo; moveComboSince = now; }

  // A single button just pressed might be the start of a two-button press:
  // wait COMBO_DELAY_MS before turning, and keep the previous command meanwhile.
  bool waitingForCombo = (combo == 1 || combo == 2) && (now - moveComboSince < COMBO_DELAY_MS);

  if (!waitingForCombo) {
    switch (combo) {
      case 3: tgtLeft = DRIVE_SPEED;      tgtRight = DRIVE_SPEED;      break; // forward
      case 4: tgtLeft = -DRIVE_SPEED;     tgtRight = -DRIVE_SPEED;     break; // back (phone/serial)
      case 1: tgtLeft = TURN_INNER_SPEED; tgtRight = TURN_OUTER_SPEED; break; // turn left
      case 2: tgtLeft = TURN_OUTER_SPEED; tgtRight = TURN_INNER_SPEED; break; // turn right
      default: tgtLeft = 0;               tgtRight = 0;                break; // stop
    }
  }

  float maxStep = (RAMP_TIME_MS == 0) ? 1000.0f : 255.0f * (dt * 1000.0f) / RAMP_TIME_MS;
  curLeft  = approach(curLeft,  tgtLeft,  maxStep);
  curRight = approach(curRight, tgtRight, maxStep);

  setMotor(motors[0], (int)curLeft);
  setMotor(motors[1], (int)curLeft);
  setMotor(motors[2], (int)curRight);
  setMotor(motors[3], (int)curRight);
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
  {"trFL",   "Trim front left",          "Wheels",P_FLOAT, &TRIM_FRONT_LEFT,     0,    1.5,   0.01},
  {"trFR",   "Trim front right",         "Wheels",P_FLOAT, &TRIM_FRONT_RIGHT,    0,    1.5,   0.01},
  {"trRL",   "Trim rear left",           "Wheels",P_FLOAT, &TRIM_REAR_LEFT,      0,    1.5,   0.01},
  {"trRR",   "Trim rear right",          "Wheels",P_FLOAT, &TRIM_REAR_RIGHT,     0,    1.5,   0.01},
  {"invFL",  "Invert front left",        "Wheels",P_BOOL,  &INVERT_FRONT_LEFT,   0,    1,     1},
  {"invFR",  "Invert front right",       "Wheels",P_BOOL,  &INVERT_FRONT_RIGHT,  0,    1,     1},
  {"invRL",  "Invert rear left",         "Wheels",P_BOOL,  &INVERT_REAR_LEFT,    0,    1,     1},
  {"invRR",  "Invert rear right",        "Wheels",P_BOOL,  &INVERT_REAR_RIGHT,   0,    1,     1},
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
  motors[0].trim = TRIM_FRONT_LEFT;  motors[0].invert = INVERT_FRONT_LEFT;
  motors[1].trim = TRIM_REAR_LEFT;   motors[1].invert = INVERT_REAR_LEFT;
  motors[2].trim = TRIM_FRONT_RIGHT; motors[2].invert = INVERT_FRONT_RIGHT;
  motors[3].trim = TRIM_REAR_RIGHT;  motors[3].invert = INVERT_REAR_RIGHT;
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
  initMotors();
  initServos();
  initSoap();
  initButtons();
  initWifi();
  Serial.println("==============================");
  Serial.println("Dragon robot ready.");
  Serial.println("Both move buttons = forward, one = turn. Egg / Wings / Soap buttons.");
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

  if (buttons[BTN_EGG].justPressed)   eggToggle();
  if (buttons[BTN_WINGS].justPressed) wingsStart();
  if (buttons[BTN_SOAP].justPressed)  soapStart();

  updateDrive(now, dt);
  updateWings(now);
  updateSoap(now);
  updateServos(dt);
}
