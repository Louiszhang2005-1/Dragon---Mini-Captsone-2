/*
  4-Wheel Motor Control — Trial Sketch (ESP32 + 2 dual H-bridge modules)
  -----------------------------------------------------------------------
  Wiring (from your diagram):
    Module 1 (left motors)               Module 2 (right motors)
      AIN1 -> D19   (Front left)           AIN1 -> D26   (Front right)
      AIN2 -> D18   (Front left)           AIN2 -> D27   (Front right)
      BIN1 -> D22   (Rear left)            BIN1 -> D33   (Rear right)
      BIN2 -> D23   (Rear left)            BIN2 -> D32   (Rear right)
      STBY -> D21                          STBY -> D25
    VM = + batteries (3x AA), GND = - batteries = ESP32 GND

  The modules have no separate PWM pin (DRV8833-style), so speed is set by
  PWM directly on the IN1/IN2 pins.

  HOW TO USE
  1. Edit "TRIAL SETTINGS" below.
  2. Tools > Board > "ESP32 Dev Module", then Upload.
  3. The robot waits START_DELAY_MS, runs for TRIAL_DURATION_MS, then stops.
  4. Press the EN (reset) button on the ESP32 to repeat the trial.

  SPEED CONVENTION: -255 ... 0 ... +255
    + = forward, - = backward, magnitude = power (255 = 100%)
*/

// ======================= TRIAL SETTINGS =======================
const char* TRIAL_NAME = "Trial 1";

int SPEED_FRONT_LEFT  = 200;
int SPEED_FRONT_RIGHT = 200;
int SPEED_REAR_LEFT   = 200;
int SPEED_REAR_RIGHT  = 200;

const unsigned long START_DELAY_MS    = 3000;  // time to put the robot down
const unsigned long TRIAL_DURATION_MS = 5000;  // how long motors run (includes ramp)
const unsigned long RAMP_TIME_MS      = 500;   // smooth start (0 = instant)

// Calibration: scale a wheel that runs faster/slower (1.00 = unchanged)
float TRIM_FRONT_LEFT  = 1.00;
float TRIM_FRONT_RIGHT = 1.00;
float TRIM_REAR_LEFT   = 1.00;
float TRIM_REAR_RIGHT  = 1.00;

// Set true if a wheel spins backward when given a positive speed.
// (Right-side motors are mirrored, so they often need this.)
bool INVERT_FRONT_LEFT  = false;
bool INVERT_FRONT_RIGHT = false;
bool INVERT_REAR_LEFT   = false;
bool INVERT_REAR_RIGHT  = false;
// ==============================================================


// ========================= PINS ===============================
const uint8_t STBY_LEFT  = 21;
const uint8_t STBY_RIGHT = 25;

const uint32_t PWM_FREQ = 20000;  // 20 kHz: above hearing, no motor whine
const uint8_t  PWM_BITS = 8;      // duty 0..255

struct Motor {
  const char* name;
  uint8_t in1;
  uint8_t in2;
  uint8_t ch1;   // LEDC channels (only used on ESP32 core 2.x)
  uint8_t ch2;
  int   speed;
  float trim;
  bool  invert;
};

Motor motors[4] = {
  // name          IN1  IN2  ch1 ch2
  {"Front Left",   19,  18,  0,  1,  0, 1.0, false},  // Module 1, A
  {"Rear Left",    22,  23,  2,  3,  0, 1.0, false},  // Module 1, B
  {"Front Right",  26,  27,  4,  5,  0, 1.0, false},  // Module 2, A
  {"Rear Right",   33,  32,  6,  7,  0, 1.0, false},  // Module 2, B
};
// ==============================================================


// --- PWM helpers that work on both ESP32 Arduino core 2.x and 3.x ---
void pwmAttach(uint8_t pin, uint8_t ch) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
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

// Drive one motor, value -255..255.
// Uses "slow decay" (one input held HIGH, PWM on the other), which gives a
// more linear speed response and better torque at low speed.
void setMotor(Motor &m, int value) {
  value = constrain(value, -255, 255);
  if (m.invert) value = -value;

  if (value > 0) {          // forward: IN1 = HIGH, IN2 = PWM (inverted)
    pwmWrite(m.in1, m.ch1, 255);
    pwmWrite(m.in2, m.ch2, 255 - value);
  } else if (value < 0) {   // backward: IN2 = HIGH, IN1 = PWM (inverted)
    pwmWrite(m.in1, m.ch1, 255 + value);
    pwmWrite(m.in2, m.ch2, 255);
  } else {                  // brake (both HIGH)
    pwmWrite(m.in1, m.ch1, 255);
    pwmWrite(m.in2, m.ch2, 255);
  }
}

void coastMotor(Motor &m) {  // both LOW = free-wheel
  pwmWrite(m.in1, m.ch1, 0);
  pwmWrite(m.in2, m.ch2, 0);
}

int targetSpeed(const Motor &m) {
  return constrain((int)(m.speed * m.trim), -255, 255);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  motors[0].speed = SPEED_FRONT_LEFT;  motors[0].trim = TRIM_FRONT_LEFT;  motors[0].invert = INVERT_FRONT_LEFT;
  motors[1].speed = SPEED_REAR_LEFT;   motors[1].trim = TRIM_REAR_LEFT;   motors[1].invert = INVERT_REAR_LEFT;
  motors[2].speed = SPEED_FRONT_RIGHT; motors[2].trim = TRIM_FRONT_RIGHT; motors[2].invert = INVERT_FRONT_RIGHT;
  motors[3].speed = SPEED_REAR_RIGHT;  motors[3].trim = TRIM_REAR_RIGHT;  motors[3].invert = INVERT_REAR_RIGHT;

  for (int i = 0; i < 4; i++) {
    pwmAttach(motors[i].in1, motors[i].ch1);
    pwmAttach(motors[i].in2, motors[i].ch2);
    coastMotor(motors[i]);
  }

  // Enable both driver modules
  pinMode(STBY_LEFT, OUTPUT);
  pinMode(STBY_RIGHT, OUTPUT);
  digitalWrite(STBY_LEFT, HIGH);
  digitalWrite(STBY_RIGHT, HIGH);

  // Print trial parameters for your lab notes
  Serial.println("==============================");
  Serial.print("Starting: "); Serial.println(TRIAL_NAME);
  for (int i = 0; i < 4; i++) {
    int s = targetSpeed(motors[i]);
    Serial.printf("%-12s: %4d  (%3d%% %s)\n", motors[i].name, s, abs(s) * 100 / 255,
                  s > 0 ? "forward" : (s < 0 ? "backward" : "stopped"));
  }
  Serial.printf("Duration: %lu ms\n", TRIAL_DURATION_MS);

  delay(START_DELAY_MS);

  // ---- Ramp up ----
  unsigned long t0 = millis();
  if (RAMP_TIME_MS > 0) {
    unsigned long elapsed;
    while ((elapsed = millis() - t0) < RAMP_TIME_MS) {
      float k = (float)elapsed / RAMP_TIME_MS;
      for (int i = 0; i < 4; i++) setMotor(motors[i], (int)(targetSpeed(motors[i]) * k));
      delay(10);
    }
  }

  // ---- Run ----
  for (int i = 0; i < 4; i++) setMotor(motors[i], targetSpeed(motors[i]));
  Serial.println("Running...");
  while (millis() - t0 < TRIAL_DURATION_MS) delay(10);

  // ---- Stop: brake briefly, then free-wheel and disable drivers ----
  for (int i = 0; i < 4; i++) setMotor(motors[i], 0);
  delay(300);
  for (int i = 0; i < 4; i++) coastMotor(motors[i]);
  digitalWrite(STBY_LEFT, LOW);
  digitalWrite(STBY_RIGHT, LOW);

  Serial.println("Trial complete. Press EN/reset to run again.");
}

void loop() {
  // Trial runs once per reset.
}
