/*
================================================================
                 ROBO LINE DASH 2026
              AI-ASSISTED MAZE SOLVER  (corrected)

ESP32-WROOM
8-channel digital line sensor
TB6612FNG
SH1106 OLED
HW-504 joystick
3 status LEDs

================================================================
FIXED PIN MAP  (unchanged)
================================================================

QTR:
D1 -> D4
D2 -> RX2 / GPIO16
D3 -> TX2 / GPIO17
D4 -> D5
D5 -> D18
D6 -> D19
D7 -> D23
D8 -> D15

QTR VCC -> 3V3
QTR IR  -> 3V3
QTR GND -> GND

TB6612:
PWMA -> D25
AIN1 -> D26
AIN2 -> D27

PWMB -> D32
BIN1 -> D33
BIN2 -> D14

STBY -> 3V3
VCC  -> 3V3
VM   -> battery+
GND  -> common GND

OLED:
SDA -> D21
SCL/SCK -> D22
VCC -> 3V3
GND -> GND

JOYSTICK:
VRy -> VN / GPIO39
SW  -> D35
VCC -> 3V3
GND -> GND

LED:
Yellow -> TX0 / GPIO1
Blue   -> RX0 / GPIO3
Green  -> D13

Red = power indicator

================================================================
HARDWARE NOTES (no pin changes needed)
================================================================

* GPIO35 has NO internal pull-up. Add an external 10k resistor
  from SW (GPIO35) to 3V3, otherwise the button reads random values.
* GPIO1/GPIO3 are UART0, so Serial is NOT used in this firmware.
  The yellow LED may flicker briefly at boot (boot log) - harmless.
* This firmware does NOT use QTRSensors.h. The sensor board is
  read as plain digital inputs.

================================================================
HOW TO USE
================================================================

1. Power on. Hold joystick button at power-up = DRY RUN (learn).
   Release = MAIN RUN (replay shortest path).
   If no route is stored, MAIN falls back to DRY automatically.
2. Follow the OLED: sensor over WHITE, then over BLACK line.
3. Place robot at the maze start. It starts after a short countdown.
4. Dry run uses the LEFT-HAND rule, removes dead ends, and saves
   the shortest path to flash when the finish square is reached.
5. Power-cycle, place at start, release the button -> main run.

TUNE FIRST:  INCH_MS  (how far the robot rolls forward on a junction
so the wheel axle sits over the junction centre).

================================================================
*/

#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// ================================================================
// OLED
// ================================================================

#define OLED_SDA 21
#define OLED_SCL 22

Adafruit_SH1106G display(128, 64, &Wire, -1);
bool oledOk = false;

// Set to 1 to draw telemetry while driving (slows control a bit).
#define LIVE_OLED 0

// ================================================================
// PREFERENCES
// ================================================================

Preferences prefs;

// ================================================================
// SENSOR PINS
// ================================================================

const uint8_t sensorPins[8] = {
  4,      // D1
  5,      // D2
  16,     // D3 = RX2
  17,     // D4 = TX2
  18,     // D5
  19,     // D6
  23,     // D7
  15      // D8
};

uint8_t sensorRaw[8];
uint8_t sensorFiltered[8];   // 1 = black line, 0 = white floor
uint8_t sensorHistory[8];

bool blackIsLow = true;

// ================================================================
// TB6612
// ================================================================

#define PWMA 25
#define AIN1 26
#define AIN2 27

#define PWMB 32
#define BIN1 33
#define BIN2 14

// STBY is permanently connected to 3V3.

// If a motor spins the wrong way, flip these (no rewiring needed).
const bool INVERT_LEFT  = false;
const bool INVERT_RIGHT = false;

#define PWM_FREQ 20000
#define PWM_BITS 8
#define CH_A 0     // only used on ESP32 core 2.x
#define CH_B 1

// ================================================================
// JOYSTICK
// ================================================================

#define JOY_Y  39
#define JOY_SW 35

// ================================================================
// LEDs
// ================================================================

#define LED_YELLOW 1
#define LED_BLUE   3
#define LED_GREEN  13

// ================================================================
// SENSOR GEOMETRY
// ================================================================

const int sensorWeight[8] = {
  -3500, -2500, -1500, -500,
   500,  1500,  2500,  3500
};

// ================================================================
// PID
// ================================================================

float Kp = 0.105;
float Ki = 0.000;
float Kd = 0.72;

float previousError = 0.0;
float integral = 0.0;

// ================================================================
// SPEED
// ================================================================

const int MIN_SPEED  = 75;
const int BASE_SPEED = 155;
const int MAX_SPEED  = 235;

int currentSpeed = BASE_SPEED;

// ================================================================
// MANEUVER PARAMETERS  (tune on the real robot)
// ================================================================

const int      TURN_PWM             = 155;
const int      INCH_SPEED           = 110;
const uint16_t INCH_MS              = 70;    // roll-forward at junctions
const uint16_t STRAIGHT_PUSH_MS     = 40;    // extra push when going straight
const uint16_t TURN_TIMEOUT_MS      = 1800;
const uint16_t U_TURN_MIN_MS        = 200;
const uint16_t DEAD_END_MS          = 90;    // line missing this long = dead end
const int      LOST_CREEP           = 90;
const int      EDGE_ERROR           = 2800;  // |error| above this = curve, not dead end
const uint8_t  JUNCTION_CONFIRM     = 2;     // consecutive samples
const uint16_t JUNCTION_LOCKOUT_MS  = 200;
const uint16_t START_COUNTDOWN_MS   = 2500;

// ================================================================
// MAZE PATH
// ================================================================

#define MAX_PATH 120

char path[MAX_PATH];             // path being recorded (dry run)
uint8_t pathLength = 0;

char shortestPath[MAX_PATH];     // route used by main run
uint8_t shortestLength = 0;

uint8_t replayIndex = 0;

// ================================================================
// MODES / STATES
// ================================================================

enum RunMode {
  MODE_DRY,
  MODE_MAIN
};

RunMode runMode = MODE_DRY;

enum SensorState {
  STATE_LOST,
  STATE_STRAIGHT,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_LEFT_STRAIGHT,
  STATE_RIGHT_STRAIGHT,
  STATE_LEFT_RIGHT,
  STATE_CROSS,
  STATE_DEAD_END,
  STATE_FINISH
};

SensorState sensorState = STATE_LOST;

// ================================================================
// TIMERS / COUNTERS
// ================================================================

unsigned long lastSensorTime   = 0;
unsigned long lastDisplayTime  = 0;
unsigned long lastJunctionTime = 0;
unsigned long lostSince        = 0;

const uint16_t SENSOR_PERIOD_US = 2000;

uint8_t junctionCount = 0;

// ================================================================
// FORWARD DECLARATIONS
// ================================================================

void setMotors(int left, int right);
void stopMotors();
void updateOLED();
void showMessage(const char* a, const char* b, const char* c);
void executeTurn(char turn);
void recordTurn(char turn);
void saveRoute();
void loadRoute();

// ================================================================
// PWM ABSTRACTION  (works on ESP32 Arduino core 2.x and 3.x)
// ================================================================

void pwmBegin() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(PWMA, PWM_FREQ, PWM_BITS);
  ledcAttach(PWMB, PWM_FREQ, PWM_BITS);
#else
  ledcSetup(CH_A, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PWMA, CH_A);
  ledcSetup(CH_B, PWM_FREQ, PWM_BITS);
  ledcAttachPin(PWMB, CH_B);
#endif
}

void pwmWrite(uint8_t pin, uint8_t channel, uint8_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  (void)channel;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(channel, duty);
#endif
}

// ================================================================
// SENSOR FILTER  (3-sample majority)
// ================================================================

uint8_t stableRead(uint8_t pin, uint8_t index) {

  uint8_t value = digitalRead(pin);

  sensorHistory[index] =
    ((sensorHistory[index] << 1) | value) & 0x07;

  uint8_t h = sensorHistory[index];

  uint8_t ones =
    (h & 1) + ((h >> 1) & 1) + ((h >> 2) & 1);

  return (ones >= 2) ? 1 : 0;
}

// ================================================================
// SENSOR READ
// Returns true when a NEW sample was taken.
// ================================================================

bool readSensors(bool force = false) {

  unsigned long now = micros();

  if (!force && (now - lastSensorTime) < SENSOR_PERIOD_US)
    return false;

  lastSensorTime = now;

  for (uint8_t i = 0; i < 8; i++) {

    sensorRaw[i] = stableRead(sensorPins[i], i);

    // 1 = BLACK LINE, 0 = WHITE FLOOR
    sensorFiltered[i] = blackIsLow ? !sensorRaw[i] : sensorRaw[i];
  }

  return true;
}

// Take several forced samples so the majority filter is fully settled.
void sampleSettled() {

  for (uint8_t k = 0; k < 4; k++) {
    readSensors(true);
    delayMicroseconds(700);
  }
}

uint8_t countActive() {

  uint8_t a = 0;

  for (uint8_t i = 0; i < 8; i++)
    if (sensorFiltered[i]) a++;

  return a;
}

// ================================================================
// SENSOR POLARITY CALIBRATION
// Retries until a clear difference between white and black is seen.
// ================================================================

void calibrateSensorPolarity() {

  while (true) {

    showMessage("SENSOR CAL", "OVER WHITE", "WAIT...");
    delay(2000);

    uint16_t white[8] = {0};

    for (uint16_t n = 0; n < 200; n++) {
      for (uint8_t i = 0; i < 8; i++)
        white[i] += digitalRead(sensorPins[i]);
      delay(4);
    }

    showMessage("SENSOR CAL", "OVER BLACK LINE", "WAIT...");
    delay(3000);

    uint16_t black[8] = {0};

    for (uint16_t n = 0; n < 200; n++) {
      for (uint8_t i = 0; i < 8; i++)
        black[i] += digitalRead(sensorPins[i]);
      delay(4);
    }

    long diff = 0;

    for (uint8_t i = 0; i < 8; i++)
      diff += (long)black[i] - (long)white[i];

    // Summed over 8 channels x 200 samples. Need a clear difference.
    if (labs(diff) >= 100) {

      blackIsLow = (diff < 0);

      showMessage("SENSOR CAL OK",
                  blackIsLow ? "BLACK = LOW" : "BLACK = HIGH",
                  "");
      delay(800);
      return;
    }

    showMessage("CAL FAILED", "NO DIFFERENCE", "RETRYING...");
    delay(1500);
  }
}

// ================================================================
// SENSOR CLASSIFIER
// ================================================================

bool isJunctionState(SensorState s) {

  switch (s) {
    case STATE_LEFT:
    case STATE_RIGHT:
    case STATE_LEFT_STRAIGHT:
    case STATE_RIGHT_STRAIGHT:
    case STATE_LEFT_RIGHT:
    case STATE_CROSS:
    case STATE_FINISH:      // wide black: junction handler decides finish
      return true;

    default:
      return false;
  }
}

void classifySensors() {

  uint8_t active = countActive();

  bool L = sensorFiltered[0] && sensorFiltered[1];
  bool R = sensorFiltered[6] && sensorFiltered[7];

  bool C =
    sensorFiltered[2] || sensorFiltered[3] ||
    sensorFiltered[4] || sensorFiltered[5];

  if (active == 0)
    sensorState = STATE_LOST;
  else if (active >= 7)
    sensorState = STATE_FINISH;
  else if (L && R)
    sensorState = C ? STATE_CROSS : STATE_LEFT_RIGHT;
  else if (L)
    sensorState = C ? STATE_LEFT_STRAIGHT : STATE_LEFT;
  else if (R)
    sensorState = C ? STATE_RIGHT_STRAIGHT : STATE_RIGHT;
  else
    sensorState = STATE_STRAIGHT;

  if (isJunctionState(sensorState)) {
    if (junctionCount < 255) junctionCount++;
  } else {
    junctionCount = 0;
  }
}

// ================================================================
// SETUP
// ================================================================

void setup() {

  // Sensors
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
    sensorHistory[i] = 0;
  }

  // Motors
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pwmBegin();
  stopMotors();

  // Joystick
  // GPIO35 has no internal pull-up -> use external 10k to 3V3.
  pinMode(JOY_Y, INPUT);
  pinMode(JOY_SW, INPUT);

  // LEDs
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, LOW);

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  Wire.setClock(400000);

  oledOk = display.begin(0x3C, true);

  if (!oledOk) {
    // Keep running without a display; blink blue as a warning.
    for (uint8_t i = 0; i < 4; i++) {
      digitalWrite(LED_BLUE, HIGH);
      delay(120);
      digitalWrite(LED_BLUE, LOW);
      delay(120);
    }
  }

  showMessage("ROBO LINE DASH", "AI MAZE SOLVER", "STARTING...");
  delay(800);

  // Mode selection: button held at startup = DRY RUN
  if (digitalRead(JOY_SW) == LOW) {
    runMode = MODE_DRY;
  } else {
    runMode = MODE_MAIN;
  }

  calibrateSensorPolarity();

  if (runMode == MODE_MAIN) {

    loadRoute();

    if (shortestLength == 0) {
      // No stored route: explore instead.
      runMode = MODE_DRY;
    }
  }

  digitalWrite(LED_YELLOW, runMode == MODE_DRY ? HIGH : LOW);
  digitalWrite(LED_BLUE,   runMode == MODE_MAIN ? HIGH : LOW);

  pathLength = 0;
  path[0] = '\0';
  replayIndex = 0;

  showMessage(
    runMode == MODE_DRY ? "DRY RUN" : "MAIN RUN",
    runMode == MODE_DRY ? "LEARNING MAZE" : "SHORTEST PATH",
    "PLACE AT START"
  );

  delay(START_COUNTDOWN_MS);

  // Prime the filter with real data before driving.
  sampleSettled();

  lastJunctionTime = millis();
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() {

  // Everything runs on fresh samples only, so the PID derivative is
  // always computed over one sensor period.
  if (!readSensors())
    return;

  classifySensors();

#if LIVE_OLED
  if (millis() - lastDisplayTime >= 250) {
    updateOLED();
    lastDisplayTime = millis();
  }
#endif

  // Junction / finish
  if (junctionCount >= JUNCTION_CONFIRM &&
      (millis() - lastJunctionTime) >= JUNCTION_LOCKOUT_MS) {

    handleJunction();
    return;
  }

  followLine();
}

// ================================================================
// CONTROL RESET
// ================================================================

void resetControl() {

  previousError = 0.0;
  integral = 0.0;
  lostSince = 0;
  junctionCount = 0;
}

// ================================================================
// BRANCH CAPTURE
// ================================================================

void captureBranches(bool &seenLeft, bool &seenRight) {

  if (sensorFiltered[0] && sensorFiltered[1]) seenLeft = true;
  if (sensorFiltered[6] && sensorFiltered[7]) seenRight = true;
}

// ================================================================
// DECISION ENGINE
// Strict left-hand rule: L > S > R > U.
// (Visit-count penalties were removed: they made the robot drift
//  away from the left-hand rule, which can break maze solving.)
// ================================================================

char aiChooseTurn(bool left, bool straight, bool right) {

  int leftScore     = left     ? 300 : -1000;
  int straightScore = straight ? 200 : -1000;
  int rightScore    = right    ? 100 : -1000;

  if (leftScore >= straightScore && leftScore >= rightScore && left)
    return 'L';

  if (straightScore >= rightScore && straight)
    return 'S';

  if (right)
    return 'R';

  return 'U';
}

// ================================================================
// FINISH
// ================================================================

void finishRun() {

  stopMotors();

  if (runMode == MODE_DRY) {

    // Current simplified path = best route found.
    shortestLength = pathLength;

    for (uint8_t i = 0; i < pathLength; i++)
      shortestPath[i] = path[i];

    shortestPath[shortestLength] = '\0';

    saveRoute();   // flash write happens only here, while stopped
  }

  digitalWrite(LED_GREEN, HIGH);

  while (true) {

    sensorState = STATE_FINISH;
    updateOLED();
    stopMotors();
    delay(250);
  }
}

// ================================================================
// JUNCTION HANDLER
// ================================================================

void handleJunction() {

  bool seenLeft  = false;
  bool seenRight = false;

  // Branches visible at the moment of detection.
  captureBranches(seenLeft, seenRight);

  // Roll forward so the axle reaches the junction centre.
  // Keep sampling: the side branches may only be visible briefly.
  setMotors(INCH_SPEED, INCH_SPEED);

  unsigned long t0 = millis();

  while (millis() - t0 < INCH_MS) {

    if (readSensors())
      captureBranches(seenLeft, seenRight);
  }

  stopMotors();
  delay(30);

  // Now look at what is under / ahead of the array.
  sampleSettled();

  uint8_t active = countActive();

  bool straight =
    sensorFiltered[2] || sensorFiltered[3] ||
    sensorFiltered[4] || sensorFiltered[5];

  // Still (almost) all black after rolling past the junction:
  // that is the finish square, not a T-bar.
  if (active >= 7) {
    finishRun();   // never returns
    return;
  }

  char decision;

  if (runMode == MODE_DRY) {

    decision = aiChooseTurn(seenLeft, straight, seenRight);
    recordTurn(decision);

  } else {

    if (replayIndex < shortestLength)
      decision = shortestPath[replayIndex++];
    else
      decision = 'S';
  }

  updateOLED();   // robot is stopped, so the slow I2C transfer is free

  executeTurn(decision);

  resetControl();
  lastJunctionTime = millis();
}

// ================================================================
// DEAD END (dry run only)
// ================================================================

void handleDeadEnd() {

  stopMotors();
  sensorState = STATE_DEAD_END;
  delay(20);

  recordTurn('U');
  updateOLED();

  executeTurn('U');

  resetControl();
  lastJunctionTime = millis();
}

// ================================================================
// TURN LOCK
// First wait until the centre sensors LEAVE the old line (or are
// already off it), then stop as soon as they find the next line.
// ================================================================

void lockOntoNewLine(uint16_t minMs) {

  unsigned long start = millis();
  bool leftOldLine = false;

  while (millis() - start < TURN_TIMEOUT_MS) {

    if (readSensors()) {

      bool centre = sensorFiltered[3] || sensorFiltered[4];

      if (!centre) {
        leftOldLine = true;
      } else if (leftOldLine && (millis() - start) >= minMs) {
        stopMotors();
        delay(25);
        return;
      }
    }
  }

  stopMotors();
}

// ================================================================
// TURN EXECUTION
// ================================================================

void executeTurn(char turn) {

  if (turn == 'S') {

    setMotors(125, 125);
    delay(STRAIGHT_PUSH_MS);
    return;
  }

  if (turn == 'L') {

    setMotors(-TURN_PWM, TURN_PWM);
    lockOntoNewLine(0);
    return;
  }

  if (turn == 'R') {

    setMotors(TURN_PWM, -TURN_PWM);
    lockOntoNewLine(0);
    return;
  }

  if (turn == 'U') {

    setMotors(TURN_PWM, -TURN_PWM);
    lockOntoNewLine(U_TURN_MIN_MS);
    return;
  }
}

// ================================================================
// LINE FOLLOWING PID
// ================================================================

void followLine() {

  long weighted = 0;
  long total = 0;

  for (uint8_t i = 0; i < 8; i++) {

    if (sensorFiltered[i]) {
      weighted += sensorWeight[i];
      total++;
    }
  }

  // Line lost
  if (total == 0) {
    handleLost();
    return;
  }

  lostSince = 0;

  float error = (float)weighted / (float)total;

  // Integral only near centre (anti wind-up)
  if (fabsf(error) < 1800.0f) {

    integral += error;
    integral = constrain(integral, -5000.0f, 5000.0f);

  } else {

    integral *= 0.85f;
  }

  float derivative = error - previousError;

  // Adaptive derivative: stronger damping on sharp movement
  float dynamicKd = Kd + (fabsf(error) / 6000.0f) * 0.25f;

  float correction =
    (Kp * error) +
    (Ki * integral) +
    (dynamicKd * derivative);

  previousError = error;

  // Adaptive speed: slow down with curvature / wide patterns
  int curvature = (int)fabsf(error);

  int speedReduction =
    map(constrain(curvature, 0, 3500), 0, 3500, 0, 75);

  if (countActive() >= 4)
    speedReduction += 15;

  currentSpeed = constrain(BASE_SPEED - speedReduction,
                           MIN_SPEED, MAX_SPEED);

  int left  = currentSpeed + (int)correction;
  int right = currentSpeed - (int)correction;

  setMotors(left, right);
}

// ================================================================
// LINE LOSS
// ================================================================

void recoverLine(unsigned long lostTime) {

  if (lostTime < 120) {

    if (previousError < 0) setMotors(-60, 120);
    else                   setMotors(120, -60);

  } else if (lostTime < 450) {

    if (previousError < 0) setMotors(-100, 145);
    else                   setMotors(145, -100);

  } else {

    if (previousError < 0) setMotors(-125, 155);
    else                   setMotors(155, -125);
  }
}

void handleLost() {

  if (lostSince == 0)
    lostSince = millis();

  unsigned long lostTime = millis() - lostSince;

  // Line went off the sensor edge in a curve: pivot back to it.
  if (fabsf(previousError) >= EDGE_ERROR) {
    recoverLine(lostTime);
    return;
  }

  // Line vanished while roughly centred: creep forward briefly
  // (covers small gaps), then treat as dead end.
  if (lostTime < DEAD_END_MS) {
    setMotors(LOST_CREEP, LOST_CREEP);
    return;
  }

  if (runMode == MODE_DRY)
    handleDeadEnd();
  else
    recoverLine(lostTime);
}

// ================================================================
// PATH RECORDING
// ================================================================

void recordTurn(char turn) {

  if (pathLength >= MAX_PATH - 1)
    return;

  path[pathLength++] = turn;
  path[pathLength] = '\0';

  simplifyPath();
}

// ================================================================
// PATH ANGLE
// ================================================================

int turnAngle(char turn) {

  switch (turn) {
    case 'L': return 270;
    case 'R': return 90;
    case 'U': return 180;
    case 'S': return 0;
    default:  return 0;
  }
}

// ================================================================
// PATH SIMPLIFICATION
// Only valid when the MIDDLE move is a U-turn (dead end):
//   L U L -> S    L U R -> U    L U S -> R
//   R U R -> S    R U L -> U    R U S -> L
//   S U L -> R    S U R -> L    S U S -> U
// ================================================================

void simplifyPath() {

  while (pathLength >= 3 && path[pathLength - 2] == 'U') {

    int total =
      (turnAngle(path[pathLength - 3]) + 180 +
       turnAngle(path[pathLength - 1])) % 360;

    char replacement;

    if      (total == 0)   replacement = 'S';
    else if (total == 90)  replacement = 'R';
    else if (total == 180) replacement = 'U';
    else                   replacement = 'L';

    path[pathLength - 3] = replacement;
    pathLength -= 2;
    path[pathLength] = '\0';
  }
}

// ================================================================
// SAVE ROUTE TO ESP32 NVS
// ================================================================

void saveRoute() {

  prefs.begin("robodash", false);

  prefs.putUChar("len", shortestLength);
  prefs.putString("route", String(shortestPath));

  prefs.end();
}

// ================================================================
// LOAD ROUTE
// ================================================================

void loadRoute() {

  prefs.begin("robodash", true);

  String saved = prefs.getString("route", "");

  prefs.end();

  uint8_t n = 0;

  for (uint16_t i = 0; i < saved.length() && n < MAX_PATH - 1; i++) {

    char c = saved[i];

    if (c == 'L' || c == 'R' || c == 'S' || c == 'U')
      shortestPath[n++] = c;
  }

  shortestLength = n;
  shortestPath[shortestLength] = '\0';

  replayIndex = 0;
}

// ================================================================
// MOTOR CONTROL
// ================================================================

void driveMotor(uint8_t in1, uint8_t in2,
                uint8_t pwmPin, uint8_t channel, int speed) {

  if (speed > 0) {

    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    pwmWrite(pwmPin, channel, speed);

  } else if (speed < 0) {

    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    pwmWrite(pwmPin, channel, -speed);

  } else {

    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    pwmWrite(pwmPin, channel, 0);
  }
}

void setMotors(int left, int right) {

  left  = constrain(left,  -255, 255);
  right = constrain(right, -255, 255);

  if (INVERT_LEFT)  left  = -left;
  if (INVERT_RIGHT) right = -right;

  driveMotor(AIN1, AIN2, PWMA, CH_A, left);
  driveMotor(BIN1, BIN2, PWMB, CH_B, right);
}

// Short brake (TB6612: IN1 = IN2 = HIGH) for a crisp stop.
void stopMotors() {

  pwmWrite(PWMA, CH_A, 0);
  pwmWrite(PWMB, CH_B, 0);

  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

// ================================================================
// OLED
// ================================================================

void updateOLED() {

  if (!oledOk) return;

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);

  // Header
  display.setCursor(0, 0);
  display.print(runMode == MODE_DRY ? "DRY " : "MAIN ");
  display.print("AI");

  // Sensors
  display.setCursor(0, 12);
  display.print("S:");

  for (uint8_t i = 0; i < 8; i++)
    display.print(sensorFiltered[i]);

  // State
  display.setCursor(0, 24);
  display.print("ST:");

  switch (sensorState) {
    case STATE_LOST:           display.print("LOST");     break;
    case STATE_STRAIGHT:       display.print("STRAIGHT"); break;
    case STATE_LEFT:           display.print("LEFT");     break;
    case STATE_RIGHT:          display.print("RIGHT");    break;
    case STATE_LEFT_STRAIGHT:  display.print("L+S");      break;
    case STATE_RIGHT_STRAIGHT: display.print("R+S");      break;
    case STATE_LEFT_RIGHT:     display.print("L+R");      break;
    case STATE_CROSS:          display.print("CROSS");    break;
    case STATE_DEAD_END:       display.print("DEAD");     break;
    case STATE_FINISH:         display.print("FINISH");   break;
  }

  // Route: recorded path (dry) or stored route (main)
  display.setCursor(0, 36);
  display.print("P:");

  const char* p = (runMode == MODE_DRY) ? path : shortestPath;
  uint8_t len   = (runMode == MODE_DRY) ? pathLength : shortestLength;

  for (uint8_t i = 0; i < len && i < 13; i++)
    display.print(p[i]);

  // Speed / replay index
  display.setCursor(0, 48);
  display.print("SPD:");
  display.print(currentSpeed);
  display.print(" R:");
  display.print(replayIndex);

  display.display();
}

// ================================================================
// MESSAGE SCREEN
// ================================================================

void showMessage(const char* a, const char* b, const char* c) {

  if (!oledOk) return;

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 8);
  display.println(a);

  display.setCursor(0, 28);
  display.println(b);

  display.setCursor(0, 48);
  display.println(c);

  display.display();
}
