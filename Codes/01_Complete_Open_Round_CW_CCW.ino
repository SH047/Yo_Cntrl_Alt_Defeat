// ============================================================
// WRO FE 2026 — OPEN CHALLENGE — v3b (v3 + wall clamp + ToF dropout hold)
//
// Your open-challenge code, with four changes (each marked CHANGED):
//   1. ONE tuning block at the top — everything you may want to
//      change during practice lives there, nothing else is magic.
//   2. ENCODER brought in: leg distance, per-leg log, optional
//      minimum-leg guard, and a second FINISH mode (by distance).
//      FINISH_MODE = 0 (time, as before) or 1 (encoder distance).
//      In BOTH modes the log prints time AND distance so you can
//      compare them before switching.
//   3. HEADING control is P + D with a measured dt (was P only).
//   4. ToF sensors run in CONTINUOUS mode; loop() just reads the
//      latest value instead of blocking ~25 ms per sensor.
//
// Nothing about corner decisions, direction lock, crash recovery
// or the finish run-through has been changed in behaviour — only
// pulled into functions and parameterised.
// ============================================================

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>      // CHANGED v3: Bluetooth logging

// ============================================================
// PINS
// ============================================================
#define IN1_PIN        26   // Motor PWM
#define IN2_PIN        27   // Motor DIR
#define SERVO_PIN      13
#define NEOPIXEL_PIN   14
#define START_BTN_PIN  25
#define ENCODER_A_PIN  18   // CHANGED: encoder (from the test sketch)
#define ENCODER_B_PIN  19

// I2C mux channels
#define TCAADDR        0x70
#define TOF_FRONT_CH   0
#define TOF_RIGHT_CH   1
#define TOF_LEFT_CH    2
#define BNO_CH         3

// ############################################################
// ##                                                        ##
// ##   TUNING — change things HERE, nowhere else            ##
// ##                                                        ##
// ############################################################

// ---------- Speeds (PWM 0-255) ----------
#define BASE_SPEED          130   // straights
#define TURN_SPEED          125   // during a turn
#define REVERSE_SPEED       75   // crash-recovery reverse

// ---------- Servo ----------
#define SERVO_CENTER_DEG    100
#define SERVO_MIN_DEG        28
#define SERVO_MAX_DEG       142
#define SERVO_SHIFT_LEFT     70   // fixed arc during a LEFT turn
#define SERVO_SHIFT_RIGHT   130   // fixed arc during a RIGHT turn

// ---------- Heading control (straights) ----------  CHANGED: + D
#define HEADING_KP          1.2   // deg servo per deg error (was 1.2)
#define HEADING_KD          0.05  // start 0.03-0.08; 0 = old P-only
#define HEADING_MAX_CORR     30   // clamp on (P+D) before wall term

// ---------- Wall centring (straights + finish) ----------
#define WALL_KP             1.2
#define WALL_TARGET_CM     42.0   // desired distance from the wall
#define WALL_SEE_CM        80.0   // a wall farther than this is "not seen"
#define WALL_MAX_CORR        12   // CHANGED v3b: wall term can nudge, never dominate

// ---------- Corner decision ----------
#define TURN_THRESHOLD_CM  85.0   // front wall closer than this -> consider turning
#define SIDE_GAP_CM        50.0   // a side farther than this is "open"
#define BLIND_LOCK_CM      40.0   // front closer than this -> turn even if sides unclear
#define TURN_COOLDOWN_MS   1000   // no new corner within this time of the last
#define MIN_LEG_MM            0   // CHANGED: no new corner until this many mm
                                  // driven since the last one. 0 = off.
                                  // Try 600 once the encoder is trusted.

// ---------- Turn exit ----------
#define TURN_EXIT_DEG       8.0   // heading error below this = turn done
#define TURN_TIMEOUT_MS    6500   // give up on a turn after this
#define TURN_BLEND_DEG        30   // CHANGED (optional): once error < this,
                                  // steer with the PD controller instead of
                                  // the fixed SERVO_SHIFT. 0 = off (old
                                  // behaviour). Try 30 for a smoother exit.

// ---------- Crash recovery ----------
#define REVERSE_START_CM   12.0   // front closer than this -> reverse
#define REVERSE_STOP_CM    20.0   // front farther than this -> stop reversing

// ---------- Finish ----------
#define STOP_AFTER_TURNS     12
#define FINISH_MODE           0   // CHANGED: 0 = by TIME (as before)
                                  //          1 = by ENCODER distance
#define FINISH_TIME_MS     2000   // used when FINISH_MODE = 0
#define FINISH_MM          1200   // used when FINISH_MODE = 1

// ---------- Encoder ----------  CHANGED
#define TICKS_PER_METER    90.0   // from the test sketch — verify!

// ---------- Logging ----------  CHANGED v3: Bluetooth
#define LOG_TO_BT             1   // 1 = also send to phone, 0 = Serial only
#define BT_NAME     "YoLabs-FE"   // name shown in the phone's BT list
#define LOG_EVERY_MS        250

// ---------- ToF dropout hold ----------  CHANGED v3b
#define TOF_HOLD_MS         150   // keep last good reading this long before
                                  // believing a "no target" (999)
#define LOG_BATCH            10   // lines per BT write

// ############################################################
// ##   end of tuning                                        ##
// ############################################################


// ============================================================
// OBJECTS & STATE
// ============================================================
Adafruit_VL53L0X tof[3];
Adafruit_BNO055  bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo            steeringServo;
BluetoothSerial  SerialBT;                 // CHANGED v3
Adafruit_NeoPixel strip(16, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

int   TURN = 0;
float target_heading = 0.0;
bool  isTurning = false;
bool  isReversing = false;
int   turnDirection = 0;         // this turn: +1 right, -1 left
int   lockedTurnDirection = 0;   // whole run: +1 CW, -1 CCW, 0 unknown
int   currentServoAngle = SERVO_CENTER_DEG;

unsigned long turnStartTime   = 0;
unsigned long lastTurnEndTime = 0;
unsigned long lastSerialTime  = 0;
unsigned long runStartTime    = 0;

// CHANGED: encoder
volatile long encoderTicks = 0;
long legStartTicks = 0;          // ticks at the start of the current leg

// CHANGED: heading PD memory
float         prevHeadingError = 0;
unsigned long prevHeadingTime  = 0;

// CHANGED: last good ToF readings (cm), updated whenever a range is ready
float lastDist[3] = {999.0, 999.0, 999.0};
unsigned long lastGoodTime[3] = {0, 0, 0};     // CHANGED v3b: when lastDist was last valid

// CHANGED v3: log buffer for batched Bluetooth sends
String logBuf; int logLines = 0;


// ============================================================
// SMALL HELPERS
// ============================================================
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void setNeoPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, strip.Color(r, g, b));
  strip.show();
}

// CHANGED v3: one logging call — Serial always, Bluetooth in batches.
void logLine(const String &s) {
  Serial.println(s);
#if LOG_TO_BT
  logBuf += s; logBuf += '\n'; logLines++;
  if (logLines >= LOG_BATCH) { SerialBT.print(logBuf); logBuf = ""; logLines = 0; }
#endif
}
void logFlush() {
#if LOG_TO_BT
  if (logLines) { SerialBT.print(logBuf); logBuf = ""; logLines = 0; }
#endif
}

float getHeadingError(float target, float current) {
  float diff = target - current;
  while (diff > 180) diff -= 360;
  while (diff < -180) diff += 360;
  return diff;
}

void driveMotor(int speed, bool forward) {
  if (speed == 0) {
    analogWrite(IN1_PIN, 0);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN2_PIN, forward ? HIGH : LOW);
    analogWrite(IN1_PIN, speed);
  }
}

void centerWheels() {
  steeringServo.write(SERVO_MIN_DEG);  delay(400);
  steeringServo.write(SERVO_MAX_DEG);  delay(400);
  steeringServo.write(SERVO_CENTER_DEG); delay(500);
}


// ============================================================
// CHANGED: ENCODER
// ============================================================
void IRAM_ATTR encoderISR() {
  if (digitalRead(ENCODER_B_PIN) == HIGH) encoderTicks++;
  else                                    encoderTicks--;
}

// mm driven since a reference tick count (sign follows direction)
float mmSince(long refTicks) {
  long t;
  noInterrupts(); t = encoderTicks; interrupts();
  return (t - refTicks) * (1000.0 / TICKS_PER_METER);
}

float legMM()   { return mmSince(legStartTicks); }   // this leg
float totalMM() { return mmSince(0); }               // whole run


// ============================================================
// CHANGED: ToF in CONTINUOUS mode
//   setup: each sensor is told to range continuously on its own.
//   read : select the channel, take the latest result if one is
//          ready, else keep the previous value. Never blocks.
// ============================================================
void tofStartAll() {
  for (int i = 0; i < 3; i++) {
    tcaselect(i);
    tof[i].startRangeContinuous(30);   // new sample every ~30 ms
  }
}

float getDistance(uint8_t ch) {
  tcaselect(ch);
  if (tof[ch].isRangeComplete()) {
    uint16_t mm = tof[ch].readRangeResult();
    if (mm > 0 && mm <= 2000) {
      lastDist[ch] = mm / 10.0;                        // good sample
      lastGoodTime[ch] = millis();
    } else if (millis() - lastGoodTime[ch] > TOF_HOLD_MS) {
      lastDist[ch] = 999.0;                            // CHANGED v3b: only after
    }                                                  // TOF_HOLD_MS of misses
  }
  return lastDist[ch];
}

float readHeading() {
  tcaselect(BNO_CH);
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}


// ============================================================
// CONTROL PIECES
// ============================================================

// CHANGED: heading P + D. Returns servo correction in degrees.
float headingPD(float headingError) {
  unsigned long now = millis();
  float dt = (now - prevHeadingTime) / 1000.0;
  if (dt < 0.001) dt = 0.001;
  if (dt > 0.2)   dt = 0.2;          // one slow loop must not spike D
  float dTerm = HEADING_KD * (headingError - prevHeadingError) / dt;
  prevHeadingError = headingError;
  prevHeadingTime  = now;
  float corr = HEADING_KP * headingError + dTerm;
  return constrain(corr, -HEADING_MAX_CORR, HEADING_MAX_CORR);
}

// Wall-centring term — identical logic to before, now in one place.
float wallCentering(float distL, float distR) {
  float wc = 0;
  if (distL < WALL_SEE_CM && distR < WALL_SEE_CM) {
    wc = (distR - distL) * (WALL_KP * 0.5);
  } else if (lockedTurnDirection == 1) {
    if      (distR < WALL_SEE_CM) wc -= (WALL_TARGET_CM - distR) * WALL_KP;
    else if (distL < WALL_SEE_CM) wc += (WALL_TARGET_CM - distL) * WALL_KP;
  } else if (lockedTurnDirection == -1) {
    if      (distL < WALL_SEE_CM) wc += (WALL_TARGET_CM - distL) * WALL_KP;
    else if (distR < WALL_SEE_CM) wc -= (WALL_TARGET_CM - distR) * WALL_KP;
  } else {
    if      (distR < WALL_SEE_CM) wc -= (WALL_TARGET_CM - distR) * WALL_KP;
    else if (distL < WALL_SEE_CM) wc += (WALL_TARGET_CM - distL) * WALL_KP;
  }
  return constrain(wc, -WALL_MAX_CORR, WALL_MAX_CORR);   // CHANGED v3b
}

// One "drive straight" step: heading PD + wall term -> servo, motor on.
void driveStraightStep(float headingError, float distL, float distR, int speed) {
  float correction = headingPD(headingError) + wallCentering(distL, distR);
  currentServoAngle = constrain((int)(SERVO_CENTER_DEG + correction), SERVO_MIN_DEG, SERVO_MAX_DEG);
  steeringServo.write(currentServoAngle);
  driveMotor(speed, true);
}

// Corner decision — same rules as before, parameterised.
bool decideTurn(float distF, float distL, float distR) {
  if (distF > TURN_THRESHOLD_CM || distF <= 2.0) return false;
  if (millis() - lastTurnEndTime < TURN_COOLDOWN_MS) return false;
  if (MIN_LEG_MM > 0 && legMM() < MIN_LEG_MM) return false;   // CHANGED

  bool rightOpen = (distR > SIDE_GAP_CM);
  bool leftOpen  = (distL > SIDE_GAP_CM);

  if (lockedTurnDirection == 0) {
    if (rightOpen && !leftOpen)      { lockedTurnDirection =  1; logLine("LOCKED: CLOCKWISE");      return true; }
    else if (leftOpen && !rightOpen) { lockedTurnDirection = -1; logLine("LOCKED: ANTI-CLOCKWISE"); return true; }
    else if (distF <= BLIND_LOCK_CM) {
      lockedTurnDirection = (distR > distL) ? 1 : -1;
      logLine(String("BLIND LOCK: ") + (lockedTurnDirection == 1 ? "CW" : "CCW"));
      return true;
    }
    return false;
  }
  if (lockedTurnDirection ==  1 && (rightOpen || distF <= BLIND_LOCK_CM)) return true;
  if (lockedTurnDirection == -1 && (leftOpen  || distF <= BLIND_LOCK_CM)) return true;
  return false;
}

void startTurn() {
  isTurning = true;
  turnStartTime = millis();
  setNeoPixels(255, 165, 0);
  turnDirection = lockedTurnDirection;
  if (turnDirection == 1) { target_heading += 90.0; currentServoAngle = SERVO_SHIFT_RIGHT; }
  else                    { target_heading -= 90.0; currentServoAngle = SERVO_SHIFT_LEFT;  }
  if (target_heading >= 360.0) target_heading -= 360.0;
  if (target_heading <    0.0) target_heading += 360.0;
  logLine("TURN " + String(TURN + 1) + " start, leg was " + String(legMM(), 0) + " mm");
}

void endTurn() {
  isTurning = false;
  isReversing = false;
  TURN++;
  lastTurnEndTime = millis();
  legStartTicks = encoderTicks;                 // CHANGED: new leg starts here
  currentServoAngle = SERVO_CENTER_DEG;
  steeringServo.write(currentServoAngle);
  setNeoPixels(0, 255, 255);
}

// Turn step — fixed arc, crash recovery, optional PD blend near the end.
void turnStep(float headingError, float distF, float distL, float distR) {
  if (distF < REVERSE_START_CM)     isReversing = true;
  else if (distF > REVERSE_STOP_CM) isReversing = false;

  if (isReversing) {
    int reverseAngle = (turnDirection == 1) ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;
    steeringServo.write(reverseAngle);
    currentServoAngle = reverseAngle;
    driveMotor(REVERSE_SPEED, false);
  } else if (TURN_BLEND_DEG > 0 && fabs(headingError) < TURN_BLEND_DEG) {
    driveStraightStep(headingError, distL, distR, TURN_SPEED);   // CHANGED (optional)
  } else {
    steeringServo.write(currentServoAngle);
    driveMotor(TURN_SPEED, true);
  }

  if (fabs(headingError) < TURN_EXIT_DEG || (millis() - turnStartTime > TURN_TIMEOUT_MS)) {
    endTurn();
  }
}

// After 12 turns: keep centring and drive into the section, then stop.
// CHANGED: ends by TIME or by ENCODER DISTANCE; logs both either way.
void finishRun() {
  setNeoPixels(255, 0, 255);
  unsigned long t0 = millis();
  long ticks0 = encoderTicks;
  logLine(String("12 turns done. Finishing by ") + (FINISH_MODE == 0 ? "TIME" : "ENCODER"));

  while (true) {
    unsigned long elapsed = millis() - t0;
    float driven = mmSince(ticks0);
    bool done = (FINISH_MODE == 0) ? (elapsed >= FINISH_TIME_MS) : (driven >= FINISH_MM);
    if (done) break;

    float distR = getDistance(TOF_RIGHT_CH);
    float distL = getDistance(TOF_LEFT_CH);
    float headingError = getHeadingError(target_heading, readHeading());
    driveStraightStep(headingError, distL, distR, BASE_SPEED);
    delay(15);
  }

  driveMotor(0, true);
  setNeoPixels(255, 0, 0);
  logLine("FINISHED time " + String(millis() - t0) + " ms, distance " + String(mmSince(ticks0), 0) +
          " mm | run total " + String((millis() - runStartTime) / 1000.0, 1) + " s, " + String(totalMM(), 0) + " mm");
  logFlush();
  while (1) delay(1000);
}

// CSV: t_ms,turn,state,F,L,R,h_err,servo,leg_mm
void periodicLog(float distF, float distL, float distR, float headingError) {
  if (millis() - lastSerialTime < LOG_EVERY_MS) return;
  lastSerialTime = millis();
  const char* st = !isTurning ? "STRAIGHT" : (isReversing ? "REVERSE" : (turnDirection == 1 ? "TURN_CW" : "TURN_CCW"));
  logLine(String(millis() - runStartTime) + "," + String(TURN) + "," + st + "," +
          String(distF, 0) + "," + String(distL, 0) + "," + String(distR, 0) + "," +
          String(headingError, 1) + "," + String(currentServoAngle) + "," + String(legMM(), 0));
}


// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
#if LOG_TO_BT
  SerialBT.begin(BT_NAME);                        // CHANGED v3
#endif

  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(START_BTN_PIN, INPUT_PULLUP);

  // CHANGED: encoder
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

  strip.begin();
  setNeoPixels(255, 0, 0);
  Wire.begin();

  steeringServo.setPeriodHertz(50);
  steeringServo.attach(SERVO_PIN, 500, 2400);
  centerWheels();
  driveMotor(0, true);

  for (int i = 0; i < 3; i++) {
    tcaselect(i);
    if (!tof[i].begin()) {
      Serial.print("ERROR: ToF channel "); Serial.print(i); Serial.println(" missing!");
      while (1) { setNeoPixels(255, 255, 0); delay(100); setNeoPixels(0, 0, 0); delay(100); }
    }
  }
  tofStartAll();                                   // CHANGED

  tcaselect(BNO_CH);
  if (!bno.begin()) {
    Serial.println("ERROR: No BNO055 detected!");
    while (1) { setNeoPixels(255, 255, 0); delay(100); setNeoPixels(0, 0, 0); delay(100); }
  }
  delay(100);
  bno.setExtCrystalUse(true);

  setNeoPixels(0, 255, 0);
  logLine("Ready. Press START."); logFlush();
  while (digitalRead(START_BTN_PIN) == HIGH) delay(50);
  delay(50);
  while (digitalRead(START_BTN_PIN) == LOW) delay(10);

  target_heading = readHeading();
  prevHeadingError = 0;
  prevHeadingTime  = millis();
  encoderTicks = 0;
  legStartTicks = 0;
  runStartTime = millis();

  setNeoPixels(0, 255, 255);
  logLine("Run started. Heading locked at " + String(target_heading));
  logLine("t_ms,turn,state,F,L,R,h_err,servo,leg_mm");    // CSV header
}


// ============================================================
// LOOP
// ============================================================
void loop() {
  if (TURN >= STOP_AFTER_TURNS) finishRun();     // never returns

  float distF = getDistance(TOF_FRONT_CH);
  float distR = getDistance(TOF_RIGHT_CH);
  float distL = getDistance(TOF_LEFT_CH);
  float headingError = getHeadingError(target_heading, readHeading());

  if (!isTurning) {
    if (decideTurn(distF, distL, distR)) startTurn();
    else driveStraightStep(headingError, distL, distR, BASE_SPEED);
  } else {
    turnStep(headingError, distF, distL, distR);
  }

  periodicLog(distF, distL, distR, headingError);
}
