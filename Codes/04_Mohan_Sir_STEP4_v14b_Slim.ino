// WRO FE 2026 - Obstacle - STEP 4 v14b slim. HuskyLens I2C: ID 1 = GREEN, ID 2 = RED.
// v14b: v14 pillar handling unchanged + LANE_RETURN 1, RETURN_FRACTION 1.0, MIN_LEG_MM 2000, side-open needs 2 readings, ignore camera blocks with bottom < 30
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include "HUSKYLENS.h"

#define IN1_PIN        26
#define IN2_PIN        27
#define SERVO_PIN      13
#define NEOPIXEL_PIN   14
#define START_BTN_PIN  25
#define ENCODER_A_PIN  18
#define ENCODER_B_PIN  19
#define TCAADDR        0x70
#define TOF_FRONT_CH   0
#define TOF_RIGHT_CH   1
#define TOF_LEFT_CH    2
#define BNO_CH         3

#define MOTOR_ON              1

#define CLEAR_MM            150

#define NEED_SCALE          1.0
#define THETA_SCALE         1.0

#define TURN_R_MM           400
#define STEER_LAG_MM         60
#define THETA_MIN            10
#define THETA_MAX            35

#define CAM_HFOV_DEG         55

#define CAM_K              4352
#define CAM_H0               15

#define SWERVE_BLEND_DEG     10
#define SERVO_SHIFT_LEFT     70
#define SERVO_SHIFT_RIGHT   130
#define OBST_SPEED           80
#define PILLAR_SPEED         80
#define STOP_FRONT_CM      15.0

#define TURN_SPEED           80
#define TURN_THRESHOLD_CM  85.0

#define SIDE_GAP_CM        70.0

#define BLIND_LOCK_CM      40.0
#define TURN_COOLDOWN_MS   1000
#define MIN_LEG_MM         2000
#define TURN_EXIT_DEG       8.0
#define TURN_TIMEOUT_MS    6500

#define TURNS_TO_RUN         12
#define LOST_DEG             45

#define STALL_MS            500

#define TURN_FWD_DEG         45
#define SPLIT_REV_SPEED      70
#define SPLIT_REV_BLEND_DEG  10
#define REV_MIN_MM          300

#define TURN_THROUGH_DEG      0

#define LANE_ANCHOR_SEE_CM 80.0
#define LANE_ANCHOR_GAIN   0.05
#define FINISH_MM          1000

#define STOP_TIMEOUT_MS  200000

#define FRAME_W             320
#define GREEN_ID              1
#define RED_ID                2
#define MIN_PILLAR_H         12

#define PILLAR_W_MAX        130
#define PILLAR_ASPECT_MIN   1.0
#define ENTRY_MIN_BOTTOM     30
#define SIDE_OPEN_MIN         2
#define ENTRY_BOTTOM_MAX    235

#define PIL_H_SLOPE        0.47
#define PIL_H_OFS          30.0
#define PIL_H_MIN_FRAC     0.55

#define PIL_H_MIN_FRAC_G   0.45

#define RED_ASPECT_MIN     1.00
#define GREEN_ASPECT_MIN   0.75
#define CLIP_EDGE_PX          4
#define CLIP_BOTTOM_PX      236
#define TRACK_MAX_DX         60
#define TRACK_DX_PER_100MS   20
#define TRACK_MAX_BACK       25

#define PASS_PUSH_MAX_MM    150

#define COMMIT_CAP_MM      1200

#define UNSTICK_MM          150

#define BACKUP_WINDOW_MM    800
#define BACKUP_BOTTOM       190
#define BACKUP_WRONG_X     0.60
#define BACKUP_OK_BOTTOM    150

#define BACKUP_MAX_MM       200
#define BACK_STALL_MS       300
#define BACK_TIMEOUT_MS    2500

#define PASS_HOLD_MM        200

#define RETURN_DEG           25
#define RETURN_FRACTION     1.0
#define RETURN_TOL_MM        40
#define RETURN_MAX_MM      1200
#define ACQ_STRAIGHT_DEG      8

#define PASS_HOLD_MS        400

#define SERVO_CENTER_DEG    100
#define SERVO_MIN_DEG        28
#define SERVO_MAX_DEG       142

#define HEADING_KP          1.2
#define HEADING_KD         0.05
#define HEADING_MAX_CORR     30

#define WALL_KP             1.2
#define WALL_TARGET_CM     42.0
#define WALL_SEE_CM        80.0
#define WALL_MAX_CORR        12

#define TICKS_PER_METER   657.0
#define TOF_HOLD_MS         150

#define BT_NAME     "YoLabs-FE"
#define LOG_BATCH            10

Adafruit_VL53L0X tof[3];
Adafruit_BNO055  bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo            steeringServo;
BluetoothSerial  SerialBT;
Adafruit_NeoPixel strip(16, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
HUSKYLENS        husky;

float target_heading = 0.0;
int   TURN = 0;
bool  isTurning = false;
int   turnDirection = 0; int lockedTurnDirection = 0;
unsigned long turnStartTime = 0, lastTurnEndTime = 0;
float turnExitHeading = 0;
int   turnPhase = 0;
long  revStartTicks = 0;
float turnStartHeading = 0;
long  legStartTicks = 0;
float swerve_heading = 0.0;
int   currentServoAngle = SERVO_CENTER_DEG;
unsigned long runStartTime = 0;
volatile long encoderTicks = 0;
float prevHeadingError = 0; unsigned long prevHeadingTime = 0;
float lastDist[3] = {999.0, 999.0, 999.0};
unsigned long lastGoodTime[3] = {0, 0, 0};
String logBuf; int logLines = 0;
int   rOpenCnt = 0, lOpenCnt = 0;

enum PillarState { P_NONE, P_COMMIT, P_HOLD, P_RECOVER, P_BACK };
PillarState pState = P_NONE;
int   pX = 0, pY = 0, pW = 0, pH = 0, pColor = 0, pCount = 0;
bool  pSeen = false;
unsigned long pLastSeen = 0;
unsigned long phaseStartMs = 0; long phaseStartTicks = 0;

float latCarMM = 0;
float fwdCarMM = 0;
long  latLastTicks = 0;
float returnTargetLat = 0;
float planTheta = 0;
float pillarFwdMM = 0;

bool  pMeas = false;
int   pRej = 0;
int   refX = 0, refBottom = 0;
unsigned long refMs = 0;
float planPassMM = 0;
float pillarFwdFirst = 0;
bool  pushCapLogged = false;
bool  holdFromCap = false;
long  commitStartTicks = 0;
int   motorCmdSpeed = 0; bool motorCmdFwd = true;
bool  backupUsed = false;
bool  backForPillar = false; float backMaxMM = 0;
long  backLastTicks = 0; unsigned long backLastMove = 0;

void tcaselect(uint8_t i) { if (i > 7) return; Wire.beginTransmission(TCAADDR); Wire.write(1 << i); Wire.endTransmission(); }
void setNeoPixels(uint8_t r, uint8_t g, uint8_t b) { for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, strip.Color(r, g, b)); strip.show(); }
void logLine(const String &s) {
  Serial.println(s);
  logBuf += s; logBuf += '\n'; logLines++;
  if (logLines >= LOG_BATCH) { SerialBT.print(logBuf); logBuf = ""; logLines = 0; }
}
void logFlush() {
  if (logLines) { SerialBT.print(logBuf); logBuf = ""; logLines = 0; }
}
float getHeadingError(float target, float current) { float d = target - current; while (d > 180) d -= 360; while (d < -180) d += 360; return d; }
void driveMotor(int speed, bool forward) {
  motorCmdSpeed = speed; motorCmdFwd = forward;
#if MOTOR_ON
  if (speed == 0) { analogWrite(IN1_PIN, 0); digitalWrite(IN2_PIN, LOW); }
  else { digitalWrite(IN2_PIN, forward ? HIGH : LOW); analogWrite(IN1_PIN, speed); }
#else
  analogWrite(IN1_PIN, 0); digitalWrite(IN2_PIN, LOW);
#endif
}
void setServo(int deg) { currentServoAngle = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG); steeringServo.write(currentServoAngle); }
void centerWheels() { steeringServo.write(SERVO_MIN_DEG); delay(400); steeringServo.write(SERVO_MAX_DEG); delay(400); steeringServo.write(SERVO_CENTER_DEG); delay(500); }
void IRAM_ATTR encoderISR() { if (digitalRead(ENCODER_B_PIN) == HIGH) encoderTicks++; else encoderTicks--; }
float mmSince(long ref) { long t; noInterrupts(); t = encoderTicks; interrupts(); return (t - ref) * (1000.0 / TICKS_PER_METER); }
void tofStartAll() { for (int i = 0; i < 3; i++) { tcaselect(i); tof[i].startRangeContinuous(30); } }
float getDistance(uint8_t ch) {
  tcaselect(ch);
  if (tof[ch].isRangeComplete()) {
    uint16_t mm = tof[ch].readRangeResult();
    if (mm > 0 && mm <= 2000) { lastDist[ch] = mm / 10.0; lastGoodTime[ch] = millis(); }
    else if (millis() - lastGoodTime[ch] > TOF_HOLD_MS) lastDist[ch] = 999.0;
  }
  return lastDist[ch];
}
float readHeading() { tcaselect(BNO_CH); sensors_event_t e; bno.getEvent(&e); return e.orientation.x; }
float headingPD(float err) {
  unsigned long now = millis();
  float dt = constrain((now - prevHeadingTime) / 1000.0, 0.001, 0.2);
  float d = HEADING_KD * (err - prevHeadingError) / dt;
  prevHeadingError = err; prevHeadingTime = now;
  return constrain(HEADING_KP * err + d, -HEADING_MAX_CORR, HEADING_MAX_CORR);
}
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
  return constrain(wc, -WALL_MAX_CORR, WALL_MAX_CORR);
}

void driveArcToHeading(float hdgTarget, float currentHeading, int speed) {
  float err = getHeadingError(hdgTarget, currentHeading);
  if (err >  SWERVE_BLEND_DEG)      setServo(SERVO_SHIFT_RIGHT);
  else if (err < -SWERVE_BLEND_DEG) setServo(SERVO_SHIFT_LEFT);
  else setServo(SERVO_CENTER_DEG + headingPD(err));
  driveMotor(speed, true);
}

void driveStraightStep(float headingError, float distL, float distR, int speed) {
  setServo(SERVO_CENTER_DEG + headingPD(headingError) + wallCentering(distL, distR));
  driveMotor(speed, true);
}

int gateClass(int id, int xc, int yc, int w, int h) {
  int bottom = yc + h / 2;
  bool clipB = (bottom >= CLIP_BOTTOM_PX);
  bool clipS = (xc - w / 2 <= CLIP_EDGE_PX) || (xc + w / 2 >= FRAME_W - CLIP_EDGE_PX);
  if (clipB || clipS) return 1;
  float aspMin = (id == GREEN_ID) ? GREEN_ASPECT_MIN : RED_ASPECT_MIN;
  if ((float)h < aspMin * (float)w) return 0;
  float frac = (id == GREEN_ID) ? PIL_H_MIN_FRAC_G : PIL_H_MIN_FRAC;
  if ((float)h < frac * (PIL_H_SLOPE * bottom + PIL_H_OFS)) return 0;
  return 2;
}

bool readPillar(bool tracking) {
  pSeen = false; pCount = 0; pRej = 0;
  if (!husky.request()) return false;
  int bestY = -1; int bestDx = 100000; bool bestMeas = false;
  while (husky.available()) {
    HUSKYLENSResult r = husky.read();
    if (r.command != COMMAND_RETURN_BLOCK) continue;
    if (r.ID != RED_ID && r.ID != GREEN_ID) continue;
    if (r.height < MIN_PILLAR_H) continue;
    pCount++;
    if (!tracking) {
      if (r.width > PILLAR_W_MAX) continue;
      if (r.yCenter + r.height / 2 > ENTRY_BOTTOM_MAX) continue;
      if (r.yCenter + r.height / 2 < ENTRY_MIN_BOTTOM) continue;
      if ((float)r.height / (float)(r.width > 1 ? r.width : 1) < PILLAR_ASPECT_MIN) continue;
      int g = gateClass(r.ID, r.xCenter, r.yCenter, r.width, r.height);
      if (g == 0) { pRej++; continue; }
      if (r.yCenter > bestY) { bestY = r.yCenter; pX = r.xCenter; pY = r.yCenter; pW = r.width; pH = r.height; pColor = r.ID; pSeen = true; bestMeas = (g == 2); }
    } else {
      if (r.ID != pColor) continue;
      int g = gateClass(r.ID, r.xCenter, r.yCenter, r.width, r.height);
      if (g == 0) { pRej++; continue; }
      int dx = abs(r.xCenter - refX);
      int dxLim = TRACK_MAX_DX + (int)(TRACK_DX_PER_100MS * ((millis() - refMs) / 100.0));
      if (dx > dxLim) { pRej++; continue; }
      if (r.yCenter + r.height / 2 < refBottom - TRACK_MAX_BACK) { pRej++; continue; }
      if (dx < bestDx) { bestDx = dx; pX = r.xCenter; pY = r.yCenter; pW = r.width; pH = r.height; pSeen = true; bestMeas = (g == 2); }
    }
  }
  pMeas = pSeen && bestMeas;
  if (pSeen) pLastSeen = millis();
  if (pSeen && tracking) { refX = pX; refBottom = pY + pH / 2; refMs = millis(); }
  return pSeen;
}

float lateralGain(float thetaDeg, float fwdMM, float yawNowDeg) {
  float th = radians(thetaDeg), y0 = radians(min(yawNowDeg, thetaDeg));
  float lag = (fabs(yawNowDeg) <= 0.5) ? STEER_LAG_MM : 0;
  float arcFwd = TURN_R_MM * (sin(th) - sin(y0));
  float arcLat = TURN_R_MM * (cos(y0) - cos(th));
  float f = fwdMM - lag - arcFwd;
  if (f < 0) { float a = y0 + max(0.0f, fwdMM - lag) / TURN_R_MM; return TURN_R_MM * (cos(y0) - cos(min(a, th))); }
  return arcLat + f * tan(th);
}

void trackLateral(float offRightDeg) {
  long t; noInterrupts(); t = encoderTicks; interrupts();
  float ds = (t - latLastTicks) * (1000.0 / TICKS_PER_METER); latLastTicks = t;
  latCarMM += ds * sin(radians(offRightDeg));
  fwdCarMM += ds * cos(radians(offRightDeg));
}

float planSwerve(float offRightDeg, bool logIt) {
  float bottom = pY + pH / 2.0;
  float d_mm   = 10.0 * CAM_K / max(bottom - CAM_H0, 5.0f);
  float beta   = (pX / (float)FRAME_W - 0.5) * CAM_HFOV_DEG;
  float bearL  = beta + offRightDeg;
  float pilLat = latCarMM + d_mm * sin(radians(bearL));
  float fwd    = d_mm * cos(radians(bearL));
  planPassMM   = fwdCarMM + fwd;
  float need;
  int sgn;
  if (pColor == RED_ID) { sgn = +1; need = (pilLat + CLEAR_MM) - latCarMM; }
  else                  { sgn = -1; need = latCarMM - (pilLat - CLEAR_MM); }
  need *= NEED_SCALE;
  float yawNow = sgn * offRightDeg;
  int th = THETA_MIN;
  if (need > 0) { for (th = THETA_MIN; th < THETA_MAX; th++) if (lateralGain(th, fwd, yawNow) >= need) break; }
  th = constrain((int)(th * THETA_SCALE + 0.5), THETA_MIN, THETA_MAX);
  if (logIt) logLine("  plan: d=" + String(d_mm, 0) + "mm bearing=" + String(bearL, 1) + " pillarLat=" + String(pilLat, 0) +
                     " carLat=" + String(latCarMM, 0) + " need=" + String(need, 0) + " fwd=" + String(fwd, 0) + " -> theta=" + String(sgn * th) +
                     " (max gain " + String(lateralGain(THETA_MAX, fwd, yawNow), 0) + (pMeas ? "" : ", clipped block") + ")");
  return sgn * th;
}
void setSwerveHeading(float thetaSigned) {
  swerve_heading = target_heading + thetaSigned;
  if (swerve_heading >= 360) swerve_heading -= 360;
  if (swerve_heading <    0) swerve_heading += 360;
}

int pillarCount = 0;
void acquirePillar(float offRightDeg, int n) {
  pillarCount = n;
  planTheta = planSwerve(offRightDeg, true);
  pillarFwdMM = planPassMM; pillarFwdFirst = planPassMM; pushCapLogged = false; holdFromCap = false;
  commitStartTicks = encoderTicks;
  refX = pX; refBottom = pY + pH / 2; refMs = millis();
  setSwerveHeading(planTheta);
  pState = P_COMMIT;
  logLine(String("PILLAR #") + pillarCount + " " + (pColor == RED_ID ? "RED" : "GREEN") + " x=" + String(pX / (float)FRAME_W, 2) + " bottom=" + String(pY + pH / 2) +
          "  -> SWERVE " + String(planTheta, 0) + " deg hdg=" + String(swerve_heading, 1) + "  carLat=" + String(latCarMM, 0) + " yaw=" + String(offRightDeg, 1) + " mm=" + String(mmSince(0), 0));
}

void stopRun(const String &why);
void enterPhase(PillarState s, const char* name);

void startBack(float maxMM, bool forPillar, const char* why) {
  driveMotor(0, true); delay(100);
  backMaxMM = maxMM; backForPillar = forPillar;
  if (forPillar) backupUsed = true;
  long t; noInterrupts(); t = encoderTicks; interrupts();
  backLastTicks = t; backLastMove = millis();
  logLine(String("  BACK ") + String(maxMM, 0) + " mm: " + why + "  h_err=" + String(getHeadingError(target_heading, readHeading()), 1) + " mm=" + String(mmSince(0), 0));
  enterPhase(P_BACK, "BACK");
}
bool backupWanted() {
  if (TURN == 0 || backupUsed) return false;
  float leg = mmSince(legStartTicks);
  if (leg > BACKUP_WINDOW_MM) return false;
  if (pY + pH / 2 < BACKUP_BOTTOM) return false;
  float xf = pX / (float)FRAME_W;
  return (pColor == RED_ID) ? (xf >= BACKUP_WRONG_X) : (xf <= 1.0 - BACKUP_WRONG_X);
}

float legMM() { return mmSince(legStartTicks); }
bool decideTurn(float distF, float distL, float distR) {
  if (distF > TURN_THRESHOLD_CM || distF <= 2.0) return false;
  if (millis() - lastTurnEndTime < TURN_COOLDOWN_MS) return false;
  if (TURN > 0 && MIN_LEG_MM > 0 && legMM() < MIN_LEG_MM) return false;
  bool rightOpen = (rOpenCnt >= SIDE_OPEN_MIN), leftOpen = (lOpenCnt >= SIDE_OPEN_MIN);
  if (lockedTurnDirection == 0) {
    if (rightOpen && !leftOpen)      { lockedTurnDirection =  1; logLine("LOCKED: CLOCKWISE");      return true; }
    else if (leftOpen && !rightOpen) { lockedTurnDirection = -1; logLine("LOCKED: ANTI-CLOCKWISE"); return true; }
    else if (distF <= BLIND_LOCK_CM) { lockedTurnDirection = (distR > distL) ? 1 : -1; logLine(String("BLIND LOCK: ") + (lockedTurnDirection == 1 ? "CW" : "CCW")); return true; }
    return false;
  }
  if (lockedTurnDirection ==  1 && (rightOpen || distF <= BLIND_LOCK_CM)) return true;
  if (lockedTurnDirection == -1 && (leftOpen  || distF <= BLIND_LOCK_CM)) return true;
  return false;
}
void startTurn() {
  isTurning = true; turnStartTime = millis(); setNeoPixels(255, 255, 200);
  turnDirection = lockedTurnDirection;
  if (turnDirection == 1) { target_heading += 90.0; currentServoAngle = SERVO_SHIFT_RIGHT; }
  else                    { target_heading -= 90.0; currentServoAngle = SERVO_SHIFT_LEFT;  }
  if (target_heading >= 360.0) target_heading -= 360.0;
  if (target_heading <    0.0) target_heading += 360.0;
  turnPhase = 0; turnStartHeading = readHeading();
  turnExitHeading = target_heading + turnDirection * TURN_THROUGH_DEG;
  if (turnExitHeading >= 360.0) turnExitHeading -= 360.0;
  if (turnExitHeading <    0.0) turnExitHeading += 360.0;
  logLine("TURN " + String(TURN + 1) + " start, leg " + String(legMM(), 0) + " mm, new lane hdg=" + String(target_heading, 1) + " exit via " + String(turnExitHeading, 1));
}
float laneCentreLat(float distL, float distR) {
  if (distL < LANE_ANCHOR_SEE_CM && distR < LANE_ANCHOR_SEE_CM) return (distL - distR) * 5.0;
  return latCarMM;
}
void endTurn(const char* why, float distL, float distR) {
  isTurning = false; TURN++;
  lastTurnEndTime = millis(); legStartTicks = encoderTicks;
  latCarMM = laneCentreLat(distL, distR); fwdCarMM = 0; pillarCount = 0;
  pState = P_NONE;
  backupUsed = false;
  currentServoAngle = SERVO_CENTER_DEG; steeringServo.write(currentServoAngle);
  setNeoPixels(255, 255, 200);
  logLine(String("TURN ") + TURN + " done (" + why + ")  lane centre anchor carLat=" + String(latCarMM, 0));
}

bool turnStep(float laneError, float currentHeading, float distF, float distL, float distR) {
    float turned = fabs(getHeadingError(currentHeading, turnStartHeading));
    float err = getHeadingError(target_heading, currentHeading);
    if (turnPhase == 0) {
      steeringServo.write(currentServoAngle); driveMotor(TURN_SPEED, true);
      { static long lastT = 0; static unsigned long lastMove = 0; long t; noInterrupts(); t = encoderTicks; interrupts();
        if (t != lastT) { lastT = t; lastMove = millis(); }
        if (lastMove == 0) lastMove = millis();
        if (millis() - lastMove > STALL_MS) { logLine("  split turn: STALL in fwd half after " + String(turned, 0) + " deg -> reversing"); turned = TURN_FWD_DEG; lastMove = millis(); }
      }
      if (turned >= TURN_FWD_DEG) {
        driveMotor(0, true); delay(150);
        currentServoAngle = (turnDirection == 1) ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;
        steeringServo.write(currentServoAngle); delay(200);
        turnPhase = 1; revStartTicks = encoderTicks;
        logLine("  split turn: fwd " + String(turned, 0) + " deg done, reversing  mm=" + String(mmSince(0), 0));
      }
      return false;
    }

    float reversed = -mmSince(revStartTicks);
    if (fabs(err) > SPLIT_REV_BLEND_DEG) { steeringServo.write(currentServoAngle); }
    else { setServo(SERVO_CENTER_DEG - headingPD(err)); }
    driveMotor(SPLIT_REV_SPEED, false);
    bool headingOK = fabs(err) < TURN_EXIT_DEG;
    if ((headingOK && reversed >= REV_MIN_MM) || millis() - turnStartTime > TURN_TIMEOUT_MS) {
      driveMotor(0, true); delay(150);
      logLine("  split turn: reverse done h_err=" + String(err, 1) + " reversed=" + String(reversed, 0) + " mm  mm=" + String(mmSince(0), 0));
      endTurn(headingOK ? "split" : "timeout", distL, distR);
    }
    return false;
}

bool phaseDone(unsigned long ms, float mm) {
  return mmSince(phaseStartTicks) >= mm;
}
void enterPhase(PillarState s, const char* name) {
  pState = s; phaseStartMs = millis(); phaseStartTicks = encoderTicks;
  logLine(String("  -> ") + name);
}

bool pillarStep(float headingError, float currentHeading, float distL, float distR) {
  bool seen = (pState == P_BACK) ? false : readPillar(pState != P_NONE);

  switch (pState) {
    case P_NONE:
      if (seen) {
        if (backupWanted()) {
          startBack(BACKUP_MAX_MM, true, (String(pColor == RED_ID ? "RED" : "GREEN") + " x=" + String(pX / (float)FRAME_W, 2) + " bottom=" + String(pY + pH / 2) + " too close, must cross").c_str());
          return true;
        }
        acquirePillar(-headingError, 1);
      }
      return false;

    case P_COMMIT:
      if (seen && pMeas) {
        float th = planSwerve(-headingError, false);
        float pass = min(planPassMM, pillarFwdFirst + PASS_PUSH_MAX_MM);
        if (planPassMM > pass && !pushCapLogged) { logLine("  pass point capped at first plan +" + String(PASS_PUSH_MAX_MM) + " mm (asked +" + String(planPassMM - pillarFwdFirst, 0) + ")"); pushCapLogged = true; }
        pillarFwdMM = pass;
        bool take = (fabs(th - planTheta) >= 2);
        if (take) { planTheta = th; setSwerveHeading(planTheta); logLine("  re-plan theta=" + String(planTheta, 0) + " hdg=" + String(swerve_heading, 1)); }
      }
      driveArcToHeading(swerve_heading, currentHeading, PILLAR_SPEED);

      {
        bool capped = (mmSince(commitStartTicks) >= COMMIT_CAP_MM);
        if (fwdCarMM >= pillarFwdMM || capped) {
          holdFromCap = capped && fwdCarMM < pillarFwdMM;
          enterPhase(P_HOLD, "HOLD");
          logLine(String("     ") + (capped && fwdCarMM < pillarFwdMM ? "COMMIT CAP " + String(COMMIT_CAP_MM) + " mm reached" : String("nose level with pillar")) +
                  ": fwd=" + String(fwdCarMM, 0) + " lat=" + String(latCarMM, 0) + " h_err=" + String(headingError, 1) + (seen ? " (pillar still in view)" : " (pillar out of view)"));
        }
      }
      return true;

    case P_HOLD:
      {
        int lastColor = pColor;
        bool newSeen = readPillar(false);
        if (newSeen && pColor == lastColor && !holdFromCap) { acquirePillar(-headingError, pillarCount + 1); return true; }
        pColor = lastColor;
      }
      driveArcToHeading(swerve_heading, currentHeading, PILLAR_SPEED);
      if (phaseDone(PASS_HOLD_MS, PASS_HOLD_MM)) { returnTargetLat = latCarMM * (1.0 - RETURN_FRACTION); enterPhase(P_RECOVER, "RECOVER"); logLine("     return target carLat=" + String(returnTargetLat, 0) + " from " + String(latCarMM, 0)); }
      return true;

    case P_BACK: {

      setServo(SERVO_CENTER_DEG - headingPD(headingError));
      driveMotor(SPLIT_REV_SPEED, false);
      long t; noInterrupts(); t = encoderTicks; interrupts();
      if (t != backLastTicks) { backLastTicks = t; backLastMove = millis(); }
      float backed = -mmSince(phaseStartTicks);
      bool stall   = (millis() - backLastMove > BACK_STALL_MS);
      bool timeout = (millis() - phaseStartMs > BACK_TIMEOUT_MS);
      bool clean   = false;
      if (!backForPillar) { pSeen = false; pMeas = false; pRej = 0; }
      if (backForPillar) {
        bool s = readPillar(false);
        clean = s && pMeas && (pY + pH / 2 <= BACKUP_OK_BOTTOM);
      }
      if (backed >= backMaxMM || stall || timeout || clean) {
        driveMotor(0, true); delay(150);
        logLine(String("  back done: ") + (clean ? "clean view" : backed >= backMaxMM ? "max distance" : stall ? "stall (wall behind)" : "timeout") +
                " backed=" + String(backed, 0) + " mm h_err=" + String(headingError, 1) + " mm=" + String(mmSince(0), 0));
        enterPhase(P_NONE, "NONE");
        if (clean) acquirePillar(-headingError, 1);
      }
      return true;
    }

    case P_RECOVER:
      {
        bool nextSeen = readPillar(false);
        if (nextSeen && fabs(headingError) <= ACQ_STRAIGHT_DEG) { acquirePillar(-headingError, pillarCount + 1); return true; }
        float arcBack = TURN_R_MM * (1 - cos(radians(RETURN_DEG)));
        float err = latCarMM - returnTargetLat;
        float hdg;
        if (nextSeen)                                  hdg = target_heading;
        else if (fabs(err) > arcBack + RETURN_TOL_MM)  hdg = target_heading + (err > 0 ? -RETURN_DEG : RETURN_DEG);
        else                                           hdg = target_heading;
        if (hdg >= 360) hdg -= 360; if (hdg < 0) hdg += 360;
        driveArcToHeading(hdg, currentHeading, OBST_SPEED);
        bool onLine = (fabs(latCarMM - returnTargetLat) <= RETURN_TOL_MM) && (fabs(headingError) < 5);
        if (!nextSeen && (onLine || mmSince(phaseStartTicks) >= RETURN_MAX_MM)) {
          enterPhase(P_NONE, onLine ? "NONE (back on line)" : "NONE (return timeout)");
          logLine("     carLat=" + String(latCarMM, 0) + " h_err=" + String(headingError, 1) + " mm=" + String(mmSince(0), 0));
        }
      }
      return true;
  }
  return false;
}

const char* pStateName() {
  switch (pState) { case P_NONE: return "NONE"; case P_COMMIT: return "COMMIT"; case P_HOLD: return "HOLD"; case P_RECOVER: return "RECOVER"; case P_BACK: return "BACK"; }
  return "?";
}

bool stalledForward() {
  static long lastT = 0; static unsigned long lastMove = 0; static bool wasFwd = false;
  long t; noInterrupts(); t = encoderTicks; interrupts();
  bool fwdCmd = (motorCmdSpeed > 0 && motorCmdFwd);
  if (!fwdCmd || !wasFwd || t != lastT) { lastT = t; lastMove = millis(); }
  wasFwd = fwdCmd;
  return fwdCmd && (millis() - lastMove > STALL_MS);
}

void stopRun(const String &why) {
  driveMotor(0, true); steeringServo.write(SERVO_CENTER_DEG); setNeoPixels(255, 0, 0);
  logLine("STOP " + why + " t=" + String(millis() - runStartTime) + " ms  mm=" + String(mmSince(0), 0));
  logFlush(); while (1) delay(1000);
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin(BT_NAME);
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT); pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP); pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

  strip.begin(); setNeoPixels(255, 0, 0);
  Wire.begin(); Wire.setClock(100000);
  steeringServo.setPeriodHertz(50); steeringServo.attach(SERVO_PIN, 500, 2400);
  centerWheels(); driveMotor(0, true);

  while (!husky.begin(Wire)) { logLine("HuskyLens not found, retrying"); delay(500); }
  for (int i = 0; i < 3; i++) { tcaselect(i); if (!tof[i].begin()) { logLine("ToF " + String(i) + " missing"); while (1) { setNeoPixels(255,255,0); delay(100); setNeoPixels(0,0,0); delay(100); } } }
  tofStartAll();
  tcaselect(BNO_CH);
  if (!bno.begin()) { logLine("BNO055 missing"); while (1) { setNeoPixels(255,255,0); delay(100); setNeoPixels(0,0,0); delay(100); } }
  delay(100); bno.setExtCrystalUse(true);

  setNeoPixels(255, 255, 200);
  logLine(String("STEP 4 v14b slim ready (MOTOR_ON=") + MOTOR_ON + "). Press START."); logFlush();
  while (digitalRead(START_BTN_PIN) == HIGH) delay(50);
  delay(50); while (digitalRead(START_BTN_PIN) == LOW) delay(10);

  target_heading = readHeading();
  prevHeadingError = 0; prevHeadingTime = millis();
  encoderTicks = 0; latLastTicks = 0; latCarMM = 0; fwdCarMM = 0; legStartTicks = 0; runStartTime = millis();
  { float dL = getDistance(TOF_LEFT_CH), dR = getDistance(TOF_RIGHT_CH); delay(40); dL = getDistance(TOF_LEFT_CH); dR = getDistance(TOF_RIGHT_CH);
    latCarMM = laneCentreLat(dL, dR); logLine("start lane anchor carLat=" + String(latCarMM, 0) + " (L=" + String(dL, 0) + " R=" + String(dR, 0) + ")"); }
  setNeoPixels(255, 255, 200);
  logLine("heading locked " + String(target_heading, 1));
  logLine("t_ms,mm,state,F,L,R,hdg,tgt,h_err,servo,col,x,x_frac,y,bottom,h,w,blocks,carLat,turn,meas,rej");
}

void loop() {
  float distF = getDistance(TOF_FRONT_CH);
  float distR = getDistance(TOF_RIGHT_CH);
  float distL = getDistance(TOF_LEFT_CH);
  float currentHeading = readHeading();
  float headingError = getHeadingError(target_heading, currentHeading);
  trackLateral(-headingError);
  rOpenCnt = (distR > SIDE_GAP_CM) ? rOpenCnt + 1 : 0;
  lOpenCnt = (distL > SIDE_GAP_CM) ? lOpenCnt + 1 : 0;

  if (TURN >= TURNS_TO_RUN && pState == P_NONE && !isTurning && legMM() >= FINISH_MM)
    stopRun("finished: " + String(TURN) + " corners, " + String(legMM(), 0) + " mm into start section");

  { static bool lostLogged = false;
    if (!isTurning && fabs(headingError) > LOST_DEG) {
      if (!lostLogged) { logLine("LOST: hdg=" + String(currentHeading, 1) + " lane=" + String(target_heading, 1) + " err=" + String(headingError, 1) + " in " + pStateName() + " -> PD to lane"); lostLogged = true; }
      pState = P_NONE;
    } else if (fabs(headingError) < LOST_DEG - 10) lostLogged = false;
  }

  bool cornerAllowed = (pState == P_NONE || pState == P_RECOVER) && !pSeen;
  if (isTurning) {
    turnStep(headingError, currentHeading, distF, distL, distR);
  } else if (cornerAllowed && decideTurn(distF, distL, distR)) {
    if (pState != P_NONE) logLine(String("  corner during ") + pStateName() + ": pillar phase abandoned");
    pState = P_NONE; startTurn();
  } else if (!pillarStep(headingError, currentHeading, distL, distR)) {
    driveStraightStep(headingError, distL, distR, OBST_SPEED);
    if (LANE_ANCHOR_GAIN > 0) latCarMM += LANE_ANCHOR_GAIN * (laneCentreLat(distL, distR) - latCarMM);
  }

  logLine(String(millis() - runStartTime) + "," + String(mmSince(0), 0) + "," + pStateName() + "," + String(distF, 0) + "," + String(distL, 0) + "," + String(distR, 0) + "," +
          String(currentHeading, 1) + "," + String(target_heading, 1) + "," + String(headingError, 1) + "," + String(currentServoAngle) + "," +
          (pSeen ? (pColor == RED_ID ? "R" : "G") : "-") + "," +
          String(pSeen ? pX : 0) + "," + String(pSeen ? pX / (float)FRAME_W : 0, 2) + "," +
          String(pSeen ? pY : 0) + "," + String(pSeen ? pY + pH / 2 : 0) + "," +
          String(pSeen ? pH : 0) + "," + String(pSeen ? pW : 0) + "," + String(pCount) + "," + String(latCarMM, 0) + "," + (isTurning ? "T" : "") + String(TURN) + "," +
          (pSeen ? (pMeas ? "M" : "C") : "-") + "," + String(pRej));

#if MOTOR_ON

  if (!isTurning && pState != P_BACK && stalledForward()) {
    logLine(String("STALL in ") + pStateName() + ": encoder frozen " + STALL_MS + " ms  F=" + String(distF, 0) + " h_err=" + String(headingError, 1));
    startBack(UNSTICK_MM, false, "unstick");
  }
#endif
#if MOTOR_ON
  if (pState == P_NONE && !isTurning && distF <= STOP_FRONT_CM && distF > 2.0)
    stopRun("front " + String(distF, 0) + " cm");
#endif
  if (millis() - runStartTime > STOP_TIMEOUT_MS) stopRun("timeout");
  if (digitalRead(START_BTN_PIN) == LOW)         stopRun("button");
}
