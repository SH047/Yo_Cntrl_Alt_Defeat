// WRO FE 2026 - Obstacle STEP 5 v8 (USE_LINE_GATE switch, default off; pillar shape 1.2 / 110): LINE_SEARCH creep when the line is missing, 'closed' side = seen closed in last SIDE_CLOSED_MM, slow near walls, 70 for set-up/turn/reverse (rear ToF on mux ch 6; FIRST_LEG_MIN_MM for the first corner; optional per-bulge leg minimums): straight-run bulges (v7) + gated precision corners (corner test v1).
// Leg = STRAIGHT (pillar trigger armed) -> BULGE (locked) -> SETTLE -> STRAIGHT ... -> CORNER when ALL gates pass:
//   distance gate (leg >= LEG_MIN_MM), sides gate (turn side open on 2 readings, other side closed, car within CORNER_MAX_ERR of lane),
//   front gate (F <= CORNER_TRIGGER_CM), line gate (clockwise: BLUE seen >= LINE_MIN_W wide in last LINE_WINDOW_MM; anticlockwise: ORANGE; first corner: either), not in any manoeuvre.
// Corner = STOP + measure -> ADJUST to FRONT_SET_CM -> TURN 90 (forward arc) -> REVERSE until rear ToF reads REAR_SET_CM (or REVERSE_MM if REAR_SET_CM is 0 / rear out of range) -> next leg.
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
#define TOF_REAR_CH    6
#define BNO_CH         3

// ---- pillar / bulge ----
#define TRIGGER_BOTTOM     130    // start the bulge when the pillar's bottom (y + h/2) reaches this
#define TRIGGER_FRAMES       3
#define TRIGGER_MAX_TURN     5    // only trigger within this many degrees of the lane heading
#define MIN_PILLAR_H        12
#define PILLAR_MAX_W       110    // a pillar block is never wider than this (near pillars 55-83)
#define PILLAR_HW_MIN      1.2    // pillar: h >= 1.2 * w (real pillars 1.3-1.9; the orange-line-as-red block is ~1.0)
#define CENTER_TOL        0.12
#define THETA_CENTER        45
#define THETA_FAR           65
#define THETA_NEAR          30
#define PEAK_MM            250
#define BULGE_SERVO_DEG     38    // mirrored limits below are +/-38
#define EXIT_EARLY_DEG       5    // overshoot after a lock flip is ~3 deg (opposite lock stops the rotation)
#define SETTLE_MM          200    // straight after a bulge, nothing armed
#define MAX_BULGES_PER_LEG   2
// ---- corner gates ----
#define LEG_MIN_MM        2400    // corner allowed only after this encoder distance from the last corner's reverse point (any number of bulges)
#define FIRST_LEG_MIN_MM   300    // first corner only: the car starts mid-section, so the first wall can come much sooner
#define USE_BULGE_LEGS       0    // 0 = distance gate uses LEG_MIN_MM for every leg; 1 = uses LEG_MM_0/1/2 by the number of bulges in this leg
#define LEG_MM_0          2400    // USE_BULGE_LEGS=1: no bulge in this leg
#define LEG_MM_1          3200    // USE_BULGE_LEGS=1: one bulge
#define LEG_MM_2          4000    // USE_BULGE_LEGS=1: two bulges
#define USE_LINE_GATE        0    // 0 = corner does not need a line (lines still logged); 1 = line gate + LINE_SEARCH active
#define LINE_WINDOW_MM     600    // the line must have been seen within this travel
#define LINE_WINDOW2_MM   1500    // LINE_SEARCH fallback: if the creep reaches LINE_SEARCH_MIN_CM without a line, accept the corner if the line was seen within this
#define LINE_SEARCH_MIN_CM  25    // LINE_SEARCH: creep forward until the line shows, or until F reaches this
#define SIDE_CLOSED_MM     500    // the other side counts as closed if it read < SIDE_OPEN_CM at any point in the last this many mm
#define NEAR_WALL_CM        80    // on a straight, slow down to NEAR_SPEED when F is below this
#define NEAR_SPEED          70
#define LINE_MIN_W         130    // a line block must be at least this wide (near pillars reach ~110)
#define SIDE_OPEN_CM        70    // side reads open when > this (999 counts)
#define SIDE_OPEN_COUNT      2
#define CORNER_MAX_ERR       5    // car within this many degrees of the lane heading
#define CORNER_TRIGGER_CM   60    // front <= this
#define SAFETY_STOP_CM      12    // front this close with gates failing -> stop the run
// ---- corner sequence ----
#define TURN_DIR             0    // +1 right / -1 left / 0 auto: locked at the first corner from the open side
#define FRONT_SET_CM        50    // distance to set from the front wall before the turn
#define FRONT_TOL_CM         1
#define ADJUST_SPEED        70
#define TURN_DEG            90
#define TURN_SERVO_DEG      38
#define TURN_EARLY_DEG      18
#define TURN_SPEED          70
#define REAR_SET_CM          0    // after the turn, reverse until the rear ToF reads this (0 = ignore rear, use REVERSE_MM)
#define REAR_TOL_CM          2
#define REVERSE_MM         300    // fallback: straight reverse distance when the rear ToF is not used or reads 999 (0 = none)
#define REVERSE_MAX_MM     600    // never reverse more than this
#define REVERSE_SPEED       70
#define CORNERS_TO_RUN      12
#define FINISH_MM         1000    // after the last corner: run the normal straight (pillars still handled, corners off) and stop once leg >= this and no manoeuvre is running
// ---- general ----
#define SPEED               80
#define HEADING_KP         1.2
#define RUN_TIMEOUT_MS  200000
#define SERVO_CENTER_DEG   104
#define SERVO_MIN_DEG       66
#define SERVO_MAX_DEG      142
#define TICKS_PER_METER  657.0
#define BT_NAME   "YoLabs-FE"
#define LOG_BATCH            8
#define BLUE_ID 1
#define ORANGE_ID 2
#define RED_ID 4
#define GREEN_ID 5

Adafruit_VL53L0X tof[4];
Adafruit_BNO055  bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo            steeringServo;
BluetoothSerial  SerialBT;
Adafruit_NeoPixel strip(16, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
HUSKYLENS        husky;

enum Phase { STRAIGHT, ARC1, ARC2, PEAK, ARC3, ARC4, SETTLE, LINE_SEARCH, C_ADJUST, C_TURN, C_REVERSE };
const char* phaseNames[] = { "STRAIGHT", "ARC1", "ARC2", "PEAK", "ARC3", "ARC4", "SETTLE", "LINE_SEARCH", "C_ADJUST", "C_TURN", "C_REVERSE" };
Phase phase = STRAIGHT;
volatile long encoderTicks = 0;
float laneHeading = 0, fwdMM = 0; long fwdLastTicks = 0;
long  phaseTicks = 0, legStartTicks = 0; unsigned long phaseMs = 0, runStart = 0;
int   servoNow = SERVO_CENTER_DEG;
float lastDist[4] = {999, 999, 999, 999};
String logBuf; int logLines = 0;
bool  bSeen[7]; int bX[7], bY[7], bW[7], bH[7];
int   bulgeDir = 0, bulgeTheta = THETA_CENTER, seenCount = 0, seenId = 0, bulgesThisLeg = 0, pillarCount = 0;
float lastLineMM[3] = {-100000, -100000, -100000};   // index by ID: 1 blue, 2 orange
int   turnDir = TURN_DIR, corners = 0, lOpen = 0, rOpen = 0;
float lClosedMM = -100000, rClosedMM = -100000;   // last distance at which each side read closed
float turnStartHeading = 0;
bool  gateLogged = false, finishing = false;

void tcaselect(uint8_t i) { Wire.beginTransmission(TCAADDR); Wire.write(1 << i); Wire.endTransmission(); }
void setNeoPixels(uint8_t r, uint8_t g, uint8_t b) { for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, strip.Color(r, g, b)); strip.show(); }
void logLine(const String &s) { Serial.println(s); logBuf += s; logBuf += '\n'; if (++logLines >= LOG_BATCH) { SerialBT.print(logBuf); logBuf = ""; logLines = 0; } }
void logFlush() { if (logLines) { SerialBT.print(logBuf); logBuf = ""; logLines = 0; } }
void IRAM_ATTR encoderISR() { if (digitalRead(ENCODER_B_PIN) == HIGH) encoderTicks++; else encoderTicks--; }
float mmSince(long ref) { long t; noInterrupts(); t = encoderTicks; interrupts(); return (t - ref) * (1000.0 / TICKS_PER_METER); }
float legMM() { return mmSince(legStartTicks); }
void driveMotor(int speed, bool fwd) { if (speed == 0) { analogWrite(IN1_PIN, 0); digitalWrite(IN2_PIN, LOW); } else { digitalWrite(IN2_PIN, fwd ? HIGH : LOW); analogWrite(IN1_PIN, speed); } }
void setServo(int deg) { servoNow = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG); steeringServo.write(servoNow); }
float readHeading() { tcaselect(BNO_CH); sensors_event_t e; bno.getEvent(&e); return e.orientation.x; }
float wrap180(float d) { while (d > 180) d -= 360; while (d < -180) d += 360; return d; }
int tofIdx(uint8_t ch) { return (ch == TOF_REAR_CH) ? 3 : ch; }
float getDistance(uint8_t ch) {
  int i = tofIdx(ch); tcaselect(ch);
  if (tof[i].isRangeComplete()) { uint16_t mm = tof[i].readRangeResult(); lastDist[i] = (mm > 0 && mm <= 2000) ? mm / 10.0 : 999.0; }
  return lastDist[i];
}
float frontAverage(int n) { float sum = 0; int got = 0; for (int k = 0; k < n; k++) { delay(35); float f = getDistance(TOF_FRONT_CH); if (f < 999) { sum += f; got++; } } return got ? sum / got : 999; }
bool isLineShape(int i)   { return bW[i] >= LINE_MIN_W; }
bool isPillarShape(int i) { return bH[i] >= MIN_PILLAR_H && bW[i] <= PILLAR_MAX_W && bH[i] >= PILLAR_HW_MIN * bW[i]; }
void readCamera() {
  for (int i = 0; i < 7; i++) bSeen[i] = false;
  if (!husky.request()) return;
  while (husky.available()) {
    HUSKYLENSResult r = husky.read();
    if (r.command != COMMAND_RETURN_BLOCK || r.ID < 1 || r.ID > 6) continue;
    int i = r.ID;
    if (bSeen[i] && r.width * r.height <= bW[i] * bH[i]) continue;
    bSeen[i] = true; bX[i] = r.xCenter; bY[i] = r.yCenter; bW[i] = r.width; bH[i] = r.height;
  }
  for (int i = BLUE_ID; i <= ORANGE_ID; i++) if (bSeen[i] && isLineShape(i)) lastLineMM[i] = mmSince(0);
}
String camCol(int i) { return bSeen[i] ? String(bX[i]) + "/" + bY[i] + "/" + bW[i] + "/" + bH[i] : String("-"); }
void enterPhase(Phase p, float turned) { phase = p; phaseTicks = encoderTicks; phaseMs = millis(); logLine(String("-> ") + phaseNames[p] + "  turned=" + String(turned, 1) + " mm=" + String(mmSince(0), 0) + " leg=" + String(legMM(), 0) + " t=" + String(millis() - runStart)); }
void finish(const String &why) {
  driveMotor(0, true); setServo(SERVO_CENTER_DEG);
  logLine("STOP " + why + "  corners=" + corners + " pillars=" + pillarCount + " mm=" + String(mmSince(0), 0) + " t=" + String(millis() - runStart));
  logFlush(); while (1) delay(1000);
}

void setup() {
  Serial.begin(115200); SerialBT.begin(BT_NAME);
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT); pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP); pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);
  strip.begin(); setNeoPixels(250, 250, 200);
  Wire.begin(); Wire.setClock(100000);
  steeringServo.setPeriodHertz(50); steeringServo.attach(SERVO_PIN, 500, 2400);
  setServo(SERVO_CENTER_DEG); driveMotor(0, true);
  while (!husky.begin(Wire)) { logLine("HuskyLens not found, retrying"); delay(500); }
  { const uint8_t chs[4] = { TOF_FRONT_CH, TOF_RIGHT_CH, TOF_LEFT_CH, TOF_REAR_CH };
    for (int i = 0; i < 4; i++) { tcaselect(chs[i]); if (!tof[i].begin()) { logLine(String("ToF ch") + chs[i] + " missing"); while (1) delay(1000); } tof[i].startRangeContinuous(30); } }
  tcaselect(BNO_CH);
  if (!bno.begin()) { logLine("BNO055 missing"); while (1) delay(1000); }
  delay(100); bno.setExtCrystalUse(true);

  logLine(String("STEP 5 v8 ready: linegate=") + USE_LINE_GATE + "  legmin=") + (USE_BULGE_LEGS ? String(LEG_MM_0) + "/" + LEG_MM_1 + "/" + LEG_MM_2 + " by bulges" : String(LEG_MIN_MM)) + ", first " + FIRST_LEG_MIN_MM + ", corners=" + CORNERS_TO_RUN + "+" + FINISH_MM + "mm" + " rear=" + REAR_SET_CM + "cm" + " trig F<=" + CORNER_TRIGGER_CM + " set=" + FRONT_SET_CM + " rev=" + REVERSE_MM + " theta C/F/N=" + THETA_CENTER + "/" + THETA_FAR + "/" + THETA_NEAR + ". Press START."); logFlush();
  while (digitalRead(START_BTN_PIN) == HIGH) delay(50);
  delay(50); while (digitalRead(START_BTN_PIN) == LOW) delay(10);
  laneHeading = readHeading(); encoderTicks = 0; runStart = millis(); legStartTicks = 0; fwdLastTicks = 0;
  logLine("start hdg=" + String(laneHeading, 1));
  logLine("t_ms,mm,leg,fwd,phase,phase_mm,F,L,R,B,hdg,h_err,servo,bulges,ID1_blue,ID2_orange,ID3_magenta,ID4_red,ID5_green,ID6");
}

void loop() {
  float F = getDistance(TOF_FRONT_CH), R = getDistance(TOF_RIGHT_CH), L = getDistance(TOF_LEFT_CH), B = getDistance(TOF_REAR_CH);
  float hdg = readHeading();
  float hErr = wrap180(laneHeading - hdg);                       // + = car must turn right
  { long t; noInterrupts(); t = encoderTicks; interrupts(); fwdMM += (t - fwdLastTicks) * (1000.0 / TICKS_PER_METER) * cos(radians(hErr)); fwdLastTicks = t; }
  rOpen = (R > SIDE_OPEN_CM) ? rOpen + 1 : 0;  lOpen = (L > SIDE_OPEN_CM) ? lOpen + 1 : 0;
  if (R <= SIDE_OPEN_CM) rClosedMM = mmSince(0);  if (L <= SIDE_OPEN_CM) lClosedMM = mmSince(0);
  readCamera();
  float turned = (bulgeDir == 0 ? 1 : bulgeDir) * wrap180(hdg - laneHeading);   // + = rotated toward the bulge side

  switch (phase) {
    case STRAIGHT: {
      setServo(SERVO_CENTER_DEG + HEADING_KP * hErr); driveMotor((F <= NEAR_WALL_CM && F > 2) ? NEAR_SPEED : SPEED, true);
      if (finishing && legMM() >= FINISH_MM) finish("finished: " + String(legMM(), 0) + " mm into the start section");
      // ---- corner gates (off in the finish section) ----
      float legMin = LEG_MIN_MM;
      if (USE_BULGE_LEGS) legMin = (bulgesThisLeg == 0) ? LEG_MM_0 : (bulgesThisLeg == 1) ? LEG_MM_1 : LEG_MM_2;
      if (corners == 0) legMin = FIRST_LEG_MIN_MM;
      bool distOK  = legMM() >= legMin;
      bool rO = rOpen >= SIDE_OPEN_COUNT, lO = lOpen >= SIDE_OPEN_COUNT;
      bool rC = (mmSince(0) - rClosedMM) <= SIDE_CLOSED_MM, lC = (mmSince(0) - lClosedMM) <= SIDE_CLOSED_MM;
      bool sidesOK = fabs(hErr) <= CORNER_MAX_ERR && ((turnDir >= 0 && rO && lC && !lO) || (turnDir <= 0 && lO && rC && !rO));   // turn side open, other side seen closed recently
      bool frontOK = (F <= CORNER_TRIGGER_CM && F > 2);
      float blueAgo = mmSince(0) - lastLineMM[BLUE_ID], orangeAgo = mmSince(0) - lastLineMM[ORANGE_ID];
      bool lineOK  = !USE_LINE_GATE || ((turnDir == 1) ? (blueAgo <= LINE_WINDOW_MM) : (turnDir == -1) ? (orangeAgo <= LINE_WINDOW_MM) : (blueAgo <= LINE_WINDOW_MM || orangeAgo <= LINE_WINDOW_MM));
      if (!finishing && frontOK && !gateLogged) { gateLogged = true;
        logLine(String("front ") + String(F, 0) + " cm: dist=" + distOK + " sides=" + sidesOK + " line=" + lineOK + " (L=" + String(L, 0) + " R=" + String(R, 0) + " h_err=" + String(hErr, 1) + " leg=" + String(legMM(), 0) + "/" + String(legMin, 0) + " bulges=" + bulgesThisLeg + " blue " + String(blueAgo, 0) + " mm ago, orange " + String(orangeAgo, 0) + " mm ago)"); }
      if (F > CORNER_TRIGGER_CM) gateLogged = false;
      if (!finishing && frontOK && distOK && sidesOK && !lineOK) {
        driveMotor(0, true); setServo(SERVO_CENTER_DEG); delay(300);
        if (turnDir == 0) { turnDir = rO ? 1 : -1; logLine(String("LOCKED: ") + (turnDir == 1 ? "CLOCKWISE" : "ANTI-CLOCKWISE")); }
        logLine("LINE_SEARCH: front/dist/sides ok, line missing (blue " + String(blueAgo, 0) + ", orange " + String(orangeAgo, 0) + " mm ago) -> creep to find it");
        enterPhase(LINE_SEARCH, hErr); break;
      }
      if (!finishing && frontOK && distOK && lineOK && sidesOK) {
        driveMotor(0, true); setServo(SERVO_CENTER_DEG); delay(300);
        if (turnDir == 0) { turnDir = rO ? 1 : -1; logLine(String("LOCKED: ") + (turnDir == 1 ? "CLOCKWISE" : "ANTI-CLOCKWISE")); }
        float fAvg = frontAverage(10);
        logLine(String("CORNER ") + (corners + 1) + ": F(moving)=" + String(F, 0) + " F(stopped)=" + String(fAvg, 1) + " L=" + String(L, 0) + " R=" + String(R, 0) + " leg=" + String(legMM(), 0) + " fwd=" + String(fwdMM, 0) + " bulges=" + bulgesThisLeg + " blue " + String(blueAgo, 0) + " mm ago, orange " + String(orangeAgo, 0) + " mm ago");
        enterPhase(C_ADJUST, hErr); break;
      }
      if (F <= SAFETY_STOP_CM && F > 2) finish("front " + String(F, 0) + " cm, corner gates failed (dist=" + distOK + " sides=" + sidesOK + " line=" + lineOK + ")");
      // ---- pillar trigger ----
      if (bulgesThisLeg < MAX_BULGES_PER_LEG) {
        int id = 0, bottom = 0;
        if (bSeen[RED_ID]   && isPillarShape(RED_ID))   { id = RED_ID;   bottom = bY[RED_ID] + bH[RED_ID] / 2; }
        if (bSeen[GREEN_ID] && isPillarShape(GREEN_ID)) { int b = bY[GREEN_ID] + bH[GREEN_ID] / 2; if (id == 0 || b > bottom) { id = GREEN_ID; bottom = b; } }
        if (id != 0 && id == seenId) seenCount++; else { seenId = id; seenCount = (id != 0) ? 1 : 0; }
        if (id != 0 && seenCount >= TRIGGER_FRAMES && bottom >= TRIGGER_BOTTOM && fabs(hErr) <= TRIGGER_MAX_TURN) {
          bulgeDir = (id == RED_ID) ? 1 : -1;
          float xf = bX[id] / 320.0, side = bulgeDir * (xf - 0.5); const char* which;
          if (fabs(xf - 0.5) <= CENTER_TOL) { bulgeTheta = THETA_CENTER; which = "CENTER"; } else if (side > 0) { bulgeTheta = THETA_FAR; which = "FAR"; } else { bulgeTheta = THETA_NEAR; which = "NEAR"; }
          pillarCount++; bulgesThisLeg++;
          logLine(String("PILLAR #") + pillarCount + " " + (id == RED_ID ? "RED -> bulge RIGHT" : "GREEN -> bulge LEFT") + " theta=" + bulgeTheta + " (" + which + ") x=" + String(xf, 2) + " bottom=" + bottom + " w=" + bW[id] + " h=" + bH[id] + " F=" + String(F, 0) + " L=" + String(L, 0) + " R=" + String(R, 0) + " leg=" + String(legMM(), 0));
          enterPhase(ARC1, 0); setServo(SERVO_CENTER_DEG + bulgeDir * BULGE_SERVO_DEG);
        }
      }
      break; }
    case ARC1: setServo(SERVO_CENTER_DEG + bulgeDir * BULGE_SERVO_DEG); driveMotor(SPEED, true);
      if (turned >= bulgeTheta - EXIT_EARLY_DEG) { enterPhase(ARC2, turned); setServo(SERVO_CENTER_DEG - bulgeDir * BULGE_SERVO_DEG); } break;
    case ARC2: setServo(SERVO_CENTER_DEG - bulgeDir * BULGE_SERVO_DEG); driveMotor(SPEED, true);
      if (turned <= EXIT_EARLY_DEG) { enterPhase(PEAK, turned); setServo(SERVO_CENTER_DEG); } break;
    case PEAK: setServo(SERVO_CENTER_DEG + HEADING_KP * hErr); driveMotor(SPEED, true);
      if (mmSince(phaseTicks) >= PEAK_MM) { enterPhase(ARC3, turned); setServo(SERVO_CENTER_DEG - bulgeDir * BULGE_SERVO_DEG); } break;
    case ARC3: setServo(SERVO_CENTER_DEG - bulgeDir * BULGE_SERVO_DEG); driveMotor(SPEED, true);
      if (turned <= -(bulgeTheta - EXIT_EARLY_DEG)) { enterPhase(ARC4, turned); setServo(SERVO_CENTER_DEG + bulgeDir * BULGE_SERVO_DEG); } break;
    case ARC4: setServo(SERVO_CENTER_DEG + bulgeDir * BULGE_SERVO_DEG); driveMotor(SPEED, true);
      if (turned >= -EXIT_EARLY_DEG) { enterPhase(SETTLE, turned); setServo(SERVO_CENTER_DEG); bulgeDir = 0; seenCount = 0; seenId = 0; } break;
    case SETTLE: setServo(SERVO_CENTER_DEG + HEADING_KP * hErr); driveMotor(SPEED, true);
      if (mmSince(phaseTicks) >= SETTLE_MM) enterPhase(STRAIGHT, hErr); break;

    case LINE_SEARCH: {
      int lineId = (turnDir == -1) ? ORANGE_ID : BLUE_ID;
      float ago = mmSince(0) - lastLineMM[lineId];
      bool found = ago <= 50;                                   // seen in the last few loops
      bool tooClose = (F <= LINE_SEARCH_MIN_CM && F > 2) || millis() - phaseMs > 4000;
      if (found || tooClose) {
        driveMotor(0, true); setServo(SERVO_CENTER_DEG); delay(300);
        bool accept = found || (ago <= LINE_WINDOW2_MM);
        logLine(String("  line search: ") + (found ? "line found" : "no line") + " after " + String(mmSince(phaseTicks), 0) + " mm, F=" + String(F, 0) + ", last line " + String(ago, 0) + " mm ago -> " + (accept ? "CORNER" : "NOT a corner"));
        if (!accept) finish("front " + String(F, 0) + " cm, no line within " + String(LINE_WINDOW2_MM) + " mm -> no corner");
        float fAvg = frontAverage(10);
        logLine(String("CORNER ") + (corners + 1) + ": F(stopped)=" + String(fAvg, 1) + " L=" + String(L, 0) + " R=" + String(R, 0) + " leg=" + String(legMM(), 0) + " fwd=" + String(fwdMM, 0) + " bulges=" + bulgesThisLeg + " (after line search)");
        enterPhase(C_ADJUST, hErr);
      } else { setServo(SERVO_CENTER_DEG + HEADING_KP * hErr); driveMotor(ADJUST_SPEED, true); }
      break; }
    case C_ADJUST: {
      if (F >= 999) { driveMotor(0, true); if (millis() - phaseMs <= 4000) break; }   // no valid front reading yet: wait, don't decide
      float d = (F >= 999) ? 0 : F - FRONT_SET_CM;
      if (fabs(d) <= FRONT_TOL_CM || millis() - phaseMs > 4000) {
        driveMotor(0, true); delay(200);
        logLine("  set-up done: F=" + String(frontAverage(6), 1) + " (target " + FRONT_SET_CM + ") moved " + String(mmSince(phaseTicks), 0) + " mm h_err=" + String(hErr, 1) + (millis() - phaseMs > 4000 ? " TIMEOUT" : ""));
        turnStartHeading = readHeading(); setServo(SERVO_CENTER_DEG + turnDir * TURN_SERVO_DEG); delay(200);
        enterPhase(C_TURN, 0);
      } else if (d > 0) { setServo(SERVO_CENTER_DEG + HEADING_KP * hErr); driveMotor(ADJUST_SPEED, true); }
      else               { setServo(SERVO_CENTER_DEG - HEADING_KP * hErr); driveMotor(ADJUST_SPEED, false); }
      break; }
    case C_TURN: {
      float t = turnDir * wrap180(hdg - turnStartHeading);
      setServo(SERVO_CENTER_DEG + turnDir * TURN_SERVO_DEG); driveMotor(TURN_SPEED, true);
      if (t >= TURN_DEG - TURN_EARLY_DEG || millis() - phaseMs > 5000) {
        driveMotor(0, true); setServo(SERVO_CENTER_DEG); delay(200);
        laneHeading += turnDir * TURN_DEG; if (laneHeading >= 360) laneHeading -= 360; if (laneHeading < 0) laneHeading += 360;
        logLine("  turn done: turned=" + String(turnDir * wrap180(readHeading() - turnStartHeading), 1) + " in " + String(mmSince(phaseTicks), 0) + " mm  B=" + String(getDistance(TOF_REAR_CH), 0) + " F=" + String(getDistance(TOF_FRONT_CH), 0) + " L=" + String(getDistance(TOF_LEFT_CH), 0) + " R=" + String(getDistance(TOF_RIGHT_CH), 0) + " new lane hdg=" + String(laneHeading, 1));
        enterPhase(C_REVERSE, 0);
      }
      break; }
    case C_REVERSE: {
      bool useRear = (REAR_SET_CM > 0 && B < 999);
      bool done;
      if (useRear) done = (B <= REAR_SET_CM + REAR_TOL_CM);
      else         done = (REVERSE_MM <= 0 || -mmSince(phaseTicks) >= REVERSE_MM);
      if (-mmSince(phaseTicks) >= REVERSE_MAX_MM || millis() - phaseMs > 5000) done = true;
      if (!done) { setServo(SERVO_CENTER_DEG - HEADING_KP * hErr); driveMotor(REVERSE_SPEED, false); }
      else {
        driveMotor(0, true); setServo(SERVO_CENTER_DEG); delay(200);
        logLine(String("  reverse done (") + (useRear ? "rear ToF" : "distance") + "): " + String(-mmSince(phaseTicks), 0) + " mm  B=" + String(getDistance(TOF_REAR_CH), 0) + " F=" + String(getDistance(TOF_FRONT_CH), 0) + " L=" + String(getDistance(TOF_LEFT_CH), 0) + " R=" + String(getDistance(TOF_RIGHT_CH), 0) + " h_err=" + String(wrap180(laneHeading - readHeading()), 1));
        corners++; legStartTicks = encoderTicks; fwdMM = 0; bulgesThisLeg = 0; lastLineMM[BLUE_ID] = lastLineMM[ORANGE_ID] = -100000; seenCount = 0; seenId = 0; rOpen = lOpen = 0; gateLogged = false;
        logLine(String("CORNER ") + corners + " done");
        if (corners >= CORNERS_TO_RUN) { finishing = true; logLine("finish section: pillars on, corners off, stop after " + String(FINISH_MM) + " mm"); }
        enterPhase(STRAIGHT, 0);
      }
      break; }
  }

  logLine(String(millis() - runStart) + "," + String(mmSince(0), 0) + "," + String(legMM(), 0) + "," + String(fwdMM, 0) + "," + phaseNames[phase] + "," + String(mmSince(phaseTicks), 0) + "," +
          String(F, 0) + "," + String(L, 0) + "," + String(R, 0) + "," + String(B, 0) + "," + String(hdg, 1) + "," + String(hErr, 1) + "," + servoNow + "," + bulgesThisLeg + "," +
          camCol(1) + "," + camCol(2) + "," + camCol(3) + "," + camCol(4) + "," + camCol(5) + "," + camCol(6));
  if (millis() - runStart > RUN_TIMEOUT_MS) finish("timeout");
  if (digitalRead(START_BTN_PIN) == LOW)   finish("button");
}
