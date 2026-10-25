#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------- servo channels on the PCA9685 ----------
const uint8_t CH_SHOULDER = 4, CH_ELBOW = 0, CH_WRIST = 1;  // team-confirmed wiring: do not change
const uint8_t CH_CLAW_L = 15, CH_CLAW_R = 14;              // two claw servos, one per jaw (left/right seen from above)
const int CLAW_OPEN = 20, CLAW_CLOSED = 95;                // right jaw angles; tune so CLOSED grips without stalling
const bool CLAW_L_MIRROR = true;                           // left jaw is the mirror image: it gets 180 - angle
const bool WRIST_LOCK = true;                              // claw sits loosely on the wrist: hold the wrist straight
                                                           // (90 = 1500 us) and ignore every J wrist angle
inline bool isClaw(uint8_t ch) { return ch == CH_CLAW_L || ch == CH_CLAW_R; }
const int SERVO_US_MIN = 500, SERVO_US_MAX = 2500, SERVO_US_MID = 1500;
const float US_PER_DEG_270 = 2000.0 / 270.0;               // ANNIMOS 60 kg / 25 kg: 7.41 us per real degree
const float US_PER_DEG_180 = 2000.0 / 180.0;               // 9 g claw servo: 11.1 us per degree
const uint8_t PCA_ADDR = 0x40;
const uint32_t PCA_OSC_HZ = 27000000;
Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
bool pcaOk = false;

// ---------- servo slew limit: no command from anywhere can snap a joint ----------
const float SERVO_DEG_PER_S = 60.0;                        // arm joints move at most this fast
const unsigned long SERVO_TICK_MS = 20;                    // one PCA9685 update per 50 Hz servo frame
float servoNow[16], servoGoal[16];
bool servoKnown[16];                                       // false until a channel's first command (that one snaps:
                                                           // we don't know where the servo physically is)
unsigned long lastServoTick = 0;
// Per-channel trim, degrees, added to every command so that protocol 90 = the limb physically straight.
// Calibrate with test_motors.py: arm, 'shoulder 90', nudge +5/-5 until straight, trim = final - 90.
float SERVO_TRIM_DEG[16] = {0};                           // index = PCA9685 channel

// ---------- base stepper on the TB6600 ----------
const int PIN_PUL = 9, PIN_DIR = 3, PIN_ENA = 4;
const unsigned int PULSE_HIGH_US = 12;                     // driver needs >= 10 us HIGH
const float STEPS_PER_DEG = 3200.0 / 360.0;                // 1/16 microstep, no gearbox: change if geared
const unsigned long STEP_US = 1000;                        // time between pulses: higher = slower base
long baseNow = 0, baseGoal = 0;                            // in steps, 0 = where it was at power on
const float BASE_HOME_DEG = 135;                           // the Python side's home for the base
bool lastDirHigh = false;
bool stepperOn = false;                                   // TB6600 enabled only while the base has somewhere to go

// ---------- commands from the laptop ----------
const unsigned long POLL_MS = 5;                           // Monitor.available() is a round trip to Linux
bool armed = false;
String line;

bool initPca() {
  Wire.beginTransmission(PCA_ADDR);
  if (Wire.endTransmission() != 0) return false;
  if (!pwm.begin()) return false;
  pwm.setOscillatorFrequency(PCA_OSC_HZ);
  pwm.setPWMFreq(50);
  pcaOk = true;
  allServosOff();
  return true;
}

void allServosOff() {                                      // no pulses: the joints go limp
  if (!pcaOk) return;
  for (uint8_t ch = 0; ch < 16; ch++) { pwm.setPWM(ch, 0, 4096); servoKnown[ch] = false; }
}

void servoWrite(uint8_t ch, float deg) {                  // deg = real degrees, 90 = straight (after trim)
  deg += SERVO_TRIM_DEG[ch];
  float perDeg = isClaw(ch) ? US_PER_DEG_180 : US_PER_DEG_270;
  long us = SERVO_US_MID + (long)((deg - 90) * perDeg + ((deg >= 90) ? 0.5f : -0.5f));
  pwm.writeMicroseconds(ch, constrain(us, (long)SERVO_US_MIN, (long)SERVO_US_MAX));
}

void servoDeg(uint8_t ch, int deg) {                      // sets the goal; loop() slews toward it
  if (!pcaOk) return;
  deg = constrain(deg, 0, 180);
  if (isClaw(ch)) {                                       // the claw just goes
    servoNow[ch] = deg; servoKnown[ch] = true;
    servoWrite(ch, deg);
  } else if (!servoKnown[ch]) {                            // first command on this channel: the servo's real
    servoNow[ch] = 90; servoKnown[ch] = true;              // position is unknown, so the one unavoidable jump
    servoWrite(ch, 90);                                    // is to straight (home); the rest of the move slews
  }
  servoGoal[ch] = deg;
}

void freezeServos() {                                     // STOP / ARM 0: stay where the slew got to
  for (uint8_t ch = 0; ch < 16; ch++) servoGoal[ch] = servoNow[ch];
}

void tickServos() {
  if (!pcaOk || millis() - lastServoTick < SERVO_TICK_MS) return;
  lastServoTick = millis();
  const float stepDeg = SERVO_DEG_PER_S * SERVO_TICK_MS / 1000.0;
  for (uint8_t ch = 0; ch < 16; ch++) {
    if (!servoKnown[ch] || servoNow[ch] == servoGoal[ch]) continue;
    float d = servoGoal[ch] - servoNow[ch];
    if (d > stepDeg) d = stepDeg; else if (d < -stepDeg) d = -stepDeg;
    servoNow[ch] += d;
    servoWrite(ch, servoNow[ch]);
  }
}

void setup() {
  pinMode(PIN_ENA, OUTPUT); digitalWrite(PIN_ENA, HIGH);   // stepper disabled first thing
  pinMode(PIN_PUL, OUTPUT); digitalWrite(PIN_PUL, LOW);
  pinMode(PIN_DIR, OUTPUT); digitalWrite(PIN_DIR, LOW);
  Bridge.begin();
  Monitor.begin();
  Wire.begin();
  initPca();
  Monitor.println(pcaOk ? "READY" : "READY (PCA9685 not found: servos off)");
}

void stepper(bool on) {                                   // enabling pulls holding current through the coils:
  if (on == stepperOn) return;                             // keep it off until the first base move, so ARM 1
  digitalWrite(PIN_ENA, on ? LOW : HIGH);                  // alone never loads the supply
  stepperOn = on;
  if (on) delay(5);                                        // let the driver wake before the first pulse
}

void disarm(const char *why) {
  armed = false; baseGoal = baseNow;
  stepper(false);
  freezeServos();
  // servos keep their last pulse so the arm holds its pose instead of dropping; cut power with the kill switch
  Monitor.print("OK "); Monitor.println(why);
}

void handle(String cmd) {
  cmd.trim();
  if (cmd == "PING") { Monitor.println("PONG"); return; }
  if (cmd == "STOP") { disarm("STOP"); return; }
  if (cmd == "ARM 0") { disarm("DISARM"); return; }
  if (cmd == "ARM 1") {
    if (!pcaOk) initPca();
    armed = true;                                          // stepper stays disabled until a base command
    Monitor.println(pcaOk ? "OK ARM" : "OK ARM (no PCA9685: servos off)");
    return;
  }
  if (!armed) return;                                      // everything below needs ARM 1

  if (cmd.startsWith("CLAW ")) {
    int a = cmd.endsWith("1") ? CLAW_CLOSED : CLAW_OPEN;
    servoDeg(CH_CLAW_R, a);
    servoDeg(CH_CLAW_L, CLAW_L_MIRROR ? 180 - a : a);
    return;
  }
  if (cmd.startsWith("J ")) {
    int sp = cmd.indexOf(' ', 2);
    if (sp < 0) return;
    String joint = cmd.substring(2, sp);
    float deg = cmd.substring(sp + 1).toFloat();
    if (joint == "base") {                                 // no lround(): the UNO Q core doesn't export it
      float steps = (deg - BASE_HOME_DEG) * STEPS_PER_DEG;
      baseGoal = (long)(steps + (steps >= 0 ? 0.5f : -0.5f));
    }
    else if (joint == "shoulder") servoDeg(CH_SHOULDER, (int)deg);
    else if (joint == "elbow") servoDeg(CH_ELBOW, (int)deg);
    else if (joint == "wrist") servoDeg(CH_WRIST, WRIST_LOCK ? 90 : (int)deg);
  }
}

unsigned long lastStep = 0, lastPoll = 0;

void readCommands() {
  int n = Monitor.available();
  while (n-- > 0) {
    int c = Monitor.read();
    if (c < 0) break;
    if (c == '\n') { handle(line); line = ""; }
    else if (c != '\r' && line.length() < 40) line += (char)c;
  }
}

void loop() {
  if (millis() - lastPoll >= POLL_MS) {
    lastPoll = millis();
    readCommands();
  }
  tickServos();                                            // slew every servo toward its goal
  if (armed && baseNow == baseGoal) stepper(false);         // goal reached: release the holding current
  // one base step at a time, never blocking the command reader
  if (armed && baseNow != baseGoal && micros() - lastStep >= STEP_US) {
    lastStep = micros();
    stepper(true);
    bool fwd = baseGoal > baseNow;
    if (fwd != lastDirHigh) {                              // TB6600 wants a pause after a DIR change
      digitalWrite(PIN_DIR, fwd ? HIGH : LOW);
      lastDirHigh = fwd;
      delayMicroseconds(12);
    }
    digitalWrite(PIN_PUL, HIGH);
    delayMicroseconds(PULSE_HIGH_US);
    digitalWrite(PIN_PUL, LOW);
    baseNow += fwd ? 1 : -1;
  }
}
