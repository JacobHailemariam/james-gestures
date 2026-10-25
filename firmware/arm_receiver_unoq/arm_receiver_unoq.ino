// arm_receiver_unoq.ino: receives james-gestures commands and drives the arm, on the Arduino UNO Q.
//
// Hardware (matches bringup_unoq.ino, which found it all working):
//   servos:  PCA9685 16-channel PWM board on I2C (0x40), channels below, 50 Hz. ANNIMOS RDS5160SG (60 kg,
//            shoulder) and RDS3225SG (25 kg, elbow/wrist) sweep 270 deg over 500..2500 us; the 9 g claw servo
//            sweeps 180 deg over the same pulses. Protocol degrees are REAL degrees with 90 = centre (1500 us).
//            Servos powered from the PCA9685's V+ terminal (own 5-6 V supply), ground shared with the board.
//   base:    stepper on a TB6600-style driver: PUL D9, DIR D3, ENA D4 (ENA HIGH = disabled).
//            PUL-/DIR-/ENA- to GND. Driver set to 3200 pulses/rev (1/16).
//   The laptop's USB reaches this sketch through Monitor (demo.py --port COM3). Serial = the D0/D1 pins.
//
// Protocol (one command per line):
//   ARM 1 | ARM 0          enable / disable motion (starts disabled, all servo outputs off)
//   STOP                   freeze everything and disable
//   J <joint> <deg>        joint = base | shoulder | elbow | wrist
//   CLAW 1 | CLAW 0        close / open the claw
//   PING                   replies PONG, moves nothing
//
// Untested on the assembled arm: tune CLAW_* and STEPS_PER_DEG there. Keep the kill switch.

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------- servo channels on the PCA9685 ----------
const uint8_t CH_SHOULDER = 4, CH_ELBOW = 0, CH_WRIST = 1, CH_CLAW = 15;   // team-confirmed wiring: do not change
const int CLAW_OPEN = 20, CLAW_CLOSED = 95;                // tune so CLOSED grips without stalling
const int SERVO_US_MIN = 500, SERVO_US_MAX = 2500, SERVO_US_MID = 1500;
const float US_PER_DEG_270 = 2000.0 / 270.0;               // ANNIMOS 60 kg / 25 kg: 7.41 us per real degree
const float US_PER_DEG_180 = 2000.0 / 180.0;               // 9 g claw servo: 11.1 us per degree
const uint8_t PCA_ADDR = 0x40;
const uint32_t PCA_OSC_HZ = 27000000;
Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
bool pcaOk = false;

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
  for (uint8_t ch = 0; ch < 16; ch++) pwm.setPWM(ch, 0, 4096);
}

void servoDeg(uint8_t ch, int deg) {                      // deg = real degrees, 90 = servo centre
  if (!pcaOk) return;
  deg = constrain(deg, 0, 180);
  float perDeg = (ch == CH_CLAW) ? US_PER_DEG_180 : US_PER_DEG_270;
  long us = SERVO_US_MID + (long)((deg - 90) * perDeg + ((deg >= 90) ? 0.5f : -0.5f));
  pwm.writeMicroseconds(ch, constrain(us, (long)SERVO_US_MIN, (long)SERVO_US_MAX));
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

  if (cmd.startsWith("CLAW ")) { servoDeg(CH_CLAW, cmd.endsWith("1") ? CLAW_CLOSED : CLAW_OPEN); return; }
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
    else if (joint == "wrist") servoDeg(CH_WRIST, (int)deg);
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
