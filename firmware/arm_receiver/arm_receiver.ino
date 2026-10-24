// arm_receiver.ino: receives james-gestures commands over USB serial and drives the arm.
//
// Protocol (one command per line, 115200 baud):
//   ARM 1 | ARM 0          enable / disable motion (starts disabled)
//   STOP                   freeze everything and disable
//   J <joint> <deg>        joint = base | shoulder | elbow | wrist
//   CLAW 1 | CLAW 0        close / open the claw
//
// Hardware assumed (change the pins/constants to match the real build):
//   base:     stepper on a STEP/DIR driver (A4988 / DRV8825 / TMC2208)
//   shoulder, elbow, wrist, claw: hobby servos on PWM pins, powered from their OWN 6 V supply
//             (share ground with the board, never power servos from the board's 5 V pin)
//   kill switch: a real switch on the servo/stepper power line. This sketch is not a safety system.
//
// Written for a standard Arduino core (Servo library). For the Arduino UNO Q use
// ../arm_receiver_unoq/arm_receiver_unoq.ino instead: there the laptop's USB reaches the sketch via Monitor.
// Untested on hardware: treat the pins and limits as a starting point.

#include <Servo.h>

// ---------- pins ----------
const int PIN_STEP = 2, PIN_DIR = 3, PIN_STEP_EN = 4;      // stepper driver (EN is active LOW)
const int PIN_SHOULDER = 9, PIN_ELBOW = 10, PIN_WRIST = 11, PIN_CLAW = 6;

// ---------- base stepper ----------
const float STEPS_PER_DEG = 200.0 * 16 / 360.0;            // 200 step motor, 1/16 microstepping, no gearbox
const unsigned int STEP_US = 400;                          // time between steps: higher = slower base
long baseNow = 0, baseGoal = 0;                            // in steps, 0 = where it was at power on
const float BASE_HOME_DEG = 135;                           // the Python side's home for the base

// ---------- servos ----------
Servo sShoulder, sElbow, sWrist, sClaw;
const int CLAW_OPEN = 20, CLAW_CLOSED = 95;                // tune so CLOSED grips without stalling

bool armed = false;
String line;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP_EN, OUTPUT);
  digitalWrite(PIN_STEP_EN, HIGH);                         // stepper off until ARM 1
  Serial.println("READY");
}

void attachAll() {
  if (!sShoulder.attached()) {
    sShoulder.attach(PIN_SHOULDER); sElbow.attach(PIN_ELBOW);
    sWrist.attach(PIN_WRIST); sClaw.attach(PIN_CLAW);
  }
}

void detachAll() {                                         // servos go limp: no holding torque
  sShoulder.detach(); sElbow.detach(); sWrist.detach(); sClaw.detach();
}

void handle(String cmd) {
  cmd.trim();
  if (cmd == "STOP") {
    armed = false; baseGoal = baseNow;
    digitalWrite(PIN_STEP_EN, HIGH);
    // keep servos attached so the arm holds its pose instead of dropping; cut power with the kill switch
    Serial.println("OK STOP");
    return;
  }
  if (cmd == "ARM 1") { armed = true; attachAll(); digitalWrite(PIN_STEP_EN, LOW); Serial.println("OK ARM"); return; }
  if (cmd == "ARM 0") { armed = false; baseGoal = baseNow; digitalWrite(PIN_STEP_EN, HIGH); Serial.println("OK DISARM"); return; }
  if (!armed) return;                                      // everything below needs ARM 1

  if (cmd.startsWith("CLAW ")) {
    sClaw.write(cmd.endsWith("1") ? CLAW_CLOSED : CLAW_OPEN);
    return;
  }
  if (cmd.startsWith("J ")) {
    int sp = cmd.indexOf(' ', 2);
    if (sp < 0) return;
    String joint = cmd.substring(2, sp);
    float deg = cmd.substring(sp + 1).toFloat();
    if (joint == "base") baseGoal = lround((deg - BASE_HOME_DEG) * STEPS_PER_DEG);
    else if (joint == "shoulder") sShoulder.write(constrain((int)deg, 0, 180));
    else if (joint == "elbow") sElbow.write(constrain((int)deg, 0, 180));
    else if (joint == "wrist") sWrist.write(constrain((int)deg, 0, 180));
  }
}

unsigned long lastStep = 0;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') { handle(line); line = ""; }
    else if (c != '\r' && line.length() < 40) line += c;
  }
  // one base step at a time, never blocking the serial reader
  if (armed && baseNow != baseGoal && micros() - lastStep >= STEP_US) {
    lastStep = micros();
    bool fwd = baseGoal > baseNow;
    digitalWrite(PIN_DIR, fwd ? HIGH : LOW);
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(3);
    digitalWrite(PIN_STEP, LOW);
    baseNow += fwd ? 1 : -1;
  }
}
