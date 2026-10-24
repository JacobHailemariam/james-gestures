// arm_receiver_unoq.ino: arm_receiver.ino for the Arduino UNO Q.
//
// Same protocol and pins as arm_receiver.ino, so the Python side is unchanged (demo.py --port COM3):
//   ARM 1 | ARM 0          enable / disable motion (starts disabled)
//   STOP                   freeze everything and disable
//   J <joint> <deg>        joint = base | shoulder | elbow | wrist
//   CLAW 1 | CLAW 0        close / open the claw
//   PING                   replies PONG, moves nothing (connection check)
//
// What is different on the UNO Q:
//   - The laptop's USB-C cable goes to the Linux side, not to this MCU. `Serial` here is the D0/D1 pins.
//     Text sent to the board's USB serial port reaches the sketch through `Monitor` (Arduino_RouterBridge),
//     relayed by the arduino-router service on Linux.
//   - Every Monitor.available() is a round trip to Linux, and it blocks until Linux answers. So the sketch
//     checks for commands every POLL_MS and steps the base in between, instead of polling on every loop.
//   - The headers are 3.3 V logic. Hobby servos normally accept a 3.3 V signal. A4988: power its VDD (logic)
//     pin from the board's 3.3 V, not 5 V, or 3.3 V STEP/DIR may not register. DRV8825 is fine as is;
//     TMC2208: set VIO to 3.3 V. D3 (DIR) is not 5 V tolerant: never connect a 5 V signal to it.
//   - Servo pins 6, 9, 10, 11 are PWM pins on the UNO Q too. Needs a Servo library release with UNO Q support.
//
// Upload with Arduino App Lab: new app, paste this into the sketch, add the Servo library to the sketch, run.
// Untested on hardware: treat the pins and limits as a starting point. Not a safety system: keep the kill switch.

#include <Arduino_RouterBridge.h>
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

// ---------- commands from the laptop ----------
const unsigned long POLL_MS = 5;                           // how often to ask Linux for new bytes

bool armed = false;
String line;

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP_EN, OUTPUT);
  digitalWrite(PIN_STEP_EN, HIGH);                         // stepper off until ARM 1
  Bridge.begin();
  Monitor.begin();
  Monitor.println("READY");                                // dropped if the laptop isn't listening yet
}

void attachAll() {
  if (!sShoulder.attached()) {
    sShoulder.attach(PIN_SHOULDER); sElbow.attach(PIN_ELBOW);
    sWrist.attach(PIN_WRIST); sClaw.attach(PIN_CLAW);
  }
}

void handle(String cmd) {
  cmd.trim();
  if (cmd == "PING") { Monitor.println("PONG"); return; }
  if (cmd == "STOP") {
    armed = false; baseGoal = baseNow;
    digitalWrite(PIN_STEP_EN, HIGH);
    // keep servos attached so the arm holds its pose instead of dropping; cut power with the kill switch
    Monitor.println("OK STOP");
    return;
  }
  if (cmd == "ARM 1") { armed = true; attachAll(); digitalWrite(PIN_STEP_EN, LOW); Monitor.println("OK ARM"); return; }
  if (cmd == "ARM 0") { armed = false; baseGoal = baseNow; digitalWrite(PIN_STEP_EN, HIGH); Monitor.println("OK DISARM"); return; }
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
    if (joint == "base") {                                 // no lround(): the UNO Q core doesn't export it
      float steps = (deg - BASE_HOME_DEG) * STEPS_PER_DEG;
      baseGoal = (long)(steps + (steps >= 0 ? 0.5f : -0.5f));
    }
    else if (joint == "shoulder") sShoulder.write(constrain((int)deg, 0, 180));
    else if (joint == "elbow") sElbow.write(constrain((int)deg, 0, 180));
    else if (joint == "wrist") sWrist.write(constrain((int)deg, 0, 180));
  }
}

unsigned long lastStep = 0, lastPoll = 0;

void readCommands() {
  int n = Monitor.available();                             // one round trip to Linux, then read() is local
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
  // one base step at a time, never blocking the command reader
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
