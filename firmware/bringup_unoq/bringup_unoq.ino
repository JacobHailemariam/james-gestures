// bringup.ino: one-at-a-time hardware test for the robotic arm on the Arduino UNO Q.
//
// Nothing moves on its own. Open App Lab's Serial Monitor and type one command per line:
//   scan    I2C scan, reports whether the PCA9685 (0x40) answers
//   servo   sweep channel 0 (shoulder) slowly 90 -> 120 -> 60 -> 90, twice, then hold at 90
//   base    enable the stepper, 200 steps one way, pause, 200 steps back, disable
//   off     stepper disabled (D4 HIGH) and all 16 servo outputs cut (no signal)
//   help    show this list
// Typing anything while the servo sweeps aborts the sweep. The kill switch is still the real stop.

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ---------- stepper driver (TB6600-style, opto inputs, PUL-/DIR-/ENA- to GND) ----------
const int PIN_PUL = 9, PIN_DIR = 3, PIN_ENA = 4;           // ENA: HIGH = driver disabled, LOW = enabled
const unsigned int PULSE_HIGH_US = 12;                     // driver needs >= 10 us HIGH
const unsigned int DIR_SETUP_US = 12;                      // driver needs ~10 us between DIR change and pulse
const unsigned long TEST_STEP_PERIOD_US = 2000;            // 500 pulses/s: slow on purpose
const int TEST_STEPS = 200;                                // 200 of 3200 pulses/rev = 22.5 degrees

// ---------- PCA9685 ----------
const uint8_t PCA_ADDR = 0x40;
const uint32_t PCA_OSC_HZ = 27000000;
const float SERVO_HZ = 50;
const int SERVO_US_MIN = 500, SERVO_US_MAX = 2500;         // 0..180 degrees
const uint8_t TEST_CHANNEL = 0;                            // shoulder
Adafruit_PWMServoDriver pwm(PCA_ADDR, Wire);
bool pcaOk = false;

char buf[48];
size_t len = 0;
bool heardFromUser = false;
unsigned long lastBannerMs = 0;

void printHelp() {
  Monitor.println("BRINGUP ready. Commands: scan | servo | base | off | help");
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void allServosOff() {
  if (!pcaOk) return;
  for (uint8_t ch = 0; ch < 16; ch++) pwm.setPWM(ch, 0, 4096);   // 4096 = full off: no pulses at all
}

bool initPca() {
  if (!i2cPresent(PCA_ADDR)) return false;
  if (!pwm.begin()) return false;
  pwm.setOscillatorFrequency(PCA_OSC_HZ);                  // must come before setPWMFreq
  pwm.setPWMFreq(SERVO_HZ);
  pcaOk = true;
  allServosOff();                                          // the PCA9685 keeps old outputs across a sketch restart
  return true;
}

void stepperOff() {
  digitalWrite(PIN_ENA, HIGH);
}

void servoDeg(uint8_t ch, int deg) {
  deg = constrain(deg, 0, 180);
  int us = SERVO_US_MIN + (long)(SERVO_US_MAX - SERVO_US_MIN) * deg / 180;
  pwm.writeMicroseconds(ch, us);
}

void i2cScan() {
  Monitor.println("I2C scan on Wire (SDA/SCL pins next to AREF)...");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) {
      Monitor.print("  found 0x");
      if (a < 16) Monitor.print("0");
      Monitor.println(a, HEX);
      found++;
    }
  }
  Monitor.print("Devices found: ");
  Monitor.println(found);
  if (i2cPresent(PCA_ADDR)) {
    Monitor.println("PCA9685 at 0x40: FOUND (0x70 is its all-call address, also normal)");
    if (!pcaOk) Monitor.println(initPca() ? "PCA9685 initialised, all outputs off" : "PCA9685 init FAILED");
  } else {
    Monitor.println("PCA9685 at 0x40: NOT FOUND. Check VCC/GND/SDA/SCL and the A0-A5 address pads.");
  }
}

// true if the user typed something (it is consumed so it doesn't run as a command)
bool userInterrupt() {
  int n = Monitor.available();
  if (n <= 0) return false;
  while (n-- > 0) Monitor.read();
  return true;
}

// moves one degree at a time; returns false if aborted
bool sweepTo(int &deg, int target, unsigned long msPerDeg) {
  while (deg != target) {
    deg += (target > deg) ? 1 : -1;
    servoDeg(TEST_CHANNEL, deg);
    if (deg % 10 == 0) {
      Monitor.print("  ch0 = ");
      Monitor.println(deg);
    }
    delay(msPerDeg);
    if (userInterrupt()) return false;
  }
  return true;
}

void servoTest() {
  if (!pcaOk && !initPca()) {
    Monitor.println("ERR PCA9685 not found, run 'scan'");
    return;
  }
  Monitor.println("Servo ch0: going to 90 (first pulse, it may jump), type anything to abort");
  int deg = 90;
  servoDeg(TEST_CHANNEL, deg);
  delay(1000);
  for (int cycle = 1; cycle <= 2; cycle++) {
    Monitor.print("Sweep cycle ");
    Monitor.println(cycle);
    if (!sweepTo(deg, 120, 40) || !sweepTo(deg, 60, 40) || !sweepTo(deg, 90, 40)) {
      Monitor.print("ABORTED, holding at ");
      Monitor.println(deg);
      return;
    }
  }
  Monitor.println("Servo test done, holding at 90. Type 'off' to cut the signal (the joint will go limp).");
}

void stepMany(bool dirHigh, int n) {
  digitalWrite(PIN_DIR, dirHigh ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_PUL, HIGH);
    delayMicroseconds(PULSE_HIGH_US);
    digitalWrite(PIN_PUL, LOW);
    delayMicroseconds(TEST_STEP_PERIOD_US - PULSE_HIGH_US);
  }
}

void baseTest() {
  Monitor.println("Base: enabling driver (D4 LOW)");
  digitalWrite(PIN_ENA, LOW);
  delay(100);
  Monitor.println("Base: 200 steps with DIR HIGH");
  stepMany(true, TEST_STEPS);
  Monitor.println("Base: pause 1 s");
  delay(1000);
  Monitor.println("Base: 200 steps with DIR LOW");
  stepMany(false, TEST_STEPS);
  delay(200);
  stepperOff();
  Monitor.println("Base test done, driver disabled (D4 HIGH). It should be back where it started.");
}

void handle(char *cmd) {
  for (char *p = cmd; *p; p++) *p = tolower(*p);
  heardFromUser = true;
  if (!strcmp(cmd, "scan")) i2cScan();
  else if (!strcmp(cmd, "servo")) servoTest();
  else if (!strcmp(cmd, "base")) baseTest();
  else if (!strcmp(cmd, "off")) {
    stepperOff();
    allServosOff();
    Monitor.println("OFF: stepper disabled, servo outputs cut");
  }
  else if (!strcmp(cmd, "help")) printHelp();
  else {
    Monitor.print("Unknown command: ");
    Monitor.println(cmd);
    printHelp();
  }
}

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  digitalWrite(PIN_ENA, HIGH);                             // stepper disabled first thing
  pinMode(PIN_PUL, OUTPUT);
  digitalWrite(PIN_PUL, LOW);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_DIR, LOW);

  Bridge.begin();
  Monitor.begin();
  Wire.begin();
  initPca();                                               // silently; 'scan' reports the result
}

void loop() {
  // Output sent before the Serial Monitor is open is lost, so repeat the banner until the user types.
  if (!heardFromUser && millis() - lastBannerMs >= 5000) {
    lastBannerMs = millis();
    printHelp();
  }

  int n = Monitor.available();
  while (n-- > 0) {
    int c = Monitor.read();
    if (c < 0) break;
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        buf[len] = '\0';
        handle(buf);
      }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = (char)c;
    }
  }
  delay(10);
}
