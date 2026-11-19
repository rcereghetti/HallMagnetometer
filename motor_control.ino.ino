// ---------- Settings (change these if needed) ----------
const int STEP_PIN   = 2;    // CNC shield X step
const int DIR_PIN    = 5;    // CNC shield X direction
const int EN_PIN     = 8;    // CNC shield enable (LOW = on)
const int SENSOR_PIN = A0;   // optical sensor (analog)

const bool UP_DIR = LOW;    // flip to LOW if "up" goes the wrong way

// With the wiring described (resistor from 5 V to the phototransistor
// collector, A0 on that point), a BLOCKED beam gives a HIGH reading.
// Flip this to false if your sensor behaves the opposite way.
const bool BLOCKED_IS_HIGH = true;

int threshold = 500;         // 0-1023, can be changed with the "T" command

const long MIN_SPEED = 20;   // steps per second (limits of the "V" command)
const long MAX_SPEED = 3000;

const unsigned long STREAM_MS = 50;   // time between streamed sensor values

// Calibration ("C" command): finds the free level and the blocked level of the
// sensor and sets the threshold exactly in the middle, then homes again.
// The distances below are in steps and assume 0.625 um per step (1/8 microstepping,
// 8000 steps = 5 mm). If you change the microstepping, scale them (and UM_PER_STEP
// in the Python file).
const long CAL_SPEED        = 1500;  // steps per second during the scans (= 0.94 mm/s)
const long CAL_MAX_STEPS    = 10000; // safety limit for each scan (= 6.25 mm)
const int  CAL_WINDOW       = 200;   // steps used to decide "the reading is flat" (= 0.125 mm)
const int  CAL_FLAT         = 8;     // max spread (ADC counts) in that window to count as flat
const int  CAL_MIN_CONTRAST = 100;   // free and blocked levels must differ by at least this
const long CAL_BACKOFF      = 1000;  // steps below the edge before the final home (= 0.625 mm)
// --------------------------------------------------------

// Position = motor steps BELOW the home sensor (0 = sensor, grows going down).
// It is only meaningful after the motor has reached the sensor once (homed).
long pos = 0;
bool homed = false;

unsigned long stepPeriodUs = 1600;    // time per step; 1600 us = 625 steps/s
                                      // change it with "V <steps per second>"

bool streaming = false;               // "W 1" = stream sensor value + position
unsigned long lastStream = 0;

enum { MOVE_DONE, MOVE_SENSOR, MOVE_STOPPED };

bool sensorHit() {
  int v = analogRead(SENSOR_PIN);
  return BLOCKED_IS_HIGH ? (v > threshold) : (v < threshold);
}

void waitUntil(unsigned long t) {
  while ((long)(micros() - t) < 0) {}
}

// While streaming, sends "A <sensor value> <position>" every STREAM_MS.
void streamTick() {
  if (!streaming) return;
  unsigned long now = millis();
  if (now - lastStream >= STREAM_MS) {
    lastStream = now;
    Serial.print("A ");
    Serial.print(analogRead(SENSOR_PIN));
    Serial.print(' ');
    Serial.println(pos);
  }
}

// Commands that only change a setting: W, T, V. Returns true if handled.
// They are also accepted while the motor is moving, so they can never
// get in front of a STOP.
bool handleSetting(String cmd) {
  char c = cmd.charAt(0);
  if (c == 'W') {                       // "W 1" = stream on, "W 0" = off
    streaming = (cmd.substring(1).toInt() != 0);
    Serial.println(streaming ? "STREAM ON" : "STREAM OFF");
    return true;
  }
  if (c == 'T') {                       // "T 400" = set threshold to 400
    int t = cmd.substring(1).toInt();
    if (t >= 0 && t <= 1023) {
      threshold = t;
      Serial.print("THRESHOLD ");
      Serial.println(threshold);
    } else {
      Serial.println("ERR");
    }
    return true;
  }
  if (c == 'V') {                       // "V 300" = speed 300 steps per second
    long v = cmd.substring(1).toInt();
    if (v >= MIN_SPEED && v <= MAX_SPEED) {
      stepPeriodUs = 1000000UL / (unsigned long)v;
      Serial.print("SPEED ");
      Serial.println(v);
    } else {
      Serial.println("ERR");
    }
    return true;
  }
  return false;
}

// Moves the motor 'steps' steps. Updates pos on every step.
// Moving up stops at the sensor (safety) and sets pos = 0, unless
// stopAtSensor is false (only the calibration does that).
// 'S' stops any move. Steps are spaced exactly stepPeriodUs apart.
int moveSteps(long steps, bool up, bool stopAtSensor) {
  digitalWrite(DIR_PIN, up ? UP_DIR : !UP_DIR);
  delayMicroseconds(10);
  for (long i = 0; i < steps; i++) {
    unsigned long t0 = micros();
    unsigned long period = stepPeriodUs;

    if (up && stopAtSensor && sensorHit()) {
      pos = 0;
      homed = true;
      return MOVE_SENSOR;
    }

    while (Serial.available() && (Serial.peek() == '\n' || Serial.peek() == '\r')) {
      Serial.read();                    // skip leftover line ends
    }
    if (Serial.available()) {
      char p = Serial.peek();
      if (p == 'S') {
        Serial.read();
        return MOVE_STOPPED;
      }
      if (p == 'W' || p == 'T' || p == 'V') {
        String s = Serial.readStringUntil('\n');
        s.trim();
        handleSetting(s);
      }
    }
    streamTick();

    digitalWrite(STEP_PIN, HIGH);
    unsigned long hi = t0 + period / 2;
    unsigned long now = micros();
    if ((long)(hi - now) < 5) hi = now + 5;      // pulse at least 5 us
    waitUntil(hi);
    digitalWrite(STEP_PIN, LOW);
    pos += up ? -1 : 1;
    waitUntil(t0 + period);
  }
  return MOVE_DONE;
}

void reportMove(int r, bool isHome) {
  if (r == MOVE_SENSOR) Serial.print(isHome ? "HOMED" : "SENSOR");
  else if (r == MOVE_STOPPED) Serial.print("STOPPED");
  else Serial.print("DONE");
  Serial.print(" pos=");
  Serial.println(pos);
}

// ---------- Calibration ----------
// Sensor value averaged over 4 readings (less noise).
int avgSensor() {
  long sum = 0;
  for (int i = 0; i < 4; i++) sum += analogRead(SENSOR_PIN);
  return (int)(sum / 4);
}

bool isBlockedValue(int v) {
  return BLOCKED_IS_HIGH ? (v > threshold) : (v < threshold);
}

// Moves one step at a time (up or down) until the reading is flat AND on the
// wanted side of the current threshold (wantBlocked = true: blocked plateau,
// false: free plateau). The mean of that flat stretch is returned in 'level'.
// Returns 0 = found, 1 = not found within CAL_MAX_STEPS, 2 = stopped by 'S'.
int scanForPlateau(bool up, bool wantBlocked, int &level) {
  int wmin = 1023, wmax = 0, wn = 0;
  long wsum = 0;
  long steps = 0, afterCross = 0;
  bool crossed = false;                 // reading has been on the wanted side
  while (true) {
    int v = avgSensor();
    if (isBlockedValue(v) == wantBlocked) crossed = true;
    if (v < wmin) wmin = v;
    if (v > wmax) wmax = v;
    wsum += v;
    wn++;
    if (wn >= CAL_WINDOW) {
      if (wmax - wmin <= CAL_FLAT) {
        int mean = (int)(wsum / wn);
        if (isBlockedValue(mean) == wantBlocked) {
          level = mean;
          return 0;
        }
      }
      wmin = 1023; wmax = 0; wn = 0; wsum = 0;
    }
    if (crossed ? (afterCross >= CAL_MAX_STEPS) : (steps >= CAL_MAX_STEPS)) return 1;
    if (moveSteps(1, up, false) == MOVE_STOPPED) return 2;
    steps++;
    if (crossed) afterCross++;
  }
}

// 0 = done (ready for the final home), 1 = plateau not found, 2 = stopped,
// 3 = free and blocked levels too close together, 4 = could not come back down.
int calibrateRun() {
  int freeLevel = 0, blockedLevel = 0;
  int r = scanForPlateau(false, false, freeLevel);     // down to the free plateau
  if (r != 0) return r;
  r = scanForPlateau(true, true, blockedLevel);        // up, through the edge, to the blocked plateau
  if (r != 0) return r;

  int contrast = BLOCKED_IS_HIGH ? (blockedLevel - freeLevel) : (freeLevel - blockedLevel);
  if (contrast < CAL_MIN_CONTRAST) return 3;

  threshold = (freeLevel + blockedLevel) / 2;
  Serial.print("CAL free=");
  Serial.print(freeLevel);
  Serial.print(" blocked=");
  Serial.print(blockedLevel);
  Serial.print(" threshold=");
  Serial.println(threshold);
  Serial.print("THRESHOLD ");
  Serial.println(threshold);

  // come back down below the edge, so the final home approaches from below
  long n = 0;
  while (isBlockedValue(avgSensor())) {
    if (n >= 2 * CAL_MAX_STEPS) return 4;
    if (moveSteps(1, false, false) == MOVE_STOPPED) return 2;
    n++;
  }
  if (moveSteps(CAL_BACKOFF, false, false) == MOVE_STOPPED) return 2;
  return 0;
}

void calibrate() {
  unsigned long oldPeriod = stepPeriodUs;
  stepPeriodUs = 1000000UL / CAL_SPEED;
  Serial.println("CAL started");
  int r = calibrateRun();
  stepPeriodUs = oldPeriod;                            // back to the normal speed
  if (r == 0) reportMove(moveSteps(2000000L, true, true), true);   // final home: pos = 0
  else if (r == 2) reportMove(MOVE_STOPPED, false);
  else if (r == 1) Serial.println("ERR CAL plateau not found");
  else if (r == 3) Serial.println("ERR CAL contrast too small");
  else Serial.println("ERR CAL could not come back down");
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);   // keeps A0 from floating when the beam is blocked
  digitalWrite(EN_PIN, LOW);           // driver on
  Serial.begin(9600);
  Serial.setTimeout(100);
  Serial.println("READY");
}

void loop() {
  streamTick();
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (handleSetting(cmd)) return;

  char c = cmd.charAt(0);

  if (c == 'H') {                       // HOME: go up until the sensor
    if (sensorHit()) {
      pos = 0;
      homed = true;
      Serial.println("HOMED pos=0");
    } else {
      reportMove(moveSteps(2000000L, true, true), true);
    }
  }
  else if (c == 'M') {                  // "M 200" = 200 steps up, "M -200" = down
    long n = cmd.substring(1).toInt();
    reportMove(moveSteps(abs(n), n > 0, true), false);
  }
  else if (c == 'G') {                  // "G 1500" = go to position 1500 (needs homing first)
    long target = cmd.substring(1).toInt();
    if (!homed) {
      Serial.println("ERR NOHOME");
    } else if (target < 0) {
      Serial.println("ERR RANGE");
    } else {
      long d = target - pos;            // d < 0 means up
      reportMove(moveSteps(abs(d), d < 0, true), false);
    }
  }
  else if (c == 'C') {                  // CALIBRATE: threshold = middle of the edge, then home
    calibrate();
  }
  else if (c == '?') {                  // report sensor state, raw value, position
    int v = analogRead(SENSOR_PIN);
    Serial.print(sensorHit() ? "BLOCKED" : "FREE");
    Serial.print(" value=");
    Serial.print(v);
    Serial.print(" threshold=");
    Serial.print(threshold);
    Serial.print(" pos=");
    Serial.print(pos);
    Serial.print(" homed=");
    Serial.println(homed ? 1 : 0);
  }
  else if (c == 'S') {
    Serial.println("STOPPED");
  }
  else {
    Serial.println("ERR");
  }
}
