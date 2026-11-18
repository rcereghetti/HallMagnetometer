// ---------- Settings (change these if needed) ----------
const int STEP_PIN   = 2;    // CNC shield X step
const int DIR_PIN    = 5;    // CNC shield X direction
const int EN_PIN     = 8;    // CNC shield enable (LOW = on)
const int SENSOR_PIN = A0;   // optical sensor (analog)

const bool UP_DIR = HIGH;    // flip to LOW if "up" goes the wrong way

// With the wiring described (resistor from 5 V to the phototransistor
// collector, A0 on that point), a BLOCKED beam gives a HIGH reading.
// Flip this to false if your sensor behaves the opposite way.
const bool BLOCKED_IS_HIGH = true;

int threshold = 500;         // 0-1023, can be changed with the "T" command

const long MIN_SPEED = 20;   // steps per second (limits of the "V" command)
const long MAX_SPEED = 3000;

const unsigned long STREAM_MS = 50;   // time between streamed sensor values
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
// Moving up always stops at the sensor (safety) and sets pos = 0.
// 'S' stops any move. Steps are spaced exactly stepPeriodUs apart.
int moveSteps(long steps, bool up) {
  digitalWrite(DIR_PIN, up ? UP_DIR : !UP_DIR);
  delayMicroseconds(10);
  for (long i = 0; i < steps; i++) {
    unsigned long t0 = micros();
    unsigned long period = stepPeriodUs;

    if (up && sensorHit()) {
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
      reportMove(moveSteps(2000000L, true), true);
    }
  }
  else if (c == 'M') {                  // "M 200" = 200 steps up, "M -200" = down
    long n = cmd.substring(1).toInt();
    reportMove(moveSteps(abs(n), n > 0), false);
  }
  else if (c == 'G') {                  // "G 1500" = go to position 1500 (needs homing first)
    long target = cmd.substring(1).toInt();
    if (!homed) {
      Serial.println("ERR NOHOME");
    } else if (target < 0) {
      Serial.println("ERR RANGE");
    } else {
      long d = target - pos;            // d < 0 means up
      reportMove(moveSteps(abs(d), d < 0), false);
    }
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
