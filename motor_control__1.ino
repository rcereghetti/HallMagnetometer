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

int threshold = 512;         // 0-1023, can be changed with the "T" command
const unsigned int STEP_DELAY_US = 800; // smaller = faster motor

const unsigned long STREAM_MS = 50;     // time between streamed sensor values
// --------------------------------------------------------

bool streaming = false;      // "W 1" = send the raw sensor value, "W 0" = stop
unsigned long lastStream = 0;

bool sensorHit() {
  int v = analogRead(SENSOR_PIN);
  return BLOCKED_IS_HIGH ? (v > threshold) : (v < threshold);
}

// While streaming, sends "A <value>" every STREAM_MS (also while moving).
void streamTick() {
  if (!streaming) return;
  unsigned long now = millis();
  if (now - lastStream >= STREAM_MS) {
    lastStream = now;
    Serial.print("A ");
    Serial.println(analogRead(SENSOR_PIN));
  }
}

// "W 1" / "W 0"
void handleStream(String cmd) {
  streaming = (cmd.substring(1).toInt() != 0);
  Serial.println(streaming ? "STREAM ON" : "STREAM OFF");
}

// Moves the motor. Returns true if it stopped because of the sensor.
// Moving up always stops at the sensor (safety). 'S' stops any move.
bool moveSteps(long steps, bool up) {
  digitalWrite(DIR_PIN, up ? UP_DIR : !UP_DIR);
  delayMicroseconds(10);
  for (long i = 0; i < steps; i++) {
    if (up && sensorHit()) return true;
    if (Serial.available()) {
      char p = Serial.peek();
      if (p == 'S') {
        Serial.read();
        return false;
      }
      if (p == 'W') {               // stream on/off must not block the STOP
        handleStream(Serial.readStringUntil('\n'));
      }
    }
    streamTick();
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
  return false;
}

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);      // driver on
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

  char c = cmd.charAt(0);

  if (c == 'H') {                       // HOME: go up until the sensor
    if (sensorHit()) {
      Serial.println("HOMED");
    } else {
      bool hit = moveSteps(2000000L, true);
      Serial.println(hit ? "HOMED" : "STOPPED");
    }
  }
  else if (c == 'M') {                  // "M 200" = 200 steps up, "M -200" = down
    long n = cmd.substring(1).toInt();
    bool up = (n > 0);
    bool hit = moveSteps(abs(n), up);
    Serial.println(hit ? "SENSOR" : "DONE");
  }
  else if (c == 'T') {                  // "T 400" = set threshold to 400
    int t = cmd.substring(1).toInt();
    if (t >= 0 && t <= 1023) {
      threshold = t;
      Serial.print("THRESHOLD ");
      Serial.println(threshold);
    } else {
      Serial.println("ERR");
    }
  }
  else if (c == 'W') {                  // "W 1" = stream sensor value, "W 0" = stop
    handleStream(cmd);
  }
  else if (c == '?') {                  // report sensor state and raw value
    int v = analogRead(SENSOR_PIN);
    Serial.print(sensorHit() ? "BLOCKED" : "FREE");
    Serial.print(" value=");
    Serial.print(v);
    Serial.print(" threshold=");
    Serial.println(threshold);
  }
  else if (c == 'S') {
    Serial.println("STOPPED");
  }
  else {
    Serial.println("ERR");
  }
}
