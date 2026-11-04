/*
  bed_node.ino
  Heated bed controller for the 6-axis robotic AM cell.

  Hardware
    Arduino Mega 2560 + W5100 Ethernet shield
    A0   100k NTC through a 4k7 1% divider  (5V - 4k7 - A0 - NTC - GND)
    D8   SSR-40 DA terminal 3 (+), Mega GND to terminal 4 (-)

  Pins taken by the shield: 4, 10, 50, 51, 52, 53. Do not reuse.

  Serial: 115200 baud. Commands also work over Ethernet on port 8888.
    SET <celsius>    set target, e.g.  SET 60
    OFF              target 0, heater off
    STATUS           print one status line
    RESET            clear a latched abort

  SAFETY
    The SSR fails closed. This firmware is the second line of defence,
    not the first. The KSD301 thermal cutoff in the mains leg is what
    actually protects you. Do not run this bed unattended.
*/

#include <SPI.h>
#include <Ethernet.h>

// ---------------- pins ----------------
const uint8_t PIN_SSR   = 8;
const uint8_t PIN_THERM = A0;

// ---------------- divider / thermistor ----------------
const float R_FIXED    = 4700.0;    // 4k7 1% metal film
const float R_NOMINAL  = 100000.0;  // NTC resistance at 25 C
const float T_NOMINAL  = 25.0;      // C
const float BETA       = 3950.0;    // typical for a 100k bed thermistor
const float ADC_MAX    = 1023.0;

// ---------------- limits ----------------
const float    T_TARGET_MAX  = 110.0;   // highest setpoint accepted
const float    T_ABORT       = 120.0;   // hard abort
const float    T_JUMP_ABORT  = 20.0;    // C between consecutive samples
const float    T_SENSOR_MIN  = -15.0;   // open / shorted sensor
const float    T_SENSOR_MAX  = 250.0;
const uint32_t RUNAWAY_MS    = 45000;   // full drive with no rise
const float    RUNAWAY_RISE  = 2.0;     // C that must appear in that window

// ---------------- PID ----------------
// Starting values. Tune these once the bed is running.
float Kp = 22.0;
float Ki = 0.9;
float Kd = 120.0;
const float I_CLAMP = 60.0;

// ---------------- slow PWM ----------------
const uint32_t WINDOW_MS = 1000;   // SSR switches at zero crossing, keep this slow

// ---------------- network ----------------
byte      MAC[]  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x11 };
IPAddress IP(192, 168, 1, 11);
EthernetServer server(8888);

// ---------------- state ----------------
float    target    = 0.0;
float    tempC     = 0.0;
float    tempPrev  = 0.0;
float    iTerm     = 0.0;
float    lastErr   = 0.0;
float    duty      = 0.0;         // 0..1
bool     aborted   = false;
char     abortMsg[48] = "";

uint32_t windowStart   = 0;
uint32_t lastPid       = 0;
uint32_t lastReport    = 0;
uint32_t runawayStart  = 0;
float    runawayRef    = 0.0;

char     inBuf[32];
uint8_t  inLen = 0;

// ---------------- temperature ----------------
float readTempC() {
  // average 16 samples to knock down ADC noise
  uint32_t acc = 0;
  for (uint8_t i = 0; i < 16; i++) { acc += analogRead(PIN_THERM); delayMicroseconds(200); }
  float raw = acc / 16.0;

  if (raw < 2.0 || raw > ADC_MAX - 2.0) return NAN;   // open or shorted

  // 5V - R_FIXED - A0 - NTC - GND   =>   R_ntc = R_FIXED * raw / (ADC_MAX - raw)
  float r = R_FIXED * raw / (ADC_MAX - raw);

  float steinhart = log(r / R_NOMINAL) / BETA;
  steinhart += 1.0 / (T_NOMINAL + 273.15);
  return (1.0 / steinhart) - 273.15;
}

// ---------------- abort ----------------
void doAbort(const char *why) {
  if (aborted) return;
  aborted = true;
  target = 0.0;
  duty = 0.0;
  digitalWrite(PIN_SSR, LOW);
  strncpy(abortMsg, why, sizeof(abortMsg) - 1);
  Serial.print(F("ABORT: ")); Serial.println(abortMsg);
}

// ---------------- command handling ----------------
void reply(Print &out, const char *s) { out.println(s); }

void statusLine(Print &out) {
  out.print(F("T=")); out.print(tempC, 1);
  out.print(F(" SET=")); out.print(target, 1);
  out.print(F(" DUTY=")); out.print(duty * 100.0, 0);
  out.print(F(" STATE=")); out.println(aborted ? abortMsg : "ok");
}

void handleCommand(char *line, Print &out) {
  while (*line == ' ') line++;
  for (char *p = line; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

  if (strncmp(line, "SET", 3) == 0) {
    float v = atof(line + 3);
    if (aborted)              { reply(out, "ERR aborted, send RESET"); return; }
    if (v < 0 || v > T_TARGET_MAX) { reply(out, "ERR setpoint out of range"); return; }
    target = v;
    iTerm = 0.0;
    runawayStart = millis();
    runawayRef = tempC;
    reply(out, "OK");
  }
  else if (strncmp(line, "OFF", 3) == 0)    { target = 0.0; iTerm = 0.0; reply(out, "OK"); }
  else if (strncmp(line, "STATUS", 6) == 0) { statusLine(out); }
  else if (strncmp(line, "RESET", 5) == 0)  { aborted = false; abortMsg[0] = 0; iTerm = 0.0; reply(out, "OK"); }
  else                                      { reply(out, "ERR unknown"); }
}

// ---------------- setup ----------------
void setup() {
  pinMode(PIN_SSR, OUTPUT);
  digitalWrite(PIN_SSR, LOW);
  pinMode(53, OUTPUT);           // required for SPI master mode on the Mega

  Serial.begin(115200);
  Serial.println(F("bed node starting"));

  Ethernet.begin(MAC, IP);
  server.begin();
  Serial.print(F("listening on ")); Serial.print(Ethernet.localIP()); Serial.println(F(":8888"));

  tempC = readTempC();
  if (isnan(tempC)) tempC = 20.0;
  tempPrev = tempC;
  windowStart = millis();
  lastPid = millis();
}

// ---------------- loop ----------------
void loop() {
  uint32_t now = millis();

  // --- PID at 4 Hz ---
  if (now - lastPid >= 250) {
    float dt = (now - lastPid) / 1000.0;
    lastPid = now;

    float t = readTempC();

    if (isnan(t) || t < T_SENSOR_MIN || t > T_SENSOR_MAX) {
      doAbort("sensor fault");
    } else {
      if (fabs(t - tempPrev) > T_JUMP_ABORT) doAbort("temperature jump");
      tempPrev = t;
      tempC = t;
    }

    if (tempC > T_ABORT) doAbort("over temperature");

    // thermal runaway: driving hard and nothing is happening
    if (!aborted && target > 0 && duty > 0.8) {
      if (now - runawayStart > RUNAWAY_MS) {
        if (tempC - runawayRef < RUNAWAY_RISE) doAbort("no rise under full drive");
        runawayStart = now;
        runawayRef = tempC;
      }
    } else {
      runawayStart = now;
      runawayRef = tempC;
    }

    if (aborted || target <= 0.0) {
      duty = 0.0;
      iTerm = 0.0;
    } else {
      float err = target - tempC;
      iTerm += Ki * err * dt;
      if (iTerm > I_CLAMP)  iTerm = I_CLAMP;
      if (iTerm < 0)        iTerm = 0;
      float dTerm = Kd * (err - lastErr) / dt;
      lastErr = err;

      float out = Kp * err + iTerm + dTerm;
      duty = out / 255.0;
      if (duty < 0) duty = 0;
      if (duty > 1) duty = 1;
    }
  }

  // --- slow PWM window ---
  if (now - windowStart >= WINDOW_MS) windowStart += WINDOW_MS;
  bool on = (!aborted) && (duty > 0.01) && ((now - windowStart) < (uint32_t)(duty * WINDOW_MS));
  digitalWrite(PIN_SSR, on ? HIGH : LOW);

  // --- serial commands ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLen) { inBuf[inLen] = 0; handleCommand(inBuf, Serial); inLen = 0; }
    } else if (inLen < sizeof(inBuf) - 1) inBuf[inLen++] = c;
  }

  // --- ethernet commands ---
  EthernetClient client = server.available();
  if (client) {
    char buf[32]; uint8_t n = 0;
    uint32_t t0 = millis();
    while (client.connected() && millis() - t0 < 200) {
      while (client.available()) {
        char c = client.read();
        if (c == '\n' || c == '\r') {
          if (n) { buf[n] = 0; handleCommand(buf, client); n = 0; }
        } else if (n < sizeof(buf) - 1) buf[n++] = c;
        t0 = millis();
      }
    }
    client.stop();
  }

  // --- status every 2 s ---
  if (now - lastReport >= 2000) { lastReport = now; statusLine(Serial); }
}
