/*
  extruder_node.ino
  Toolhead controller for the 6-axis robotic AM cell.

  Hardware
    Arduino Mega 2560 + W5100 Ethernet shield
    D18  interrupt from PC817 optocoupler   <- IRC5 digital output, extrude gate
    A0   0-5 V from the AO divider          <- IRC5 analog output, extrusion rate
    D5   TMC2209 STEP        D6  DIR        D7  EN   (EN is active LOW)
    D9   MAX31865 CS, SPI shared on 50/51/52
    D11  MOSFET 1 SIG+, heater cartridge
    D12  MOSFET 2 SIG+, part cooling fan

  Pins taken by the shield: 4, 10, 50, 51, 52, 53. Do not reuse.

  Library required: "Adafruit MAX31865" via Library Manager.

  Serial 115200. Same commands over Ethernet on port 8888.
    SET <celsius>   hotend target, e.g. SET 210
    OFF             heater off
    FAN <0-100>     part cooling fan percent
    E <mm>          jog the extruder, e.g. E 100   (used for E-step calibration)
    COLDJOG <mm>    bench test only, turns the motor with NO FILAMENT loaded
    ESTEP <n>       set steps per mm at runtime
    STATUS          one status line
    RESET           clear a latched abort

  V5 (2026-11-01): heater now driven through a NOYITO PC817 opto into the
  HiLetgo MOSFET Control plug (D11 HIGH = heater on, unchanged). A single
  MAX31865 fault read no longer aborts; it must repeat 4 reads in a row.
  STATUS adds RTDERR=<count>/<last fault code>.
*/

#include <SPI.h>
#include <Ethernet.h>
#include <Adafruit_MAX31865.h>

// =====================================================================
//  BENCH MODE SWITCH
// =====================================================================
//  1 = bench testing with a broken or missing RTD.
//        The temperature sensor is ignored entirely.
//        The heater is HARD DISABLED. SET is refused. PWM is forced to 0.
//        tempC is faked at 25 C so the cold-extrude interlock still blocks
//        the automatic gate path and the E command.
//        Only COLDJOG turns the motor. That is the whole point of this mode.
//
//  0 = normal operation. Set this back to 0 the moment the correct
//      MAX31865 board (Adafruit PID 3648, 4300 ohm reference) is fitted.
//      You cannot print in bench mode. The heater will not come on.
// =====================================================================
#define BENCH_NO_RTD 0

// ---------------- pins ----------------
const uint8_t PIN_GATE   = 18;   // hardware interrupt pin. ACTIVE LOW via the PC817.
const uint8_t PIN_RATE   = A0;
const uint8_t PIN_STEP   = 5;
const uint8_t PIN_DIR    = 6;
const uint8_t PIN_EN     = 7;
const uint8_t PIN_MAXCS  = 9;
const uint8_t PIN_HEATER = 11;
const uint8_t PIN_FAN    = 12;

// ---------------- extruder ----------------
// MEASURED 2026-10-18 on the APUS 2, not inherited. Bench method: mark the
// filament past the extruder outlet, measure outlet-to-mark, COLDJOG 50,
// measure again. First pass fed 147 mm for a commanded 50, giving
//   829 * 50 / 147 = 282
// Re-run at 282 fed 50.0 mm for a commanded 50. The 829 it replaced was the
// APUS 1 figure and was out by almost 3x.
// To recalibrate: new = old * commanded / measured. ESTEP sets it at runtime.
float    stepsPerMM     = 282.0;
const float MAX_FEED_MM_S = 10.0;   // what full scale on the analog input means
const float MIN_FEED_MM_S = 0.4;    // below this the timer cannot divide far enough

// Direction. Measured on the bench 2026-10-18: with the coils wired
// black-A2 / orange-A1 / red-B1 / blue-B2, DIR LOW is what feeds filament
// INTO the hotend. Everything above talks in "forward = positive mm" and
// this one flag does the translation, so the robot-driven path, the E
// command and COLDJOG all agree. Flip it if you ever rewire the coils.
const bool DIR_INVERT = true;

inline void setDir(bool forward) {
  digitalWrite(PIN_DIR, (forward != DIR_INVERT) ? HIGH : LOW);
}

// ---------------- PT1000 / MAX31865 ----------------
Adafruit_MAX31865 rtd = Adafruit_MAX31865(PIN_MAXCS);
const float RREF      = 4300.0;   // must match the board marking 4301
const float RNOMINAL  = 1000.0;   // PT1000 at 0 C

// ---------------- limits ----------------
const float    T_TARGET_MAX = 300.0;
const float    T_ABORT      = 320.0;
const float    T_JUMP_ABORT = 25.0;
const float    T_SENSOR_MIN = -20.0;
const float    T_SENSOR_MAX = 450.0;
const float    T_COLD_EXTRUDE = 170.0;   // refuse to step below this
const uint32_t RUNAWAY_MS   = 25000;
const float    RUNAWAY_RISE = 3.0;

// ---------------- PID ----------------
float Kp = 18.0;
float Ki = 1.2;
float Kd = 90.0;
const float I_CLAMP = 120.0;

// ---------------- network ----------------
byte      MAC[]  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xBE, 0x10 };
IPAddress IP(192, 168, 1, 10);
EthernetServer server(8888);

// ---------------- state ----------------
volatile bool  gateOpen   = false;
volatile int32_t stepsLeft = 0;    // for manual jog, -1 means continuous
volatile bool  stepping   = false;

float    target   = 0.0;
float    tempC    = 0.0;
float    tempPrev = 0.0;
float    iTerm    = 0.0;
float    lastErr  = 0.0;
uint8_t  heatPwm  = 0;
uint8_t  fanPwm   = 0;
float    feedNow  = 0.0;
bool     aborted  = false;
char     abortMsg[48] = "";

// V5: one bad RTD read no longer kills the heater. A fault has to repeat on
// RTD_FAULT_LIMIT reads in a row (4 reads = 1 s) before it aborts. A faulted
// read is thrown away and the last good temperature is kept. STATUS shows how
// many were ignored and the last fault code, so a real wiring problem still shows.
const uint8_t RTD_FAULT_LIMIT = 4;
uint8_t  rtdFaultRun   = 0;
uint16_t rtdFaultCount = 0;
uint8_t  rtdLastFault  = 0;

uint32_t lastPid      = 0;
uint32_t lastReport   = 0;
uint32_t lastRate     = 0;
uint32_t runawayStart = 0;
float    runawayRef   = 0.0;

char    inBuf[32];
uint8_t inLen = 0;

// ---------------- step timer (Timer5, CTC) ----------------
// NOT Timer1. On the Mega, pins 11 and 12 are OC1A and OC1B. Putting the step
// period in OCR1A meant every analogWrite() to the heater or the fan silently
// overwrote the step rate, and every step-rate change moved the heater duty.
// Timer5 owns pins 44/45/46, which this board does not use, so the two are
// now independent. Do not move the heater or the fan onto 11/12/13 of Timer1
// without checking this again.
void stepTimerStop() {
  TCCR5B &= ~((1 << CS52) | (1 << CS51) | (1 << CS50));
  stepping = false;
}

void stepTimerStart(float mmPerSec) {
  if (mmPerSec < MIN_FEED_MM_S) { stepTimerStop(); return; }
  float sps = mmPerSec * stepsPerMM;
  uint32_t ocr = (uint32_t)(2000000.0 / sps) - 1;   // 16 MHz / prescaler 8
  if (ocr > 65535) ocr = 65535;
  if (ocr < 40)    ocr = 40;                        // ceiling on step rate
  noInterrupts();
  OCR5A  = (uint16_t)ocr;
  TCNT5  = 0;
  TCCR5B = (1 << WGM52) | (1 << CS51);              // CTC, prescaler 8
  TIMSK5 |= (1 << OCIE5A);
  interrupts();
  stepping = true;
}

ISR(TIMER5_COMPA_vect) {
  if (stepsLeft == 0) return;
  PORTE |= (1 << PE3);            // D5 high
  __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\t");
  PORTE &= ~(1 << PE3);           // D5 low
  if (stepsLeft > 0) stepsLeft--;
}

// ---------------- gate interrupt ----------------
void onGateChange() {
  // PC817 collector pulls the pin DOWN when the IRC5 output is on, so the
  // gate signal arrives inverted. LOW means extrude.
  gateOpen = (digitalRead(PIN_GATE) == LOW);
}

// ---------------- abort ----------------
void doAbort(const char *why) {
  if (aborted) return;
  aborted = true;
  target = 0.0;
  heatPwm = 0;
  analogWrite(PIN_HEATER, 0);
  stepTimerStop();
  stepsLeft = 0;
  digitalWrite(PIN_EN, HIGH);     // release the motor
  strncpy(abortMsg, why, sizeof(abortMsg) - 1);
  Serial.print(F("ABORT: ")); Serial.println(abortMsg);
}

// ---------------- commands ----------------
void reply(Print &out, const char *s) { out.println(s); }

void statusLine(Print &out) {
  out.print(F("T="));    out.print(tempC, 1);
  out.print(F(" SET=")); out.print(target, 1);
  out.print(F(" PWM=")); out.print(heatPwm);
  out.print(F(" GATE=")); out.print(gateOpen ? 1 : 0);
  out.print(F(" FEED=")); out.print(feedNow, 2);
  out.print(F(" FAN="));  out.print(fanPwm);
  out.print(F(" RTDERR=")); out.print(rtdFaultCount);
  if (rtdFaultCount) { out.print(F("/0x")); out.print(rtdLastFault, HEX); }
#if BENCH_NO_RTD
  out.print(F(" STATE=")); out.println(aborted ? abortMsg : "BENCH heater disabled");
#else
  out.print(F(" STATE=")); out.println(aborted ? abortMsg : "ok");
#endif
}

void handleCommand(char *line, Print &out) {
  while (*line == ' ') line++;
  for (char *p = line; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

  if (strncmp(line, "SET", 3) == 0) {
#if BENCH_NO_RTD
    reply(out, "ERR bench mode, heater disabled, no temperature sensor");
#else
    float v = atof(line + 3);
    if (aborted)                   { reply(out, "ERR aborted, send RESET"); return; }
    if (v < 0 || v > T_TARGET_MAX) { reply(out, "ERR setpoint out of range"); return; }
    target = v; iTerm = 0.0;
    runawayStart = millis(); runawayRef = tempC;
    reply(out, "OK");
#endif
  }
  else if (strncmp(line, "OFF", 3) == 0) { target = 0.0; iTerm = 0.0; reply(out, "OK"); }
  else if (strncmp(line, "FAN", 3) == 0) {
    int v = atoi(line + 3);
    if (v < 0) v = 0; if (v > 100) v = 100;
    fanPwm = (uint8_t)(v * 255L / 100);
    analogWrite(PIN_FAN, fanPwm);
    reply(out, "OK");
  }
  else if (line[0] == 'E' && line[1] == ' ') {
    if (aborted)               { reply(out, "ERR aborted"); return; }
    if (tempC < T_COLD_EXTRUDE) { reply(out, "ERR cold extrude blocked"); return; }
    float mm = atof(line + 2);
    digitalWrite(PIN_EN, LOW);
    setDir(mm >= 0);
    noInterrupts();
    stepsLeft = (int32_t)(fabs(mm) * stepsPerMM);
    interrupts();
    stepTimerStart(3.0);
    reply(out, "OK");
  }
  else if (strncmp(line, "COLDJOG", 7) == 0) {
    // Bench test only. Turns the motor with the cold-extrude interlock bypassed,
    // so there must be NO FILAMENT loaded. Cold extrusion chews the drive gears.
    if (aborted) { reply(out, "ERR aborted"); return; }
    float mm = atof(line + 7);
    if (fabs(mm) > 50.0) { reply(out, "ERR coldjog limited to 50 mm"); return; }
    digitalWrite(PIN_EN, LOW);
    setDir(mm >= 0);
    noInterrupts();
    stepsLeft = (int32_t)(fabs(mm) * stepsPerMM);
    interrupts();
    stepTimerStart(2.0);
    reply(out, "OK coldjog, no filament should be loaded");
  }
  else if (strncmp(line, "ESTEP", 5) == 0) {
    float v = atof(line + 5);
    if (v > 10 && v < 5000) { stepsPerMM = v; reply(out, "OK"); }
    else reply(out, "ERR out of range");
  }
  else if (strncmp(line, "STATUS", 6) == 0) statusLine(out);
  else if (strncmp(line, "RESET", 5) == 0)  { aborted = false; abortMsg[0] = 0; iTerm = 0.0; rtdFaultRun = 0; reply(out, "OK"); }
  else reply(out, "ERR unknown");
}

// ---------------- setup ----------------
void setup() {
  pinMode(PIN_STEP, OUTPUT);   digitalWrite(PIN_STEP, LOW);
  pinMode(PIN_DIR, OUTPUT);    setDir(true);
  pinMode(PIN_EN, OUTPUT);     digitalWrite(PIN_EN, HIGH);   // disabled at boot
  pinMode(PIN_HEATER, OUTPUT); analogWrite(PIN_HEATER, 0);
  pinMode(PIN_FAN, OUTPUT);    analogWrite(PIN_FAN, 0);
  pinMode(PIN_GATE, INPUT_PULLUP);   // no external pull-up fitted
  pinMode(53, OUTPUT);              // required for SPI master mode on the Mega
  pinMode(10, OUTPUT); digitalWrite(10, HIGH);   // W5100 CS idle high
  pinMode(4,  OUTPUT); digitalWrite(4,  HIGH);   // shield SD card CS idle high

  Serial.begin(115200);
  Serial.println(F("extruder node starting"));
#if BENCH_NO_RTD
  Serial.println(F("*****************************************************"));
  Serial.println(F("*  BENCH MODE. RTD IGNORED. HEATER HARD DISABLED.   *"));
  Serial.println(F("*  Motor test only, via COLDJOG. NO FILAMENT.       *"));
  Serial.println(F("*  Set BENCH_NO_RTD back to 0 before any printing.  *"));
  Serial.println(F("*****************************************************"));
#endif

  rtd.begin(MAX31865_2WIRE);

  TCCR5A = 0; TCCR5B = 0; TIMSK5 = 0;
  attachInterrupt(digitalPinToInterrupt(PIN_GATE), onGateChange, CHANGE);

  Ethernet.begin(MAC, IP);
  server.begin();
  Serial.print(F("listening on ")); Serial.print(Ethernet.localIP()); Serial.println(F(":8888"));

#if BENCH_NO_RTD
  tempC = 25.0;                 // fake, keeps the cold-extrude interlock armed
#else
  tempC = rtd.temperature(RNOMINAL, RREF);
  if (isnan(tempC)) tempC = 20.0;
#endif
  tempPrev = tempC;
  lastPid = millis();
}

// ---------------- loop ----------------
void loop() {
  uint32_t now = millis();

  // --- temperature + PID at 4 Hz ---
  if (now - lastPid >= 250) {
    float dt = (now - lastPid) / 1000.0;
    lastPid = now;

#if BENCH_NO_RTD
    // Sensor ignored completely. Hold a fake 25 C so the cold-extrude
    // interlock stays armed and only COLDJOG can turn the motor.
    rtd.clearFault();
    tempC = 25.0; tempPrev = 25.0; target = 0.0;
#else
    float t = rtd.temperature(RNOMINAL, RREF);
    uint8_t fault = rtd.readFault();
    if (fault) {
      rtd.clearFault();
      rtdLastFault = fault;
      if (rtdFaultCount < 65535) rtdFaultCount++;
      if (++rtdFaultRun >= RTD_FAULT_LIMIT) {
        char m[40];
        snprintf(m, sizeof(m), "RTD fault 0x%02X", fault);
        doAbort(m);
      }
      // faulted read is not trusted: keep the last good tempC
    } else {
      rtdFaultRun = 0;
      if (isnan(t) || t < T_SENSOR_MIN || t > T_SENSOR_MAX) doAbort("sensor out of range");
      else {
        if (fabs(t - tempPrev) > T_JUMP_ABORT) doAbort("temperature jump");
        tempPrev = t; tempC = t;
      }
    }

    if (tempC > T_ABORT) doAbort("over temperature");
#endif

    if (!aborted && target > 0 && heatPwm > 200) {
      if (now - runawayStart > RUNAWAY_MS) {
        if (tempC - runawayRef < RUNAWAY_RISE) doAbort("no rise under full drive");
        runawayStart = now; runawayRef = tempC;
      }
    } else { runawayStart = now; runawayRef = tempC; }

    if (aborted || target <= 0.0) { heatPwm = 0; iTerm = 0.0; }
    else {
      float err = target - tempC;
      iTerm += Ki * err * dt;
      if (iTerm > I_CLAMP) iTerm = I_CLAMP;
      if (iTerm < 0)       iTerm = 0;
      float dTerm = Kd * (err - lastErr) / dt;
      lastErr = err;
      float out = Kp * err + iTerm + dTerm;
      if (out < 0)   out = 0;
      if (out > 255) out = 255;
      heatPwm = (uint8_t)out;
    }
#if BENCH_NO_RTD
    heatPwm = 0;                       // belt and braces, never drive the heater
#endif
    analogWrite(PIN_HEATER, heatPwm);
  }

  // --- extrusion gate + rate at 50 Hz ---
  if (now - lastRate >= 20) {
    lastRate = now;
    bool manualJog;
    noInterrupts(); manualJog = (stepsLeft > 0); interrupts();

    if (!manualJog) {
      if (aborted || !gateOpen || tempC < T_COLD_EXTRUDE) {
        stepTimerStop();
        noInterrupts(); stepsLeft = 0; interrupts();
        digitalWrite(PIN_EN, HIGH);
        feedNow = 0.0;
      } else {
        int raw = analogRead(PIN_RATE);
        feedNow = (raw / 1023.0) * MAX_FEED_MM_S;
        digitalWrite(PIN_EN, LOW);
        setDir(true);
        noInterrupts(); stepsLeft = -1; interrupts();   // continuous
        stepTimerStart(feedNow);
      }
    } else {
      int32_t remaining;
      noInterrupts(); remaining = stepsLeft; interrupts();
      if (remaining == 0) { stepTimerStop(); digitalWrite(PIN_EN, HIGH); }
    }
  }

  // --- serial ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') { if (inLen) { inBuf[inLen] = 0; handleCommand(inBuf, Serial); inLen = 0; } }
    else if (inLen < sizeof(inBuf) - 1) inBuf[inLen++] = c;
  }

  // --- ethernet ---
  EthernetClient client = server.available();
  if (client) {
    char buf[32]; uint8_t n = 0; uint32_t t0 = millis();
    while (client.connected() && millis() - t0 < 200) {
      while (client.available()) {
        char c = client.read();
        if (c == '\n' || c == '\r') { if (n) { buf[n] = 0; handleCommand(buf, client); n = 0; } }
        else if (n < sizeof(buf) - 1) buf[n++] = c;
        t0 = millis();
      }
    }
    client.stop();
  }

  if (now - lastReport >= 2000) { lastReport = now; statusLine(Serial); }
}
