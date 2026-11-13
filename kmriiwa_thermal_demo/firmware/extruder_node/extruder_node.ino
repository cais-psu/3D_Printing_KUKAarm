/*
  extruder_node.ino
  Toolhead controller for the 6-axis robotic AM cell.

  Based on cais-psu/3D_Printing_KUKAarm at
  0933da52dc42983a0212b5a4731fdb199820c6cf. The additions implement timed
  and leased manual feed and retain D18/A0 control for external operation.

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
    JOG <mm/s> <ms> one bounded forward feed with the external gate inactive
    FEED <mm/s> <8-hex-token>  start manual feed with a 1500 ms lease
    KEEP <8-hex-token>         renew an active lease with the same token
    STOP            stop manual feed, timed or continuous
    STATUS          one status line
    RESET           clear a latched abort

  V5 (2026-11-01): heater now driven through a NOYITO PC817 opto into the
  HiLetgo MOSFET Control plug (D11 HIGH = heater on, unchanged).
  One MAX31865 fault pauses continuous feed and turns off heater output.
  Five consecutive faults end the feed lease; ten latch the heater off.
  STATUS reports RTDERR=<count>/<last fault code>, STOP_REASON=<code>,
  and PAUSE=<none|rtd_fault> for the leased manual feed.
*/

#include <SPI.h>
#include <Ethernet.h>
#include <Adafruit_MAX31865.h>
#include <util/atomic.h>

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
const uint16_t MAX_JOG_MS = 5000;
const uint16_t FEED_LEASE_MS = 1500;
const uint16_t MANUAL_SENSOR_MAX_AGE_MS = 500;
const uint16_t MANUAL_RTD_PAUSE_MS = 500;
const uint32_t STEP_TIMER_HZ = 2000000UL;

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
enum ManualFeedMode : uint8_t { MANUAL_INACTIVE, MANUAL_TIMED, MANUAL_LEASED };
enum FeedStopReason : uint8_t {
  STOP_NONE, STOP_RTD_FAULT, STOP_TEMPERATURE_LOW, STOP_INVALID_TEMPERATURE,
  STOP_TEMPERATURE_JUMP, STOP_OVER_TEMPERATURE, STOP_SENSOR_STALE,
  STOP_LEASE_TIMEOUT, STOP_GATE, STOP_OPERATOR, STOP_HEAT_OFF, STOP_TARGET_LOW,
  STOP_TIMED_DONE, STOP_HEATER_RUNAWAY, STOP_RESET, STOP_ABORTED
};
void startManualFeed(float rate, uint16_t duration, ManualFeedMode mode,
                     uint32_t token, Print &out);
void stopManualFeedLocked(FeedStopReason reason);
void stopManualFeed(FeedStopReason reason);
FeedStopReason manualTimeStopReason(uint32_t now);
FeedStopReason manualTemperatureStopReason();
const __FlashStringHelper *feedStopReasonCode(FeedStopReason reason);
void doAbort(const char *why, FeedStopReason reason);

volatile bool  gateOpen   = false;
volatile int32_t stepsLeft = 0;    // for manual jog, -1 means continuous
volatile bool  stepping   = false;
volatile ManualFeedMode manualMode = MANUAL_INACTIVE;
volatile bool manualPaused = false;
volatile FeedStopReason lastFeedStop = STOP_NONE;
volatile bool  inhibitAnalogGate = false;
volatile uint32_t manualDeadline = 0;
volatile uint32_t manualPauseDeadline = 0;
volatile uint32_t lastGoodRtd = 0;
uint32_t leaseToken = 0;           // retained until the next accepted manual feed
bool     rtdSampleValid = false;
float    manualRate = 0.0;

float    target   = 0.0;
float    tempC    = 0.0;
float    tempPrev = 0.0;
float    iTerm    = 0.0;
float    lastErr  = 0.0;
uint8_t  heatPwm  = 0;
uint8_t  lastValidHeatPwm = 0;
uint8_t  fanPwm   = 0;
float    feedNow  = 0.0;
bool     aborted  = false;
char     abortMsg[48] = "";

const uint8_t FEED_RTD_FAULT_LIMIT = 5;
const uint8_t HEATER_RTD_FAULT_LIMIT = 10;
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
bool    serialOverflow = false;
EthernetClient ethernetClient;
char ethernetBuf[32];
uint8_t ethernetLen = 0;
bool ethernetOverflow = false;
uint32_t ethernetStarted = 0;

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

// Call with interrupts masked. Only the first stop of active manual feed wins.
void stopManualFeedLocked(FeedStopReason reason) {
  if (manualMode == MANUAL_INACTIVE) return;
  manualMode = MANUAL_INACTIVE;
  manualPaused = false;
  if (digitalRead(PIN_GATE) == LOW) inhibitAnalogGate = true;
  stepTimerStop();
  stepsLeft = 0;
  digitalWrite(PIN_EN, HIGH);
  lastFeedStop = reason;
}

void stopManualFeed(FeedStopReason reason) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    stopManualFeedLocked(reason);
  }
}

// Called only from an ISR or with interrupts masked; deadlines are 32-bit.
FeedStopReason manualTimeStopReason(uint32_t now) {
  if ((int32_t)(now - manualDeadline) >= 0) {
    return manualMode == MANUAL_TIMED ? STOP_TIMED_DONE : STOP_LEASE_TIMEOUT;
  }
  if (!manualPaused && now - lastGoodRtd >= MANUAL_SENSOR_MAX_AGE_MS) return STOP_SENSOR_STALE;
  return STOP_NONE;
}

void handleRtdFault() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    rtdSampleValid = false;
    if (manualMode == MANUAL_INACTIVE) {
      stepTimerStop();
      stepsLeft = 0;
      digitalWrite(PIN_EN, HIGH);
      feedNow = 0.0;
    } else if (manualMode == MANUAL_TIMED || rtdFaultRun >= FEED_RTD_FAULT_LIMIT) {
      stopManualFeedLocked(STOP_RTD_FAULT);
    } else if (!manualPaused) {
      const uint32_t now = millis();
      const FeedStopReason reason = manualTimeStopReason(now);
      if (reason != STOP_NONE) {
        stopManualFeedLocked(reason);
      } else if (gateOpen || digitalRead(PIN_GATE) == LOW) {
        stopManualFeedLocked(STOP_GATE);
      } else {
        // Keep Timer5 active for watchdogs. Its ISR must suppress all pulses.
        manualPaused = true;
        manualPauseDeadline = now + MANUAL_RTD_PAUSE_MS;
        stepping = false;
        digitalWrite(PIN_EN, HIGH);
      }
    }
  }
}

void resumeManualAfterRtdSample(uint32_t sampledAt) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (manualMode == MANUAL_LEASED && manualPaused) {
      const uint32_t now = millis();
      FeedStopReason reason = manualTimeStopReason(now);
      if (reason == STOP_NONE && (gateOpen || digitalRead(PIN_GATE) == LOW)) reason = STOP_GATE;
      // The deadline only permits automatic recovery. Keep the motor disabled
      // while waiting for the fault count, even after this window closes.
      if (reason == STOP_NONE && (int32_t)(now - manualPauseDeadline) >= 0) reason = STOP_SENSOR_STALE;
      if (reason == STOP_NONE) reason = manualTemperatureStopReason();
      if (reason == STOP_NONE && now - sampledAt >= MANUAL_SENSOR_MAX_AGE_MS) reason = STOP_SENSOR_STALE;
      if (reason != STOP_NONE) {
        stopManualFeedLocked(reason);
      } else {
        // Restart the pulse period and discard any pending Timer5 comparison.
        lastGoodRtd = sampledAt;
        digitalWrite(PIN_EN, LOW);
        TCNT5 = 0;
        TIFR5 = (1 << OCF5A);
        stepping = true;
        manualPaused = false;
      }
    }
  }
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
  if (manualMode != MANUAL_INACTIVE) {
    const FeedStopReason reason = manualTimeStopReason(millis());
    if (reason != STOP_NONE) {
      stopManualFeedLocked(reason);
      return;
    }
    if (manualPaused) return;
  }
  if (stepsLeft == 0) return;
  PORTE |= (1 << PE3);            // D5 high
  __asm__ __volatile__("nop\n\tnop\n\tnop\n\tnop\n\t");
  PORTE &= ~(1 << PE3);           // D5 low
  if (stepsLeft > 0) stepsLeft--;
  if (manualMode == MANUAL_TIMED && stepsLeft == 0) stopManualFeedLocked(STOP_TIMED_DONE);
}

// ---------------- gate interrupt ----------------
void onGateChange() {
  // PC817 collector pulls the pin DOWN when the IRC5 output is on, so the
  // gate signal arrives inverted. LOW means extrude.
  gateOpen = (digitalRead(PIN_GATE) == LOW);
  if (gateOpen && manualMode != MANUAL_INACTIVE) {
    // A conflict must release the gate before analog feed can start again.
    inhibitAnalogGate = true;
    stopManualFeedLocked(STOP_GATE);
  }
  if (!gateOpen) inhibitAnalogGate = false;
}

// ---------------- abort ----------------
void doAbort(const char *why, FeedStopReason reason) {
  if (aborted) return;
  stopManualFeed(reason);
  aborted = true;
  target = 0.0;
  heatPwm = 0;
  lastValidHeatPwm = 0;
  analogWrite(PIN_HEATER, 0);
  stepTimerStop();
  stepsLeft = 0;
  digitalWrite(PIN_EN, HIGH);     // release the motor
  strncpy(abortMsg, why, sizeof(abortMsg) - 1);
  Serial.print(F("ABORT: ")); Serial.println(abortMsg);
}

// ---------------- commands ----------------
void reply(Print &out, const char *s) { out.println(s); }

const __FlashStringHelper *feedStopReasonCode(FeedStopReason reason) {
  switch (reason) {
    case STOP_NONE: return F("none");
    case STOP_RTD_FAULT: return F("rtd_fault");
    case STOP_TEMPERATURE_LOW: return F("temperature_low");
    case STOP_INVALID_TEMPERATURE: return F("invalid_temperature");
    case STOP_TEMPERATURE_JUMP: return F("temperature_jump");
    case STOP_OVER_TEMPERATURE: return F("over_temperature");
    case STOP_SENSOR_STALE: return F("sensor_stale");
    case STOP_LEASE_TIMEOUT: return F("lease_timeout");
    case STOP_GATE: return F("gate");
    case STOP_OPERATOR: return F("operator_stop");
    case STOP_HEAT_OFF: return F("heat_off");
    case STOP_TARGET_LOW: return F("target_low");
    case STOP_TIMED_DONE: return F("timed_done");
    case STOP_HEATER_RUNAWAY: return F("heater_runaway");
    case STOP_RESET: return F("reset");
    case STOP_ABORTED: return F("aborted");
  }
  return F("unknown");
}

void statusLine(Print &out) {
  out.print(F("T="));    out.print(tempC, 1);
  out.print(F(" SET=")); out.print(target, 1);
  out.print(F(" PWM=")); out.print(heatPwm);
  out.print(F(" GATE=")); out.print(gateOpen ? 1 : 0);
  out.print(F(" FEED=")); out.print(feedNow, 2);
  out.print(F(" FAN="));  out.print(fanPwm);
  out.print(F(" RTDERR=")); out.print(rtdFaultCount);
  if (rtdFaultCount) { out.print(F("/0x")); out.print(rtdLastFault, HEX); }
  FeedStopReason reason;
  bool paused;
  float activeRate;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    reason = lastFeedStop;
    paused = manualPaused;
    activeRate = manualMode != MANUAL_INACTIVE && !paused ? manualRate : 0.0;
  }
  out.print(F(" JOG=1 LEASE=1 ACTIVE=")); out.print(activeRate, 3);
  out.print(F(" STOP_REASON=")); out.print(feedStopReasonCode(reason));
  out.print(F(" PAUSE=")); out.print(paused ? F("rtd_fault") : F("none"));
#if BENCH_NO_RTD
  out.print(F(" STATE=")); out.println(aborted ? abortMsg : "BENCH heater disabled");
#else
  out.print(F(" STATE=")); out.println(aborted ? abortMsg : "ok");
#endif
}

bool parseManualRate(const char *&args, float &rate) {
  // Accept an unsigned decimal speed with a following argument.
  while (*args == ' ') args++;
  const char *rateStart = args;
  while (*args >= '0' && *args <= '9') args++;
  if (*args == '.') {
    args++;
    const char *fractionStart = args;
    while (*args >= '0' && *args <= '9') args++;
    if (args == fractionStart) return false;
  }
  if (args == rateStart || *args != ' ') return false;
  rate = atof(rateStart);
  if (!isfinite(rate) || rate < MIN_FEED_MM_S || rate > MAX_FEED_MM_S) return false;
  while (*args == ' ') args++;
  return true;
}

bool parseJogDuration(const char *args, uint16_t &duration) {
  if (*args < '0' || *args > '9') return false;
  uint16_t value = 0;
  while (*args >= '0' && *args <= '9') {
    const uint32_t next = 10UL * value + (*args++ - '0');
    if (next > MAX_JOG_MS) return false;
    value = (uint16_t)next;
  }
  while (*args == ' ') args++;
  if (*args != '\0' || value == 0) return false;
  duration = value;
  return true;
}

bool parseLeaseToken(const char *args, uint32_t &token) {
  while (*args == ' ') args++;
  uint32_t value = 0;
  for (uint8_t i = 0; i < 8; i++, args++) {
    const char digit = *args;
    uint8_t nibble;
    if (digit >= '0' && digit <= '9') nibble = digit - '0';
    else if (digit >= 'A' && digit <= 'F') nibble = digit - 'A' + 10;
    else return false;
    value = (value << 4) | nibble;
  }
  if (*args != '\0' || value == 0) return false;
  token = value;
  return true;
}

FeedStopReason manualTemperatureStopReason() {
  if (aborted) return STOP_ABORTED;
  if (!rtdSampleValid || !isfinite(tempC) || !isfinite(target)) return STOP_INVALID_TEMPERATURE;
  if (tempC < T_COLD_EXTRUDE) return STOP_TEMPERATURE_LOW;
  if (target < T_COLD_EXTRUDE) return STOP_TARGET_LOW;
  return STOP_NONE;
}

void startManualFeed(float rate, uint16_t duration, ManualFeedMode mode,
                     uint32_t token, Print &out) {
  if (manualTemperatureStopReason() != STOP_NONE) {
    reply(out, "ERR manual feed requires a valid, hot nozzle and target");
    return;
  }

  // Round the period up, so Timer5 does not exceed the requested speed.
  const float ticks = ceilf(STEP_TIMER_HZ / (rate * stepsPerMM));
  if (!isfinite(ticks) || ticks < 41 || ticks > 65536.0) {
    reply(out, "ERR manual feed speed outside timer range for E-steps");
    return;
  }
  const uint32_t period = (uint32_t)ticks;
  const int32_t steps = mode == MANUAL_LEASED ? -1 : (int32_t)((2000UL * duration) / period);
  if (steps == 0) {
    reply(out, "ERR JOG duration too short for one step");
    return;
  }
  const float actualRate = STEP_TIMER_HZ / (ticks * stepsPerMM);
  bool started = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    const uint32_t now = millis();
    if (manualMode == MANUAL_INACTIVE && stepsLeft == 0 && !gateOpen && digitalRead(PIN_GATE) == HIGH &&
        now - lastGoodRtd < MANUAL_SENSOR_MAX_AGE_MS) {
      setDir(true);
      digitalWrite(PIN_EN, LOW);
      stepsLeft = steps;
      manualDeadline = now + duration;
      manualRate = actualRate;
      leaseToken = token;
      lastFeedStop = STOP_NONE;
      manualPaused = false;
      manualMode = mode;
      stepping = true;
      OCR5A = (uint16_t)(period - 1);
      TCNT5 = 0;
      TIFR5 = (1 << OCF5A);
      TIMSK5 |= (1 << OCIE5A);
      TCCR5B = (1 << WGM52) | (1 << CS51);
      started = true;
    }
  }
  reply(out, started ? "OK" : "ERR manual feed requires an idle motor, inactive gate, and fresh sensor");
}

void keepManualFeed(const char *args, Print &out) {
  uint32_t token;
  if (!parseLeaseToken(args, token)) {
    reply(out, "ERR KEEP requires an eight-digit hex lease token");
    return;
  }
  // leaseToken is changed only by an accepted start in this foreground loop.
  if (leaseToken != token) {
    reply(out, "ERR KEEP token mismatch");
    return;
  }
  FeedStopReason reason = manualTemperatureStopReason();
  FeedStopReason pauseReason = STOP_NONE;
  if (aborted) pauseReason = STOP_ABORTED;
  else if (!isfinite(target)) pauseReason = STOP_INVALID_TEMPERATURE;
  else if (target < T_COLD_EXTRUDE) pauseReason = STOP_TARGET_LOW;
  bool renewed = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (manualMode == MANUAL_LEASED) {
      const uint32_t now = millis();
      if (manualPaused) reason = pauseReason;
      if (reason == STOP_NONE) reason = manualTimeStopReason(now);
      if (reason == STOP_NONE && (gateOpen || digitalRead(PIN_GATE) == LOW)) reason = STOP_GATE;
      if (reason == STOP_NONE) {
        manualDeadline = now + FEED_LEASE_MS;
        renewed = true;
      } else {
        stopManualFeedLocked(reason);
      }
    }
    reason = lastFeedStop;
  }
  if (renewed) {
    reply(out, "OK");
  } else if (reason != STOP_NONE) {
    out.print(F("ERR feed stopped: "));
    out.println(feedStopReasonCode(reason));
  } else {
    reply(out, "ERR KEEP lease inactive");
  }
}

void handleCommand(char *line, Print &out) {
  while (*line == ' ') line++;
  char *end = line + strlen(line);
  while (end > line && end[-1] == ' ') *--end = '\0';
  for (char *p = line; *p; p++) if (*p >= 'a' && *p <= 'z') *p -= 32;

  if (strncmp(line, "JOG ", 4) == 0 || strcmp(line, "JOG") == 0) {
    const char *args = line + 3;
    float rate;
    uint16_t duration;
    if (parseManualRate(args, rate) && parseJogDuration(args, duration)) {
      startManualFeed(rate, duration, MANUAL_TIMED, 0, out);
    } else {
      reply(out, "ERR JOG requires 0.4-10 mm/s and 1-5000 ms");
    }
  }
  else if (strncmp(line, "FEED ", 5) == 0 || strcmp(line, "FEED") == 0) {
    const char *args = line + 4;
    float rate;
    uint32_t token;
    if (parseManualRate(args, rate) && parseLeaseToken(args, token)) {
      startManualFeed(rate, FEED_LEASE_MS, MANUAL_LEASED, token, out);
    } else {
      reply(out, "ERR FEED requires 0.4-10 mm/s and an eight-digit hex lease token");
    }
  }
  else if (strncmp(line, "KEEP ", 5) == 0 || strcmp(line, "KEEP") == 0) {
    keepManualFeed(line + 4, out);
  }
  else if (strcmp(line, "STOP") == 0) {
    stopManualFeed(STOP_OPERATOR);
    reply(out, "OK");
  }
  else if (strncmp(line, "SET", 3) == 0) {
#if BENCH_NO_RTD
    reply(out, "ERR bench mode, heater disabled, no temperature sensor");
#else
    float v = atof(line + 3);
    if (aborted)                   { reply(out, "ERR aborted, send RESET"); return; }
    if (!isfinite(v) || v < 0 || v > T_TARGET_MAX) { reply(out, "ERR setpoint out of range"); return; }
    if (v < T_COLD_EXTRUDE) stopManualFeed(v == 0.0 ? STOP_HEAT_OFF : STOP_TARGET_LOW);
    target = v; iTerm = 0.0;
    runawayStart = millis(); runawayRef = tempC;
    reply(out, "OK");
#endif
  }
  else if (strncmp(line, "OFF", 3) == 0) {
    stopManualFeed(STOP_HEAT_OFF);
    target = 0.0; iTerm = 0.0; reply(out, "OK");
  }
  else if (strncmp(line, "FAN", 3) == 0) {
    int v = atoi(line + 3);
    if (v < 0) v = 0; if (v > 100) v = 100;
    fanPwm = (uint8_t)(v * 255L / 100);
    analogWrite(PIN_FAN, fanPwm);
    reply(out, "OK");
  }
  else if (line[0] == 'E' && line[1] == ' ') {
    if (manualMode != MANUAL_INACTIVE) { reply(out, "ERR manual feed active"); return; }
    if (aborted)               { reply(out, "ERR aborted"); return; }
    if (!rtdSampleValid)       { reply(out, "ERR invalid temperature sample"); return; }
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
    if (manualMode != MANUAL_INACTIVE) { reply(out, "ERR manual feed active"); return; }
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
    if (manualMode != MANUAL_INACTIVE) { reply(out, "ERR manual feed active"); return; }
    float v = atof(line + 5);
    if (v > 10 && v < 5000) { stepsPerMM = v; reply(out, "OK"); }
    else reply(out, "ERR out of range");
  }
  else if (strncmp(line, "STATUS", 6) == 0) statusLine(out);
  else if (strncmp(line, "RESET", 5) == 0) {
    stopManualFeed(STOP_RESET);
    rtdSampleValid = false;
    aborted = false; abortMsg[0] = 0; iTerm = 0.0; rtdFaultRun = 0;
    reply(out, "OK");
  }
  else reply(out, "ERR unknown");
}

void readCommands(Stream &stream, char *buffer, uint8_t &length, bool &overflow) {
  // Bound input work per loop. Discard the whole line on overflow or NUL.
  for (uint8_t count = 0; count < 32 && stream.available(); count++) {
    const int input = stream.read();
    if (input < 0) return;
    const char c = (char)input;
    if (c == '\n' || c == '\r') {
      if (overflow) reply(stream, "ERR invalid or overlong command");
      else if (length) {
        buffer[length] = '\0';
        handleCommand(buffer, stream);
      }
      length = 0;
      overflow = false;
    } else if (c == '\0' || length >= sizeof(inBuf) - 1) {
      overflow = true;
    } else if (!overflow) {
      buffer[length++] = c;
    }
  }
}

void serviceEthernet() {
  // Keep partial commands between loop passes without waiting for more bytes.
  if (!ethernetClient) {
    ethernetClient = server.available();
    if (!ethernetClient) return;
    ethernetClient.setConnectionTimeout(2);
    ethernetStarted = millis();
    ethernetLen = 0;
    ethernetOverflow = false;
  }
  if (!ethernetClient.connected() || millis() - ethernetStarted >= 200) {
    ethernetClient.stop();
    return;
  }
  readCommands(ethernetClient, ethernetBuf, ethernetLen, ethernetOverflow);
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
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    attachInterrupt(digitalPinToInterrupt(PIN_GATE), onGateChange, CHANGE);
    gateOpen = (digitalRead(PIN_GATE) == LOW);
    // A gate held active at boot must release before analog feed can start.
    inhibitAnalogGate = gateOpen;
  }

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
    bool validSample = false;
    uint32_t sampledAt = 0;
    const bool recoveringFromRtdFault = rtdFaultRun != 0;

#if BENCH_NO_RTD
    // Sensor ignored completely. Hold a fake 25 C so the cold-extrude
    // interlock stays armed and only COLDJOG can turn the motor.
    rtd.clearFault();
    tempC = 25.0; tempPrev = 25.0; target = 0.0;
    rtdSampleValid = false;
    stopManualFeed(STOP_INVALID_TEMPERATURE);
#else
    float t = rtd.temperature(RNOMINAL, RREF);
    uint8_t fault = rtd.readFault();
    if (fault) {
      if (rtdFaultRun < HEATER_RTD_FAULT_LIMIT) ++rtdFaultRun;
      handleRtdFault();
      heatPwm = 0;
      analogWrite(PIN_HEATER, 0);
      rtd.clearFault();
      rtdLastFault = fault;
      if (rtdFaultCount < 65535) rtdFaultCount++;
      if (rtdFaultRun == HEATER_RTD_FAULT_LIMIT && !aborted) {
        char m[40];
        snprintf(m, sizeof(m), "RTD fault 0x%02X", fault);
        doAbort(m, STOP_RTD_FAULT);
      }
      // faulted read is not trusted: keep the last good tempC
    } else {
      rtdFaultRun = 0;
      if (!isfinite(t) || t < T_SENSOR_MIN || t > T_SENSOR_MAX) {
        rtdSampleValid = false;
        doAbort("sensor out of range", STOP_INVALID_TEMPERATURE);
      } else if (fabs(t - tempPrev) > T_JUMP_ABORT) {
        rtdSampleValid = false;
        doAbort("temperature jump", STOP_TEMPERATURE_JUMP);
      } else {
        if (t < T_COLD_EXTRUDE) stopManualFeed(STOP_TEMPERATURE_LOW);
        else if (t > T_ABORT) stopManualFeed(STOP_OVER_TEMPERATURE);
        tempPrev = t; tempC = t;
        rtdSampleValid = true;
        validSample = true;
        sampledAt = millis();
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
          if (!manualPaused) lastGoodRtd = sampledAt;
        }
      }
    }

    if (tempC > T_ABORT) doAbort("over temperature", STOP_OVER_TEMPERATURE);
#endif

    if (validSample) {
      // A bad sample cuts the output without resetting the runaway baseline.
      if (!aborted && target > 0 && lastValidHeatPwm > 200) {
        if (now - runawayStart > RUNAWAY_MS) {
          if (tempC - runawayRef < RUNAWAY_RISE) doAbort("no rise under full drive", STOP_HEATER_RUNAWAY);
          runawayStart = now; runawayRef = tempC;
        }
      } else { runawayStart = now; runawayRef = tempC; }
    }

    // Run the hard thermal checks above before re-enabling a paused driver.
    if (validSample && manualPaused) resumeManualAfterRtdSample(sampledAt);

    if (aborted || target <= 0.0) { heatPwm = 0; lastValidHeatPwm = 0; iTerm = 0.0; }
    else if (validSample) {
      float err = target - tempC;
      iTerm += Ki * err * dt;
      if (iTerm > I_CLAMP) iTerm = I_CLAMP;
      if (iTerm < 0)       iTerm = 0;
      float dTerm = recoveringFromRtdFault ? 0.0 : Kd * (err - lastErr) / dt;
      lastErr = err;
      float out = Kp * err + iTerm + dTerm;
      if (out < 0)   out = 0;
      if (out > 255) out = 255;
      heatPwm = (uint8_t)out;
      lastValidHeatPwm = heatPwm;
    }
#if BENCH_NO_RTD
    heatPwm = 0;                       // belt and braces, never drive the heater
#endif
    analogWrite(PIN_HEATER, heatPwm);
  }

  // --- extrusion gate + rate at 50 Hz ---
  if (manualMode == MANUAL_INACTIVE && now - lastRate >= 20) {
    lastRate = now;
    bool manualJog;
    noInterrupts(); manualJog = (stepsLeft > 0); interrupts();

    if (!manualJog) {
      if (aborted || !rtdSampleValid || !gateOpen || inhibitAnalogGate || tempC < T_COLD_EXTRUDE) {
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
  readCommands(Serial, inBuf, inLen, serialOverflow);

  // --- ethernet ---
  serviceEthernet();

  if (now - lastReport >= 2000) { lastReport = now; statusLine(Serial); }
}
