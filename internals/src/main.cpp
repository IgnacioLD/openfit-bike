#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <math.h>
#include "esp_timer.h"
#include "power_model.h"

/* ===================== Pins & Names ===================== */
#define DEVICE_NAME "LibrePulse Bike"
static const int MAGNET_SENSOR_PIN = 27;  // avoid strapping pins like GPIO12
static const int RESISTANCE_PIN    = 34;  // ADC1 only

/* ===================== Model Params ===================== */
// Magnet on the CRANK: cadence_rpm is CRANK RPM.
static const uint8_t  MAGNETS_PER_REV = 2;      // magnets per crank revolution
static const float    CADENCE_MIN     = 5.0f;   // rpm sanity
static const float    CADENCE_MAX     = 130.0f;
static const uint32_t DEBOUNCE_US     = 6000;   // reed/hall debounce
static const uint32_t NOTIFY_INTERVAL_MS = 100; // FTMS @10 Hz

/* ===================== Calibration & Prefs ===================== */
struct Cal {
  // 3-point capture for resistance (ADC space)
  int   adc_free   = 2600;  // slack / free spin (knob loose)
  int   adc_engage = 3000;  // where resistance starts
  int   adc_high   = 2200;  // hardest you'd still pedal
  bool  invert     = true;  // ADC decreases as resistance increases?
  uint8_t deadzone_pct = 5; // informational only

  // Motion model
  int   wheel_circ_mm  = 2148;  // 27.5" MTB ~2148 mm
  float gear_ratio     = 2.2f;  // wheel revs per crank rev (typical mid-gear)
  float speed_res_factor = 0.0f; // optional: speed drops with resistance (0..1)

  // Power model (see lib/PowerModel): flywheel geometry plus the brake's
  // friction and magnetic torque per knob position, measured by `spindown`.
  power::Flywheel fly;
  power::ResistanceCurve brake;
};

// Uncalibrated brake: a rough felt-pad friction ramp (0.5 to 25 Nm at the
// crank, soft start, hard near the top) and no magnetic term. Replaced point
// by point by spin-downs.
static void defaultBrake(power::ResistanceCurve& c) {
  static const uint8_t ramp[power::kLutPoints] = { 0, 0, 2, 6, 12, 24, 45, 65, 80, 92, 100 };
  for (int i = 0; i < power::kLutPoints; ++i) {
    c.friction_nm[i] = 0.5f + 24.5f * ramp[i] / 100.0f;
    c.magnetic_nms[i] = 0.0f;
    c.measured[i] = false;
  }
}

Preferences prefs;
Cal cal;

/* ===================== Runtime State ===================== */
// Times come from esp_timer_get_time(): 64-bit microseconds since boot, so
// they never wrap. (micros() wraps every ~71.6 min, which used to break the
// "no pulse for 3 s" check and drag cadence to 0 on long rides.)
volatile int64_t  lastPulseUs = 0;
volatile uint32_t pulseCount = 0;   // debounced magnet pulses since boot
volatile int64_t  lastRevUs = 0;    // time of the pulse that completed the latest full crank rev

// Every debounced pulse time, for the power model. The loop drains it far
// faster than pulses arrive (< 10 per second).
static const uint32_t PULSE_RING = 32;
volatile int64_t  pulseRing[PULSE_RING];

power::PowerEstimator powerEstimator(MAGNETS_PER_REV);

float cadence_rpm = 0.0f;  // CRANK RPM
float speed_kmh   = 0.0f;
float power_w     = 0.0f;
int   resistance_pct = 0;  // 0..100%

bool deviceConnected = false;

// BLE
BLEServer*         pServer = nullptr;
BLECharacteristic* pBikeDataCharacteristic = nullptr;
BLECharacteristic* pCscMeasurementCharacteristic = nullptr;

/* ===================== ISR ===================== */
void IRAM_ATTR magnetISR() {
  int64_t now = esp_timer_get_time();
  int64_t dt  = now - lastPulseUs;
  if (dt >= DEBOUNCE_US) {
    lastPulseUs = now;
    pulseRing[pulseCount % PULSE_RING] = now;
    pulseCount++;
    if (pulseCount % MAGNETS_PER_REV == 0) lastRevUs = now;
  }
}

/* ===================== Utils ===================== */
static inline float ema(float prev, float sample, float alpha) {
  return prev + alpha * (sample - prev);
}

static inline int readADCavg(uint8_t samples = 32) {
  int sum = 0;
  for (uint8_t i = 0; i < samples; ++i) sum += analogRead(RESISTANCE_PIN);
  return sum / samples;
}

// Wait for a full line (Enter). Trim CR/LF.
String readLine() {
  String s = "";
  while (true) {
    while (!Serial.available()) { delay(5); }
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') break;
    s += c;
  }
  s.trim();
  return s;
}

static void loadPrefs() {
  prefs.begin("bike", true);
  cal.adc_free         = prefs.getInt   ("adc_free",   cal.adc_free);
  cal.adc_engage       = prefs.getInt   ("adc_engage", cal.adc_engage);
  cal.adc_high         = prefs.getInt   ("adc_high",   cal.adc_high);
  cal.invert           = prefs.getBool  ("invert",     cal.invert);
  cal.deadzone_pct     = prefs.getUChar ("dz",         cal.deadzone_pct);
  cal.wheel_circ_mm    = prefs.getInt   ("circ_mm",    cal.wheel_circ_mm);
  cal.gear_ratio       = prefs.getFloat ("ratio",      cal.gear_ratio);
  cal.speed_res_factor = prefs.getFloat ("srf",        cal.speed_res_factor);
  cal.fly.mass_kg        = prefs.getFloat ("fly_m",      cal.fly.mass_kg);
  cal.fly.radius_m       = prefs.getFloat ("fly_r",      cal.fly.radius_m);
  cal.fly.inertia_factor = prefs.getFloat ("fly_k",      cal.fly.inertia_factor);
  cal.fly.ratio          = prefs.getFloat ("fly_g",      cal.fly.ratio);
  defaultBrake(cal.brake);
  if (prefs.getBytes("brake", &cal.brake, sizeof(cal.brake)) != sizeof(cal.brake)) defaultBrake(cal.brake);
  prefs.end();
}

static void savePrefs() {
  prefs.begin("bike", false);
  prefs.putInt   ("adc_free",   cal.adc_free);
  prefs.putInt   ("adc_engage", cal.adc_engage);
  prefs.putInt   ("adc_high",   cal.adc_high);
  prefs.putBool  ("invert",     cal.invert);
  prefs.putUChar ("dz",         cal.deadzone_pct);
  prefs.putInt   ("circ_mm",    cal.wheel_circ_mm);
  prefs.putFloat ("ratio",      cal.gear_ratio);
  prefs.putFloat ("srf",        cal.speed_res_factor);
  prefs.putFloat ("fly_m",      cal.fly.mass_kg);
  prefs.putFloat ("fly_r",      cal.fly.radius_m);
  prefs.putFloat ("fly_k",      cal.fly.inertia_factor);
  prefs.putFloat ("fly_g",      cal.fly.ratio);
  prefs.putBytes ("brake",      &cal.brake, sizeof(cal.brake));
  prefs.end();
}

static void printPowerModel() {
  Serial.printf("[FLY] mass=%.2f kg | diameter=%.1f cm | inertia factor=%.2f | ratio=%.2f -> I at crank=%.3f kg*m^2\n",
      cal.fly.mass_kg, cal.fly.radius_m * 200.0f, cal.fly.inertia_factor, cal.fly.ratio, power::crankInertia(cal.fly));
  Serial.println("[BRAKE] res%  friction Nm  magnetic Nm*s/rad  (crank torque = friction + magnetic * omega)");
  for (int i = 0; i < power::kLutPoints; ++i) {
    Serial.printf("        %3d%%  %10.2f  %17.3f  %s\n", i * 10, cal.brake.friction_nm[i], cal.brake.magnetic_nms[i],
        cal.brake.measured[i] ? "measured" : "default");
  }
}

static void printCalibration() {
  Serial.printf("[CAL] free=%d, engage=%d, high=%d | invert=%s | deadzone=%u%%\n",
      cal.adc_free, cal.adc_engage, cal.adc_high,
      cal.invert ? "true":"false", cal.deadzone_pct);
  Serial.printf("[CFG] wheel=%d mm | ratio=%.2f | speed_res_factor=%.2f\n",
      cal.wheel_circ_mm, cal.gear_ratio, cal.speed_res_factor);
  printPowerModel();
}

/* Map raw ADC -> 0..100% (anchor 0% at ENGAGE, 100% at HIGH; auto-invert) */
static float mapADCtoPercent(int raw) {
  int lo = min(cal.adc_engage, cal.adc_high);
  int hi = max(cal.adc_engage, cal.adc_high);
  if (hi == lo) hi = lo + 1;

  float x = (float)(raw - lo) / (float)(hi - lo);
  if (x < 0.0f) x = 0.0f;
  if (x > 1.0f) x = 1.0f;

  // If ADC decreases as resistance increases, ENGAGE > HIGH → invert
  bool needInvert = (cal.adc_high < cal.adc_engage);
  if (needInvert) x = 1.0f - x;

  return x * 100.0f; // percent
}

/* ===================== Cadence / Resistance / Speed / Power ===================== */
void updateResistance() {
  static bool  adcInit = false;
  static float resEMA  = 0.0f;

  if (!adcInit) {
    analogReadResolution(12);                           // 0..4095
    analogSetPinAttenuation(RESISTANCE_PIN, ADC_11db);  // wider range
    adcInit = true;
  }
  int raw = readADCavg(16);
  float pct = mapADCtoPercent(raw);
  resEMA = ema(resEMA, pct, 0.2f);
  resistance_pct = (int)(resEMA + 0.5f);
}

static inline float speedPerCadence_kmh() {
  // Optional coupling: ratio_eff = ratio * (1 - srf * r_norm)  (clamped ≥ 0.2)
  float r_norm = resistance_pct / 100.0f;
  float ratio_eff = cal.gear_ratio * (1.0f - cal.speed_res_factor * r_norm);
  if (ratio_eff < 0.2f) ratio_eff = 0.2f;
  return ratio_eff * (cal.wheel_circ_mm * 60.0f / 1e6f);
}

/* ===================== Spin-down calibration ===================== */
// Measures the brake at the current knob position: spin up past
// SPINDOWN_ARM_RPM, take your feet off and let the fixed-gear flywheel coast
// to a stop. The deceleration curve gives friction and magnetic torque.
enum class SpindownState { Idle, Armed, Recording };
static SpindownState spindownState = SpindownState::Idle;
static const float SPINDOWN_ARM_RPM = 80.0f;
static const int SPINDOWN_MAX = 240;
static double spindownT[SPINDOWN_MAX];
static float spindownW[SPINDOWN_MAX];
static int spindownN = 0;
static int spindownResistance = 0;
static uint32_t spindownStartMs = 0;

static void startSpindown() {
  spindownState = SpindownState::Armed;
  spindownN = 0;
  spindownStartMs = millis();
  Serial.println("\n=== Spin-down ===");
  Serial.printf("Knob at %d%%. Pedal up past %.0f rpm, then take your feet off the pedals\n", resistance_pct, SPINDOWN_ARM_RPM);
  Serial.println("and let them coast to a stop. Keep clear of the spinning cranks. Type 'cancel' to abort.");
}

static void finishSpindown() {
  spindownState = SpindownState::Idle;
  // Skip the first revolution after the peak: the rider may still be easing off.
  int peak = 0;
  for (int i = 1; i < spindownN; ++i) if (spindownW[i] > spindownW[peak]) peak = i;
  int from = peak + MAGNETS_PER_REV;
  power::SpindownFit fit = from < spindownN
      ? power::fitSpindown(spindownT + from, spindownW + from, spindownN - from, 20.0f * power::kTwoPi / 60.0f, MAGNETS_PER_REV)
      : power::SpindownFit{};
  if (!fit.ok || fit.r2 < 0.9f) {
    Serial.printf("Spin-down not usable (%d points, r2=%.3f). Spin faster before letting go and keep your feet off.\n",
        fit.points, fit.r2);
    return;
  }
  if (abs(resistance_pct - spindownResistance) > 5) {
    Serial.printf("The knob moved during the spin-down (%d%% -> %d%%). Try again without touching it.\n",
        spindownResistance, resistance_pct);
    return;
  }
  float inertia = power::crankInertia(cal.fly);
  float friction = fmaxf(0.0f, inertia * fit.decel_const);
  float magnetic = fmaxf(0.0f, inertia * fit.decel_per_omega);
  int idx = (spindownResistance + 5) / 10;
  if (idx > power::kLutPoints - 1) idx = power::kLutPoints - 1;
  cal.brake.friction_nm[idx] = friction;
  cal.brake.magnetic_nms[idx] = magnetic;
  cal.brake.measured[idx] = true;
  savePrefs();
  float w90 = 90.0f * power::kTwoPi / 60.0f;
  Serial.printf("Knob %d%% (stored at %d%%): friction %.2f Nm + magnetic %.3f Nm*s/rad (%d points, r2=%.3f)\n",
      spindownResistance, idx * 10, friction, magnetic, fit.points, fit.r2);
  Serial.printf("  -> holding 90 rpm here takes %.0f W. Repeat at other knob positions.\n", (friction + magnetic * w90) * w90);
  printPowerModel();
}

static void onSpeedSample() {
  if (spindownState == SpindownState::Armed && powerEstimator.rpm() >= SPINDOWN_ARM_RPM) {
    spindownState = SpindownState::Recording;
    spindownResistance = resistance_pct;
    spindownN = 0;
    Serial.println("Recording. Feet off now and let it coast.");
  }
  if (spindownState == SpindownState::Recording && spindownN < SPINDOWN_MAX) {
    spindownT[spindownN] = powerEstimator.omegaTime();
    spindownW[spindownN] = powerEstimator.omega();
    ++spindownN;
  }
}

static void updateSpindown() {
  if (spindownState == SpindownState::Idle) return;
  bool stopped = powerEstimator.rpm() < 15.0f;
  if (spindownState == SpindownState::Recording && (stopped || spindownN >= SPINDOWN_MAX)) {
    finishSpindown();
  } else if (millis() - spindownStartMs > 120000) {
    spindownState = SpindownState::Idle;
    Serial.println("Spin-down timed out.");
  }
}

/* Feed every new magnet pulse to the power model. */
void updatePower() {
  static uint32_t consumed = 0;
  noInterrupts();
  uint32_t produced = pulseCount;
  interrupts();
  if (produced - consumed > PULSE_RING) consumed = produced - PULSE_RING;  // never expected; drop the oldest
  while (consumed != produced) {
    noInterrupts();
    int64_t us = pulseRing[consumed % PULSE_RING];
    interrupts();
    ++consumed;
    if (powerEstimator.onPulse(us / 1e6, (float)resistance_pct, cal.fly, cal.brake)) onSpeedSample();
  }
  powerEstimator.onTick(esp_timer_get_time() / 1e6);
  float rpm = powerEstimator.cadenceRpm();
  cadence_rpm = (rpm >= CADENCE_MIN && rpm <= CADENCE_MAX) ? rpm : 0.0f;
  power_w = cadence_rpm > 0.0f ? powerEstimator.powerW() : 0.0f;
}

void updateMetrics() {
  updateResistance();
  updatePower();
  updateSpindown();
  speed_kmh = cadence_rpm * speedPerCadence_kmh();
}

/* ===================== BLE: FTMS Indoor Bike Data ===================== */
void sendBLEData() {
  static uint32_t lastNotifyMs = 0;
  uint32_t nowMs = millis();
  if (!deviceConnected) return;
  if ((nowMs - lastNotifyMs) < NOTIFY_INTERVAL_MS) return;
  lastNotifyMs = nowMs;

  // Include: Instantaneous Speed, Instantaneous Cadence, Resistance Level, Instantaneous Power
  const uint16_t flags = 0x0064; // bits 2(cadence),5(resistance),6(power); MoreData=0 => include speed

  const uint16_t speedValue    = (uint16_t)(speed_kmh   * 100.0f + 0.5f); // 0.01 km/h
  const uint16_t cadenceValue  = (uint16_t)(cadence_rpm * 2.0f   + 0.5f); // 0.5 rpm
  const int16_t  resistLevel   = (int16_t)resistance_pct;                 // SINT16
  const int16_t  powerValue    = (int16_t)(power_w + 0.5f);               // SINT16 W

  uint8_t data[10];
  data[0] = flags & 0xFF;           data[1] = (flags >> 8) & 0xFF;
  data[2] = speedValue & 0xFF;      data[3] = (speedValue >> 8) & 0xFF;
  data[4] = cadenceValue & 0xFF;    data[5] = (cadenceValue >> 8) & 0xFF;
  data[6] = resistLevel & 0xFF;     data[7] = (resistLevel >> 8) & 0xFF;
  data[8] = powerValue & 0xFF;      data[9] = (powerValue >> 8) & 0xFF;

  pBikeDataCharacteristic->setValue(data, sizeof(data));
  pBikeDataCharacteristic->notify();
}

/* ===================== BLE: CSC Measurement ===================== */
// Cycling Speed and Cadence (0x1816) crank data: cumulative crank revolutions
// and the time of the last full revolution in 1/1024 s. Unlike the FTMS
// cadence (EMA-smoothed per half turn), this is the raw revolution timing,
// so clients can derive exact per-revolution cadence. Notified as soon as a
// revolution completes, plus a 1 s heartbeat so clients see the crank stop.
static const uint32_t CSC_HEARTBEAT_MS = 1000;

void sendCSCData() {
  static uint32_t lastSentRevs = UINT32_MAX;
  static uint32_t lastNotifyMs = 0;
  if (!deviceConnected) return;

  noInterrupts();
  uint32_t revs  = pulseCount / MAGNETS_PER_REV;
  int64_t  revUs = lastRevUs;
  interrupts();

  uint32_t nowMs = millis();
  if (revs == lastSentRevs && (nowMs - lastNotifyMs) < CSC_HEARTBEAT_MS) return;
  lastSentRevs = revs;
  lastNotifyMs = nowMs;

  const uint16_t cumRevs   = (uint16_t)(revs & 0xFFFF);
  const uint16_t eventTime = (uint16_t)(((uint64_t)revUs * 1024ULL / 1000000ULL) & 0xFFFF);

  uint8_t data[5];
  data[0] = 0x02;  // flags: crank revolution data present
  data[1] = cumRevs & 0xFF;   data[2] = (cumRevs >> 8) & 0xFF;
  data[3] = eventTime & 0xFF; data[4] = (eventTime >> 8) & 0xFF;

  pCscMeasurementCharacteristic->setValue(data, sizeof(data));
  pCscMeasurementCharacteristic->notify();
}

/* ===================== BLE callbacks ===================== */
class ServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    deviceConnected = true;
    Serial.println("Client Connected");
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    Serial.println("Client Disconnected");
    BLEDevice::startAdvertising();
  }
};

/* ===================== Calibration Flow ===================== */
void runCalibration() {
  Serial.println("\n=== Resistance Calibration (3-point) ===");
  Serial.println("1) FREE: knob fully loose (slack). Press ENTER to capture...");
  (void)readLine(); delay(200);
  int freeVal = readADCavg(64);
  Serial.printf("FREE  = %d\n", freeVal);

  Serial.println("2) ENGAGE: turn knob until resistance JUST STARTS. Press ENTER...");
  (void)readLine(); delay(200);
  int engageVal = readADCavg(64);
  Serial.printf("ENGAGE= %d\n", engageVal);

  Serial.println("3) HIGH: hardest you'd still pedal. Press ENTER...");
  (void)readLine(); delay(200);
  int highVal = readADCavg(64);
  Serial.printf("HIGH  = %d\n", highVal);

  cal.adc_free   = freeVal;
  cal.adc_engage = engageVal;
  cal.adc_high   = highVal;

  // Derived settings
  cal.invert = (cal.adc_high < cal.adc_engage);
  int span_total = abs(cal.adc_high - cal.adc_free);
  int span_gap   = abs(cal.adc_engage - cal.adc_free);
  float dz = (span_total > 0) ? (float)span_gap / (float)span_total : 0.0f;
  if (dz < 0.0f) dz = 0.0f;
  if (dz > 0.6f) dz = 0.6f;
  cal.deadzone_pct = (uint8_t)roundf(dz * 100.0f);

  savePrefs();
  Serial.println("Saved calibration:");
  printCalibration();
  Serial.println("=== Done ===\n");
}

/* ===================== Commands ===================== */
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  cal                 - run 3-point resistance calibration (FREE, ENGAGE, HIGH)");
  Serial.println("  show                - print current calibration/params and LUT");
  Serial.println("  raw                 - stream raw ADC -> % (Ctrl+C to stop monitor)");
  Serial.println("  setcirc <mm>        - set wheel circumference in mm (1500..3000), e.g. setcirc 2148");
  Serial.println("  setratio <x>        - set base gear ratio (wheel revs per crank rev), e.g. setratio 2.2");
  Serial.println("  setspeedres <f>     - set resistance→ratio factor 0..1 (0=off). e.g. setspeedres 0.3");
  Serial.println("  spindown            - measure the brake at the current knob position (see prompts)");
  Serial.println("  cancel              - abort a spin-down");
  Serial.println("  showpower           - print flywheel and brake model");
  Serial.println("  setfly <kg> <diam_cm> <factor> <ratio> - flywheel mass, diameter, inertia factor (0.5 disc .. 1 rim), flywheel revs per crank rev");
  Serial.println("  setbrake <pct> <friction_Nm> <magnetic_Nms> - set one brake point by hand");
  Serial.println("  clearpower          - forget all spin-down measurements");
  Serial.println("  help                - show this help");
}

void handleCommand(const String& cmdLine) {
  if (!cmdLine.length()) return;

  if (cmdLine.equalsIgnoreCase("cal")) { runCalibration(); return; }
  if (cmdLine.equalsIgnoreCase("show")) { printCalibration(); return; }
  if (cmdLine.equalsIgnoreCase("raw")) {
    Serial.println("Raw ADC stream (Ctrl+C to stop monitor):");
    for (int i=0;i<2000;i++) {
      int raw = readADCavg(8);
      float pct = mapADCtoPercent(raw);
      Serial.printf("raw=%d -> %5.1f%%\n", raw, pct);
      delay(50);
      if (Serial.available()) break;
    }
    return;
  }
  if (cmdLine.startsWith("setcirc")) {
    int sp = cmdLine.indexOf(' ');
    if (sp > 0) {
      int mm = cmdLine.substring(sp+1).toInt();
      if (mm >= 1500 && mm <= 3000) { cal.wheel_circ_mm = mm; savePrefs(); Serial.printf("Wheel circumference set to %d mm\n", cal.wheel_circ_mm); }
      else Serial.println("Enter 1500..3000 mm.");
    } else { Serial.printf("Current circumference: %d mm\nUsage: setcirc 2148\n", cal.wheel_circ_mm); }
    return;
  }
  if (cmdLine.startsWith("setratio")) {
    int sp = cmdLine.indexOf(' ');
    if (sp > 0) {
      float r = cmdLine.substring(sp+1).toFloat();
      if (r > 0.3f && r < 5.0f) { cal.gear_ratio = r; savePrefs(); Serial.printf("Base gear ratio set to %.2f\n", cal.gear_ratio); }
      else Serial.println("Enter 0.3..5.0 (typical ~2.2)");
    } else { Serial.printf("Current base ratio: %.2f\nUsage: setratio 2.2\n", cal.gear_ratio); }
    return;
  }
  if (cmdLine.startsWith("setspeedres")) {
    int sp = cmdLine.indexOf(' ');
    if (sp > 0) {
      float f = cmdLine.substring(sp+1).toFloat();
      if (f < 0.0f) f = 0.0f; if (f > 1.0f) f = 1.0f;
      cal.speed_res_factor = f; savePrefs();
      Serial.printf("Resistance→ratio factor set to %.2f\n", cal.speed_res_factor);
    } else { Serial.printf("Current speed_res_factor: %.2f\nUsage: setspeedres 0.3\n", cal.speed_res_factor); }
    return;
  }
  if (cmdLine.equalsIgnoreCase("spindown")) { startSpindown(); return; }
  if (cmdLine.equalsIgnoreCase("cancel")) { spindownState = SpindownState::Idle; Serial.println("Spin-down cancelled."); return; }
  if (cmdLine.equalsIgnoreCase("showpower")) { printPowerModel(); return; }
  if (cmdLine.equalsIgnoreCase("clearpower")) { defaultBrake(cal.brake); savePrefs(); Serial.println("Brake model reset to defaults."); return; }
  if (cmdLine.startsWith("setfly")) {
    float m, d, k, g;
    if (sscanf(cmdLine.c_str(), "setfly %f %f %f %f", &m, &d, &k, &g) == 4 && m > 0 && d > 0 && k > 0 && k <= 1.0f && g > 0) {
      cal.fly.mass_kg = m; cal.fly.radius_m = d / 200.0f; cal.fly.inertia_factor = k; cal.fly.ratio = g;
      savePrefs();
      printPowerModel();
    } else {
      Serial.println("Usage: setfly <kg> <diameter_cm> <factor 0.5..1> <ratio>, e.g. setfly 6.5 40 0.8 6.25");
    }
    return;
  }
  if (cmdLine.startsWith("setbrake")) {
    int pct; float f, mg;
    if (sscanf(cmdLine.c_str(), "setbrake %d %f %f", &pct, &f, &mg) == 3 && pct >= 0 && pct <= 100 && f >= 0 && mg >= 0) {
      int idx = (pct + 5) / 10;
      cal.brake.friction_nm[idx] = f; cal.brake.magnetic_nms[idx] = mg; cal.brake.measured[idx] = true;
      savePrefs();
      printPowerModel();
    } else {
      Serial.println("Usage: setbrake <pct 0..100> <friction_Nm> <magnetic_Nms>");
    }
    return;
  }
  if (cmdLine.equalsIgnoreCase("help")) { printHelp(); return; }

  Serial.println("Unknown command. Type 'help'.");
}

/* ===================== Setup / Loop ===================== */
void setup() {
  Serial.begin(115200);

  pinMode(MAGNET_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MAGNET_SENSOR_PIN), magnetISR, FALLING);

  pinMode(RESISTANCE_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(RESISTANCE_PIN, ADC_11db);

  loadPrefs();
  printCalibration();
  Serial.println("Type 'help' for commands.");

  // BLE init
  BLEDevice::init(DEVICE_NAME);
  BLEDevice::setMTU(185);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // FTMS (0x1826) with Indoor Bike Data (0x2AD2)
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x1826));
  pBikeDataCharacteristic = pService->createCharacteristic(
    BLEUUID((uint16_t)0x2AD2),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pBikeDataCharacteristic->addDescriptor(new BLE2902()); // CCCD
  pService->start();

  // CSC (0x1816): CSC Measurement (0x2A5B, notify) + CSC Feature (0x2A5C, read)
  BLEService* pCscService = pServer->createService(BLEUUID((uint16_t)0x1816));
  pCscMeasurementCharacteristic = pCscService->createCharacteristic(
    BLEUUID((uint16_t)0x2A5B),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCscMeasurementCharacteristic->addDescriptor(new BLE2902()); // CCCD
  BLECharacteristic* pCscFeature = pCscService->createCharacteristic(
    BLEUUID((uint16_t)0x2A5C),
    BLECharacteristic::PROPERTY_READ
  );
  uint8_t cscFeature[2] = { 0x02, 0x00 }; // crank revolution data supported
  pCscFeature->setValue(cscFeature, sizeof(cscFeature));
  pCscService->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID((uint16_t)0x1826);
  adv->addServiceUUID((uint16_t)0x1816);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("Bike Ready!");
}

void loop() {
  // Serial command handler (line-based)
  if (Serial.available()) {
    String cmd = readLine();
    handleCommand(cmd);
  }

  updateMetrics();
  sendBLEData();
  sendCSCData();

  // Human-readable log (2 Hz)
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 500) {
    lastPrint = now;
    int raw = analogRead(RESISTANCE_PIN);
    float r_norm = resistance_pct / 100.0f;
    float ratio_eff = cal.gear_ratio * (1.0f - cal.speed_res_factor * r_norm);
    if (ratio_eff < 0.2f) ratio_eff = 0.2f;
    Serial.printf("Cad: %.1f RPM | Spd: %.2f km/h | Pwr: %.1f W | Res: %d%% | raw=%d | ratio_eff=%.2f\n",
                  cadence_rpm, speed_kmh, power_w, resistance_pct, raw, ratio_eff);
  }
}
