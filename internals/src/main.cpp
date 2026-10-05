#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <math.h>
#include "esp_timer.h"
#include "power_model.h"

/* ===================== BLE UUIDs ===================== */
// Birota calibration service; mirrored in the app's BleConstants.
#define CAL_SERVICE_UUID "4b3c0001-8e1f-4f9a-b6d2-6a7c2d1e0a10"
#define CAL_CONTROL_UUID "4b3c0002-8e1f-4f9a-b6d2-6a7c2d1e0a10"
#define CAL_STATUS_UUID  "4b3c0003-8e1f-4f9a-b6d2-6a7c2d1e0a10"
#define CAL_MODEL_UUID   "4b3c0004-8e1f-4f9a-b6d2-6a7c2d1e0a10"

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

static void savePrefs();

// Missing keys keep their defaults. Floats and blobs are read only when
// present: the Preferences library logs an error for every missing one.
static float prefFloat(const char* key, float fallback) {
  return prefs.isKey(key) ? prefs.getFloat(key, fallback) : fallback;
}

static void loadPrefs() {
  // Read-write so the namespace is created on first boot (read-only fails with NOT_FOUND).
  prefs.begin("bike", false);
  cal.adc_free         = prefs.getInt   ("adc_free",   cal.adc_free);
  cal.adc_engage       = prefs.getInt   ("adc_engage", cal.adc_engage);
  cal.adc_high         = prefs.getInt   ("adc_high",   cal.adc_high);
  cal.invert           = prefs.getBool  ("invert",     cal.invert);
  cal.deadzone_pct     = prefs.getUChar ("dz",         cal.deadzone_pct);
  cal.wheel_circ_mm    = prefs.getInt   ("circ_mm",    cal.wheel_circ_mm);
  cal.gear_ratio       = prefFloat("ratio", cal.gear_ratio);
  cal.speed_res_factor = prefFloat("srf",   cal.speed_res_factor);
  cal.fly.mass_kg        = prefFloat("fly_m", cal.fly.mass_kg);
  cal.fly.radius_m       = prefFloat("fly_r", cal.fly.radius_m);
  cal.fly.inertia_factor = prefFloat("fly_k", cal.fly.inertia_factor);
  cal.fly.ratio          = prefFloat("fly_g", cal.fly.ratio);
  defaultBrake(cal.brake);
  bool hasBrake = prefs.isKey("brake");
  if (hasBrake && prefs.getBytes("brake", &cal.brake, sizeof(cal.brake)) != sizeof(cal.brake)) defaultBrake(cal.brake);
  prefs.end();
  if (!hasBrake) savePrefs();  // first boot of this firmware: store the defaults
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
static const float SPINDOWN_MIN_START_RPM = 60.0f;
static const float SPINDOWN_MIN_RPM = 20.0f;  // below this, stiction: stop using data
static const int SPINDOWN_MAX = 400;
static double spindownPulses[SPINDOWN_MAX];
static int spindownN = 0;
static int spindownResistance = 0;
static double spindownRelease = -1.0;  // known release time (s), or -1 when started from serial
static uint32_t spindownStartMs = 0;

// Calibration status, notified to the app (16 bytes, fits the default MTU):
//   [0] state  [1] reason  [2] knob % at start  [3] stored point %
//   [4..7] friction Nm  [8..11] magnetic Nm*s/rad  [12..15] fit rms rpm (float32 LE)
enum CalState : uint8_t { CAL_IDLE = 0, CAL_RECORDING = 1, CAL_DONE = 2, CAL_FAILED = 3 };
enum CalReason : uint8_t {
  CAL_OK = 0, CAL_TOO_SLOW = 1, CAL_NOISY = 2, CAL_KNOB_MOVED = 3, CAL_TIMEOUT = 4, CAL_CANCELLED = 5,
};
BLECharacteristic* pCalStatusCharacteristic = nullptr;
BLECharacteristic* pCalModelCharacteristic = nullptr;

static void publishCalStatus(uint8_t state, uint8_t reason, int knob, int stored = 0,
                             float friction = 0, float magnetic = 0, float rms = 0) {
  if (!pCalStatusCharacteristic) return;
  uint8_t data[16];
  data[0] = state; data[1] = reason; data[2] = (uint8_t)knob; data[3] = (uint8_t)stored;
  memcpy(data + 4, &friction, 4);
  memcpy(data + 8, &magnetic, 4);
  memcpy(data + 12, &rms, 4);
  pCalStatusCharacteristic->setValue(data, sizeof(data));
  if (deviceConnected) pCalStatusCharacteristic->notify();
}

// Model, readable by the app: 11 x {friction f32, magnetic f32, measured u8}
// then the flywheel {mass, radius, inertia factor, ratio} as f32.
static void publishModel() {
  if (!pCalModelCharacteristic) return;
  uint8_t data[power::kLutPoints * 9 + 16];
  int o = 0;
  for (int i = 0; i < power::kLutPoints; ++i) {
    memcpy(data + o, &cal.brake.friction_nm[i], 4); o += 4;
    memcpy(data + o, &cal.brake.magnetic_nms[i], 4); o += 4;
    data[o++] = cal.brake.measured[i] ? 1 : 0;
  }
  float fly[4] = {cal.fly.mass_kg, cal.fly.radius_m, cal.fly.inertia_factor, cal.fly.ratio};
  memcpy(data + o, fly, sizeof(fly)); o += sizeof(fly);
  pCalModelCharacteristic->setValue(data, o);
}

static void failSpindown(uint8_t reason, const char* message) {
  spindownState = SpindownState::Idle;
  Serial.println(message);
  publishCalStatus(CAL_FAILED, reason, spindownResistance);
}

static void beginRecording(double releaseTime) {
  spindownState = SpindownState::Recording;
  spindownResistance = resistance_pct;
  spindownRelease = releaseTime;
  spindownN = 0;
  spindownStartMs = millis();
  publishCalStatus(CAL_RECORDING, CAL_OK, spindownResistance);
}

/** Serial flow: waits for the rider to pass SPINDOWN_ARM_RPM, then records. */
static void armSpindown() {
  spindownState = SpindownState::Armed;
  spindownStartMs = millis();
  Serial.println("\n=== Spin-down ===");
  Serial.printf("Knob at %d%%. Pedal up past %.0f rpm, then take your feet off the pedals\n", resistance_pct, SPINDOWN_ARM_RPM);
  Serial.println("and let them coast to a stop. Keep clear of the spinning cranks. Type 'cancel' to abort.");
  Serial.println("Easier: use Calibrate power in the Birota app, which counts you down.");
}

/** App flow: the rider has just taken their feet off; record from now. */
static void startSpindownNow() {
  if (powerEstimator.rpm() < SPINDOWN_MIN_START_RPM) {
    spindownResistance = resistance_pct;
    failSpindown(CAL_TOO_SLOW, "Spin-down: too slow at release.");
    return;
  }
  beginRecording(esp_timer_get_time() / 1e6);
  Serial.printf("Spin-down started from the app at %.0f rpm, knob %d%%.\n", powerEstimator.rpm(), spindownResistance);
}

static void cancelSpindown() {
  if (spindownState == SpindownState::Idle) return;
  failSpindown(CAL_CANCELLED, "Spin-down cancelled.");
}

static void finishSpindown() {
  spindownState = SpindownState::Idle;
  power::SpindownFit fit = power::fitSpindown(spindownPulses, spindownN, MAGNETS_PER_REV,
      SPINDOWN_MIN_RPM * power::kTwoPi / 60.0f, spindownRelease);
  if (!fit.ok || fit.r2 < 0.95f || fit.rms_rpm > 3.0f) {
    Serial.printf("Spin-down not usable (%d windows, r2=%.3f, rms=%.1f rpm). Spin faster before letting go and keep your feet off.\n",
        fit.points, fit.r2, fit.rms_rpm);
    publishCalStatus(CAL_FAILED, CAL_NOISY, spindownResistance, 0, 0, 0, fit.rms_rpm);
    return;
  }
  if (abs(resistance_pct - spindownResistance) > 5) {
    char msg[96];
    snprintf(msg, sizeof(msg), "The knob moved during the spin-down (%d%% -> %d%%). Try again without touching it.",
        spindownResistance, resistance_pct);
    failSpindown(CAL_KNOB_MOVED, msg);
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
  publishModel();
  publishCalStatus(CAL_DONE, CAL_OK, spindownResistance, idx * 10, friction, magnetic, fit.rms_rpm);
  float w90 = 90.0f * power::kTwoPi / 60.0f;
  Serial.printf("Knob %d%% (stored at %d%%): friction %.2f Nm + magnetic %.3f Nm*s/rad (%d windows, r2=%.3f, rms=%.1f rpm)\n",
      spindownResistance, idx * 10, friction, magnetic, fit.points, fit.r2, fit.rms_rpm);
  Serial.printf("  -> holding 90 rpm here takes %.0f W. Repeat at other knob positions.\n", (friction + magnetic * w90) * w90);
  printPowerModel();
}

static void onPulseForSpindown(double t) {
  if (spindownState == SpindownState::Armed && powerEstimator.rpm() >= SPINDOWN_ARM_RPM) {
    beginRecording(-1.0);
    Serial.println("Recording. Feet off now and let it coast.");
  }
  if (spindownState == SpindownState::Recording && spindownN < SPINDOWN_MAX) {
    spindownPulses[spindownN++] = t;
  }
}

static void updateSpindown() {
  if (spindownState == SpindownState::Idle) return;
  bool stopped = powerEstimator.rpm() < 15.0f;
  if (spindownState == SpindownState::Recording && (stopped || spindownN >= SPINDOWN_MAX)) {
    finishSpindown();
  } else if (millis() - spindownStartMs > 120000) {
    failSpindown(CAL_TIMEOUT, "Spin-down timed out.");
  }
}

// Commands written by the app, run from loop() rather than the BLE task.
static volatile uint8_t pendingCalCommand = 0;

class CalControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();
    if (!v.empty()) pendingCalCommand = (uint8_t)v[0];
  }
};

static void handleCalCommand() {
  uint8_t cmd = pendingCalCommand;
  if (!cmd) return;
  pendingCalCommand = 0;
  switch (cmd) {
    case 0x01: startSpindownNow(); break;
    case 0x02: cancelSpindown(); break;
    case 0x03:
      defaultBrake(cal.brake);
      savePrefs();
      publishModel();
      Serial.println("Brake model reset from the app.");
      break;
    default: break;
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
    double t = us / 1e6;
    powerEstimator.onPulse(t, (float)resistance_pct, cal.fly, cal.brake);
    onPulseForSpindown(t);
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
  if (cmdLine.equalsIgnoreCase("spindown")) { armSpindown(); return; }
  if (cmdLine.equalsIgnoreCase("cancel")) { cancelSpindown(); return; }
  if (cmdLine.equalsIgnoreCase("showpower")) { printPowerModel(); return; }
  if (cmdLine.equalsIgnoreCase("clearpower")) { defaultBrake(cal.brake); savePrefs(); publishModel(); Serial.println("Brake model reset to defaults."); return; }
  if (cmdLine.startsWith("setfly")) {
    float m, d, k, g;
    if (sscanf(cmdLine.c_str(), "setfly %f %f %f %f", &m, &d, &k, &g) == 4 && m > 0 && d > 0 && k > 0 && k <= 1.0f && g > 0) {
      float oldInertia = power::crankInertia(cal.fly);
      cal.fly.mass_kg = m; cal.fly.radius_m = d / 200.0f; cal.fly.inertia_factor = k; cal.fly.ratio = g;
      // Measured brake points are deceleration x inertia: keep them consistent.
      power::rescaleMeasured(cal.brake, oldInertia, power::crankInertia(cal.fly));
      savePrefs();
      publishModel();
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
      publishModel();
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

  // Birota calibration service (custom): control (write), status (notify), model (read).
  BLEService* pCalService = pServer->createService(BLEUUID(CAL_SERVICE_UUID));
  BLECharacteristic* pCalControl = pCalService->createCharacteristic(
    BLEUUID(CAL_CONTROL_UUID),
    BLECharacteristic::PROPERTY_WRITE
  );
  pCalControl->setCallbacks(new CalControlCallbacks());
  pCalStatusCharacteristic = pCalService->createCharacteristic(
    BLEUUID(CAL_STATUS_UUID),
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCalStatusCharacteristic->addDescriptor(new BLE2902());
  pCalModelCharacteristic = pCalService->createCharacteristic(
    BLEUUID(CAL_MODEL_UUID),
    BLECharacteristic::PROPERTY_READ
  );
  pCalService->start();
  publishCalStatus(CAL_IDLE, CAL_OK, resistance_pct);
  publishModel();

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

  handleCalCommand();
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
