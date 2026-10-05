#pragma once
// Physical power model for a fixed-gear flywheel bike (magnetic + felt-pad
// brake), free of Arduino dependencies so it can be unit-tested on the host.
//
// Referred to the crank, the rider's torque balances
//
//     tau = I * alpha  +  friction(r)  +  magnetic(r) * omega
//
// where omega is crank angular speed (rad/s), alpha its derivative, I the
// flywheel inertia seen at the crank (I_flywheel * ratio^2), friction(r) the
// felt pad's Coulomb torque and magnetic(r) the eddy-current brake's viscous
// coefficient, both functions of the resistance knob r (0..100 %).
// Power is tau * omega.
//
// friction and magnetic are measured per knob position with a spin-down:
// with no rider torque, I * alpha = -(friction + magnetic * omega), so a
// straight-line fit of deceleration against speed gives both, in absolute
// units, once I is known from the flywheel's mass, radius and gear ratio.

#include <math.h>
#include <stdint.h>

namespace power {

constexpr float kTwoPi = 6.28318530718f;
constexpr int kLutPoints = 11;  // resistance 0, 10, ..., 100 %

struct Flywheel {
  float mass_kg = 6.5f;
  float radius_m = 0.20f;
  // I = factor * m * r^2: 0.5 for a uniform disc, ~0.8 when the mass sits in
  // the rim and spokes.
  float inertia_factor = 0.8f;
  // Flywheel revolutions per crank revolution.
  float ratio = 6.25f;
};

/** Flywheel inertia seen at the crank, kg*m^2. */
inline float crankInertia(const Flywheel& f) {
  return f.inertia_factor * f.mass_kg * f.radius_m * f.radius_m * f.ratio * f.ratio;
}

/** Resistive crank torque per knob position: friction_nm + magnetic_nms * omega. */
struct ResistanceCurve {
  float friction_nm[kLutPoints];
  float magnetic_nms[kLutPoints];
  bool measured[kLutPoints];
};

/**
 * Friction and magnetic coefficients at resistance [pct], interpolated
 * linearly. When any point has been measured, only measured points are used
 * (nearest one beyond the measured range), so one spin-down never gets mixed
 * with uncalibrated defaults.
 */
inline void resistanceAt(const ResistanceCurve& c, float pct, float* friction, float* magnetic) {
  if (pct < 0.0f) pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  bool anyMeasured = false;
  for (int i = 0; i < kLutPoints; ++i) anyMeasured |= c.measured[i];

  int lo = -1, hi = -1;
  for (int i = 0; i < kLutPoints; ++i) {
    if (anyMeasured && !c.measured[i]) continue;
    float x = i * 10.0f;
    if (x <= pct) lo = i;
    if (x >= pct && hi < 0) hi = i;
  }
  if (lo < 0) lo = hi;
  if (hi < 0) hi = lo;
  if (lo == hi) {
    *friction = c.friction_nm[lo];
    *magnetic = c.magnetic_nms[lo];
    return;
  }
  float f = (pct - lo * 10.0f) / ((hi - lo) * 10.0f);
  *friction = c.friction_nm[lo] + f * (c.friction_nm[hi] - c.friction_nm[lo]);
  *magnetic = c.magnetic_nms[lo] + f * (c.magnetic_nms[hi] - c.magnetic_nms[lo]);
}

/**
 * Turns magnet pulses into cadence and power.
 *
 * Speed comes from full revolutions (pulse i vs pulse i - pulsesPerRev), so
 * magnets that are not exactly evenly spaced do not make it ripple. A new
 * full-revolution speed is available every pulse; acceleration is the change
 * between consecutive ones.
 */
class PowerEstimator {
 public:
  static constexpr int kMaxPulsesPerRev = 8;

  explicit PowerEstimator(int pulsesPerRev) : ppr_(pulsesPerRev < 1 ? 1 : (pulsesPerRev > kMaxPulsesPerRev ? kMaxPulsesPerRev : pulsesPerRev)) {}

  /** A debounced magnet pulse at [t] seconds (monotonic). Returns true when a new speed is available. */
  bool onPulse(double t, float resistancePct, const Flywheel& fly, const ResistanceCurve& curve) {
    if (count_ > 0 && t - times_[(head_ + kRing - 1) % kRing] > kGapSec) reset();
    times_[head_] = t;
    head_ = (head_ + 1) % kRing;
    if (count_ < kRing) ++count_;
    if (count_ < ppr_ + 1) return false;

    double tOld = times_[(head_ + kRing - 1 - ppr_) % kRing];
    double period = t - tOld;
    if (period <= 0.0) return false;
    float omega = (float)(kTwoPi / period);
    double mid = t - period / 2.0;

    if (hasSpeed_) {
      float alpha = (float)((omega - omega_) / (mid - mid_));
      alpha_ = hasAlpha_ ? alpha_ + kAlphaSmoothing * (alpha - alpha_) : alpha;
      hasAlpha_ = true;
    }
    omega_ = omega;
    mid_ = mid;
    hasSpeed_ = true;
    cadence_ = hasCadence_ ? cadence_ + kCadenceSmoothing * (rpm() - cadence_) : rpm();
    hasCadence_ = true;

    float friction, magnetic;
    resistanceAt(curve, resistancePct, &friction, &magnetic);
    float torque = crankInertia(fly) * (hasAlpha_ ? alpha_ : 0.0f) + friction + magnetic * omega_;
    float instant = torque * omega_;
    if (instant < 0.0f) instant = 0.0f;  // the flywheel driving the legs is not rider power
    power_ += kPowerSmoothing * (instant - power_);
    return true;
  }

  /** Call regularly; drops cadence and power to 0 once the crank has stopped. */
  void onTick(double now) {
    if (count_ > 0 && now - times_[(head_ + kRing - 1) % kRing] > kGapSec) reset();
  }

  /** Latest full-revolution crank speed, rad/s. */
  float omega() const { return hasSpeed_ ? omega_ : 0.0f; }
  /** Time the latest speed refers to (middle of its revolution), seconds. */
  double omegaTime() const { return mid_; }
  float rpm() const { return omega() * 60.0f / kTwoPi; }
  float cadenceRpm() const { return hasCadence_ ? cadence_ : 0.0f; }
  float powerW() const { return power_; }

  void reset() {
    count_ = 0;
    head_ = 0;
    hasSpeed_ = hasAlpha_ = hasCadence_ = false;
    omega_ = alpha_ = cadence_ = power_ = 0.0f;
  }

 private:
  static constexpr int kRing = kMaxPulsesPerRev + 1;
  // A full crank revolution slower than this (< 20 rpm) is a stop.
  static constexpr double kGapSec = 3.0;
  static constexpr float kAlphaSmoothing = 0.3f;
  static constexpr float kCadenceSmoothing = 0.5f;
  static constexpr float kPowerSmoothing = 0.35f;

  int ppr_;
  double times_[kRing] = {0};
  int head_ = 0;
  int count_ = 0;
  bool hasSpeed_ = false, hasAlpha_ = false, hasCadence_ = false;
  float omega_ = 0.0f, alpha_ = 0.0f, cadence_ = 0.0f, power_ = 0.0f;
  double mid_ = 0.0;
};

struct SpindownFit {
  bool ok = false;
  int points = 0;
  float decel_const = 0.0f;   // a in -alpha = a + b * omega, rad/s^2
  float decel_per_omega = 0.0f;  // b, 1/s
  float r2 = 0.0f;
  float omega_max = 0.0f;
  float omega_min = 0.0f;
};

/**
 * Fits -alpha = a + b * omega to a spin-down from full-revolution speed
 * samples [w] (rad/s, each the mean speed over one revolution, as from
 * PowerEstimator) at the revolutions' mid times [t] (s), taken every
 * 1/pulsesPerRev revolution. Uses the samples after the peak speed whose
 * speed is at least [minOmega].
 *
 * Integrating the motion over two revolution windows one revolution apart
 * gives, exactly up to second-order terms,
 *     -(w[i+1] - w[i-1]) / (t[i+1] - t[i-1]) = a + b * 2*pi / (t[i+1] - t[i-1])
 * so the regressor is the mean speed between the windows, not w[i]; using
 * w[i] biases the fit when only a few revolutions are available.
 */
inline SpindownFit fitSpindown(const double* t, const float* w, int n, float minOmega, int pulsesPerRev = 2) {
  SpindownFit fit;
  if (n < 5) return fit;
  int peak = 0;
  for (int i = 1; i < n; ++i) if (w[i] > w[peak]) peak = i;

  double sx = 0, sy = 0, sxx = 0, sxy = 0, syy = 0;
  int m = 0;
  float wMin = w[peak];
  const int step = pulsesPerRev;  // samples per revolution: windows one revolution apart
  for (int i = peak; i + step < n; ++i) {
    if (w[i + step] < minOmega) break;
    double dt = t[i + step] - t[i];
    if (dt <= 0) continue;
    double x = kTwoPi / dt;
    double y = -(w[i + step] - w[i]) / dt;
    sx += x; sy += y; sxx += x * x; sxy += x * y; syy += y * y;
    ++m;
    if (w[i] < wMin) wMin = w[i];
  }
  fit.points = m;
  if (m < 5) return fit;
  double varX = sxx - sx * sx / m;
  double varY = syy - sy * sy / m;
  double cov = sxy - sx * sy / m;
  if (varX <= 1e-9) return fit;
  double b = cov / varX;
  double a = (sy - b * sx) / m;
  fit.decel_per_omega = (float)b;
  fit.decel_const = (float)a;
  fit.r2 = varY > 1e-12 ? (float)(cov * cov / (varX * varY)) : 0.0f;
  fit.omega_max = w[peak];
  fit.omega_min = wMin;
  fit.ok = true;
  return fit;
}

}  // namespace power
