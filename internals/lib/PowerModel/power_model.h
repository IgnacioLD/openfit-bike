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
    ++pulses_;
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
  /** Crank angle travelled since the last reset at that time, radians (from the pulse count). */
  double omegaAngle() const { return (pulses_ - ppr_ / 2.0) * kTwoPi / ppr_; }
  float rpm() const { return omega() * 60.0f / kTwoPi; }
  float cadenceRpm() const { return hasCadence_ ? cadence_ : 0.0f; }
  float powerW() const { return power_; }

  void reset() {
    count_ = 0;
    head_ = 0;
    pulses_ = 0;
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
  long pulses_ = 0;
  bool hasSpeed_ = false, hasAlpha_ = false, hasCadence_ = false;
  float omega_ = 0.0f, alpha_ = 0.0f, cadence_ = 0.0f, power_ = 0.0f;
  double mid_ = 0.0;
};

struct SpindownFit {
  bool ok = false;
  int points = 0;
  float decel_const = 0.0f;      // a in -alpha = a + b * omega, rad/s^2
  float decel_per_omega = 0.0f;  // b, 1/s
  float r2 = 0.0f;               // of the speed fit
  float rms_rpm = 0.0f;          // residual of the speed fit
  float omega_max = 0.0f;
  float omega_min = 0.0f;
};

/**
 * Fits a spin-down straight from the magnet pulse times [pulses] (seconds,
 * consecutive, no rider torque from the release on).
 *
 * With no rider torque -alpha = a + b * omega. Integrated, the speed is
 * omega(t) = c - a * t - b * theta(t), linear in (c, a, b) and free of
 * differentiation, so it stays stable with the few revolutions a heavily
 * braked flywheel coasts. Each window between consecutive pulses gives its
 * mean speed (window angle / duration) at its mid time and its time-averaged
 * angle; for a decelerating crank the latter sits alpha * T^2 / 12 past the
 * window's middle angle, a correction refined over a few iterations.
 *
 * With two magnets, every half revolution is a window, and the magnets'
 * deviation from 180 degrees is fitted as one more unknown (it alternates
 * the window angle by +-eps), which doubles the data compared with full
 * revolutions. Only windows after the peak speed and after [releaseTime]
 * (when known, in the pulses' clock), while faster than [minOmega], are used: below that, stiction makes friction non-constant.
 */
inline SpindownFit fitSpindown(const double* pulses, int n, int pulsesPerRev, float minOmega, double releaseTime = -1.0) {
  SpindownFit fit;
  const bool halfWindows = pulsesPerRev == 2;
  const int span = halfWindows ? 1 : pulsesPerRev;    // pulses per window
  const double angle = kTwoPi * span / pulsesPerRev;  // nominal window angle
  const int windows = n - span;
  if (windows < 6) return fit;

  constexpr int kMaxWindows = 512;
  static double tm[kMaxWindows], period[kMaxWindows], th[kMaxWindows], speed[kMaxWindows];
  static int sign[kMaxWindows];
  int count = windows < kMaxWindows ? windows : kMaxWindows;
  for (int k = 0; k < count; ++k) {
    period[k] = pulses[k + span] - pulses[k];
    tm[k] = (pulses[k + span] + pulses[k]) / 2.0;
    th[k] = k * kTwoPi / pulsesPerRev + angle / 2.0;
    speed[k] = period[k] > 0 ? angle / period[k] : 0.0;
    sign[k] = halfWindows ? ((k % 2 == 0) ? 1 : -1) : 0;
  }
  // Peak of the speed (pairs averaged so the magnet offset does not pick it).
  int peak = 0;
  double best = -1;
  for (int k = 0; k + 1 < count; ++k) {
    double v = (speed[k] + speed[k + 1]) / 2.0;
    if (v > best) { best = v; peak = k; }
  }
  // Rider torque must be gone from every window used. With a known release
  // time, start at the first window that begins after it; otherwise skip the
  // window holding the peak, which may still carry some.
  int start = peak + 1;
  if (releaseTime >= 0) {
    start = peak;
    while (start < count && pulses[start] < releaseTime) ++start;
  }
  int end = start;
  while (end < count && speed[end] >= minOmega) ++end;
  int m = end - start;
  fit.points = m;
  if (m < 6) return fit;
  peak = start;

  const int P = halfWindows ? 4 : 3;  // c, -a, -b [, -eps]
  double coef[4] = {0, 0, 0, 0};
  double a = 0, b = 0;
  for (int iter = 0; iter < 4; ++iter) {
    double S[4][5] = {{0}};
    for (int k = peak; k < end; ++k) {
      double alpha = a + b * speed[k];
      double thBar = th[k] + alpha * period[k] * period[k] / 12.0;
      double x[4] = {1.0, tm[k] - tm[peak], thBar - th[peak], -sign[k] / period[k]};
      for (int r = 0; r < P; ++r) {
        for (int c = 0; c < P; ++c) S[r][c] += x[r] * x[c];
        S[r][P] += x[r] * speed[k];
      }
    }
    // Gaussian elimination with partial pivoting.
    for (int col = 0; col < P; ++col) {
      int piv = col;
      for (int r = col + 1; r < P; ++r) if (fabs(S[r][col]) > fabs(S[piv][col])) piv = r;
      if (fabs(S[piv][col]) < 1e-12) return fit;
      for (int c = 0; c <= P; ++c) { double tmp = S[col][c]; S[col][c] = S[piv][c]; S[piv][c] = tmp; }
      for (int r = 0; r < P; ++r) {
        if (r == col) continue;
        double f = S[r][col] / S[col][col];
        for (int c = col; c <= P; ++c) S[r][c] -= f * S[col][c];
      }
    }
    for (int r = 0; r < P; ++r) coef[r] = S[r][P] / S[r][r];
    a = -coef[1];
    b = -coef[2];
  }

  double mean = 0;
  for (int k = peak; k < end; ++k) mean += speed[k];
  mean /= m;
  double ssRes = 0, ssTot = 0, wMin = speed[peak], wMax = speed[peak];
  for (int k = peak; k < end; ++k) {
    double alpha = a + b * speed[k];
    double thBar = th[k] + alpha * period[k] * period[k] / 12.0;
    double pred = coef[0] + coef[1] * (tm[k] - tm[peak]) + coef[2] * (thBar - th[peak]) - coef[3] * sign[k] / period[k];
    ssRes += (speed[k] - pred) * (speed[k] - pred);
    ssTot += (speed[k] - mean) * (speed[k] - mean);
    if (speed[k] < wMin) wMin = speed[k];
    if (speed[k] > wMax) wMax = speed[k];
  }
  fit.decel_const = (float)a;
  fit.decel_per_omega = (float)b;
  fit.r2 = ssTot > 1e-12 ? (float)(1.0 - ssRes / ssTot) : 0.0f;
  fit.rms_rpm = (float)(sqrt(ssRes / m) * 60.0 / kTwoPi);
  fit.omega_max = (float)wMax;
  fit.omega_min = (float)wMin;
  fit.ok = true;
  return fit;
}

}  // namespace power
