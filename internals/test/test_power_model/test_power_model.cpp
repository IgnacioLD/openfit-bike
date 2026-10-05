#include <unity.h>
#include <math.h>
#include <vector>
#include "power_model.h"

using namespace power;

static ResistanceCurve flatCurve(float friction, float magnetic) {
  ResistanceCurve c{};
  for (int i = 0; i < kLutPoints; ++i) {
    c.friction_nm[i] = friction;
    c.magnetic_nms[i] = magnetic;
    c.measured[i] = false;
  }
  return c;
}

/**
 * Simulates the bike: integrates crank angle under rider torque [riderTorque(t)]
 * against inertia and the brake, and emits a pulse whenever a magnet passes.
 * Magnets sit at [magnetAngles] (radians within one revolution).
 */
struct Sim {
  Flywheel fly;
  ResistanceCurve curve;
  std::vector<double> pulses;
  std::vector<double> omegaAt;  // true crank speed at each pulse

  template <typename Torque>
  void run(double seconds, double omega0, Torque riderTorque, std::vector<double> magnetAngles) {
    const double dt = 1e-4;
    double I = crankInertia(fly);
    double angle = 0, omega = omega0;
    for (double t = 0; t < seconds; t += dt) {
      double f, m;
      float ff, mm;
      resistanceAt(curve, 50.0f, &ff, &mm);
      f = ff; m = mm;
      double brake = omega > 0 ? f + m * omega : 0;
      double alpha = (riderTorque(t, omega) - brake) / I;
      double next = angle + omega * dt;
      omega += alpha * dt;
      if (omega < 0) omega = 0;
      for (double a : magnetAngles) {
        double base = floor(angle / (2 * M_PI)) * 2 * M_PI;
        for (double k = base; k <= next + 2 * M_PI; k += 2 * M_PI) {
          double p = k + a;
          if (p > angle && p <= next) {
            pulses.push_back(t + (p - angle) / ((next - angle) / dt));
            omegaAt.push_back(omega);
          }
        }
      }
      angle = next;
    }
  }
};

void test_crank_inertia_for_the_librepulse_flywheel() {
  Flywheel f;  // 6.5 kg, 0.20 m radius, rim-weighted 0.8, ratio 6.25
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 8.125f, crankInertia(f));
}

void test_steady_cadence_power_is_brake_torque_times_speed() {
  Sim sim;
  sim.curve = flatCurve(12.0f, 0.8f);
  // Rider torque exactly balancing the brake at 90 rpm keeps speed constant.
  double w = 90 * 2 * M_PI / 60;
  sim.run(20, w, [&](double, double om) { return 12.0 + 0.8 * om; }, {0, M_PI});
  PowerEstimator est(2);
  for (double t : sim.pulses) est.onPulse(t, 50, sim.fly, sim.curve);
  double expected = (12.0 + 0.8 * w) * w;
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 90.0f, est.cadenceRpm());
  TEST_ASSERT_FLOAT_WITHIN(expected * 0.02f, (float)expected, est.powerW());
}

void test_uneven_magnets_do_not_ripple_cadence_or_power() {
  Sim sim;
  sim.curve = flatCurve(12.0f, 0.8f);
  double w = 80 * 2 * M_PI / 60;
  // Second magnet at 170 degrees instead of 180.
  sim.run(20, w, [&](double, double om) { return 12.0 + 0.8 * om; }, {0, 170 * M_PI / 180});
  PowerEstimator est(2);
  float minP = 1e9, maxP = -1;
  for (size_t i = 0; i < sim.pulses.size(); ++i) {
    est.onPulse(sim.pulses[i], 50, sim.fly, sim.curve);
    if (sim.pulses[i] > 10) {
      minP = fminf(minP, est.powerW());
      maxP = fmaxf(maxP, est.powerW());
    }
  }
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 80.0f, est.cadenceRpm());
  TEST_ASSERT_TRUE(maxP - minP < 3.0f);
}

void test_acceleration_adds_the_flywheel_inertia() {
  Sim sim;
  sim.curve = flatCurve(10.0f, 0.5f);
  // 15 Nm above the brake: alpha = 15 / 8.125 rad/s^2.
  sim.run(6, 6.0, [&](double, double om) { return 10.0 + 0.5 * om + 15.0; }, {0, M_PI});
  PowerEstimator est(2);
  for (double t : sim.pulses) est.onPulse(t, 50, sim.fly, sim.curve);
  // The estimate averages the last revolution and is smoothed over about one
  // more, so while power keeps rising it trails the instantaneous value by a
  // few percent.
  double om = est.omega();
  double expected = (10.0 + 0.5 * om + 15.0) * om;
  TEST_ASSERT_FLOAT_WITHIN(expected * 0.06f, (float)expected, est.powerW());
  // Without the inertia term the estimate would miss the 15 Nm of acceleration torque.
  TEST_ASSERT_TRUE(est.powerW() > (10.0 + 0.5 * om) * om * 1.3);
}

void test_spindown_fit_recovers_friction_and_magnetic_brake() {
  Sim sim;
  sim.curve = flatCurve(9.0f, 1.2f);
  sim.run(25, 110 * 2 * M_PI / 60, [](double, double) { return 0.0; }, {0, M_PI});
  PowerEstimator est(2);
  std::vector<double> t;
  std::vector<float> w;
  for (double p : sim.pulses) {
    if (est.onPulse(p, 50, sim.fly, sim.curve)) {
      t.push_back(est.omegaTime());
      w.push_back(est.omega());
    }
  }
  SpindownFit fit = fitSpindown(t.data(), w.data(), (int)t.size(), 20 * 2 * M_PI / 60);
  TEST_ASSERT_TRUE(fit.ok);
  float I = crankInertia(sim.fly);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 9.0f, fit.decel_const * I);
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.2f, fit.decel_per_omega * I);
  TEST_ASSERT_TRUE(fit.r2 > 0.98f);
}

void test_resistance_interpolates_only_between_measured_points() {
  ResistanceCurve c = flatCurve(99.0f, 9.0f);  // uncalibrated defaults
  c.friction_nm[2] = 4.0f;  c.magnetic_nms[2] = 0.2f;  c.measured[2] = true;   // 20 %
  c.friction_nm[6] = 20.0f; c.magnetic_nms[6] = 1.0f;  c.measured[6] = true;   // 60 %
  float f, m;
  resistanceAt(c, 40.0f, &f, &m);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 12.0f, f);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 0.6f, m);
  resistanceAt(c, 90.0f, &f, &m);  // beyond the measured range: nearest measured
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 20.0f, f);
  resistanceAt(c, 0.0f, &f, &m);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 4.0f, f);
}

void test_stopping_resets_to_zero() {
  Flywheel fly;
  ResistanceCurve c = flatCurve(10, 0.5);
  PowerEstimator est(2);
  for (int i = 0; i < 20; ++i) est.onPulse(i * 0.375, 50, fly, c);
  TEST_ASSERT_TRUE(est.cadenceRpm() > 70);
  est.onTick(7.125 + 3.5);
  TEST_ASSERT_EQUAL_FLOAT(0.0f, est.cadenceRpm());
  TEST_ASSERT_EQUAL_FLOAT(0.0f, est.powerW());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crank_inertia_for_the_librepulse_flywheel);
  RUN_TEST(test_steady_cadence_power_is_brake_torque_times_speed);
  RUN_TEST(test_uneven_magnets_do_not_ripple_cadence_or_power);
  RUN_TEST(test_acceleration_adds_the_flywheel_inertia);
  RUN_TEST(test_spindown_fit_recovers_friction_and_magnetic_brake);
  RUN_TEST(test_resistance_interpolates_only_between_measured_points);
  RUN_TEST(test_stopping_resets_to_zero);
  return UNITY_END();
}
