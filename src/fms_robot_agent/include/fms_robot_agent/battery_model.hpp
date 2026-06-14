#pragma once
#include <algorithm>

namespace fms_robot_agent {

class BatteryModel {
public:
  static constexpr double LOW_THRESHOLD  = 20.0;
  static constexpr double FULL_THRESHOLD = 95.0;

  explicit BatteryModel(double initial_soc = 100.0)
  : soc_(std::clamp(initial_soc, 0.0, 100.0)) {}

  // Call every dt seconds. is_moving and is_charging are mutually exclusive.
  void update(double dt, bool is_moving, bool is_charging) {
    if (is_charging) {
      soc_ += CHARGE_RATE * dt;
    } else if (is_moving) {
      soc_ -= DRAIN_MOVING * dt;
    } else {
      soc_ -= DRAIN_IDLE * dt;
    }
    soc_ = std::clamp(soc_, 0.0, 100.0);
  }

  double soc()     const { return soc_; }
  bool   is_low()  const { return soc_ < LOW_THRESHOLD; }
  bool   is_full() const { return soc_ >= FULL_THRESHOLD; }

  void set_soc(double v) { soc_ = std::clamp(v, 0.0, 100.0); }

private:
  static constexpr double DRAIN_IDLE   = 0.02;  // % per second while stationary
  static constexpr double DRAIN_MOVING = 0.10;  // % per second while navigating
  static constexpr double CHARGE_RATE  = 1.00;  // % per second while docked

  double soc_;
};

}  // namespace fms_robot_agent
