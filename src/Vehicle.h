#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <cmath>
#include "config.h"

class Vehicle {
 public:
  Vehicle() = default;

  Vehicle(const double s, const double d, const double v)
      : s(s), d(d), v(v), lane(Vehicle::d_to_lane(d)) {

  }

  void set_position(const double s, const double d) {
    this->s = s;
    this->d = d;
    this->lane = Vehicle::d_to_lane(d);
  }

  void set_velocity(const double v) {
    this->v = v;
  }

  void set_state_s(VehicleState state) {
    this->state_s = state;
    this->state_s.pos = fmod(state.pos, TRACK_SIZE);
  }

  void set_state_d(VehicleState state) {
    this->state_d = state;
  }

  VehicleState get_state_s() const {
    return this->state_s;
  }

  VehicleState get_state_d() const {
    return this->state_d;
  }

  int get_lane() const {
    return this->lane;
  }

  static double position_at(VehicleState current_state, double time) {
    double s = current_state.pos;
    double v = current_state.vel;
    double a = current_state.acc;
    return s + v * time + 0.5 * a * time * time;
  }

  static int d_to_lane(double d) {
    for (int i = 0; i < LANES; ++i) {
      double left_end = std::max(LANE_CENTERS[i]  - .5 * LANE_WIDTH, MIN_LANE);
      double right_end = std::min(LANE_CENTERS[i] + .5 * LANE_WIDTH, MAX_LANE);

      if (left_end <= d && d < right_end) {
        return i;
      }
    }
    return 1;
    //return static_cast<int>(std::lround((std::fmod(d, 4.0)) - 2.0));
  }

  static double lane_to_d(int lane) {
    return 2 + 4 * lane;
  }

 private:
  VehicleState state_s{}, state_d{};
  double s = 0.0, d = 0.0, v = 0.0;
  int lane = 0;
};

#endif //PATH_PLANNING_VEHICLE_H
