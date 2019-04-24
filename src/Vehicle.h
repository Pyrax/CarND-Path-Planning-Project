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

  double get_position_s() const {
    return this->s;
  }

  void set_velocity(const double v) {
    this->v = v;
  }

  double get_velocity() const {
    return this->v;
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

  void set_behavior(BehaviorState behavior) {
    this->behavior = behavior;
  }

  BehaviorState get_behavior() const {
    return this->behavior;
  }

  int get_lane() const {
    return this->lane;
  }

  bool get_vehicle_ahead(vector<Vehicle> vehicles, Vehicle &vehicle_ahead) {
    bool found_vehicle = false;
    double min_s = 10000000.0; // to get nearest vehicle

    double own_pos = this->get_position_s();
    for (auto &vehicle : vehicles) {
      double pos = vehicle.get_position_s();
      bool in_same_lane = vehicle.get_lane() == this->get_lane();
      bool is_ahead = pos > own_pos;
      bool nearest = pos < min_s;
      bool in_range = std::fabs(pos - own_pos) <= MAX_FRONT_GAP;

      if (in_same_lane && is_ahead && nearest && in_range) {
        min_s = pos;
        vehicle_ahead = vehicle;
        found_vehicle = true;
      }
    }

    return found_vehicle;
  }

  bool get_vehicle_behind(vector<Vehicle> vehicles, Vehicle &vehicle_behind) {
    bool found_vehicle = false;
    double max_s = -1.0; // to get nearest vehicle

    double own_pos = this->get_position_s();
    for (auto &vehicle : vehicles) {
      double pos = vehicle.get_position_s();
      bool in_same_lane = vehicle.get_lane() == this->get_lane();
      bool is_behind = pos < own_pos;
      bool nearest = pos > max_s;
      bool in_range = std::fabs(pos - own_pos) <= MAX_BACK_GAP;

      if (in_same_lane && is_behind && nearest && in_range) {
        max_s = pos;
        vehicle_behind = vehicle;
        found_vehicle = true;
      }
    }

    return found_vehicle;
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
  }

  static double lane_to_d(int lane) {
    return 2 + 4 * lane;
  }

 private:
  VehicleState state_s{}, state_d{};
  BehaviorState behavior = STATE_START;
  double s = 0.0, d = 0.0, v = 0.0;
  int lane = 0;
};

#endif //PATH_PLANNING_VEHICLE_H
