#ifndef PATH_PLANNING_BEHAVIORPLANNER_H
#define PATH_PLANNING_BEHAVIORPLANNER_H

#include "Vehicle.h"
#include "TrajectoryGenerator.h"

class BehaviorPlanner {
 public:
  BehaviorPlanner(Vehicle ego, std::vector<Vehicle> others)
      : ego(ego), others(others) {

  }

  Trajectory get_best_trajectory() {
    BehaviorState s = STATE_KEEP_LANE;
    double max_new_speed = MAX_SPEED_INC * NUM_POINTS + this->ego.get_velocity();
    Vehicle vehicle_ahead;

    // Consider changing lane if a vehicle is ahead of us and driving slower.
    if (this->ego.get_vehicle_ahead(others, vehicle_ahead)) {
      auto ahead_vel = vehicle_ahead.get_velocity();
      double vel_threshold = ahead_vel + LANE_CHANGE_VEL_THRESHOLD;

      if (max_new_speed > vel_threshold) {
        const int left_lane = this->ego.get_lane() - 1;
        const int right_lane = this->ego.get_lane() + 1;
        double left_lane_speed = this->left_lane_available() ? this->get_lane_speed(left_lane) : 0.0;
        double right_lane_speed = this->right_lane_available() ? this->get_lane_speed(right_lane) : 0.0;

        if (left_lane_speed > vel_threshold || right_lane_speed > vel_threshold) {
          if (right_lane_speed > left_lane_speed) {
            s = STATE_CHANGE_LANE_RIGHT;
          } else {
            s = STATE_CHANGE_LANE_LEFT;
          }
        }
      }
    }

    TrajectoryGenerator gen{this->ego, this->others, s};
    this->next_state = s;
    return gen.get_trajectory();
  }

  BehaviorState get_next_state() const {
    return this->next_state;
  }

 private:
  Vehicle ego;

  std::vector<Vehicle> others;

  BehaviorState next_state = STATE_KEEP_LANE;

  bool left_lane_available() const {
    return this->ego.get_lane() - 1 >= 0;
  }

  bool right_lane_available() const {
    return this->ego.get_lane() + 1 < LANES;
  }

  double get_lane_speed(int lane) {
    double lane_speed = 0.0;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (!this->ego.get_vehicle_behind_for_lane(others, vehicle_behind, lane)) {
      if (this->ego.get_vehicle_ahead_for_lane(others, vehicle_ahead, lane)) {
        if (vehicle_ahead.get_position_s() > this->ego.get_position_s() + MIN_FRONT_GAP) {
          lane_speed = vehicle_ahead.get_velocity();
        }
      } else {
        lane_speed = MAX_SPEED;
      }
    }
    return lane_speed;
  }
};

#endif //PATH_PLANNING_BEHAVIORPLANNER_H
