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

    /*for (auto &state : get_next_possible_states()) {

    }*/

    BehaviorState s = STATE_KEEP_LANE;

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

  std::vector<BehaviorState> get_next_possible_states() const {
    std::vector<BehaviorState> next_states{};
    switch (this->ego.get_behavior()) {
      case STATE_KEEP_LANE:
      case STATE_KEEP_SPEED:
        next_states.push_back(STATE_KEEP_SPEED);
      case STATE_CHANGE_LANE_LEFT:
      case STATE_CHANGE_LANE_RIGHT:
        next_states.push_back(STATE_KEEP_LANE);
        // As a lane change does not complete in one time step we need to
        // push lane change as next step again.
        if (this->can_change_lane_left()) {
          next_states.push_back(STATE_CHANGE_LANE_LEFT);
        }
        if (this->can_change_lane_right()) {
          next_states.push_back(STATE_CHANGE_LANE_RIGHT);
        }
      default:
        break;
    }
    return next_states;
  }

  bool can_change_lane_left() const {
    return this->ego.get_lane() - 1 >= 0;
  }

  bool can_change_lane_right() const {
    return this->ego.get_lane() + 1 < LANES;
  }
};

#endif //PATH_PLANNING_BEHAVIORPLANNER_H
