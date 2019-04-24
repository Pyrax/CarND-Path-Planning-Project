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
      case STATE_CHANGE_LANE_LEFT:
      case STATE_CHANGE_LANE_RIGHT:
        next_states.push_back(STATE_KEEP_LANE);
        next_states.push_back(STATE_CHANGE_LANE_LEFT);
        next_states.push_back(STATE_CHANGE_LANE_RIGHT);
        break;
      case STATE_KEEP_LANE:
      case STATE_KEEP_SPEED:
      default:
        next_states.push_back(STATE_KEEP_LANE);
        next_states.push_back(STATE_KEEP_SPEED);
        next_states.push_back(STATE_CHANGE_LANE_LEFT);
        next_states.push_back(STATE_CHANGE_LANE_RIGHT);
    }
    return next_states;
  }
};

#endif //PATH_PLANNING_BEHAVIORPLANNER_H
