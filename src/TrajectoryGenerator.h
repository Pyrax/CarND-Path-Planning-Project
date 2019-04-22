#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include "config.h"
#include "JMT.h"
#include "Vehicle.h"

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(Vehicle veh, BehaviorState state) {
    TrajectoryState target_state{};
    TrajectoryJMT jmt{};

    switch(state) {
      case STATE_KEEP_SPEED:
        target_state = this->constant_speed_trajectory(veh);
        break;
      case STATE_KEEP_LANE:
        target_state = this->keep_lane_trajectory(veh);
        break;
      case STATE_CHANGE_LANE_LEFT:
      case STATE_CHANGE_LANE_RIGHT:
        break;
      case STATE_PREPARE_CHANGE_LANE_LEFT:
      case STATE_PREPARE_CHANGE_LANE_RIGHT:
        break;
      case STATE_START:
      default:
        target_state = this->start_trajectory(veh);
    }

    jmt.s = JMT{veh.get_state_s(), target_state.s, PLANNING_PERIOD};
    jmt.d = JMT{veh.get_state_d(), target_state.d, PLANNING_PERIOD};
    this->trajectory = { target_state, jmt };
  }

  Trajectory get_trajectory() const {
    return this->trajectory;
  }

 private:
  Trajectory trajectory{};

  TrajectoryState constant_speed_trajectory(Vehicle veh) const {
    double target_s_pos = veh.get_state_s().pos + PLANNING_PERIOD * veh.get_state_s().vel;
    double veh_vel = veh.get_state_s().vel;

    double target_d = Vehicle::lane_to_d(veh.get_lane());

    return {
      VehicleState{target_s_pos, veh_vel, 0.0},
      VehicleState{target_d, 0.0, 0.0}
    };
  }

  TrajectoryState keep_lane_trajectory(Vehicle veh) const {
    double target_s_pos = veh.get_state_s().pos + PLANNING_PERIOD * veh.get_state_s().vel;
    double veh_vel = veh.get_state_s().vel;

    double target_d = Vehicle::lane_to_d(veh.get_lane());

    return {
        VehicleState{target_s_pos, veh_vel, 0.0},
        VehicleState{target_d, 0.0, 0.0}
    };
  }

  TrajectoryState start_trajectory(Vehicle veh) const {
    double target_s_pos = veh.get_state_s().pos + 40.0;
    double target_speed = MAX_SPEED;

    double target_d = Vehicle::lane_to_d(veh.get_lane()); // stay in current lane

    return {
      VehicleState{target_s_pos, target_speed, 0.0},
      VehicleState{target_d, 0.0, 0.0}
    };
  }
};

#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
