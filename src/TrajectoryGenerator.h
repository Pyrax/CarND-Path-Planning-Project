#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include "config.h"
#include "JMT.h"
#include "Vehicle.h"

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(Vehicle veh, BehaviorState state) {
    Trajectory trajectory{};

    switch(state) {
      case STATE_KEEP_SPEED:
        trajectory = this->constant_speed_trajectory(veh);
        break;
      case STATE_KEEP_LANE:
        trajectory = this->keep_lane_trajectory(veh);
        break;
      case STATE_CHANGE_LANE_LEFT:
      case STATE_CHANGE_LANE_RIGHT:
        break;
      case STATE_PREPARE_CHANGE_LANE_LEFT:
      case STATE_PREPARE_CHANGE_LANE_RIGHT:
        break;
      case STATE_START:
      default:
        trajectory = this->start_trajectory(veh);
    }

    this->trajectory = trajectory;
    this->jmt_s = JMT{veh.get_state_s(), trajectory.target_s, PLANNING_PERIOD};
    this->jmt_d = JMT{veh.get_state_d(), trajectory.target_d, PLANNING_PERIOD};
  }

  Trajectory get_trajectory() const {
    return this->trajectory;
  }

  JMT get_jmt_s() const {
    return this->jmt_s;
  }

  JMT get_jmt_d() const {
    return this->jmt_d;
  }

 private:
  Trajectory trajectory{};
  JMT jmt_s, jmt_d;

  Trajectory constant_speed_trajectory(Vehicle veh) const {
    double target_s_pos = veh.get_state_s().pos + PLANNING_PERIOD * veh.get_state_s().vel;
    double veh_vel = veh.get_state_s().vel;

    double target_d = Vehicle::lane_to_d(veh.get_lane());

    return {
      VehicleState{target_s_pos, veh_vel, 0.0},
      VehicleState{target_d, 0.0, 0.0}
    };
  }

  Trajectory keep_lane_trajectory(Vehicle veh) const {
    double target_s_pos = veh.get_state_s().pos + PLANNING_PERIOD * veh.get_state_s().vel;
    double veh_vel = veh.get_state_s().vel;

    double target_d = Vehicle::lane_to_d(veh.get_lane());

    return {
        VehicleState{target_s_pos, veh_vel, 0.0},
        VehicleState{target_d, 0.0, 0.0}
    };
  }

  Trajectory start_trajectory(Vehicle veh) const {
    double target_s_pos = veh.get_state_s().pos + 60.0;
    double target_speed = MAX_SPEED;

    double target_d = Vehicle::lane_to_d(veh.get_lane()); // stay in current lane

    return {
      VehicleState{target_s_pos, target_speed, 0.0},
      VehicleState{target_d, 0.0, 0.0}
    };
  }
};

#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
