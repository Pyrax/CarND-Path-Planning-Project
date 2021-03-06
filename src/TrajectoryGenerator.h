#ifndef PATH_PLANNING_TRAJECTORYGENERATOR_H
#define PATH_PLANNING_TRAJECTORYGENERATOR_H

#include "config.h"
#include "JMT.h"
#include "Vehicle.h"

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(Vehicle ego, vector<Vehicle> others, BehaviorState state) {
    TrajectoryState target_state{};
    TrajectoryJMT jmt{};

    const int direction = (state == STATE_CHANGE_LANE_LEFT) ? -1 : 1; // only use if state is a lane change

    switch(state) {
      case STATE_KEEP_LANE:
        target_state = this->keep_lane_trajectory(ego, others);
        break;
      case STATE_CHANGE_LANE_LEFT:
      case STATE_CHANGE_LANE_RIGHT:
        target_state = this->lane_change_trajectory(ego, others, direction);
        break;
      case STATE_KEEP_SPEED:
      default:
        target_state = this->constant_speed_trajectory(ego);
    }

    jmt.s = JMT{ego.get_state_s(), target_state.s, PLANNING_PERIOD};
    jmt.d = JMT{ego.get_state_d(), target_state.d, PLANNING_PERIOD};
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

  TrajectoryState keep_lane_trajectory(Vehicle ego, vector<Vehicle> others) const {
    double current_d = Vehicle::lane_to_d(ego.get_lane()); // stay in current lane
    VehicleState target_long_state = this->get_long_movement(ego, others, ego.get_lane());

    return {
        target_long_state,
        VehicleState{current_d, 0.0, 0.0}
    };
  }

  TrajectoryState lane_change_trajectory(Vehicle ego, vector<Vehicle> others, int direction) {
    double target_d = Vehicle::lane_to_d(ego.get_lane() + direction);
    VehicleState target_long_state = this->get_long_movement(ego, others, ego.get_lane() + direction);

    return {
        target_long_state,
        VehicleState{target_d, 0.0, 0.0}
    };
  }

  VehicleState get_long_movement(Vehicle ego, vector<Vehicle> others, int target_lane) const {
    double t = PLANNING_PERIOD;
    double t2 = t * t;
    double target_dist;
    double target_vel;
    VehicleState current_s = ego.get_state_s();

    double max_new_speed = MAX_SPEED_INC * NUM_POINTS + current_s.vel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (ego.get_vehicle_ahead_for_lane(others, vehicle_ahead, target_lane, FRONT_FOV)) {
      auto ahead_vel = vehicle_ahead.get_velocity();
      auto ahead_pos = vehicle_ahead.get_position_s();

      if (ego.get_vehicle_behind_for_lane(others, vehicle_behind, target_lane, BACK_BUFFER)) {
        // if vehicle behind comes to close we just keep the speed of the vehicle ahead
        target_vel = ahead_vel - VEL_TOLERANCE;
      } else {
        // otherwise we try to get as close to the vehicle ahead as possible
        double pos_diff = std::fabs(ahead_pos - current_s.pos);
        double vel_diff = std::fabs(ahead_vel - current_s.vel);
        double distance_to_close = pos_diff - FRONT_BUFFER;

        double max_velocity_in_front =
            distance_to_close / t +
            ahead_vel -
            current_s.acc * t;

        target_vel = std::min(std::min(max_velocity_in_front - VEL_TOLERANCE, max_new_speed), MAX_SPEED);
      }
    } else {
      target_vel = std::min(max_new_speed, MAX_SPEED);
    }

    const double avg_vel = std::min((target_vel + current_s.vel) / t, MAX_SPEED);
    target_dist = current_s.pos + avg_vel * t;

    return {target_dist, target_vel, 0.0};
  }
};

#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
