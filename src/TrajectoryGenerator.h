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
    VehicleState target_long_state = this->get_long_movement(ego, others);
    double current_d = Vehicle::lane_to_d(ego.get_lane()); // stay in current lane

    return {
        target_long_state,
        VehicleState{current_d, 0.0, 0.0}
    };
  }

  TrajectoryState lane_change_trajectory(Vehicle ego, vector<Vehicle> others, int direction) {
    VehicleState target_long_state = this->get_long_movement(ego, others);
    double target_d = Vehicle::lane_to_d(ego.get_lane() + direction);

    return {
        target_long_state,
        VehicleState{target_d, 0.0, 0.0}
    };
  }

  VehicleState get_long_movement(Vehicle ego, vector<Vehicle> others) const {
    double t = PLANNING_PERIOD;
    double t2 = t * t;
    double target_dist;
    double target_vel;
    VehicleState current_s = ego.get_state_s();

    double max_new_speed = MAX_SPEED_INC * t * NUM_POINTS / TICK_RATE + current_s.vel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (ego.get_vehicle_ahead(others, vehicle_ahead)) {
      auto ahead_vel = vehicle_ahead.get_velocity();
      auto ahead_pos = vehicle_ahead.get_position_s();

      if (ego.get_vehicle_behind(others, vehicle_behind)) {
        target_vel = ahead_vel - VEL_TOLERANCE;
      } else {
        double pos_diff = std::fabs(ahead_pos - current_s.pos);
        double vel_diff = std::fabs(ahead_vel - current_s.vel);
        double distance_to_close = pos_diff - MIN_FRONT_GAP;

        double max_velocity_in_front =
            distance_to_close / t +
            ahead_vel -
            current_s.acc * t;

        target_vel = std::min(std::min(max_velocity_in_front, max_new_speed), MAX_SPEED);
      }
    } else {
      target_vel = std::min(max_new_speed, MAX_SPEED);
    }

    const double avg_vel = (target_vel + current_s.vel) / t;
    // const double avg_acc = (target_vel - current_s.vel) / t;
    // target_dist = current_s.pos + avg_vel * t + 0.5 * avg_acc * t2;
    target_dist = current_s.pos + avg_vel * t;

    return {target_dist, target_vel, 0.0};
  }
};

#endif //PATH_PLANNING_TRAJECTORYGENERATOR_H
