#ifndef PATH_PLANNING_CONFIG_H
#define PATH_PLANNING_CONFIG_H

const int LANES = 3;
const double LANE_CENTERS[] {2.0, 6.0, 10.0};
const double LANE_WIDTH = 4.0;
const double MIN_LANE = 0.0;
const double MAX_LANE = 12.0;

const double TRACK_SIZE = 6945.554;

const double TICK_RATE = 0.02;
const double PLANNING_PERIOD = 1.0;
const int NUMBER_POINTS = static_cast<int>(PLANNING_PERIOD/TICK_RATE);

const int MIN_POINTS = 2;

const double CONV = 0.224;
const double MAX_SPEED = 20.0;
const double INIT_S = 40.0;

enum BehaviorState {
  STATE_START,
  STATE_KEEP_SPEED,
  STATE_KEEP_LANE,
  STATE_CHANGE_LANE_LEFT,
  STATE_CHANGE_LANE_RIGHT,
  STATE_PREPARE_CHANGE_LANE_LEFT,
  STATE_PREPARE_CHANGE_LANE_RIGHT,
};

struct VehicleState {
  double pos;
  double vel;
  double acc;
};

struct Trajectory {
  VehicleState target_s;
  VehicleState target_d;
};

struct Map {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> dx;
  std::vector<double> dy;
};

#endif //PATH_PLANNING_CONFIG_H
