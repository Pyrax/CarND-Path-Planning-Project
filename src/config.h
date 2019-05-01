#ifndef PATH_PLANNING_CONFIG_H
#define PATH_PLANNING_CONFIG_H

#include "Coords.h"
#include "JMT.h"

const double TRACK_SIZE = 6945.554;
const int LANES = 3;
const double LANE_CENTERS[] {2.0, 6.0, 10.0};
const double LANE_WIDTH = 4.0;
const double MIN_LANE = 0.0;
const double MAX_LANE = 12.0;

const double TICK_RATE = 0.02;
const double PLANNING_PERIOD = 2.0;
const int NUM_POINTS = 50;
const int REUSE_POINTS = 10;

const double MAX_SPEED = 21.0;
const double MAX_ACCEL = 10.0;
const double MAX_SPEED_INC = MAX_ACCEL * TICK_RATE;
const double MAX_JERK = 10.0;
const double MS_TO_MPH = 2.24; // factor to convert from meters per second to miles per hour

const double VEL_TOLERANCE = 0.1; // small tolerance to substract from other vehicles velocity
const double LANE_CHANGE_VEL_THRESHOLD = 4.0;

const double MIN_FRONT_GAP = 20.0; // important for keeping a minimum distance when generating trajectories
const double MIN_BACK_GAP = 8.0; // needs to be considered when changing lanes for example
const double MAX_FRONT_GAP = 60.0; // vehicles which are further away are not considered to be in front
const double MAX_BACK_GAP = 10.0; // vehicles which are further away are not considered to be behind

enum BehaviorState {
  STATE_KEEP_SPEED,
  STATE_KEEP_LANE,
  STATE_CHANGE_LANE_LEFT,
  STATE_CHANGE_LANE_RIGHT,
};

struct VehicleState {
  double pos;
  double vel;
  double acc;
};

struct TrajectoryState {
  VehicleState s;
  VehicleState d;
};

struct TrajectoryJMT {
  JMT s;
  JMT d;
};

struct Trajectory {
  TrajectoryState target;
  TrajectoryJMT jmt;
};

struct FrenetPath {
  std::vector<VehicleState> s;
  std::vector<VehicleState> d;
};

struct Map {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> s;
  std::vector<double> dx;
  std::vector<double> dy;
};

struct Path {
  Coords xy_coords;
  FrenetPath frenet_coords;
};

#endif //PATH_PLANNING_CONFIG_H
