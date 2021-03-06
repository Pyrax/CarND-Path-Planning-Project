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

const double MAX_SPEED = 20.5;
const double MAX_ACCEL = 10.0;
const double MAX_SPEED_INC = MAX_ACCEL * TICK_RATE;
const double MS_TO_MPH = 2.24; // factor to convert from meters per second to miles per hour

const double VEL_TOLERANCE = 0.2; // small tolerance to substract from other vehicles velocity
const double LANE_CHANGE_VEL_THRESHOLD = 2.0;

const double FRONT_FOV = 60.0;
const double FRONT_FOV_CHANGE = 100.0;
const double FRONT_BUFFER = 30.0;
const double FRONT_BUFFER_CHANGE = 15.0;
const double BACK_BUFFER = 10.0;
const double BACK_BUFFER_CHANGE = 15.0;

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
