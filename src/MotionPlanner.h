#ifndef PATH_PLANNING_MOTIONPLANNER_H
#define PATH_PLANNING_MOTIONPLANNER_H

#include <utility>
#include "config.h"
#include "JMT.h"
#include "spline.h"

class MotionPlanner {
 public:
  explicit MotionPlanner (Map map) : map(map) {
    this->spline_x.set_points(map.s, map.x);
    this->spline_y.set_points(map.s, map.y);
    this->spline_dx.set_points(map.s, map.dx);
    this->spline_dy.set_points(map.s, map.dy);
  }

  Path generate_path(const JMT &jmt_s, const JMT &jmt_d, const FrenetPath &prev_path_frenet, Coords prev_path_xy, const int prev_reuse) const {
    std::vector<double> x_pts{};
    std::vector<double> y_pts{};
    std::vector<VehicleState> path_s{};
    std::vector<VehicleState> path_d{};

    x_pts.reserve(static_cast<unsigned long>(NUM_POINTS));
    y_pts.reserve(static_cast<unsigned long>(NUM_POINTS));
    path_s.reserve(static_cast<unsigned long>(NUM_POINTS));
    path_d.reserve(static_cast<unsigned long>(NUM_POINTS));

    for (int i = 0; i < prev_reuse; ++i) {
      x_pts.push_back(prev_path_xy.get_x()[i]);
      y_pts.push_back(prev_path_xy.get_y()[i]);

      path_s.push_back(prev_path_frenet.s[i]);
      path_d.push_back(prev_path_frenet.d[i]);
    }

    for (int i = 0; i < NUM_POINTS - prev_reuse; ++i) {
      const double s      = jmt_s.solve(i * TICK_RATE);
      const double s_dot  = jmt_s.solve_dot(i * TICK_RATE);
      const double s_ddot = jmt_s.solve_ddot(i * TICK_RATE);

      const double d      = jmt_d.solve(i * TICK_RATE);
      const double d_dot  = jmt_d.solve_dot(i * TICK_RATE);
      const double d_ddot = jmt_d.solve_ddot(i * TICK_RATE);

      const double s_mod = fmod(s, TRACK_SIZE);
      const double x = this->spline_x(s_mod);
      const double y = this->spline_y(s_mod);
      const double dx = this->spline_dx(s_mod);
      const double dy = this->spline_dy(s_mod);

      path_s.push_back({ s, s_dot, s_ddot });
      path_d.push_back({ d, d_dot, d_ddot });

      x_pts.push_back(x + dx * d);
      y_pts.push_back(y + dy * d);
    }

    return {
      { x_pts, y_pts },
      { path_s, path_d }
    };
  }

 private:
  Map map;

  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;
};

#endif //PATH_PLANNING_MOTIONPLANNER_H
