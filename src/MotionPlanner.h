#ifndef PATH_PLANNING_MOTIONPLANNER_H
#define PATH_PLANNING_MOTIONPLANNER_H

#include <utility>
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

  Coords generate_path(const JMT &jmt_s, const JMT &jmt_d, const double t, const int points, const int current_points) const {
    std::vector<double> x_pts{};
    std::vector<double> y_pts{};

    x_pts.reserve(static_cast<unsigned long>(points));
    y_pts.reserve(static_cast<unsigned long>(points));

    for (int i = 0; i < points - current_points; ++i) {
      const double s = jmt_s.solve(i * t);
      const double d = jmt_d.solve(i * t);

      const double s_mod = fmod(s, TRACK_SIZE);
      const double x = this->spline_x(s_mod);
      const double y = this->spline_y(s_mod);
      const double dx = this->spline_dx(s_mod);
      const double dy = this->spline_dy(s_mod);

      x_pts.push_back(x + dx * d);
      y_pts.push_back(y + dy * d);
    }

    return {x_pts, y_pts};
  }

 private:
  Map map;

  tk::spline spline_x;
  tk::spline spline_y;
  tk::spline spline_dx;
  tk::spline spline_dy;
};

#endif //PATH_PLANNING_MOTIONPLANNER_H
