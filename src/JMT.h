#ifndef PATH_PLANNING_JMT_H
#define PATH_PLANNING_JMT_H

#include <cmath>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Geometry"
#include "Eigen-3.3/Eigen/QR"

struct VehicleState;

class JMT {
 public:
  JMT() = default;

  /**
   * Calculate the Jerk Minimizing Trajectory that connects the initial state
   * to the final state in time T.
   *
   * @param start - the vehicles start location given as a length three array
   *   corresponding to initial values of [s, s_dot, s_double_dot]
   * @param end - the desired end state for vehicle. Like "start" this is a
   *   length three array.
   * @param T - The duration, in seconds, over which this maneuver should occur.
   *
   * @output an array of length 6, each value corresponding to a coefficent in
   *   the polynomial:
   *   s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
   *
   * EXAMPLE
   *   > JMT([0, 10, 0], [10, 10, 0], 1)
   *     [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
   */
  JMT(const VehicleState &start, const VehicleState &end, double T);

  double solve(double T) const;

  double solve_dot(double T) const;

  double solve_ddot(double T) const;

 private:
  Eigen::VectorXd coefficients = Eigen::VectorXd(6);
};

#endif //PATH_PLANNING_JMT_H
