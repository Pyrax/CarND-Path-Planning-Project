#ifndef PATH_PLANNING_JMT_H
#define PATH_PLANNING_JMT_H

#include <cmath>
#include <iostream>
#include <vector>
#include "config.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Geometry"
#include "Eigen-3.3/Eigen/QR"

class JMT {
 public:
  JMT() = default;

  JMT(const VehicleState &start, const VehicleState &end, const double T) {
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
    Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
    Eigen::MatrixXd B = Eigen::MatrixXd(3, 1);

    const double T2 = T * T;
    const double T3 = T * T2;
    const double T4 = T * T3;
    const double T5 = T * T4;

    A << T3, T4, T5,
        3 * T2, 4 * T3, 5 * T4,
        6 * T, 12 * T2, 20 * T3;

    B << end.pos - (start.pos + start.vel * T + 0.5 * start.acc * T2),
        end.vel - (start.vel + start.acc * T),
        end.acc - start.acc;

    Eigen::MatrixXd C = A.inverse() * B;

    this->coefficients << start.pos, start.vel, start.acc * 0.5, C(0), C(1), C(2);
  }

  double solve(const double T) const {
    double val = 0.0;
    for (int i = 0; i < this->coefficients.size(); ++i) {
      val += this->coefficients[i] * pow(T, i);
    }
    return val;
  }

  double solve_dot(const double T) const {
    double val = 0.0;
    for (int i = 1; i < this->coefficients.size(); ++i) {
      val += i * this->coefficients[i] * pow(T, i - 1);
    }
    return val;
  }

  double solve_ddot(const double T) const {
    double val = 0.0;
    for (int i = 2; i < this->coefficients.size(); ++i) {
      val += (i - 1) * i *this->coefficients[i] * pow(T, i - 2);
    }
    return val;
  }

 private:
  Eigen::VectorXd coefficients = Eigen::VectorXd(6);
};

#endif //PATH_PLANNING_JMT_H
