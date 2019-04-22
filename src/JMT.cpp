#include "JMT.h"
#include "config.h"

JMT::JMT(const VehicleState &start, const VehicleState &end, const double T) {
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

double JMT::solve(const double T) const {
  double val = 0.0;
  for (int i = 0; i < this->coefficients.size(); ++i) {
    val += this->coefficients[i] * pow(T, i);
  }
  return val;
}

double JMT::solve_dot(const double T) const {
  double val = 0.0;
  for (int i = 1; i < this->coefficients.size(); ++i) {
    val += i * this->coefficients[i] * pow(T, i - 1);
  }
  return val;
}

double JMT::solve_ddot(const double T) const {
  double val = 0.0;
  for (int i = 2; i < this->coefficients.size(); ++i) {
    val += (i - 1) * i *this->coefficients[i] * pow(T, i - 2);
  }
  return val;
}
