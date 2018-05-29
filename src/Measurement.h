#ifndef MEASUREMENT_H_
#define MEASUREMENT_H_

#include <iostream>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Measurement {
 protected:
  // measurement covariance matrix
  Eigen::MatrixXd R_;

 public:
  Measurement(MatrixXd R) : R_(R){};

  virtual ~Measurement() {}

  virtual const VectorXd cartesian(const VectorXd& x);

  // returns the covariance matrix
  const MatrixXd& R() { return R_; }
};

#endif