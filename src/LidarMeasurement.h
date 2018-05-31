#ifndef LIDARMEASUREMENT_H_
#define LIDARMEASUREMENT_H_

#include "Measurement.h"

class LidarMeasurement : public Measurement {
 protected:
  // measurement matrix
  Eigen::MatrixXd H_;

 public:
  /**
   * Constructor
   */
  LidarMeasurement(double std_laspx = 0.15, double std_laspy = 0.15)
      : Measurement(MatrixXd(2, 2)), H_(2, 5) {
    // measurement covariance matrix for lidar
    R_ << std_laspx * std_laspx, 0, 0, std_laspy * std_laspy;

    // measurement matrix for lidar
    H_.fill(0);
    H_(0, 0) = 1;
    H_(1, 1) = 1;
  }

  /**
   * Destructor
   */
  virtual ~LidarMeasurement() {}

  const VectorXd cartesian(const VectorXd& x) { return x; }

  // returns the measurement matrix
  const MatrixXd& H() { return H_; }
};

#endif
