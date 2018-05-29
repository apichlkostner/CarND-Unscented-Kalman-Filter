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
  LidarMeasurement() : Measurement(MatrixXd(2, 2)), H_(2, 4) {
    // measurement covariance matrix for lidar
    R_ << 0.0225, 0, 0, 0.0225;

    // measurement matrix for lidar
    H_ << 1, 0, 0, 0, 0, 1, 0, 0;
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
