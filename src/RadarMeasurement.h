#ifndef RADARMEASUREMENT_H_
#define RADARMEASUREMENT_H_

#include "Measurement.h"

class RadarMeasurement : public Measurement {
 public:
  /**
   * Constructor
   */
  RadarMeasurement(double std_radr = 0.3, double std_radrd = 0.3,
                   double std_radphi = 0.03)
      : Measurement(MatrixXd(3, 3)) {
    // radar covariance matrix
    R_ << std_radr * std_radr, 0, 0, 0, std_radphi * std_radphi, 0, 0, 0,
        std_radrd * std_radrd;
  }

  /**
   * Destructor
   */
  virtual ~RadarMeasurement() {}

  const VectorXd cartesian(const VectorXd& x) { return polar2cartesian(x); }

  static MatrixXd Sigma2Meas(const MatrixXd& Xsig_pred);

  static VectorXd polar2cartesian(const VectorXd& meas) {
    VectorXd cartesian(4);

    float rho = meas(0);
    float phi = meas(1);
    float rho_dot = meas(2);

    float sin_phi = sin(phi);
    float cos_phi = cos(phi);

    cartesian << rho * cos_phi, rho * sin_phi, rho_dot * cos_phi,
        rho_dot * sin_phi;

    return cartesian;
  }

  static VectorXd cartesian2polar(const VectorXd& x_state) {
    VectorXd polar(3);

    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    float pxy2 = sqrt(px * px + py * py);

    // near the origin the polar coordinates are assumed to be 0
    if (pxy2 < 0.000001) {
      polar << 0, 0, 0;
    } else {
      polar << pxy2, atan2(py, px), (px * vx + py * vy) / pxy2;
    }

    return polar;
  }

 private:
};

#endif
