#include "RadarMeasurement.h"

MatrixXd RadarMeasurement::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3, 4);
  // recover state parameters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float p_xy2 = (px * px) + (py * py);
  float p_xy2s = sqrt(p_xy2);
  float p_xy32 = p_xy2 * p_xy2s;

  // near origin the polar coordinates are assumed to be 0
  if (p_xy2 < 0.0000001) {
    Hj << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    return Hj;
  }

  float r2c0 = py * (vx * py - vy * px) / p_xy32;

  Hj << px / p_xy2s, py / p_xy2s, 0.0, 0.0, -py / p_xy2, px / p_xy2, 0.0, 0.0,
      r2c0, r2c0, px / p_xy2s, py / p_xy2s;

  return Hj;
}

MatrixXd RadarMeasurement::Sigma2Meas(const MatrixXd& Xsig_pred) {
  constexpr int n_z = 3;
  int n_aug = Xsig_pred.rows() + 2;
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug + 1);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug + 1; i++) {
    double p_x = Xsig_pred(0, i);
    double p_y = Xsig_pred(1, i);
    double v = Xsig_pred(2, i);
    double yaw = Xsig_pred(3, i);

    double v1 = cos(yaw) * v;
    double v2 = sin(yaw) * v;

    // measurement model
    Zsig(0, i) = sqrt(p_x * p_x + p_y * p_y);                          // r
    Zsig(1, i) = atan2(p_y, p_x);                                      // phi
    Zsig(2, i) = (p_x * v1 + p_y * v2) / sqrt(p_x * p_x + p_y * p_y);  // r_dot
  }

  return Zsig;
}