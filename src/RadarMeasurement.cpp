#include "RadarMeasurement.h"

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