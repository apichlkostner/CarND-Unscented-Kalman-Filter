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
  float r2c1 = px * (vy * px - vx * py) / p_xy32;

  Hj << px / p_xy2s, py / p_xy2s, 0.0, 0.0, -py / p_xy2, px / p_xy2, 0.0, 0.0,
      r2c0, r2c0, px / p_xy2s, py / p_xy2s;

  return Hj;
}