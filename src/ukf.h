#ifndef UKF_H
#define UKF_H

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "LidarMeasurement.h"
#include "RadarMeasurement.h"
#include "measurement_package.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class UKF {
 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * ProcessMeasurement
   * @param meas_package The latest measurement data of either radar or laser
   */
  void ProcessMeasurement(MeasurementPackage meas_package);

  const VectorXd x() const { return x_; }

 private:
  ///* last timestamp in us
  long long time_us_;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a_;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd_;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx_;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy_;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr_;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi_;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd_;

  ///* Weights of sigma points
  VectorXd weights_;

  ///* State dimension
  int n_x_;

  ///* Augmented state dimension
  int n_aug_;

  ///* Sigma point spreading parameter
  double lambda_;

  ///* initially set to false, set to true in first call of ProcessMeasurement
  bool is_initialized_;

  ///* if this is false, laser measurements will be ignored (except for init)
  bool use_laser_;

  ///* if this is false, radar measurements will be ignored (except for init)
  bool use_radar_;

  ///* state vector: [pos1 pos2 vel_abs yaw_angle yaw_rate] in SI units and rad
  VectorXd x_;

  ///* state covariance matrix
  MatrixXd P_;

  ///* predicted sigma points matrix
  MatrixXd Xsig_pred_;

  RadarMeasurement radar_measurement_;

  /**
   * Prediction Predicts sigma points, the state, and the state covariance
   * matrix
   * @param delta_t Time between k and k+1 in s
   */
  MatrixXd Prediction(double dt);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateLidar(const MeasurementPackage& meas_package, double dt);

  /**
   * Updates the state and the state covariance matrix using a radar measurement
   * @param meas_package The measurement at k+1
   */
  void UpdateRadar(const MeasurementPackage& meas_package,
                   const MatrixXd& Xsig_pred, double dt);

  void PredictMeanAndCovariance(const MatrixXd& Xsig_pred);

  VectorXd RadarMean(const MatrixXd& Xsig_pred, const MatrixXd& Zsig);

  MatrixXd RadarInno(const MatrixXd& Xsig_pred, const MatrixXd& Zsig,
                     const VectorXd& z_pred);

  MatrixXd RadarKalmanGain(const MatrixXd& S, const MatrixXd& Xsig_pred,
                           const MatrixXd& Zsig, const VectorXd z_pred);

  MatrixXd SigmaPointPrediction(double dt, const MatrixXd& Xsig_aug);

  MatrixXd GenerateSigmaPoints();

  MatrixXd AugmentedSigmaPoints();

  MatrixXd Sigma2Meas(const MatrixXd& Xsig_pred);
};

#endif /* UKF_H */
