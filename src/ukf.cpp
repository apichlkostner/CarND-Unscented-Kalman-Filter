#include "ukf.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;

  // DO NOT MODIFY measurement noise values below these are provided by the
  // sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  // DO NOT MODIFY measurement noise values above these are provided by the
  // sensor manufacturer.

  /**
TODO:

Complete the initialization. See ukf.h for other member properties.

Hint: one or more values initialized above might be wildly off...
*/
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if (!is_initialized_) {
    time_us_ = meas_package.timestamp_;
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_(0),
          meas_package.raw_measurements_(1), 0, 0, 0;
    } else {
      VectorXd rad_meas =
          RadarMeasurement::polar2cartesian(meas_package.raw_measurements_);
      double v = sqrt(rad_meas(0) * rad_meas(0) + rad_meas(1) * rad_meas(1));

      x_ << rad_meas(0), rad_meas(1), v, 0, 0;
    }

    is_initialized_ = true;
  } else {
    if (meas_package.timestamp_ >= time_us_) {
      double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
      time_us_ = meas_package.timestamp_;

      MatrixXd Xsig_pred = Prediction(dt);

      if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package, dt);
      } else {
        UpdateRadar(meas_package, Xsig_pred, dt);
      }
    } else {
      // discard measurements with timestamps in the past
    }
  }
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} dt the change in time (in seconds) between the last
 * measurement and this one.
 */
MatrixXd UKF::Prediction(double dt) {
  /**
TODO:

Complete this function! Estimate the object's location. Modify the state
vector, x_. Predict sigma points, the state, and the state covariance matrix.
*/
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(const MeasurementPackage& meas_package, double dt) {
  /**
TODO:

Complete this function! Use lidar data to update the belief about the object's
position. Modify the state vector, x_, and covariance, P_.

You'll also need to calculate the lidar NIS.
*/
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package,
                      const MatrixXd& Xsig_pred, double dt) {
  constexpr int n_z = 3;
  // set vector for weights
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights(i) = weight;
  }

  // create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);

  /*******************************************************************************
   * Student part begin
   ******************************************************************************/

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points

    // extract values for better readibility
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

  // mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights(i) * Zsig.col(i);
  }

  // innovation covariance matrix S
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    z_diff(1) = Tools::NormalizeAngle(z_diff(1));

    S = S + weights(i) * z_diff * z_diff.transpose();
  }

  // add measurement noise covariance matrix
  MatrixXd R = MatrixXd(n_z, n_z);
  R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0,
      std_radrd_ * std_radrd_;
  S = S + R;

  // write result
  // *z_out = z_pred;
  // *S_out = S;
}

void UKF::PredictMeanAndCovariance(VectorXd& x_out, MatrixXd& P_out,
                                   const MatrixXd& Xsig_pred) {
  VectorXd weights = VectorXd(2 * n_aug_ + 1);
  VectorXd x = VectorXd(n_x_);
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // set weights
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights(i) = weight;
  }

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    x = x + weights(i) * Xsig_pred.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    VectorXd x_diff = Xsig_pred.col(i) - x;

    // angle normalization
    x_diff(3) = Tools::NormalizeAngle(x_diff(3));

    P = P + weights(i) * x_diff * x_diff.transpose();
  }

  P_out = P;
  x_out = x;
}

MatrixXd UKF::SigmaPointPrediction(double dt, const MatrixXd& Xsig_aug) {
  MatrixXd Xsig_pred = MatrixXd(n_x_, 2 * n_aug_ + 1);

  // predict sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    // extract values for better readability
    double p_x = Xsig_aug(0, i);
    double p_y = Xsig_aug(1, i);
    double v = Xsig_aug(2, i);
    double yaw = Xsig_aug(3, i);
    double yawd = Xsig_aug(4, i);
    double nu_a = Xsig_aug(5, i);
    double nu_yawdd = Xsig_aug(6, i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
      px_p = p_x + v / yawd * (sin(yaw + yawd * dt) - sin(yaw));
      py_p = p_y + v / yawd * (cos(yaw) - cos(yaw + yawd * dt));
    } else {
      px_p = p_x + v * dt * cos(yaw);
      py_p = p_y + v * dt * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * dt;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * dt * dt * cos(yaw);
    py_p = py_p + 0.5 * nu_a * dt * dt * sin(yaw);
    v_p = v_p + nu_a * dt;

    yaw_p = yaw_p + 0.5 * nu_yawdd * dt * dt;
    yawd_p = yawd_p + nu_yawdd * dt;

    // write predicted sigma point into right column
    Xsig_pred(0, i) = px_p;
    Xsig_pred(1, i) = py_p;
    Xsig_pred(2, i) = v_p;
    Xsig_pred(3, i) = yaw_p;
    Xsig_pred(4, i) = yawd_p;
  }

  return Xsig_pred;
}

MatrixXd UKF::AugmentedSigmaPoints() {
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  // create augmented mean state
  x_aug.head(n_x_) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;

  // create augmented covariance matrix
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(n_x_, n_x_) = std_a_ * std_a_;
  P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

  // create square root matrix
  MatrixXd L = P_aug.llt().matrixL();

  double factor = sqrt(lambda_ + n_aug_);

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
    Xsig_aug.col(i + 1) = x_aug + factor * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - factor * L.col(i);
  }

  return P_aug;
}

MatrixXd UKF::GenerateSigmaPoints() {
  MatrixXd Xsig = MatrixXd(n_x_, 2 * n_x_ + 1);
  MatrixXd A = P_.llt().matrixL();

  // set first column of sigma point matrix
  Xsig.col(0) = x_;

  double factor = sqrt(lambda_ + n_x_);

  // set remaining sigma points
  for (int i = 0; i < n_x_; i++) {
    Xsig.col(i + 1) = x_ + factor * A.col(i);
    Xsig.col(i + 1 + n_x_) = x_ - factor * A.col(i);
  }

  return Xsig;
}