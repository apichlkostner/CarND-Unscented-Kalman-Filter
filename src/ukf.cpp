#include "ukf.h"
#include <iostream>
#include "Eigen/Dense"
#include "RadarMeasurement.h"
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

  n_x_ = 5;

  n_aug_ = n_x_ + 2;

  lambda_ = 3 - n_x_;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  weights_ = VectorXd(2 * n_aug_ + 1);
  double weight_0 = lambda_ / (lambda_ + n_aug_);
  weights_(0) = weight_0;
  for (int i = 1; i < 2 * n_aug_ + 1; i++) {
    double weight = 0.5 / (n_aug_ + lambda_);
    weights_(i) = weight;
  }

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 1;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 1;

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
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  // if the time between two measurements is too high the state can't be
  // predicted precise enough
  constexpr double DTMAX = 0.2;
  double dt = (meas_package.timestamp_ - time_us_) / 1000000.0;

  if (!is_initialized_ || abs(dt) > DTMAX) {
    time_us_ = meas_package.timestamp_;
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      x_ << meas_package.raw_measurements_(0),
          meas_package.raw_measurements_(1), 5, 0, 0;

      MatrixXd R = lidar_measurement_.R();

      P_ << R(0), 0, 0, 0, 0, 0, R(1), 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 5, 0,
          0, 0, 0, 0, 5;
    } else {
      VectorXd rad_meas =
          RadarMeasurement::polar2cartesian(meas_package.raw_measurements_);

      x_ << rad_meas(0), rad_meas(1), rad_meas(2), 0, 0;
      MatrixXd R = radar_measurement_.R();
      P_ << R(0), 0, 0, 0, 0, 0, R(1), 0, 0, 0, 0, 0, R(2), 0, 0, 0, 0, 0, 5,
          0, 0, 0, 0, 0, 5;
    }

    is_initialized_ = true;
  } else {
    if (meas_package.timestamp_ >= time_us_) {
      time_us_ = meas_package.timestamp_;

      MatrixXd Xsig_pred = Prediction(dt);

      if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
        UpdateLidar(meas_package, dt);
      } else {
        if (!v_initialized_) {
          MatrixXd R = radar_measurement_.R();
          x_(2) = meas_package.raw_measurements_(2);
          P_(2, 2) = R(2);
          v_initialized_ = true;
        }
        UpdateRadar(meas_package, Xsig_pred, dt);
      }
    } else {
      // discard measurements with timestamps in the past
    }
  }

  cout << "x = " << x_(0) << " " << x_(1) << " " << x_(2) << " " << x_(3) << " "
       << x_(4) << " " << endl;
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
  const MatrixXd Xsig_aug = AugmentedSigmaPoints();
  const MatrixXd Xsig_pred = SigmaPointPrediction(dt, Xsig_aug);

  PredictMeanAndCovariance(Xsig_pred);

  return Xsig_pred;
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
  const MatrixXd H = lidar_measurement_.H();
  VectorXd z_pred = H * x_;
  VectorXd z = meas_package.raw_measurements_;
  VectorXd y = z - z_pred;

  MatrixXd Ht = H.transpose();  // used twice
  MatrixXd S = H * P_ * Ht + lidar_measurement_.R();
  MatrixXd PHt = P_ * Ht;

  MatrixXd K = PHt * S.inverse();

  // new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);

  P_ = (I - K * H) * P_;
}

VectorXd UKF::RadarMean(const MatrixXd& Xsig_pred, const MatrixXd& Zsig) {
  constexpr int n_z = 3;
  VectorXd z_pred = VectorXd(n_z);

  z_pred.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    z_pred = z_pred + weights_(i) * Zsig.col(i);
  }

  return z_pred;
}

MatrixXd UKF::RadarInno(const MatrixXd& Xsig_pred, const MatrixXd& Zsig,
                        const VectorXd& z_pred) {
  constexpr int n_z = 3;
  MatrixXd S = MatrixXd(n_z, n_z);
  S.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;

    // angle normalization
    z_diff(1) = Tools::NormalizeAngle(z_diff(1));

    S = S + weights_(i) * z_diff * z_diff.transpose();
  }

  return S;
}

MatrixXd UKF::RadarKalmanGain(const MatrixXd& S, const MatrixXd& Xsig_pred,
                              const MatrixXd& Zsig, const VectorXd z_pred) {
  constexpr int n_z = 3;
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  // calculate cross correlation matrix
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // 2n+1 simga points
    // cout << z_pred << " " << Zsig << endl;
    // residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    // angle normalization
    z_diff(1) = Tools::NormalizeAngle(z_diff(1));

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    // angle normalization
    x_diff(3) = Tools::NormalizeAngle(x_diff(3));

    Tc = Tc + weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  return K;
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(const MeasurementPackage& meas_package,
                      const MatrixXd& Xsig_pred, double dt) {
  // create matrix for sigma points in measurement space
  MatrixXd Zsig = RadarMeasurement::Sigma2Meas(Xsig_pred);

  // mean predicted measurement
  VectorXd z_pred = RadarMean(Xsig_pred, Zsig);

  // innovation covariance matrix S
  MatrixXd S = RadarInno(Xsig_pred, Zsig, z_pred);

  // add measurement noise covariance matrix
  S = S + radar_measurement_.R();

  MatrixXd K = RadarKalmanGain(S, Xsig_pred, Zsig, z_pred);

  // residual
  VectorXd z = meas_package.raw_measurements_;
  VectorXd z_diff = z - z_pred;

  // angle normalization
  z_diff(1) = Tools::NormalizeAngle(z_diff(1));

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();
}

void UKF::PredictMeanAndCovariance(const MatrixXd& Xsig_pred) {
  VectorXd x = VectorXd(n_x_);
  MatrixXd P = MatrixXd(n_x_, n_x_);

  // predicted state mean
  x.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    x = x + weights_(i) * Xsig_pred.col(i);
  }

  // predicted state covariance matrix
  P.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  // iterate over sigma points
    VectorXd x_diff = Xsig_pred.col(i) - x;

    // angle normalization
    x_diff(3) = Tools::NormalizeAngle(x_diff(3));

    P = P + weights_(i) * x_diff * x_diff.transpose();
  }

  P_ = P;
  x_ = x;
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

  return Xsig_aug;
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