#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include <fstream>
#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
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
	std_a_ = 0.6;

	// Process noise standard deviation yaw acceleration in rad/s^2
	std_yawdd_ = 0.5;

	/**
	 * DO NOT MODIFY measurement noise values below.
	 * These are provided by the sensor manufacturer.
	 */

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

	/**
	 * End DO NOT MODIFY section for measurement noise values
	 */

	 /**
	  * TODO: Complete the initialization. See ukf.h for other member properties.
	  * Hint: one or more values initialized above might be wildly off...
	  */
	is_initialized_ = false;
	n_x_ = 5;               // set state dimension
	n_aug_ = 7;             // set augmented dimension
	lambda_ = 3 - n_aug_;   // set lambda_

	// set weights
	weights_ = VectorXd(2 * n_aug_ + 1);
	weights_.fill(0.5 / (lambda_ + n_aug_));
	weights_(0) = lambda_ / (lambda_ + n_aug_);

	// Initialize sigma point matrix
	Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
	/**
	 * TODO: Complete this function! Make sure you switch between lidar and radar
	 * measurements.
	 */
	if (!use_laser_ && meas_package.sensor_type_ == MeasurementPackage::LASER) {
		return;
	}
	if (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		return;
	}

	if (!is_initialized_) {
		// initializing time stamp
		time_us_ = meas_package.timestamp_;

		// first measurement
		x_ << 0, 0, 0, 0, 0;
		if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
			x_(0) = meas_package.raw_measurements_[0];
			x_(1) = meas_package.raw_measurements_[1];
		}
		else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
			double rho = meas_package.raw_measurements_[0];
			double phi = meas_package.raw_measurements_[1];
			x_(0) = rho * cos(phi);
			x_(1) = rho * sin(phi);
		}

		// initializing state covariance matrix P_
		P_ << 1, 0, 0, 0, 0,
			0, 1, 0, 0, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0,
			0, 0, 0, 0, 1;

		// done initializing, no need to predict or update
		is_initialized_ = true;
		return;
	}

	double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
	time_us_ = meas_package.timestamp_;

	// Predict
	Prediction(delta_t);

	// Update
	if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
		UpdateLidar(meas_package);
		std::ofstream myfileI ("lidar_output.txt", std::ios::app);
		if (myfileI.is_open()) {
        myfileI << NIS_lidar_ << "\n";
				myfileI.close();
    }
    else std::cout << "Unable to open file for writing";
	}
	else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
		UpdateRadar(meas_package);
		std::ofstream myfileI ("radar_output.txt", std::ios::app);
		if (myfileI.is_open()) {
        myfileI << NIS_radar_ << "\n";
				myfileI.close();
    }
    else std::cout << "Unable to open file for writing";
	}

	std::cout << "x_ = " << x_ << std::endl;
	std::cout << "P_ = " << P_ << std::endl;

}

void UKF::Prediction(double delta_t) {
	/**
	 * TODO: Complete this function! Estimate the object's location.
	 * Modify the state vector, x_. Predict sigma points, the state,
	 * and the state covariance matrix.
	 */

	 // create augmented mean vector
	VectorXd x_aug = VectorXd(7);
	// create augmented state covariance
	MatrixXd P_aug = MatrixXd(7, 7);
	// create sigma point matrix
	MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

	// create augmented mean state
	// The mean of the noise values is zero
	x_aug.setZero();
	x_aug.head(n_x_) = x_;

	// create augmented covariance matrix
	P_aug.setZero();
	P_aug.topLeftCorner(n_x_, n_x_) = P_;
	P_aug(n_x_, n_x_) = std_a_ * std_a_;
	P_aug(n_x_ + 1, n_x_ + 1) = std_yawdd_ * std_yawdd_;

	// create square root matrix
	MatrixXd A = P_aug.llt().matrixL();

	// create augmented sigma points
	Xsig_aug.col(0) = x_aug;
	for (int i = 0; i < n_aug_; ++i) {
		Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A.col(i);
		Xsig_aug.col(n_aug_ + i + 1) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i);
	}

	// Predict sigma points
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		double px = Xsig_aug(0, i);
		double py = Xsig_aug(1, i);
		double v = Xsig_aug(2, i);
		double yaw = Xsig_aug(3, i);
		double yawdot = Xsig_aug(4, i);
		double nu_a = Xsig_aug(5, i);
		double nu_yawdd = Xsig_aug(6, i);
		if (fabs(yawdot) > 0.0001) {
			Xsig_pred_(0, i) = px + v / yawdot * (sin(yaw + yawdot * delta_t) - sin(yaw)) + 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
			Xsig_pred_(1, i) = py + v / yawdot * (-cos(yaw + yawdot * delta_t) + cos(yaw)) + 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
		}
		else {
			Xsig_pred_(0, i) = px + v * cos(yaw) * delta_t + 0.5 * delta_t * delta_t * cos(yaw) * nu_a;
			Xsig_pred_(1, i) = py + v * sin(yaw) * delta_t + 0.5 * delta_t * delta_t * sin(yaw) * nu_a;
		}
		Xsig_pred_(2, i) = v + delta_t * nu_a;
		Xsig_pred_(3, i) = yaw + yawdot * delta_t + 0.5 * delta_t * delta_t * nu_yawdd;
		Xsig_pred_(4, i) = yawdot + delta_t * nu_yawdd;
	}

	// predict state mean
	x_.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		x_ += weights_(i) * Xsig_pred_.col(i);
	}

	// predict state covariance matrix
	P_.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;
		P_ += weights_(i) * x_diff * x_diff.transpose();
	}
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
	/**
	 * TODO: Complete this function! Use lidar data to update the belief
	 * about the object's position. Modify the state vector, x_, and
	 * covariance, P_.
	 * You can also calculate the lidar NIS, if desired.
	 */
	 // transform sigma points into measurement space
	MatrixXd Zsig = MatrixXd(2, 2 * n_aug_ + 1);
	Zsig.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		double px = Xsig_pred_(0, i);
		double py = Xsig_pred_(1, i);
		Zsig(0, i) = px;
		Zsig(1, i) = py;
	}

	// calculate mean predicted measurement
	VectorXd z_pred = VectorXd(2);
	z_pred.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred += weights_(i) * Zsig.col(i);
	}

	// calculate innovation covariance matrix S
	MatrixXd S = MatrixXd(2, 2);
	S.setZero();
	MatrixXd R = MatrixXd(2, 2);
	R << std_laspx_ * std_laspx_, 0,
		0, std_laspy_ * std_laspy_;
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		S += weights_(i) * (Zsig.col(i) - z_pred) * (Zsig.col(i) - z_pred).transpose();
	}
	S += R;

	// create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(5, 2);
	Tc.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		Tc += weights_(i) * (Xsig_pred_.col(i) - x_) * (Zsig.col(i) - z_pred).transpose();
	}

	// calculate Kalman gain K;
	MatrixXd K = MatrixXd(5, 2);
	MatrixXd S_inverse = S.inverse();
	K = Tc * S_inverse;

	// update state mean and covariance matrix
	VectorXd z = meas_package.raw_measurements_;
	x_ = x_ + K * (z - z_pred);
	P_ = P_ - K * S * K.transpose();

	// Normalized innovation Squared (NIS):
	NIS_lidar_ = (z - z_pred).transpose() * S_inverse * (z - z_pred);
}


void UKF::UpdateRadar(MeasurementPackage meas_package) {
	/**
	* TODO: Complete this function! Use radar data to update the belief
	* about the object's position. Modify the state vector, x_, and
	* covariance, P_.
	* You can also calculate the radar NIS, if desired.
	*/
	// transform sigma points into measurement space
	MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);   // create matrix for sigma points in measurement space
	Zsig.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
		double px = Xsig_pred_(0, i);
		double py = Xsig_pred_(1, i);
		double v = Xsig_pred_(2, i);
		double yaw = Xsig_pred_(3, i);
		double s = px * px + py * py;
		if (s < 0.0001) {
			s = 0.0001;
		}
		Zsig(0, i) = sqrt(s);
		Zsig(1, i) = atan2(py, px);
		Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / sqrt(s);
	}

	// calculate mean predicted measurement
	VectorXd z_pred = VectorXd(3);
	z_pred.setZero();
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		z_pred += weights_(i) * Zsig.col(i);
	}

	// calculate innovation covariance matrix S
	MatrixXd S = MatrixXd(3, 3);
	S.setZero();
	MatrixXd R = MatrixXd(3, 3);
	R << std_radr_ * std_radr_, 0, 0,
		0, std_radphi_ * std_radphi_, 0,
		0, 0, std_radrd_ * std_radrd_;
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
		S += weights_(i) * z_diff * z_diff.transpose();
	}
	S += R;

	// create matrix for cross correlation Tc
	MatrixXd Tc = MatrixXd(5, 3);
	Tc.setZero();
	// calculate cross correlation matrix
	for (int i = 0; i < 2 * n_aug_ + 1; i++) {
		VectorXd z_diff = Zsig.col(i) - z_pred;
		// angle normalization
		while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
		while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;

		// state difference
		VectorXd x_diff = Xsig_pred_.col(i) - x_;
		// angle normalization
		while (x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
		while (x_diff(3) < -M_PI) x_diff(3) += 2.*M_PI;

		Tc += weights_(i) * x_diff * z_diff.transpose();
	}

	// calculate Kalman gain K;
	MatrixXd K = MatrixXd(5, 3);
	MatrixXd S_inverse = S.inverse();
	K = Tc * S_inverse;

	// update state mean and covariance matrix
	VectorXd z = meas_package.raw_measurements_;
	VectorXd z_diff = z - z_pred;
	while (z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
	while (z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
	x_ = x_ + K * z_diff;
	P_ = P_ - K * S * K.transpose();

	// Normalized innovation Squared (NIS):
	NIS_radar_ = z_diff.transpose() * S_inverse * z_diff;
}
