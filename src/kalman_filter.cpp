#include "kalman_filter.h"
#include <math.h>
#include "tools.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
const float EPS = pow(10, -5);

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
	MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	/**
	TODO:
	* predict the state
	*/
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Kalman Filter equations
	*/
	const VectorXd y = z - H_ * x_;
	const MatrixXd PH = P_ * H_.transpose();
	MatrixXd S = H_ * PH + R_;
	MatrixXd K = PH * S.inverse();

	// new estimate
	x_ += K*y;
	P_ -= K * H_ * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Extended Kalman Filter equations
	*/
	float px = x_(0);
	const float py = x_(1);
	const float vx = x_(2);
	const float vy = x_(3);

	if (fabs(px)<EPS) {
		px = EPS;
	}

	float sqPxy = sqrt(pow(px, 2) + pow(py, 2));

	if (fabs(sqPxy) < EPS) {
		sqPxy = EPS;
	}
	VectorXd h(3);
	h << sqPxy, atan2(py,px), (px*vx + py*vy) / sqPxy;

	Tools mytools;

	MatrixXd Hj = mytools.CalculateJacobian(x_);
	VectorXd y = z - h;

	// Avoid unneccessary looping:
	if (fabs(y(1)) > 20 * M_PI) {
		float divide = fabs(y(1)) / (2*M_PI);
		int round_divide = floorf(divide);
		if (y(1) > 0) {
			y(1) -= round_divide;
		}
		else if (y(1) < 0) {
			y(1) += round_divide;
		}
	}


	while (y(1) > M_PI) {
		y(1) -= 2 * M_PI;
	}
	while (y(1) < -M_PI) {
		y(1) += 2 * M_PI;
	}

	MatrixXd PHj = P_ * Hj.transpose();
	MatrixXd S = Hj * PHj + R_;
	MatrixXd K = PHj * S.inverse();
	// new estimate
	x_ += K*y;
	P_ -= K* Hj * P_;
}
