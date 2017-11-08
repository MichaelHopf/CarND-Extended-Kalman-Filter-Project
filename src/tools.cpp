#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {
	/**
	TODO:
	* Calculate the RMSE here.
	*/
	VectorXd RMSE(4);
	RMSE << 0, 0, 0, 0;

	// check if it's ok to go on
	if (estimations.size() != ground_truth.size() || estimations.size() == 0)
	{
		std::cout << "Invalid estimation or ground truth data! \n";
		return RMSE;
	}
	else
	{
		for (unsigned int i = 0; i < estimations.size(); ++i)
		{
			VectorXd residuals = estimations[i] - ground_truth[i];

			residuals = residuals.array() * residuals.array();

			RMSE += residuals;
		}
	}

	RMSE /= estimations.size();

	RMSE = RMSE.array().sqrt();

	return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	/**
	TODO:
	* Calculate a Jacobian here.
	*/
	MatrixXd J(3, 4);
	const float EPS = pow(10, -5);
	const float px = x_state(0);
	const float py = x_state(1);
	const float vx = x_state(2);
	const float vy = x_state(3);

	const float div1 = pow(px, 2) + pow(py, 2);
	float div2 = sqrt(div1);
	float div3 = div1*div2;

	if (fabs(div2) < EPS) {
		div2 = EPS;
	}
	if (fabs(div3) < EPS) {
		div3 = EPS;
	}

	const float px_div2 = px / div2;
	const float py_div2 = py / div2;
	const float vxpy_vypx_div3 = (vx*py - vy*px)/div3;

	J << px_div2, py_div2, 0, 0,
		-py / div1, px / div1, 0, 0,
		py*vxpy_vypx_div3, -px*vxpy_vypx_div3, px_div2, py_div2;

	return J;
}
