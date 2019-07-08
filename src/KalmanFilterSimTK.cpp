// Copyright (c) 2015, Guillaume Durandau and Massimo Sartori
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "KalmanFilterSimTK.h"

KalmanFilterSimTK::KalmanFilterSimTK(): first_pass_(true)
{
	double r = 10;
	double p = 10;
	double sigma_da = 10;
	double dt = 0.01; // 100Hz
	init(r, p, sigma_da, dt);
}

KalmanFilterSimTK::KalmanFilterSimTK(const double& r, const double& p, const double& sigma_da, const double& dt): first_pass_(true)
{
	init(r, p, sigma_da, dt);
}

KalmanFilterSimTK::~KalmanFilterSimTK() {

}

void KalmanFilterSimTK::init(const double& r, const double& p, const double& sigma_da, const double& dt)
{
	u_.setToZero();

	dt_ = dt;
	
	_sigma_da = sigma_da;

	A_ = SimTK::fMat33(	1, dt_, std::pow(dt_, 2) / 2,
						0, 1, 	dt_,
						0, 0, 	1
					);

	Q_ = SimTK::fMat33(	std::pow(dt_, 6) / 6,   std::pow(dt_, 5) / 12, std::pow(dt_, 4) / 6,
						std::pow(dt_, 5) / 12,  std::pow(dt_, 4) / 6,  std::pow(dt_, 3) / 2,
						std::pow(dt_, 4) / 6,   std::pow(dt_, 3) / 2,  std::pow(dt_, 2)
					);

	Q_ = Q_ *  _sigma_da;

	P_ = SimTK::fMat33(1) * p;

	B_.setToZero();

	H_ = SimTK::fVec3(1,0,0);

	K_.setToZero();

	x_.setToZero();

	r_ = r;

	z_ = 0;
}

void KalmanFilterSimTK::updateDt( const double & dt)
{
	dt_ = dt;

	A_ = SimTK::fMat33(	1, dt_, std::pow(dt_, 2) / 2,
						0, 1, 	dt_,
						0, 0, 	1
					);

	Q_ = SimTK::fMat33(	std::pow(dt_, 6) / 6,   std::pow(dt_, 5) / 12, std::pow(dt_, 4) / 6,
						std::pow(dt_, 5) / 12,  std::pow(dt_, 4) / 6,  std::pow(dt_, 3) / 2,
						std::pow(dt_, 4) / 6,   std::pow(dt_, 3) / 2,  std::pow(dt_, 2)
					);
	
	Q_ = Q_ *  _sigma_da;
}

const SimTK::fVec3& KalmanFilterSimTK::computeKalmanFilter(const double& q)
{
	if(first_pass_)
	{
		x_[0] = q;
		first_pass_ = false;
	}
	else
	{
		z_ = q;
		//Prediction for state vector and covariance:
		x_ = A_ * x_ + B_ * u_;

		P_ = A_ * P_ * A_.positionalTranspose() + Q_;

		//Compute Kalman gain factor:
		K_ = (P_ * H_) * 1/(H_.positionalTranspose() * P_ * H_ + r_);

		//Correction based on observation:
		x_ = x_ + K_ * (z_ - SimTK::dot(H_, x_));

		P_ = P_ - K_ * H_.positionalTranspose() * P_;
	}

	return x_;
}
