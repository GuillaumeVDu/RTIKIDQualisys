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

#ifndef KALMANFILTERSIMTK_H_
#define KALMANFILTERSIMTK_H_

#include "InvKinInvDynRealTime.h"
#include <OpenSim/OpenSim.h>

/**
 * Class for the Kalman filter using SimTK library for filtering the joint angle and compute the velocity and acceleration.
 */

class KalmanFilterSimTK {
public:
	/*
	 * Default Constructor
	 */
	KalmanFilterSimTK();

	/**
	 * Constructor
	 */
	KalmanFilterSimTK(const double& r, const double& p, const double& sigma_da, const double& dt);

	/*
	 * Destructor
	 */
	virtual ~KalmanFilterSimTK();

	/**
	 * Computation of the Kalman filter.
	 * @param q position of the joint.
	 * @return Vector of the position, velocity and acceleration.
	 */
	const SimTK::fVec3& computeKalmanFilter(const double& q);
	
	void updateDt( const double & dt);

protected:

	/**
	 * Initialization of the different filter parameter private method.
	 */
	void init(const double& r, const double& p, const double& sigma_da, const double& dt);

	double dt_; //!< Time between two sample
	double z_; //!< Measured value
	double r_; //!< Covariance of the observation noise
	double _sigma_da;

	SimTK::fVec3 u_; //!< Control Signal
	SimTK::fMat33 A_; //!< State transition model
	SimTK::fMat33 Q_; //!< Covariance of the process noise
	SimTK::fMat33 P_; //!< Covariance error
	SimTK::fMat33 B_; //!< Control-input model
	SimTK::fVec3 H_; //!< Observation model
	SimTK::fVec3 K_; //!< Kalman gain
	SimTK::fVec3 x_; //!< Current estimation
	bool first_pass_; //!< First pass for the initialization of the Kalman filter
};

#endif /* KALMANFILTERSIMTK_H_ */
