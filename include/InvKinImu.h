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

#ifndef INVKINIMU_H_
#define INVKINIMU_H_

#include "InvKin.h"
#include <algorithm>

/**
 * Class for computing the joints angle using IMU.
 */

class InvKinImu: public InvKin
{
public:

	/**
	 * Constructor
	 */
	InvKinImu(boost::shared_ptr<SimTK::State>& ptr_si, boost::shared_ptr<OpenSim::Model>& ptr_model,
			const std::vector<string> bodyNames);

	/**
	 * Constructor
	 */
	InvKinImu(boost::shared_ptr<SimTK::State>& ptr_si, boost::shared_ptr<OpenSim::Model>& ptr_model);

	/**
	 * Destructor
	 */
	virtual ~InvKinImu();

	/**
	 * Compute kinematics using the IMu rotation.
	 *
	 * \f$R^{F}_{M} = R^{G^{-1}}_{F}R^{G}_{IMU}R^{M}_{IMU_{INIT}} \f$
	 *
	 * With \f$F\f$ the parent body referential, \f$G\f$ the ground referential, \f$IMU\f$ the IMU referential,
	 *  \f$M\f$ this body and \f$IMU_{INIT}\f$ the initial IMU referential.
	 *
	 * @param imuToPosition Map with IMU name as key and x, y, z, w quaternion of the IMU.
	 */
	void computeKinematics(const std::map<string, std::vector<double> >& imuToPosition);

	/**
	 * Compute kinematics using the IMu rotation.
	 *
	 * \f$R^{F}_{M} = R^{G^{-1}}_{F}R^{G}_{IMU}R^{M}_{IMU_{INIT}} \f$
	 *
	 * With \f$F\f$ the parent body referential, \f$G\f$ the ground referential, \f$IMU\f$ the IMU referential,
	 *  \f$M\f$ this body and \f$IMU_{INIT}\f$ the initial IMU referential.
	 *
	 * @param imuToPosition Map with IMU name as key and x, y, z, w quaternion of the IMU.
	 */
	void computeKinematics(SimTK::Array_<SimTK::dQuaternion>& imuToPosition);

	/**
	 * Get the computed angles.
	 * @param dofName Name of the joint
	 */
	inline double getAngle(const string& dofName)
	{
		return ptr_model_->getCoordinateSet().get(dofName).getValue(*ptr_si_);
	}

	/**
	 * Get the joint velocity.
	 * @param dofName Name of the joint
	 * @todo Return 0 or delete this method because it can be misleading since the velocity is not computed.
	 */
	inline double getVelocity(const string& dofName)
	{
		return ptr_model_->getCoordinateSet().get(dofName).getSpeedValue(*ptr_si_);
	}

	/**
	 * Get the joint acceleration.
	 * @param dofName Name of the joint
	 * @todo return 0 or delete this method because it can be misleading since the velocity is not computed.
	 */
	inline double getAcceleration(const string& dofName)
	{
		return ptr_model_->getCoordinateSet().get(dofName).getAccelerationValue(*ptr_si_);
	}

protected:

	std::vector<string> bodyNames_; //!< Vector of the names of the bodies.
	std::vector<SimTK::Rotation_<double> > imuRotationInit_M_IMU_; //!< Vector of rotation of the bodies.
	SimTK::Rotation_<double> base_rotation_;
	std::vector<SimTK::Quaternion_<double> > imuQuaternionInit_M_IMU_; //!< Vector of quaternion of the bodies.
	bool firstPass_; //!< first pass member for computing the initial rotation.
};

#endif /* INVKINIMU_H_ */
