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

#ifndef INVKINMARKER_H_
#define INVKINMARKER_H_

#include "InvKin.h"
#include "InvKinInvDynRealTime.h"
#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include "IKSolverRT.h"
#include "XMLInterpreterV2.h"
#include <boost/timer/timer.hpp>

/**
 * Class for computing the joints angle using OpenSim inverse kinematic for the marker.
 */

class InvKinMarker: public InvKin
{
public:
	/**
	 * Constructor
	 */
	InvKinMarker(boost::shared_ptr<SimTK::State>& ptr_si, boost::shared_ptr<OpenSim::Model>& ptr_model);

	/**
	 * Destructor
	 */
	virtual ~InvKinMarker();

	/**
	 * Initialization method.
	 * @param xmlInterpreter Shared pointer to XMLInterpreter class for having the configuration of the IK.
	 */
	void init(const boost::shared_ptr<XMLInterpreter>& xmlInterpreter);

	/**
	 * Compute kinematics using std type.
	 * @param markerToPosition Map with marker name as key and x, y, z position of the marker.
	 */
	void computeKinematics(const std::map<string, std::vector<double> >& markerToPosition);

	/**
	 * Compute kinematics using SimTK type.
	 * @param markerGoal Array of marker position with the order of the OpenSim model.
	 */
	void computeKinematics(const SimTK::Array_<SimTK::Vec3>& markerGoal);

	/**
	 * Get the computed angles.
	 * @param dofName Name of the joint
	 */
	inline double getAngle(const string& dofName)
	{

		ptr_model_->getMultibodySystem().realize(*ptr_si_, SimTK::Stage::Position);
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
	/*
	 * Set the marker goal using std type. Private method use by computeKinematics.
	 */
	void setMarkerRef(const std::map<string, std::vector<double> >& markerToPosition);

	IKSolverRT* ptr_ikSolver_; //!< Pointer to OpenSim Ik solver.
	std::vector<std::string> markerNames_; //!< Vector of the names of the markers.
	bool firstpass_; //!< Firstpass_ member for the OpenSim Ik solver (call assembler).
};

#endif /* INVKINMARKER_H_ */
