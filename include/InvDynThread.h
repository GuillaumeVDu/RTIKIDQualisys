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

#ifndef INVDYN_H_
#define INVDYN_H_

#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <boost/timer/timer.hpp>

/**
 * Class for using the OpenSim inverse dynamic solver.
 */

class InvDyn
{
public:
	/**
	 * Constructor
	 */
	InvDyn(SimTK::State& si, OpenSim::Model& model);

	/**
	 * Destructor
	 */
	virtual ~InvDyn();

	/**
	 * Compute the torque with only the position from the OpenSim state.
	 * @return Simtk vector with the torque for all joints (joints order of the model).
	 */
	inline SimTK::Vector computeTorque()
	{
		return _idSolver->solve(_si);
	}

	/**
	 * Compute the torque using the OpenSim state and the provided acceleration.
	 * @param udot Simtk vector with the accelerations for all joints (joints order of the model).
	 * @return Simtk vector with the torque for all joints (joints order of the model).
	 */
	inline SimTK::Vector computeTorque(const SimTK::Vector& udot)
	{
		return _idSolver->solve(_si, udot);
	}

	/**
	 * Compute the torque using the OpenSim state, the position in the ground referential of the
	 * ground reaction force, the ground reaction force, the ground reaction torque and the provided acceleration.
	 * For now, only one force can be applied.
	 * @param force The ground reaction force (fx, fy, fz).
	 * @param positionGround The position of the ground reaction force (x, y, z).
	 * @param torque The ground reaction torque (tx, ty, tz).
	 * @param appliedBodyName The name of the body on where the forces are applied.
	 * @param udot Simtk vector with the accelerations for all joints (joints order of the model).
	 * @return Simtk vector with the torque for all joints (joints order of the model).
	 */
	SimTK::Vector computeTorque(const SimTK::Array_<SimTK::fVec9>& grf,
			const std::vector<std::string>& appliedBodyName, const SimTK::Vector& udot);

	/**
	 * Compute the torque using the OpenSim state, the position in the ground referential of the
	 * ground reaction force, the ground reaction force and the ground reaction torque.
	 * For now, only one force can be applied.
	 * @param force The ground reaction force (fx, fy, fz).
	 * @param positionGround The position of the ground reaction force (x, y, z).
	 * @param torque The ground reaction torque (tx, ty, tz).
	 * @param appliedBodyName The name of the body on where the forces are applied.
	 * @return Simtk vector with the torque for all joints (joints order of the model).
	 */
	SimTK::Vector computeTorque(const SimTK::Array_<SimTK::fVec9>& grf,
			const std::vector<std::string>& appliedBodyName);

protected:
	SimTK::State& _si; //!< Shared pointer to OpenSim state.
	OpenSim::Model& _model; //!< Shared pointer to OpenSim model
	OpenSim::InverseDynamicsSolver* _idSolver; //!< Shared pointer to OpenSim Inverse dynamic solver.
};

#endif /* INVDYN_H_ */
