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

#include "InvDyn.h"

InvDyn::InvDyn(boost::shared_ptr<SimTK::State>& ptr_si, boost::shared_ptr<OpenSim::Model> ptr_model) :
		ptr_si_(ptr_si), ptr_model_(ptr_model), ptr_idSolver_(new OpenSim::InverseDynamicsSolver(*ptr_model_))
{

}

InvDyn::~InvDyn()
{
}

SimTK::Vector InvDyn::computeTorque(const SimTK::Array_<SimTK::fVec9>& grf, const std::vector<std::string>& appliedBodyName, const SimTK::Vector& udot)
{

#ifdef VERBOSE
	std::cout << "force: " << force[0] << ", " << force[1] << ", " << force[2] << std::endl
	std::cout << "positionGround: " << positionGround[0] << ", " << positionGround[1] << ", " << positionGround[2] << std::endl
	std::cout << "torque: " << torque[0] << ", " << torque[1] << ", " << torque[2] << std::endl
#endif
	// Inspiration for this method from the OpenSim source code for the computation of external forces.

	ptr_model_->getMultibodySystem().realize(*(ptr_si_), SimTK::Stage::Dynamics);

	// Get the ground body reference.
	const OpenSim::Body& ground = ptr_model_->getGroundBody();

	// Get the force applied on all the bodies in the model.
	SimTK::Vector_<SimTK::SpatialVec> appliedBodyForces = ptr_model_->getMultibodySystem().getRigidBodyForces(*ptr_si_,
			SimTK::Stage::Dynamics);


	for(std::vector<std::string>::const_iterator itName = appliedBodyName.begin(); itName < appliedBodyName.end(); itName++)
	{
		const unsigned int& cpt = std::distance<std::vector<std::string>::const_iterator>(appliedBodyName.begin(), itName);
//		std::cout << *itName << std::endl;
		// Position in the applied body.
		SimTK::Vec3 pAppliedBody(SimTK::NaN);

		SimTK::Vec3 force(grf[cpt][0],grf[cpt][1], grf[cpt][2]);
		SimTK::Vec3 torque(grf[cpt][3],grf[cpt][4], grf[cpt][5]);
		SimTK::Vec3 position(grf[cpt][6],grf[cpt][7], grf[cpt][8]);

		// Get the reference of the body on which the force is applied.
		const OpenSim::Body& appliedToBody = ptr_model_->getBodySet().get(itName->c_str());

		// Transform the position in the ground to the position on the body.
		ptr_model_->getSimbodyEngine().transformPosition(*ptr_si_, ground, position, appliedToBody, pAppliedBody);

		// Add on this variable the new force from the ground reaction force on the applied body.
		ptr_model_->getMatterSubsystem().addInStationForce(*ptr_si_, SimTK::MobilizedBodyIndex(appliedToBody.getIndex()),
				pAppliedBody, force, appliedBodyForces);

		// Add on this variable the new torque from the ground reaction torque on the applied body.
		ptr_model_->getMatterSubsystem().addInBodyTorque(*ptr_si_, SimTK::MobilizedBodyIndex(appliedToBody.getIndex()),
				torque, appliedBodyForces);
	}

#ifdef VERBOSE
	std::cout << "appliedBodyForces before: " << " ";
	for(int i = 0; i < ptr_model_->getBodySet().getSize(); i++)
	{
		std::cout << appliedBodyForces[i][1] << " ";
	}
	std::cout << std::endl;
#endif

	// Get the mobility forces of all the bodies.
	const SimTK::Vector &appliedMobilityForces = ptr_model_->getMultibodySystem().getMobilityForces(*ptr_si_,
			SimTK::Stage::Dynamics);

#ifdef VERBOSE
	std::cout << "appliedBodyForces after: " << " ";
	for(int i = 0; i < ptr_model_->getBodySet().getSize(); i++)
	{
		std::cout << appliedBodyForces[i][1] << " ";
	}
	std::cout << std::endl;

	std::cout << "appliedMobilityForces" << " ";
	for(int i = 0; i < ptr_model_->getCoordinateSet().getSize(); i++)
	{
		std::cout << appliedMobilityForces[i] << " ";
	}
	std::cout << std::endl;
#endif

	// Compute the torque usimg the OpenSim inverse dynamics solver.
	return ptr_idSolver_->solve(*ptr_si_, udot, appliedMobilityForces, appliedBodyForces);
}

SimTK::Vector InvDyn::computeTorque(const SimTK::Array_<SimTK::fVec9>& grf, const std::vector<std::string>& appliedBodyName)
{
	// Inspiration for this method from the OpenSim source code for the computation of external forces.

	// Acceleration
	SimTK::Vector udot;

	// Resize to the good numbers of coordinate in the model.
	udot.resize(ptr_model_->getCoordinateSet().getSize());

	// Set it to zero.
	udot.setToZero();

	// Call the computeTorque method with zero acceleration.
	return computeTorque(grf, appliedBodyName, udot);
}
