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

#include "InvKinImu.h"

InvKinImu::InvKinImu(boost::shared_ptr<SimTK::State>& ptr_si, boost::shared_ptr<OpenSim::Model>& ptr_model,
		const std::vector<string> bodyNames) :
		InvKin(ptr_si, ptr_model), bodyNames_(bodyNames), firstPass_(true)
{
	for (std::vector<string>::const_iterator it1 = bodyNames_.begin(); it1 != bodyNames_.end(); it1++)
	{
		std::cout << *it1 << std::endl;
	}
	base_rotation_.setRotationFromAngleAboutX(90);

}

InvKinImu::InvKinImu(boost::shared_ptr<SimTK::State>& ptr_si, boost::shared_ptr<OpenSim::Model>& ptr_model) :
		InvKin(ptr_si, ptr_model), firstPass_(true)
{
	std::set<std::string> coordinateSet;

	for(int i = 0; i < ptr_model_->getCoordinateSet().getSize(); i++)
	{
		coordinateSet.insert(ptr_model_->getCoordinateSet().get(i).getName());
	}

	for(int i = 0; i < ptr_model_->getConstraintSet().getSize(); i++)
	{
		const OpenSim::CoordinateCouplerConstraint& ccc =  dynamic_cast<OpenSim::CoordinateCouplerConstraint&>(ptr_model_->getConstraintSet().get(i));
		coordinateSet.erase(ccc.getDependentCoordinateName());
//		const OpenSim::Array< std::string > coordianteNames = ccc.getIndependentCoordinateNames();
//		for(int j = 0; j < coordianteNames.getSize(); j++)
//			coordinateSet.insert(coordianteNames.get(j));
	}

	for(int i = 0; i < ptr_model_->getBodySet().getSize(); i++)
	{
		const OpenSim::Body& body = ptr_model_->getBodySet().get(i);
		if(body.hasJoint())
		{
			const OpenSim::CoordinateSet& coordSet = body.getJoint().getCoordinateSet();
			for(int j = 0; j < coordSet.getSize(); j++)
			{
				if(coordinateSet.find(coordSet.get(j).getName()) != coordinateSet.end())
				{
					bodyNames_.push_back(body.getName());
					break;
				}
			}
		}
	}
	for (std::vector<string>::const_iterator it1 = bodyNames_.begin(); it1 != bodyNames_.end(); it1++)
	{
		std::cout << *it1 << std::endl;
	}

}

InvKinImu::~InvKinImu()
{

}

void InvKinImu::computeKinematics(const std::map<string, std::vector<double> >& imuToPosition)
{
	// Realize the position of in the states.
	ptr_model_->getMultibodySystem().realize(*ptr_si_, SimTK::Stage::Position);

	// For the first pass get the initial rotation.
	if (firstPass_)
	{
		firstPass_ = false;
		for (std::vector<string>::const_iterator it1 = bodyNames_.begin(); it1 != bodyNames_.end(); it1++)
		{
			if (*it1 != "ground")  // No rotation between ground and ground.
			{
				// Get the rotation of the IMu on this body
				const std::vector<double>& imuQuaternion = imuToPosition.at(*it1);

				// Create the Rotation from the quaternion.
				SimTK::Rotation_<double> imuRotation(
						SimTK::Quaternion_<double>(imuQuaternion[0], imuQuaternion[1], imuQuaternion[2],
								imuQuaternion[3]));

				// Get the initial Rotation of the bodies.
				const SimTK::Rotation_<double>& bodyRotation = ptr_model_->getMatterSubsystem().getMobilizedBody(
						ptr_model_->getBodySet().get(*it1).getIndex()).getBodyRotation(*ptr_si_);

				// Get the transform between the initial IMU rotation and the initial body rotation.
				imuRotationInit_M_IMU_.push_back(bodyRotation.invert() * imuRotation);
			}
		}
	}
	else
	{
		for (std::vector<string>::const_iterator it1 = bodyNames_.begin(); it1 != bodyNames_.end(); it1++)
		{
			if (*it1 != "ground") // No rotation between ground and ground.
			{
				// Get the rotation of the IMu on this body
				const std::vector<double>& quaternion_pre = imuToPosition.at(*(it1 - 1));
				const std::vector<double>& quaternion = imuToPosition.at(*it1);

				// Create the Rotation from the quaternion.
				SimTK::Rotation_<double> imuRotation(
						SimTK::Quaternion_<double>(quaternion[0], quaternion[1], quaternion[2], quaternion[3]));

				// Get the parent body rotation.
				const SimTK::Rotation_<double>& parentBodyRotation =
						ptr_model_->getMatterSubsystem().getMobilizedBody(
								ptr_model_->getBodySet().get(
										ptr_model_->getBodySet().get(*it1).getJoint().getParentName()).getIndex()).getBodyRotation(
								*ptr_si_);

				// Compute the new rotation of this body.
				SimTK::Rotation_<double> rotation_F_M = parentBodyRotation.invert() * imuRotation
						* imuRotationInit_M_IMU_[std::distance<std::vector<string>::const_iterator>(bodyNames_.begin(),
								it1)];

				// Set the rotation of this body using the previous computed rotation.
				ptr_model_->getMatterSubsystem().getMobilizedBody(ptr_model_->getBodySet().get(*it1).getIndex()).setQToFitRotation(
						*ptr_si_, rotation_F_M);

				// Realize the position in the state.
				ptr_model_->getMultibodySystem().realize(*ptr_si_, SimTK::Stage::Position);
			}
		}
	}
}

void InvKinImu::computeKinematics(SimTK::Array_<SimTK::dQuaternion>& imuToPosition)
{
	// Realize the position of in the states.
	ptr_model_->getMultibodySystem().realize(*ptr_si_, SimTK::Stage::Position);

	// For the first pass get the initial rotation.
//	if (firstPass_)
//	{
//		unsigned short cpt = 0;
//		firstPass_ = false;
//		for (std::vector<string>::const_iterator it1 = bodyNames_.begin(); it1 != bodyNames_.end(); it1++)
//		{
//			if (*it1 != "ground")  // No rotation between ground and ground.
//			{
//				// Get the rotation of the IMu on this body
//				const SimTK::dQuaternion& imuQuaternion = imuToPosition.at(std::distance<std::vector<string>::const_iterator>(bodyNames_.begin(), it1) - 1);
//
//				// Create the Rotation from the quaternion.
//				SimTK::Rotation_<double> imuRotation(imuQuaternion);
//
//				// Get the initial Rotation of the bodies.
//				const SimTK::Rotation_<double>& bodyRotation = ptr_model_->getMatterSubsystem().getMobilizedBody(
//						ptr_model_->getBodySet().get(*it1).getIndex()).getBodyRotation(*ptr_si_);
//
//				// Get the transform between the initial IMU rotation and the initial body rotation.
//				imuRotationInit_M_IMU_.push_back(imuRotation);//bodyRotation.invert() *
//			}
//		}
//	}
//	else
//	{
		for (std::vector<string>::const_iterator it1 = bodyNames_.begin(); it1 != bodyNames_.end(); it1++)
		{
			if (*it1 != "ground") // No rotation between ground and ground.
			{
				const int& cpt = std::distance<std::vector<string>::const_iterator>(bodyNames_.begin(), it1);
				// Get the rotation of the IMU on this body
				SimTK::Rotation_<double> rotation_pre;
				if(cpt - 1 == 0)
					rotation_pre = SimTK::Rotation_<double>(imuToPosition.at(cpt - 1)) * base_rotation_;
				else
					rotation_pre = SimTK::Rotation_<double>(imuToPosition.at(cpt - 1));
				SimTK::Rotation_<double> rotation(imuToPosition.at(cpt));
				SimTK::Rotation_<double> temp_rotation = rotation * rotation_pre.invert();
				// Create the Rotation from the quaternion.
				SimTK::Rotation_<double>  imuRotation(temp_rotation);


				// Get the parent body rotation.
//				const SimTK::Rotation_<double>& parentBodyRotation =
//						ptr_model_->getMatterSubsystem().getMobilizedBody(
//								ptr_model_->getBodySet().get(
//										ptr_model_->getBodySet().get(*it1).getJoint().getParentName()).getIndex()).getBodyRotation(
//								*ptr_si_);

//				const SimTK::Rotation_<double>& rotInit = imuRotationInit_M_IMU_[std::distance<std::vector<string>::const_iterator>(bodyNames_.begin(),
//						it1) - 1];

				// Compute the new rotation of this body.
				SimTK::Rotation_<double> rotation_F_M = imuRotation; //parentBodyRotation.invert() * imuRotation * rotInit;

				// Set the rotation of this body using the previous computed rotation.
				ptr_model_->getMatterSubsystem().getMobilizedBody(ptr_model_->getBodySet().get(*it1).getIndex()).setQToFitRotation(
						*ptr_si_, rotation_F_M);

				// Realize the position in the state.
				ptr_model_->getMultibodySystem().realize(*ptr_si_, SimTK::Stage::Position);
			}
		}
//	}
}
