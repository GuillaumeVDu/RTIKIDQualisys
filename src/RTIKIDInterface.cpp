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

#include "RTIKIDInterface.h"

//#include <boost/timer.hpp>
//#undef timer
//#include <boost/timer/timer.hpp>

RTIKIDInterface::RTIKIDInterface(const string& xmlName)
{

	// create XML interpreter for having parameters
	xmlInterpreter_ = boost::shared_ptr<XMLInterpreter>(
			new XMLInterpreter(xmlName));
	xmlInterpreter_->readXML();

	// Create OpenSim model
	ptr_model_ = boost::shared_ptr<OpenSim::Model>(
			new OpenSim::Model(getOsimFile()));

// 	ptr_model_->setUseVisualizer(xmlInterpreter_->getVisualizerUse());

	// Create OpenSim state
	ptr_si_ = boost::shared_ptr<SimTK::State>(
			new SimTK::State(ptr_model_->initSystem()));

	OpenSim::ForceSet& modelForces = ptr_model_->updForceSet();

	for (int i = 0; i < modelForces.getSize(); i++)
	{
		modelForces[i].setDisabled(*(ptr_si_), true);
	}

//	OpenSim::Set<OpenSim::Muscle>& muscles = ptr_model_->updMuscles();
//	for(int i=0; i<muscles.getSize(); i++){
//		muscles[i].setDisabled(*(ptr_si_), true);
//	}

//	OpenSim::Set<OpenSim::Actuator>& acts = ptr_model_->updActuators();
//	for(int i=0; i<acts.getSize(); i++){
//		acts[i].setDisabled(*(ptr_si_), true);
//	}

// Create visualizer and set-up option
// 	if (xmlInterpreter_->getVisualizerUse())
// 	{
// 		ptr_model_->updDisplayHints().setShowWrapGeometry(true);
// 		ptr_model_->updDisplayHints().setShowMarkers(false);
// 		ptr_model_->updDisplayHints().setShowMusclePaths(false);
// 		ptr_model_->updDisplayHints().setShowFrames(true);
// 		ptr_model_->updDisplayHints().setShowForces(true);
// 		ptr_model_->updDisplayHints().setShowDebugGeometry(true);
// 		ptr_model_->updDisplayHints().setShowContactGeometry(true);
// 		string winTitle("RealTime IK and ID");
// 		ptr_model_->updVisualizer().updSimbodyVisualizer().setWindowTitle(
// 				winTitle);
// 
// 	}
	// Set up Ik
	switch (xmlInterpreter_->GetIKType())
	{
	case XMLInterpreter::Marker:
	{
		ik_ = boost::shared_ptr<InvKinMarker>(
				new InvKinMarker(ptr_si_, ptr_model_));
		dynamic_cast<InvKinMarker*>(ik_.get())->init(xmlInterpreter_);
		break;
	}
	case XMLInterpreter::IMU:
	{
		ik_ = boost::shared_ptr<InvKinImu>(
				new InvKinImu(ptr_si_, ptr_model_,
						xmlInterpreter_->getImuBodies()));
		break;
	}
	}

	// Set up ID id needed
	if (xmlInterpreter_->getUseID())
		id_ = boost::shared_ptr<InvDyn>(new InvDyn(ptr_si_, ptr_model_));

	// Create joint name vector
	const OpenSim::CoordinateSet& coordSet = ptr_model_->getCoordinateSet();
	for (int i = 0; i < coordSet.getSize(); i++)
		dofNames_.push_back(coordSet.get(i).getName());

	// Create muscle name vector
	for (int i = 0; i < ptr_model_->getMuscles().getSize(); i++)
		muscleNames_.push_back(ptr_model_->getMuscles().get(i).getName());

	// Create Kalman filter if needed
	if (xmlInterpreter_->getKalmanUse())
	{
		for (int i = 0; i < coordSet.getSize(); i++)
			kalmanVect_.push_back(
					boost::shared_ptr<KalmanFilterSimTK>(
							new KalmanFilterSimTK(xmlInterpreter_->getR(),
									xmlInterpreter_->getP(),
									xmlInterpreter_->getSigmaDa(),
									xmlInterpreter_->getDt())));
		qddotSimTK_.resize(coordSet.getSize());
	}
}

RTIKIDInterface::~RTIKIDInterface()
{
}

void RTIKIDInterface::runIK(
		const std::map<string, std::vector<double> >& values)
{
	switch (xmlInterpreter_->GetIKType())
	{
	case XMLInterpreter::Marker:
	{
		dynamic_cast<InvKinMarker*>(ik_.get())->computeKinematics(values);
		break;
	}
	case XMLInterpreter::IMU:
	{
		dynamic_cast<InvKinImu*>(ik_.get())->computeKinematics(values);
		break;
	}
	}
}

void RTIKIDInterface::computeKalmanFilter()
{
	if (xmlInterpreter_->getKalmanUse())
	{
		const OpenSim::CoordinateSet& coordSet = ptr_model_->getCoordinateSet();
		for (std::vector<string>::const_iterator it = dofNames_.begin();
				it != dofNames_.end(); it++)
		{
//			boost::timer::auto_cpu_timer auto_t2;
			SimTK::Vec3 states;

			// Compute the angle filtered, velocity and acceleration
			states =
					kalmanVect_[std::distance<
							std::vector<string>::const_iterator>(
							dofNames_.begin(), it)]->computeKalmanFilter(
							getAngle(*it));
			//std::cout << *it << std::endl;
			// If the joint is not locked set the angle and the velocity otherwise set the acceleration to zero
			if (!coordSet.get(*it).getLocked(*(ptr_si_)))
			{
//				boost::timer::auto_cpu_timer auto_t3;
				ptr_model_->updMatterSubsystem().getMobilizedBody(coordSet.get(*it).getBodyIndex()).setOneQ(*(ptr_si_),coordSet.get(*it).getMobilizerQIndex(),states[0]);

				coordSet.get(*it).setSpeedValue(*(ptr_si_), states[1]);
				qddot_[*it] = states[2];
				qddotSimTK_.set(
						std::distance<std::vector<string>::const_iterator>(
								dofNames_.begin(), it), states[2]);
			}
			else
			{
				qddotSimTK_.set(
						std::distance<std::vector<string>::const_iterator>(
								dofNames_.begin(), it), 0);
				qddot_[*it] = 0;
			}
		}
	}
}

double RTIKIDInterface::getAcceleration(const string& dofName)
{
	if (xmlInterpreter_->getKalmanUse())
		return qddot_.at(dofName);
	else
	{
		if (ptr_si_->getSystemStage() != SimTK::Stage::Acceleration)
			ptr_model_->getMultibodySystem().realize(*(ptr_si_),
					SimTK::Stage::Acceleration);
		return ptr_model_->getCoordinateSet().get(dofName).getAccelerationValue(
				*(ptr_si_));
	}
}

double RTIKIDInterface::getVelocity(const string& dofName)
{
	if (ptr_si_->getSystemStage() != SimTK::Stage::Velocity)
		ptr_model_->getMultibodySystem().realize(*(ptr_si_),
				SimTK::Stage::Velocity);
	return ptr_model_->getCoordinateSet().get(dofName).getSpeedValue(*(ptr_si_));
}

double RTIKIDInterface::getAngle(const string& dofName)
{
	if (ptr_si_->getSystemStage() != SimTK::Stage::Position)
		ptr_model_->getMultibodySystem().realize(*(ptr_si_),
				SimTK::Stage::Position);
	return ptr_model_->getCoordinateSet().get(dofName).getValue(*(ptr_si_));
}

SimTK::Vector RTIKIDInterface::computeTorqueSimTK()
{
	if (xmlInterpreter_->getUseID())
		if (xmlInterpreter_->getKalmanUse())
			return id_->computeTorque(qddotSimTK_);
		else
			return id_->computeTorque();
	else
	{
		SimTK::Vector nullVect(SimTK::NaN);
		return nullVect;
	}
}

SimTK::Vector RTIKIDInterface::computeTorqueSimTK(const SimTK::Array_<SimTK::fVec9>& grf,
		const std::vector<std::string>& appliedBodyName)
{
	if (xmlInterpreter_->getUseID())
		if (xmlInterpreter_->getKalmanUse())
			return id_->computeTorque(grf,
					appliedBodyName, qddotSimTK_);
		else
			return id_->computeTorque(grf,
					appliedBodyName);
	else
	{
		SimTK::Vector nullVect(SimTK::NaN);
		return nullVect;
	}
}

std::vector<double> RTIKIDInterface::computeTorqueStd(const SimTK::Array_<SimTK::fVec9>& grf,
		const std::vector<std::string>& appliedBodyName)
{
	if (xmlInterpreter_->getUseID())
	{
		SimTK::Vector simTKvect;
		if (xmlInterpreter_->getKalmanUse())
			simTKvect = id_->computeTorque(grf,
					appliedBodyName, qddotSimTK_);
		else
			simTKvect = id_->computeTorque(grf,
					appliedBodyName);
		std::vector<double> stdVect;
		for (int i = 0; i < simTKvect.size(); i++)
			stdVect.push_back(simTKvect[i]);
		return stdVect;
	}
	else
	{
		std::vector<double> nullVect;
		return nullVect;
	}
}

std::vector<double> RTIKIDInterface::computeTorqueStd()
{
	if (xmlInterpreter_->getUseID())
	{
		SimTK::Vector simTKvect;
		if (xmlInterpreter_->getKalmanUse())
			simTKvect = id_->computeTorque(qddotSimTK_);
		else
			simTKvect = id_->computeTorque();
		std::vector<double> stdVect;
		for (int i = 0; i < simTKvect.size(); i++)
			stdVect.push_back(simTKvect[i]);
		return stdVect;
	}
	else
	{
		std::vector<double> nullVect;
		return nullVect;
	}
}

