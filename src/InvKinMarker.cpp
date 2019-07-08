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

#include "InvKinMarker.h"

InvKinMarker::InvKinMarker(boost::shared_ptr<SimTK::State>& ptr_si, boost::shared_ptr<OpenSim::Model>& ptr_model) :
		InvKin(ptr_si, ptr_model), firstpass_(true)
{
}

InvKinMarker::~InvKinMarker()
{
	delete ptr_ikSolver_;
}

void InvKinMarker::init(const boost::shared_ptr<XMLInterpreter>& xmlInterpreter)
{
	// create the OpenSim inverse kinematics.
	ptr_ikSolver_ = new IKSolverRT(*ptr_model_, xmlInterpreter->getMarkersNames(), xmlInterpreter->getMarkersWeights());

	// Get the markers names from the XML file.
	markerNames_ = xmlInterpreter->getMarkersNames();

	// Set the accuracy of the IK from the XML file use more computation times when small accuracy asked.
	ptr_ikSolver_->setAccuracy(xmlInterpreter->getMaxMarkerError());

	// Set if we enforce constraint of the IK (respect of the limit) use more computation times when used.
	ptr_ikSolver_->setEnforceContraint(xmlInterpreter->getEnforceConstraintUse());
}

void InvKinMarker::computeKinematics(const std::map<string, std::vector<double> >& markerToPosition)
{
#ifdef TIME
	cout << "Angle:  ";
	boost::timer::auto_cpu_timer t;
#endif

	// Set the marker position in the OpenSim IK using private method.
	setMarkerRef(markerToPosition);

	// If first pass call assemble method (have to called at the beginning) else call track method (track small change).
	if (firstpass_)
	{
		ptr_ikSolver_->assemble(*ptr_si_);
		firstpass_ = false;
	}
	else
		ptr_ikSolver_->track(*ptr_si_);
}

void InvKinMarker::computeKinematics(const SimTK::Array_<SimTK::Vec3>& markerGoal)
{
#ifdef TIME
	cout << "Angle:  ";
	boost::timer::auto_cpu_timer t;
#endif


	// Set the marker position in the OpenSim IK.
	ptr_ikSolver_->setMarkerGoal(markerGoal);

	// If first pass call assemble method (have to called at the beginning) else call track method (track small change).
	if (firstpass_)
	{
		ptr_ikSolver_->assemble(*ptr_si_);
		firstpass_ = false;
	}
	else
		ptr_ikSolver_->track(*ptr_si_);
}

void InvKinMarker::setMarkerRef(const std::map<string, std::vector<double> >& markerToPosition)
{
	// Transform the map of marker position into an SimTK array of marker position.
	SimTK::Array_<SimTK::Vec3> markerGoal;
	SimTK::Vec3 markerPosSimTK;

	// markerNames_ have to have the same order as the OpenSim model.s
	for (std::vector<std::string>::const_iterator it1 = markerNames_.begin(); it1 != markerNames_.end(); it1++)
	{
		const std::vector<double>& markerPos = markerToPosition.at(*it1);
		markerPosSimTK.set(0, markerPos[0]);
		markerPosSimTK.set(1, markerPos[1]);
		markerPosSimTK.set(2, markerPos[2]);
		markerGoal.push_back(markerPosSimTK);
	}

	// Set the marker position in the OpenSim IK.
	ptr_ikSolver_->setMarkerGoal(markerGoal);
}

