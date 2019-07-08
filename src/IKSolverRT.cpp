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

#include "IKSolverRT.h"

IKSolverRT::IKSolverRT(const OpenSim::Model &model, const std::vector<std::string>& markerNames,
		const std::vector<double>& markerWeights, double constraintWeight) :
		Solver(model), markerNames_(markerNames), markerWeights_(markerWeights), constraintWeight_(constraintWeight), enforceContraint_(
				false)
{
	assembler_ = NULL;
	accuracy_ = 1e-4;
}

IKSolverRT::~IKSolverRT()
{
	delete assembler_;
	//delete markerAssemblyCondition_;
}

void IKSolverRT::setupGoals(SimTK::State &s)
{
	// Setup coordinates performed by the base class

	// Crate the IK solver.
	//delete assembler_;
	assembler_ = new SimTK::Assembler(getModel().getMultibodySystem());

	// Set Accuracy of the IK.
	assembler_->setAccuracy(accuracy_);

	// Set the Assembler weight.
	assembler_->setSystemConstraintsWeight(constraintWeight_);

	// Enforce the constraint limit of each joint, if needed.
	if (enforceContraint_)
	{
		// Get model coordinates.
		const OpenSim::CoordinateSet &modelCoordSet = getModel().getCoordinateSet();

		// Restrict solution to set range of any of the coordinates that are clamped.
		for (int i = 0; i < modelCoordSet.getSize(); ++i)
		{
			const OpenSim::Coordinate& coord = modelCoordSet[i];
			if (coord.getClamped(s))
			{
				assembler_->restrictQ(coord.getBodyIndex(), SimTK::MobilizerQIndex(coord.getMobilizerQIndex()),
						coord.getRangeMin(), coord.getRangeMax());
			}
		}
	}

	// Create the markers for the IK solver.
	markerAssemblyCondition_ = new SimTK::Markers();

	// get markers defined by the model
	const OpenSim::MarkerSet &modelMarkerSet = getModel().getMarkerSet();

	int index = -1;
	// Loop through all markers in the reference.
	for (std::vector<std::string>::const_iterator it1 = markerNames_.begin(); it1 != markerNames_.end(); it1++)
	{
		// Check if we have this marker in the model, else ignore it
		index = modelMarkerSet.getIndex(*it1, index);
		if (index >= 0)
		{
			const OpenSim::Marker &marker = modelMarkerSet.get(*it1);
			const SimTK::MobilizedBody &mobod = getModel().getMatterSubsystem().getMobilizedBody(
					marker.getBody().getIndex());
			markerAssemblyCondition_->addMarker(marker.getName(), mobod, marker.getOffset(),
					markerWeights_[std::distance<std::vector<std::string>::const_iterator>(markerNames_.begin(), it1)]);
		}
	}

	// Add marker goal to the IK objective.
	assembler_->adoptAssemblyGoal(markerAssemblyCondition_);

	// lock-in the order that the observations (markers) are in and this cannot change from frame to frame
	// and we can use an array of just the data for updating
	markerAssemblyCondition_->defineObservationOrder(markerNames_);

	updateGoals(s);
}

void IKSolverRT::updateGoals(const SimTK::State &s)
{
	// update coordinates performed by the base class
	markerAssemblyCondition_->moveAllObservations(markerValues_);
}

void IKSolverRT::assemble(SimTK::State &state)
{
	// Make a working copy of the state that will be used to set the internal state of the solver
	// This is necessary because we may wish to disable redundant constraints, but do not want this
	// to effect the state of constraints the user expects
	SimTK::State s = state;

	// Make sure goals are up-to-date.
	setupGoals(s);

	// Let assembler perform some internal setup
	assembler_->initialize(s);

	try
	{
		// Now do the assembly and return the updated state.
		assembler_->assemble();

		// Update the q's in the state passed in
		assembler_->updateFromInternalState(s);
		state.updQ() = s.getQ();
		state.updU() = s.getU();

		// Get model coordinates
		const OpenSim::CoordinateSet &modelCoordSet = getModel().getCoordinateSet();

		// Make sure the locks in original state are restored
		for (int i = 0; i < modelCoordSet.getSize(); ++i)
		{
			bool isLocked = modelCoordSet[i].getLocked(state);
			if (isLocked)
				modelCoordSet[i].setLocked(state, isLocked);

		}

	} catch (const std::exception& ex)
	{
		std::string msg = "AssemblySolver::assemble() Failed: ";
		msg += ex.what();
		throw OpenSim::Exception(msg);
	}
}

void IKSolverRT::track(SimTK::State &s)
{

	// move the target locations or angles, etc... just do not change number of goals
	// and their type (constrained vs. weighted)

	if (assembler_ && assembler_->isInitialized())
	{
		updateGoals(s);
	}
	else
	{
		throw OpenSim::Exception("AssemblySolver::track() failed: assemble() must be called first.");
	}

	try
	{
		// Now do the assembly and return the updated state.
		assembler_->track(s.getTime());

		// update the state from the result of the assembler
		assembler_->updateFromInternalState(s);

	} catch (const std::exception& ex)
	{
		std::cout << "AssemblySolver::track() attempt Failed: " << ex.what() << std::endl;
		throw OpenSim::Exception("AssemblySolver::track() attempt failed.");
	}
}

