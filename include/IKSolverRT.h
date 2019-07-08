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

#ifndef IKSOLVERRT_H_
#define IKSOLVERRT_H_

#include "InvKinInvDynRealTime.h"
#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/CoordinateReference.h>
#include <OpenSim/Simulation/Solver.h>

/**
 * IK solver for real-time use inspired from the OpenSim basic IK solver (AssemblySolver.h).
 */

class IKSolverRT: public OpenSim::Solver
{
OpenSim_DECLARE_CONCRETE_OBJECT(IKSolverRT, Solver)
	;
public:
	/*
	 * Constructor
	 */
	IKSolverRT(const OpenSim::Model &model, const std::vector<std::string>& markerNames,
			const std::vector<double>& markerWeights, double constraintWeight = SimTK::Infinity);

	/**
	 * Destructor
	 */
	virtual ~IKSolverRT();

	/**
	 * Copy operator
	 */
	IKSolverRT& operator=(IKSolverRT& ikSolverRT)
	{
		swap(*this, ikSolverRT);
		return *this;
	}

	/**
	 * Set marker position.
	 * @param markerGoal Array of marker position with the order of the OpenSim model.
	 */
	inline void setMarkerGoal(const SimTK::Array_<SimTK::Vec3>& markerGoal)
	{
		markerValues_ = markerGoal;
	}

	/** Assemble a model configuration that meets the assembly conditions
	 * (desired values and constraints) starting from an initial state that
	 * does not have to satisfy the constraints.
	 *
	 * Taken from AssemblySolver.h
	 */
	virtual void assemble(SimTK::State &s);

	/**
	 * Obtain a model configuration that meets the assembly conditions
	 * (desired values and constraints) given a state that satisfies or
	 * is close to satisfying the constraints. Note there can be no change
	 * in the number of constrainst or desired coordinates. Desired
	 * coordinate values can and should be updated between repeated calls
	 * to track a desired trajectory of coordinate values.
	 *
	 * Taken from AssemblySolver.h
	 */
	virtual void track(SimTK::State &s);

	/**
	 * Set the accuracy of the IK.
	 */
	inline void setAccuracy(double accuracy)
	{
		accuracy_ = accuracy;
	}

	/**
	 * Set enforced constraint (respect the joint limit).
	 */
	inline void setEnforceContraint(bool enforceContraint)
	{
		enforceContraint_ = enforceContraint;
	}

protected:
	/** Internal method to convert the CoordinateReferences into goals of the
	 * assembly solver. Subclasses, can add override  to include other goals
	 * such as point of interest matching (Marker tracking). This method is
	 * automatically called by assemble.
	 * */
	virtual void setupGoals(SimTK::State &s);

	/** Internal method to update the time, reference values and/or their
	 * weights that define the goals, based on the passed in state
	 */
	virtual void updateGoals(const SimTK::State &s);

	/**
	 * Swap for copy operator.
	 */
	friend void swap(IKSolverRT& first, IKSolverRT& second)
	{
		using std::swap;

		swap(first.assembler_, second.assembler_);
		swap(first.accuracy_, second.accuracy_);
		swap(first.constraintWeight_, second.constraintWeight_);
		swap(first.markerAssemblyCondition_, second.markerAssemblyCondition_);
		swap(first.markerNames_, second.markerNames_);
		swap(first.markerWeights_, second.markerWeights_);
		swap(first.markerValues_, second.markerValues_);
	}

	SimTK::Assembler* assembler_; //!< SimTK assembler for solving the IK problem.
	double accuracy_; //!< Accuracy of the IK.
	double constraintWeight_; //!< Assembler constraint weight.
	SimTK::Markers* markerAssemblyCondition_; //!< Marker goal.
	std::vector<std::string> markerNames_; //!< Vector of the names of the markes.
	std::vector<double> markerWeights_; //!< Vector of the markers weight.
	SimTK::Array_<SimTK::Vec3> markerValues_; //!< Position of the marker in a x, y and z format.
	bool enforceContraint_; //!< enforce the constraint of the IK (respect of the joint limit).
};

#endif /* IKSOLVERRT_H_ */
