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

#ifndef INVKIN_H_
#define INVKIN_H_

#include "InvKinInvDynRealTime.h"
#include <OpenSim/OpenSim.h>

/**
 * Virtual class for the inverse kinematic computation.
 */

class InvKin {
public:
	/**
	 * Constructor
	 */
	InvKin(boost::shared_ptr<SimTK::State>& ptr_si, boost::shared_ptr<OpenSim::Model>& ptr_model);

	/**
	 * Destructor
	 */
	virtual ~InvKin();

	/**
	 * Virtual method for the computation of the kinematics
	 */
	virtual void computeKinematics(const std::map<string, std::vector<double> >& positionMap) = 0;

	/**
	 * Virtual method for getting the angle of the joint dofName.
	 */
	virtual double getAngle(const string& dofName) = 0;
protected:
	boost::shared_ptr<SimTK::State>& ptr_si_;
	boost::shared_ptr<OpenSim::Model>& ptr_model_;
};

#endif /* INVKIN_H_ */
