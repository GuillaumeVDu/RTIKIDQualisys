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

#include "XMLInterpreter.h"

XMLInterpreter::XMLInterpreter(const string& xmlFileName) :
		idUse_(false), ikMarkerUse_(false), maxMarkerError_(0.001), visualizerUse_(
				false), enfConstUse_(false), r_(1), p_(1), sigma_da_(1), externalLoadUse_(false) {
	try {
#ifdef DONT_INITIALIZE
		std::auto_ptr<ExecutionIKType> executionPointer(executionIK(xmlFileName));
#pragma message ( "XML file open with xml_schema::flags::dont_initialize" )
#endif
#ifndef DONT_INITIALIZE
		std::auto_ptr<ExecutionIKType> executionPointer(executionIK(xmlFileName));
#endif
		ptr_execution_ = executionPointer;
	} catch (const xml_schema::exception& e) {
		cout << e << endl;
		exit(EXIT_FAILURE);
	}
}

void XMLInterpreter::readXML() {
	try {
		const ExecutionIKType::id_type& idType = ptr_execution_->id();

		// get if we use the ID.
		idUse_ = idType.use();

// 		std::cout << idType.plateForceBody().size() << std::endl;
		// Get the name of the body on which the ground reaction force is applied.
		for(int i = 0; i < idType.plateForceBody().size(); i++)
		{
// 			std::cout << idType.plateForceBody().at(i) << std::endl;
			appliedBody_.push_back(idType.plateForceBody().at(i));
		}

		const ExecutionIKType::ikOption_type& ikOptionType =
				ptr_execution_->ikOption();

		// Get if we use the OpenSim visualizer.
		visualizerUse_ = ikOptionType.visualizer().use();

		// Get if we enforced the IK constraint (joint limits).
		enfConstUse_ = ikOptionType.enforceIKConstraint().use();

		// Get if we use the kalman filter.
		kalmanUse_ = ikOptionType.kalman().use();

		// Get the kalman filter parameters.
		if (kalmanUse_) {
			r_ = ikOptionType.kalman().kalmanOption().r();
			p_ = ikOptionType.kalman().kalmanOption().p();
			sigma_da_ = ikOptionType.kalman().kalmanOption().sigma_da();
			dt_ = ikOptionType.kalman().kalmanOption().dt();
		}

		const ExecutionIKType::ik_type& ikType = ptr_execution_->ik();

		// Get if we use the IMU type IK and get the parameters.
		if (ikType.imus().present()) {
			ikType_ = IMU;
			typedef ExecutionIKType::ik_type::imus_type::imu_sequence IMUSeq;
			const IMUSeq& imuType = ikType.imus().get().imu();
			for (IMUSeq::const_iterator it1 = imuType.begin();
					it1 != imuType.end(); it1++) {
				imuNames_.push_back(it1->name());
				imuBodies_.push_back(it1->body());
			}

			// Get if we use the markers type IK and get the parameters.
		} else if (ikType.markers().present()) {
			ikType_ = Marker;
			typedef ExecutionIKType::ik_type::markers_type::marker_sequence MarkerSeq;
			const MarkerSeq& markerType = ikType.markers().get().marker();
			for (MarkerSeq::const_iterator it1 = markerType.begin();
					it1 != markerType.end(); it1++) {
				markerNames_.push_back(it1->name());
				markerWeights_.push_back(it1->weight());
				maxMarkerError_ = ikType.markers()->maxError();
			}
		}

		// Get the filename of the lab xml.
		labFile_ = ptr_execution_->LabFile();

		// Get the filename of the translation file
		translateFile_ = ptr_execution_->TranslateFile();

		// Get the filename of the Osim file.
		osimFile_ = ptr_execution_->OsimFile();

		// Get the IP for the motion capture system.
		ip_ = ptr_execution_->ip();

		// Get the port for the motion capture system.
		port_ = ptr_execution_->port();

	} catch (const xml_schema::exception& e) {
		cout << e << endl;
		exit(EXIT_FAILURE);
	}

}

XMLInterpreter::~XMLInterpreter() {
}

