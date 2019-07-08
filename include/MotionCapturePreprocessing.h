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

#ifndef MOTIONCAPTUREPREPROCESSING_H_
#define MOTIONCAPTUREPREPROCESSING_H_

#include "InvKinInvDynRealTime.h"
#include "Laboratory.hxx"
#include <Filter.h>
#include <OpenSim/OpenSim.h>

/**
 * Class for the pre-processing of the motion capture system and force plate.
 */

class MotionCapturePreprocessing {
public:

	/*
	 * Constructor
	 */
	MotionCapturePreprocessing(const string& labXml);

	/*
	 * Destructor
	 */
	virtual ~MotionCapturePreprocessing();

	/**
	 * Read the XML file from the lab.XSD.
	 */
	void readXML();

	/**
	 * Get the rotation of the marker for having OpenSim compatibility.
	 */
	inline const SimTK::Rotation_<double>& getMarkerRotation()
	{
		return markerRotation_;
	}

	/**
	 * Get the number of forces plates.
	 */
	inline const unsigned int& getNbOfPlate()
	{
		return nbOfForcePlate_;
	}
	
	inline void setNumberOfMarker (unsigned int nbOfMarker)
	{
		_nbOfMarker = nbOfMarker;
	}
	
	inline const std::vector<SimTK::fVec3>& getForcesFiltered()
	{

		return _forcesFilter;
	}
	
	inline const std::vector<SimTK::fVec3>& getForcesUnFiltered()
	{
		return _forces;
	}
	
	inline const std::vector<SimTK::fVec3>& getCOFFiltered()
	{
		return _cofFilter;
	}
	
	inline const std::vector<SimTK::fVec3>& getCOFUnFiltered()
	{
		return _cof;
	}
	
	inline const std::vector<SimTK::fVec3>& getTorquesFiltered()
	{
		return _torquesFilter;
	}
	
	inline const std::vector<SimTK::fVec3>& getTorquesUnFiltered()
	{
		return _torques;
	}
	
	inline const std::vector<std::vector<SimTK::fVec3> >& getForcesFilteredVect()
	{
		return _forcesFilterVect;
	}
	
	inline const std::vector<std::vector<SimTK::fVec3> >& getForcesUnFilteredVect()
	{
		return _forcesVect;
	}
	
	inline const std::vector<std::vector<SimTK::fVec3> >& getCOFFilteredVect()
	{
		return _cofFilterVect;
	}
	
	inline const std::vector<std::vector<SimTK::fVec3> >& getCOFUnFilteredVect()
	{
		return _cofVect;
	}
	
	inline const std::vector<std::vector<SimTK::fVec3> >& getTorquesFilteredVect()
	{
		return _torquesFilterVect;
	}
	
	inline const std::vector<std::vector<SimTK::fVec3> >& getTorquesUnFilteredVect()
	{
		return _torquesVect;
	}
	
	void InitMarkerFilter(const std::vector<float>& aCoeff, const std::vector<float>& bCoeff, const std::vector<std::vector<std::vector<float> > >& initValue);
	
	void InitGRFFilter(const std::vector<float>& aCoeff, const std::vector<float>& bCoeff, const std::vector<std::vector<std::vector<float> > >& initValue);

	/**
	 * Compute the forces, torques and position of the ground reaction forces.
	 * @param index number of the forces plate
	 * @param force forces vector from the motion capture system.
	 * @param torque torques vector from the motion capture system.
	 * @return the forces, torques and position (in this order) in a vector of 9.
	 */
	void ComputeForcePlate(int index, const SimTK::fVec3& force, const SimTK::fVec3& torque, const SimTK::fVec3& position);
	
	void ComputeMarker( SimTK::Array_<SimTK::fVec3>& markerData);
	
	void resetGRFUnFiltVect()
	{
		_forcesVect.clear();
		_cofVect.clear();
		_torquesVect.clear();
	}
	
	void resetGRFFiltVect()
	{
		_forcesFilterVect.clear();
		_cofFilterVect.clear();
		_torquesFilterVect.clear();
	}

protected:
	std::auto_ptr<LaboratoryType> 			laboratory_;			 	//!< Pointer to the xml information from the xml file.
	SimTK::Rotation_<double>				markerRotation_; 			//!< Rotation of the marker from the motion capture sytem to the OpenSim referential.
	std::vector<SimTK::Rotation_<double> > 	forcePlateRotation_; 		//!< Rotation of the force plate from the motion capture sytem to the OpenSim referential.
	std::vector<int> 						forcePlateTypes_; 			//!< Type of the force plate (see Lab.xsd)
	std::vector<SimTK::fVec3> 				forcePlateOrigin_; 			//!< Force Plate origin
	std::vector<SimTK::fVec3> 				forcePlateCenterGlobal_;  	//!< Force Plate center of pressure
	std::vector<std::vector<fFilter*> > 	_markerfFilter; 			// 3 (X,Y,Z) * nb of marker
	std::vector<std::vector<fFilter*> > 	_grffFilter; 				// 6 (force and torque) * nb of plate
	
	std::vector<SimTK::fVec3> 				_forces;
	std::vector<SimTK::fVec3> 				_cof;
	std::vector<SimTK::fVec3> 				_torques;
	std::vector<SimTK::fVec3> 				_forcesFilter;
	std::vector<SimTK::fVec3> 				_cofFilter;
	std::vector<SimTK::fVec3> 				_torquesFilter;
	
	std::vector<std::vector<SimTK::fVec3> >				_forcesVect;
	std::vector<std::vector<SimTK::fVec3> >				_cofVect;
	std::vector<std::vector<SimTK::fVec3> >				_torquesVect;
	std::vector<std::vector<SimTK::fVec3> >				_forcesFilterVect;
	std::vector<std::vector<SimTK::fVec3> >				_cofFilterVect;
	std::vector<std::vector<SimTK::fVec3> >				_torquesFilterVect;
	unsigned int 							_nbOfMarker;
	unsigned int 							nbOfForcePlate_; 			//!< Number of force plates
	

	
	inline void rotateQualysisFrames(const SimTK::fVec3& vect, SimTK::fVec3& result)
	{
		result = markerRotation_ * vect;
	}
	
	inline void rotateForcePlateFrames(const SimTK::fVec3& vect, SimTK::fVec3& result, int index)
	{
		result = forcePlateRotation_.at(index) * vect;
 		result[0] = -result[0];
	}
	
	inline void rotateForcePlateThenQualysisFrames(const SimTK::fVec3& vect, SimTK::fVec3& result, int index)
	{
		rotateForcePlateFrames(vect, result, index);
		rotateQualysisFrames(result, result);
	}
};

#endif /* MOTIONCAPTUREPREPROCESSING_H_ */
