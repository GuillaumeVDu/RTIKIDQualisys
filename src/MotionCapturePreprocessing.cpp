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

#include "MotionCapturePreprocessing.h"

MotionCapturePreprocessing::MotionCapturePreprocessing ( const string& labXml ) :
	nbOfForcePlate_ ( 0 )
{
	try
	{
		std::auto_ptr<LaboratoryType> laboratoryPointer ( Laboratory ( labXml ) );
		laboratory_ = laboratoryPointer;
	}
	catch ( const xml_schema::exception& e )
	{
		cout << e << endl;
		exit ( EXIT_FAILURE );
	}
}

MotionCapturePreprocessing::~MotionCapturePreprocessing()
{
	for ( int i = 0; i < _nbOfMarker; i++ )
		for ( int j = 0; j < 3; j++ )
			delete _markerfFilter[i][j];

	for ( int i = 0; i < nbOfForcePlate_; i++ )
		for ( int j = 0; j < 6; j++ )
			delete _grffFilter[i][j];
}

void MotionCapturePreprocessing::readXML()
{
	// Rotation for the marker from the xml file.
	const LaboratoryType::CoordinateSystemOrientation_type& coordinateSystemOrientation =
		laboratory_->CoordinateSystemOrientation();

	if ( coordinateSystemOrientation == "XYZ" )
		markerRotation_.setRotationToIdentityMatrix();
	else if ( coordinateSystemOrientation == "XZY" )
		markerRotation_ = SimTK::Rotation_<double> ( SimTK::convertDegreesToRadians ( -90 ),
				SimTK::CoordinateAxis::XCoordinateAxis() );
	else if ( coordinateSystemOrientation == "ZYX" )
		markerRotation_ = SimTK::Rotation_<double> ( SimTK::convertDegreesToRadians ( 90 ),
				SimTK::CoordinateAxis::YCoordinateAxis() );
	else if ( coordinateSystemOrientation == "ZXY" )
		markerRotation_.setRotationFromTwoAnglesTwoAxes ( SimTK::SpaceRotationSequence,
				SimTK::convertDegreesToRadians ( 90 ), SimTK::CoordinateAxis::XCoordinateAxis(),
				SimTK::convertDegreesToRadians ( 90 ), SimTK::CoordinateAxis::ZCoordinateAxis() );
	else if ( coordinateSystemOrientation == "YXZ" )
		markerRotation_.setRotationFromTwoAnglesTwoAxes ( SimTK::SpaceRotationSequence,
				SimTK::convertDegreesToRadians ( 90 ), SimTK::CoordinateAxis::ZCoordinateAxis(),
				SimTK::convertDegreesToRadians ( -180 ), SimTK::CoordinateAxis::YCoordinateAxis() );
	else if ( coordinateSystemOrientation == "YZX" )
		markerRotation_.setRotationFromTwoAnglesTwoAxes ( SimTK::SpaceRotationSequence,
				SimTK::convertDegreesToRadians ( -90 ), SimTK::CoordinateAxis::XCoordinateAxis(),
				SimTK::convertDegreesToRadians ( -90 ), SimTK::CoordinateAxis::YCoordinateAxis() );
	else if ( coordinateSystemOrientation == "-XZY" )
		markerRotation_.setRotationFromTwoAnglesTwoAxes ( SimTK::SpaceRotationSequence,
				SimTK::convertDegreesToRadians ( -90 ), SimTK::CoordinateAxis::XCoordinateAxis(),
				SimTK::convertDegreesToRadians ( 180 ), SimTK::CoordinateAxis::YCoordinateAxis() );
	else
	{
		cout << "Rotation not implemented." << endl;
		exit ( 1 );
	}

	nbOfForcePlate_ = laboratory_->NumberOfForcePlatforms();

	// Computation of the rotation for the forces plate.
	typedef LaboratoryType::ForcePlatformsList_sequence ForcePlatSeq;
	typedef LaboratoryType::ForcePlatformsList_type::ForcePlatform_sequence ForcePlatSeqSeq;
	typedef LaboratoryType::ForcePlatformsList_type::ForcePlatform_type::FPtoGlobalRotations_type FPGType;
	const ForcePlatSeq& forcePlatSeq = laboratory_->ForcePlatformsList();


	for ( ForcePlatSeq::const_iterator it = forcePlatSeq.begin(); it != forcePlatSeq.end(); it++ )
	{
		for ( ForcePlatSeqSeq::const_iterator it2 = it->ForcePlatform().begin(); it2 != it->ForcePlatform().end(); it2++ )
		{
			const ForcePlatformOrigin& origin = it2->origin();
			forcePlateOrigin_.push_back ( SimTK::fVec3 ( origin.x(), origin.y(), origin.z() ) );
			const ForcePlatformOrigin& originGlobal = it2->originGlobal();
			forcePlateCenterGlobal_.push_back ( SimTK::fVec3 ( originGlobal.x(), originGlobal.y(), originGlobal.z() ) );
			forcePlateTypes_.push_back ( it2->Type() );
			const FPGType& globalRot = it2->FPtoGlobalRotations();


// 			_h.push_back ( ( originGlobal.z() - origin.z() ) / 1000 );

			if ( globalRot.Rot().size() == 2 )
			{
				SimTK::Rotation_<double> rot;
				SimTK::CoordinateAxis axisCoord1 ( 1 );
				SimTK::CoordinateAxis axisCoord2 ( 1 );
				const string& axis1 = globalRot.Rot() [0].Axis();
				const double& deg1 = globalRot.Rot() [0].Degrees();
				const string& axis2 = globalRot.Rot() [1].Axis();
				const double& deg2 = globalRot.Rot() [1].Degrees();

				if ( axis1 == "X" )
					axisCoord1 = SimTK::CoordinateAxis::XCoordinateAxis();
				else if ( axis1 == "Y" )
					axisCoord1 = SimTK::CoordinateAxis::YCoordinateAxis();
				else if ( axis1 == "Z" )
					axisCoord1 = SimTK::CoordinateAxis::ZCoordinateAxis();
				else
				{
					cout << "Axis 1 for force plate not good." << endl;
					exit ( 1 );
				}

				if ( axis2 == "X" )
					axisCoord2 = SimTK::CoordinateAxis::XCoordinateAxis();
				else if ( axis2 == "Y" )
					axisCoord2 = SimTK::CoordinateAxis::YCoordinateAxis();
				else if ( axis2 == "Z" )
					axisCoord2 = SimTK::CoordinateAxis::ZCoordinateAxis();
				else
				{
					cout << "Axis 2 for force plate not good." << endl;
					exit ( 1 );
				}

				rot.setRotationFromTwoAnglesTwoAxes ( SimTK::SpaceRotationSequence, SimTK::convertDegreesToRadians ( deg1 ),
						axisCoord1, SimTK::convertDegreesToRadians ( deg2 ), axisCoord2 );
				forcePlateRotation_.push_back ( rot );

			}
			else if ( globalRot.Rot().size() == 1 )
			{
				SimTK::Rotation_<double> rot;
				SimTK::CoordinateAxis axisCoord ( 1 );
				const string& axis1 = globalRot.Rot() [0].Axis();
				const double& deg1 = globalRot.Rot() [0].Degrees();

				if ( axis1 == "X" )
					axisCoord = SimTK::CoordinateAxis::XCoordinateAxis();
				else if ( axis1 == "Y" )
					axisCoord = SimTK::CoordinateAxis::YCoordinateAxis();
				else if ( axis1 == "Z" )
					axisCoord = SimTK::CoordinateAxis::ZCoordinateAxis();
				else
				{
					cout << "Axis for force plate not good." << endl;
					exit ( 1 );
				}

				rot = SimTK::Rotation_<double> ( SimTK::convertDegreesToRadians ( - deg1 ), axisCoord );
				forcePlateRotation_.push_back ( rot );
			}
			else
			{
				SimTK::Rotation_<double> identity;
				identity.setRotationToIdentityMatrix();
				forcePlateRotation_.push_back ( identity );
			}
		}
	}

// 	for ( int i = 0; i < forcePlateRotation_.size(); i++ )
// 		forcePlateCenterGlobalRotated_.push_back ( SimTK::fVec3 ( markerRotation_ * forcePlateCenterGlobal_[i] ) );

	_grffFilter.resize ( nbOfForcePlate_ );
	_markerfFilter.resize ( _nbOfMarker );
	_forces.resize ( nbOfForcePlate_ );
	_cof.resize ( nbOfForcePlate_ );
	_torques.resize ( nbOfForcePlate_ );
	_forcesFilter.resize ( nbOfForcePlate_ );
	_cofFilter.resize ( nbOfForcePlate_ );
	_torquesFilter.resize ( nbOfForcePlate_ );

	for ( int i = 0; i < nbOfForcePlate_; i++ )
	{
		_forces.at ( i ).setToZero();
		_cof.at ( i ).setToZero();
		_torques.at ( i ).setToZero();
		_forcesFilter.at ( i ).setToZero();
		_cofFilter.at ( i ).setToZero();
		_torquesFilter.at ( i ).setToZero();
	}
}

void MotionCapturePreprocessing::InitMarkerFilter ( const std::vector<float>& aCoeff, const std::vector<float>& bCoeff, const std::vector<std::vector<std::vector<float> > >& initValue )
{
	for ( int i = 0; i < _nbOfMarker; i++ )
		for ( int j = 0; j < 3; j++ )
			_markerfFilter.at ( i ).push_back ( new fFilter ( aCoeff, bCoeff, initValue.at ( i ).at ( j ) ) );
}

void MotionCapturePreprocessing::InitGRFFilter ( const std::vector<float>& aCoeff, const std::vector<float>& bCoeff, const std::vector<std::vector<std::vector<float> > >& initValue )
{
	for ( int i = 0; i < nbOfForcePlate_; i++ )
		for ( int j = 0; j < 6; j++ )
			_grffFilter.at ( i ).push_back ( new fFilter ( aCoeff, bCoeff, initValue.at ( i ).at ( j ) ) );
}

void MotionCapturePreprocessing::ComputeMarker ( SimTK::Array_<SimTK::fVec3>& markerData )
{
	for ( std::vector<std::vector<fFilter*> >::iterator it1 = _markerfFilter.begin();
			it1 < _markerfFilter.end(); it1++ )
	{
		const int& cpt1 = std::distance<std::vector<std::vector<fFilter*> >::iterator > ( _markerfFilter.begin(), it1 );

		for ( std::vector<fFilter*>::iterator it2 = it1->begin();  it2 < it1->end(); it2++ )
		{
			const int& cpt2 = std::distance<std::vector<fFilter*>::iterator > ( it1->begin(), it2 );
			markerData[cpt1][cpt2] = ( *it2 )->filter ( markerData[cpt1][cpt2] );
		}

		rotateQualysisFrames ( markerData[cpt1], markerData[cpt1] );
	}
}

void MotionCapturePreprocessing::ComputeForcePlate(int index, const SimTK::fVec3& force,
	const SimTK::fVec3& torque, const SimTK::fVec3& position)
{

	/*rotateForcePlateFrames(force, _forces.at(index), index);
	rotateForcePlateFrames(torque, _torques.at(index), index);
	rotateForcePlateFrames(position, _cof.at(index), index);*/
	// 	_forces.at ( index ) = force;
	// 	_torques.at ( index ) = torque;
	// 	_cof.at ( index ) = position;
	_cof.at(index) = _cof.at(index) / 1000;
	_cof.at(index)[0] = -_cof.at(index)[0]; // --------------------------as MoToNMS----------------------------------

	std::vector<fFilter*> filterTemp = _grffFilter.at(index);

	char cpt = 0;

	for (int i = 0; i < 3; i++)
	{
		_forcesFilter.at(index)[i] = filterTemp.at(cpt)->filter(force[i]);
		// 		_forcesFilter.at ( index ) [i] = force[i] ;
		cpt++;
	}

	SimTK::fVec3 torquesFiltered;

	for (int i = 0; i < 3; i++)
	{
		torquesFiltered[i] = filterTemp.at(cpt)->filter(torque[i]);
		// 		torquesFiltered[i] =  torque[i];
		cpt++;
	}

	_cofFilter.at(index).setToZero();
	_torquesFilter.at(index).setToZero();

	if (_forcesFilter.at(index)[2] > 5)
	{
		
		// COPx = (-(My + (Z0 - padding).Fx)/Fz)
		_cofFilter.at(index)[0] = (-(torquesFiltered[1] + (forcePlateOrigin_.at(index)[2] / 1000) * _forcesFilter.at(index)[0]) / _forcesFilter.at(index)[2]);

		// COPy = ((Mx - (Z0 - padding).Fy)/Fz)
		_cofFilter.at(index)[1] = ((torquesFiltered[0] - (forcePlateOrigin_.at(index)[2] / 1000) * (_forcesFilter.at(index)[1])) / _forcesFilter.at(index)[2]);

		// 		_cofFilter.at ( index ) = - _cofFilter.at ( index );

		//-----------------------------------------------------------------------------
		// the Force_y (_forcesFilter.at ( index ) [1]) is inverted to make the torque coincide with the one in MoToNMS
		//-----------------------------------------------------------------------------

		// Tz = Mz - (( COPx ) * Fy) + (( COPy ) * Fx)
		_torquesFilter.at ( index ) [2] = torquesFiltered[2] - ( ( _cofFilter.at ( index ) [0] ) *  (- _forcesFilter.at ( index ) [1]) )
				+ ( ( _cofFilter.at ( index ) [1] ) * _forcesFilter.at ( index ) [0] );

// 		_torquesFilter.at ( index ) [2] = _torquesFilter.at ( index ) [2] / 1000;
		_cofFilter.at ( index ) = - _cofFilter.at ( index );
		rotateForcePlateFrames ( _cofFilter.at ( index ), _cofFilter.at ( index ), index );

		_cofFilter.at(index)[1] = 0.0; //the COP_Y has to be zero
				
		if ( forcePlateTypes_.at ( index ) == 3 )
		{
			_cofFilter.at ( index ) [0] = _cofFilter.at ( index ) [0] + forcePlateCenterGlobal_.at ( index ) [0] / 1000;
			_cofFilter.at ( index ) [1] = _cofFilter.at ( index ) [1] + forcePlateCenterGlobal_.at ( index ) [1] / 1000;
		}
		else
		{
			_cofFilter.at ( index ) [0] = _cofFilter.at ( index ) [0] + ( forcePlateCenterGlobal_.at ( index ) [0] + forcePlateOrigin_.at ( index ) [0] ) / 1000;
			_cofFilter.at ( index ) [1] = _cofFilter.at ( index ) [1] + ( forcePlateCenterGlobal_.at ( index ) [1] + forcePlateOrigin_.at ( index ) [1] ) / 1000;
			_cofFilter.at ( index ) [2] = _cofFilter.at ( index ) [2] + ( forcePlateCenterGlobal_.at ( index ) [2]) / 1000;
		}
		
		rotateQualysisFrames(_cofFilter.at ( index ), _cofFilter.at ( index ));

		rotateForcePlateThenQualysisFrames ( _forcesFilter.at ( index ), _forcesFilter.at ( index ), index );
		rotateForcePlateThenQualysisFrames ( _torquesFilter.at ( index ), _torquesFilter.at ( index ), index );
		
		
	}
	
	_forcesVect.push_back(_forces);
	_cofVect.push_back(_cof);
	_torquesVect.push_back(_torques);
	_forcesFilterVect.push_back(_forcesFilter);
	_cofFilterVect.push_back(_cofFilter);
	_torquesFilterVect.push_back(_torquesFilter);

}

