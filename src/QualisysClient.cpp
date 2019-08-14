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

#include "QualisysClient.h"

QualisysClient::QualisysClient() :
	serverAddress_ ( "10.76.115.143" ), basePort_ ( 22222 ), _initFilter(false)
{
	_verbose = 2;
}

QualisysClient::QualisysClient ( const std::string& serverAddress, int basePort ) :
	serverAddress_ ( serverAddress ), basePort_ ( basePort ), _initFilter(false)
{
	_verbose = 1;
}

QualisysClient::~QualisysClient()
{
 	poRTProtocol_.StopCapture();
	poRTProtocol_.StreamFramesStop();
	poRTProtocol_.CloseMeasurement();
	poRTProtocol_.Disconnect();
	delete _motionCapturePreProcessing;
}

void QualisysClient::init ( const string& labXml )
{
	// Create the force plate and markers pre-processing class and read the configuration in the XML file
	_motionCapturePreProcessing = new MotionCapturePreprocessing ( labXml );
	_motionCapturePreProcessing->setNumberOfMarker(_markerNames.size());
	_motionCapturePreProcessing->readXML();

	COUT << "Connecting to the Qualisys Motion Tracking system specified at: " << serverAddress_ << ":" << basePort_
			<< endl;

	// Connect to the Qualisys software
	if ( !poRTProtocol_.Connect ( ( char* ) serverAddress_.data(), basePort_, 0, 1, 7 ) )
	{
		COUT << "Could not find the Qualisys Motion Tracking system at: " << serverAddress_ << ":" << basePort_ << endl;
		exit ( 1 );
	}

	firstPass_ = true;

}

bool QualisysClient::receiveData()
{
	// Get the packet
	pRTPacket_ = poRTProtocol_.GetRTPacket();

	// Get the frame number
	frameNumber_ = pRTPacket_->GetFrameNumber();

	// Get the markers data
	poRTProtocol_.GetCurrentFrame ( CRTProtocol::cComponent3d );

	SimTK::fVec3 marker;
	SimTK::fVec3 force;
	SimTK::fVec3 torque;
	SimTK::fVec3 postition;
	markerPos_.clear();

	// Test if we receive marker data
	if ( poRTProtocol_.ReceiveRTPacket ( eType_, true ) != NULL )
	{
		switch ( eType_ )
		{
				// Process if we have an error
			case CRTPacket::PacketError:
				COUT << "Error when streaming frames: " << poRTProtocol_.GetRTPacket()->GetErrorString() << endl;
				exit ( 1 );
				break;

				// Process if we don't have anymore data to process
			case CRTPacket::PacketNoMoreData:  // No more data
				COUT << "No more data" << endl;
				exit ( 0 );
				break;

				// Process if we received markers data
			case CRTPacket::PacketData:

				// Get the numbers of markers received
				markerCount_ = pRTPacket_->Get3DMarkerCount();

				// If we received one or more markers
				if ( markerCount_ <= 0 )
				{
					COUT << "No marker Found" << endl;
					exit ( 0 );
				}
				else
				{
					for ( int i = 0; i < markerCount_; i++ )
					{
						// Get the position of the marker
						pRTPacket_->Get3DMarker ( i, marker[0], marker[1], marker[2] );

						// If the data is no good exit
						if ( _verbose > 1 )
							if ( isnan ( marker[0] ) || isnan ( marker[1] ) || isnan ( marker[2] ) )
								COUT << "Marker " << _markerNames.at ( i ) << " not detected." << std::endl;
							
						// Change to meters
						marker = marker / 1000;
							
						markerPos_.push_back ( marker );

						// Change the referential to OpenSim referential
						marker = _motionCapturePreProcessing->getMarkerRotation() * marker;

						// Save the postion
						_markerPosRotate.push_back ( marker );
					}
				}

				break;

			default:
				COUT << "Unknown CRTPacket case" << endl;
		}
	}
	else
		return false;

	if ( _motionCapturePreProcessing->getNbOfPlate() > 0 )
	{
		// Get the data for the force plate
		poRTProtocol_.GetCurrentFrame ( CRTProtocol::cComponentForce );

		// Test if we receive force plate data
		if ( poRTProtocol_.ReceiveRTPacket ( eType_, true ) != NULL )
		{
			switch ( eType_ )
			{
					// Process if we have an error
				case CRTPacket::PacketError:
					COUT << "Error when streaming frames: " << poRTProtocol_.GetRTPacket()->GetErrorString() << endl;
					exit ( 1 );
					break;

					// Process if we don't have anymore data to process
				case CRTPacket::PacketNoMoreData:  // No more data
					COUT << "No more data" << endl;
					exit ( 0 );
					break;

					// Process if we received force plate data
				case CRTPacket::PacketData:
					forcePlateCount_ = pRTPacket_->GetForcePlateCount();

					// If we don't have force plate in the system
					if ( forcePlateCount_ <= 0 )
					{
						COUT << "No Plate Found" << endl;
						exit ( 0 );
					}
					else
					{
// 						_motionCapturePreProcessing->resetGRF();
						for ( int i = 0; i < forcePlateCount_; i++ )
						{
							// Get the numbers of force received
							forceCount_ = pRTPacket_->GetForceCount ( i );
							
// 							COUT << "forceCount_: " << forceCount_ << std::endl << std::flush;
// 							COUT << "GetForceSinglePlateCount: " << pRTPacket_->GetForceSinglePlateCount() << std::endl << std::flush;
							

							// Get the force plate data for the plate i
							for ( int j = 0; j < forceCount_; j++ )
							{
								CRTPacket::SForce forcePacket;
								pRTPacket_->GetForceData ( i, 0, forcePacket );
								force[0] = forcePacket.fForceX;
								force[1] = forcePacket.fForceY;
								force[2] = forcePacket.fForceZ;
								torque[0] = forcePacket.fMomentX;
								torque[1] = forcePacket.fMomentY;
								torque[2] = forcePacket.fMomentZ;
								postition[0] = forcePacket.fApplicationPointX;
								postition[1] = forcePacket.fApplicationPointY;
								postition[2] = forcePacket.fApplicationPointZ;
								if(_initFilter) //We should maybe add a test to compare the number of force plate from XML and the one from Qualisys
									_motionCapturePreProcessing->ComputeForcePlate ( i, force, torque, postition );
							}
						}
					}

					break;

				default:
					COUT << "Unknown CRTPacket case" << endl;
			}
		}
		else
			return false;
	}
	return true;
}
