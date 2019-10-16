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

#include "CommonCEINMS.h"
#include "InvKinInvDynRealTime.h"
#include "MotionCapturePreprocessing.h"
#include <OpenSim/OpenSim.h>
#include "RTProtocol.h"
#pragma warning( pop )

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <limits.h>
#include <iomanip>
#include <sstream>
#include <ctime>

#ifndef QUALISYSCLIENT_H_
#define QUALISYSCLIENT_H_

/**
 * Class for connecting to Qualisys motion capture system and gets information like force plate and markers position.
 */

class QualisysClient
{
	public:

		/**
		 * Default constructor
		 */
		QualisysClient();

		/**
		 * Constructor
		 * @param serverAddress IP address of the computer
		 * @param basePort port of the Qualisys motion capture software
		 */
		QualisysClient ( const std::string& serverAddress, int basePort );

		/**
		 * Destructor
		 */
		virtual ~QualisysClient();

		/**
		 * Initialization method for connecting to the Qualisys software.
		 */
		void init ( const string& labXml );

		/**
		 * Receive Data from the Qualisys software
		 */
		bool receiveData();

		/**
		 * Get the marker data
		 * @return An array of vector containing the position (x, y, z) of the markers.
		 */
		inline const SimTK::Array_<SimTK::fVec3>& getDataUnfiltered()
		{
			return markerPos_;
		}
		
		inline const SimTK::Array_<SimTK::fVec3>& getDatafiltered()
		{
			_motionCapturePreProcessing->ComputeMarker(markerPos_);
			return markerPos_;
		}

		inline const SimTK::Array_<SimTK::fVec9>& getDataForcePlateUnfiltered()
		{
			_grfData.clear();
			for ( int i = 0; i < forcePlateCount_; i++ )
			{
				SimTK::fVec9 grf;
				grf[0] = _motionCapturePreProcessing->getForcesUnFiltered()[i][0];
				grf[1] = _motionCapturePreProcessing->getForcesUnFiltered()[i][1];
				grf[2] = _motionCapturePreProcessing->getForcesUnFiltered()[i][2];
				grf[3] = _motionCapturePreProcessing->getTorquesUnFiltered()[i][0];
				grf[4] = _motionCapturePreProcessing->getTorquesUnFiltered()[i][1];
				grf[5] = _motionCapturePreProcessing->getTorquesUnFiltered()[i][2];
				grf[6] = _motionCapturePreProcessing->getCOFUnFiltered()[i][0];
				grf[7] = _motionCapturePreProcessing->getCOFUnFiltered()[i][1];
				grf[8] = _motionCapturePreProcessing->getCOFUnFiltered()[i][2];
				_grfData.push_back(grf);
			}
			return _grfData;
		}
		
		inline std::vector<SimTK::Array_<SimTK::fVec9> > getDataForcePlateUnfilteredVect()
		{
			std::vector<SimTK::Array_<SimTK::fVec9> > grfData;
			for ( int v = 0; v < _motionCapturePreProcessing->getCOFUnFilteredVect().size(); v ++)
			{
				SimTK::Array_<SimTK::fVec9> grfArray;
				for ( int i = 0; i < forcePlateCount_; i++ )
				{
					SimTK::fVec9 grf;
					grf[0] = _motionCapturePreProcessing->getForcesUnFilteredVect()[v][i][0];
					grf[1] = _motionCapturePreProcessing->getForcesUnFilteredVect()[v][i][1];
					grf[2] = _motionCapturePreProcessing->getForcesUnFilteredVect()[v][i][2];
					grf[3] = _motionCapturePreProcessing->getTorquesUnFilteredVect()[v][i][0];
					grf[4] = _motionCapturePreProcessing->getTorquesUnFilteredVect()[v][i][1];
					grf[5] = _motionCapturePreProcessing->getTorquesUnFilteredVect()[v][i][2];
					grf[6] = _motionCapturePreProcessing->getCOFUnFilteredVect()[v][i][0];
					grf[7] = _motionCapturePreProcessing->getCOFUnFilteredVect()[v][i][1];
					grf[8] = _motionCapturePreProcessing->getCOFUnFilteredVect()[v][i][2];
					grfArray.push_back(grf);
				}
				grfData.push_back(grfArray);
			}
			_motionCapturePreProcessing->resetGRFUnFiltVect();
			return grfData;
		}
		
		inline const SimTK::Array_<SimTK::fVec9>& getDataForcePlatefiltered()
		{
			_grfDataFiltered.clear();
			for ( int i = 0; i < forcePlateCount_; i++ )
			{
				SimTK::fVec9 grf;
				grf[0] = _motionCapturePreProcessing->getForcesFiltered()[i][0];
				grf[1] = _motionCapturePreProcessing->getForcesFiltered()[i][1];
				grf[2] = _motionCapturePreProcessing->getForcesFiltered()[i][2];
				grf[3] = _motionCapturePreProcessing->getTorquesFiltered()[i][0];
				grf[4] = _motionCapturePreProcessing->getTorquesFiltered()[i][1];
				grf[5] = _motionCapturePreProcessing->getTorquesFiltered()[i][2];
				grf[6] = _motionCapturePreProcessing->getCOFFiltered()[i][0];
				grf[7] = _motionCapturePreProcessing->getCOFFiltered()[i][1];
				grf[8] = _motionCapturePreProcessing->getCOFFiltered()[i][2];
				_grfDataFiltered.push_back(grf);
			}
			return _grfDataFiltered;
		}
		
		inline std::vector<SimTK::Array_<SimTK::fVec9> > getDataForcePlatefilteredVect()
		{
			std::vector<SimTK::Array_<SimTK::fVec9> > grfData;
			for ( int v = 0; v < _motionCapturePreProcessing->getCOFFilteredVect().size(); v ++)
			{
				SimTK::Array_<SimTK::fVec9> grfArray;
				for ( int i = 0; i < forcePlateCount_; i++ )
				{
					SimTK::fVec9 grf;
					grf[0] = _motionCapturePreProcessing->getForcesFilteredVect()[v][i][0];
					grf[1] = _motionCapturePreProcessing->getForcesFilteredVect()[v][i][1];
					grf[2] = _motionCapturePreProcessing->getForcesFilteredVect()[v][i][2];
					grf[3] = _motionCapturePreProcessing->getTorquesFilteredVect()[v][i][0];
					grf[4] = _motionCapturePreProcessing->getTorquesFilteredVect()[v][i][1];
					grf[5] = _motionCapturePreProcessing->getTorquesFilteredVect()[v][i][2];
					grf[6] = _motionCapturePreProcessing->getCOFFilteredVect()[v][i][0];
					grf[7] = _motionCapturePreProcessing->getCOFFilteredVect()[v][i][1];
					grf[8] = _motionCapturePreProcessing->getCOFFilteredVect()[v][i][2];
					grfArray.push_back(grf);
				}
				grfData.push_back(grfArray);
			}
			_motionCapturePreProcessing->resetGRFFiltVect();
			return grfData;
		}

		/**
		 * Get the frame Number
		 */
		inline const unsigned int& getFrameNumber()
		{
			return frameNumber_;
		}

		/**
		 * Get the numbers of markers received
		 */
		inline const int& getNumbersOfMarkersReceived()
		{
			return markerCount_;
		}

		/**
		 * Get the numbers of force plates received
		 */
		inline const unsigned int& getNumberOfForcePlate()
		{
			return _motionCapturePreProcessing->getNbOfPlate();
		}

		inline void quit()
		{
			
		}

		inline void setVerbose ( int verbose )
		{
			_verbose = verbose;
		}

		inline void setMarkerNames ( const std::vector<std::string>& markerNames )
		{
			_markerNames = markerNames;
			last_markerSet.resize(_markerNames.size());
			for (int i = 0; i < _markerNames.size(); i++)
			{
				last_markerSet.at(i).setToZero();
			}

		}
		
		inline void InitMarkerFilter(const std::vector<float>& aCoeff, const std::vector<float>& bCoeff, const std::vector<std::vector<std::vector<float> > >& initValue)
		{
			_motionCapturePreProcessing->InitMarkerFilter(aCoeff, bCoeff, initValue);
		}
	
		inline void InitGRFFilter(const std::vector<float>& aCoeff, const std::vector<float>& bCoeff, const std::vector<std::vector<std::vector<float> > >& initValue)
		{
			_initFilter = true;
			_motionCapturePreProcessing->InitGRFFilter(aCoeff, bCoeff, initValue);
		}

		inline void set_last_marker(SimTK::fVec3& marker, int index)
		{
			last_markerSet.at(index)[0] = marker[0];
			last_markerSet.at(index)[1] = marker[1];
			last_markerSet.at(index)[2] = marker[2];
			
		}

		inline SimTK::fVec3 get_last_marker(int index)
		{
			SimTK::fVec3 marker;
			marker[0] = last_markerSet.at(index)[0];
			marker[1] = last_markerSet.at(index)[1];
			marker[2] = last_markerSet.at(index)[2];
			return marker;
		}

		inline double get_weight(int index)
		{
			return markers_weight.at(index);
		}

		inline void set_weights(std::vector<double>& weights)
		{
			markers_weight = weights;
		}

	protected:
		SimTK::Array_<SimTK::fVec3> markerPos_; //!< SimTK array of vector containing the position (x, y, z) of the markers.
		SimTK::Array_<SimTK::fVec3> _markerPosRotate; //!< SimTK array of vector containing the position (x, y, z) of the markers from the frames N-1.
		SimTK::Array_<SimTK::fVec9> _grfData; //!< SimTK array of vector containing the forces (Fx, Fy, Fz), position (x, y, z) and torque (Tx, Ty, Tz).
		SimTK::Array_<SimTK::fVec9> _grfDataFiltered; //!< SimTK array of vector containing the forces (Fx, Fy, Fz), position (x, y, z) and torque (Tx, Ty, Tz).
		string serverAddress_; //!< IP address
		int basePort_; //!< Port
		CRTPacket* pRTPacket_; //!< Packet containing the data
		CRTPacket::EPacketType eType_; //!< Type of packet
		CRTProtocol poRTProtocol_; //!< Class for the communication with the Qualisys software.
		unsigned int frameNumber_; //!< Number of frames
		int markerCount_; //!< Number of marker received
		int forcePlateCount_; //!< Number of forces plates received
		int forceCount_; //!< Number of forces received
		bool firstPass_;
		MotionCapturePreprocessing* _motionCapturePreProcessing; //!< Class for pre-processsing the data from the force plates and the markers
		bool _initFilter;
		int _verbose;
		std::vector<std::string> _markerNames;

		std::vector<SimTK::fVec3>last_markerSet; //----------------------------- NaN Value
		std::vector<double> markers_weight;
};

#endif /* QUALISYSCLIENT_H_ */
