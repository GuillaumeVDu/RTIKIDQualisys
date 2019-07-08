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

#ifndef OSIMIKPLUGIN_H_
#define OSIMIKPLUGIN_H_

#include "InvKinInvDynRealTime.h"
#include "execution.hxx"
#include "ProducersPluginVirtual.h"
#include "QualisysClient.h"
#include "RTIKIDInterface.h"
#include <OpenSim/OpenSim.h>
#include "TranslateOpenSimCEINMS.h"
#include <ctime>
#include <Filter.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

/**
 * Plugin for CEINMS-RT for real-time use of the IK using Qualisys markers
 */

class OsimIKPlugin: public ProducersPluginVirtual
{
	public:
		/**
		 * Constructor
		 */
		OsimIKPlugin();

		/**
		 * Destructor
		 */
		virtual ~OsimIKPlugin();

		/**
		 * Initialization method
		 * @param xmlName Subject specific XMl filename, not use here (we use the osim files)
		 * @param executionName Execution xml filename for CEINMS-RT
		 */
		void init ( string xmlName, string executionName );

		/**
		 * Get the data with the name of the joint mapping the angle
		 */
		const map<string, double>& GetDataMap();

		const double& getTime()
		{
			boost::mutex::scoped_lock lock ( _mutexTimeCurrent );
			return _timeCurrent;
		}

		/**
		 * Get the name of the joint. Warning this a set, so there is not order
		 */
		const set<string>& GetNameSet()
		{
			return _dofNameSet;
		}

		void stop();

		void setDirectory ( std::string outDirectory, std::string inDirectory = std::string() )
		{
			_outDirectory = outDirectory;
			_inDirectory = inDirectory;
		}

		void setVerbose ( int verbose )
		{
			_verbose = verbose;
		}

		void setRecord ( bool record )
		{
			_record = record;
		}

		const map<string, double>& GetDataMapTorque();

	protected:

		/**
		 * boost thread method for the kalman filter with respect to the timing
		 */
		void filterAngle();

		void provider();


		typedef std::vector<std::string> VecStr;
		typedef std::vector<std::vector<double> > VecVecDouble;
		typedef VecVecDouble::const_iterator VecVecDoubleCI;
		typedef VecStr::const_iterator VecStrCI;
		typedef std::vector<std::vector<float> >::const_iterator VecVecFloatCI;
		typedef std::vector<float>::const_iterator VecFloatCI;
		typedef std::vector<double>::const_iterator VecDoubleCI;


		RTIKIDInterface* 		_interface; 			//!< Class for all the IK and kalman computation
		QualisysClient* 		_client; 				//!< Class for connecting to the Qualisys motion capture system
		TranslateOpenSimCEINMS* _translate; 			//!< class for translation between CEINMS name and OpenSim name
		double 					_timeCurrent; 			//!< The current time of the data
		boost::mutex 			_angleDataMutex; 		//!< Mutex for accessing the data
		boost::mutex 			_mutexTimeCurrent;
		std::vector<double> 	_angleData; 			//!< Vector of the angle for passing the data between the thread and the GetDataMap method
		std::vector<double> 	_torqueData; 
		boost::thread* 			_filterThread; 			//!< Pointer to boost thread
		double 					_fixedTimeComputation; 	//!< fixed time computation for the Kalman filter

		bool 							_end;				//!< Boolean for ending all the thread.
		boost::mutex 					_mutexEnd;			//!< Mutex for accessing the bool for ending all thread.

		boost::thread*					_provider;			//!< Boost thread for provider;
		boost::mutex 					_mutexProvider;		//!< Mutex for accesing the providers data.

		SimTK::Array_<SimTK::fVec9> 	_grfData;			//!< GRF data from the providers.
		SimTK::Array_<SimTK::fVec3> 	_markerData;		//!< Marker data from the providers.
		unsigned long 					_framesNumber;		//!< Frames numbers from the providers.
		double 							_timeStamp;			//!< Time stamp from the providers.
		bool 							_providerDataReady;	//!< Boolean for knowing if new data is availible.
		boost::condition_variable 		_condProvider;		//!< Condition for waiting for new data.

		std::ofstream* _IKFile;
		std::ofstream* _IDFile;
		std::ofstream* _TimingFile;
		std::ofstream* _forceFile;
		std::ofstream* _markerFile;
		std::ofstream* _forceFileFilt;
		std::ofstream* _markerFileFilt;

		std::vector<std::vector<fFilter*> > 		_markerfFilter; // 3 (X,Y,Z) * nb of marker
		std::vector<std::vector<fFilter*> > 		_grffFilter; // 6 (force and torque) * nb of plate
		std::vector< std::vector<double> > 			_ikSaveInFile;
		std::vector< std::vector<double> > 			_idSaveInFile;
		std::vector<double> 						_ikTimeSaveInFile;
		std::vector<double> 						_timingSaveInFile;
		std::vector<double> 						_markerTimeSaveInFile;
		std::vector<double> 						_forcePlateTimeSaveInFile;
		std::vector< SimTK::Array_<SimTK::fVec9> > 	_forcePlateSaveInFile;
		std::vector< SimTK::Array_<SimTK::fVec3> > 	_markerSaveInFile;
		std::vector< SimTK::Array_<SimTK::fVec9> > 	_forcePlateFiltSaveInFile;
		std::vector< SimTK::Array_<SimTK::fVec3> > 	_markerFiltSaveInFile;
		std::vector<std::string> 					_dofNameVect;
		std::map<string, double> 					_mapAngleDataToDofName; //!< Map for the data the name of the joint is mapping the angle
		std::set<string> 							_dofNameSet; //!< Set of the joint name
		std::string _outDirectory;
		std::string _inDirectory;
		bool _record;
		int _verbose;
		map<string, double> _torque;
};

#endif /* OSIMIKPLUGIN_H_ */
