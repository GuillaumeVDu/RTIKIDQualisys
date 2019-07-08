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

#ifndef IKANDIDCOMPUTATION_H
#define IKANDIDCOMPUTATION_H

#include "CommonCEINMS.h"
#include "OpenSimFileLogger.h"

#define timer   timer_class
#include <boost/timer.hpp>
#undef timer

#include <boost/timer/timer.hpp>

#include <boost/thread/condition_variable.hpp>
#include "InvKinInvDynRealTime.h"
#include <boost/thread/mutex.hpp>
#include "KalmanFilterSimTK.h"
#include <OpenSim/OpenSim.h>
#include <boost/thread.hpp>
#include "QualisysClient.h"
#include "XMLInterpreterV2.h"
#include <Filter.h>
#include "InvKinMarker.h"
#include "Laboratory.hxx"
#include "InvDynThread.h"
#include <vector>
#include <cmath>
#include <set>
#include <boost/filesystem.hpp>

/**
 * @brief Class for computing in real-time IK and ID from QTM data.
 * 
 * We use threading for computing IK in parallels.
 * We use the same Kalman filter for all worker fro better output.
 * We finally compute the IK. 
 **/
class IKAndIDComputation
{
	public:
		/**
		 * @brief Constructor
		 * @param executionIKFileName File name of the XML file for the IK.
		 **/
		IKAndIDComputation ( const string& executionIKFileName );

		/**
		 * @brief Desctructor
		 **/
		~IKAndIDComputation();

		/**
		 * @brief Public fonction for stopping all the thread.
		 **/
		inline void stop()
		{
			_mutexEnd.lock();
			_end = true;
			_mutexEnd.unlock();
		}
		
		void start();
		
		void addMass(const std::string& bodyName, double mass);
		
		inline std::vector<double> getTimeIK()
		{
			std::vector<double> time;
			_mutexResult.lock();
			time = _timeIK;
			_timeIK.clear();
			_mutexResult.unlock();
			return time;
		}
		
		inline std::vector<double> getTimeID()
		{
			std::vector<double> time;
			_mutexResult.lock();
			time = _timeID;
			_timeID.clear();
			_mutexResult.unlock();
			return time;
		}
		
		inline std::vector<std::vector<double> > getIKData()
		{
			std::vector<std::vector<double> > temp;
			_mutexResult.lock();
			temp = _ikResult;
			_ikResult.clear();
			_mutexResult.unlock();
			return temp;
		}
		
		inline std::vector<std::vector<double> > getIDData()
		{
			std::vector<std::vector<double> > temp;
			_mutexResult.lock();
			temp = _idResult;
			_idResult.clear();
			_mutexResult.unlock();
			return temp;
		}
		
		inline std::vector<string> getDOFName()
		{
				return _dofNames;
		}
		
		void setDirectory ( std::string outDirectory )
		{
			_outDirectory = outDirectory;
		}

		void setVerbose ( int verbose )
		{
			_verbose = verbose;
		}

		void setRecord ( bool record )
		{
			_record = record;
		}

	protected:
		
		typedef std::vector<std::string> VecStr;
		typedef std::vector<std::vector<double> > VecVecDouble;
		typedef VecVecDouble::const_iterator VecVecDoubleCI;
		typedef VecStr::const_iterator VecStrCI;
		typedef std::vector<std::vector<float> >::const_iterator VecVecFloatCI;
		typedef std::vector<float>::const_iterator VecFloatCI;
		typedef std::vector<double>::const_iterator VecDoubleCI;

		std::string 						_executionIKFileName; 	//!< File name of the XML file for the IK.
		int 								_numberOfThread;		//!< Number of worker;
		boost::shared_ptr<XMLInterpreter> 	_xmlInterpreter;		//!< Interpreter for XML file for the IK.
		SimTK::State*						_si; 					//!< Base OpenSim state. The state use in the workers are copy of this one.
		boost::mutex 						_mutexSi;
		OpenSim::Model* 					_model; 				//!< Base OpenSim model. The model use in the workers are copy of this one.
		std::vector<string> 				_dofNames;				//!< DOF names of the model.
		std::vector<string> 				_muscleNames;			//!< Muscle Names of the model.
		boost::mutex 						_mutexInit;				//!< Mutex for the initialisation of the worker.
		std::vector<boost::thread*> 		_threadWorker;			//!< Vector of the thread worker (IK, ID ,and Kalman).
		boost::thread*						_supervisor;			//!< Boost thread for supervisor;
		boost::thread*						_provider;				//!< Boost thread for provider;
		boost::barrier*						_barrier;

		std::vector<KalmanFilterSimTK*> _kalman;		//!< Kalman filter for the position velocity and acceleration filtering.
		boost::mutex 					_mutexKalman;	//!< Mutex for accessing the kalman filter.
		
		std::vector<boost::condition_variable*> 					_condWorkIKToDo; 				//!< Condition for beginning IK.
		std::vector<boost::condition_variable*> 					_condWorkIDToDo; 				//!< Condition for beginning Kalman and ID.
		std::vector<SimTK::Array_<SimTK::fVec9> > 					_grfDataWorker;  				//!< Vector GRF for each worker.
		std::vector<SimTK::Array_<SimTK::fVec3> > 					_markerDataWorker;  			//!< Vector of marker for each worker.
		std::vector<double> 										_timeWorker;					//!< Time of the data being processed by the worker.
		std::vector<unsigned long> 									_framesNumbersInWorkerKalman;	/**< Frames number of the data being processed in the IK and Kalman.
																									* Put to zero when no processing is being done by the IK or Kalman.**/
		std::vector<unsigned long> 									_framesNumbersInWorkerID; 		/**< Frames number of the data being processed in the IK, ID and Kalman.
																									* Put to zero when no processing is being done by the OK, Kalman or ID. **/
		boost::mutex 												_mutexWorkerForSetData; 		//!< Mutex for setting the worker data.

		std::vector<std::vector<double> > 				_ikResultWorker;		//!< Vector of the IK result for each worker.
		std::vector<std::vector<std::vector<double> > >	_idResultWorker;		//!< Vector of the ID result for each worker.
		std::vector<std::vector<double> >				_idTimeStamp;			//!< Vector of times stamp computed by the workers.
		boost::mutex 									_mutexWorkerForGetData;	//!< Mutex for accessing the worker data.

		std::vector<bool> 	_workerFree;			//!< Boolean for knowing if the worker is not doing any work.
		std::vector<bool> 	_workerIKDone;			//!< Boolean for knowing if the IK of the worker is finish.
		std::vector<bool> 	_workerIDDone;			//!< Boolean for knowing if the ID of the worker is finish.
		boost::mutex 		_mutexWorkerForBool;	//!< Mutex for accesing the Boolean of each worker.

		bool 			_end;		//!< Boolean for ending all the thread.
		boost::mutex 	_mutexEnd;	//!< Mutex for accessing the bool for ending all thread.

		SimTK::Array_<SimTK::fVec9>						_grfData;		//!< GRF data from the providers.
		SimTK::Array_<SimTK::fVec3> 					_markerData;	//!< Marker data from the providers.
		unsigned long 									_framesNumber;	//!< Frames numbers from the providers.
		double 											_timeStamp;		//!< Time stamp from the providers.
		bool 											_dataReady;		//!< Boolean for knowing if new data is availible.
		boost::condition_variable 						_condProvider;	//!< Condition for waiting for new data.
		boost::mutex 									_mutexProvider;	//!< Mutex for accesing the providers data.

		std::vector<std::vector<double> > 	_ikResult;		//!< IK result of this class.
		std::vector<std::vector<double> > 	_idResult;		//!< ID result of this class.
		std::vector<double> 				_timeIK;		//!< Time stamp for the IK.
		std::vector<double> 				_timeID;		//!< Time stamp for the ID.
		boost::mutex 						_mutexResult;	//!< Mutex for accessing the result of the IK, ID and the time stamp.

		std::string _outDirectory;
		bool _record;
		int _verbose;
		
		OpenSimFileLogger<int>* _logger;
		boost::mutex 			_loggerMutex;
		
		/**
		 * @brief Providers of the marker and GRF data. Connect to QTM software via TCP/IP.
		 **/
		void provider();

		/**
		 * @brief Worker that do the IK, Kalman and ID.
		 * @param rank Numbers of this thrad.
		 **/
		void worker ( unsigned int rank );

		/**
		 * @brief Supervisor for the worker. Give and get the data from the providers and the workers.
		 **/
		void supervisor();
		
		/**
		 * @brief Method for giving the data from the providers to the worker and tell them that they have work todo.
		 * @param markerData Marker data.
		 * @param grfData GRF data.
		 * @param currentFramesNumber Frames number.
		 * @param currentTimeStamp time stamp.
		 */
		void giveWorkToWorker ( const SimTK::Array_<SimTK::fVec3>& markerData,
				const SimTK::Array_<SimTK::fVec9>& grfData,
				const unsigned long& currentFramesNumber, const double& currentTimeStamp );
		
		std::vector<double> openSimToStd(const SimTK::Array_<SimTK::fVec3>& data);
		
		std::vector<double> openSimToStd(const SimTK::Array_<SimTK::fVec9>& data);

};

#endif // IKANDIDCOMPUTATION_H
