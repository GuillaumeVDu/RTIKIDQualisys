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

#include "IKAndIDComputation.h"

IKAndIDComputation::IKAndIDComputation ( const string& executionIKFileName ) :
	_executionIKFileName ( executionIKFileName )
{
	std::cout << "\033[1;31mIKAndIDComputation constructor: " << this << "\033[0m" << std::endl;

	// Initialisation of the XML interpreter.
	_xmlInterpreter = boost::shared_ptr<XMLInterpreter> ( new XMLInterpreter ( _executionIKFileName ) );
	_xmlInterpreter->readXML();

	// Initialisation of the OpenSim model.
	_model = new OpenSim::Model ( _xmlInterpreter->getOsimFile() );

	// Initialisation of the OpenSim state.
	_si = & ( _model->initSystem() );

	// Disable the Model Force.
	OpenSim::ForceSet& modelForces = _model->updForceSet();

	for ( int i = 0; i < modelForces.getSize(); i++ )
		modelForces[i].setDisabled ( *_si, true );

	const OpenSim::CoordinateSet& coordSet = _model->getCoordinateSet();

	// Create the Kalman filter.
	for ( int i = 0; i < coordSet.getSize(); i++ )
		_kalman.push_back ( new KalmanFilterSimTK ( _xmlInterpreter->getR(),
				_xmlInterpreter->getP(),
				_xmlInterpreter->getSigmaDa(),
				_xmlInterpreter->getDt() ) );

	// Create joint name vector.
	for ( int i = 0; i < coordSet.getSize(); i++ )
		_dofNames.push_back ( coordSet.get ( i ).getName() );

	// Create muscle name vector.
	for ( int i = 0; i < _model->getMuscles().getSize(); i++ )
		_muscleNames.push_back ( _model->getMuscles().get ( i ).getName() );

	_framesNumber = 0;
	_dataReady = false;
	_end = false;

	// Create Worker thread and data.
	_numberOfThread = 2;

	_grfDataWorker.resize ( _numberOfThread );
	_markerDataWorker.resize ( _numberOfThread );
	_timeWorker.resize ( _numberOfThread );
	_ikResultWorker.resize ( _numberOfThread );
	_idResultWorker.resize ( _numberOfThread );
	_idTimeStamp.resize ( _numberOfThread );

	_barrier = new boost::barrier ( _numberOfThread + 2 );

	for ( int i = 0; i < _numberOfThread; i++ )
	{
		_condWorkIKToDo.push_back ( new boost::condition_variable() );
		_condWorkIDToDo.push_back ( new boost::condition_variable() );
		_workerFree.push_back ( true );
		_workerIKDone.push_back ( false );
		_workerIDDone.push_back ( false );
		_framesNumbersInWorkerID.push_back ( 0 );
		_framesNumbersInWorkerKalman.push_back ( 0 );
		_threadWorker.push_back ( new boost::thread ( boost::bind ( &IKAndIDComputation::worker, this , i ) ) );
		std::cout << "test" << std::endl;
	}

	_supervisor = new boost::thread ( boost::bind ( &IKAndIDComputation::supervisor, this ) );
	_provider = new boost::thread ( boost::bind ( &IKAndIDComputation::provider, this ) );
}

IKAndIDComputation::~IKAndIDComputation()
{
	stop();

	for ( int i = 0; i < _numberOfThread; i++ )
	{
		_threadWorker[i]->interrupt();
		_threadWorker[i]->join();
		delete _condWorkIKToDo[i];
		delete _condWorkIDToDo[i];
		delete _threadWorker[i];
	}

	_supervisor->interrupt();
	_provider->interrupt();
	_supervisor->join();
	_provider->join();
	delete _supervisor;
	delete _provider;
	delete _si;
	delete _model;

	std::cout << "\033[1;31mIKAndIDComputation destructor: " << this << "\033[0m" << std::endl;
}

void IKAndIDComputation::provider()
{
	std::cout << "\033[1;31mIKAndIDComputation provider: " << this << " ID: " << boost::this_thread::get_id() << "\033[0m" << std::endl;

	// Connect to the QTM software.
	QualisysClient client ( _xmlInterpreter->getIP(), _xmlInterpreter->getPort() );
	client.init ( _xmlInterpreter->getLabFile() );

	_barrier->wait();

	while ( true )
	{
//  		boost::timer::auto_cpu_timer auto_t3;
		// Check if we have to stop the thread.
		_mutexEnd.lock();

		if ( _end )
		{
			_mutexEnd.unlock();
			break;
		}

		_mutexEnd.unlock();

		// Receive data from QTM.
		client.receiveData();

		// Get the data.
		const SimTK::Array_<SimTK::fVec3>& markerData = client.getData();
		const SimTK::Array_< SimTK::Array_<SimTK::fVec9> >& grfData = client.getDataForcePlate();

		timeval now;
		gettimeofday ( &now, NULL );
		double timeNow = ( now.tv_sec ) + 0.000001 * now.tv_usec;

		// Give the data to the supervisor.
		{
			boost::mutex::scoped_lock lock ( _mutexProvider );
			_grfData 	= grfData;
			_markerData = markerData;
			_timeStamp 	= timeNow;
			_dataReady 	= true;
			_framesNumber++;
		}

		// Notify the supervisor that data is ready.
		_condProvider.notify_one();
	}

	client.quit();
	std::cout << "\033[1;31mQuit IKAndIDComputation provider: " << this << " ID: " << boost::this_thread::get_id() << "\033[0m" << std::endl;

}

void IKAndIDComputation::worker ( unsigned int rank )
{
	std::cout << "\033[1;31mIKAndIDComputation worker: " << this << " numbers: " << rank << " ID: " << boost::this_thread::get_id() << "\033[0m" << std::endl;

	// Copy the OpenSim model and state.
	_mutexInit.lock();
	
	OpenSim::Model model( *_model);
	SimTK::State si( model.initSystem());
	
	// Create the IK and ID class
	IKSolverRT ik ( model, _xmlInterpreter->getMarkersNames(), _xmlInterpreter->getMarkersWeights());
	ik.setAccuracy(_xmlInterpreter->getMaxMarkerError());
	ik.setEnforceContraint(_xmlInterpreter->getEnforceConstraintUse());
	
	_mutexInit.unlock();

	InvDyn id ( si, model );

	_mutexInit.lock();

	// Initialise the IK

	// Get the name of the lab file XML.
	std::string labFile = _xmlInterpreter->getLabFile();

	// Get the name of body on which the force applied.
	std::vector<string> appliedBody = _xmlInterpreter->getAppliedBody();

	// Get the DOF Name.
	std::vector<string> dofNames = _dofNames;

	// get the time between two QTM acquisition.
	double periode = _xmlInterpreter->getDt();

	_mutexInit.unlock();

	double timeStamp = 0;

	unsigned int numberOfPlate = 0;

	{
		try
		{
			std::auto_ptr<LaboratoryType> laboratoryPointer ( Laboratory ( labFile, xml_schema::flags::dont_initialize ) );
			numberOfPlate = laboratoryPointer->NumberOfForcePlatforms();
		}
		catch ( const xml_schema::exception& e )
		{
			cout << e << endl;
			exit ( EXIT_FAILURE );
		}
	}

	// Create the vector for the acceleraton value.
	SimTK::Vector qddot;
	qddot.resize ( dofNames.size() );

	SimTK::Array_<SimTK::fVec3> markerData;
	SimTK::Array_< SimTK::Array_<SimTK::fVec9> > grfData;
	unsigned long currentFramesNumber;
	bool firstPass = true;

	_barrier->wait();

	while ( true )
	{
		// Check if we have to stop the thread.
		_mutexEnd.lock();

		if ( _end )
		{
			_mutexEnd.unlock();
			break;
		}

		_mutexEnd.unlock();

		// Check if have new data from the providers.
		{
			boost::mutex::scoped_lock lock ( _mutexWorkerForBool );

			// We have to wait that we have work and also that the supervisor get the ID result.
			while ( !(!_workerFree[rank] && !_workerIDDone[rank]) ) _condWorkIKToDo[rank]->wait ( lock );

// 			std::cout << "work for: " << rank << " " << _workerFree[rank] << " : " << _workerIDDone[rank] << std::endl;
		}

		_mutexWorkerForSetData.lock();

		// Get the data from the supervisor.
		markerData	= _markerDataWorker[rank];
		grfData 	= _grfDataWorker[rank];
		timeStamp 	= _timeWorker[rank];

		_mutexWorkerForSetData.unlock();
		
		_mutexSi.lock();
		si = *_si;
		_mutexSi.unlock();

 		

		// Compute IK
		
		ik.setMarkerGoal(markerData);

		// If first pass call assemble method (have to called at the beginning) else call track method (track small change).
		if (firstPass)
		{
// 			std::cout << "waiting assemble: " << rank << std::endl;
// 			_mutexInit.lock();
			ik.assemble(si);
// 			_mutexInit.unlock();
			firstPass = false;
// 			std::cout << "Finish assemble: " << rank << std::endl;
		}
		else
		{
// 			_mutexInit.lock();
// 			std::cout << "waiting track: " << rank << std::endl;
			ik.track(si);
// 			_mutexInit.unlock();
// 			std::cout << "finish track: " << rank << std::endl;
		}
		
		// Say to the supervisor that the IK computation is done.
		_mutexWorkerForBool.lock();
		_workerIKDone[rank] = true;
		_mutexWorkerForBool.unlock();

		// Wait for acces to Kalman filter after the previous kalman computation have been done.
		{
			boost::mutex::scoped_lock lock ( _mutexWorkerForBool );

			while ( _workerIKDone[rank] ) _condWorkIDToDo[rank]->wait ( lock );
		}

		// Realize the model.
		model.getMultibodySystem().realize ( si, SimTK::Stage::Position );

		const OpenSim::CoordinateSet& coordSet = model.getCoordinateSet();

// 		std::cout << "waiting kalman: " << rank << std::endl;
		_mutexKalman.lock();

		// Compute the kalman filter.
		for ( std::vector<string>::const_iterator it = dofNames.begin(); it != dofNames.end(); it++ )
		{
			SimTK::Vec3 states;

			const int& cpt = std::distance<std::vector<string>::const_iterator> ( dofNames.begin(), it );
			
			if(*it == "knee_angle_r")
				std::cout << *it << "; " << model.getCoordinateSet().get ( *it ).getValue ( si ) << "\t";
			
			if(cpt == 9)
				std::cout << model.getCoordinateSet().get ( *it ).getValue ( si ) << std::endl;

			// Compute the angle filtered, velocity and acceleration
			states = _kalman[cpt]->computeKalmanFilter ( model.getCoordinateSet().get ( *it ).getValue ( si ) );
			
			if(cpt == 9)
				std::cout << states[0] << std::endl;

			// If the joint is not locked set the angle and the velocity otherwise set the acceleration to zero
			if ( !coordSet.get ( *it ).getLocked ( si ) )
			{
				model.updMatterSubsystem().getMobilizedBody ( coordSet.get ( *it ).getBodyIndex() ).setOneQ (
					si, coordSet.get ( *it ).getMobilizerQIndex(), states[0] );

				coordSet.get ( *it ).setSpeedValue ( si, states[1] );
				qddot.set ( cpt, states[2] );
			}
			else
				qddot.set ( cpt, 0 );
		}

		_mutexKalman.unlock();
		
		_mutexSi.lock();
		*_si = si;
		_mutexSi.unlock();
// 		std::cout << "finish kalman: " << rank << std::endl;

		// We have done the klaman filtering. Tell the supervisor that this fram have been processed up to the Kalman level.
		_mutexWorkerForSetData.lock();
		_framesNumbersInWorkerKalman[rank] = 0;
		_mutexWorkerForSetData.unlock();

		// Compute ID.
		std::vector<std::vector<double> > torque;
		std::vector<double> idTimeStamp;

		SimTK::Array_<SimTK::fVec9> Ftemp;
		double idPeriode = periode / grfData[0].size();

		// QTM give us a packet of GRF data, so we have to process one after another.
		for ( int i = 0; i < grfData[0].size(); i++ )
		{
			SimTK::Array_<SimTK::fVec9> force;

			try
			{
				for ( int j = 0; j < numberOfPlate; j++ )
					force.push_back ( grfData[j][i] );
			}
			catch ( const std::exception& ex )
			{
				break;
			}

			SimTK::Vector tempSimtk = id.computeTorque ( force, appliedBody, qddot );

			std::vector<double> tempStd;

			for ( int j = 0; j < tempSimtk.size(); j++ )
				tempStd.push_back ( tempSimtk[j] );

			torque.push_back ( tempStd );

			// We also have to create specific time stamp for the ID.
			if ( i == 0 )
				idTimeStamp.push_back ( timeStamp );
			else
				idTimeStamp.push_back ( idTimeStamp.back() + idPeriode );
		}

		std::vector<double> ikResult;

		// Transform data from SimTK style to std style.
		for ( std::vector<string>::const_iterator it = dofNames.begin(); it != dofNames.end(); it++ )
		{
			ikResult.push_back ( model.getCoordinateSet().get( *it ).getValue ( si ));
		}
		
		_mutexSi.lock();
		*_si = si;
		_mutexSi.unlock();

// 		std::cout << "finish ID: " << rank << std::endl;

		// Give the result to the supervisor.
		_mutexResult.lock();
		_ikResultWorker[rank] 	= ikResult;
		_idResultWorker[rank] 	= torque;
		_idTimeStamp[rank]		= idTimeStamp;
		_mutexResult.unlock();
		
		
		
//  		std::cout << std::fixed << std::setprecision ( 20 ) << idTimeStamp.back() << std::endl;

		// Say to the supervisor that the ID computation is done.
		_mutexWorkerForBool.lock();
		_workerIDDone[rank] = true;
		_mutexWorkerForBool.unlock();
		
	}

	std::cout << "\033[1;31mQuit IKAndIDComputation worker: " << this << " numbers: " << rank << " ID: " << boost::this_thread::get_id() << "\033[0m" << std::endl;

}

void IKAndIDComputation::giveWorkToWorker ( const SimTK::Array_<SimTK::fVec3>& markerData,
		const SimTK::Array_< SimTK::Array_<SimTK::fVec9> >& grfData, const unsigned long& currentFramesNumber,
		const double& currentTimeStamp )
{
	// Get the worker free data.
	std::vector<bool> workerFree;
	_mutexWorkerForBool.lock();
	workerFree = _workerFree;
	_mutexWorkerForBool.unlock();

	std::vector<bool>::iterator lastIterator = workerFree.end();
	lastIterator--;

	for ( std::vector<bool>::const_iterator itFreeWorker = workerFree.begin(); itFreeWorker < workerFree.end(); itFreeWorker++ )
	{
		// If a worker is free
		if ( *itFreeWorker )
		{
			const int& cpt = std::distance<std::vector<bool>::const_iterator> ( workerFree.begin(), itFreeWorker );
			{
				boost::mutex::scoped_lock lock ( _mutexWorkerForSetData );

				// Give the data to the worker
				_grfDataWorker[cpt] 				= grfData;
				_markerDataWorker[cpt] 				= markerData;
				_framesNumbersInWorkerKalman[cpt] 	= currentFramesNumber;
				_framesNumbersInWorkerID[cpt] 		= currentFramesNumber;
				_timeWorker[cpt] 					= currentTimeStamp;
			}
			

			_mutexWorkerForBool.lock();

			// The worker is no longer free.
			_workerFree[cpt] = false;

			_mutexWorkerForBool.unlock();
			
			// Notify the worker that he have work to do.
			_condWorkIKToDo[cpt]->notify_one();

// 			std::cout << "Give work to: " << cpt << std::endl;

			break;
		}
		else if ( itFreeWorker == lastIterator )
		{
			std::cout << "Not enough thread Worker" << std::endl;
		}
	}
}

void IKAndIDComputation::supervisor()
{
	std::cout << "\033[1;31mIKAndIDComputation supervisor: " << this << " ID: " << boost::this_thread::get_id() <<  "\033[0m" << std::endl;

	SimTK::Array_<SimTK::fVec3> markerData;
	SimTK::Array_< SimTK::Array_<SimTK::fVec9> > grfData;
	unsigned long currentFramesNumber;
	bool NewDataFromProviders = false;
	std::vector<bool> workerIKDone;
	std::vector<bool> workerIDDone;
	std::vector<bool> workerFree;
	double currentTimeStamp;

	_barrier->wait();

	while ( true )
	{
		// Check if we have to stop the thread.
		_mutexEnd.lock();

		if ( _end )
		{
			_mutexEnd.unlock();
			break;
		}

		_mutexEnd.unlock();

		// Wait for new data from the provider.
		{
			boost::mutex::scoped_lock lock ( _mutexProvider );

			while ( !_dataReady ) _condProvider.wait ( lock );

			// Get the data from the provider.
			markerData 			= _markerData;
			grfData 			= _grfData;
			currentFramesNumber = _framesNumber;
			currentTimeStamp 	= _timeStamp;

			// No more data ready
			_dataReady = false;
		}

		// Give the work to the availible worker.
		giveWorkToWorker ( markerData, grfData, currentFramesNumber, currentTimeStamp );

		while ( true )
		{
			_mutexEnd.lock();

			if ( _end )
			{
				_mutexEnd.unlock();
				break;
			}

			_mutexEnd.unlock();
			// Check for new data from the providers.
			{
				boost::mutex::scoped_lock lock ( _mutexProvider );

				if ( _dataReady )
				{
					// Get the data from the provider.
					markerData 			= _markerData;
					grfData 			= _grfData;
					currentFramesNumber = _framesNumber;
					currentTimeStamp 	= _timeStamp;

					// No more data ready
					_dataReady = false;

					NewDataFromProviders = true;
				}
			}

			if ( NewDataFromProviders )
			{
				// Give the work to a worker.
				giveWorkToWorker ( markerData, grfData, currentFramesNumber, currentTimeStamp );
				NewDataFromProviders = false;
			}

			// Check if a worker have finish with the IK processing.
			_mutexWorkerForBool.lock();
			workerIKDone = _workerIKDone;
			_mutexWorkerForBool.unlock();

			for ( std::vector<bool>::const_iterator itWorkerIKDone = workerIKDone.begin(); itWorkerIKDone < workerIKDone.end(); itWorkerIKDone++ )
			{
				if ( *itWorkerIKDone )
				{
					const int& cpt = std::distance<std::vector<bool>::const_iterator> ( workerIKDone.begin(), itWorkerIKDone );
					std::vector<unsigned long> framesNumbersInWorkerKalman;

					// Get the frames number up to the Kalman level.
					_mutexWorkerForSetData.lock();
					framesNumbersInWorkerKalman = _framesNumbersInWorkerKalman;
					_mutexWorkerForSetData.unlock();

					// Transform the vector to a set for efficient find (maybe too much if the vector is short).
					std::set<unsigned int> framesNumbersInWorkerSet ( framesNumbersInWorkerKalman.begin(), framesNumbersInWorkerKalman.end() );
					
// 					std::cout << "IK: " << framesNumbersInWorkerKalman[cpt] << std::endl;
// 					for(std::vector<unsigned long>::const_iterator itFrames = framesNumbersInWorkerKalman.begin(); itFrames < framesNumbersInWorkerKalman.end(); itFrames++)
// 					{
// 						std::cout << *itFrames << " : ";
// 					}
// 					std::cout << std::endl;
					
					framesNumbersInWorkerSet.erase(0);
					
					// search for the n-1 frames in the frames vector that the worker process (up to the kalman filter).
					// If yes we have to wait that this frames have been processed because we have to respect the order in the kalman filter.
					if ( framesNumbersInWorkerSet.upper_bound ( framesNumbersInWorkerKalman[cpt] - 1 ) == framesNumbersInWorkerSet.begin() || framesNumbersInWorkerKalman[cpt] == 1 )
					{
						

						_mutexWorkerForBool.lock();

						// The worker have finish the IK computation and we know it.
						_workerIKDone[cpt] = false;

						_mutexWorkerForBool.unlock();
						
						// Notify the worker that he can do the kalman filtering and the ID.
						_condWorkIDToDo[cpt]->notify_one();
// 						std::cout << "IK: " << framesNumbersInWorkerKalman[cpt] << std::endl;
// 						std::cout << "IK done: " << cpt << std::endl;
					}
				}
			}

			// Check for new data from the providers.
			{
				boost::mutex::scoped_lock lock ( _mutexProvider );

				if ( _dataReady )
				{
					// Get the data from the provider.
					markerData 			= _markerData;
					grfData 			= _grfData;
					currentFramesNumber = _framesNumber;
					currentTimeStamp 	= _timeStamp;

					// No more data ready
					_dataReady = false;

					NewDataFromProviders = true;
				}
			}

			if ( NewDataFromProviders )
			{
				// Give the work to a worker.
				giveWorkToWorker ( markerData, grfData, currentFramesNumber, currentTimeStamp );
				NewDataFromProviders = false;
			}

			// Check if a worker have finish with the ID processing.
			_mutexWorkerForBool.lock();
			workerIDDone = _workerIDDone;
			_mutexWorkerForBool.unlock();

			for ( std::vector<bool>::const_iterator itWorkerIDDone = workerIDDone.begin(); itWorkerIDDone < workerIDDone.end(); itWorkerIDDone++ )
			{
				if ( *itWorkerIDDone )
				{
					const int& cpt = std::distance<std::vector<bool>::const_iterator> ( workerIDDone.begin(), itWorkerIDDone );
					std::vector<unsigned long> framesNumbersInWorkerID;

					// Get the frames number up to the ID level.
					_mutexWorkerForSetData.lock();
					framesNumbersInWorkerID = _framesNumbersInWorkerID;
					_mutexWorkerForSetData.unlock();

					// Transform the vector to a set for efficient find (maybe too much if the vector is short).
					std::set<unsigned int> framesNumbersInWorkerSet ( framesNumbersInWorkerID.begin(), framesNumbersInWorkerID.end() );

// 					for(std::vector<unsigned long>::const_iterator itFrames = framesNumbersInWorkerID.begin(); itFrames < framesNumbersInWorkerID.end(); itFrames++)
// 					{
// 						std::cout << *itFrames << " : ";
// 					}
// 					std::cout << std::endl;
					framesNumbersInWorkerSet.erase(0);

					// search for the n-1 frames in the frames vector that the worker process (up to the ID).
					// If yes we have to wait that this frames have been processed because we have to respect the order in queue.
					if ( framesNumbersInWorkerSet.upper_bound ( framesNumbersInWorkerID[cpt] - 1 ) == framesNumbersInWorkerSet.begin() || framesNumbersInWorkerID[cpt] == 1)
					{
						// The worker is free and we are getting the data.
						_mutexWorkerForBool.lock();
						_workerIDDone[cpt] 	= false;
						_workerFree[cpt] 	= true;
						_mutexWorkerForBool.unlock();
						
// 						std::cout << "worker finish work: " << cpt << std::endl;

						// Get the data and at it to the queue
						_mutexWorkerForGetData.lock();
						_mutexResult.lock();
						_ikResult.push_back ( _ikResultWorker[cpt] );
						_timeIK.push_back ( _timeWorker[cpt] );

						for ( int i = 0; i < _idResultWorker[cpt].size(); i++ )
						{
							_idResult.push_back ( _idResultWorker[cpt][i] );
							_timeID.push_back ( _idTimeStamp[cpt][i] );
						}
						
// 						std::cout << "END " << std::fixed << std::setprecision ( 20 ) << _timeID.back() << std::endl;

						_mutexWorkerForGetData.unlock();
						_mutexResult.unlock();

						// ste the frames to zero because we have set the data in the queue.
						_mutexWorkerForSetData.lock();
						_framesNumbersInWorkerID[cpt] = 0;
						_mutexWorkerForSetData.unlock();
						
// 						std::cout << "ID: " << framesNumbersInWorkerID[cpt] << std::endl;
						
// 						std::cout << "ID done: " << cpt << std::endl;
					}
				}
			}

			// Check if all the worker are free, if yes get out of the while and wait for new data.
			_mutexWorkerForBool.lock();
			workerFree = _workerFree;
			_mutexWorkerForBool.unlock();
			bool allFree = false;

			for ( std::vector<bool>::const_iterator itFreeWorker = workerFree.begin(); itFreeWorker < workerFree.end(); itFreeWorker++ )
			{
				if ( *itFreeWorker == false )
					break;
			}

			if ( allFree )
				break;
		}
	}

	std::cout << "\033[1;31mQuit IKAndIDComputation supervisor: " << this << " ID: " << boost::this_thread::get_id() <<  "\033[0m" << std::endl;

}
