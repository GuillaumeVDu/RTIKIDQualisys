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
	_numberOfThread = _xmlInterpreter->getNumberofThreadForIK();

	_grfDataWorker.resize ( _numberOfThread );
	_markerDataWorker.resize ( _numberOfThread );
	_timeWorker.resize ( _numberOfThread );
	_ikResultWorker.resize ( _numberOfThread );
	_idResultWorker.resize ( _numberOfThread );
	_idTimeStamp.resize ( _numberOfThread );

	_barrier = new boost::barrier ( _numberOfThread + 2 );

	_verbose = 1;
	_record = false;
	_outDirectory = "/Output";

}

IKAndIDComputation::~IKAndIDComputation()
{
	stop();

	const OpenSim::CoordinateSet& coordSet = _model->getCoordinateSet();

	for ( int i = 0; i < coordSet.getSize(); i++ )
		delete _kalman[i];

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


	_provider->join();

	_supervisor->join();
	delete _barrier;
	delete _supervisor;
	delete _provider;
	delete _model;
	
	_logger->stop();
	delete _logger;
}

void IKAndIDComputation::start()
{
	
	
	if ( _record )
	{
		_logger = new OpenSimFileLogger<int>(_outDirectory);
		
		std::vector<std::string> forcePlateCol;
		forcePlateCol.push_back("ground_force_vx");
		forcePlateCol.push_back("ground_force_vy");
		forcePlateCol.push_back("ground_force_vz");
		forcePlateCol.push_back("ground_force_px");
		forcePlateCol.push_back("ground_force_py");
		forcePlateCol.push_back("ground_force_pz");
		forcePlateCol.push_back("l_ground_force_vx");
		forcePlateCol.push_back("l_ground_force_vy");
		forcePlateCol.push_back("l_ground_force_vz");
		forcePlateCol.push_back("l_ground_force_px");
		forcePlateCol.push_back("l_ground_force_py");
		forcePlateCol.push_back("l_ground_force_pz");
		forcePlateCol.push_back("ground_torque_px");
		forcePlateCol.push_back("ground_torque_py");
		forcePlateCol.push_back("ground_torque_pz");
		forcePlateCol.push_back("l_ground_torque_px");
		forcePlateCol.push_back("l_ground_torque_py");
		forcePlateCol.push_back("l_ground_torque_pz");
		_logger->addLog(Logger::ID, _dofNames);
		_logger->addLog(Logger::IK, _dofNames);
		_logger->addLog(Logger::IKTiming);
		_logger->addLog(Logger::ForcePlate,forcePlateCol);
		_logger->addLog(Logger::ForcePlateFilter, forcePlateCol);
		_logger->addLog(Logger::Marker, _xmlInterpreter->getMarkersNames());
		_logger->addLog(Logger::MarkerFilter, _xmlInterpreter->getMarkersNames());
	}

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
	}

	_supervisor = new boost::thread ( boost::bind ( &IKAndIDComputation::supervisor, this ) );
	_provider = new boost::thread ( boost::bind ( &IKAndIDComputation::provider, this ) );

}

void IKAndIDComputation::addMass ( const std::string& bodyName, double mass )
{

}

void IKAndIDComputation::provider()
{

	// Connect to the QTM software.
	_mutexInit.lock();
	QualisysClient client ( _xmlInterpreter->getIP(), _xmlInterpreter->getPort() );

	client.setMarkerNames ( _xmlInterpreter->getMarkersNames() );
	client.setVerbose ( _verbose );

	client.init ( _xmlInterpreter->getLabFile() );
	_mutexInit.unlock();

	bool firstPass = true;

	std::vector<std::vector<fFilter*> > markerfFilter; // 3 (X,Y,Z) * nb of marker
	std::vector<std::vector<fFilter*> > grffFilter; // 6 (force and torque) * nb of plate

	_barrier->wait();

	double timePast = getTime();

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
		if ( client.receiveData() )
		{

			// Get the data.

			SimTK::Array_<SimTK::fVec3> markerData = client.getDataUnfiltered();
			SimTK::Array_<SimTK::fVec9> grfData = client.getDataForcePlateUnfiltered();

			double timeNow = getTime();

			if(_record)
			{
				boost::mutex::scoped_lock lock(_loggerMutex);
				_logger->log(Logger::Marker, timeNow, openSimToStd(markerData));
				_logger->log(Logger::ForcePlate, timeNow, openSimToStd(grfData));
			}

			timePast = timeNow;

			if ( firstPass )
			{
				_mutexInit.lock();
				std::vector<std::vector<std::vector<float> > > initPosition ( _xmlInterpreter->getMarkersNames().size() );

				for ( int i = 0 ; i < _xmlInterpreter->getMarkersNames().size(); i++ )
				{
					initPosition[i].resize ( 3 );

					if ( std::isnan ( markerData[i][0] ) || std::isnan ( markerData[i][1] ) || std::isnan ( markerData[i][2] ) )
					{
						boost::mutex::scoped_lock lock ( _mutexSi );
						SimTK::Vec3 marker = _model->getMarkerSet().get ( i ).getOffset();
						const OpenSim::SimbodyEngine& engine = _model->getSimbodyEngine();
						_model->getMultibodySystem().realize ( *_si, SimTK::Stage::Position );
						engine.transformPosition ( *_si, _model->getMarkerSet().get ( i ).getBody(), marker, engine.getGroundBody(), marker );
						initPosition[i][0].resize ( _xmlInterpreter->getBCoeffMarker().size(), marker[0] );
						initPosition[i][1].resize ( _xmlInterpreter->getBCoeffMarker().size(), marker[1] );
						initPosition[i][2].resize ( _xmlInterpreter->getBCoeffMarker().size(), marker[2] );
					}
					else
					{
						initPosition[i][0].resize ( _xmlInterpreter->getBCoeffMarker().size(), markerData[i][0] );
						initPosition[i][1].resize ( _xmlInterpreter->getBCoeffMarker().size(), markerData[i][1] );
						initPosition[i][2].resize ( _xmlInterpreter->getBCoeffMarker().size(), markerData[i][2] );
					}
				}

				std::vector<float> atempMarker ( _xmlInterpreter->getACoeffMarker().begin(), _xmlInterpreter->getACoeffMarker().end() );
				std::vector<float> btempMarker ( _xmlInterpreter->getBCoeffMarker().begin(), _xmlInterpreter->getBCoeffMarker().end() );

				client.InitMarkerFilter ( atempMarker, btempMarker, initPosition );


				_mutexInit.unlock();

				std::vector<std::vector<std::vector<float> > > initGrf ( client.getNumberOfForcePlate() );

				for ( int i = 0; i < client.getNumberOfForcePlate(); i++ )
				{
					initGrf[i].resize ( 6 );

					for ( int j = 0; j < 6; j++ )
					{
						boost::mutex::scoped_lock lock ( _mutexInit );
						initGrf[i][j].resize ( _xmlInterpreter->getBCoeffGrf().size(), 0 );
					}
				}

				std::vector<float> atempGrf ( _xmlInterpreter->getACoeffGrf().begin(), _xmlInterpreter->getACoeffGrf().end() );
				std::vector<float> btempGrf ( _xmlInterpreter->getBCoeffGrf().begin(), _xmlInterpreter->getBCoeffGrf().end() );

				client.InitGRFFilter ( atempGrf, btempGrf, initGrf );

				firstPass = false;
			}


			markerData = client.getDatafiltered();
			grfData = client.getDataForcePlatefiltered();

			if(_record)
			{
				boost::mutex::scoped_lock lock(_loggerMutex);
				_logger->log(Logger::MarkerFilter, timeNow, openSimToStd(markerData));
				_logger->log(Logger::ForcePlateFilter, timeNow, openSimToStd(grfData));
			}

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
		else
		{
			_mutexEnd.lock();
			_end  = true;
			_mutexEnd.unlock();
		}
	}

	client.quit();

}

void IKAndIDComputation::worker ( unsigned int rank )
{

	// Copy the OpenSim model and state.
	_mutexInit.lock();

	OpenSim::Model model ( *_model );
	SimTK::State si ( model.initSystem() );

	OpenSim::ForceSet& modelForces = model.updForceSet();

	for ( int i = 0; i < modelForces.getSize(); i++ )
		modelForces[i].setDisabled ( si, true );

	// Create the IK and ID class
	IKSolverRT ik ( model, _xmlInterpreter->getMarkersNames(), _xmlInterpreter->getMarkersWeights() );
	ik.setAccuracy ( _xmlInterpreter->getMaxMarkerError() );
	ik.setEnforceContraint ( _xmlInterpreter->getEnforceConstraintUse() );


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
			COUT << e << endl;
			exit ( EXIT_FAILURE );
		}
	}

	// Create the vector for the acceleraton value.
	SimTK::Vector qddot;
	qddot.resize ( dofNames.size() );

	SimTK::Array_<SimTK::fVec3> markerData;
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
			while ( ! ( !_workerFree[rank] && !_workerIKDone[rank] ) ) _condWorkIKToDo[rank]->wait ( lock );
		}

		_mutexWorkerForSetData.lock();

		// Get the data from the supervisor.
		markerData	= _markerDataWorker[rank];
		timeStamp 	= _timeWorker[rank];

		_mutexWorkerForSetData.unlock();

		_mutexSi.lock();
		si = *_si;
		_mutexSi.unlock();



		// Compute IK

		ik.setMarkerGoal ( markerData );

		// If first pass call assemble method (have to called at the beginning) else call track method (track small change).
		try
		{
			if ( firstPass )
			{
				ik.assemble ( si );
				firstPass = false;
			}
			else
				ik.track ( si );
		}
		catch ( const std::exception& ex )
		{
			_mutexEnd.lock();
			_end = true;
			_mutexEnd.unlock();
			break;
		}

		std::vector<double> ikResult;

		// Transform data from SimTK style to std style.
		for ( std::vector<string>::const_iterator it = dofNames.begin(); it != dofNames.end(); it++ )
		{
			ikResult.push_back ( model.getCoordinateSet().get ( *it ).getValue ( si ) );
		}

		_mutexResult.lock();
		_ikResultWorker[rank] 	= ikResult;
		_mutexResult.unlock();

		// Say to the supervisor that the IK computation is done.
		_mutexWorkerForBool.lock();
		_workerIKDone[rank] = true;
		_mutexWorkerForBool.unlock();
	}

}

void IKAndIDComputation::giveWorkToWorker ( const SimTK::Array_<SimTK::fVec3>& markerData,
		const SimTK::Array_<SimTK::fVec9>& grfData, const unsigned long& currentFramesNumber,
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
				_timeWorker[cpt] 					= currentTimeStamp;
				_framesNumbersInWorkerKalman[cpt] 	= currentFramesNumber;
			}

			_mutexWorkerForBool.lock();

			// The worker is no longer free.
			_workerFree[cpt] = false;

			_mutexWorkerForBool.unlock();

			// Notify the worker that he have work to do.
			_condWorkIKToDo[cpt]->notify_one();

			break;
		}
	}
}

void IKAndIDComputation::supervisor()
{

	SimTK::Array_<SimTK::fVec3> markerData;
	SimTK::Array_<SimTK::fVec9> grfData;
	unsigned long currentFramesNumber;
	bool NewDataFromProviders = false;
	bool firstPassKalman = true;
	std::vector<bool> workerIKDone;
	std::vector<bool> workerIDDone;
	std::vector<bool> workerFree;
	double currentTimeStamp;
	SimTK::Array_<SimTK::fVec9> grfDataForID;
	double timeStamp = 0;
	double pastTimeStamp = 0;
	unsigned int numberOfPlate = 0;
	double dt = 0;
	SimTK::Vector qddot;

	_mutexInit.lock();

	std::vector<string> appliedBody = _xmlInterpreter->getAppliedBody();
	double periode = _xmlInterpreter->getDt();
	InvDyn id ( *_si, *_model );
	qddot.resize ( _dofNames.size() );
	std::vector<string> dofNames = _dofNames;

	{
		try
		{
			std::auto_ptr<LaboratoryType> laboratoryPointer ( Laboratory ( _xmlInterpreter->getLabFile(), xml_schema::flags::dont_initialize ) );
			numberOfPlate = laboratoryPointer->NumberOfForcePlatforms();
		}
		catch ( const xml_schema::exception& e )
		{
			COUT << e << endl;
			exit ( EXIT_FAILURE );
		}
	}
	_mutexInit.unlock();

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
			NewDataFromProviders = true;

			// No more data ready
			_dataReady = false;
		}

		// Give the work to the availible worker.
		if ( NewDataFromProviders )
		{
			giveWorkToWorker ( markerData, grfData, currentFramesNumber, currentTimeStamp );
			NewDataFromProviders = false;
		}

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

					framesNumbersInWorkerSet.erase ( 0 );

					// search for the n-1 frames in the frames vector that the worker process (up to the kalman filter).
					// If yes we have to wait that this frames have been processed because we have to respect the order in the kalman filter.
					if ( framesNumbersInWorkerSet.upper_bound ( framesNumbersInWorkerKalman[cpt] - 1 ) == framesNumbersInWorkerSet.begin() || framesNumbersInWorkerKalman[cpt] == 1 )
					{
// 						boost::timer::auto_cpu_timer auto_t3;

						_mutexWorkerForSetData.lock();
						_framesNumbersInWorkerKalman[cpt] = 0;
						_mutexWorkerForSetData.unlock();

						_mutexWorkerForGetData.lock();
						_mutexResult.lock();
						_ikResult.push_back ( _ikResultWorker[cpt] );
						_timeIK.push_back ( _timeWorker[cpt] );
						
						grfDataForID = _grfDataWorker[cpt];
						timeStamp =  _timeWorker[cpt];

						const OpenSim::CoordinateSet& coordSet = _model->getCoordinateSet();

						for ( std::vector<string>::const_iterator it = dofNames.begin(); it != dofNames.end(); it++ )
						{
							_mutexSi.lock();

							if ( !coordSet.get ( *it ).getLocked ( *_si ) )
							{
								const int& i = std::distance<std::vector<string>::const_iterator> ( dofNames.begin(), it );
								_model->updMatterSubsystem().getMobilizedBody ( coordSet.get ( *it ).getBodyIndex() ).setOneQ (
									*_si, coordSet.get ( *it ).getMobilizerQIndex(),
									_ikResultWorker[cpt][i] );
							}

							_mutexSi.unlock();
						}

						_mutexWorkerForGetData.unlock();
						_mutexResult.unlock();

						if(_record)
						{
							double timeDoubleNow = getTime();
							boost::mutex::scoped_lock lock(_loggerMutex);
							_logger->log(Logger::IK, timeStamp, _ikResultWorker[cpt]);
							_logger->log(Logger::IKTiming, timeStamp, timeDoubleNow - timeStamp);
						}

						_mutexWorkerForBool.lock();
						_workerFree[cpt] 	= true;
					
						// The worker have finish the IK computation and we know it.
						_workerIKDone[cpt] = false;
						workerIKDone[cpt] = false;
						_mutexWorkerForBool.unlock();

						if ( firstPassKalman )
						{
							pastTimeStamp = timeStamp;
						}
						else
						{
							dt = timeStamp - pastTimeStamp ;
							pastTimeStamp = timeStamp;
						}

						for ( std::vector<string>::const_iterator it = dofNames.begin(); it != dofNames.end(); it++ )
						{
							SimTK::Vec3 states;

							const int& cpt = std::distance<std::vector<string>::const_iterator> ( dofNames.begin(), it );

							_mutexSi.lock();

							if ( firstPassKalman )
							{
								firstPassKalman = false;
							}
							else
							{
								_kalman[cpt]->updateDt ( dt );
							}

							// Compute the angle filtered, velocity and acceleration
							states = _kalman[cpt]->computeKalmanFilter ( _model->getCoordinateSet().get ( *it ).getValue ( *_si ) );

							// If the joint is not locked set the angle and the velocity otherwise set the acceleration to zero
							if ( !coordSet.get ( *it ).getLocked ( *_si ) )
							{
								_model->updMatterSubsystem().getMobilizedBody ( coordSet.get ( *it ).getBodyIndex() ).setOneQ (
									*_si, coordSet.get ( *it ).getMobilizerQIndex(), states[0] );

								coordSet.get ( *it ).setSpeedValue ( *_si, states[1] );
								qddot.set ( cpt, states[2] );

							}
							else
								qddot.set ( cpt, 0 );

							_mutexSi.unlock();
						}

						double idTimeStamp;

						// QTM give us a packet of GRF data, so we have to process one after another.
						SimTK::Vector tempSimtk = id.computeTorque ( grfDataForID, appliedBody, qddot );

						std::vector<double> tempStd;

						for ( int j = 0; j < tempSimtk.size(); j++ )
							tempStd.push_back ( tempSimtk[j] );

						_mutexResult.lock();
						_idResult.push_back ( tempStd );
						_timeID.push_back ( timeStamp );
						_mutexResult.unlock();
						
						idTimeStamp = timeStamp;
						if(_record)
						{
							boost::mutex::scoped_lock lock(_loggerMutex);
							_logger->log(Logger::ID, timeStamp, tempStd);
						}
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

}

std::vector<double> IKAndIDComputation::openSimToStd(const SimTK::Array_<SimTK::fVec3>& data)
{
	std::vector<double> out;
	for(SimTK::Array_<SimTK::fVec3>::const_iterator it = data.begin(); it != data.end(); it++)
		for(int cpt = 0; cpt < 3; cpt++)
			out.push_back((*it)[cpt]);
	return out;
}
		
std::vector<double> IKAndIDComputation::openSimToStd(const SimTK::Array_<SimTK::fVec9>& data)
{
	std::vector<double> out;
	for(SimTK::Array_<SimTK::fVec9>::const_iterator it = data.begin(); it != data.end(); it++)
	{
		for(int cpt = 0; cpt < 3; cpt++)
			out.push_back((*it)[cpt]);
		for(int cpt = 6; cpt < 9; cpt++)
			out.push_back((*it)[cpt]);
	}
	for(SimTK::Array_<SimTK::fVec9>::const_iterator it = data.begin(); it != data.end(); it++)
		for(int cpt = 3; cpt < 6; cpt++)
			out.push_back((*it)[cpt]);
	return out;
}
