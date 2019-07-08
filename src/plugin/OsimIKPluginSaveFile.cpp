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

#include "OsimIKPluginSaveFile.h"

OsimIKPlugin::OsimIKPlugin()
{
	_verbose = 1;
	_record = true;
	_outDirectory = "/Output";

}

OsimIKPlugin::~OsimIKPlugin()
{

}

void OsimIKPlugin::stop()
{
	_mutexEnd.lock();
	_end = true;
	_mutexEnd.unlock();

	_filterThread->join();
	_provider->join();

	if ( _record )
	{
		*_IKFile << "Coordinates";
		*_IKFile << std::endl;

		*_IKFile << "version=1";
		*_IKFile << std::endl;

		*_IKFile << "nRows=" << _ikSaveInFile.size();
		*_IKFile << std::endl;

		*_IKFile << "nColumns=" << _dofNameVect.size() + 1;
		*_IKFile << std::endl;

		*_IKFile << "inDegrees=no";
		*_IKFile << std::endl;

		*_IKFile << "endheader";
		*_IKFile << std::endl;

		*_IKFile << "time\t";

		for ( VecStrCI it = _dofNameVect.begin(); it != _dofNameVect.end(); it++ )
			*_IKFile << *it << "\t";

		*_IKFile << std::endl;

		// IK fill
		for ( VecVecDoubleCI it1 = _ikSaveInFile.begin(); it1 != _ikSaveInFile.end();
				it1++ )
		{
			*_IKFile << std::fixed << std::setprecision ( 20 )
					<< _ikTimeSaveInFile[std::distance<VecVecDoubleCI> ( _ikSaveInFile.begin(),
							it1 )] << "\t";

			for ( VecDoubleCI it2 = it1->begin(); it2 != it1->end(); it2++ )
				*_IKFile << std::fixed << std::setprecision ( 10 ) << *it2 << "\t";

			*_IKFile << std::endl;
		}

// 	// ID header
// 	*_IDFile << "Inverse Dynamics Generalized Forces";
// 	*_IDFile << std::endl;
//
// 	*_IDFile << "version=1";
// 	*_IDFile << std::endl;
//
// 	*_IDFile << "nRows=" << _idSaveInFile.size();
// 	*_IDFile << std::endl;
//
// 	*_IDFile << "nColumns=" << _dofNameVect.size() + 1;
// 	*_IDFile << std::endl;
//
// 	*_IDFile << "inDegrees=no";
// 	*_IDFile << std::endl;
//
// 	*_IDFile << "endheader";
// 	*_IDFile << std::endl;
//
// 	*_IDFile << "time\t";
//
// 	for ( VecStrCI it = _dofNameVect.begin(); it != _dofNameVect.end(); it++ )
// 		*_IDFile << *it << "\t";
//
// 	*_IDFile << std::endl;
//
// 	// ID fill
// 	for ( VecVecDoubleCI it1 = _idSaveInFile.begin(); it1 != _idSaveInFile.end(); it1++ )
// 	{
// 		*_IDFile << std::fixed << std::setprecision ( 20 ) << _ikTimeSaveInFile[std::distance<VecVecDoubleCI> ( _idSaveInFile.begin(), it1 )] << "\t";
//
// 		for ( VecDoubleCI it2 = it1->begin(); it2 != it1->end(); it2++ )
// 			*_IDFile << std::fixed << std::setprecision ( 10 ) << *it2 << "\t";
//
// 		*_IDFile << std::endl;
// 	}

		// GRF file header
		*_forceFile << "force.mot";
		*_forceFile << std::endl;

		*_forceFile << "version=1";
		*_forceFile << std::endl;

		*_forceFile << "nRows=" << _forcePlateSaveInFile.size();
		*_forceFile << std::endl;

		*_forceFile << "nColumns=19";
		*_forceFile << std::endl;

		*_forceFile << "inDegrees=no";
		*_forceFile << std::endl;

		*_forceFile << "endheader";
		*_forceFile << std::endl;

		*_forceFile << "time" << "\t";
		*_forceFile << "ground_force_vx" << "\t" << "ground_force_vy" << "\t"
				<< "ground_force_vz" << "\t";
		*_forceFile << "ground_force_px" << "\t" << "ground_force_py" << "\t"
				<< "ground_force_pz" << "\t";
		*_forceFile << "l_ground_force_vx" << "\t" << "l_ground_force_vy" << "\t"
				<< "l_ground_force_vz" << "\t";
		*_forceFile << "l_ground_force_px" << "\t" << "l_ground_force_py" << "\t"
				<< "l_ground_force_pz" << "\t";
		*_forceFile << "ground_torque_px" << "\t" << "ground_torque_py" << "\t"
				<< "ground_torque_pz" << "\t";
		*_forceFile << "l_ground_torque_px" << "\t" << "l_ground_torque_py" << "\t"
				<< "l_ground_torque_pz";
		*_forceFile << std::endl;

		// GRF filling
		for ( std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator it1 =
				_forcePlateSaveInFile.begin(); it1 != _forcePlateSaveInFile.end(); it1++ )
		{
			const unsigned short& cpt = std::distance <
					std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator > (
							_forcePlateSaveInFile.begin(), it1 );
			*_forceFile << std::fixed << std::setprecision ( 20 ) << _forcePlateTimeSaveInFile[cpt] << "\t";

			for ( int i = 0; i < _forcePlateSaveInFile[cpt].size(); i++ )
			{
				//Force
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][0] << "\t";
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][1] << "\t";
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][2] << "\t";
				//Position
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][6] << "\t";
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][7] << "\t";
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][8] << "\t";
			}

			for ( int i = 0; i < _forcePlateSaveInFile[cpt].size(); i++ )
			{
				//Torque
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][3] << "\t";
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][4] << "\t";
				*_forceFile << std::fixed << std::setprecision ( 10 ) << _forcePlateSaveInFile[cpt][i][5] << "\t";
			}

			*_forceFile << std::endl;
		}

		// TRC file header
		*_markerFile << "PathFileType" << "\t" << "4" << "\t" << "(X/Y/Z)" << "\t"
				<< "marker.trc";
		*_markerFile << std::endl;

		*_markerFile << "DataRate" << "\t" << "CameraRate" << "\t" << "NumFrames"
				<< "\t" << "NumMarkers" << "\t" << "Units" << "\t" << "OrigDataRate"
				<< "\t" << "OrigDataStartFrame" << "\t" << "OrigNumFrames";
		*_markerFile << std::endl;

		*_markerFile << "64\t" << "64\t" << _markerSaveInFile.size() << "\t"
				<< _interface->getModel()->getMarkerSet().getSize() << "\t" << "m\t"
				<< "64\t" << "1\t" << _markerSaveInFile.size();
		*_markerFile << std::endl;

		*_markerFile << "Frame#" << "\t" << "time" << "\t";

		for ( int i = 0; i < _interface->getModel()->getMarkerSet().getSize(); i++ )
			*_markerFile << _interface->getModel()->getMarkerSet().get ( i ).getName()
					<< "\t\t\t";

		*_markerFile << std::endl;

		*_markerFile << "\t\t";

		for ( int i = 0; i < _interface->getModel()->getMarkerSet().getSize(); i++ )
			*_markerFile << "x" << i << "\t" << "y" << i << "\t" << "z" << i
					<< "\t";

		*_markerFile << std::endl;

		*_markerFile << std::endl;

		// Marker filling
		for ( std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator it1 =
				_markerSaveInFile.begin(); it1 != _markerSaveInFile.end(); it1++ )
		{
			const unsigned short& cpt = std::distance <
					std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator > (
							_markerSaveInFile.begin(), it1 );
			*_markerFile << cpt << "\t";
			*_markerFile << std::fixed << std::setprecision ( 20 ) << _markerTimeSaveInFile[cpt] << "\t";

			for ( int i = 0; i < _markerSaveInFile[cpt].size(); i++ )
				for ( int j = 0; j < _markerSaveInFile[cpt][i].size(); j++ )
					*_markerFile << _markerSaveInFile[cpt][i][j] << "\t";

			*_markerFile << std::endl;
		}

		for ( VecDoubleCI it = _timingSaveInFile.begin(); it < _timingSaveInFile.end(); it++ )
		{
			*_TimingFile << *it << std::endl;
		}

		// TRC file header
		*_markerFileFilt << "PathFileType" << "\t" << "4" << "\t" << "(X/Y/Z)" << "\t"
				<< "markerFilt.trc";
		*_markerFileFilt << std::endl;

		*_markerFileFilt << "DataRate" << "\t" << "CameraRate" << "\t" << "NumFrames"
				<< "\t" << "NumMarkers" << "\t" << "Units" << "\t" << "OrigDataRate"
				<< "\t" << "OrigDataStartFrame" << "\t" << "OrigNumFrames";
		*_markerFileFilt << std::endl;

		*_markerFileFilt << "64\t" << "64\t" << _markerFiltSaveInFile.size() << "\t"
				<< _interface->getModel()->getMarkerSet().getSize() << "\t" << "m\t"
				<< "64\t" << "1\t" << _markerFiltSaveInFile.size();
		*_markerFileFilt << std::endl;

		*_markerFileFilt << "Frame#" << "\t" << "time" << "\t";

		for ( int i = 0; i < _interface->getModel()->getMarkerSet().getSize(); i++ )
			*_markerFileFilt << _interface->getModel()->getMarkerSet().get ( i ).getName()
					<< "\t\t\t";

		*_markerFileFilt << std::endl;

		*_markerFileFilt << "\t\t";

		for ( int i = 0; i < _interface->getModel()->getMarkerSet().getSize(); i++ )
			*_markerFileFilt << "x" << i << "\t" << "y" << i << "\t" << "z" << i
					<< "\t";

		*_markerFileFilt << std::endl;

		*_markerFileFilt << std::endl;

		// Marker filling
		for ( std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator it1 =
				_markerFiltSaveInFile.begin(); it1 != _markerFiltSaveInFile.end(); it1++ )
		{
			const unsigned short& cpt = std::distance <
					std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator > (
							_markerFiltSaveInFile.begin(), it1 );
			*_markerFileFilt << cpt << "\t";
			*_markerFileFilt << std::fixed << std::setprecision ( 20 ) << _markerTimeSaveInFile[cpt] << "\t";

			for ( int i = 0; i < _markerFiltSaveInFile[cpt].size(); i++ )
				for ( int j = 0; j < _markerFiltSaveInFile[cpt][i].size(); j++ )
					*_markerFileFilt << _markerFiltSaveInFile[cpt][i][j] << "\t";

			*_markerFileFilt << std::endl;
		}

		// GRF file header
		*_forceFileFilt << "force.mot";
		*_forceFileFilt << std::endl;

		*_forceFileFilt << "version=1";
		*_forceFileFilt << std::endl;

		*_forceFileFilt << "nRows=" << _forcePlateFiltSaveInFile.size();
		*_forceFileFilt << std::endl;

		*_forceFileFilt << "nColumns=19";
		*_forceFileFilt << std::endl;

		*_forceFileFilt << "inDegrees=no";
		*_forceFileFilt << std::endl;

		*_forceFileFilt << "endheader";
		*_forceFileFilt << std::endl;

		*_forceFileFilt << "time" << "\t";
		*_forceFileFilt << "ground_force_vx" << "\t" << "ground_force_vy" << "\t"
				<< "ground_force_vz" << "\t";
		*_forceFileFilt << "ground_force_px" << "\t" << "ground_force_py" << "\t"
				<< "ground_force_pz" << "\t";
		*_forceFileFilt << "l_ground_force_vx" << "\t" << "l_ground_force_vy" << "\t"
				<< "l_ground_force_vz" << "\t";
		*_forceFileFilt << "l_ground_force_px" << "\t" << "l_ground_force_py" << "\t"
				<< "l_ground_force_pz" << "\t";
		*_forceFileFilt << "ground_torque_px" << "\t" << "ground_torque_py" << "\t"
				<< "ground_torque_pz" << "\t";
		*_forceFileFilt << "l_ground_torque_px" << "\t" << "l_ground_torque_py" << "\t"
				<< "l_ground_torque_pz";
		*_forceFileFilt << std::endl;

		// GRF filling
		for ( std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator it1 =
				_forcePlateFiltSaveInFile.begin(); it1 != _forcePlateFiltSaveInFile.end(); it1++ )
		{
			const unsigned short& cpt = std::distance <
					std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator > (
							_forcePlateFiltSaveInFile.begin(), it1 );
			*_forceFileFilt << std::fixed << std::setprecision ( 20 ) << _forcePlateTimeSaveInFile[cpt] << "\t";

			for ( int i = 0; i < _forcePlateFiltSaveInFile[cpt].size(); i++ )
			{
				//Force
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][0] << "\t";
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][1] << "\t";
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][2] << "\t";
				//Position
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][6] << "\t";
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][7] << "\t";
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][8] << "\t";
			}

			for ( int i = 0; i < _forcePlateFiltSaveInFile[cpt].size(); i++ )
			{
				//Torque
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][3] << "\t";
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][4] << "\t";
				*_forceFileFilt << std::fixed << std::setprecision ( 10 ) << _forcePlateFiltSaveInFile[cpt][i][5] << "\t";
			}

			*_forceFileFilt << std::endl;
		}

		_IKFile->close();
		_TimingFile->close();
		_markerFile->close();
		_forceFile->close();
		_markerFileFilt->close();
		_forceFileFilt->close();
		delete _IKFile;
// 	delete _IDFile;
		delete _TimingFile;
		delete _markerFile;
		delete _forceFile;
		delete _markerFileFilt;
		delete _forceFileFilt;
	}

	if ( _verbose > 1 )
		std::cout << "\033[1;32mIK thread end\033[0m" << std::endl;

	delete _filterThread;
	delete _provider;
	delete _interface;
	delete _client;
	delete _translate;
}

void OsimIKPlugin::init ( string xmlName, string executionName )
{
	// Get the execution XML files
	std::auto_ptr<ExecutionType> executionPointer;

	_timeCurrent = -1;

	try
	{
		std::auto_ptr<ExecutionType> temp ( execution ( executionName, xml_schema::flags::dont_initialize ) );
		executionPointer = temp;
	}
	catch ( const xml_schema::exception& e )
	{
		cout << e << endl;
		exit ( EXIT_FAILURE );
	}

	// Get the XML filename for the IK
	const string& AngleFile = executionPointer->ConsumerPlugin().AngleDeviceFile().get();

	// Create the interface for the IK and Kalman
	_interface = new RTIKIDInterface ( AngleFile );

	// Create the client for the Qualisys motion capture system
	_client = new QualisysClient ( _interface->getIP(), _interface->getPort() );
	_client->setMarkerNames ( _interface->getMarkersNames() );
	_client->setVerbose ( _verbose );
	_client->init ( _interface->getLabFile() );

	// Create the translation class for CEINMS and OpenSim
	_translate = new TranslateOpenSimCEINMS ( _interface->getTranslateFile() );

	// Create the joint name set
	for ( int i = 0; i < _interface->getModel()->getCoordinateSet().getSize(); i++ )
	{
		const string& jointName = _interface->getModel()->getCoordinateSet().get ( i ).getName();
		_dofNameVect.push_back ( jointName );

		try
		{
			_dofNameSet.insert ( _translate->OpenSimToCEINMS ( jointName ) );
		}
		catch ( const std::out_of_range& oor )
		{
			std::cout << "Joint no found in translate file: " << jointName << std::endl;
		}

		_angleData.push_back ( 0 );
	}

	// Get the Kalman dt
	_fixedTimeComputation = _interface->getDt();

	_end = false;
	_providerDataReady = false;

	if ( _record )
	{
		std::stringstream ss;
		ss << "./";
		ss << _outDirectory;
		boost::filesystem::path dir ( ss.str().c_str() );

		if ( !boost::filesystem::exists ( dir ) )
			if ( !boost::filesystem::create_directory ( dir ) )
				std::cout << "Error in creating directory: " << ss.str() << std::endl;

		ss << "/ik.mot";
		_IKFile = new std::ofstream ( ss.str().c_str() );
// 	_IDFile = new std::ofstream ( "./Output/id.sto" );
		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/ikidTiming.sto";
		_TimingFile = new std::ofstream ( ss.str().c_str() );
		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/forcePlate.mot";
		_forceFile = new std::ofstream ( ss.str().c_str() );
		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/marker.trc";
		_markerFile = new std::ofstream ( ss.str().c_str() );
		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/forcePlateFilt.mot";
		_forceFileFilt = new std::ofstream ( ss.str().c_str() );
		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/markerFilt.trc";
		_markerFileFilt = new std::ofstream ( ss.str().c_str() );
	}

	// fs: 128 Hz fc:10 Hz delay: ~0.027ms
	static const float arr1[] = { -1.3229, 0.5000};
	std::vector<float> aCoeffMarker ( arr1, arr1 + sizeof ( arr1 ) / sizeof ( arr1[0] ) );

	static const float arr2[] = {0.0443, 0.0886, 0.0443};
	std::vector<float> bCoeffMarker ( arr2, arr2 + sizeof ( arr2 ) / sizeof ( arr2[0] ) );

	// fs: 2048 Hz fc 9 Hz delay: ~0.029ms
	static const float arr3[] = { -1.9610, 0.9617};
	std::vector<float> aCoeffGrf ( arr3, arr3 + sizeof ( arr3 ) / sizeof ( arr3[0] ) );

	static const float arr4[] = {0.0001869, 0.0003739, 0.0001869};
	std::vector<float> bCoeffGrf ( arr4, arr4 + sizeof ( arr4 ) / sizeof ( arr4[0] ) );


// 	// fs: 64 Hz fc:10 Hz delay: ~0.027ms
// 	static const float arr1[] = {-0.6997, 0.2595};
// 	std::vector<float> aCoeffMarker (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );
//
// 	static const float arr2[] = {0.1399, 0.2799, 0.1399};
// 	std::vector<float> bCoeffMarker (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );
//
// 	// fs: 1024 Hz fc 9 Hz delay: ~0.029ms
// 	static const float arr3[] = {-1.9219, 0.9249};
// 	std::vector<float> aCoeffGrf (arr3, arr3 + sizeof(arr3) / sizeof(arr3[0]) );
//
// 	static const float arr4[] = {0.0007, 0.0015, 0.0007};
// 	std::vector<float> bCoeffGrf (arr4, arr4 + sizeof(arr4) / sizeof(arr4[0]) );

// 	// fs: 32 Hz fc:10 Hz delay: ~0.046ms
// 	static const float arr1[] = {0.4629, 0.2097};
// 	std::vector<float> aCoeffMarker (arr1, arr1 + sizeof(arr1) / sizeof(arr1[0]) );
//
// 	static const float arr2[] = {0.4182, 0.8363, 0.4182};
// 	std::vector<float> bCoeffMarker (arr2, arr2 + sizeof(arr2) / sizeof(arr2[0]) );
//
// 	// fs: 512 Hz fc 10 Hz delay: ~0.046ms order: 3
// 	static const float arr3[] = {-2.7547, 2.5386, -0.7822};
// 	std::vector<float> aCoeffGrf (arr3, arr3 + sizeof(arr3) / sizeof(arr3[0]) );
//
// 	static const float arr4[] = {0.2051, 0.6153, 0.6153, 0.2051};
// 	std::vector<float> bCoeffGrf (arr4, arr4 + sizeof(arr4) / sizeof(arr4[0]) );


	_markerfFilter.resize ( _interface->getMarkersNames().size() );

	for ( int i = 0; i < _interface->getMarkersNames().size(); i++ )
		for ( int j = 0; j < 3; j++ )
			_markerfFilter[i].push_back ( new fFilter ( aCoeffMarker, bCoeffMarker ) );

	_grffFilter.resize ( _client->getNumberOfForcePlate() );

	for ( int i = 0; i < _client->getNumberOfForcePlate(); i++ )
		for ( int j = 0; j < 6; j++ )
			_grffFilter[i].push_back ( new fFilter ( aCoeffGrf, bCoeffGrf ) );

	// Create the boost thread
	_filterThread = new boost::thread ( boost::bind ( &OsimIKPlugin::filterAngle, this ) );
	_provider = new boost::thread ( boost::bind ( &OsimIKPlugin::provider, this ) );

// 	std::cout << "init ik" << std::endl;
}

void OsimIKPlugin::provider()
{
	timeval now;
	gettimeofday ( &now, NULL );
	double timePast = ( now.tv_sec ) + 0.000001 * now.tv_usec;


	while ( true )
	{
		boost::timer::auto_cpu_timer auto_t3;
		// Check if we have to stop the thread.
		_mutexEnd.lock();

		if ( _end )
		{
			_mutexEnd.unlock();
			break;
		}


		_mutexEnd.unlock();


// 		std::cout << "provider ik" << std::endl;


		_client->receiveData();
		SimTK::Array_<SimTK::fVec3> markerData = _client->getData();
		SimTK::Array_<SimTK::Array_<SimTK::fVec9> > grfData = _client->getDataForcePlate();

		timeval now;
		gettimeofday ( &now, NULL );
		double timeNow = ( now.tv_sec ) + 0.000001 * now.tv_usec;

		{
			boost::mutex::scoped_lock lock ( _mutexProvider );
			_grfData 	= grfData.back();
			_markerData = markerData;
			_timeStamp 	= timeNow;
			_providerDataReady 	= true;
		}

		// Notify the supervisor that data is ready.
		_condProvider.notify_one();

// 		std::cout << "provider ik send" << std::endl;

		if ( _record )
		{
			_markerSaveInFile.push_back ( markerData );
			_markerTimeSaveInFile.push_back ( timeNow );
		}

		double periode = ( timeNow - timePast ) / double ( grfData[0].size() );

		if ( _record )
		{
			for ( int i = 0; i < grfData[0].size(); i++ )
			{

				SimTK::Array_<SimTK::fVec9> force;

				try
				{
					for ( int j = 0; j < _client->getNumberOfForcePlate(); j++ )
						force.push_back ( grfData[j][i] );
				}
				catch ( const std::exception& ex )
				{
					break;
				}

				_forcePlateSaveInFile.push_back ( force );

				if ( i == 0 )
					_forcePlateTimeSaveInFile.push_back ( timePast );
				else
					_forcePlateTimeSaveInFile.push_back ( _forcePlateTimeSaveInFile.back() + periode );
			}
		}

		for ( std::vector<std::vector<fFilter*> >::iterator it1 = _markerfFilter.begin();
				it1 < _markerfFilter.end(); it1++ )
		{
			const int& cpt1 = std::distance<std::vector<std::vector<fFilter*> >::iterator > ( _markerfFilter.begin(), it1 );

			for ( std::vector<fFilter*>::iterator it2 = it1->begin();  it2 < it1->end(); it2++ )
			{
				const int& cpt2 = std::distance<std::vector<fFilter*>::iterator > ( it1->begin(), it2 );
				markerData[cpt1][cpt2] = ( *it2 )->filter ( markerData[cpt1][cpt2] );
			}
		}

		if ( _record )
			_markerFiltSaveInFile.push_back ( markerData );

		for ( std::vector<std::vector<fFilter*> >::iterator it1 = _grffFilter.begin();
				it1 < _grffFilter.end(); it1++ )
		{
			const int& cpt1 = std::distance<std::vector<std::vector<fFilter*> >::iterator > ( _grffFilter.begin(), it1 );

			for ( int i = 0; i < grfData[cpt1].size(); i++ )
			{
				for ( std::vector<fFilter*>::iterator it2 = it1->begin();  it2 < it1->end(); it2++ )
				{
					const int& cpt2 = std::distance<std::vector<fFilter*>::iterator > ( it1->begin(), it2 );
					grfData[cpt1][i][cpt2] = ( *it2 )->filter ( grfData[cpt1][i][cpt2] );
				}

// 				SimTK::fVec3 cop;
// 				cop.setToZero();
// 				cop[0] = ( -1 * ( grfData[cpt1][i][4] + client->getForcePlateCenterGlobalRotated() [cpt1][2] * grfData[cpt1][i][0] )
// 						/ grfData[cpt1][i][2] ) + client->getForcePlateCenterGlobalRotated() [cpt1][0];
// 				cop[1] = ( ( grfData[cpt1][i][3] - ( client->getForcePlateCenterGlobalRotated() [cpt1][2] * grfData[cpt1][i][1] ) )
// 						/ grfData[cpt1][i][2] ) + client->getForcePlateCenterGlobalRotated() [cpt1][1];
// 				grfData[cpt1][i][6] = cop[0];
// 				grfData[cpt1][i][7] = cop[1];
// 				grfData[cpt1][i][8] = cop[2];
			}
		}

		if ( _record )
		{
			for ( int i = 0; i < grfData[0].size(); i++ )
			{
				SimTK::Array_<SimTK::fVec9> force;

				try
				{
					for ( int j = 0; j < _client->getNumberOfForcePlate(); j++ )
						force.push_back ( grfData[j][i] );
				}
				catch ( const std::exception& ex )
				{
					break;
				}

				_forcePlateFiltSaveInFile.push_back ( force );
			}
		}

		timePast = timeNow;
	}
}

void OsimIKPlugin::filterAngle()
{
	timeval timeinitStruct, now;
	double timeNow, timeSub, tinmeInit;
	struct timespec tim;

	SimTK::Array_<SimTK::fVec3> markerData;
	SimTK::Array_<SimTK::fVec9> grfData;
	double currentTimeStamp;

	std::vector<double> angleData;

	while ( true )
	{
		//		boost::timer::auto_cpu_timer auto_t3;
		// get the initial time
		gettimeofday ( &timeinitStruct, NULL );
		tinmeInit = ( timeinitStruct.tv_sec ) + 0.000001 * timeinitStruct.tv_usec;

		_mutexEnd.lock();

		if ( _end )
		{
			_mutexEnd.unlock();
			break;
		}

		_mutexEnd.unlock();

		{
			boost::mutex::scoped_lock lock ( _mutexProvider );

			while ( !_providerDataReady ) _condProvider.wait ( lock );

			// Get the data from the provider.
			markerData 			= _markerData;
			grfData 			= _grfData;
			currentTimeStamp 	= _timeStamp;

			// No more data ready
			_providerDataReady = false;
		}

		// get the data and process it
		_angleDataMutex.lock();
		_interface->runIKMarker ( markerData );
		_angleDataMutex.unlock();
		_interface->computeKalmanFilter();

		angleData.clear();

		// fill the data vector
		for ( VecStrCI it = _dofNameVect.begin(); it < _dofNameVect.end(); it++ )
			angleData.push_back ( _interface->getAngle ( *it ) );


		_angleDataMutex.lock();
		_angleData = angleData;
		_angleDataMutex.unlock();

		_mutexTimeCurrent.lock();
		_timeCurrent = currentTimeStamp;
		_mutexTimeCurrent.unlock();

		if ( _record )
		{

			_ikSaveInFile.push_back ( angleData );
			_ikTimeSaveInFile.push_back ( currentTimeStamp );
		}


		std::vector<double> torqueData = _interface->computeTorqueStd ( grfData, _interface->getAppliedBody() );
		
		_angleDataMutex.lock();
		_torqueData = torqueData;
		_angleDataMutex.unlock();
// 		_idSaveInFile.push_back ( torque );

		// Sleep until the dt time is realized
		// this is for the kalman filter
		gettimeofday ( &now, NULL );
		timeNow = ( now.tv_sec ) + 0.000001 * now.tv_usec;
		timeSub = _fixedTimeComputation - ( timeNow - tinmeInit );

		if ( _record )
			_timingSaveInFile.push_back ( timeNow - tinmeInit );

		/*
				if ( timeSub > 0 )
					boost::this_thread::sleep ( boost::posix_time::nanoseconds ( ( timeSub * 1000000000L ) - 80000L ) );
				else if ( timeSub < 0 )
					std::cout << "Process use more time that specified. Need: " << -timeSub << " more." << std::endl;*/
	}
}

const map<string, double>& OsimIKPlugin::GetDataMap()
{
	timeval tv;
	std::vector<double> angleData;

	// Get the data from the boost thread
	_angleDataMutex.lock();
	angleData = _angleData;
	_angleDataMutex.unlock();

	// Fill the map
	for ( VecStrCI it = _dofNameVect.begin(); it < _dofNameVect.end(); it++ )
	{
		try
		{
			_mapAngleDataToDofName[_translate->OpenSimToCEINMS ( *it )] = angleData[std::distance<VecStrCI> ( _dofNameVect.begin(), it )];
		}
		catch ( const std::out_of_range& oor )
		{
			continue;
		}
	}

	return _mapAngleDataToDofName;
}

const map<string, double>& OsimIKPlugin::GetDataMapTorque()
{
	
	std::vector<double> torqueData;
	
	
	_angleDataMutex.lock();
	torqueData = _torqueData;
	_angleDataMutex.unlock();
	
	for ( VecStrCI it = _dofNameVect.begin(); it < _dofNameVect.end(); it++ )
	{
		try
		{
			_torque[_translate->OpenSimToCEINMS ( *it )] = torqueData[std::distance<VecStrCI> ( _dofNameVect.begin(), it )];
		}
		catch ( const std::out_of_range& oor )
		{
			continue;
		}
	}
	
	return _torque;
}

// For the run-time loading of the library

extern "C" ProducersPluginVirtual* create()
{
	return new OsimIKPlugin;
}

extern "C" void destroy ( ProducersPluginVirtual* p )
{
	delete p;
}
