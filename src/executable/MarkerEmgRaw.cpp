#include "QualisysClient.h"

#define timer   timer_class
#include <boost/timer.hpp>
#undef timer

#include <boost/timer/timer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <ctime>

#include "SyncToolsIK.h"
#include "SyncToolsEMG.h"
#include <csignal>
#include <queue>
#include <fstream>
#include <cmath>
#include "OTSocket.h"


QualisysClient* client;

typedef std::vector<std::string> VecStr;
typedef std::vector<std::vector<double> > VecVecDouble;
typedef VecVecDouble::const_iterator VecVecDoubleCI;
typedef VecStr::const_iterator VecStrCI;
typedef std::vector<std::vector<float> >::const_iterator VecVecFloatCI;
typedef std::vector<float>::const_iterator VecFloatCI;
typedef std::vector<double>::const_iterator VecDoubleCI;

std::vector<std::string> dofName;

boost::barrier* _barrier;


std::queue<SimTK::Array_<SimTK::Array_<SimTK::fVec9> > > _grfData;		//!< GRF data from the providers.
std::queue<SimTK::Array_<SimTK::fVec3> > 				_markerData;	//!< Marker data from the providers.
std::queue<unsigned long>								_framesNumber;	//!< Frames numbers from the providers.
std::queue<double> 										_timeStamp;		//!< Time stamp from the providers.
bool 													_dataReady;		//!< Boolean for knowing if new data is availible.
boost::condition_variable 								_condProvider;	//!< Condition for waiting for new data.
boost::mutex 											_mutexProvider;	//!< Mutex for accesing the providers data.



double fixedTimeComputation_; //!< Time between to data capture on the OT amplifier
OTSocket::OTData otData_; //!< Structure for the OT data received
OTSocket* otSocket_; //!< Socket class for the OT biolab software
OTSocket::Config config_; //!< Configuration of the OT biolab
std::vector<std::string> nameVect_; //!< Vector of channel names


void SigintHandler ( int sig )
{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = true;
	SyncToolsIK::Shared::endThreadMutex.unlock();
}

void EMG();
void provider();

int main ( int argc, char** argv )
{
//	try
//	{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = false;
	SyncToolsIK::Shared::endThreadMutex.unlock();


	signal ( SIGINT, SigintHandler );


	_dataReady = false;

	client = new QualisysClient ( interface->getIP(), interface->getPort() );
	client->init ( interface->getLabFile() );


	_barrier = new boost::barrier ( 2 );




	// Create Socket for OT biolab
	otSocket_ = new OTSocket ( executionEMGPointer->ip(), executionEMGPointer->port() );

	// Get OT biolab configuration
	config_ = otSocket_->getConfig();

	otData_.emg.resize ( config_.nbEMG );

	// Time between two capture
	fixedTimeComputation_ = 1 / double ( config_.Samplerate );

	// Send start command to OT biolab
	otSocket_->start();

	boost::thread emgThread ( EMG );
	boost::thread providerThread ( provider );

	emgThread.join();
	providerThread.join();

	delete client;
	delete otSocket_;

	std::cout << "Quitting the main" << std::endl;

	return 0;
}

void EMG()
{
	timeval timeinitStruct, now;
	double timeEmg, timeSub, tinmeInit;
	struct timespec tim;
	OTSocket::OTData otData;

	std::ofstream emgFile ( "./Output/emg.sto" );

	VecVecDouble emgVect;
	std::vector<double> emgTime;

	_barrier->wait();

	// Filter the data
	while ( true )
	{
		//boost::timer::auto_cpu_timer auto_t;

		SyncToolsIK::Shared::endThreadMutex.lock();

		if ( SyncToolsIK::Shared::endThread )
		{
			SyncToolsIK::Shared::endThreadMutex.unlock();
			break;
		}

		SyncToolsIK::Shared::endThreadMutex.unlock();

		otData = otSocket_->readData();

		timeval tv;
		gettimeofday ( &tv, NULL );
		timeEmg = ( tv.tv_sec ) + ( 0.000001 * tv.tv_usec ) - ( otData.emg[0].size() * fixedTimeComputation_ );
		
		// Compute the base time stamp of the first captured data

		std::vector<double> emgTimeTemp;
		
		for ( int i = 0; i < otData.emg[0].size(); i++ )
		{

			std::vector<double> temp;
			std::vector<double> tempFilt;

			for ( std::vector<std::vector<float> >::const_iterator it = otData.emg.begin(); it != otData.emg.end(); it++ )
			{
				temp.push_back ( double ( ( *it ) [i] ) );
			}

			emgTimeTemp.push_back ( timeEmg + i * fixedTimeComputation_ );
			emgVect.push_back ( temp );
			emgTime.push_back ( timeEmg + i * fixedTimeComputation_ );
		}
	}

	otSocket_->disconnect();

	emgFile << "emg";
	emgFile << std::endl;

	emgFile << "nColumns=" << nameVect_.size() + 1;
	emgFile << std::endl;

	emgFile << "nRows=" << emgVect.size();
	emgFile << std::endl;

	emgFile << "endheader";
	emgFile << std::endl;

	

	emgFile << "time" << "\t";

	for ( VecStrCI it = nameVect_.begin(); it != nameVect_.end(); it++ )
	{
		emgFile << *it << "\t";
	}

	emgFile << std::endl;

	// EMG filling
	for ( VecVecDoubleCI it1 = emgVect.begin(); it1 != emgVect.end(); it1++ )
	{
		emgFile << std::fixed << std::setprecision ( 20 ) << emgTime[std::distance<VecVecDoubleCI> ( emgVect.begin(), it1 )] << "\t";

		for ( VecDoubleCI it2 = it1->begin(); it2 != it1->end(); it2++ )
			emgFile << std::fixed << std::setprecision ( 15 ) << *it2 << "\t";

		emgFile << std::endl;
	}

	

	emgFile.close();
}

void provider()
{

	double timeNow;
	std::vector<double> markerData;
	std::vector<std::vector<double> > grfData;
	std::ofstream forceFile ( "./Output/forcePlate.mot" );
	std::ofstream markerFile ( "./Output/marker.trc" );
	std::vector<double> 						markerTimeSaveInFile;
	std::vector<double> 						forcePlateTimeSaveInFile;
	std::vector< std::vector<std::vector<double> > > 	forcePlateSaveInFile;
	std::vector< std::vector<double> > 	markerSaveInFile;
	timeval now;
	gettimeofday ( &now, NULL );
	double timePast = ( now.tv_sec ) + 0.000001 * now.tv_usec;

	bool _firstPass = true;
	
	std::vector<double> h = client->getH();
	std::vector<SimTK::fVec3> forcePlateCenterGlobal = client->getForcePlateCenterGlobal();

	_barrier->wait();

	while ( true )
	{
		{
			boost::timer::auto_cpu_timer auto_t3;

			// Check if we have to stop the thread.
			SyncToolsIK::Shared::endThreadMutex.lock();

			if ( SyncToolsIK::Shared::endThread )
			{
				SyncToolsIK::Shared::endThreadMutex.unlock();
				break;
			}

			SyncToolsIK::Shared::endThreadMutex.unlock();

			client->receiveData();
			markerData = client->getData();
			grfData = client->getDataForcePlate();

			std::cout << grfData[0].size() << std::endl;

			timeval now;
			gettimeofday ( &now, NULL );
			timeNow = ( now.tv_sec ) + 0.000001 * now.tv_usec;


			markerSaveInFile.push_back ( markerData );
			markerTimeSaveInFile.push_back ( timeNow );

			double periode = ( timeNow - timePast ) / double ( grfData[0].size() );

			for ( int i = 0; i < grfData[0].size(); i++ )
			{
				SimTK::Array_<SimTK::fVec9> force;

				try
				{
					for ( int j = 0; j < client->getNumberOfForcePlate(); j++ )
						force.push_back ( grfData[j][i] );
				}
				catch ( const std::exception& ex )
				{
					break;
				}

				forcePlateSaveInFile.push_back ( force );

				if ( i == 0 )
					forcePlateTimeSaveInFile.push_back ( timePast );
				else
					forcePlateTimeSaveInFile.push_back ( forcePlateTimeSaveInFile.back() + periode );
			}

			timePast = timeNow;
		}


		{
			boost::mutex::scoped_lock lock ( _mutexProvider );
			_grfData.push ( grfData );
			_markerData.push ( markerData );
			_timeStamp.push ( timeNow );
			_dataReady 	= true;
		}

		// Notify the supervisor that data is ready.
		_condProvider.notify_one();
	}

	client->quit();

	std::cout << "Save data rovider " << std::endl;

	// GRF file header
	forceFile << "force.mot";
	forceFile << std::endl;

	forceFile << "version=1";
	forceFile << std::endl;

	forceFile << "nRows=" << forcePlateSaveInFile.size();
	forceFile << std::endl;

	forceFile << "nColumns=19";
	forceFile << std::endl;

	forceFile << "inDegrees=no";
	forceFile << std::endl;

	forceFile << "endheader";
	forceFile << std::endl;

	forceFile << "time" << "\t";
	forceFile << "ground_force_vx" << "\t" << "ground_force_vy" << "\t"
			<< "ground_force_vz" << "\t";
	forceFile << "ground_force_px" << "\t" << "ground_force_py" << "\t"
			<< "ground_force_pz" << "\t";
	forceFile << "l_ground_force_vx" << "\t" << "l_ground_force_vy" << "\t"
			<< "l_ground_force_vz" << "\t";
	forceFile << "l_ground_force_px" << "\t" << "l_ground_force_py" << "\t"
			<< "l_ground_force_pz" << "\t";
	forceFile << "ground_torque_px" << "\t" << "ground_torque_py" << "\t"
			<< "ground_torque_pz" << "\t";
	forceFile << "l_ground_torque_px" << "\t" << "l_ground_torque_py" << "\t"
			<< "l_ground_torque_pz";
	forceFile << std::endl;

	// GRF filling
	for ( std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator it1 =
			forcePlateSaveInFile.begin(); it1 != forcePlateSaveInFile.end(); it1++ )
	{
		const unsigned int& cpt = std::distance <
				std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator > (
						forcePlateSaveInFile.begin(), it1 );
		forceFile << std::fixed << std::setprecision ( 20 ) << forcePlateTimeSaveInFile[cpt] << "\t";

		for ( int i = 0; i < forcePlateSaveInFile[cpt].size(); i++ )
		{
			//Force
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][0] << "\t";
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][1] << "\t";
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][2] << "\t";
			//Position
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][6] << "\t";
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][7] << "\t";
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][8] << "\t";
		}

		for ( int i = 0; i < forcePlateSaveInFile[cpt].size(); i++ )
		{
			//Torque
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][3] << "\t";
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][4] << "\t";
			forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][5] << "\t";
		}

		forceFile << std::endl;
	}

	// TRC file header
	markerFile << "PathFileType" << "\t" << "4" << "\t" << "(X/Y/Z)" << "\t"
			<< "marker.trc";
	markerFile << std::endl;

	markerFile << "DataRate" << "\t" << "CameraRate" << "\t" << "NumFrames"
			<< "\t" << "NumMarkers" << "\t" << "Units" << "\t" << "OrigDataRate"
			<< "\t" << "OrigDataStartFrame" << "\t" << "OrigNumFrames";
	markerFile << std::endl;

	markerFile << "64\t" << "64\t" << markerSaveInFile.size() << "\t"
			<< interface->getModel()->getMarkerSet().getSize() << "\t" << "m\t"
			<< "64\t" << "1\t" << markerSaveInFile.size();
	markerFile << std::endl;

	markerFile << "Frame#" << "\t" << "time" << "\t";

	for ( int i = 0; i < interface->getModel()->getMarkerSet().getSize(); i++ )
		markerFile << interface->getModel()->getMarkerSet().get ( i ).getName()
				<< "\t\t\t";

	markerFile << std::endl;

	markerFile << "\t\t";

	for ( int i = 0; i < interface->getModel()->getMarkerSet().getSize(); i++ )
		markerFile << "x" << i << "\t" << "y" << i << "\t" << "z" << i
				<< "\t";

	markerFile << std::endl;

	markerFile << std::endl;

	// Marker filling
	for ( std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator it1 =
			markerSaveInFile.begin(); it1 != markerSaveInFile.end(); it1++ )
	{
		const unsigned int& cpt = std::distance <
				std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator > (
						markerSaveInFile.begin(), it1 );
		markerFile << cpt << "\t";
		markerFile << std::fixed << std::setprecision ( 20 ) << markerTimeSaveInFile[cpt] << "\t";

		for ( int i = 0; i < markerSaveInFile[cpt].size(); i++ )
			for ( int j = 0; j < markerSaveInFile[cpt][i].size(); j++ )
				markerFile << markerSaveInFile[cpt][i][j] << "\t";

		markerFile << std::endl;
	}



	std::cout << "Quitting the provider" << std::endl;
}
