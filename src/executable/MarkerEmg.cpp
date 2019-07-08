#include "QualisysClient.h"
#include "RTIKIDInterface.h"
#include <OpenSim/OpenSim.h>

#define timer   timer_class
#include <boost/timer.hpp>
#undef timer

#include <boost/timer/timer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <ctime>
#include <xercesc/util/PlatformUtils.hpp>

#include <QApplication>
#include <mainwindowEMG.h>
#include <QMainWindow>
#include "XMLInterpreterV2.h"
#include "SyncToolsIK.h"
#include "SyncToolsEMG.h"
#include <csignal>
#include <Filter.h>
#include <queue>
#include <fstream>
#include <cmath>
#include "OTSocket.h"
#include "EMGPreProcessing.h"
#include "executionEMG.hxx"
#include "NMSmodel.hxx"
#include "ExecutionEmgXml.h"
#include <boost/filesystem.hpp>
#include "CLIOption.h"

RTIKIDInterface* interface;
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
std::string _outDirectory;
bool _record;
bool _printTime;


std::queue<SimTK::Array_<SimTK::fVec9> > 				_grfData;		//!< GRF data from the providers.
std::queue<SimTK::Array_<SimTK::fVec3> > 				_markerData;	//!< Marker data from the providers.
std::queue<unsigned long>								_framesNumber;	//!< Frames numbers from the providers.
std::queue<double> 										_timeStamp;		//!< Time stamp from the providers.
bool 													_dataReady;		//!< Boolean for knowing if new data is availible.
boost::condition_variable 								_condProvider;	//!< Condition for waiting for new data.
boost::mutex 											_mutexProvider;	//!< Mutex for accesing the providers data.



double fixedTimeComputation_; //!< Time between to data capture on the OT amplifier
OTSocket::OTData otData_; //!< Structure for the OT data received
std::vector<EMGPreProcessing*> emgPreProcessingVect_; //!< Vector of class for the EMG pre-processing
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
void thread();
void provider();

int main ( int argc, char** argv )
{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = false;
	SyncToolsIK::Shared::endThreadMutex.unlock();

	xercesc::XMLPlatformUtils::Initialize();

	signal ( SIGINT, SigintHandler );
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	QApplication::setGraphicsSystem ( "raster" );
#endif
	QApplication qtApp ( argc, argv );

	CLIOption cliOption ( argc, argv );

	const std::string& executionIKFileName = cliOption.getxmlIKFilePath();
	const std::string& EMGFile = cliOption.getxmlEMGFilePath();
	const std::string& xmlName = cliOption.getxmlSubjectFilePath();

	_record = cliOption.getRecord();
	_outDirectory = cliOption.getRecordFilePath();
	_printTime = cliOption.getPrintTime();

	SyncToolsIK::Shared::positionMutex.lock();
	SyncToolsIK::Shared::newPositionData = false;
	SyncToolsIK::Shared::positionMutex.unlock();
	SyncToolsIK::Shared::torqueMutex.lock();
	SyncToolsIK::Shared::newTorqueData = false;
	SyncToolsIK::Shared::torqueMutex.unlock();
	SyncToolsIK::Shared::NbOfComputation = 0;

	_dataReady = false;

	interface = new RTIKIDInterface ( executionIKFileName );
	client = new QualisysClient ( interface->getIP(), interface->getPort() );
	client->setMarkerNames ( interface->getMarkersNames() );
	client->init ( interface->getLabFile() );

	const OpenSim::CoordinateSet& coordSet = interface->updModel()->getCoordinateSet();

	for ( int i = 0; i < coordSet.getSize(); i++ )
		dofName.push_back ( coordSet.get ( i ).getName() );

	_barrier = new boost::barrier ( 3 );

	// Subject specific XML
	std::auto_ptr<NMSmodelType> subjectPointer;

	try
	{
		subjectPointer = std::auto_ptr<NMSmodelType> ( subject ( xmlName, xml_schema::flags::dont_initialize ) );
	}
	catch ( const xml_schema::exception& e )
	{
		cout << e << endl;
		exit ( EXIT_FAILURE );
	}

	NMSmodelType::Channels_type& channels ( subjectPointer->Channels() );
	ChannelsType::Channel_sequence& channelSequence ( channels.Channel() );

	ExecutionEmgXml* _executionEmgXml = new ExecutionEmgXml ( EMGFile );

	// Get filter parameters
	const std::vector<double>& aCoeffHP = _executionEmgXml->getACoeffHP();
	const std::vector<double>& bCoeffHP = _executionEmgXml->getBCoeffHP();
	const std::vector<double>& aCoeffLP = _executionEmgXml->getACoeffLP();
	const std::vector<double>& bCoeffLP = _executionEmgXml->getBCoeffLP();

	if ( aCoeffHP.size() == 0 || bCoeffHP.size() == 0 || aCoeffLP.size() == 0 || bCoeffLP.size() == 0 )
	{
		std::cout << "Error: Filter coefficients size is zero." << std::endl;
		exit ( EXIT_FAILURE );
	}

	// Get Max EMG for normalization
	const std::vector<double>& maxAmp = _executionEmgXml->getMaxEmg();

	if ( maxAmp.size() == 0 )
	{
		std::cout << "Error: Max EMG for Normalization size is zero." << std::endl;
		exit ( EXIT_FAILURE );
	}

	// Create Socket for OT biolab
	otSocket_ = new OTSocket ( _executionEmgXml->getIP(), _executionEmgXml->getPort() );

	// Get OT biolab configuration
	config_ = otSocket_->getConfig();

	otData_.emg.resize ( config_.nbEMG );

	if ( config_.nbEMG != channelSequence.size() )
	{
		std::cout << "Error: Number of channel from Ot Biolab is: " << config_.nbEMG
				<< " and number of channel from file is: " << channelSequence.size()
				<< " Numbers of channel have to be the same." << std::endl;
		exit ( EXIT_FAILURE );
	}

	// Create the EMG pre-processing class
	for ( ChannelsType::Channel_iterator it = channelSequence.begin(); it != channelSequence.end(); it++ )
	{
		nameVect_.push_back ( it->name() );
		emgPreProcessingVect_.push_back (
			new EMGPreProcessing ( aCoeffLP, bCoeffLP, aCoeffHP, bCoeffHP,
					maxAmp[std::distance<ChannelsType::Channel_iterator> ( channelSequence.begin(), it )] ) );
		otData_.emg[std::distance<ChannelsType::Channel_iterator> ( channelSequence.begin(), it )].push_back ( 0 );
	}

	// Time between two capture
	fixedTimeComputation_ = 1 / double ( config_.Samplerate );

	// Send start command to OT biolab
	otSocket_->start();

	MainWindow gui ( interface->getDt(), interface->getDofNames(), interface->getOsimFile(), nameVect_ );

	gui.show();

	boost::thread emgThread ( EMG );
	boost::thread providerThread ( provider );
	boost::thread workerThread ( thread );

	qtApp.exec();

	emgThread.join();
	workerThread.join();
	providerThread.join();

	std::cout << "Max Emg: ";
	std::vector<double> emgMax;

	for ( std::vector<std::string>::const_iterator it = nameVect_.begin(); it != nameVect_.end(); it++ )
	{
		emgMax.push_back ( emgPreProcessingVect_[std::distance<std::vector<std::string>::const_iterator> (
				nameVect_.begin(), it )]->getMax() );
		std::cout << *it << " ; " << emgPreProcessingVect_[std::distance<std::vector<std::string>::const_iterator> (
				nameVect_.begin(), it )]->getMax() << "\t";
		delete emgPreProcessingVect_[std::distance<std::vector<std::string>::const_iterator> ( nameVect_.begin(), it )];
	}

	std::cout << std::endl;

	_executionEmgXml->setMaxEmg ( emgMax );
	_executionEmgXml->UpdateEmgXmlFile();

	delete _executionEmgXml;
	delete interface;
	delete client;
	delete otSocket_;



	xercesc::XMLPlatformUtils::Terminate();

	std::cout << "Quitting the main" << std::endl;

	return 0;
}

void EMG()
{
	timeval timeinitStruct, now;
	double timeEmg, timeSub, tinmeInit;
	struct timespec tim;
	OTSocket::OTData otData;
	OTSocket::OTData otDataFilt;
	otDataFilt.emg.resize ( config_.nbEMG );

	std::ofstream* _emgFiltFile;
	std::ofstream* _emgFile;

	if ( _record )
	{

		std::stringstream ss;
		ss << "./";
		ss << _outDirectory;
		boost::filesystem::path dir ( ss.str().c_str() );

		if ( !boost::filesystem::exists ( dir ) )
			if ( !boost::filesystem::create_directory ( dir ) )
				std::cout << "Error in creating directory: " << ss.str() << std::endl;

		ss << "/emgFilt.sto";
		_emgFiltFile = new std::ofstream ( ss.str().c_str() );

		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/emg.sto";
		_emgFile = new std::ofstream ( ss.str().c_str() );
	}

	VecVecDouble emgVect;
	VecVecDouble emgFiltVect;
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

		// The data is send by packet of fixed data for each EMG channel
		for ( std::vector<std::vector<float> >::const_iterator it = otData.emg.begin(); it != otData.emg.end(); it++ )
		{
			const int& cpt1 = std::distance<std::vector<std::vector<float> >::const_iterator> ( otData.emg.begin(), it );
			otDataFilt.emg[cpt1].clear();

			for ( std::vector<float>::const_iterator it2 = it->begin(); it2 != it->end(); it2++ )
			{
				const int& cpt2 = std::distance<std::vector<float>::const_iterator> ( it->begin(), it2 );
				otDataFilt.emg[cpt1].push_back ( emgPreProcessingVect_[cpt1]->computeData ( *it2 ) );
			}
		}

		timeval tv;
		gettimeofday ( &tv, NULL );
		timeEmg = ( tv.tv_sec ) + ( 0.000001 * tv.tv_usec ) - ( otData.emg[0].size() * fixedTimeComputation_ );

		// Compute the base time stamp of the first captured data

		std::vector<std::vector<double> > emgFiltTemp;
		std::vector<double> emgTimeTemp;

		for ( int i = 0; i < otData.emg[0].size(); i++ )
		{

			std::vector<double> temp;
			std::vector<double> tempFilt;

			for ( std::vector<std::vector<float> >::const_iterator it = otData.emg.begin(); it != otData.emg.end(); it++ )
			{
				temp.push_back ( double ( ( *it ) [i] ) );
				tempFilt.push_back ( double ( otDataFilt.emg[std::distance<std::vector<std::vector<float> >::const_iterator> ( otData.emg.begin(), it )][i] ) );
			}

			emgFiltTemp.push_back ( tempFilt );
			emgTimeTemp.push_back ( timeEmg + i * fixedTimeComputation_ );

			if ( _record )
			{
				emgVect.push_back ( temp );
				emgFiltVect.push_back ( tempFilt );
			}

			emgTime.push_back ( timeEmg + i * fixedTimeComputation_ );
		}

		SyncToolsEMG::Shared::EMGMutex.lock();
		SyncToolsEMG::Shared::emg.insert ( SyncToolsEMG::Shared::emg.end(), emgFiltTemp.begin(), emgFiltTemp.end() );
		SyncToolsEMG::Shared::newEmg = true;
		SyncToolsEMG::Shared::timeEMG.insert ( SyncToolsEMG::Shared::timeEMG.end(), emgTimeTemp.begin(), emgTimeTemp.end() );
		SyncToolsEMG::Shared::EMGMutex.unlock();
	}

	otSocket_->disconnect();


	if ( _record )
	{
		*_emgFile << "emg";
		*_emgFile << std::endl;

		*_emgFile << "nColumns=" << nameVect_.size() + 1;
		*_emgFile << std::endl;

		*_emgFile << "nRows=" << emgVect.size();
		*_emgFile << std::endl;

		*_emgFile << "endheader";
		*_emgFile << std::endl;

		*_emgFiltFile << "emgFilt";
		*_emgFiltFile << std::endl;

		*_emgFiltFile << "nColumns=" << nameVect_.size() + 1;
		*_emgFiltFile << std::endl;

		*_emgFiltFile << "nRows=" << emgFiltVect.size();
		*_emgFiltFile << std::endl;

		*_emgFiltFile << "endheader";
		*_emgFiltFile << std::endl;

		*_emgFile << "time" << "\t";
		*_emgFiltFile << "time" << "\t";

		for ( VecStrCI it = nameVect_.begin(); it != nameVect_.end(); it++ )
		{
			*_emgFile << *it << "\t";
			*_emgFiltFile << *it << "\t";
		}

		*_emgFile << std::endl;
		*_emgFiltFile << std::endl;

		// EMG filling
		for ( VecVecDoubleCI it1 = emgVect.begin(); it1 != emgVect.end(); it1++ )
		{
			*_emgFile << std::fixed << std::setprecision ( 20 ) << emgTime[std::distance<VecVecDoubleCI> ( emgVect.begin(), it1 )] << "\t";

			for ( VecDoubleCI it2 = it1->begin(); it2 != it1->end(); it2++ )
				*_emgFile << std::fixed << std::setprecision ( 15 ) << *it2 << "\t";

			*_emgFile << std::endl;
		}

		// EMG filt filling
		for ( VecVecDoubleCI it1 = emgFiltVect.begin(); it1 != emgFiltVect.end(); it1++ )
		{
			*_emgFiltFile << emgTime[std::distance<VecVecDoubleCI> ( emgFiltVect.begin(), it1 )] << "\t";

			for ( VecDoubleCI it2 = it1->begin(); it2 != it1->end(); it2++ )
				*_emgFiltFile << std::fixed << std::setprecision ( 15 ) << *it2 << "\t";

			*_emgFiltFile << std::endl;
		}

		_emgFile->close();
		_emgFiltFile->close();
		delete _emgFile;
		delete _emgFiltFile;

	}
}

void provider()
{
	std::vector<std::vector<fFilter*> > markerfFilter; // 3 (X,Y,Z) * nb of marker
	std::vector<std::vector<fFilter*> > grffFilter; // 6 (force and torque) * nb of plate

	double timeNow;
	SimTK::Array_<SimTK::fVec3> markerData;
	SimTK::Array_<SimTK::fVec9> grfData;
	std::ofstream* forceFile;
	std::ofstream* markerFile;
	std::ofstream* forceFileFilt;
	std::ofstream* markerFileFilt;

	if ( _record )
	{
		std::stringstream ss;
		ss << "./";
		ss << _outDirectory;
		boost::filesystem::path dir ( ss.str().c_str() );

		if ( !boost::filesystem::exists ( dir ) )
			if ( !boost::filesystem::create_directory ( dir ) )
				std::cout << "Error in creating directory: " << ss.str() << std::endl;


		ss << "/forcePlate.mot";
		forceFile = new std::ofstream ( ss.str().c_str() );

		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/marker.trc";
		markerFile = new std::ofstream ( ss.str().c_str() );

		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/forcePlateFilt.mot";
		forceFileFilt = new std::ofstream ( ss.str().c_str() );

		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/markerFilt.trc";
		markerFileFilt = new std::ofstream ( ss.str().c_str() );
	}

	std::vector<double> 						markerTimeSaveInFile;
	std::vector<double> 						forcePlateTimeSaveInFile;
	std::vector< SimTK::Array_<SimTK::fVec9> > 	forcePlateSaveInFile;
	std::vector< SimTK::Array_<SimTK::fVec3> > 	markerSaveInFile;
	std::vector< SimTK::Array_<SimTK::fVec9> > 	forcePlateFiltSaveInFile;
	std::vector< SimTK::Array_<SimTK::fVec3> > 	markerFiltSaveInFile;
	timeval now;
	gettimeofday ( &now, NULL );
	double timePast = ( now.tv_sec ) + 0.000001 * now.tv_usec;

	bool _firstPass = true;

	_barrier->wait();

	while ( true )
	{
		{
			if ( _printTime )
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
			markerData = client->getDataUnfiltered();
			grfData = client->getDataForcePlateUnfiltered();

			timeval now;
			gettimeofday ( &now, NULL );
			timeNow = ( now.tv_sec ) + 0.000001 * now.tv_usec;

			if ( _firstPass )
			{
				std::vector<std::vector<std::vector<float> > > initPosition ( interface->getMarkersNames().size() );

				for ( int i = 0 ; i < interface->getMarkersNames().size(); i++ )
				{
					initPosition[i].resize ( 3 );

					if ( std::isnan ( markerData[i][0] ) || std::isnan ( markerData[i][1] ) || std::isnan ( markerData[i][2] ) )
					{
						SimTK::Vec3 marker = interface->updModel()->getMarkerSet().get ( i ).getOffset();
						const OpenSim::SimbodyEngine& engine = interface->updModel()->getSimbodyEngine();
						interface->updModel()->getMultibodySystem().realize ( * ( interface->updState() ), SimTK::Stage::Position );
						engine.transformPosition ( *interface->updState(), interface->updModel()->getMarkerSet().get ( i ).getBody(), marker, engine.getGroundBody(), marker );
						initPosition[i][0].resize ( interface->getInterpreter()->getBCoeffMarker().size(), marker[0] );
						initPosition[i][1].resize ( interface->getInterpreter()->getBCoeffMarker().size(), marker[1] );
						initPosition[i][2].resize ( interface->getInterpreter()->getBCoeffMarker().size(), marker[2] );
					}
					else
					{
						initPosition[i][0].resize ( interface->getInterpreter()->getBCoeffMarker().size(), markerData[i][0] );
						initPosition[i][1].resize ( interface->getInterpreter()->getBCoeffMarker().size(), markerData[i][1] );
						initPosition[i][2].resize ( interface->getInterpreter()->getBCoeffMarker().size(), markerData[i][2] );
					}
				}

				markerfFilter.resize ( interface->getMarkersNames().size() );

				std::vector<float> atempMarker ( interface->getInterpreter()->getACoeffMarker().begin(), interface->getInterpreter()->getACoeffMarker().end() );
				std::vector<float> btempMarker ( interface->getInterpreter()->getBCoeffMarker().begin(), interface->getInterpreter()->getBCoeffMarker().end() );

				client->InitMarkerFilter ( atempMarker, btempMarker, initPosition );

				std::vector<std::vector<std::vector<float> > > initGrf ( client->getNumberOfForcePlate() );

				for ( int i = 0; i < client->getNumberOfForcePlate(); i++ )
				{
					initGrf[i].resize ( 6 );

					for ( int j = 0; j < 6; j++ )
					{
						initGrf[i][j].resize ( interface->getInterpreter()->getBCoeffGrf().size(), 0 );
					}
				}

				grffFilter.resize ( client->getNumberOfForcePlate() );

				std::vector<float> atempGrf ( interface->getInterpreter()->getACoeffGrf().begin(), interface->getInterpreter()->getACoeffGrf().end() );
				std::vector<float> btempGrf ( interface->getInterpreter()->getBCoeffGrf().begin(), interface->getInterpreter()->getBCoeffGrf().end() );

				client->InitGRFFilter ( atempGrf, btempGrf, initGrf );

				_firstPass = false;
			}


			if ( _record )
			{
				markerSaveInFile.push_back ( markerData );
				markerTimeSaveInFile.push_back ( timeNow );
			}

			if ( _record )
			{
				forcePlateSaveInFile.push_back ( grfData );
				forcePlateTimeSaveInFile.push_back ( timePast );
			}

			timePast = timeNow;
		}

		markerData = client->getDatafiltered();
		grfData = client->getDataForcePlatefiltered();

		if ( _record )
		{
			forcePlateFiltSaveInFile.push_back ( grfData );
			markerFiltSaveInFile.push_back ( markerData );
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

	if ( _record )
	{

		std::cout << "Save data Provider " << std::endl;

		// GRF file header
		*forceFile << "force.mot";
		*forceFile << std::endl;

		*forceFile << "version=1";
		*forceFile << std::endl;

		*forceFile << "nRows=" << forcePlateSaveInFile.size();
		*forceFile << std::endl;

		*forceFile << "nColumns=19";
		*forceFile << std::endl;

		*forceFile << "inDegrees=no";
		*forceFile << std::endl;

		*forceFile << "endheader";
		*forceFile << std::endl;

		*forceFile << "time" << "\t";
		*forceFile << "ground_force_vx" << "\t" << "ground_force_vy" << "\t"
				<< "ground_force_vz" << "\t";
		*forceFile << "ground_force_px" << "\t" << "ground_force_py" << "\t"
				<< "ground_force_pz" << "\t";
		*forceFile << "l_ground_force_vx" << "\t" << "l_ground_force_vy" << "\t"
				<< "l_ground_force_vz" << "\t";
		*forceFile << "l_ground_force_px" << "\t" << "l_ground_force_py" << "\t"
				<< "l_ground_force_pz" << "\t";
		*forceFile << "ground_torque_px" << "\t" << "ground_torque_py" << "\t"
				<< "ground_torque_pz" << "\t";
		*forceFile << "l_ground_torque_px" << "\t" << "l_ground_torque_py" << "\t"
				<< "l_ground_torque_pz";
		*forceFile << std::endl;

		// GRF filling
		for ( std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator it1 =
				forcePlateSaveInFile.begin(); it1 != forcePlateSaveInFile.end(); it1++ )
		{
			const unsigned int& cpt = std::distance <
					std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator > (
							forcePlateSaveInFile.begin(), it1 );
			*forceFile << std::fixed << std::setprecision ( 20 ) << forcePlateTimeSaveInFile[cpt] << "\t";

			for ( int i = 0; i < forcePlateSaveInFile[cpt].size(); i++ )
			{
				//Force
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][0] << "\t";
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][1] << "\t";
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][2] << "\t";
				//Position
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][6] << "\t";
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][7] << "\t";
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][8] << "\t";
			}

			for ( int i = 0; i < forcePlateSaveInFile[cpt].size(); i++ )
			{
				//Torque
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][3] << "\t";
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][4] << "\t";
				*forceFile << std::fixed << std::setprecision ( 10 ) << forcePlateSaveInFile[cpt][i][5] << "\t";
			}

			*forceFile << std::endl;
		}

		// TRC file header
		*markerFile << "PathFileType" << "\t" << "4" << "\t" << "(X/Y/Z)" << "\t"
				<< "marker.trc";
		*markerFile << std::endl;

		*markerFile << "DataRate" << "\t" << "CameraRate" << "\t" << "NumFrames"
				<< "\t" << "NumMarkers" << "\t" << "Units" << "\t" << "OrigDataRate"
				<< "\t" << "OrigDataStartFrame" << "\t" << "OrigNumFrames";
		*markerFile << std::endl;

		*markerFile << "64\t" << "64\t" << markerSaveInFile.size() << "\t"
				<< interface->getModel()->getMarkerSet().getSize() << "\t" << "m\t"
				<< "64\t" << "1\t" << markerSaveInFile.size();
		*markerFile << std::endl;

		*markerFile << "Frame#" << "\t" << "time" << "\t";

		for ( int i = 0; i < interface->getModel()->getMarkerSet().getSize(); i++ )
			*markerFile << interface->getModel()->getMarkerSet().get ( i ).getName()
					<< "\t\t\t";

		*markerFile << std::endl;

		*markerFile << "\t\t";

		for ( int i = 0; i < interface->getModel()->getMarkerSet().getSize(); i++ )
			*markerFile << "x" << i << "\t" << "y" << i << "\t" << "z" << i
					<< "\t";

		*markerFile << std::endl;

		*markerFile << std::endl;

		// Marker filling
		for ( std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator it1 =
				markerSaveInFile.begin(); it1 != markerSaveInFile.end(); it1++ )
		{
			const unsigned int& cpt = std::distance <
					std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator > (
							markerSaveInFile.begin(), it1 );
			*markerFile << cpt << "\t";
			*markerFile << std::fixed << std::setprecision ( 20 ) << markerTimeSaveInFile[cpt] << "\t";

			for ( int i = 0; i < markerSaveInFile[cpt].size(); i++ )
				for ( int j = 0; j < markerSaveInFile[cpt][i].size(); j++ )
					*markerFile << markerSaveInFile[cpt][i][j] << "\t";

			*markerFile << std::endl;
		}


		// TRC file header
		*markerFileFilt << "PathFileType" << "\t" << "4" << "\t" << "(X/Y/Z)" << "\t"
				<< "markerFilt.trc";
		*markerFileFilt << std::endl;

		*markerFileFilt << "DataRate" << "\t" << "CameraRate" << "\t" << "NumFrames"
				<< "\t" << "NumMarkers" << "\t" << "Units" << "\t" << "OrigDataRate"
				<< "\t" << "OrigDataStartFrame" << "\t" << "OrigNumFrames";
		*markerFileFilt << std::endl;

		*markerFileFilt << "64\t" << "64\t" << markerFiltSaveInFile.size() << "\t"
				<< interface->getModel()->getMarkerSet().getSize() << "\t" << "m\t"
				<< "64\t" << "1\t" << markerFiltSaveInFile.size();
		*markerFileFilt << std::endl;

		*markerFileFilt << "Frame#" << "\t" << "time" << "\t";

		for ( int i = 0; i < interface->getModel()->getMarkerSet().getSize(); i++ )
			*markerFileFilt << interface->getModel()->getMarkerSet().get ( i ).getName()
					<< "\t\t\t";

		*markerFileFilt << std::endl;

		*markerFileFilt << "\t\t";

		for ( int i = 0; i < interface->getModel()->getMarkerSet().getSize(); i++ )
			*markerFileFilt << "x" << i << "\t" << "y" << i << "\t" << "z" << i
					<< "\t";

		*markerFileFilt << std::endl;

		*markerFileFilt << std::endl;

		// Marker filling
		for ( std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator it1 =
				markerFiltSaveInFile.begin(); it1 != markerFiltSaveInFile.end(); it1++ )
		{
			const unsigned int& cpt = std::distance <
					std::vector<SimTK::Array_<SimTK::fVec3> >::const_iterator > (
							markerFiltSaveInFile.begin(), it1 );
			*markerFileFilt << cpt << "\t";
			*markerFileFilt << std::fixed << std::setprecision ( 20 ) << markerTimeSaveInFile[cpt] << "\t";

			for ( int i = 0; i < markerFiltSaveInFile[cpt].size(); i++ )
				for ( int j = 0; j < markerFiltSaveInFile[cpt][i].size(); j++ )
					*markerFileFilt << markerFiltSaveInFile[cpt][i][j] << "\t";

			*markerFileFilt << std::endl;
		}

		// GRF file header
		*forceFileFilt << "force.mot";
		*forceFileFilt << std::endl;

		*forceFileFilt << "version=1";
		*forceFileFilt << std::endl;

		*forceFileFilt << "nRows=" << forcePlateFiltSaveInFile.size();
		*forceFileFilt << std::endl;

		*forceFileFilt << "nColumns=19";
		*forceFileFilt << std::endl;

		*forceFileFilt << "inDegrees=no";
		*forceFileFilt << std::endl;

		*forceFileFilt << "endheader";
		*forceFileFilt << std::endl;

		*forceFileFilt << "time" << "\t";
		*forceFileFilt << "ground_force_vx" << "\t" << "ground_force_vy" << "\t"
				<< "ground_force_vz" << "\t";
		*forceFileFilt << "ground_force_px" << "\t" << "ground_force_py" << "\t"
				<< "ground_force_pz" << "\t";
		*forceFileFilt << "l_ground_force_vx" << "\t" << "l_ground_force_vy" << "\t"
				<< "l_ground_force_vz" << "\t";
		*forceFileFilt << "l_ground_force_px" << "\t" << "l_ground_force_py" << "\t"
				<< "l_ground_force_pz" << "\t";
		*forceFileFilt << "ground_torque_px" << "\t" << "ground_torque_py" << "\t"
				<< "ground_torque_pz" << "\t";
		*forceFileFilt << "l_ground_torque_px" << "\t" << "l_ground_torque_py" << "\t"
				<< "l_ground_torque_pz";
		*forceFileFilt << std::endl;

		// GRF filling
		for ( std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator it1 =
				forcePlateFiltSaveInFile.begin(); it1 != forcePlateFiltSaveInFile.end(); it1++ )
		{
			const unsigned int& cpt = std::distance <
					std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator > (
							forcePlateFiltSaveInFile.begin(), it1 );
			*forceFileFilt << std::fixed << std::setprecision ( 20 ) << forcePlateTimeSaveInFile[cpt] << "\t";

			for ( int i = 0; i < forcePlateFiltSaveInFile[cpt].size(); i++ )
			{
				//Force
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][0] << "\t";
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][1] << "\t";
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][2] << "\t";
				//Position
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][6] << "\t";
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][7] << "\t";
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][8] << "\t";
			}

			for ( int i = 0; i < forcePlateFiltSaveInFile[cpt].size(); i++ )
			{
				//Torque
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][3] << "\t";
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][4] << "\t";
				*forceFileFilt << std::fixed << std::setprecision ( 10 ) << forcePlateFiltSaveInFile[cpt][i][5] << "\t";
			}

			*forceFileFilt << std::endl;
		}

		forceFileFilt->close();
		delete forceFileFilt;
		markerFileFilt->close();
		delete markerFileFilt;
		forceFile->close();
		delete forceFile;
		markerFile->close();
		delete markerFile;
	}

	for ( int i = 0; i < interface->getMarkersNames().size(); i++ )
		for ( int j = 0; j < 3; j++ )
			delete markerfFilter[i][j];

	for ( int i = 0; i < client->getNumberOfForcePlate(); i++ )
		for ( int j = 0; j < 6; j++ )
			delete grffFilter[i][j];

	std::cout << "Quitting the provider" << std::endl;
}

void thread()
{
	double fixedTimeComputation = interface->getDt();
	timeval tv;
	timeval now;
	gettimeofday ( &tv, NULL );
	double time = ( tv.tv_sec ) + 0.000001 * tv.tv_usec;
	double timeNow;
	double timeConsume;
	double timeSub;

	if ( _printTime )
		boost::timer::auto_cpu_timer auto_t;

	struct timespec tim;
	timeval timeConsumeStruct;

	std::ofstream* IKFile;
	std::ofstream* IDFile;

	if ( _record )
	{
		std::stringstream ss;
		ss << "./";
		ss << _outDirectory;
		boost::filesystem::path dir ( ss.str().c_str() );

		ss << "/ik.mot";
		IKFile = new std::ofstream ( ss.str().c_str() );

		ss.clear();
		ss.str ( std::string() );
		ss << "./";
		ss << _outDirectory;
		ss << "/id.mot";
		IDFile = new std::ofstream ( ss.str().c_str() );
	}

	std::vector< std::vector<double> > 			_ikSaveInFile;
	std::vector< std::vector<double> > 			_idSaveInFile;
	std::vector<double> 						_ikTimeSaveInFile;
	std::vector<double> 						_idTimeSaveInFile;

	std::queue<SimTK::Array_<SimTK::fVec9> > 				grfData;	//!< GRF data from the providers.
	std::queue<SimTK::Array_<SimTK::fVec3> > 				markerData;	//!< Marker data from the providers.
	std::queue<double> timeStamp;

	const OpenSim::CoordinateSet& coords = interface->getModel()->getCoordinateSet();
	std::vector<double> positionBase;

	for ( int i = 0; i < coords.getSize(); ++i )
	{
		positionBase.push_back ( interface->getAngle ( coords[i].getName() ) );
	}

	_barrier->wait();

	while ( true )
	{
// 			boost::timer::auto_cpu_timer auto_t3;
		SyncToolsIK::Shared::endThreadMutex.lock();

		if ( SyncToolsIK::Shared::endThread )
		{
			SyncToolsIK::Shared::endThreadMutex.unlock();
			break;
		}

		SyncToolsIK::Shared::endThreadMutex.unlock();

		{
			boost::mutex::scoped_lock lock ( _mutexProvider );

			while ( !_dataReady ) _condProvider.wait ( lock );

			// Get the data from the provider.
			markerData 			= _markerData;
			grfData 			= _grfData;
			timeStamp 			= _timeStamp;
			std::queue<SimTK::Array_<SimTK::fVec9> > empty1;
			std::swap ( _grfData, empty1 );
			std::queue<SimTK::Array_<SimTK::fVec3> > empty2;
			std::swap ( _markerData, empty2 );
			std::queue<double> empty3;
			std::swap ( _timeStamp, empty3 );

			// No more data ready
			_dataReady = false;
		}

		while ( grfData.size() > 0 )
		{
// 			boost::timer::auto_cpu_timer auto_t3;

			try
			{
				interface->runIKMarker ( markerData.front() );
				markerData.pop();
			}
			catch ( const std::exception& ex )
			{
				SyncToolsIK::Shared::endThreadMutex.lock();
				SyncToolsIK::Shared::endThread = true;
				SyncToolsIK::Shared::endThreadMutex.unlock();
				break;
			}

			{
				{
					interface->computeKalmanFilter();
				}
				const OpenSim::CoordinateSet& coords = interface->getModel()->getCoordinateSet();
				std::vector<double> position;
				{

					for ( int i = 0; i < coords.getSize(); ++i )
						position.push_back ( interface->getAngle ( coords[i].getName() ) );

					SyncToolsIK::Shared::positionMutex.lock();
					SyncToolsIK::Shared::position = position;
					SyncToolsIK::Shared::newPositionData = true;
					SyncToolsIK::Shared::positionMutex.unlock();
				}

				_ikSaveInFile.push_back ( position );
				_ikTimeSaveInFile.push_back ( timeStamp.front() );
			}

			{
				std::vector<std::vector<double> > multTorque;
				std::vector<double> time;

				multTorque.push_back ( interface->computeTorqueStd ( grfData.front(), interface->getAppliedBody() ) );
				time.push_back ( timeStamp.front() );
				_idSaveInFile.push_back ( multTorque.back() );
				_idTimeSaveInFile.push_back ( time.back() );

				grfData.pop();
				timeStamp.pop();

				SyncToolsIK::Shared::MultTorqueMutex.lock();
				SyncToolsIK::Shared::MultTorque.insert ( SyncToolsIK::Shared::MultTorque.end(), multTorque.begin(), multTorque.end() );
				SyncToolsIK::Shared::newMultTorqueData = true;
				SyncToolsIK::Shared::timeTorque.insert ( SyncToolsIK::Shared::timeTorque.end(), time.begin(), time.end() );
				SyncToolsIK::Shared::NbOfComputation++;
				SyncToolsIK::Shared::MultTorqueMutex.unlock();

			}
		}
	}

	if ( _record )
	{

		std::cout << "Save data thread " << std::endl;

		*IKFile << "Coordinates";
		*IKFile << std::endl;

		*IKFile << "version=1";
		*IKFile << std::endl;

		*IKFile << "nRows=" << _ikSaveInFile.size();
		*IKFile << std::endl;

		*IKFile << "nColumns=" << dofName.size() + 1;
		*IKFile << std::endl;

		*IKFile << "inDegrees=no";
		*IKFile << std::endl;

		*IKFile << "endheader";
		*IKFile << std::endl;

		*IKFile << "time\t";

		for ( VecStrCI it = dofName.begin(); it != dofName.end(); it++ )
			*IKFile << *it << "\t";

		*IKFile << std::endl;

		// IK fill
		for ( VecVecDoubleCI it1 = _ikSaveInFile.begin(); it1 != _ikSaveInFile.end();
				it1++ )
		{
			*IKFile << std::fixed << std::setprecision ( 20 )
					<< _ikTimeSaveInFile[std::distance<VecVecDoubleCI> ( _ikSaveInFile.begin(),
							it1 )] << "\t";

			for ( VecDoubleCI it2 = it1->begin(); it2 != it1->end(); it2++ )
				*IKFile << std::fixed << std::setprecision ( 10 ) << *it2 << "\t";

			*IKFile << std::endl;
		}

		IKFile->close();
		delete IKFile;


		// ID header
		*IDFile << "Inverse Dynamics Generalized Forces";
		*IDFile << std::endl;

		*IDFile << "version=1";
		*IDFile << std::endl;

		*IDFile << "nRows=" << _idSaveInFile.size();
		*IDFile << std::endl;

		*IDFile << "nColumns=" << dofName.size() + 1;
		*IDFile << std::endl;

		*IDFile << "inDegrees=no";
		*IDFile << std::endl;

		*IDFile << "endheader";
		*IDFile << std::endl;

		*IDFile << "time\t";

		for ( VecStrCI it = dofName.begin(); it != dofName.end(); it++ )
			*IDFile << *it << "\t";

		*IDFile << std::endl;

		// ID fill
		for ( VecVecDoubleCI it1 = _idSaveInFile.begin(); it1 != _idSaveInFile.end(); it1++ )
		{
			*IDFile << std::fixed << std::setprecision ( 20 ) << _idTimeSaveInFile[std::distance<VecVecDoubleCI> ( _idSaveInFile.begin(), it1 )] << "\t";

			for ( VecDoubleCI it2 = it1->begin(); it2 != it1->end(); it2++ )
				*IDFile << std::fixed << std::setprecision ( 10 ) << *it2 << "\t";

			*IDFile << std::endl;
		}

		IDFile->close();
		delete IDFile;

	}

	std::cout << "Quitting the Thread" << std::endl;
}
