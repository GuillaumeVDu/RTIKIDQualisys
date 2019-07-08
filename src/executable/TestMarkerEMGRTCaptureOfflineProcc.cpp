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
#include <mainwindow.h>
#include <QMainWindow>
#include "XMLInterpreterV2.h"
#include "SyncToolsIK.h"
#include <csignal>
#include <Filter.h>
#include <queue>
#include <fstream>
#include <cmath>
#include <boost/filesystem.hpp>
#include "CLIOption.h"
#include "CommonCEINMS.h"
#include "OpenSimFileLogger.h"
#include "OTSocket.h"
#include "ExecutionEmgXml.h"
#include "EMGPreProcessing.h"
#include "executionEMG.hxx"
#include "NMSmodel.hxx"

std::string executionIKFileName;
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

std::vector<SimTK::Array_<SimTK::fVec9> > 				_grfData;		//!< GRF data from the providers.
std::queue<SimTK::Array_<SimTK::fVec3> > 				_markerData;	//!< Marker data from the providers.
std::queue<unsigned long>								_framesNumber;	//!< Frames numbers from the providers.
std::queue<double> 										_timeStamp;		//!< Time stamp from the providers.
std::vector<double> 									_timeStampGrf;		//!< Time stamp from the providers.
bool 													_dataReady;		//!< Boolean for knowing if new data is availible.
boost::condition_variable 								_condProvider;	//!< Condition for waiting for new data.
boost::mutex 											_mutexProvider;	//!< Mutex for accesing the providers data.
boost::barrier* _barrier;
std::string _outDirectory;
std::string EMGFile;
std::string xmlName;
bool _record;
bool _printTime;
OTSocket::OTData otData_; //!< Structure for the OT data received
std::vector<EMGPreProcessing*> emgPreProcessingVect_; //!< Vector of class for the EMG pre-processing
OTSocket* otSocket_; //!< Socket class for the OT biolab software
OTSocket::Config config_; //!< Configuration of the OT biolab

void EMG();

std::vector<double> openSimToStd ( const SimTK::Array_<SimTK::fVec3>& data )
{
	std::vector<double> out;

	for ( SimTK::Array_<SimTK::fVec3>::const_iterator it = data.begin(); it != data.end(); it++ )
		for ( int cpt = 0; cpt < 3; cpt++ )
			out.push_back ( ( *it ) [cpt] );

	return out;
}

std::vector<double> openSimToStd ( const SimTK::Array_<SimTK::fVec9>& data )
{
	std::vector<double> out;

	for ( SimTK::Array_<SimTK::fVec9>::const_iterator it = data.begin(); it != data.end(); it++ )
	{
		for ( int cpt = 0; cpt < 3; cpt++ )
			out.push_back ( ( *it ) [cpt] );

		for ( int cpt = 6; cpt < 9; cpt++ )
			out.push_back ( ( *it ) [cpt] );
	}

	for ( SimTK::Array_<SimTK::fVec9>::const_iterator it = data.begin(); it != data.end(); it++ )
		for ( int cpt = 3; cpt < 6; cpt++ )
			out.push_back ( ( *it ) [cpt] );

	return out;
}


void SigintHandler ( int sig )
{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = true;
	SyncToolsIK::Shared::endThreadMutex.unlock();
}

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
	executionIKFileName = cliOption.getxmlIKFilePath();
	_record = cliOption.getRecord();
	_outDirectory = cliOption.getRecordFilePath();
	_printTime = cliOption.getPrintTime();
	EMGFile = cliOption.getxmlEMGFilePath();
	xmlName = cliOption.getxmlSubjectFilePath();

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

	MainWindow gui ( interface->getDt(), interface->getDofNames(), interface->getOsimFile() );

	_barrier = new boost::barrier ( 3 );

	gui.show();

	boost::thread emgThread ( EMG );
	boost::thread providerThread ( provider );
	boost::thread workerThread ( thread );

	qtApp.exec();

	emgThread.join();
	workerThread.join();
	providerThread.join();

	xercesc::XMLPlatformUtils::Terminate();

	delete interface;
	delete client;
	delete _barrier;

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
	std::vector<std::string> nameVect_;
	double pastTime, timeNow;
	bool firstPass = true;

	std::string ip = "10.76.115.143";
	std::string port = "31000";

	// Subject specific XML
	std::auto_ptr<NMSmodelType> subjectPointer;

	try
	{
		subjectPointer = std::auto_ptr<NMSmodelType> ( subject ( xmlName, xml_schema::flags::dont_initialize ) );
	}
	catch ( const xml_schema::exception& e )
	{
		COUT << e << endl;

		exit ( EXIT_FAILURE );
	}

	NMSmodelType::Channels_type& channels ( subjectPointer->Channels() );
	ChannelsType::Channel_sequence& channelSequence ( channels.Channel() );


	ExecutionEmgXml* _executionEmgXml = new ExecutionEmgXml ( EMGFile );

	// Gte IP and port
	ip = _executionEmgXml->getIP();
	port = _executionEmgXml->getPort();

	// Get filter parameters
	const std::vector<double>& aCoeffHP = _executionEmgXml->getACoeffHP();
	const std::vector<double>& bCoeffHP = _executionEmgXml->getBCoeffHP();
	const std::vector<double>& aCoeffLP = _executionEmgXml->getACoeffLP();
	const std::vector<double>& bCoeffLP = _executionEmgXml->getBCoeffLP();

	if ( aCoeffHP.size() == 0 || bCoeffHP.size() == 0 || aCoeffLP.size() == 0 || bCoeffLP.size() == 0 )
	{
		COUT << "Error: Filter coefficients size is zero." << std::endl;

		exit ( EXIT_FAILURE );
	}

	// Get Max EMG for normalization
	const std::vector<double>& maxAmp = _executionEmgXml->getMaxEmg();

	if ( maxAmp.size() == 0 )
	{
		COUT << "Error: Max EMG for Normalization size is zero." << std::endl;

		exit ( EXIT_FAILURE );
	}

	// Create Socket for OT biolab
	otSocket_ = new OTSocket ( ip, port );

	// Get OT biolab configuration
	config_ = otSocket_->getConfig();

	otData_.emg.resize ( config_.nbEMG );

	otDataFilt.emg.resize ( config_.nbEMG );

	if ( config_.nbEMG != channelSequence.size() )
	{
		COUT << "Error: Number of channel from Ot Biolab is: " << config_.nbEMG
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


	OpenSimFileLogger<int>* _logger;

	if ( _record )
	{
		_logger = new OpenSimFileLogger<int> ( _outDirectory );

		_logger->addLog ( Logger::Emgs, nameVect_ );
		_logger->addLog ( Logger::EmgsFilter, nameVect_ );
	}

	double fixedTimeComputation_ = 1 / double ( config_.Samplerate );

	// Send start command to OT biolab
	otSocket_->start();

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
		timeNow = ( tv.tv_sec ) + ( 0.000001 * tv.tv_usec );
		double periode = ( timeNow - pastTime ) / otData.emg[0].size();
		timeEmg = pastTime;

//
		if ( firstPass )
		{
			timeEmg = timeNow - ( otData.emg[0].size() * fixedTimeComputation_ );
			periode = fixedTimeComputation_;
			pastTime = false;
		}



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
			emgTimeTemp.push_back ( timeEmg + i * periode );

			if ( _record )
			{
				_logger->log ( Logger::Emgs, timeEmg + i * periode, temp );
				_logger->log ( Logger::EmgsFilter, timeEmg + i * periode, tempFilt );
			}
		}

		pastTime = timeNow;
	}

	if ( _record )
	{
		_logger->stop();
		delete _logger;
	}

	otSocket_->disconnect();

	std::vector<double> emgMax;

	for ( std::vector<std::string>::const_iterator it = nameVect_.begin(); it != nameVect_.end(); it++ )
	{
		emgMax.push_back ( emgPreProcessingVect_[std::distance<std::vector<std::string>::const_iterator> (
				nameVect_.begin(), it )]->getMax() );

// 		if ( _verbose > 1 )
		COUT << *it << " ; " << emgPreProcessingVect_[std::distance<std::vector<std::string>::const_iterator> (
				nameVect_.begin(), it )]->getMax() << "\t";

		delete emgPreProcessingVect_[std::distance<std::vector<std::string>::const_iterator> ( nameVect_.begin(), it )];
	}

	_executionEmgXml->setMaxEmg ( emgMax );
	_executionEmgXml->UpdateEmgXmlFile();

// 	if ( _verbose > 1 )
	COUT << std::endl;

	delete _executionEmgXml;


}


void provider()
{
	double timeNow;
	double periodeGrf = 0.000488281;
	SimTK::Array_<SimTK::fVec3> markerData;
	std::vector<SimTK::Array_<SimTK::fVec9> > grfData;

	OpenSimFileLogger<int>* _logger;
	boost::shared_ptr<XMLInterpreter> _xmlInterpreter = boost::shared_ptr<XMLInterpreter> ( new XMLInterpreter ( executionIKFileName ) );
	_xmlInterpreter->readXML();

	if ( _record )
	{
		_logger = new OpenSimFileLogger<int> ( _outDirectory );
		std::vector<std::string> forcePlateCol;
		forcePlateCol.push_back ( "ground_force_vx" );
		forcePlateCol.push_back ( "ground_force_vy" );
		forcePlateCol.push_back ( "ground_force_vz" );
		forcePlateCol.push_back ( "ground_force_px" );
		forcePlateCol.push_back ( "ground_force_py" );
		forcePlateCol.push_back ( "ground_force_pz" );
		forcePlateCol.push_back ( "l_ground_force_vx" );
		forcePlateCol.push_back ( "l_ground_force_vy" );
		forcePlateCol.push_back ( "l_ground_force_vz" );
		forcePlateCol.push_back ( "l_ground_force_px" );
		forcePlateCol.push_back ( "l_ground_force_py" );
		forcePlateCol.push_back ( "l_ground_force_pz" );
		forcePlateCol.push_back ( "ground_torque_px" );
		forcePlateCol.push_back ( "ground_torque_py" );
		forcePlateCol.push_back ( "ground_torque_pz" );
		forcePlateCol.push_back ( "l_ground_torque_px" );
		forcePlateCol.push_back ( "l_ground_torque_py" );
		forcePlateCol.push_back ( "l_ground_torque_pz" );
		_logger->addLog ( Logger::ForcePlate, forcePlateCol );
		_logger->addLog ( Logger::ForcePlateFilter, forcePlateCol );
		_logger->addLog ( Logger::Marker, _xmlInterpreter->getMarkersNames() );
		_logger->addLog ( Logger::MarkerFilter, _xmlInterpreter->getMarkersNames() );
	}

	timeval now;
	gettimeofday ( &now, NULL );
	double timePast = ( now.tv_sec ) + 0.000001 * now.tv_usec;

	bool _firstPass = true;
	
	_barrier->wait();


	while ( true )
	{
		{
// 		  COUT << "1" << std::endl << std::flush;

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
			grfData = client->getDataForcePlateUnfilteredVect();

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

				std::vector<float> atempGrf ( interface->getInterpreter()->getACoeffGrf().begin(), interface->getInterpreter()->getACoeffGrf().end() );
				std::vector<float> btempGrf ( interface->getInterpreter()->getBCoeffGrf().begin(), interface->getInterpreter()->getBCoeffGrf().end() );

				client->InitGRFFilter ( atempGrf, btempGrf, initGrf );

				_firstPass = false;
			}

// 			double periode = ( timeNow - timePast ) / double ( grfData.size() );


// 			timePast = timeNow;
		}
		
// 		COUT << "2" << std::endl << std::flush;

		periodeGrf = ( timeNow - timePast ) / double ( grfData.size() );
		
// 		std::cout << "periodeGrf: " << periodeGrf << std::endl << std::flush;
		
		if(grfData.size() != 0)
		  timePast = timeNow;

		if ( _record )
		{
			_logger->log ( Logger::Marker, timeNow, openSimToStd ( markerData ) );

			for ( std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator it = grfData.begin(); it != grfData.end(); it++ )
			{
				const int& cpt = std::distance<std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator> ( grfData.begin(), it );
				_logger->log ( Logger::ForcePlate, timeNow - ( grfData.size() - cpt - 1 ) * periodeGrf, openSimToStd ( *it ) );
			}
		}

		markerData = client->getDatafiltered();
		grfData = client->getDataForcePlatefilteredVect();
		


		if ( _record )
		{
			_logger->log ( Logger::MarkerFilter, timeNow, openSimToStd ( markerData ) );

			for ( std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator it = grfData.begin(); it != grfData.end(); it++ )
			{
				const unsigned int& cpt = std::distance<std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator> ( grfData.begin(), it );
// 				std::cout <<( grfData.size() - cpt ) * periodeGrf << std::endl << std::flush;
				_logger->log ( Logger::ForcePlateFilter, timeNow - ( grfData.size() - cpt - 1 ) * periodeGrf, openSimToStd ( *it ) );
			}
		}
		
		

		std::vector<double> timeGrf;

		for ( std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator it = grfData.begin(); it != grfData.end(); it++ )
		{
			const int& cpt = std::distance<std::vector<SimTK::Array_<SimTK::fVec9> >::const_iterator> ( grfData.begin(), it );
			timeGrf.push_back ( timeNow - ( grfData.size() - cpt - 1 ) * periodeGrf );
		}
		
// 		if(timeGrf.size() != 0)
// 		  std::cout << std::setprecision(15) << timeNow << " = " << timeGrf.front() << " - " << timeGrf.back() << std::endl << std::flush;
// 		else
// 		    std::cout << std::setprecision(15) << timeNow << std::endl << std::flush;
		{
			boost::mutex::scoped_lock lock ( _mutexProvider );
			_grfData.insert ( _grfData.end(), grfData.begin(), grfData.end() );
			_markerData.push ( markerData );
			_timeStamp.push ( timeNow );
			_timeStampGrf.insert ( _timeStampGrf.end(), timeGrf.begin(), timeGrf.end() );
			_dataReady 	= true;
		}

		// Notify the supervisor that data is ready.
		_condProvider.notify_all();
		
// 		COUT << "3" << std::endl << std::flush;
	}
	
	

	client->quit();

	if ( _record )
	{
		_logger->stop();
		delete _logger;
	}

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

	OpenSimFileLogger<int>* _logger;



	std::vector<SimTK::Array_<SimTK::fVec9> > grfData;		//!< GRF data from the providers.
	std::queue<SimTK::Array_<SimTK::fVec3> > 				markerData;	//!< Marker data from the providers.
	std::queue<double> timeStamp;
	std::vector<double> timeStampGrf;

	const OpenSim::CoordinateSet& coords = interface->getModel()->getCoordinateSet();
	std::vector<double> positionBase;
	std::vector<std::string> dofNames;

	for ( int i = 0; i < coords.getSize(); ++i )
	{
		positionBase.push_back ( interface->getAngle ( coords[i].getName() ) );
		dofNames.push_back ( coords.get ( i ).getName() );
	}

	if ( _record )
	{
		_logger = new OpenSimFileLogger<int> ( _outDirectory );
		_logger->addLog ( Logger::ID, dofNames );
		_logger->addLog ( Logger::IK, dofNames );
		_logger->addLog ( Logger::IKTiming );
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

			if ( _grfData.size() != 0 )
			{
				grfData 			= _grfData;
				markerData 			= _markerData;
				_grfData.clear();
				timeStamp 			= _timeStamp;
				timeStampGrf 		= _timeStampGrf;
				_timeStampGrf.clear();
				// 			std::queue<SimTK::Array_<SimTK::fVec9> > empty1;
				// 			std::swap ( _grfData, empty1 );
				std::queue<SimTK::Array_<SimTK::fVec3> > empty2;
				std::swap ( _markerData, empty2 );
				std::queue<double> empty3;
				std::swap ( _timeStamp, empty3 );
			}

			// No more data ready
			_dataReady = false;
		}

		if ( grfData.size() != 0 )
		{
		  
		  
			while ( markerData.size() > 0 )
			{
// 			boost::timer::auto_cpu_timer auto_t3;
// // 			  std::cout << "markerData.size(): " << markerData.size() << std::endl << std::flush;
// 			  std::cout << "grfData.size(): " << grfData.size() << std::endl << std::flush;
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

					if ( _record )
					{
						timeval timeNow;
						gettimeofday ( &timeNow, NULL );
						double timeDoubleNow = ( timeNow.tv_sec ) + 0.000001 * timeNow.tv_usec;
						_logger->log ( Logger::IK, timeStamp.front(), position );
						_logger->log ( Logger::IKTiming, timeStamp.front(), timeDoubleNow - timeStamp.front() );
					}
				}

				{
					std::vector<std::vector<double> > multTorque;
					std::vector<double> time;
					{
						int cpt = 0;
						
// 						std::cout << std::setprecision(15) << timeStamp.front() << " = " << timeStampGrf.front() << " - " << timeStampGrf.back() << std::endl << std::flush;

						for ( std::vector<double>::const_iterator it = timeStampGrf.begin(); it != timeStampGrf.end(); it++ )
						{
// 							cpt++;
						  
// 							std::cout << std::setprecision(15) << "timeStamp.front() == *it: " << timeStamp.front() << " == " << *it << std::endl << std::flush;

							if ( timeStamp.front() < *it )
							{
							  
// 							  std::cout << "cpt: " << cpt << std::endl << std::flush;
								break;
							}
							
							cpt++;
							
							if ( timeStamp.front() == *it )
							{
							  
// 							  std::cout << "cpt: " << cpt << std::endl << std::flush;
								break;
							}
						}
						
						if(cpt >= timeStampGrf.size())
						  cpt = timeStampGrf.size() - 1;

						timeStampGrf.erase ( timeStampGrf.begin(), timeStampGrf.begin() + cpt );

						multTorque.push_back ( interface->computeTorqueStd ( grfData[cpt], interface->getAppliedBody() ) );
						time.push_back ( timeStamp.front() );

						if ( _record )
							_logger->log ( Logger::ID, time.back(), multTorque.back() );

						grfData.erase ( grfData.begin(), grfData.begin() + cpt );
						
// 						std::cout << "grfData.size(): " << grfData.size() << std::endl << std::flush;
// 					grfData.pop();
						timeStamp.pop();

						SyncToolsIK::Shared::MultTorqueMutex.lock();
						SyncToolsIK::Shared::MultTorque.insert ( SyncToolsIK::Shared::MultTorque.end(), multTorque.begin(), multTorque.end() );
						SyncToolsIK::Shared::newMultTorqueData = true;
						SyncToolsIK::Shared::timeTorque.insert ( SyncToolsIK::Shared::timeTorque.end(), time.begin(), time.end() );
						SyncToolsIK::Shared::NbOfComputation++;
						SyncToolsIK::Shared::MultTorqueMutex.unlock();
						
						if(markerData.size() == 0)
						{
// 						  std::cout << "clear " << std::endl << std::flush;
						  grfData.clear();
						  timeStampGrf.clear();
// 						  std::cout << "grfData.size(): " << grfData.size() << std::endl << std::flush;
						}

					}
				}
			}
		}
	}

	if ( _record )
	{
		_logger->stop();
		delete _logger;
	}

	std::cout << "Quitting the Thread" << std::endl;
}

