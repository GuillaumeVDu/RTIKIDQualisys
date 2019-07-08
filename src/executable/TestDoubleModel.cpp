#include "IKAndIDComputation.h"
#include "RTIKIDInterface.h"

#define timer   timer_class
#include <boost/timer.hpp>
#undef timer

#include <boost/timer/timer.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <ctime>

#include <QApplication>
#include <mainwindow2Model.h>
#include <QMainWindow>
#include "XMLInterpreterV2.h"
#include "SyncToolsIK.h"
#include <csignal>
#include <xercesc/util/PlatformUtils.hpp>
#include <OpenSim/OpenSim.h>
#include "XMLInterpreterV2.h"

std::vector<double> initvalue;

void SigintHandler ( int sig )
{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = true;
	SyncToolsIK::Shared::endThreadMutex.unlock();
}

void thread ( std::string executionIKFileName );

int main ( int argc, char** argv )
{
	std::cout << "\033[1;31mMain: " << "\033[0m" << std::endl;
//	try
//	{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = false;
	SyncToolsIK::Shared::endThreadMutex.unlock();

	xercesc::XMLPlatformUtils::Initialize(); // for the thread safe of the parsing of the XML use xml_schema::flags::dont_initialize when parsing.


	signal ( SIGINT, SigintHandler );
#if QT_VERSION < QT_VERSION_CHECK(5, 0, 0)
	QApplication::setGraphicsSystem ( "raster" );
#endif
	QApplication qtApp ( argc, argv );

	std::string executionIKFileName = argv[1];

	SyncToolsIK::Shared::positionMutex.lock();
	SyncToolsIK::Shared::newPositionData = false;
	SyncToolsIK::Shared::positionMutex.unlock();
	SyncToolsIK::Shared::torqueMutex.lock();
	SyncToolsIK::Shared::newTorqueData = false;
	SyncToolsIK::Shared::torqueMutex.unlock();

	boost::shared_ptr<XMLInterpreter> xmlInterpreter = boost::shared_ptr<XMLInterpreter> ( new XMLInterpreter ( executionIKFileName ) );
	xmlInterpreter->readXML();

	OpenSim::Model* model = new OpenSim::Model ( xmlInterpreter->getOsimFile() );

	const OpenSim::CoordinateSet& coordSet = model->getCoordinateSet();
	std::vector<string> 				dofNames;
	
	

	for ( int i = 0; i < coordSet.getSize(); i++ )
	{
		dofNames.push_back ( coordSet.get ( i ).getName() );
		initvalue.push_back(coordSet.get ( i ).get_default_value());
	}

	MainWindow gui ( 0, dofNames, xmlInterpreter->getOsimFile() );
//
	delete model;
	xmlInterpreter.reset();

	boost::thread workerThread ( thread, executionIKFileName );

	gui.show();



	qtApp.exec();

	workerThread.join();

	xercesc::XMLPlatformUtils::Terminate();

	return 0;
}

void thread ( std::string executionIKFileName )
{
	std::cout << "\033[1;31mthread: " << "\033[0m" << std::endl;
	IKAndIDComputation comp ( executionIKFileName );
	
	std::vector<std::vector<double> > tempVect;
	std::vector<double> tempTime;
	
	comp.start();

	while ( true )
	{
		SyncToolsIK::Shared::endThreadMutex.lock();

		if ( SyncToolsIK::Shared::endThread )
		{
			SyncToolsIK::Shared::endThreadMutex.unlock();
			comp.stop();
			break;
		}

		SyncToolsIK::Shared::endThreadMutex.unlock();

		SyncToolsIK::Shared::positionMutex.lock();
		tempVect = comp.getIKData();
		if(tempVect.size() != 0)
		{
			for(int j = 0; j < 6; j++)
			{
				tempVect.back().at(j) = initvalue.at(j);
			}
			SyncToolsIK::Shared::position = tempVect.back();
			SyncToolsIK::Shared::newPositionData = true;
		}
		comp.getTimeIK();
		SyncToolsIK::Shared::positionMutex.unlock();

		SyncToolsIK::Shared::MultTorqueMutex.lock();
		tempVect = comp.getIDData();
		SyncToolsIK::Shared::MultTorque.insert(SyncToolsIK::Shared::MultTorque.end(), tempVect.begin(), tempVect.end());
		tempTime = comp.getTimeID();
		SyncToolsIK::Shared::timeTorque.insert(SyncToolsIK::Shared::timeTorque.end(), tempTime.begin(), tempTime.end());
		SyncToolsIK::Shared::newMultTorqueData = true;
		SyncToolsIK::Shared::MultTorqueMutex.unlock();

		boost::this_thread::sleep ( boost::posix_time::milliseconds ( 30 ) ); // 33 Hz

	}
	std::cout << "\033[1;31mquit\033[0m" << std::endl;
}
