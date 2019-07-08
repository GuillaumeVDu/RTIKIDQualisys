#include "QualisysClient.h"
#include "RTIKIDInterface.h"
#include <OpenSim/OpenSim.h>

#define timer   timer_class
#include <boost/timer.hpp>
#undef timer

#include <boost/timer/timer.hpp>
#include <boost/thread/thread.hpp>
#include <ctime>
#include <xercesc/util/PlatformUtils.hpp>

#define SIMTK
#include "XsensDriver.h"


int main(int argc, char **argv) {
	try {


		//xercesc::XMLPlatformUtils::Initialize();
		string xmlName(argv[1]);

		RTIKIDInterface interface(xmlName);

		XsensDriver xsens;

		if(xsens.initWithoutNcurses() > 0)
		{
			std::cout << "Error in initialisation of IMU." << std::endl;
			exit(1);
		}

		int Nframes = 0;
		int index = 0;
		double temp;
		double tot = 0;
		double fixedTimeComputation = interface.getDt();
		timeval tv;
		timeval now;
		gettimeofday (&tv, NULL);
		double time = (tv.tv_sec) + 0.000001 * tv.tv_usec;
		double timeNow;
		double timeConsume;
		double timeSub;
		boost::timer::auto_cpu_timer auto_t;
		struct timespec tim;

		SimTK::Array_<SimTK::dQuaternion> quat;

		cout << "Numbers of DOF: "
				<< interface.getDofNames().size() << endl;


		while(true)
		{
			boost::timer::auto_cpu_timer auto_t2;
			timeval timeConsumeStruct;
			gettimeofday (&timeConsumeStruct, NULL);
			timeConsume = (timeConsumeStruct.tv_sec) + 0.000001 * timeConsumeStruct.tv_usec;
//			interface.setTime(timeNow - timeConsume);
			do
			{
				quat = xsens.getSimTKQuaternion();
				gettimeofday (&now, NULL);
				timeNow = (now.tv_sec) + 0.000001 * now.tv_usec;
				timeSub = fixedTimeComputation - (timeNow - timeConsume);
				//std::cout << timeSub << std::endl;
			}while(timeSub > 0);
			interface.setTime(timeNow);
			interface.runIMUMarker(quat);
			interface.computeKalmanFilter();
			interface.show();
//
//			SimTK::dRotation rot(quat[0]);
////			std::cout << rot.convertRotationToBodyFixedXYZ() << std::endl;
//
//			interface.show();
//			Nframes++;
//			gettimeofday (&now, NULL);
//			timeNow = (now.tv_sec) + 0.000001 * now.tv_usec;
//			timeSub = fixedTimeComputation - (timeNow - timeConsume);
//			if(timeSub > 0)
//			{
//				tim.tv_sec = 0;
//				tim.tv_nsec = (timeSub * 1000000000L) - 80000L; // 80000 ns for te computing betwwen the time capture and the sleep
//				//cout << tim.tv_nsec << endl;
//				nanosleep(&tim, (struct timespec *)NULL);
//			}
//			else if(timeSub < 0)
//			{
//				cout << "Process use more time that specified" << endl;
//			}
		}
		cout << "Numbers of frames: " << Nframes << endl;

		//xercesc::XMLPlatformUtils::Terminate();
	} catch (const std::exception &ex) {
		cout << "Error: " << ex.what() << endl;
		return 1;
	}

	cout << "OpenSim environment simulation completed successfully." << endl;
	return 0;
}
