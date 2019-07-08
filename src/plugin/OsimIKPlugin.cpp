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

#include "OsimIKPlugin.h"

OsimIKPlugin::OsimIKPlugin()
{

}

OsimIKPlugin::~OsimIKPlugin()
{

}

void OsimIKPlugin::init(string xmlName, string executionName)
{
	// Get the execution XML files
	std::auto_ptr<ExecutionType> executionPointer;
	try
	{
		std::auto_ptr<ExecutionType> temp(
				execution(executionName, xml_schema::flags::dont_initialize));
		executionPointer = temp;
	} catch (const xml_schema::exception& e)
	{
		cout << e << endl;
		exit(EXIT_FAILURE);
	}

	// Get the XML filename for the IK
	const string& AngleFile =
			executionPointer->ConsumerPlugin().AngleDeviceFile().get();

	// Create the interface for the IK and Kalman
	interface_ = new RTIKIDInterface(AngleFile);

	interface_->show();

	// Create the client for the Qualisys motion capture system
	client_ = new QualisysClient(interface_->getIP(), interface_->getPort());
	client_->init(interface_->getLabFile());

// 	std::cout << interface_->getTranslateFile() << std::endl;

	// Create the translation class for CEINMS and OpenSim
	translate_ = new TranslateOpenSimCEINMS(interface_->getTranslateFile());

	// Create the joint name set
	for (int i = 0; i < interface_->getModel()->getCoordinateSet().getSize();
			i++)
	{
		const string& jointName =
				interface_->getModel()->getCoordinateSet().get(i).getName();
		try
		{
			nameSet_.insert(translate_->OpenSimToCEINMS(jointName));
		} catch (const std::out_of_range& oor)
		{
			std::cout << "Joint no found in translate file: " << jointName
					<< std::endl;
		}

		angleData_.push_back(0);
	}

	// Get the Kalman dt
	fixedTimeComputation_ = interface_->getDt();

	threadEnd_ = true;

// 	timeval tv;
// 	gettimeofday(&tv, NULL);
// 	timeInit_ = (tv.tv_sec) + (0.000001 * tv.tv_usec);
	
	timeInit_ = -1;

	// Create the boost thread
	filterThread = new boost::thread(
			boost::bind(&OsimIKPlugin::filterAngle, this));
}

void OsimIKPlugin::filterAngle()
{
	timeval timeinitStruct, now;
	double timeNow, timeSub, tinmeInit;
	struct timespec tim;
	std::vector<double> angleData;

	while (threadEnd_)
	{
		// get the initial time
		gettimeofday(&timeinitStruct, NULL);
		tinmeInit = (timeinitStruct.tv_sec) + 0.000001 * timeinitStruct.tv_usec;

		// get the data and process it
		client_->receiveData();
		interface_->runIKMarker(client_->getData());
// 		interface_->computeKalmanFilter();

		angleData.clear();

		// fill the data vector
		for (int i = 0;
				i < interface_->getModel()->getCoordinateSet().getSize(); i++)
		{
			const string& jointName =
					interface_->getModel()->getCoordinateSet().get(i).getName();
			angleData.push_back(interface_->getAngle(jointName));
		}
		angleDataMutex_.lock();
		angleData_ = angleData;
		angleDataMutex_.unlock();

// 		interface_->show();

		// Sleep until the dt time is realized
		// this is for the kalman filter
// 		gettimeofday(&now, NULL);
// 		timeNow = (now.tv_sec) + 0.000001 * now.tv_usec;
// 		timeSub = fixedTimeComputation_ - (timeNow - tinmeInit);
		/*if (timeSub > 0)
			boost::this_thread::sleep(
					boost::posix_time::nanoseconds(
							(timeSub * 1000000000L) - 80000L));*/
	}
}

const map<string, double>& OsimIKPlugin::GetDataMap()
{
	timeval tv;
	std::vector<double> angleData;

	// Get the data from the boost thread
	angleDataMutex_.lock();
	angleData = angleData_;
	angleDataMutex_.unlock();

	// Fill the map
	for (int i = 0; i < interface_->getModel()->getCoordinateSet().getSize();
			i++)
	{
		const string& jointName =
				interface_->getModel()->getCoordinateSet().get(i).getName();
		try
		{
			mapData_[translate_->OpenSimToCEINMS(jointName)] = angleData[i];
		} catch (const std::out_of_range& oor)
		{

		}
	}

	// Get the current time for the time stamp on the data
	gettimeofday(&tv, NULL);
	time_ = (tv.tv_sec) + 0.000001 * tv.tv_usec;
	return mapData_;
}

// For the run-time loading of the library

extern "C" ProducersPluginVirtual* create()
{
	return new OsimIKPlugin;
}

extern "C" void destroy(ProducersPluginVirtual* p)
{
	delete p;
}
