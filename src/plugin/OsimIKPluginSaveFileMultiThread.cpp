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

#include "OsimIKPluginSaveFileMultiThread.h"

OsimIKPlugin::OsimIKPlugin()
{
	_verbose = 1;
	_record = true;
}

OsimIKPlugin::~OsimIKPlugin()
{

}

void OsimIKPlugin::stop()
{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = true;
	SyncToolsIK::Shared::endThreadMutex.unlock();
	delete _ikIDcomp;
}

void OsimIKPlugin::init ( string xmlName, string executionName )
{
	SyncToolsIK::Shared::endThreadMutex.lock();
	SyncToolsIK::Shared::endThread = false;
	SyncToolsIK::Shared::endThreadMutex.unlock();

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
		COUT << e << endl;
		exit ( EXIT_FAILURE );
	}

	// Get the XML filename for the IK
	const std::string& AngleFile = executionPointer->ConsumerPlugin().AngleDeviceFile().get();

	_ikIDcomp = new IKAndIDComputation ( AngleFile );
	_ikIDcomp->setDirectory ( _outDirectory );
	_ikIDcomp->setRecord ( _record );
	_ikIDcomp->setVerbose ( _verbose );

	XMLInterpreter xmlInterpreter ( AngleFile );
	xmlInterpreter.readXML();


	_translate = new TranslateOpenSimCEINMS ( xmlInterpreter.getTranslateFile() );

	_dofNameVect = _ikIDcomp->getDOFName();
	_dofNameSet = std::set<std::string> ( _dofNameVect.begin(), _dofNameVect.end() );

	for ( std::vector<std::string>::const_iterator it = _dofNameVect.begin(); it < _dofNameVect.end(); it++ )
		_pastAngleData.push_back ( 0 );

	_ikIDcomp->start();
	
}

const map<string, double>& OsimIKPlugin::GetDataMap()
{
	timeval tv;

	// Get the data from the boost thread
	std::vector<std::vector<double> > angleData;
	std::vector<double> tempTime;

	angleData = _ikIDcomp->getIKData();

	if ( angleData.size() != 0 )
	{
		tempTime = _ikIDcomp->getTimeIK();

		if ( tempTime.size() != 0 )
			_timeCurrent = tempTime.back();

		_pastAngleData = angleData.back();
	}

	// Fill the map
	for ( VecStrCI it = _dofNameVect.begin(); it < _dofNameVect.end(); it++ )
	{
		try
		{
			_mapAngleDataToDofName[_translate->OpenSimToCEINMS ( *it )] = _pastAngleData[std::distance<VecStrCI> ( _dofNameVect.begin(), it )];
		}
		catch ( const std::out_of_range& oor )
		{
			COUT << "out of range IK" << std::endl;
			continue;
		}
	}
	
	return _mapAngleDataToDofName;
}

const map<string, double>& OsimIKPlugin::GetDataMapTorque()
{
	std::vector<std::vector<double> > TorqueData = _ikIDcomp->getIDData();

	if ( TorqueData.size() != 0 )
		_pastTorqueData = TorqueData.back();

	for ( VecStrCI it = _dofNameVect.begin(); it < _dofNameVect.end(); it++ )
	{
		try
		{
			double data = _pastTorqueData.at(std::distance<VecStrCI> ( _dofNameVect.begin(), it ));
			_torque[_translate->OpenSimToCEINMS ( *it )] = data;
		}
		catch ( const std::out_of_range& oor )
		{
			COUT << *it << std::endl;
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
