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

#ifndef OSIMIKPLUGIN_H_
#define OSIMIKPLUGIN_H_

#include "InvKinInvDynRealTime.h"
#include "execution.hxx"
#include "ProducersPluginVirtual.h"
#include "QualisysClient.h"
#include "RTIKIDInterface.h"
#include <OpenSim/OpenSim.h>
#include "TranslateOpenSimCEINMS.h"
#include <ctime>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/filesystem.hpp>

/**
 * Plugin for CEINMS-RT for real-time use of the IK using Qualisys markers
 */

class OsimIKPlugin: public ProducersPluginVirtual
{
	public:
		/**
		 * Constructor
		 */
		OsimIKPlugin();

		/**
		 * Destructor
		 */
		virtual ~OsimIKPlugin();

		/**
		 * Initialization method
		 * @param xmlName Subject specific XMl filename, not use here (we use the osim files)
		 * @param executionName Execution xml filename for CEINMS-RT
		 */
		void init ( string xmlName, string executionName );

		/**
		 * Get the data with the name of the joint mapping the angle
		 */
		const map<string, double>& GetDataMap();

		const double& getTime()
		{
			return time_;
		}

		/**
		 * Get the name of the joint. Warning this a set, so there is not order
		 */
		const set<string>& GetNameSet()
		{
			return nameSet_;
		}

		void stop()
		{
			threadEnd_ = false;
			filterThread->join();
			std::cout << "\033[1;32mIK thread end\033[0m" << std::endl;
			delete filterThread;
			delete interface_;
			delete client_;
			delete translate_;
		}

		void setDirectory ( std::string outDirectory, std::string inDirectory = std::string() )
		{
			_outDirectory = outDirectory;
			_inDirectory = inDirectory;
		}

		void setVerbose ( int verbose )
		{
			_verbose = verbose;
		}

		void setRecord ( bool record )
		{
			_record = record;
		}

		const map<string, double>& GetDataMapTorque()
		{
			return _torque;
		}

	protected:

		/**
		 * boost thread method for the kalman filter with respect to the timing
		 */
		void filterAngle();

		map<string, double> mapData_; //!< Map for the data the name of the joint is mapping the angle
		set<string> nameSet_; //!< Set of the joint name
		RTIKIDInterface* interface_; //!< Class for all the IK and kalman computation
		QualisysClient* client_; //!< Class for connecting to the Qualisys motion capture system
		TranslateOpenSimCEINMS* translate_; //!< class for translation between CEINMS name and OpenSim name
		double time_; //!< The current time of the data
		double timeInit_;
		bool threadEnd_; //!< Bool for ending the threading of filterAngle method
		boost::mutex angleDataMutex_; //!< Mutex for accessing the data
		std::vector<double> angleData_; //!< Vector of the angle for passing the data between the thread and the GetDataMap method
		boost::thread* filterThread; //!< Pointer to boost thread
		double fixedTimeComputation_; //!< fixed time computation for the Kalman filter
		std::string _outDirectory;
		std::string _inDirectory;
		bool _record;
		int _verbose;
		map<string, double> _torque;

};

#endif /* OSIMIKPLUGIN_H_ */
