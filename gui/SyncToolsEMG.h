// This source code is part of:
// "Calibrated EMG-Informed Neuromusculoskeletal Modeling (CEINMS) Toolbox". 
// Copyright (C) 2015 David G. Lloyd, Monica Reggiani, Massimo Sartori, Claudio Pizzolato
//
// CEINMS is not free software. You can not redistribute it without the consent of the authors.
// The recipient of this software shall provide the authors with a full set of software, 
// with the source code, and related documentation in the vent of modifications and/or additional developments to CEINMS. 
//
// The methododologies and ideas implemented in this code are described in the manuscripts below, which should be cited in all publications making use of this code:
// Sartori M., Reggiani M., Farina D., Lloyd D.G., (2012) "EMG-Driven Forward-Dynamic Estimation of Muscle Force and Joint Moment about Multiple Degrees of Freedom in the Human Lower Extremity," PLoS ONE 7(12): e52618. doi:10.1371/journal.pone.0052618
// Sartori M., Farina D., Lloyd D.G., (2014) “Hybrid neuromusculoskeletal modeling to best track joint moments using a balance between muscle excitations derived from electromyograms and optimization,” J. Biomech., vol. 47, no. 15, pp. 3613–3621, 
//
// Please, contact the authors to receive a copy of the "non-disclosure" and "material transfer" agreements:
// email: david.lloyd@griffith.edu.au, monica.reggiani@gmail.com, massimo.srt@gmail.com, claudio.pizzolato.uni@gmail.com 
#ifndef SyncToolsEMG_h
#define SyncToolsEMG_h

#include <boost/thread/mutex.hpp>
#include <boost/thread/barrier.hpp>
#include <boost/thread/condition_variable.hpp>

#include "Semaphore.h"

#include <string>
using std::string;
#include <list>
using std::list;
#include <vector>
using std::vector;
#include <map>
using std::map;
#include <queue>

namespace SyncToolsEMG {
    namespace Shared{

    extern std::vector<std::vector<double> > emg;
    extern bool newEmg;
	extern std::vector<double> timeEMG;
    extern boost::mutex EMGMutex;

    };
};

#endif
