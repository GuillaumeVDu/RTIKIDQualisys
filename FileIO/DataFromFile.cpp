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

#include <vector>
using std::vector;
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::endl;
#include <sstream>
using std::stringstream;
#include <stdlib.h>
#include "DataFromFile.h"
#define _USE_MATH_DEFINES
#include <math.h>

//DataFromFile::DataFromFile(const DataFromFile& orig)
//:dataFile_(orig.dataFilename_.c_str()) {

//  if (!dataFile_.is_open()) {
//    cout << "ERROR: " << dataFilename << " could not be open\n";
//    exit(EXIT_FAILURE);
//  }
//  // and then you open it because ifstream cannot be used with copy constructor and operator =
//  dataFileName_ = orig.dataFileName_;
//  noMuscles_ = orig.noMuscles_;
//  muscleNames_ = orig.muscleNames_;
//  noTimeSteps_ = orig.noTimeSteps_;
//  currentDataTime_ = orig.currentDataTime_;
//  currentData_ = orig.currentData_;
//  currentTimeStep_ = orig.currentTimeStep_;
//}

DataFromFile::DataFromFile ( const string& dataFileName )
	: dataFile_ ( dataFileName.c_str() ), dataFileName_ ( dataFileName ), noTimeSteps_ ( 0 ), currentDataTime_ ( 0. ), currentTimeStep_ ( 0 )
{
	if ( !dataFile_.is_open() )
	{
		COUT << "ERROR: " << dataFileName_ << " could not be open\n";
		exit ( EXIT_FAILURE );
	}

	headerFile_.readFile ( dataFile_, dataFileName_ );
	currentData_.resize ( headerFile_.getNumberOfColumn() - 1 ); // Minus one for the time

}

void DataFromFile::readNextData()
{

	// read time for the data currently stored in DataFromFile
	string line;
	getline ( dataFile_, line, '\n' );
	stringstream myStream ( line );
	double value;
	currentData_.clear();
	myStream >>  currentDataTime_;
	
// 	COUT << dataFileName_ << " : " << value << std::endl << std::flush;
	
// 	COUT << currentDataTime_ << std::endl << std::flush;
	
	do
	{
// 		std::string word;
		value = nan("NAN");
// 		myStream >> word; //std::setprecision(std::numeric_limits<double>::digits10 + 1)
		myStream >> value;
// 		if(!word.empty())
// 		{
// 		
// 		  if(dataFileName_ == "././cfg/TMR_Heintz_16_10_15/threeDOF_EF_WP_WFNorm_Sust1_v2//emgFilt.sto")
// 		COUT << currentDataTime_ << "-" << word << ";" << std::flush;
// 		value = std::atof(word.c_str());
// 		if(dataFileName_ == "././cfg/TMR_Heintz_16_10_15/threeDOF_EF_WP_WFNorm_Sust1_v2//emgFilt.sto")
// 		std::cout <<std::setprecision(std::numeric_limits<double>::digits10 + 1) << value << std::endl << std::flush;

		if(!std::isnan(value))
		{
			if ( headerFile_.getInDegrees() )
				value = value / 180 * M_PI;

			currentData_.push_back ( value );
// 			COUT << dataFileName_ << " : " << value << std::endl << std::flush;
// 		}
		}
	}
	while ( !myStream.eof() );

	if ( currentData_.size() != headerFile_.getNumberOfColumn() - 1 )
	{
		COUT << "ERROR: in " << dataFileName_ << " at time step " << currentTimeStep_ << " you have " << currentData_.size() << " input.\nYou need " << headerFile_.getNumberOfColumn() - 1 << endl << std::flush;
		exit ( EXIT_FAILURE );
	}

	++currentTimeStep_;
}

DataFromFile::~DataFromFile()
{
	dataFile_.close();
}
