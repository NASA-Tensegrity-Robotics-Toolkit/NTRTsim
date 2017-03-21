/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 */

/**
 * @file VSpineDistribControlBending.cpp
 * @brief Implementation of a rest length controller for VSpineDistribControlModel.
 * @author Drew Sabelhaus and Ankita Joshi
 * $Id$
 */

// This module
#include "VSpineDistribControlBending.h"
// This application
#include "VSpineDistribControlModel.h"
// This library
//#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <fstream> // for the input csv file
#include <stdexcept> // for some error handling
#include <stdlib.h> // for the atof function

#include "helpers/FileHelpers.h"

VSpineDistribControlBending::VSpineDistribControlBending():
//VSpineDistribControlBending::VSpineDistribControlBending(float Kp1, float Kp2, float Kp3, float Kp4):
  verticalRLA1(4.0),
  verticalRLA2(4.0),
  verticalRLA3(4.0),
  verticalRLA4(4.0),
  verticalRLB1(4.0),
  verticalRLB2(4.0),
  verticalRLB3(4.0),
  verticalRLB4(4.0),
  dL(0.002),        // Length Change, 0.01
  state(-1.0),
  m_updateTime(0.0)
  //m_dataObserver("logs/vertspine_1-2-3-4_")
{
  // verticalRL = 1; // cm
    //verticalRL = 7.38; //cm
    //saddleRL1 = 13.037 ; // cm
    //saddleRL2 = 13.613 ; // cm
    //saddleRL3 = 14.189 ; // cm
    //saddleRL4 = 14.766 ; // cm
	/*KpA = Kp1;
	KpB = Kp2;
	KpC = Kp3;
	KpD = Kp4;*/
  
}

void VSpineDistribControlBending::onSetup(VSpineDistribControlModel& subject){
  //m_dataObserver.onSetup(subject);

  // Set up the list of cable lengths (the trajectory.)
  // DEBUGGING:
  std::cout << "Opening and reading the inverse kinematics CSV file..." << std::endl;
  // First, declare the path to the csv file as a string
  // (Ankita: I'd recommend moving this to the constructor of this class,
  // so you're able to pass it in from the App file.)
  std::string csv_path = "../../../../src/dev/ultra-spine/VSpineDistribControl/matlab_example_csv_output.csv";
  // Next, open it as an input file stream
  // This needs to be a c-style string for some reason.
  std::ifstream csv_file(csv_path.c_str());
  // If the file wasn't opened, throw an exception.
  if( !csv_file.is_open() ) {
    throw std::invalid_argument("Couldn't open the CSV file, check the path.");
  }
  // We need to declare some temporary variables to use to store the data
  //   as it's parsed:
  std::string temp_row;
  // We need to keep a boolean variable to check if the arrays have been
  // initialized yet.
  bool areInitialized = false;
  // Now, loop over the lines in the file as they're read in.
  // Note that the getline function will return 0 when there are no more lines left.
  while( std::getline(csv_file, temp_row) ) {
    // Here, temp_row has one row of the data.
    // Convert the string into a stringstream, for more parsing of the line
    // into its individual components:
    std::stringstream temp_row_stream( temp_row );
    // Loop over it and output all elements (separated by a comma).
    // Note that the variable temp_row_stream will have value 0 when it's empty.
    // Also, we need a tracking variable to index into the vector of vectors.
    int timestep = 0;
    while( temp_row_stream ) {
      // Read in the next element from the stream into a specific string:
      std::string temp_element;
      std::getline( temp_row_stream, temp_element, ',');
      // For some reason, the getline function returns an empty string
      // at the end of a row. If temp_element is empty, break out of the loop here.
      if( temp_element.empty() ) {
	break;
      }
      // TESTING: just output to the command line.
      std::cout << "Element from the input csv file: " << temp_element << std::endl;
      // Finally, append this element to the proper list in cable_trajectory.
      // The first index into cable_trajectory is timestep.
      // If this is the first row (e.g., the arrays have not been initialized),
      // we need to create a new array then append it to the array of arrays:
      if( !areInitialized ) {
	// the array for this column:
	std::vector<double> tempArray;
	// add the first element of the columnt to the new array:
	// (the atof function turns a string into a double)
	tempArray.push_back( atof(temp_element.c_str()) );
	// add this array to the array of arrays:
	cable_trajectory.push_back( tempArray );
      }
      else {
	// The arrays already exist, so we can just index into the right place:
	cable_trajectory[timestep].push_back( atof(temp_element.c_str()) );
	// increment the counter.
	// Note that above, we don't need a counter, since no indexing is needed.
	timestep += 1;
      }
    }
    // Set the flag to true now that a line is read.
    // We're technically re-setting it every time, but who cares.
    // If it's true after the first row, it's true after every row.
    areInitialized = true;
  }
  // DEBUGGING:
  std::cout << "Finished reading in the CSV file." << std::endl;
  std::cout << "There were " << cable_trajectory.size() << " timesteps and "
	    << cable_trajectory[0].size() << " rows." << std::endl;
  // DEBUGGING:
  // Output the resulting vector of vectors, just to test that it looks right.
  std::cout << "CSV values are: " << std::endl;
  // The outer loop is along the LONG dimension (e.g., how many rows there are,
  // and the inner loop along the timesteps.
  // So, arbitrarily choose the first column vector, to get the number of rows:
  for( int i=0; i < cable_trajectory[0].size(); i++ ){
 
    for( int j=0; j < cable_trajectory.size(); i++) {
      std::cout << cable_trajectory[j][i] << "," ;
    }
    // At the end of a row, put a newline.
    std::cout << std::endl;
  }
  // Temporarily: cause the simulation to stop.
  throw std::runtime_error("Break, in VSpineDistribControlBending::onSetup.");
  
}


 //Gear Ratio 1: 1-2-3-4

void VSpineDistribControlBending::onStep(VSpineDistribControlModel& subject, double dt)
 {
    //std::cout << 5555; 
    if (dt <= 0.0)
     {
         throw std::invalid_argument("dt is not positive");
     }
     else
       {
 	updateTime += dt;
 	if (updateTime >= 1.0/100)  //Speed of actuators 
 	  {
 	    updateTime = 0.0;

	    // logging
	    notifyStep(m_updateTime);
	    //m_dataObserver.onStep(subject, m_updateTime);
 	  }
	
         // First, get all muscles (cables)
         const std::vector<tgSpringCableActuator*> muscles = subject.getAllMuscles();
        
         // get all vertical muscles
         const std::vector<tgSpringCableActuator*> v_musclesA = subject.getMuscles("vertical a");
         const std::vector<tgSpringCableActuator*> v_musclesB = subject.getMuscles("vertical b");
         const std::vector<tgSpringCableActuator*> v_musclesC = subject.getMuscles("vertical c");
         const std::vector<tgSpringCableActuator*> v_musclesD = subject.getMuscles("vertical d");
        

         // set string length for vertical muscles
         
	  std::cout << "CablesA" << std::endl;
         for (size_t i = 0; i < v_musclesA.size(); ++ i)
         {
             tgSpringCableActuator* v_musclesAi = v_musclesA[i];  
             //std::cout << v_musclesAi->getRestLength() << std::endl;
             assert(v_musclesAi != NULL);
             double liA = v_musclesAi->getCurrentLength();
             //std::cout << liA << std::endl; 
	     double errorA = liA - 5; 
	     double riA = v_musclesAi->getRestLength();
             riA = riA - 0.0001 * errorA; 
	     /*if (riA <= 0.0) {
		throw 0;
	     } */
             std::cout << riA << std::endl;
             v_musclesAi->setControlInput(riA, dt);
	} 
	  
	/*
	//std::cout << "CablesB" << std::endl;
	  for (size_t i = 0; i < v_musclesB.size(); ++ i)
	{
	     tgSpringCableActuator* v_musclesBi = v_musclesB[i];  
             //std::cout << v_musclesAi->getRestLength() << std::endl;
             assert(v_musclesBi != NULL);
             double liB = v_musclesBi->getCurrentLength();
             //std::cout << liB << std::endl; 
	     double errorB = liB - 5; 
	     double riB = v_musclesBi->getRestLength();
             riB = riB - 0.01 * errorB; 
	   /*  if (riB <= 0.0) {
		throw 1;
	     } 
             //std::cout << riB << std::endl;
             v_musclesBi->setControlInput(riB, dt); 
         } 

	 //std::cout << "CablesC" << std::endl;
         for (size_t i = 0; i < v_musclesC.size(); ++ i)
         {
             tgSpringCableActuator* v_musclesCi = v_musclesC[i];  
             //std::cout << v_musclesAi->getRestLength() << std::endl;
             assert(v_musclesCi != NULL);
             double liC = v_musclesCi->getCurrentLength();
             //std::cout << liC << std::endl; 
	     double errorC = liC - 5; 
	     double riC = v_musclesCi->getRestLength();
             riC = riC - 0.01 * errorC;
	    /* if (riC <= 0.0) {
		throw 2;
	     } 
             //std::cout << riC << std::endl;
             v_musclesCi->setControlInput(riC, dt);
	} 
	
	//std::cout << "CablesD" << std::endl;
         for (size_t i = 0; i < v_musclesD.size(); ++ i)
         {
             tgSpringCableActuator* v_musclesDi = v_musclesD[i];  
             //std::cout << v_musclesAi->getRestLength() << std::endl;
             assert(v_musclesDi != NULL);
             double liD = v_musclesDi->getCurrentLength();
             //std::cout << liD << std::endl; 
	     double errorD = liD - 5; 
	     double riD = v_musclesDi->getRestLength();
             riD = riD - 0.01 * errorD; 
	    /* if (riD <= 0.0) {
		throw 3;
	     } 
             //std::cout << riD << std::endl;
             v_musclesDi->setControlInput(riD, dt);
	} */
     }
}





