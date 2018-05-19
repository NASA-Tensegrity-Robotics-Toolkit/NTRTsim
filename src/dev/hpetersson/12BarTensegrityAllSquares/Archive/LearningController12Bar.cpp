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
 * @file LearningController12Bar.cpp
 * @brief A controller for the 12 Bar octahedron tensegrity model 
 * @author Hannah Petersson, based on code from Brian Tietz, Drew Sabelhaus and Mallory Daly
 * @version 1.0.0
 * $Id$
 */

// This module
#include "LearningController12Bar.h" /* Change .h file */

//This application
#include "yamlbuilder/TensegrityModel.h"

// Should include tgString, but compiler complains since its been
// included from BaseSpineModelLearning. Perhaps we should move things
// to a cpp over there

// This library
#include "core/tgSpringCableActuator.h"
#include "controllers/tgImpedanceController.h"  /* Is this necessary? */
#include "core/tgBasicActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"
//#include "tgDataObserver.h"

// Learning libraries
#include "learning/AnnealEvolution/AnnealEvolution.h"
#include "learning/Configuration/configuration.h"

// The C++ standard library
#include <string>
#include <cassert>
#include <stdexcept>
//#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"

//#define LOGGING

using namespace std;

LearningController12Bar::Config::Config(					int ss,
                                        					int tm,
                                                                                int om,
                                                                                int param,
                                                                                int segnum,
                                                                                double ct,
                                                                        double la,
                                                                                double ha,
                                                                                double lp,
                                                                                double hp,
                                                                                double kt,
                                                                                double kp,
                                                                                double kv,
                                                                                bool def,
                                                                                double cl,
                                                                                double lf,
                                                                                double hf) :
 	segmentSpan(ss),
        theirMuscles(tm),
        ourMuscles(om),
        params(param),
        segmentNumber(segnum),
        controlTime(ct),
        lowAmp(la),
        highAmp(ha),
        lowPhase(lp),
        highPhase(hp),
        tension(kt),
        kPosition(kp),
        kVelocity(kv),
        useDefault(def),
        controlLength(cl),
        lowFreq(lf),
        highFreq(hf)
{
    if (ss <= 0)
    {
        throw std::invalid_argument("segmentSpan parameter is negative.");
    }
    else if (tm <= 0)
    {
        throw std::invalid_argument("theirMuscles parameter is negative.");
    }
    else if (om <= 0)
    {
        throw std::invalid_argument("Our Muscles parameter is negative.");
    }
   else if (param <= 0)
    {
        throw std::invalid_argument("Edge parameters is negative.");
    }
    else if (segnum < 0)
    {
        throw std::invalid_argument("Segment number is negative.");
    }
    else if (ct < 0.0)
    {
        throw std::invalid_argument("control time is negative.");
    }
    else if (kt < 0.0)
    {
        throw std::invalid_argument("impedance control tension is negative.");
    }
    else if (kp < 0.0)
    {
        throw std::invalid_argument("impedance control position is negative.");
    }
    else if (kv < 0.0)
    {
        throw std::invalid_argument("impedance control velocity is negative.");
    }
    else if (cl < 0.0)
    {
        throw std::invalid_argument("Control Length is negative.");
    }
}


/*
 * Defining the adapters here assumes the controller is around and
 * attached for the lifecycle of the learning runs. I.E. that the setup
 * and teardown functions are used for tgModel
 */
LearningController12Bar::LearningController12Bar(LearningController12Bar::Config config, double startTime, double minLength,
		 		double rate, std::vector<std::string> tagsToControl, std::string args, std::string resourcePath,
				        std::string ec, std::string nc):

  m_config(config),
  m_startTime(startTime),
  m_minLength(minLength),
  m_rate(rate),
  m_tagsToControl(tagsToControl),
  m_timePassed(0.0),
  edgeConfigFilename(ec),
  nodeConfigFilename(nc),

  // Evolution assumes no pre-processing was done on these names
  edgeEvolution(args + "_edge", ec, resourcePath),

  // Can't have identical args or they'll overwrite each other
  nodeEvolution(args + "_node", nc, resourcePath),

  // Will be overwritten by configuration data
  nodeLearning(true),  					/* Should this be 'true' if machine learning is activated? */
  edgeLearning(true),
  m_dataObserver("logs/TCData"),
//  m_pCPGSys(NULL),
  m_updateTime(0.0),
  bogus(false)
{

  // Sanity checks:
  // Start time must be greater than or equal to zero
  if ( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // Min length must be between 0 and 1 (it is a percentage) 
  else if( m_minLength > 1) {
    throw std::invalid_argument("min_Length is a percent, must be less than 1. (100%)");
  }
  else if( m_minLength < 0.0 ) {
    throw std::invalid_argument("min_Length is a percent, must be greater than 0.");
  }
  // Rate must be greater than zero
  else if( m_rate < 0.0 ) {
    throw std::invalid_argument("Rate cannot be negative.");
  }

	// Get resource path
	std::string path;
	if (resourcePath != "")
	{
		path = FileHelpers::getResourcePath(resourcePath);
	}
	else
	{
		path = "";
	}
	
    nodeConfigData.readFile(path + nodeConfigFilename);
     edgeConfigData.readFile(path + edgeConfigFilename);
    nodeLearning = nodeConfigData.getintvalue("learning");
    edgeLearning = edgeConfigData.getintvalue("learning");

}

LearningController12Bar::~LearningController12Bar() 
{
    scores.clear();
}

/**
 * The initializeActuators method is called in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void LearningController12Bar::initializeActuators(TensegrityModel& subject, std::string tag) {

  //DEBUGGING
  std::cout << "Finding cables with the tag: " << tag << std::endl;
  
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  std::cout << "The following cables were found and will be controlled: " << std::endl;

  // If no actuators are found
  if (foundActuators.size() == 0){
    std::cout << "No actuators found." << std::endl;
  }

  // Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i++ ) {
    //std::cout << foundActuators[i] -> getTags() << std::endl; // This just prints 'cable'
    // Add the rest length of the actuator at this time to the list of all initial rest lengths
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    // DEBUGGING
    std::cout << "Cable " << i << " with rest length " << initialRL[foundActuators[i]->getTags()] << " at t = 0." << std::endl;
  }
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  cablesWithTags.insert( cablesWithTags.end(), foundActuators.begin(),
                         foundActuators.end() );
}


void LearningController12Bar::onSetup(TensegrityModel& subject)	 
{
    //Initialize the Learning Adapters
    nodeAdapter.initialize(&nodeEvolution,
                            nodeLearning,
                            nodeConfigData);
    edgeAdapter.initialize(&edgeEvolution,
                            edgeLearning,
                            edgeConfigData);

    /* Empty vector signifying no state information
     * All parameters are stateless parameters, so we can get away with
     * only doing this once
     */
    std::vector<double> state;
    double dt = 0;
    
    array_4D edgeParams = scaleEdgeActions(edgeAdapter.step(dt, state));
    array_2D nodeParams = scaleNodeActions(nodeAdapter.step(dt, state));
    
   setupController(subject, nodeParams, edgeParams);				/* Change function to setupController or similar */
    
    //initConditions = subject.getSegmentCOM(m_config.segmentNumber);   	/* This might work with the same function */

	#ifdef LOGGING // Conditional compile for data logging    
    	m_dataObserver.onSetup(subject);
	#endif    
    
	#if (0) // Conditional Compile for debug info
    	std::cout << *m_pCPGSys << std::endl;
	#endif    
    
    m_updateTime = 0.0;			
    bogus = false;
}

void LearningController12Bar::setupController(TensegrityModel& subject, array_2D nodeActions, array_4D edgeActions) 
{										
	    							
 std::cout << "Setting up the LearningController12Bar controller." << std::endl;
 // std::cout << "Finding cables with tags: " << m_tagsToControl << std::endl;
  cablesWithTags = {};
 
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag
    initializeActuators(subject, *it);
  }   
  
  // Initialize flags 
  m_retract = 1; // Begin in tetract mode 
  m_finished = 0; // True when finished retracting and returning all cables 
  
  // Initialize cable index
  m_cable_index = 0;
  
  // Output that controller setup is complete
  std::cout << "Finished setting up the controller." << std::endl;
	
}

/**
 * Changes the cables' lengths at some specified timestep.
 * @param[in] subject - the TensegrityModel that is being controlled. Must
 * have a list of allMuscles populated
 * @param[in] dt, current timestep must be positive
 */
void LearningController12Bar::onStep(TensegrityModel& subject, double dt) {
  

 
 // First, increment the accumulator variable
  m_timePassed += dt;

  if( std::fmod(m_timePassed, 1.0) == 1 )
  {
    std::cout << "Time passed: " << m_timePassed << std::endl;
  }

  // If it is past the time to start the controller, start
  if( m_timePassed > m_startTime ) 
  {
    // Retract mode (retract each cable in sequence)
    if( m_retract == 1 ) {
      int i = m_cable_index; // Grab cable index. Unnecessary?
   //  std::cout << "Cable index: " << m_cable_index << std::endl;
//       std::cout << "Entered retract mode for cable index: " << i << std::endl; // Debugging 

      double currRestLength = cablesWithTags[i]->getRestLength(); // Grab current rest length
  //    std::cout << "Current rest length is: " << currRestLength << std::endl;

      double minRestLength = initialRL[cablesWithTags[i]->getTags()] * m_minLength; // Calculate the minimum rest length for this cab$
    //  std::cout << "Minimum rest length is: " << minRestLength << std::endl;

 // If the current rest length is greater than the desired minimum
      if( currRestLength > minRestLength ) {
        // Output a progress bar for the controller, to track when controll occurs
      //  std::cout << "Control occurs, cable index: " << i << std::endl;
        // Adjust rest length of the actuator
        double nextRestLength = currRestLength - m_rate*dt;

        // DEBUGGING
        //std::cout << "m_rate:  " << m_rate << std::endl;
        //std::cout << "dt: " << dt << std::endl;
        //std::cout << "Next rest length: " << nextRestLength << std::endl;
       
        cablesWithTags[i]->setControlInput(nextRestLength, dt); 
      }
      else {
         // Cable has been retracted to min length, go to next cable
        m_cable_index = m_cable_index + 1;
        std::cout << "Cable finished retracting, go to cable with index: " << m_cable_index << std::endl;
        // If the cable index is equal to the number of cables, all cables have been retracted
        // Move to return state
        if( m_cable_index == cablesWithTags.size() ) {
          m_cable_index = 0;
          m_retract = 0;
        }
      }
    }
   
  // Return state
  else if ( m_finished == 0 ) {
    std::cout << "Made it to return state." << std::endl;
    int i = m_cable_index; // Grab cable index 
    std::cout << "Cable index is: " << i << std::endl;
    double currRestLength = cablesWithTags[i]->getRestLength(); // Grab current rest length
    double initialRestLength = initialRL[cablesWithTags[i]->getTags()]; // Calculate initial rest length for this cable
    // If the current rest length is below initial rest length
    if( currRestLength < initialRestLength ) {
      // Output a progress bar for the controller to track when control occurs
      //std::cout << "Control occured. Cable index: " << i << std::endl;
      double nextRestLength = currRestLength = m_rate*dt; // Adjust rest length of the actuator 
      // DEBUGGING
      //std::cout << "Next rest length: " << nextRestLength << std::endl;
      cablesWithTags[i]->setControlInput(nextRestLength, dt);
    } 
    else {
      // Cable has been retracted to min length, go to next cable
      m_cable_index ++; // OBS OBS OBS
      std::cout << "Cable index: " << m_cable_index << std::endl;
      // If the cable index is equal to the number of cables, all cables have been retracted. Move to return state. 
      if( m_cable_index == cablesWithTags.size() ) { 
        m_finished = 1;
      }
    } 
  }
}
}

void LearningController12Bar::onTeardown(TensegrityModel& subject)  /* This function is where the fun stuff happens */
{									/* Carefully study it. */
    //scores.clear();
    // @todo - check to make sure we ran for the right amount of time
    /*
    std::vector<double> finalConditions = subject.getSegmentCOM(m_config.segmentNumber);
    
    const double newX = finalConditions[0];
    const double newZ = finalConditions[2];
    const double oldX = initConditions[0];
    const double oldZ = initConditions[2];
    
    const double distanceMoved = sqrt((newX-oldX) * (newX-oldX) + 
                                        (newZ-oldZ) * (newZ-oldZ));
    
    if (bogus)
    {
		scores.push_back(-1.0);
    }
    else
    {
		scores.push_back(distanceMoved);
	}
    
    /// @todo - consolidate with other controller classes. 
    /// @todo - return length scale as a parameter
    double totalEnergySpent=0;
    
    std::vector<tgSpringCableActuator* > tmpStrings = subject.getAllMuscles();
    
    for(int i=0; i<tmpStrings.size(); i++)
    {
        tgSpringCableActuator::SpringCableActuatorHistory stringHist = tmpStrings[i]->getHistory();
        
        for(int j=1; j<stringHist.tensionHistory.size(); j++)
        {
            const double previousTension = stringHist.tensionHistory[j-1];
            const double previousLength = stringHist.restLengths[j-1];
            const double currentLength = stringHist.restLengths[j];
            //TODO: examine this assumption - free spinning motor may require more power
            double motorSpeed = (currentLength-previousLength);
            if(motorSpeed > 0) // Vestigial code
                motorSpeed = 0;
            const double workDone = previousTension * motorSpeed;
            totalEnergySpent += workDone;
        }
    }
    
    scores.push_back(totalEnergySpent);
    
    edgeAdapter.endEpisode(scores);
    nodeAdapter.endEpisode(scores);
    
    delete m_pCPGSys;
    m_pCPGSys = NULL;
    
    for(size_t i = 0; i < m_allControllers.size(); i++)
    {
		delete m_allControllers[i];
	}
	m_allControllers.clear();*/
}
/*
const double BaseSpineCPGControl::getCPGValue(std::size_t i) const
{
	// Error handling on input done in CPG_Equations
	return (*m_pCPGSys)[i];
}*/

double LearningController12Bar::getScore() const
{
	if (scores.size() == 2)
	{
		return scores[0];
	}
	else
	{
		throw std::runtime_error("Called before scores were obtained!");
	}
}

	

array_4D LearningController12Bar::scaleEdgeActions(vector< vector <double> > actions)
{
    std::size_t numControllers = edgeConfigData.getintvalue("numberOfControllers");
    
    // Ensure reading from the same file
    assert(numControllers == actions.size());
    assert(actions[0].size() == 2);
    
    //double lowerLimit = m_config.lowPhase;
    //double upperLimit = m_config.highPhase;
    //double range = upperLimit - lowerLimit;
    
    /*array_4D actionList(boost::extents[m_config.segmentSpan]
					[m_config.theirMuscles]
					[m_config.ourMuscles]
					[m_config.params]);
    */
    /* Horrid while loop to populate upper diagonal of matrix, since
    * its symmetric and we want to minimze parameters used in learing
    * note that i==1, j==k will refer to the same muscle
    * @todo use boost to set up array so storage is only allocated for 
    * elements that are used
    */
    int i = 0;
    int j = 0;
    int k = 0;

   /* while (i < m_config.segmentSpan)
    {
        while(j < m_config.theirMuscles)
        {
            while(k < m_config.ourMuscles)
            {
                if (actions.empty())
                {
                    std::cout << "ran out before table populated!"
                    << std::endl;
                    break;
                }
                else
                {
                    if (i == 1 && j == k)
                    {
                        // std::cout << "Skipped identical muscle" << std::endl;
                        //Skip since its the same muscle
                    }
                    else
                    {
                        std::vector<double> edgeParam = actions.back();
                        // Weight from 0 to 1
                        //actionList[i][j][k][0] = edgeParam[0];
                        // Phase offset from -pi to pi
                        //actionList[i][j][k][1] = edgeParam[1] * 
                         //                       (range) + lowerLimit;
                        actions.pop_back();
                    }
                }
                k++;
            }
            j++;
            k = j;
            
        }
        j = 0;
        k = 0;
        i++;
    }
    */
    assert(actions.empty());
    
    //return actionList;
}

array_2D LearningController12Bar::scaleNodeActions  
                            (vector< vector <double> > actions)
{
    std::size_t numControllers = nodeConfigData.getintvalue("numberOfControllers");
    std::size_t numActions = nodeConfigData.getintvalue("numberOfActions");
    
    assert( actions.size() == numControllers);
    assert( actions[0].size() == numActions);
    
    array_2D nodeActions(boost::extents[numControllers][numActions]);
    
    array_2D limits(boost::extents[2][numActions]);
    
    // Check if we need to update limits
    assert(numActions == 2);
    
    //limits[0][0] = m_config.lowFreq;
    //limits[1][0] = m_config.highFreq;
    //limits[0][1] = m_config.lowAmp;
    //limits[1][1] = m_config.highAmp;
    
    // This one is square
    for( std::size_t i = 0; i < numControllers; i++)
    {
        for( std::size_t j = 0; j < numActions; j++)
        {
            nodeActions[i][j] = ( actions[i][j] *  
                    (limits[1][j] - limits[0][j])) + limits[0][j];
        }
    }
    
    return nodeActions;
}

