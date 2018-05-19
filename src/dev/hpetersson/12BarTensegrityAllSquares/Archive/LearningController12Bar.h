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

#ifndef LEARNING_CONTROLLER_12_BAR_H
#define LEARNING_CONTROLLER_12_BAR_H

/**
 * @file LearningController12Bar.h
 * @brief A controller for the 12 Bar octahedron tensegrity model 
 * @author Hannah Petersson, based on code from Brian Tietz, Drew Sabelhaus and Mallory Daly
 * @version 1.0.0
 * $Id$
 */

// The C++ standard library
#include <vector>	
#include <string>
#include <map>				
#include "boost/multi_array.hpp"

// The NTRT core library
#include "core/tgSubject.h"
#include "core/tgObserver.h"
#include "core/tgTags.h"
#include "sensors/tgDataObserver.h"

// Learning library
#include "learning/Adapters/AnnealAdapter.h"

//This should probably be a forward declaration
//#include "BaseSpineModelLearning.h" 			

// Forward Declarations
class TensegrityModel;
class tgBasicActuator;
class CPGEquations;

// Declaration of arrays used in learning
typedef boost::multi_array<double, 2> array_2D;
typedef boost::multi_array<double, 4> array_4D;

/**
 * BaseSpineCPGControl learns the parameters for a CPG system on a
 * spine like tensegrity structure specified as a BaseSpineModelLearning. Parameters are generated by
 * AnnealEvolution and used in the CPGEquations family of classes.
 * tgImpedanceController controllers are used for the detailed muscle control.
 * Due to the number of parameters, the learned parameters are split
 * into one config file for the nodes and another for the CPG's "edges"
 */
class LearningController12Bar : public tgObserver<TensegrityModel>, public tgSubject <LearningController12Bar> /* Change names */
{
public:

 struct Config
    {
    public:
        /**
         * The only constructor. 
         */
        Config(int ss,
        int tm,
        int om,
        int param,
        int segnum = 6,
        double ct = 0.1,
        double la = 0,
        double ha = 30,
        double lp = -1 * M_PI,
        double hp = M_PI,
        double kt = 0.0,
        double kp = 1000.0,
        double kv = 100.0,
        bool def = true,
        double cl = 10.0,
        double lf = 0.0,
        double hf = 30.0);

                // Learning Parameters
                const int segmentSpan; // 3 possible muscles touching two rigid bodies
                const int theirMuscles; // 8 muscles in a segment 
                const int ourMuscles; // same as above
                const int params; // Number of parameters per edge
                const int segmentNumber;

        // CPG control frequency
        const double controlTime;

        // Limit Params
        const double lowAmp;
        const double highAmp;
        const double lowFreq;
        const double highFreq;
        const double lowPhase;
        const double highPhase;

                // Parameters for Impedance Controllers
                const double tension;
                const double kPosition;
                const double kVelocity;
                const bool useDefault;
        const double controlLength;    
	    };

	// Learning Parameters
	//const int segmentSpan; // 3 possible muscles touching two rigid bodies
	//const int theirMuscles; // 8 muscles in a segment 
	//const int ourMuscles; // same as above
	//const int params; // Number of parameters per edge
	//const int segmentNumber;
        
	// Parameters for Impedance Controllers /* Unclear if these are needed */
	//const double tension;
	//const double kPosition;
	//const double kVelocity;
	//const bool useDefault;
        //const double controlLength; 

  /**
   * Construct a 12 bar tensegrity structure (octahedron) controller.
   * @param[in] startTime, a double that determines when the controller
   * begins its motion, how many seconds after the simulation starts.
   * @param[in] minLength, a double that is the percent of the initial length
   * that this controller will reduce down to. E.g., if minLength = 0.25, 
   * controller will act until the rest length of the cables is 25% of initial.
   * @param[in] rate, the rate at which the rest length of the cables will be
   * changed. Expressed in meters/sec.
   * @param[in] tagsToControl, a vector (array) of strings, which is a list of the 
   * tags of all the
   * cables upon which to act. All the cables which have a tag in this list of tags
   * will be acted upon by this controller.
   */
   LearningController12Bar(LearningController12Bar::Config config, double startTime, double minLength, double rate,
					 std::vector<std::string> tagsToControl,
					std:: string args, std::string resourcePath = "", std::string ec = "edgeConfig.ini",
					std::string nc = "nodeConfig.ini");

   /**
   * Nothing to delete, destructor must be virtual
   */
    virtual ~LearningController12Bar();

  /**
   * Apply the controller. On setup, adjust the cable
   * lengths one time.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated // What is allMuscles?/h
   */
   virtual void onSetup(TensegrityModel& subject);

  /**
   * Changes the cables' lengths at some specified timestep.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   * @param[in] dt, current timestep must be positive
   */
   virtual void onStep(TensegrityModel& subject, double dt);
    
   virtual void onTeardown(TensegrityModel& subject);		/* Look more into this one */

   double getScore() const;				/* Unclear what this is */

protected:
    /**
     * Takes a vector of parameters reported by learning, and then 
     * converts it into a format used to assign to the CPGEdges
     * Note that if the CPG edges change, this will need to change
     */
	/* This needs to be changed to complie with the 12 bar model */
   virtual array_4D scaleEdgeActions (std::vector< std::vector <double> > actions);
   virtual array_2D scaleNodeActions (std::vector< std::vector <double> > actions);
    
  /**
   * A helper function to find and initialize the actuators that this class
   * will control.
   * @param[in] tag, a string of the tag for which to search in the list of 
   * actuators in this model.
   */
  void initializeActuators(TensegrityModel& subject, std::string tag);

    virtual void setupController(TensegrityModel& subject, array_2D nodeActions, array_4D edgeActions); /* This should be */
											/* integrated with initAct.. */
    //CPGEquations* m_pCPGSys; /* Leave for now, this variable helps update the parameters/commands */

    LearningController12Bar::Config m_config;
    
    /**
     * Hold the filename that gets passed to the respective configuration objects
     * Having two of each of these allows for different groups of parameters to be specified
     */
    std::string edgeConfigFilename;
    std::string nodeConfigFilename;
    
    /**
     * A file input/output system that uses .ini files. The object converts a string to
     * data in the filename that it has been passed.
     */
    configuration nodeConfigData;
    configuration edgeConfigData;
    
    /**
     * The object that holds all of the data for the evolution run in vectors of populations.
     * Needs to persist throughout a simulation
     */
    AnnealEvolution edgeEvolution;
    AnnealEvolution nodeEvolution;
    
    /**
     * Handles getting parameters from and returning scores to the evolution object.
     * Needs to persist throughout a simulation
     */
    AnnealAdapter edgeAdapter;
    AnnealAdapter nodeAdapter;
    
    /**
     * A booelan that indicates whether the respective group is learning this run. Is passed to the evolution adapter
     */
    bool nodeLearning;
    bool edgeLearning;
    
    std::vector<double> initConditions;
    
    std::size_t segments;
    
    tgDataObserver m_dataObserver;
    
    double m_updateTime;
    
    std::vector<double> scores;
    
    bool bogus;


private:

  /**
   * The private variables for each of the values passed in to the constructor.
   */
  double m_startTime;
  double m_minLength;
  double m_rate;
  std::vector<std::string> m_tagsToControl;

  /**
   * Need an accumulator variable to determine when to start the controller.
   */
  double m_timePassed;

  /**
   * Need integer for current cable index
   */
  int m_cable_index;

  /**
   * A list of all the actuators to control. This is populated in onSetup
   * by using m_tagsToControl.
   */
  std::vector<tgBasicActuator*> cablesWithTags;

  /**
   * The start length of each of the cables must be recorded.
   * This map takes a string (the space-separated list of all the tags for
   * an individual cable) and outputs a double (the rest length at time t=0.)
   */
  typedef std::map<tgTags, double> InitialRestLengths;
  InitialRestLengths initialRL;

  /** OBS OBS OBS OBS OBS
   * Need a boolean for returning or retracting the cable, not clear which one to use at this point
   */
  bool m_return;
  bool m_retract;
  bool m_finished; 

};

#endif // LEARNING_CONTROLLER_12_BAR_H
