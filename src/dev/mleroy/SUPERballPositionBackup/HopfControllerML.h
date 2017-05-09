/*
 * Copyright © 2017, United States Government, as represented by the
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

#ifndef HOPF_CONTROLLER_H
#define HOPF_CONTROLLER_H

/**
 * @file HorizontalSpineController.h
 * @brief Contains the definition of class HopfControllerML
 * @author Marc Leroy
 * $Id$
 */

// The NTRT core library
#include "core/tgObserver.h"
#include "core/tgSubject.h"
#include "core/tgTags.h"
#include "learning/Adapters/AnnealAdapter.h"
#include "boost/multi_array.hpp"

// The C++ standard library
#include <string>
#include <vector>
#include <map>


#define NOSCILLATORS 4
#define NSTATES 8


// Forward declarations
class TensegrityModel;
class tgBasicActuator;
//class tgKinematicActuator;
class configuration;


typedef boost::multi_array<double, 2> array_2D;


/**
 * A controller to apply the length change in the cables of the 3-bar example
 * model, for the NTRT Introduction Seminar on 2016-09-28 in BEST.
 */
class HopfControllerML : public tgObserver<TensegrityModel>, public tgSubject<HopfControllerML>
{
public:
	
    struct Config
    {
      public:
          /**
           * The only constructor. 
           */
          Config(double hOMin, double hOMax, double hMMin, double hMMax, double cMin, double cMax, 
                 double hOffEMin, double hOffEMax, double hOffOMin, double hOffOMax);
        
      // Learning Parameters
      const double hopfOmegaMin; 
      const double hopfOmegaMax;
      const double hopfMuMin;
      const double hopfMuMax;
      const double couplingMin;
      const double couplingMax;
      const double hopfOffsetEvenMin;
      const double hopfOffsetEvenMax;
      const double hopfOffsetOddMin;
      const double hopfOffsetOddMax;
    };

  /**
   * Construct a HorizontalSpineController.
   * @param[in] startTime, a double that determines when the controller
   * begins its motion, how many seconds after the simulation starts.
   * @param[in] minLength, a double that is the percent of the initial length
   * that this controller will reduce down to. E.g., if minLength = 0.25, 
   * controller will act until the rest length of the cables is 25% of initial.
   * @param[in] rate, the rate at which the rest length of the cables will be
   * changed. Expressed in meters/sec.
   * @param[in] tagsToControl, a vector (array) of strings, which is a list of the 
   * tags of all the cables upon which to act. All the cables which have a tag in this list of tags
   * will be acted upon by this controller.
   */
  HopfControllerML(HopfControllerML::Config config, std::vector<std::string> tagsToControl, double timePassed, 
          int ctr, double initRestLengths, bool saveToCSV,
          double hopfStateInit[NSTATES], double hopfVelInit[NSTATES], double hopfAccInit[NSTATES], 
          std::string args, std::string resourcePath, std::string configFile);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~HopfControllerML();

  virtual void onTeardown(TensegrityModel& subject);
  virtual double displacement(TensegrityModel& subject);
  virtual double totalEnergySpent(TensegrityModel& subject);

  /**
   * Apply the controller. On setup, adjust the cable
   * lengths one time.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   */
  virtual void onSetup(TensegrityModel& subject);
    
  /**
   * Changes the cables' lengths at some specified timestep.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(TensegrityModel& subject, double dt);

  virtual void sineTest(TensegrityModel& subject, double dt, int firstCable, int lastCable, double initRestLengths, double offset, double amp);
  virtual void checkLengths(TensegrityModel& subject, double dt, int firstCable, int lastCable);
  
  virtual void setupOscillators(TensegrityModel& subject, array_2D params);
  virtual void updateHopfState(double dt);
  virtual void hopfOscillator(TensegrityModel& subject, double dt, double m_timePassed, double *hopfState, double *hopfVel, double *hopfAcc,
                              int firstCable, int lastCable, double initRestLengths, int selectedOscillator, int hopfSelector);
  virtual void compNextHopfState(double dt, int nOscillators);
  virtual void perturbateHopf(int selectedOscillator);
  virtual std::vector<double> getBallCOM(TensegrityModel& subject);
  virtual void resetTimePassed();
  virtual array_2D scaleActions(std::vector< std::vector <double> > actions);

  virtual std::string setupCSVFiles(std::string fileDataType);
  virtual void exportHopfCSV(const double t, double *hopfState, std::vector<std::string> fileNames, std::vector<double> resultCOM);

  
  virtual void saveHistLastLengths();
  virtual void saveHistRestLengths();
  virtual void saveHistDamping();
  virtual void saveHistLastVelocities();
  virtual void saveHistTension();
  
  
protected:
  HopfControllerML::Config m_config;

  /**
   * A helper function to find and initialize the actuators that this class
   * will control.
   * @param[in] tag, a string of the tag for which to search in the list of 
   * actuators in this model.
   */
  void initializeActuators(TensegrityModel& subject, std::string tag);
  

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
   * The start length of each of the cables must be recorded.
   * This map takes a string (the space-separated list of all the tags for
   * an individual cable) and outputs a double (the rest length at time t=0.)
   */
  typedef std::map<tgTags, double> InitialRestLengths;
  InitialRestLengths initialRL;

  /**
   * A list of all the actuators to control. This is populated in onSetup
   * by using m_tagsToControl.
   */
  std::vector<tgBasicActuator*> cablesWithTags;
  //std::vector<tgKinematicActuator*> cablesWithTags;

  int nOscillators;

  double hopfOmega[NOSCILLATORS];
  double hopfMu[NOSCILLATORS];
  double coupling[NOSCILLATORS];
  double hopfOffsetEven[NOSCILLATORS];
  double hopfOffsetOdd[NOSCILLATORS];

  double hopfState[NSTATES];
  double hopfVel[NSTATES];
  double hopfAcc[NSTATES];
  
  double hopfStateFirst[NSTATES];
  double hopfVelFirst[NSTATES];
  double hopfAccFirst[NSTATES];

  int ctr;
  double initRestLengths;
  bool saveToCSV;
  std::vector<double> initialRLArray;

  std::vector<double> initPosition; // Initial position of model

    /**
   * Hold the filename that gets passed to the respective configuration objects
   * Having two of each of these allows for different groups of parameters to be specified
   */
  std::string configFilename;
  
  /**
   * A file input/output system that uses .ini files. The object converts a string to
   * data in the filename that it has been passed.
   */
  configuration configData;
  
  /**
   * The object that holds all of the data for the evolution run in vectors of populations.
   * Needs to persist throughout a simulation
   */
  AnnealEvolution evolution;
  
  /**
   * Handles getting parameters from and returning scores to the evolution object.
   * Needs to persist throughout a simulation
   */
  AnnealAdapter adapter;
  
  /**
   * A booelan that indicates whether the respective group is learning this run. Is passed to the evolution adapter
   */
  bool learning;
};

#endif // HORIZONTAL_SPINE_CONTROLLER_H