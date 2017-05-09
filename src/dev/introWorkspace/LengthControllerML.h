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

#ifndef HORIZONTAL_SPINE_CONTROLLER_H
#define HORIZONTAL_SPINE_CONTROLLER_H

/**
 * @file HorizontalSpineController.h
 * @brief Contains the definition of class LengthControllerML
 * @author Marc Leroy
 * $Id$
 */

// The NTRT core library
#include "core/tgObserver.h"
#include "core/tgSubject.h"
#include "core/tgTags.h"

// The C++ standard library
#include <string>
#include <vector>
#include <map>

#include <math.h>
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>
using namespace std;
using namespace boost::numeric::odeint;
typedef boost::array<double,2> state_type2;

// Forward declarations
class TensegrityModel;
class tgBasicActuator;

/**
 * A controller to apply the length change in the cables of the 3-bar example
 * model, for the NTRT Introduction Seminar on 2016-09-28 in BEST.
 */
class LengthControllerML : public tgObserver<TensegrityModel>, public tgSubject<LengthControllerML>
{
public:
	
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
   * tags of all the
   * cables upon which to act. All the cables which have a tag in this list of tags
   * will be acted upon by this controller.
   */
  LengthControllerML(double startTime, double minLength, double rate,
			    std::vector<std::string> tagsToControl);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~LengthControllerML() { }

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

  //typedef boost::array<double,3> state_type;
  virtual void checkLengths(TensegrityModel& subject, double dt, int firstCable, int lastCable);
  virtual void hopfController(TensegrityModel& subject, double *hopf_x, double hopf_y, double hopf_omega, double hopf_mu);

  virtual void sineTest(TensegrityModel& subject, double dt, int firstCable, int lastCable, double initRestLengths, double offset, double amp);
  virtual void test_fct();
  virtual void hopf(const state_type2 &x, state_type2 &dxdt, double t);
  virtual void write_hopf(const state_type2 &x, const double t);
  //virtual void lorenz(const state_type &x, state_type &dxdt, double t);
  //virtual void write_lorenz(const state_type &x, double t);

protected:

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

  double hopf_x;
  double hopf_y;
  double hopf_omega;
  double hopf_mu;
  int ctr;
  double initRestLengths;
  //state_type x;

};

#endif // HORIZONTAL_SPINE_CONTROLLER_H
