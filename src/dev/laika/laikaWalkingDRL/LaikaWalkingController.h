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

#ifndef LAIKA_WALKING_CONTROLLER_H
#define LAIKA_WALKING_CONTROLLER_H

/**
 * @file LaikaWalkingController.h
 * @brief Contains the definition of class LaikaWalkingController.
 * @author Edward Zhu, Drew Sabelhaus, Lara Janse van Vuuren
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

#include "NeuralNetDynamics.h"
#include "NeuralNetPolicy.h"
#include "RandomShootingMPC.h"
#include "ElevationSensor.h"

#include <numeric/ublas/vector.hpp>

using namespace boost::numeric::ublas;

// Forward declarations
class TensegrityModel;
// class LaikaWalkingModel;
class tgBasicActuator;
class tgBasicController;
class btVector3;
class btRigidBody;

/**
 * A controller to apply the length change in the cables and leg torques of the LaikaWalking
 * model. This is used for the CS294-112 DRL
 */
class LaikaWalkingController : public tgObserver<TensegrityModel>, public tgSubject<LaikaWalkingController>
{
public:

  /**
   * Construct a LaikaWalkingController.
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
  LaikaWalkingController(bool train, double target_velocity);

  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~LaikaWalkingController() { }

  /**
   * Apply the controller. On setup, adjust the cable
   * lengths one time.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   */
  virtual void onSetup(TensegrityModel& subject);

  /**
   * The onStep method is not used for this controller.
   * @param[in] subject - the TensegrityModel that is being controlled. Must
   * have a list of allMuscles populated
   * @param[in] dt, current timestep must be positive
   */
  virtual void onStep(TensegrityModel& subject, double dt);

  void updateRestLengths(std::vector<double> controlRL, double targetVel, double dt);

  void updateRestLengthsDiscrete(std::vector<double> controlRL, double targetVel, double dt);

  void updateRestLengthsContinuous(std::vector<double> controlRL, double targetVel, double dt);

  void updateTorques(std::vector<double> controlTorques);

// protected:

private:

  std::vector<tgBasicActuator*> getAllActuators(TensegrityModel& subject, std::vector<std::string> actuatorTags);

  std::vector<btRigidBody*> getRigidBodies(TensegrityModel& subject, std::vector<std::string> tags);

  vector<double> getCurrActions();

  vector<double> getCurrRestLengths();

  void setRestLengths(double dt);

  void setTorques(double dt);

  vector<double> getLaikaWalkingModelStates(TensegrityModel& subject);
  /**
   * A list of all the actuators to control.
   */
  std::vector<tgBasicActuator*> m_allActuators;
  std::vector<tgBasicController*> m_allControllers;

  std::vector<btRigidBody*> shoulderBodies;
  std::vector<btRigidBody*> hipBodies;
  btRigidBody* shoulderBody;
  btRigidBody* hipBody;
  std::vector<btRigidBody*> legBodies;

  /**
   * Number of vertebrae in the model, set in setup
   */
  int num_vertebrae;
  int num_legs;

  /**
   * Action and state space dimensions, set in setup
   */
  int state_dim;
  int cable_action_dim;
  int leg_action_dim;

  /**
   * Cable control rest lengths
   */
  std::vector<double> desCableRL;

  /**
   * Leg torques
   */
  std::vector<btVector3> legTorques;

  double worldTime;

  // NeuralNetDynamics dyn_nn;
  // NeuralNetPolicy policy_nn;

  // Vector of current discrete cable actions -1, 0, or 1
  std::vector<double> currCableAction;
  // Vector of current leg torques
  std::vector<double> currLegTorques;

  // RandomShootingMPC controller;

  bool m_train;
  double m_target_velocity;

  // ElevationSensor elev_sens;
};

#endif // LAIKA_WALKING_CONTROLLER_H
