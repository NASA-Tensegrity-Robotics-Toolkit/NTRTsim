/*
 * Copyright © 2015, United States Government, as represented by the
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

#ifndef UPPERLIMB_CONTROLLER_H
#define UPPERLIMB_CONTROLLER_H

/**
 * @file UpperLimbController.h
 * @brief Contains the definition of class UpperLimbController
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"
#include <vector>
// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

// Forward declarations
class UpperLimbModel;

//namespace std for vectors
using namespace std;

/**
 * Preferred Length Controller for UpperLimbModel. This controllers sets a preferred rest length for the muscles.
 * Constant speed motors are used in muscles to move the rest length to the preffered length over time.
 * The assumption here is that motors are constant speed independent of the tension of the muscles.
 * motorspeed and movemotors are defined at the tgBasicActuator class.
 */
class UpperLimbController : public tgObserver<UpperLimbModel>
{
public:
	
  /**
   * Construct a UpperLimbController with the initial preferred length.
   */
  
  // Note that currently this is calibrated for decimeters.
  UpperLimbController(const double prefLength, double timestep, btVector3 trajectory);
    
  /**
   * Nothing to delete, destructor must be virtual
   */
  virtual ~UpperLimbController() { }
  virtual void onSetup(UpperLimbModel& subject);
  virtual void onStep(UpperLimbModel& subject, double dt);

private:
  double m_initialLengths;
  double m_totalTime;
  double dt;

  size_t nInputNeurons;
  size_t nHiddenNeurons;
  size_t nOutputNeurons; 
  size_t nWeightsInput;  //nSynapses between input and hidden layer
  size_t nWeightsOutput; //nSynapses between hidden and output layer
  std::vector<double> inputLayer;
  //std::vector< std::vector<double> > hiddenLayer; // Use instead for deep learning
  std::vector<double> hiddenLayer;
  std::vector<double> outputLayer;
  std::vector< std::vector<double> > weights; //1+nHiddenLayers by weights-per-layer

  btVector3 initPos; // initial position of end-effector (on radius-ulna)
  btVector3 trajectory; // movement vector for the end-effector
  btVector3 goal; // target position for the end-effector (initPos + trajectory)
    
  void initializeGoal(UpperLimbModel& subject);
  void initializeNeuralNet(UpperLimbModel& subject);
  void initializeNeuralNetWeights();
  void importWeights();
  void initializeMusclePretensions(UpperLimbModel& subject);
  void populateOutputLayer();
  void setTargetLengths(UpperLimbModel& subject, double dt);
  void moveAllMotors(UpperLimbModel& subject, double dt);
  double sigmoid(double x);
  btVector3 getEndEffectorCOM(UpperLimbModel& subject);
};

#endif // UPPERLIMB_CONTROLLER_H
