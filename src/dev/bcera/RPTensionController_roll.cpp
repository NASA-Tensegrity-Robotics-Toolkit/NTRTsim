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
 * @file RPTensionController.cpp
 * @brief Implementation of six strut tensegrity.
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

#include <iostream>
// This module
#include "RPTensionController.h"
#include "RPLengthController.h"
#include "core/abstractMarker.h"
// This application
#include "RPModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgCast.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <time.h>

using namespace std;
//List of all node connections, each row represents a node's adjacent nodes
int node_conn[12][4] = {
  {4,5,8,10},  //0
  {8,10,6,7},  //1
  {4,5,9,11},  //2
  {6,7,9,11},  //3
  {8,9,0,2},   //4
  {10,11,0,2}, //5
  {8,9,1,3},   //6
  {10,11,1,3}, //7
  {0,1,4,6},   //8
  {2,3,4,6},   //9
  {0,1,5,7},   //10
  {2,3,5,7}    //11
};
//List of node cables , each row represents a node's attached cables
int node_cables[12][4] = {
  {0,1,2,3},  //0
  {4,5,6,7},  //1
  {8,9,10,11},  //2
  {12,13,14,15},  //3
  {16,17,0,8},   //4
  {18,19,2,9}, //5
  {20,21,6,12},   //6
  {22,23,7,13}, //7
  {2,4,16,20},   //8
  {10,14,17,21},   //9
  {3,5,18,22},   //10
  {11,15,19,23}    //11
};
double pretension = 800;
double target_X = -10;
double target_Z = 0;
std::vector<tgBasicActuator*> actuators;
int crit_node;
double counter;

RPTensionController::RPTensionController(const double tension) :
  m_tension(tension)
{
  if (tension < 0.0)
    {
      throw std::invalid_argument("Negative tension");
    }
}

RPTensionController::~RPTensionController()
{
  std::size_t n = m_controllers.size();
  for(std::size_t i = 0; i < n; i++)
    {
      delete m_controllers[i];
    }
  m_controllers.clear();
}	

void RPTensionController::onSetup(RPModel& subject)
{
  globalTime = 0;
  sec_count = 0;
  toggle = 0;
  counter = 0;
  m_controllers.clear(); 
  
  actuators = subject.getAllActuators();
  for (size_t i = 0; i < actuators.size(); ++i)
    {
      tgBasicActuator * const pActuator = actuators[i];
      assert(pActuator != NULL);
      tgTensionController* m_lenController = new tgTensionController(pActuator, m_tension);
      m_controllers.push_back(m_lenController);
      tgBasicController* m_lenController_length = new tgBasicController(pActuator, m_tension);
      m_controllers_length.push_back(m_lenController_length);
    }
}

int RPTensionController::findClosestNode(RPModel& subject){

  abstractMarker m1=subject.markers[subject.mGrnd[0]];
  abstractMarker m2=subject.markers[subject.mGrnd[1]];
  abstractMarker m3=subject.markers[subject.mGrnd[2]];
  
  double m1_diff = pow((m1.getWorldPosition().getX()-target_X),2)+pow((m1.getWorldPosition().getZ()-target_Z),2);
  double m2_diff = pow((m2.getWorldPosition().getX()-target_X),2)+pow((m2.getWorldPosition().getZ()-target_Z),2);
  double m3_diff = pow((m3.getWorldPosition().getX()-target_X),2)+pow((m3.getWorldPosition().getZ()-target_Z),2);
  if(m1_diff < m2_diff && m1_diff < m3_diff)
    return subject.mGrnd[0]; 
  else if(m2_diff < m3_diff)
    return subject.mGrnd[1];
  else
    return subject.mGrnd[2];
}

void RPTensionController::onStep(RPModel& subject, double dt)
{
  if (dt <= 0.0)
    {
      throw std::invalid_argument("dt is not positive");
    }
  else
    {
      globalTime += dt;
      std::size_t n = m_controllers.size();
      
      for(int p=0; p<n; p++){
	//m_controllers[p]->control(dt,pretension);
	//std::cout << actuators[p]->getTension() << " ";
      }
      
      //cout << endl;
      if(counter == 20000){
	crit_node = findClosestNode(subject);
	cout << "Closest Node: " << crit_node << endl;
	for(int p=0; p<n; p++){
	  //m_controllers[p]->control(dt,pretension);
	  std::cout << actuators[p]->getTension() << " ";
	}
	cout << endl;
	cout << "Actuated Cables: ";
	for(int i=0; i<4; i++){
	  cout << node_cables[crit_node][i] << " ";
	}
	cout << endl;
	counter = 0;
      }
      else{
	counter ++;
      }

      //cout << temp << " " << crit_node << endl;

      if(globalTime > 2){
	if(subject.tipFlag == 1 && sec_count < 4){
	  std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
	  for(int p=0; p<n; p++){
	    m_controllers_length[p]->control(dt,actuators[p]->getStartLength());
	    actuators[p]->moveMotors(dt);	    
	    //m_controllers[p] -> control(dt,pretension);
	  }
	  std::cout << "Resetting - ";
	  sec_count += dt;
	}
	else{      
	  subject.tipFlag = 0;
	  sec_count = 0;
	  //cout << crit_node << endl;
	  /*
	  for(int p=0; p<4; p++){
	    //m_controllers[node_cables[subject.mGrnd[0]][p]]->control(dt,0);
	    //m_controllers[node_cables[subject.mGrnd[1]][p]]->control(dt,0);
	    //m_controllers[node_cables[subject.mGrnd[2]][p]]->control(dt,0);
	    //cout << node_cables[crit_node][p] << endl;
	    m_controllers[node_cables[crit_node][p]]->control(dt,10);
	    //std::cout << node_cables[crit_node][p] << " ";
	    //std::cout << actuators[node_cables[crit_node][p]]->getTension() << " ";
	  }  
	  */
	  for(int p=0; p<n; p++){
	    m_controllers[p]->control(dt,0);
	  }
	}
      }
    }
  /*
  if(globalTime > 2){
    if(toggle==0){
      cout << endl << "Activating Cable Motors (Randomized Lengths) -------------------------------------" << endl;
      toggle = 1;
    }
    const std::vector<tgBasicActuator*> actuators = subject.getAllActuators();
    for(int i = 0; i<actuators.size(); i++){
      actuators[i]->moveMotors(dt);
      //cout << (double)rand_lengths[i] << " ";
    }
  }
  */

}
