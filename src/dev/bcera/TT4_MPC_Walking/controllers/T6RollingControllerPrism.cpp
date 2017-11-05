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
 * @file T6RollingControllerPrism.cpp
 * @brief Implementation of the rolling controller.
 * @author Brian Cera adapted from code by Edward Zhu
 * @version 1.0.0
 * $Id$
 */

// This module
#include "T6RollingControllerPrism.h"
#include "core/abstractMarker.h" 
// The C++ Standard Library
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <algorithm>
#include <math.h>
// Boost Matrix Library
#include "numeric/ublas/matrix.hpp"
#include <numeric/ublas/assignment.hpp>
#include <numeric/ublas/operation.hpp>
//Boost Vector Library
#include <numeric/ublas/vector.hpp>
// Utility Library
#include "../utility.hpp"

using namespace boost::numeric::ublas;

namespace{
  
  double sf = 30;
  double worldTime = 0;
  
  //Matrices holding input weights and hidden layer weights of Neural Net for CSD
  matrix<double> Face_IW(6,6);
  matrix<double> Face_LW1(20,6);
  vector<double> Face_b1(6);
  vector<double> Face_b2(20);
  vector<double> Face_input_xmin(6);
  vector<double> Face_input_xmax(6);

  //Matrices holding Neural Net weights for cable actuation
  matrix<double> Cable_IW(30,34);
  matrix<double> Cable_LW1(30,30);
  matrix<double> Cable_LW2(30,30);
  matrix<double> Cable_LW3(30,30);
  matrix<double> Cable_LW4(24,30);
  vector<double> Cable_b1(30);
  vector<double> Cable_b2(30);
  vector<double> Cable_b3(30);
  vector<double> Cable_b4(30);
  vector<double> Cable_b5(24);
  vector<double> Cable_input_xmin(34);
  vector<double> Cable_input_xmax(34);
  vector<double> Cable_output_xmin(24);
  vector<double> Cable_output_xmax(24);

  //Matrices holding Neural Net weights for cable actuation (no face explicitly defined in input)
  int layer_size = 64;
  matrix<double> Cable_NF_IW(layer_size,11);
  matrix<double> Cable_NF_LW1(layer_size,layer_size);
  matrix<double> Cable_NF_LW2(layer_size,layer_size);
  matrix<double> Cable_NF_LW3(layer_size,layer_size);
  matrix<double> Cable_NF_LW4(24,layer_size);
  vector<double> Cable_NF_b1(layer_size);
  vector<double> Cable_NF_b2(layer_size);
  vector<double> Cable_NF_b3(layer_size);
  vector<double> Cable_NF_b4(layer_size);
  vector<double> Cable_NF_b5(24);
  vector<double> Cable_NF_input_xmin(11);
  vector<double> Cable_NF_input_xmax(11);
  vector<double> Cable_NF_output_xmin(24);
  vector<double> Cable_NF_output_xmax(24);
  
  //Matrix of Nodes for each Face 
  matrix<int> F_mat(20,3);

  //Matrix of Closed-Open Face pairings
  matrix<int> OCF_MAT(20,20);

  //File for Logging Data
  std::ofstream sim_out;
  bool doLog = true;
  
}

T6RollingController::Config::Config (double gravity, const std::string& mode, int face_goal, int activate_flag, int transfer_flag) : 
  m_gravity(gravity), m_mode(mode), m_face_goal(face_goal), m_activate_flag(activate_flag), m_transfer_flag(transfer_flag)
{
  assert(m_gravity >= 0);
  assert((m_face_goal >= 0) && (m_face_goal <= 19));

  if (m_mode.compare("face") != 0) {
    std::cout << "Config: invalid arguments" << std::endl;
    std::cout << "Usage: first arg is a string for mode ('face', 'path', or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
    std::cout << "Exiting..." << std::endl;
    exit(EXIT_FAILURE);
  }
}

T6RollingController::Config::Config (double gravity, const std::string& mode, btVector3 dr_goal, int activate_flag, int transfer_flag) :
  m_gravity(gravity), m_mode(mode), m_dr_goal(dr_goal), m_activate_flag(activate_flag), m_transfer_flag(transfer_flag)
{
  assert(m_gravity >= 0);
  if (mode.compare("dr") != 0) {
    std::cout << "Config: invalid arguments" << std::endl;
    std::cout << "Usage: first arg is a string for mode ('face', 'path', or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
    std::cout << "Exiting..." << std::endl;
    exit(EXIT_FAILURE);
  }
}

T6RollingController::Config::Config (double gravity, const std::string& mode, int *path, int pathSize, int activate_flag, int transfer_flag) :
  m_gravity(gravity), m_mode(mode), m_path(path), m_path_size(pathSize), m_activate_flag(activate_flag), m_transfer_flag(transfer_flag)
{
  assert(m_gravity >= 0);
  if (mode.compare("path") != 0) {
    std::cout << "Config: invalid arguments" << std::endl;
    std::cout << "Usage: first arg is a string for mode ('face', 'path', or 'dr'). Second arg is based on mode, if 'face' was used, then an int between 0 and 7 is expected. If 'dr' was used, then a btVector3 is expected" << std::endl;
    std::cout << "Exiting..." << std::endl;
    exit(EXIT_FAILURE);
  }
}

T6RollingController::T6RollingController(const T6RollingController::Config& config) : m_config(config)
{
  c_mode = config.m_mode;
  c_face_goal = config.m_face_goal;
  c_dr_goal = config.m_dr_goal;
  c_activate_flag = config.m_activate_flag;
  c_transfer_flag = config.m_transfer_flag;

  gravVectWorld.setX(0.0);
  gravVectWorld.setY(-config.m_gravity);
  gravVectWorld.setZ(0.0);
}

T6RollingController::~T6RollingController()
{
  m_controllers.clear();
}

void T6RollingController::onSetup(PrismModel& subject)
{
  
  std::cout << "onSetup: " << c_mode << " mode chosen" << std::endl;
  
  if (c_mode.compare("face") == 0) {
    std::cout << "onSetup: Goal face: " << c_face_goal << std::endl;
    controller_mode = 1;
  }
  else if (c_mode.compare("dr") == 0) {
    std::cout << "onSetup: Dead reckoning goal: [" << c_dr_goal.x() << ", " 
	      << c_dr_goal.y() << ", " << c_dr_goal.z() << "]" << std::endl;
    controller_mode = 2;
  }
  else {
    std::cout << "onSetup: Path size is " << c_path_size << " elements" << std::endl;
    std::cout << "onSetup: Path: [";
    for (int i = 0; i < c_path_size-1; i++) {
      std::cout << *(c_path+i) << ", ";
    }
    std::cout << *(c_path+c_path_size-1) << "]" << std::endl;
    controller_mode = 3;
  }

  // Retrieve rods from model
  rods = subject.getAllRods();
  //std::cout << "onSetup: Number of rods: " << rods.size() << std::endl;

  // Convert from tgRod objects to btRigidBody objects
  for (size_t i = 0; i < rods.size(); i++) {
    tgRod* rod = rods[i];
    btRigidBody* rodBody = rod->getPRigidBody();
    rodBodies.push_back(rodBody);
  }

  /*
  //Obtain tank btRigidBody object
  tank = subject.getTank();
  */

  // Retrieve normal vectors from model
  normVects = subject.getNormVects();

  // Set up controllers for the cables
  m_controllers.clear();
  actuators = subject.getAllActuators();
  cables = subject.getAllCables();	
  //std::cout << "onSetup: Number of actuators: " << actuators.size() << std::endl;
  for (size_t i = 0; i < actuators.size(); i++) {
    tgBasicActuator* const pActuator = actuators[i];
    assert(pActuator != NULL);
    tgBasicController* m_lenController = new tgBasicController(pActuator, restLength);
    m_controllers.push_back(m_lenController);
    //std::cout << "onSetup: Cable " << i << ": " << pActuator->getCurrentLength() << std::endl;
  }

  // Find the rest length and start length of the cables
  restLength = actuators[0]->getRestLength();
  startLength = actuators[0]->getStartLength();


  // IMPORT NEURAL NET WEIGHT DATA FROM CSV FILES //////////////////////////////////////////

  std::ifstream file;
  std::string value;

  // CSD NEURAL NET WEIGHTS /////////////////////////////////////////////////////////////
  //NN Input Layer Weights 
  file.open( "Face_IW.csv" );
  for(size_t i=0; i<Face_IW.size1(); i++){
    for(size_t j=0; j<Face_IW.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Face_IW(i,j) = std::atof(value.c_str());
    }
  }
  file.close();
  

  //NN Hidden Layer 1 Weights
  file.open( "Face_LW1.csv" );
  for(size_t i=0; i<Face_LW1.size1(); i++){
    for(size_t j=0; j<Face_LW1.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Face_LW1(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  /*
  //NN Hidden Layer 2 Weights
  std::ifstream file3 ( "Face_LW2.csv" );
  std::string value3;
  for(size_t i=0; i<23; i++){
  for(size_t j=0; j<20; j++){
  getline(file3,value3,',');
  //std::cout<<value<<std::endl;
  Face_LW2(i,j) = std::atof(value3.c_str());
  }
  }
  */

  //NN Hidden Layer 1 Bias
  file.open( "Face_b1.csv" );
  for(size_t i=0; i<Face_b1.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Face_b1(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Hidden Layer 2 Bias
  file.open( "Face_b2.csv" );
  for(size_t i=0; i<Face_b2.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Face_b2(i) = std::atof(value.c_str());
  }
  file.close();
  
  /*
  //NN Output Layer Bias
  std::ifstream file6 ( "Face_b3.csv" );
  std::string value6;
  for(size_t i=0; i<23; i++){
  getline(file6,value6,',');
  //std::cout<<value<<std::endl;
  Face_b3(i) = std::atof(value6.c_str());
  }
  */

  //NN Input Layer Min
  file.open( "Face_input_xmin.csv" );
  for(size_t i=0; i<Face_input_xmin.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Face_input_xmin(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Max
  file.open( "Face_input_xmax.csv" );
  for(size_t i=0; i<Face_input_xmax.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Face_input_xmax(i) = std::atof(value.c_str());
  }
  file.close();
  
  /*
  //NN Output Layer Min
  std::ifstream file9 ( "Face_output_xmin.csv" );
  std::string value9;
  for(size_t i=0; i<23; i++){
  getline(file9,value9,',');
  //std::cout<<value<<std::endl;
  Face_output_xmin(i) = std::atof(value9.c_str());
  }

  //NN Output Layer Max
  std::ifstream file10 ( "Face_output_xmax.csv" );
  std::string value10;
  for(size_t i=0; i<23; i++){
  getline(file10,value10,',');
  //std::cout<<value<<std::endl;
  Face_output_xmax(i) = std::atof(value10.c_str());
  }
  */


  // CABLE ACTUATION NEURAL NET WEIGHTS ///////////////////////////////////////////////////

  //NN Input Layer Weights 
  file.open( "Cable_IW.csv" );
  for(size_t i=0; i<Cable_IW.size1(); i++){
    for(size_t j=0; j<Cable_IW.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_IW(i,j) = std::atof(value.c_str());
    }
  }
  file.close();
  
  file.open( "Cable_LW1.csv" );
  for(size_t i=0; i<Cable_LW1.size1(); i++){
    for(size_t j=0; j<Cable_LW1.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_LW1(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_LW2.csv" );
  for(size_t i=0; i<Cable_LW2.size1(); i++){
    for(size_t j=0; j<Cable_LW2.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_LW2(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_LW3.csv" );
  for(size_t i=0; i<Cable_LW3.size1(); i++){
    for(size_t j=0; j<Cable_LW3.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_LW3(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_LW4.csv" );
  for(size_t i=0; i<Cable_LW4.size1(); i++){
    for(size_t j=0; j<Cable_LW4.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_LW4(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  //NN Hidden Layer 1 Bias
  file.open( "Cable_b1.csv" );
  for(size_t i=0; i<Cable_b1.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_b1(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Hidden Layer 2 Bias
  file.open( "Cable_b2.csv" );
  for(size_t i=0; i<Cable_b2.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_b2(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Hidden Layer 3 Bias
  file.open( "Cable_b3.csv" );
  for(size_t i=0; i<Cable_b3.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_b3(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Hidden Layer 4 Bias
  file.open( "Cable_b4.csv" );
  for(size_t i=0; i<Cable_b4.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_b4(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Output Layer Bias
  file.open( "Cable_b5.csv" );
  for(size_t i=0; i<Cable_b5.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_b5(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Input Layer Min
  file.open( "Cable_input_xmin.csv" );
  for(size_t i=0; i<Cable_input_xmin.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_input_xmin(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Max
  file.open( "Cable_input_xmax.csv" );
  for(size_t i=0; i<Cable_input_xmax.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_input_xmax(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Min
  file.open( "Cable_output_xmin.csv" );
  for(size_t i=0; i<Cable_output_xmin.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_output_xmin(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Max
  file.open( "Cable_output_xmax.csv" );
  for(size_t i=0; i<Cable_output_xmax.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_output_xmax(i) = std::atof(value.c_str());
  }
  file.close();


  // CABLE ACTUATION NEURAL NET WEIGHTS (NO FACE) ///////////////////////////////////////////////////

  //NN Input Layer Weights 
  file.open( "Cable_NF_IW.csv" );
  for(size_t i=0; i<Cable_NF_IW.size1(); i++){
    for(size_t j=0; j<Cable_NF_IW.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_NF_IW(i,j) = std::atof(value.c_str());
    }
  }
  file.close();
  
  file.open( "Cable_NF_LW1.csv" );
  for(size_t i=0; i<Cable_NF_LW1.size1(); i++){
    for(size_t j=0; j<Cable_NF_LW1.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_NF_LW1(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_NF_LW2.csv" );
  for(size_t i=0; i<Cable_NF_LW2.size1(); i++){
    for(size_t j=0; j<Cable_NF_LW2.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_NF_LW2(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_NF_LW3.csv" );
  for(size_t i=0; i<Cable_NF_LW3.size1(); i++){
    for(size_t j=0; j<Cable_NF_LW3.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_NF_LW3(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_NF_LW4.csv" );
  for(size_t i=0; i<Cable_NF_LW4.size1(); i++){
    for(size_t j=0; j<Cable_NF_LW4.size2(); j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_NF_LW4(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  //NN Hidden Layer 1 Bias
  file.open( "Cable_NF_b1.csv" );
  for(size_t i=0; i<Cable_NF_b1.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_b1(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Hidden Layer 2 Bias
  file.open( "Cable_NF_b2.csv" );
  for(size_t i=0; i<Cable_NF_b2.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_b2(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Hidden Layer 3 Bias
  file.open( "Cable_NF_b3.csv" );
  for(size_t i=0; i<Cable_NF_b3.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_b3(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Hidden Layer 4 Bias
  file.open( "Cable_NF_b4.csv" );
  for(size_t i=0; i<Cable_NF_b4.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_b4(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Output Layer Bias
  file.open( "Cable_NF_b5.csv" );
  for(size_t i=0; i<Cable_NF_b5.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_b5(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Input Layer Min
  file.open( "Cable_NF_input_xmin.csv" );
  for(size_t i=0; i<Cable_NF_input_xmin.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_input_xmin(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Max
  file.open( "Cable_NF_input_xmax.csv" );
  for(size_t i=0; i<Cable_NF_input_xmax.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_input_xmax(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Min
  file.open( "Cable_NF_output_xmin.csv" );
  for(size_t i=0; i<Cable_NF_output_xmin.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_output_xmin(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Max
  file.open( "Cable_NF_output_xmax.csv" );
  for(size_t i=0; i<Cable_NF_output_xmax.size(); i++){
    getline(file,value,',');
    //std::cout<<value<<std::endl;
    Cable_NF_output_xmax(i) = std::atof(value.c_str());
  }
  file.close();
  ///////////////////////////////////////////////////////////////////////////////////
  
  //Matrix of Nodes per Face
  file.open( "F_mat.csv" );
  for(size_t i=0; i<F_mat.size1(); i++){
    for(size_t j=0; j<F_mat.size2(); j++){
      getline(file,value,',');
      std::cout<<value<<std::endl;
      F_mat(i,j) = (int)std::atof(value.c_str());
    }
  }
  file.close();

  //Matrix of Nodes per Face
  file.open( "open_closed_face_MAT.csv" );
  for(size_t i=0; i<OCF_MAT.size1(); i++){
    for(size_t j=0; j<OCF_MAT.size2(); j++){
      getline(file,value,',');
      std::cout<<value<<std::endl;
      OCF_MAT(i,j) = (int)std::atof(value.c_str());
    }
  }
  file.close();


  if(doLog){
    sim_out.open("Simulation_Data.csv");
    sim_out << "Face, Rod1_X, Rod1_Y, Rod1_Z, Rod2_X, Rod2_Y, Rod2_Z, Rod3_X, Rod3_Y, Rod3_Z, Rod4_X, Rod4_Y, Rod4_Z, Rod5_X, Rod5_Y, Rod5_Z, Rod6_X, Rod6_Y, Rod6_Z, ";
    sim_out << "Rod1_Vel_X, Rod1_Vel_Y, Rod1_Vel_Z, Rod2_Vel_X, Rod2_Vel_Y, Rod2_Vel_Z, Rod3_Vel_X, Rod3_Vel_Y, Rod3_Vel_Z, Rod4_Vel_X, Rod4_Vel_Y, Rod4_Vel_Z, Rod5_Vel_X, Rod5_Vel_Y, Rod5_Vel_Z, Rod6_Vel_X, Rod6_Vel_Y, Rod6_Vel_Z, ";
    sim_out << "Cable1, Cable2, Cable3, Cable4, Cable5, Cable6, Cable7, Cable8, Cable9, Cable10, Cable11, Cable12, Cable13, Cable14, Cable15, Cable16, Cable17, Cable18, Cable19, Cable20, Cable21, Cable22, Cable23, Cable24, " << std::endl;
    sim_out << std::endl;
  }

}

void T6RollingController::onStep(PrismModel& subject, double dt)
{

  
  //std::cout << std::endl;
  //std::cout << "OverallTime: " << worldTime << std::endl;
  //std::cout << "RobotState: " << subject.robotState << std::endl;
  
  worldTime += dt;
  if (dt <= 0.0) {
    throw std::invalid_argument("onStep: dt is not positive");
  }

    
  //Contact Surface Detection
  double CSD_update_period = 0.1;
  if(fmod(worldTime,CSD_update_period)<=dt){

    prevFace = currFace;
    int currFace_marker = contactSurfaceDetection_using_markers(subject,prevFace);
    std::cout << "Marker Full-state Detected Face: " << currFace_marker << std::endl;
    currFace = contactSurfaceDetection(prevFace);

    //check if open face
    vector<int> openFaces(12);
    bool isOpenFace = 0;
    openFaces <<= 2,4,5,7,10,12,13,15,17,18,19,20; //1-indexed from MATLAB
    for(int j=0;j<openFaces.size();j++){
      if(currFace==openFaces(j)-1)
	isOpenFace=1;
    }
    //if(ActiveFaceCounter >= 2 && currFace!=-1)// && !isOpenFace)
    //activeFace = currFace;

    //prepare next closed face
    if(isOpenFace){
      std::cout << "currFace: " << currFace << ", prevactiveface: " << prevactiveFace << std::endl;
      std::cout << "PROJECTED FUTURE CLOSED FACE~~~~~~~~~~~~~" << std::endl;
      activeFace = currFace;//OCF_MAT(currFace,prevactiveFace);
      prevactiveFace = activeFace;
    }
    else{
      if(activeFace!=-1)
	prevactiveFace = activeFace;
      activeFace = currFace;
    }
	
    std::cout << "Current Face: " << currFace+1 << std::endl;
    int mat_currFace = currFace + 1; //matlab 1-index

    std::cout << "Acting Current Face: " << activeFace+1 << std::endl;
    std::cout << "Currently on Open Face (boolean): " << isOpenFace << std::endl;
    std::cout << "Transitioning Flag (boolean): " << transitioning << std::endl;

    //only re-calculate Side when face is new
    if(prevactiveFace!=activeFace && activeFace!=-1){
      idleCount = 0;
      double min_x = 1e7;
      min_ordered.clear();
      min_ordered.push_back(0);
      std::cout<<F_mat(activeFace,0)<<": "<<subject.markers[F_mat(activeFace,0)].getWorldPosition().z()<<std::endl;
      for(int i=1;i<3;i++){
	double curr_x = subject.markers[F_mat(activeFace,i)].getWorldPosition().z();
	bool node_accounted = false;
	for(int j=0;j<min_ordered.size();j++){
	  if(curr_x<subject.markers[F_mat(activeFace,min_ordered[j])].getWorldPosition().z()){
	    min_ordered.insert(min_ordered.begin()+j,i);
	    break;
	  }
	  else if(j==min_ordered.size()-1){ //not lower than anything currently in list
	    min_ordered.push_back(i);
	    break;
	  }
	  
	}
       
	std::cout<<F_mat(activeFace,i)<<": "<<subject.markers[F_mat(activeFace,i)].getWorldPosition().z()<<std::endl;
	//std::cout<<subject.markers[0].getWorldPosition().x()<<std::endl;
      }
      
      vector<int> DifficultSides(20);
      DifficultSides <<= -1,0,-1,2,2,-1,0,-1,-1,0,-1,2,2,-1,0,-1,2,2,2,2;
      if(isOpenFace && DifficultSides(activeFace)==min_ordered[0])
	min_idx = min_ordered[1];
      else
	min_idx = min_ordered[0];

      
    }

    /*
    std::cout << "Index Side Order: " << std::endl;
    for(int i=0;i<min_ordered.size();i++){
      std::cout << min_ordered[i] << ", Z value: " <<
	subject.markers[F_mat(activeFace,min_ordered[i])].getWorldPosition().z() <<
	", Y value: " <<
	subject.markers[F_mat(activeFace,min_ordered[i])].getWorldPosition().y()<<std::endl;
    }
    std::cout << std::endl;
    */
    
    //roll_case = (roll_case+1)%12;
    std::cout << "Side: " << min_idx << std::endl;
    std::cout << "Idle Count: " << idleCount << std::endl;
  }
  
  

  //Actuation of Cable Restlengths
  //std::cout << "min_idx: " << min_idx << std::endl;
  double cable_update_period = 0.05;
  if(activeFace!=-1){
    if(fmod(worldTime,cable_update_period)<=dt){
      vector<double> CableRL(24);
      int side = min_idx;//roll_case;
      //CableRL = CableRestlengthCalculation(activeFace,side);
      CableRL = CableRestlengthCalculation_noFace(side); //already divided by 1000
      for(int i=0;i<24;i++){
	double min_length = (0.52-0.0187)*sf;
	double max_length = (0.68-0.0187)*sf;
	//double des_length = actuators[i]->getRestLength()+CableRL(i)*sf*cable_update_period*10*(0.15)/(0.05); //relative RL
	double des_length = (CableRL(i)-.0187)*sf;//absolute RL
	des_length = std::max(des_length,min_length);
	des_length = std::min(des_length,max_length);
	m_controllers[i]->control(dt,des_length); //scale by scaling factor here
      }

      for(int i=0;i<cables.size();i++){
	std::cout << "Cable_" << i+1 << ": " << (actuators[i]->getRestLength())/sf << ", ";
      }
      std::cout << std::endl << "Cable Current Lengths: " << std::endl;
      for(int i=0;i<cables.size();i++){
	std::cout << "Cable_" << i+1 << ": " << (actuators[i]->getCurrentLength())/sf << ", ";
      }
      std::cout << std::endl;

      std::cout << "Rod Mass: " << rodBodies[0]->getGravity() << std::endl;
      
      //actuate cables
      int timeDelay = 0;//500;
      for(int i=0;i<24;i++){
    
	vector<int> openFaces(12);
	bool isOpenFace = 0;
	openFaces <<= 2,4,5,7,10,12,13,15,17,18,19,20; //1-indexed from MATLAB
	for(int j=0;j<openFaces.size();j++){
	  if(activeFace==openFaces(j)-1)
	    isOpenFace=1;
	}

    
	if(idleCount<timeDelay && ~isOpenFace){// ||  isOpenFace){// || transitioning || currFace==-1)// )
	  m_controllers[i]->control(dt,restLength); // return to pretensioned state
	  if(i==0){
	    std::cout << "currFace: " << currFace <<  std::endl;
	    std::cout << "catch-all flag tripped" << std::endl;
	  }
	}
    

    
      }
    }
  }

  

  idleCount++;
  /*
  int resetTime = 5000;
  if(idleCount>resetTime){//4000/.01*.05){
    //Taking too long; could be running into trouble - reset cables
    bool ready = T6RollingController::setAllActuators(m_controllers, actuators, restLength, dt);
    std::cout << "RESETTING! " << std::endl;
    if(ready && idleCount>5000)
      idleCount = 0;
    //prevactiveFace = -1;
  }
  */

  for(int i=0;i<actuators.size();i++){
    actuators[i]->moveMotors(dt);
  }

  //std::cout << "rest length: " << restLength/sf << std::endl;
  //std::cout << "start length: " << startLength/sf << std::endl;
  /*
  std::cout << "Rod Speeds: " << std::endl;
  for(int i=0;i<rodBodies.size();i++){
    std::cout << "Rod " << i+1 << ": " << (rodBodies[i]->getLinearVelocity()).norm()/sf << std::endl;
  }
  */
    
}

/*
  bool T6RollingController::checkOnGround()
  {
  bool onGround = false;
	
  btVector3 rodVel = tank[0]->getLinearVelocity();
  double rodSpeed = rodVel.norm();
  if (abs(rodSpeed) < 0.001)
  onGround = true;

  return onGround;
  }
*/

vector<double> T6RollingController::CableRestlengthCalculation(int Face,int Side)
{
  int printout = 0;
  
  // Initialize vector of phi angles (angles between rod directions and +Y axis)
  vector<double> phiInput(6);
  vector<double> relthetaInput(5);
  vector<double> faceVec(20);
  vector<double> dirVec(3);

  for(size_t i=0; i<faceVec.size() ; i++){
    faceVec(i) = 0;
  }
  for(size_t i=0; i<dirVec.size() ; i++){
    dirVec(i) = 0;
  }
  faceVec(Face) = 1;
  dirVec(Side) = 1;

  double rod1theta;
  
  //rod theta and phi angles, theta W.R.T. x-axis
  for(size_t i=0; i<6; i++){
    btTransform worldTrans = rodBodies[i]->getWorldTransform();
    btMatrix3x3 robot2world = worldTrans.getBasis();
    btVector3 rodDir = robot2world*btVector3(0,1,0);
    
    //calculate relative theta
    if(i==0){
      rod1theta = atan2(-rodDir.z(),rodDir.x());
      if(rod1theta<0)
	rod1theta += 2*M_PI;
    }
    else if(i!=0){
      double theta = atan2(-rodDir.z(),rodDir.x());
      theta -= rod1theta;
      if(theta<0)
	theta += 2*M_PI;
      relthetaInput(i-1) = theta;
    }

    //calculate phi
    double angle = rodDir.angle(btVector3(0,1,0));
    phiInput(i) = angle;
  }

  //concatenate input feature vector
  vector<double> Input(34);
  Input <<= faceVec, dirVec, relthetaInput, phiInput;
  std::cout << "Input size: " << Input.size() << std::endl;

  if(printout){
    for(size_t i=0; i<Input.size() ; i++){
      std::cout << Input(i) << ", ";
    }
    std::cout << std::endl;
  }
  
  //mapminmax pre-process feature data
  scalar_vector <double> ones34(34,1);
  Input = element_div(2*(Input-Cable_input_xmin),Cable_input_xmax-Cable_input_xmin)-ones34;

  if(printout){
    std::cout << "After preprocess: " << std::endl;
    for(size_t i=0; i<Input.size() ; i++){
      std::cout << Input(i) << ", ";
    }
    std::cout << std::endl;
  }
  
  
  
  //multiply inputs by input weights - HIDDEN LAYER 1
  vector<double> hiddenLayer1(30);
  axpy_prod(Cable_IW,Input,hiddenLayer1,true); //multiply by weights
  /*
    for(size_t i=0; i<hiddenLayer.size() ; i++){
    std::cout << hiddenLayer(i) << ", ";
    }
  */
  std::cout << std::endl;
  hiddenLayer1 += Cable_b1; //add bias
  /*
    std::cout << " Before tansig1: " << std::endl;
    for(size_t i=0; i<hiddenLayer1.size() ; i++){
    std::cout << hiddenLayer1(i) << ", ";
    }
    std::cout << std::endl;
  */
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer1.size(); i++){
    hiddenLayer1(i) = 2/(1+exp(-2*hiddenLayer1(i)))-1;
  }

  if(printout){
    std::cout << "After Tansig1: " << std::endl;
    for(size_t i=0; i<hiddenLayer1.size() ; i++){
      std::cout << hiddenLayer1(i) << ", ";
    }
    std::cout << std::endl;
  }

  //multiply inputs by input weights - HIDDEN LAYER 2
  vector<double> hiddenLayer2(30);
  axpy_prod(Cable_LW1,hiddenLayer1,hiddenLayer2,true); //multiply by weights

  std::cout << std::endl;
  hiddenLayer2 += Cable_b2; //add bias
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer2.size(); i++){
    hiddenLayer2(i) = 2/(1+exp(-2*hiddenLayer2(i)))-1;
  }

  if(printout){
    std::cout << "After Tansig2: " << std::endl;
    for(size_t i=0; i<hiddenLayer2.size() ; i++){
      std::cout << hiddenLayer2(i) << ", ";
    }
    std::cout << std::endl;
  }

  //multiply inputs by input weights - HIDDEN LAYER 3
  vector<double> hiddenLayer3(30);
  axpy_prod(Cable_LW2,hiddenLayer2,hiddenLayer3,true); //multiply by weights

  std::cout << std::endl;
  hiddenLayer3 += Cable_b3; //add bias
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer3.size(); i++){
    hiddenLayer3(i) = 2/(1+exp(-2*hiddenLayer3(i)))-1;
  }
  if(printout){
    std::cout << "After Tansig3: " << std::endl;
    for(size_t i=0; i<hiddenLayer3.size() ; i++){
      std::cout << hiddenLayer3(i) << ", ";
    }
    std::cout << std::endl;
  }

  //multiply inputs by input weights - HIDDEN LAYER 4
  vector<double> hiddenLayer4(30);
  axpy_prod(Cable_LW3,hiddenLayer3,hiddenLayer4,true); //multiply by weights

  std::cout << std::endl;
  hiddenLayer4 += Cable_b4; //add bias
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer4.size(); i++){
    hiddenLayer4(i) = 2/(1+exp(-2*hiddenLayer4(i)))-1;
  }
  if(printout){
    std::cout << "After Tansig4: " << std::endl;
    for(size_t i=0; i<hiddenLayer4.size() ; i++){
      std::cout << hiddenLayer4(i) << ", ";
    }
    std::cout << std::endl;
  }


  //multiply hidden layer 2 outputs by layer1 weights
  vector<double> outputLayer(24);
  axpy_prod(Cable_LW4,hiddenLayer4,outputLayer,true); //multiply by weights
  //std::cout << "hidden layer multiplied with weights: " << std::endl;
  /*
    for(size_t i=0; i<outputLayer.size() ; i++){
    std::cout << outputLayer(i) << ", ";
    }
    std::cout << std::endl;
  */
  outputLayer += Cable_b5; //add bias

  
  //mapminmax post-process output layer
  scalar_vector <double> ones24(24,1);
  //mapminmax.reverse vvvvvvv
  outputLayer = element_prod((outputLayer + ones24)/2,Cable_output_xmax-Cable_output_xmin)+Cable_output_xmin;

  //Rescale (times 1e3 in when traind in Neural Net)
  outputLayer /= 1000;

  if(printout){
    std::cout << "Cable RL: " << std::endl;
    for(size_t i=0; i<outputLayer.size() ; i++){
      std::cout << outputLayer(i) << ", ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
  }
  
  return outputLayer;


  
}

vector<double> T6RollingController::CableRestlengthCalculation_noFace(int Side)
{
  int printout = 1;
  
  // Initialize vector of phi angles (angles between rod directions and +Y axis)
  vector<double> phiInput(6);
  vector<double> relthetaInput(5);
  vector<double> dirVec(3);

  for(size_t i=0; i<dirVec.size() ; i++){
    dirVec(i) = 0;
  }
  dirVec(Side) = 1;

  double rod1theta;
  
  //rod theta and phi angles, theta W.R.T. x-axis
  for(size_t i=0; i<6; i++){
    btTransform worldTrans = rodBodies[i]->getWorldTransform();
    btMatrix3x3 robot2world = worldTrans.getBasis();
    btVector3 rodDir = robot2world*btVector3(0,1,0);
    
    //calculate relative theta
    if(i==0){
      rod1theta = atan2(-rodDir.z(),rodDir.x());
      if(rod1theta<0)
	rod1theta += 2*M_PI;
    }
    else if(i!=0){
      double theta = atan2(-rodDir.z(),rodDir.x());
      theta -= rod1theta;
      if(theta<0)
	theta += 2*M_PI;
      relthetaInput(i-1) = theta;
    }

    //calculate phi
    double angle = rodDir.angle(btVector3(0,1,0));
    phiInput(i) = angle;
  }

  //concatenate input feature vector
  vector<double> Input(11);
  Input <<= relthetaInput, phiInput;
  std::cout << "Cable_NN Input size: " << Input.size() << std::endl;

  if(printout){
    for(size_t i=0; i<Input.size() ; i++){
      std::cout << Input(i) << ", ";
    }
    std::cout << std::endl;
  }
  
  //mapminmax pre-process feature data
  scalar_vector <double> ones14(Input.size(),1);
  Input = element_div(2*(Input-Cable_NF_input_xmin),Cable_NF_input_xmax-Cable_NF_input_xmin)-ones14;

  if(printout){
    std::cout << "After preprocess: " << std::endl;
    for(size_t i=0; i<Input.size() ; i++){
      std::cout << Input(i) << ", ";
    }
    std::cout << std::endl;
  }
  
  
  
  //multiply inputs by input weights - HIDDEN LAYER 1
  vector<double> hiddenLayer1(Cable_NF_IW.size1());
  axpy_prod(Cable_NF_IW,Input,hiddenLayer1,true); //multiply by weights
  /*
    for(size_t i=0; i<hiddenLayer.size() ; i++){
    std::cout << hiddenLayer(i) << ", ";
    }
  */
  std::cout << std::endl;
  hiddenLayer1 += Cable_NF_b1; //add bias
  /*
    std::cout << " Before tansig1: " << std::endl;
    for(size_t i=0; i<hiddenLayer1.size() ; i++){
    std::cout << hiddenLayer1(i) << ", ";
    }
    std::cout << std::endl;
  */
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer1.size(); i++){
    hiddenLayer1(i) = 2/(1+exp(-2*hiddenLayer1(i)))-1;
  }

  if(printout){
    std::cout << "After Tansig1: " << std::endl;
    for(size_t i=0; i<hiddenLayer1.size() ; i++){
      std::cout << hiddenLayer1(i) << ", ";
    }
    std::cout << std::endl;
  }

  //multiply inputs by input weights - HIDDEN LAYER 2
  vector<double> hiddenLayer2(Cable_NF_LW1.size1());
  axpy_prod(Cable_NF_LW1,hiddenLayer1,hiddenLayer2,true); //multiply by weights

  std::cout << std::endl;
  hiddenLayer2 += Cable_NF_b2; //add bias
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer2.size(); i++){
    hiddenLayer2(i) = 2/(1+exp(-2*hiddenLayer2(i)))-1;
  }

  if(printout){
    std::cout << "After Tansig2: " << std::endl;
    for(size_t i=0; i<hiddenLayer2.size() ; i++){
      std::cout << hiddenLayer2(i) << ", ";
    }
    std::cout << std::endl;
  }

  //multiply inputs by input weights - HIDDEN LAYER 3
  vector<double> hiddenLayer3(Cable_NF_LW2.size1());
  axpy_prod(Cable_NF_LW2,hiddenLayer2,hiddenLayer3,true); //multiply by weights

  std::cout << std::endl;
  hiddenLayer3 += Cable_NF_b3; //add bias
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer3.size(); i++){
    hiddenLayer3(i) = 2/(1+exp(-2*hiddenLayer3(i)))-1;
  }
  if(printout){
    std::cout << "After Tansig3: " << std::endl;
    for(size_t i=0; i<hiddenLayer3.size() ; i++){
      std::cout << hiddenLayer3(i) << ", ";
    }
    std::cout << std::endl;
  }

  //multiply inputs by input weights - HIDDEN LAYER 4
  vector<double> hiddenLayer4(Cable_NF_LW3.size1());
  axpy_prod(Cable_NF_LW3,hiddenLayer3,hiddenLayer4,true); //multiply by weights

  std::cout << std::endl;
  hiddenLayer4 += Cable_NF_b4; //add bias
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer4.size(); i++){
    hiddenLayer4(i) = 2/(1+exp(-2*hiddenLayer4(i)))-1;
  }
  if(printout){
    std::cout << "After Tansig4: " << std::endl;
    for(size_t i=0; i<hiddenLayer4.size() ; i++){
      std::cout << hiddenLayer4(i) << ", ";
    }
    std::cout << std::endl;
  }


  //multiply hidden layer 2 outputs by layer1 weights
  vector<double> outputLayer(24);
  axpy_prod(Cable_NF_LW4,hiddenLayer4,outputLayer,true); //multiply by weights
  //std::cout << "hidden layer multiplied with weights: " << std::endl;
  /*
    for(size_t i=0; i<outputLayer.size() ; i++){
    std::cout << outputLayer(i) << ", ";
    }
    std::cout << std::endl;
  */
  outputLayer += Cable_NF_b5; //add bias

  
  //mapminmax post-process output layer
  scalar_vector <double> ones24(24,1);
  //mapminmax.reverse vvvvvvv
  outputLayer = element_prod((outputLayer + ones24)/2,Cable_NF_output_xmax-Cable_NF_output_xmin)+Cable_NF_output_xmin;

  if(printout){
    std::cout << "Cable_NF RL: " << std::endl;
    for(size_t i=0; i<outputLayer.size() ; i++){
      std::cout << outputLayer(i) << ", ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
  }

  //Rescale (times 1e3 in when traind in Neural Net)
  outputLayer /= 100;
  
  return outputLayer;
  
}

typedef std::pair<double,int> mypair;
bool T6RollingController::comparator(const mypair& l,const mypair& r){return l.first < r.first;}

int T6RollingController::contactSurfaceDetection_using_markers(PrismModel& subject,int prevFace)
{
  std::vector<mypair> markers_Z(12);
  double min_Z = 1.6;
  double max_Z = 1.8;
  double Z_buffer = 0.3; //buffer height above max_Z where no other nodes (besides base 3) should be
  int inRange_Count = 0;
  int currFace = -1;
  
  for(int i=0;i<markers_Z.size();i++){
    markers_Z[i].first = subject.markers[i].getWorldPosition().y();
    markers_Z[i].second = i;
    if(markers_Z[i].first>min_Z && markers_Z[i].first<max_Z+Z_buffer)
      inRange_Count++;
  }
  std::cout << "Nodes on Ground: " << inRange_Count << std::endl;

  std::sort(markers_Z.begin(),markers_Z.end(),comparator);
  for(int i=0;i<F_mat.size1();i++){
    int arr[] = {F_mat(i,0),F_mat(i,1),F_mat(i,2)};
    std::vector<int> F_row(arr,arr+sizeof(arr)/sizeof(arr[0]));
    std::vector<int>::iterator contains1 = std::find(F_row.begin(),F_row.end(),markers_Z[0].second);
    std::vector<int>::iterator contains2 = std::find(F_row.begin(),F_row.end(),markers_Z[1].second);
    std::vector<int>::iterator contains3 = std::find(F_row.begin(),F_row.end(),markers_Z[2].second);
    if(contains1!=F_row.end() && contains2!=F_row.end() && contains3!=F_row.end()){
      std::cout << "Contact Face = " << i << std::endl;
      currFace = i;
      break;
    }
  }

  if(inRange_Count!=3)
    currFace = -1;
  if(inRange_Count>3)
    currFace = prevFace;

  for(int i=0;i<3;i++){
    std::cout << "Marker " << markers_Z[i].second << ": " << markers_Z[i].first << std::endl;
  }
  std::cout << "currFace: " << currFace << std::endl;
  return currFace;
}

  
int T6RollingController::contactSurfaceDetection(int prevFace)
{ 
  // Initialize vector of phi angles (angles between rod directions and +Y axis)
  vector<double> phiInput(6);
  vector<double> relthetaInput(5);
  btVector3 target_dir = btVector3(1,0,0);
  double rod1theta;
  
  //rod theta and phi angles, theta W.R.T. x-axis
  for(size_t i=0; i<6; i++){
    btTransform worldTrans = rodBodies[i]->getWorldTransform();
    btMatrix3x3 robot2world = worldTrans.getBasis();
    btVector3 rodDir = robot2world*btVector3(0,1,0);
    
    //calculate relative theta
    if(i==0){
      rod1theta = atan2(-rodDir.z(),rodDir.x());
      if(rod1theta<0)
	rod1theta += 2*M_PI;
    }
    else if(i!=0){
      double theta = atan2(-rodDir.z(),rodDir.x());
      theta -= rod1theta;
      if(theta<0)
	theta += 2*M_PI;
      relthetaInput(i-1) = theta;
    }

    //calculate phi
    double angle = rodDir.angle(btVector3(0,1,0));
    phiInput(i) = angle;
  }

  //target orientation
  vector <double> target_theta(1);
  target_theta(0) = atan2(-target_dir.z(),target_dir.x());
  if(target_theta(0)<0)
    target_theta(0) += 2*M_PI;

  //concatenate input feature vector
  vector<double> Input(6);
  Input <<= phiInput;//, relthetaInput;
  std::cout << "Input size: " << Input.size() << std::endl;

  for(size_t i=0; i<Input.size() ; i++){
    std::cout << Input(i) << ", ";
  }
  std::cout << std::endl;
  
  //mapminmax pre-process feature data
  scalar_vector <double> ones11(Face_input_xmin.size(),1);
  Input = element_div(2*(Input-Face_input_xmin),Face_input_xmax-Face_input_xmin)-ones11;
  std::cout << "after pre-process: " << std::endl;
  for(size_t i=0; i<Face_input_xmax.size() ; i++){
    std::cout << Face_input_xmax(i) << ", ";
  }
  /*
    std::cout << "After preprocess: " << std::endl;
    for(size_t i=0; i<Input.size() ; i++){
    std::cout << Input(i) << ", ";
    }
    std::cout << std::endl;
  */
  
  
  
  //multiply inputs by input weights - HIDDEN LAYER 1
  vector<double> hiddenLayer(Face_IW.size1());
  axpy_prod(Face_IW,Input,hiddenLayer,true); //multiply by weights
  /*
    for(size_t i=0; i<hiddenLayer.size() ; i++){
    std::cout << hiddenLayer(i) << ", ";
    }
  */
  std::cout << std::endl;
  hiddenLayer += Face_b1; //add bias
  /*
    std::cout << " Before tansig1: " << std::endl;
    for(size_t i=0; i<hiddenLayer.size() ; i++){
    std::cout << hiddenLayer(i) << ", ";
    }
    std::cout << std::endl;
  */
  
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer.size(); i++){
    hiddenLayer(i) = 2/(1+exp(-2*hiddenLayer(i)))-1;
  }
  /*
    std::cout << "After Tansig1: " << std::endl;
    for(size_t i=0; i<hiddenLayer.size() ; i++){
    std::cout << hiddenLayer(i) << ", ";
    }
    std::cout << std::endl;
  */

  /*
  //multiply inputs by input weights - HIDDEN LAYER 2
  vector<double> hiddenLayer2(20);
  axpy_prod(Face_LW1,hiddenLayer,hiddenLayer2,true); //multiply by weights
  std::cout << std::endl;
  hiddenLayer2 += Face_b2; //add bias
 
  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer2.size(); i++){
  hiddenLayer2(i) = 2/(1+exp(-2*hiddenLayer2(i)))-1;
  }
  */


  //multiply hidden layer 2 outputs by layer1 weights
  vector<double> outputLayer(Face_LW1.size1());
  axpy_prod(Face_LW1,hiddenLayer,outputLayer,true); //multiply by weights
  std::cout << "hidden layer multiplied with weights: " << std::endl;
  /*
    for(size_t i=0; i<outputLayer.size() ; i++){
    std::cout << outputLayer(i) << ", ";
    }
    std::cout << std::endl;
  */
  outputLayer += Face_b2; //add bias
  /*
    std::cout << "Before softmax: " << std::endl;
    for(size_t i=0; i<outputLayer.size() ; i++){
    std::cout << outputLayer(i) << ", ";
    }
    std::cout << std::endl;
  */

  /*
  //mapminmax post-process output layer
  scalar_vector <double> ones23(23,1);
  outputLayer = element_div(2*(outputLayer-Face_output_xmin),Face_output_xmax-Face_output_xmin)-ones23;
  */

  /*
  //softmax function for output layer (Side)
  std::cout << "Side Detection Probabilities: " << std::endl;
  double sum1 = 0;
  for(size_t i=0; i<3; i++){
  outputLayer(i) = exp(outputLayer(i));
  sum1 += outputLayer(i);
  }
  for(size_t i=0; i<3; i++){
  outputLayer(i) = outputLayer(i)/sum1;
  std::cout << outputLayer(i) << ", ";
  }
  std::cout << std::endl;
  */
  
  //softmax function for output layer (Faces)
  std::cout << "Face Detection Probabilities: " << std::endl;
  double sum2 = 0;
  for(size_t i=0; i<outputLayer.size(); i++){
    outputLayer(i) = exp(outputLayer(i));
    sum2 += outputLayer(i);
  }
  for(size_t i=0; i<outputLayer.size(); i++){
    outputLayer(i) = outputLayer(i)/sum2;
    std::cout << outputLayer(i) << ", ";
  }
  std::cout << std::endl;
  
  double threshold = 0.20;
  currSurface = prevFace;
  if(currSurface==-1)
    currSurface = 0;
  bool anyOverThreshold = 0;
  for(int i=0; i<outputLayer.size(); i++){
    if(currSurface!=-1 && (outputLayer(i)>outputLayer(currSurface)+0.10) && outputLayer(i)>=threshold){
      currSurface = i;
    }
    else if(outputLayer(i)>=threshold){
      currSurface = i;
    }
    if(outputLayer(i)>=threshold)
      anyOverThreshold = 1;
  }
  std::cout << "Contact Surface: Face " << currSurface+1 << " with Probability " << outputLayer(currSurface) << std::endl;

  if(currSurface == prevFace)
    ActiveFaceCounter++;
  else
    ActiveFaceCounter = 0;

  if(anyOverThreshold==0){
    transitioning = true;
    currSurface = -1;
  }
  else
    transitioning = false;
  
  return currSurface;
}


/*
  btVector3 T6RollingController::getRobotDir(btVector3 dirVectWorld) 
  {
  btTransform worldTrans = tank[0]->getWorldTransform();
  //btTransform worldTrans = rodBodies[2]->getWorldTransform();
  btMatrix3x3 robotToWorld = worldTrans.getBasis();
  // The basis of getWorldTransform() returns the rotation matrix from robot frame
  // to world frame. Invert this matrix to go from world to robot frame
  btMatrix3x3 worldToRobot = robotToWorld.inverse();
  // Transform the gravity vector from world frame to robot frame
  btVector3 dirVectRobot = (worldToRobot * dirVectWorld).normalize();
  //std::cout << "Gravity vector in robot frame: " << gravVectRobot << std::endl;
  return dirVectRobot;
  }
*/

bool T6RollingController::setAllActuators(std::vector<tgBasicController*>& controllers, 
					  std::vector<tgBasicActuator*>& actuators, 
					  double setLength, double dt)
{
  bool returnFin = true;
  for (size_t i = 0; i < actuators.size(); i++) {
    controllers[i]->control(dt, setLength);
    actuators[i]->moveMotors(dt);
    if (actuators[i]->getRestLength()-setLength > 0.01) {
      returnFin = false;
    }
    if (actuators[i]->getRestLength()-setLength < -0.01) {
      returnFin = false;
    }
  }
  std::cout << "Resetting Cable Lengths " << std::endl;
  return returnFin;
}

