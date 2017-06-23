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
//Boost Join
#include <boost/fusion/algorithm/transformation/join.hpp>
#include <boost/fusion/include/join.hpp>
#include <boost/range/join.hpp>
#include "assign/list_of.hpp"
// Utility Library
#include "../utility.hpp"

using namespace boost::numeric::ublas;
using namespace boost::range;

namespace{
  double sf = 30;
  double worldTime = 0;
  
  //Matrices holding input weights and hidden layer weights of Neural Net for CSD
  matrix<double> Face_IW(20,11);
  matrix<double> Face_LW1(20,20);
  //matrix<double> Face_LW2(23,20);
  vector<double> Face_b1(20);
  vector<double> Face_b2(20);
  //vector<double> Face_b3(23);
  vector<double> Face_input_xmin(11);
  vector<double> Face_input_xmax(11);
  //vector<double> Face_output_xmin(23);
  //vector<double> Face_output_xmax(23);

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
  
  matrix<double> RL_mat(20*3,24);

  //Matrix of Nodes for each Face 
  matrix<int> F_mat(20,3);
  
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

  std::ifstream file;// ( "Face_IW.csv" );
  std::string value;

  // CSD NEURAL NET WEIGHTS /////////////////////////////////////////////////////////////
  //NN Input Layer Weights 
  file.open( "Face_IW.csv" );
  for(size_t i=0; i<20; i++){
    for(size_t j=0; j<11; j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Face_IW(i,j) = std::atof(value.c_str());
    }
  }
  file.close();
  

  //NN Hidden Layer 1 Weights
  file.open( "Face_LW1.csv" );
  for(size_t i=0; i<20; i++){
    for(size_t j=0; j<20; j++){
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
  for(size_t i=0; i<20; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Face_b1(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Hidden Layer 2 Bias
  file.open( "Face_b2.csv" );
  for(size_t i=0; i<20; i++){
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
  for(size_t i=0; i<11; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Face_input_xmin(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Max
  file.open( "Face_input_xmax.csv" );
  for(size_t i=0; i<11; i++){
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
  for(size_t i=0; i<30; i++){
    for(size_t j=0; j<34; j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_IW(i,j) = std::atof(value.c_str());
    }
  }
  file.close();
  
  file.open( "Cable_LW1.csv" );
  for(size_t i=0; i<30; i++){
    for(size_t j=0; j<30; j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_LW1(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_LW2.csv" );
  for(size_t i=0; i<30; i++){
    for(size_t j=0; j<30; j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_LW2(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_LW3.csv" );
  for(size_t i=0; i<30; i++){
    for(size_t j=0; j<30; j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_LW3(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "Cable_LW4.csv" );
  for(size_t i=0; i<24; i++){
    for(size_t j=0; j<30; j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_LW4(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  //NN Hidden Layer 1 Bias
  file.open( "Cable_b1.csv" );
  for(size_t i=0; i<30; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_b1(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Hidden Layer 2 Bias
  file.open( "Cable_b2.csv" );
  for(size_t i=0; i<30; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_b2(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Hidden Layer 3 Bias
  file.open( "Cable_b3.csv" );
  for(size_t i=0; i<30; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_b3(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Hidden Layer 4 Bias
  file.open( "Cable_b4.csv" );
  for(size_t i=0; i<30; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_b4(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Output Layer Bias
  file.open( "Cable_b5.csv" );
  for(size_t i=0; i<24; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_b5(i) = std::atof(value.c_str());
  }
  file.close();

  //NN Input Layer Min
  file.open( "Cable_input_xmin.csv" );
  for(size_t i=0; i<34; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_input_xmin(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Max
  file.open( "Cable_input_xmax.csv" );
  for(size_t i=0; i<34; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_input_xmax(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Min
  file.open( "Cable_output_xmin.csv" );
  for(size_t i=0; i<24; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_output_xmin(i) = std::atof(value.c_str());
  }
  file.close();
  
  //NN Input Layer Max
  file.open( "Cable_output_xmax.csv" );
  for(size_t i=0; i<24; i++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      Cable_output_xmax(i) = std::atof(value.c_str());
  }
  file.close();

  
  

  
  file.open( "RL_mat.csv" );
  for(size_t i=0; i<20*3; i++){
    for(size_t j=0; j<24; j++){
      getline(file,value,',');
      //std::cout<<value<<std::endl;
      RL_mat(i,j) = std::atof(value.c_str());
    }
  }
  file.close();

  file.open( "F_mat_ordered.csv" );
  for(size_t i=0; i<20; i++){
    for(size_t j=0; j<3; j++){
      getline(file,value,',');
      std::cout<<value<<std::endl;
      F_mat(i,j) = (int)std::atof(value.c_str());
    }
  }
  file.close();
  

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


  /*
    std::cout << "RL: " << std::endl;
    for(size_t i=0; i<24; i++){
    std::cout << RL_mat((currFace+1)*3-3,i) << ", ";
    }
  */

  double min_x = 1e7;
  for(int i=0;i<3;i++){
    double curr_x = subject.markers[F_mat(currFace,i)].getWorldPosition().z();
    if(curr_x<min_x){
      min_x = curr_x;
      min_idx = i;
    }
    //std::cout<<F_mat(currFace,i)<<": "<<subject.markers[F_mat(currFace,i)].getWorldPosition().z()<<std::endl;
    //std::cout<<subject.markers[0].getWorldPosition().x()<<std::endl;
  }
    
  //Contact Surface Detection
  if(fmod(worldTime,0.1)<=dt){
    int prevFace = currFace;
    currFace = contactSurfaceDetection(currFace);

    //testing: ignore open faces
    vector<int> openFaces(12);
    bool isOpenFace = 0;
    openFaces <<= 2,4,5,7,10,12,13,15,17,18,19,20; //1-indexed from MATLAB
    for(int j=0;j<openFaces.size();j++){
      if(currFace==openFaces(j)-1)
	isOpenFace=1;
    }
    if(isOpenFace)
      currFace = prevFace;
    
    if(prevFace!=currFace){
      idleCount = 0;
      //roll_case = (roll_case+1)%3;

      //only re-calculate side when face is new
      double min_x = 1e7;
      for(int i=0;i<3;i++){
	double curr_x = subject.markers[F_mat(currFace,i)].getWorldPosition().z();
	if(curr_x<min_x){
	  min_x = curr_x;
	  min_idx = i;
	}
	std::cout<<F_mat(currFace,i)<<": "<<subject.markers[F_mat(currFace,i)].getWorldPosition().z()<<std::endl;
	//std::cout<<subject.markers[0].getWorldPosition().x()<<std::endl;
      }
    }
    std::cout << "Curr Face: " << currFace << std::endl;
    int mat_currFace = currFace + 1; //matlab 1-index

    //roll_case = (roll_case+1)%12;
    std::cout << "Roll Case: " << roll_case << std::endl;
    std::cout << "Idle Count: " << idleCount << std::endl;
  }
  
  

  //Actuation of Cable Restlengths
  //std::cout << "min_idx: " << min_idx << std::endl;
  if(fmod(worldTime,0.1)<=dt){
    vector<double> CableRL(24);
    int side = min_idx;//roll_case;
    CableRL = CableRestlengthCalculation(currFace,side);
    for(int i=0;i<24;i++){
      double min_length = 0.1*sf;
      double max_length = 1.2*sf;
      double des_length = actuators[i]->getRestLength()+CableRL(i)*sf*2;
      des_length = std::max(des_length,min_length);
      des_length = std::min(des_length,max_length);
      m_controllers[i]->control(dt,actuators[i]->getRestLength()+CableRL(i)*sf*2); //scale by scaling factor here

      //OLD RL MATRIX LOOKUP TABLE METHOD
      //m_controllers[i]->control(dt,RL_mat((currFace)*3+1,i)*sf/.66); //scale by scaling factor here
    
      //m_controllers[i]->control(dt,sequence[i]*sf); //scale by scaling factor here
      //std::cout << "Current Control: " << sequence[i]*sf << ", ";
      //std::cout << "Start Length: " << startLength << std::endl;
      //std::cout << "Actuator" <<  i << ": " << actuators[i]->getRestLength() << ", ";
    }
  }

  int timeDelay = 1000;
  //actuate cables
  for(int i=0;i<24;i++){
    
    //m_controllers[i]->control(dt,sequence[i]*sf); //scale by scaling factor here
    //std::cout << "Current Control: " << sequence[i]*sf << ", ";
    //std::cout << "Start Length: " << startLength << std::endl;
    //std::cout << "Actuator" <<  i << ": " << actuators[i]->getRestLength() << ", ";
    vector<int> openFaces(12);
    bool isOpenFace = 0;
    openFaces <<= 2,4,5,7,10,12,13,15,17,18,19,20; //1-indexed from MATLAB
    for(int j=0;j<openFaces.size();j++){
      if(currFace==openFaces(j)-1)
	isOpenFace=1;
    }
    if(idleCount<timeDelay || isOpenFace)
	m_controllers[i]->control(dt,actuators[i]->getStartLength());
    actuators[i]->moveMotors(dt);
  }
  
  idleCount+=1;

  if(idleCount>4000)
    idleCount = 0; //Taking too long; could be running into trouble - reset cables
  
    
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
  vector<double> Input(11);
  Input <<= phiInput, relthetaInput;
  std::cout << "Input size: " << Input.size() << std::endl;

  for(size_t i=0; i<Input.size() ; i++){
    std::cout << Input(i) << ", ";
  }
  std::cout << std::endl;
  
  //mapminmax pre-process feature data
  scalar_vector <double> ones11(11,1);
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
  vector<double> hiddenLayer(20);
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
  vector<double> outputLayer(20);
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
  
  double threshold = 0.98;
  currSurface = prevFace;
  for(int i=0; i<outputLayer.size(); i++){
    if((outputLayer(i)>outputLayer(currSurface)+0.2) && outputLayer(i)>=threshold){
      currSurface = i;
    }
  }
  std::cout << "Contact Surface: Face " << currSurface+1 << " with Probability " << outputLayer(currSurface) << std::endl;
  
  return currSurface;
}

/*
int T6RollingController::headingSurfaceDetection(btVector3& travelDirWorld, int currFace)
{
  // Initialize variables
  double dotProd;
  double maxDotProd = 0;
  int goalSurface = -1;

  // Get the direction vector in robot frame
  btVector3 travelDirRobot = getRobotDir(travelDirWorld);

  // Find the dot product between the heading vector and each face
  // As all normal vectors point away from the center of the robot,
  // The larger dot product indicates better alignment
  for (size_t i = 0; i < normVects.size(); i++) {
    // if (isAdjacentFace(currFace, i)) {
    if (isClosedFace(i)) {
      dotProd = travelDirRobot.dot(normVects[i]);
      //std::cout << dotProd << std::endl;
      if (dotProd > maxDotProd) {
	maxDotProd = dotProd;
	goalSurface = i;
      }
    }
  }

  // Catch all error state
  if (goalSurface == -1) {
    std::cout << "headingSurfaceDetection: No surface found" << std::endl;
  }

  std::cout << "headingSurfaceDetection: Goal surface: " << goalSurface << std::endl;

  return goalSurface;
}
*/

/*
btVector3 T6RollingController::getRobotGravity() 
{
  btTransform worldTrans = tank[0]->getWorldTransform();
  //btTransform worldTrans = rodBodies[2]->getWorldTransform();
  btMatrix3x3 robotToWorld = worldTrans.getBasis();
  //std::cout << robotToWorld.getRow(0) << std::endl;
  //std::cout << robotToWorld.getRow(1) << std::endl;
  //std::cout << robotToWorld.getRow(2) << std::endl;
  // The basis of getWorldTransform() returns the rotation matrix from robot frame
  // to world frame. Invert this matrix to go from world to robot frame
  btMatrix3x3 worldToRobot = robotToWorld.inverse();
  // Transform the gravity vector from world frame to robot frame
  btVector3 gravVectRobot = worldToRobot * gravVectWorld;
  //std::cout << "Gravity vector in robot frame: " << gravVectRobot << std::endl;
  return gravVectRobot;
}
*/

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

/*
std::vector<int> T6RollingController::findPath(std::vector< std::vector<int> >& adjMat, int startNode, int endNode) 
{
  // Check validity of start and end nodes
  int nodes = adjMat.size();
  if (startNode > nodes) {
    std::cout << "findPath: Start node out of bounds, exiting..." << std::endl;
    exit(EXIT_FAILURE);
  }
  else if (endNode > nodes) {
    std::cout << "findPath: End node out of bounds, exiting..." << std::endl;
    exit(EXIT_FAILURE);
  }

  // Initialize status flags for reached destination or no solution
  bool endReached = false;
  bool noSolution = false;

  if (endNode == startNode) {
    endReached = true;
  }

  // Create vectors to hold unvisited and visited sets and distances
  // the unvisited set initializs with all nodes and the visited set
  // initializes as an empty vector
  std::vector<int> unvisited;
  for (int i = 0; i < nodes; i++) {
    unvisited.push_back(i);
  }
  //std::cout << "Unvisited init: ";
  //utility::printVector(unvisited);
  std::vector<int> visited;
  std::vector<int> distances(nodes, 1000); // Intialize distance vector to some arbitrarily large value
  distances[startNode] = 0;

  // Initialize path vectors
  std::vector<int> prev(nodes, -1); // Initialize path pointer vector to -1
  std::vector<int> pathVect;
  pathVect.push_back(endNode); // Last element in pathVect is the destination node

  while ((endReached == false) && (noSolution == false)) {
    // Initialize variables
    bool currNodeFound = false;
    int currNode = -1;
    int minDist = 1000;

    // Find next unvisited node with the shortest tentative distance
    for (size_t uvNode_idx = 0; uvNode_idx < unvisited.size(); uvNode_idx++) {
      if (distances[unvisited[uvNode_idx]] < minDist) {
	minDist = distances[unvisited[uvNode_idx]];
	currNode = unvisited[uvNode_idx];
	currNodeFound = true;
      }
    }
    //std::cout << "Current node : " << currNode << std::endl;

    // Check if new node is found
    if (currNodeFound == false) {
      std::cout << "findPath: No solution found" << std::endl;
      noSolution = true;
    }

    // If node is found, continue with path finding
    if (noSolution == false) {
      // Extract row corresponding to the current node
      std::vector<int> currRow = adjMat[currNode];
      std::vector<int> neighbors;
      std::vector<int> weights;
      // Find its neighbors and their corresponding weights
      for (size_t col_idx = 0; col_idx < currRow.size(); col_idx++) {
	if (currRow[col_idx] > 0) {
	  neighbors.push_back(col_idx);
	  weights.push_back(currRow[col_idx]);
	}
      }
      //utility::printVector(neighbors);
      //utility::printVector(weights);
      // Check if neighbors have been visited already, if not calculate distance
      for (size_t neigh_idx = 0; neigh_idx < neighbors.size(); neigh_idx++) {
	if (find(visited.begin(), visited.end(), neighbors[neigh_idx]) != visited.end()) {
	  continue;
	}
	else if (neighbors[neigh_idx] == endNode) {
	  prev[endNode] = currNode;
	  endReached = true;
	  break;
	}
	else {
	  int tentativeDistance = distances[currNode] + weights[neigh_idx];
	  if (tentativeDistance < distances[neighbors[neigh_idx]]) {
	    distances[neighbors[neigh_idx]] = tentativeDistance;
	    prev[neighbors[neigh_idx]] = currNode;
	  }
	}
      }

      // Finished investigating current node, move it from unvisted to visited
      visited.push_back(currNode);
      unvisited.erase(find(unvisited.begin(), unvisited.end(), currNode));
      //utility::printVector(visited);
      //utility::printVector(unvisited);
    }
  }

  int node = endNode;
  while (node != startNode) {
    pathVect.insert(pathVect.begin(), prev[node]);
    node = prev[node];
  }

  //std::cout << "End Reached: " << endReached << ", No Solution: " << noSolution << std::endl;
  return pathVect;
}
*/

/*
bool T6RollingController::stepToFace(double dt)
{
  // Initialize flags
  bool stepFinished = false;
  bool isOnPath = false;
  // Length for cables to retract to
  //double controlLength = 0.2;
  double controlLength = restLength * 0;
	
  int cableToActuate = -1;
  // Get which cable to actuate from actuation policy table
  if (path.size() > 1) {
    cableToActuate = actuationPolicy[path[0]][path[1]];

    // Find current face
    int currFace = contactSurfaceDetection(0);

    // Perform actuation from one closed face to another
    if (isClosedFace(path[0])) {
      if (cableToActuate >= 0) {
	// path[0] is current face, path[1] is the adjacent open face, 
	// path[2] is the next closed face
	// Check if the robot has reached the next closed face
	if (currFace != path[2]) {
	  m_controllers[cableToActuate]->control(dt, controlLength);
	  //actuators[cableToActuate]->moveMotors(dt);
	  // std::cout << "stepToFace: (Closed -> Closed) Stepping..." << std::endl;
	  resetCounter++;
	  if (resetCounter > 3.0/dt) resetFlag = true;
	}
	// If it has, return all cables to rest length
	else {
	  resetFlag = true;
	  path.erase(path.begin(),path.begin()+2);
	  utility::printVector(path);
	  if (path.size() == 1) {
	    stepFinished = true;
	  }
	}
      }
      // Triggers if element called from actuation policy table is -1
      else {
	std::cout << "stepToFace: No actuation scheme available, exiting..." << std::endl;
	//exit(EXIT_FAILURE);
      }
    }
    // Perfom actuation to get from an open face to a closed face
    else {
      if (cableToActuate >= 0) {
	// Check to see if robot has reached a closed face
	if (!isClosedFace(currFace)) {
	  m_controllers[cableToActuate]->control(dt, controlLength);
	  //actuators[cableToActuate]->moveMotors(dt);
	  // std::cout << "stepToFace: (Open -> Closed) Stepping..." << std::endl;
	  resetCounter++;
	  if (resetCounter > 3.0/dt) resetFlag = true;
	}
	// If it has, return all cables to rest length
	else {
	  resetFlag = true;
	  path.erase(path.begin());
	  utility::printVector(path);
	  stepFinished = true;
	}
      }
      // Triggers if element called from actuation policy table is -1
      else {
	std::cout << "stepToFace: No actuation scheme available, exiting..." << std::endl;
	//exit(EXIT_FAILURE);
      }
    }
  }
  else stepFinished = true;

	
	
  return stepFinished;
}
*/


/*
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
*/
