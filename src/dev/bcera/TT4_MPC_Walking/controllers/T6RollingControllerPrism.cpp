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
  matrix<double> FaceSide_IW(20,12);
  matrix<double> FaceSide_LW1(20,20);
  matrix<double> FaceSide_LW2(23,20);
  vector<double> FaceSide_b1(20);
  vector<double> FaceSide_b2(20);
  vector<double> FaceSide_b3(23);
  vector<double> FaceSide_input_xmin(12);
  vector<double> FaceSide_input_xmax(12);
  vector<double> FaceSide_output_xmin(23);
  vector<double> FaceSide_output_xmax(23);

  //Matrices holding Neural Net weights for cable actuation
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

  //NN Input Layer Weights
  std::ifstream file1 ( "FaceSide_IW.csv" );
  std::string value1;
  for(size_t i=0; i<20; i++){
    for(size_t j=0; j<12; j++){
      getline(file1,value1,',');
      //std::cout<<value<<std::endl;
      FaceSide_IW(i,j) = std::atof(value1.c_str());
    }
  }

  //NN Hidden Layer 1 Weights
  std::ifstream file2 ( "FaceSide_LW1.csv" );
  std::string value2;
  for(size_t i=0; i<20; i++){
    for(size_t j=0; j<20; j++){
      getline(file2,value2,',');
      //std::cout<<value<<std::endl;
      FaceSide_LW1(i,j) = std::atof(value2.c_str());
    }
  }

  //NN Hidden Layer 2 Weights
  std::ifstream file3 ( "FaceSide_LW2.csv" );
  std::string value3;
  for(size_t i=0; i<23; i++){
    for(size_t j=0; j<20; j++){
      getline(file3,value3,',');
      //std::cout<<value<<std::endl;
      FaceSide_LW2(i,j) = std::atof(value3.c_str());
    }
  }

  //NN Hidden Layer 1 Bias
  std::ifstream file4 ( "FaceSide_b1.csv" );
  std::string value4;
  for(size_t i=0; i<20; i++){
      getline(file4,value4,',');
      //std::cout<<value<<std::endl;
      FaceSide_b1(i) = std::atof(value4.c_str());
  }

  //NN Hidden Layer 2 Bias
  std::ifstream file5 ( "FaceSide_b2.csv" );
  std::string value5;
  for(size_t i=0; i<20; i++){
      getline(file5,value5,',');
      //std::cout<<value<<std::endl;
      FaceSide_b2(i) = std::atof(value5.c_str());
  }

  //NN Output Layer Bias
  std::ifstream file6 ( "FaceSide_b3.csv" );
  std::string value6;
  for(size_t i=0; i<23; i++){
      getline(file6,value6,',');
      //std::cout<<value<<std::endl;
      FaceSide_b3(i) = std::atof(value6.c_str());
  }

  //NN Input Layer Min
  std::ifstream file7 ( "FaceSide_input_xmin.csv" );
  std::string value7;
  for(size_t i=0; i<12; i++){
      getline(file7,value7,',');
      //std::cout<<value<<std::endl;
      FaceSide_input_xmin(i) = std::atof(value7.c_str());
  }

  //NN Input Layer Max
  std::ifstream file8 ( "FaceSide_input_xmax.csv" );
  std::string value8;
  for(size_t i=0; i<12; i++){
      getline(file8,value8,',');
      //std::cout<<value<<std::endl;
      FaceSide_input_xmax(i) = std::atof(value8.c_str());
  }

  //NN Output Layer Min
  std::ifstream file9 ( "FaceSide_output_xmin.csv" );
  std::string value9;
  for(size_t i=0; i<23; i++){
      getline(file9,value9,',');
      //std::cout<<value<<std::endl;
      FaceSide_output_xmin(i) = std::atof(value9.c_str());
  }

  //NN Output Layer Max
  std::ifstream file10 ( "FaceSide_output_xmax.csv" );
  std::string value10;
  for(size_t i=0; i<23; i++){
      getline(file10,value10,',');
      //std::cout<<value<<std::endl;
      FaceSide_output_xmax(i) = std::atof(value10.c_str());
  }

  



  
  std::ifstream file11 ( "RL_mat.csv" );
  std::string value11;
  for(size_t i=0; i<20*3; i++){
    for(size_t j=0; j<24; j++){
      getline(file11,value11,',');
      //std::cout<<value<<std::endl;
      RL_mat(i,j) = std::atof(value11.c_str());
    }
  }

  std::ifstream file12 ( "F_mat.csv" );
  std::string value12;
  for(size_t i=0; i<20; i++){
    for(size_t j=0; j<3; j++){
      getline(file12,value12,',');
      std::cout<<value12<<std::endl;
      F_mat(i,j) = (int)std::atof(value12.c_str());
    }
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


  /*
    std::cout << "RL: " << std::endl;
    for(size_t i=0; i<24; i++){
    std::cout << RL_mat((currFace+1)*3-3,i) << ", ";
    }
  */

  if(priorFace==currFace)
    idleCount++;
  else{
    idleCount = 0;
    priorFace = currFace;
  }
  //std::cout<<"idleCount: "<<idleCount<<std::endl;

  /*
  //Finding Desired Side to tip over
  //if(priorFace!=currFace){
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
  */
  //  priorFace = currFace;
  //}
    

  //Actuation of Cable Restlengths
  //std::cout << "min_idx: " << min_idx << std::endl;
  for(int i=0;i<24;i++){
    m_controllers[i]->control(dt,RL_mat((currFace)*3+0,i)*sf/0.66); //scale by scaling factor here
    //m_controllers[i]->control(dt,sequence[i]*sf); //scale by scaling factor here
    //std::cout << "Current Control: " << sequence[i]*sf << ", ";
    //std::cout << "Start Length: " << startLength << std::endl;
    //std::cout << "Actuator" <<  i << ": " << actuators[i]->getRestLength() << ", ";
    
    //actuators[i]->moveMotors(dt);
  }
  //std::cout << "Roll Case: " << roll_case << std::endl;
  //std::cout << "WorldTime: " << worldTime << std::endl;
  
  /*
    float time_inc;
    if(roll_case%2==0)
    time_inc = 1.335;//1.425;
    else
    time_inc = 1.3;
    if((worldTime-last_step_time)>time_inc){
    roll_case = (roll_case+1)%12;
    last_step_time = worldTime;
    }
  */
  
  
  //Contact Surface Detection
  if(fmod(worldTime,0.1)<=dt){
    currFace = contactSurfaceDetection(currFace);
    std::cout << "Curr Face: " << currFace << std::endl;
    int mat_currFace = currFace + 1; //matlab 1-index

    //roll_case = (roll_case+1)%12;
    std::cout << "Roll Case: " << roll_case << std::endl; 
  }
    
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
  vector<double> Input(12);
  Input <<= target_theta, phiInput, relthetaInput;
  std::cout << "Input size: " << Input.size() << std::endl;

  //mapminmax pre-process feature data
  scalar_vector <double> ones12(12,1);
  Input = element_div(2*(Input-FaceSide_input_xmin),FaceSide_input_xmax-FaceSide_input_xmin)-ones12;
  
  for(size_t i=0; i<Input.size() ; i++){
    std::cout << Input(i) << ", ";
  }
  std::cout << std::endl;
  
  
  
  //multiply inputs by input weights - HIDDEN LAYER 1
  vector<double> hiddenLayer(20);
  axpy_prod(FaceSide_IW,Input,hiddenLayer,true); //multiply by weights
  /*
    for(size_t i=0; i<hiddenLayer.size() ; i++){
    std::cout << hiddenLayer(i) << ", ";
    }
  */
  std::cout << std::endl;
  hiddenLayer += FaceSide_b1; //add bias
  /*
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
    for(size_t i=0; i<hiddenLayer.size() ; i++){
    std::cout << hiddenLayer(i) << ", ";
    }
    std::cout << std::endl;
  */

  //multiply inputs by input weights - HIDDEN LAYER 2
  vector<double> hiddenLayer2(20);
  axpy_prod(FaceSide_LW1,hiddenLayer,hiddenLayer2,true); //multiply by weights
  std::cout << std::endl;
  hiddenLayer2 += FaceSide_b2; //add bias

  //tansig function (sigmoid with output mapped [-1,1])
  for(size_t i=0; i<hiddenLayer2.size(); i++){
    hiddenLayer2(i) = 2/(1+exp(-2*hiddenLayer2(i)))-1;
  }


  //multiply hidden layer 2 outputs by layer1 weights
  vector<double> outputLayer(23);
  axpy_prod(FaceSide_LW2,hiddenLayer2,outputLayer,true); //multiply by weights
  std::cout << "hidden layer multiplied with weights: " << std::endl;
  /*
    for(size_t i=0; i<outputLayer.size() ; i++){
    std::cout << outputLayer(i) << ", ";
    }
    std::cout << std::endl;
  */
  outputLayer += FaceSide_b3; //add bias
  /*
    for(size_t i=0; i<outputLayer.size() ; i++){
    std::cout << outputLayer(i) << ", ";
    }
    std::cout << std::endl;
  */

  //mapminmax post-process output layer
  scalar_vector <double> ones23(23,1);
  //outputLayer = element_div(2*(outputLayer-FaceSide_output_xmin),FaceSide_output_xmax-FaceSide_output_xmin)-ones23;

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
  
  //softmax function for output layer (Faces)
  std::cout << "Face Detection Probabilities: " << std::endl;
  double sum2 = 0;
  for(size_t i=3; i<outputLayer.size(); i++){
    outputLayer(i) = exp(outputLayer(i));
    sum2 += outputLayer(i);
  }
  for(size_t i=3; i<outputLayer.size(); i++){
    outputLayer(i) = outputLayer(i)/sum2;
    std::cout << outputLayer(i) << ", ";
  }
  std::cout << std::endl;
  */
  
  /*
    int currSurface = prevFace;
    double threshold = 0.05;
    int otherMax = -1;
    std::cout << "Prev Face: " << prevFace << std::endl;
    for(int i=0; i<outputLayer.size(); i++){
    if((otherMax==-1||outputLayer(i)>outputLayer(otherMax)) && outputLayer(i)>=threshold && i!=prevFace){
    currSurface = i;
    otherMax = i;
    }
    }
  */
  double threshold = 0.70;
  currSurface = prevFace;
  for(int i=3; i<outputLayer.size(); i++){
    if((outputLayer(i)>outputLayer(currSurface+3)) && outputLayer(i)>=threshold){
      currSurface = i-3;
    }
  }
  std::cout << "Contact Surface: Face " << currSurface+1 << " with Probability " << outputLayer(currSurface+3) << std::endl;
  
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
