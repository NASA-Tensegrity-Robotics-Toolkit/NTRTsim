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


// This module
#include "RestlenPlaybackController.h"
// This application
#include "SUPERballV2Model.h"
#include "core/tgBasicActuator.h"
#include "core/tgSpringCable.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <cstdlib>

using namespace std;

RestlenPlaybackController::RestlenPlaybackController(const char* cmdFile, int cmdStoreHz)
: cmdStoreFreq(cmdStoreHz)
{
    ifstream file(cmdFile);
    if (!file) {
        cerr << "Invalid controller file to open: " << cmdFile << endl;
        return;
    }
    
    // Load in the command file:
    string line;
    while (std::getline(file, line)) {
        //cout << "CSV Line: " << line << endl;
        std::istringstream s2(line);
        string value;
        std::vector<double> vl;
        while (getline(s2, value, ',')) {
            //cout << "Value: " << value << endl;
            double value_double = atof(value.c_str());
            vl.push_back(value_double);
        }
        cmdDataStore.push_back(vl);
    }
}

RestlenPlaybackController::~RestlenPlaybackController()
{
}

// Interpolates the command vector for the given time t.
std::vector<double> RestlenPlaybackController::interpolateForTime(double time) {
    double idx_full = time * ((double)cmdStoreFreq);
    int idx_round = (int)idx_full;
    int idx_next = idx_round + 1;
    double idx_remainder = idx_full - double(idx_round);
    
    // Bounds checking:
    if (idx_round < 0) {
        idx_round = 0;
        idx_next = 0;
    }
    if (idx_round == cmdDataStore.size() - 1) idx_next = idx_round;
    if (idx_round >= cmdDataStore.size()) {
        idx_round = cmdDataStore.size() - 1;
        idx_next = idx_round;
    }
    //cout << idx_round << " " << idx_next << " " << cmdDataStore.size() << endl;
    
    std::vector<double> base = cmdDataStore[idx_round];
    std::vector<double> next = cmdDataStore[idx_next];
    
    for (int n = 0; n < base.size(); n++) {
        base[n] = base[n] + (next[n] - base[n]) * idx_remainder;
        //cout << base[n] << endl;
    }
    return base;
}

void RestlenPlaybackController::onSetup(SUPERballV2Model& subject)
{
  m_controllers.clear(); //clear vector of controllers
  
  //get all of the tensegrity structure's cables
  actuators = subject.getAllActuators();

  //Attach a tgBasicController to each actuator
  for (size_t i = 0; i < actuators.size(); ++i)
    {
      tgBasicActuator * const pActuator = actuators[i];
      assert(pActuator != NULL);
      //instantiate controllers for each cable
      tgBasicController* m_lenController = new tgBasicController(pActuator);
      //add controller to vector
      m_controllers.push_back(m_lenController);
    }
    
    // Set starting time.
    globalTime = -1;
}

void RestlenPlaybackController::onStep(SUPERballV2Model& subject, double dt)
{
    if (dt < 0) {
        throw std::invalid_argument("dt is not positive");
        return;
    }
    globalTime += dt;
    
    double playTime = cmdDataStore.size()/(double)cmdStoreFreq;
    double t = playTime - abs(globalTime - playTime);
    //std::cout<<"t: "<<t<<std::endl;
    //t=0;
    // Get command vector: [actual lengths, tensions]
    std::vector<double> cmdVec = interpolateForTime(t);
    
    // Implement rest lengths:
    for (int i = 0; i < 24; i++) {
        // Apply world scaling to lengths and tensions:
        double actuallencmd = cmdVec[i] * 10.0;
        double tensioncmd = cmdVec[i+24] * 10.0; // These are already capped to 200N.
        
        // Add to tension to apply rest-length correction for finite radius rods
        tensioncmd += 0.1*10.0*1600.0;
        if (abs(t-playTime) < 1.0) tensioncmd *= abs(t-playTime); // Turn off tension for a short time.
        
        //if (tensioncmd>200.0*10.0)
        //    tensioncmd = 200.0*10.0; // Limit tension command to 200.0N.
        
        cout << i << " " << actuallencmd << " " << tensioncmd << endl;
        
        setRestLengthWithCmdTensionLength(i, tensioncmd, actuallencmd, dt);
    }
}

// Command rest length for the specified actuator
void RestlenPlaybackController::setRestLengthWithCmdTensionLength(int actuatorID, double tension_cmd, double length_cmd, double dt)
{
    const tgSpringCable* m_springCable = actuators[actuatorID]->getSpringCable();
    // Rest length control:
    // Note, we shorten rest lengths by 10cm, to account for rod thickness!
    double commandRestLen = length_cmd - tension_cmd/(m_springCable->getCoefK());

    if (commandRestLen < 0) commandRestLen = 0;
    m_controllers[actuatorID]->control(dt, commandRestLen);
    actuators[actuatorID]->moveMotors(dt);
}
