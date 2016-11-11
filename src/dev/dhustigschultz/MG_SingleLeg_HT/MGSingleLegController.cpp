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
 * @file MGSingleLegController.cpp
 * @brief Implementing a hand-tuned controller for MountainGoat's legs.
 * @author Dawn Hustig-Schultz
 * @date Aug 2016
 * @version 1.1.0
 * $Id$
 */

//This module
#include "MGSingleLegController.h"
//This application
#include "dev/dhustigschultz/MountainGoat2/MountainGoat2.h"

//This library
#include "core/tgSpringCableActuator.h"
#include "core/tgBasicActuator.h"
#include "core/tgKinematicActuator.h"

//The C++ standard library
#include <string>
#include <stdexcept>
#include <cassert>
#include <vector>
#include <cmath>


# define M_PI 3.14159265358979323846    

MGSingleLegController::MGSingleLegController(double timestep) :
    m_totalTime(0.0),
    dt(timestep) {}

void MGSingleLegController::onSetup(BaseQuadModelLearning& subject){
    this->m_totalTime=0.0;
}
    
void MGSingleLegController::onStep(BaseQuadModelLearning& subject, double dt){

    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;    

    setLegMuscleTargetLength(subject, dt);

    moveAllMotors(subject, dt);
}

void MGSingleLegController::onTeardown(BaseQuadModelLearning& subject) {

} 

void MGSingleLegController::moveAllMotors(BaseQuadModelLearning& subject, double dt){

    const std::vector<tgSpringCableActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
        tgBasicActuator * const pMuscle = tgCast::cast<tgSpringCableActuator, tgBasicActuator>(muscles[i]);
        assert(pMuscle != NULL);
        pMuscle->moveMotors(dt);
    }

}

void MGSingleLegController::setLegMuscleTargetLength(BaseQuadModelLearning& subject, double dt){

    double newLengthMid = 0;
    double newLengthFront = 0;
    double newLengthRear = 0;
    double newLengthOuterTricep = 0; 
 
    const double midLength = 15;
    const double rearLength = sqrt(126);
    const double frontLength = sqrt(126);
    const double outerTricepLength = sqrt(185);

    const double amplitude1 = midLength;
    const double amplitude2 = frontLength;
    const double amplitude3 = rearLength;
    const double amplitude4 = outerTricepLength/2;

    const double angular_freq = 8;

    const double phaseMid = 3*M_PI/2;
    const double phaseFront = phaseMid;
    const double phaseRear = phaseMid;
    const double phaseOuterTricep = phaseMid;
    
    const double dcOffset = midLength;
    const double dcOffset2 = frontLength;
    const double dcOffset3 = rearLength;
    const double dcOffset4 = outerTricepLength/2;

    const std::vector<tgBasicActuator*> Mid = subject.find<tgBasicActuator>("mid_bicep");
    const std::vector<tgBasicActuator*> Rear = subject.find<tgBasicActuator>("rear_abdomen");
    const std::vector<tgBasicActuator*> Front = subject.find<tgBasicActuator>("frontAbdomen");
    const std::vector<tgBasicActuator*> outerTricep = subject.find<tgBasicActuator>("outer_tricep");    

    assert(Mid[0] != NULL);
    assert(Rear[0] != NULL);
    assert(Front[0] != NULL);
    assert(outerTricep[0] != NULL);

    newLengthMid = amplitude1 * sin(angular_freq * m_totalTime + phaseMid) + dcOffset;
    newLengthFront = amplitude2 * sin(angular_freq * m_totalTime + phaseFront) + dcOffset2;
    newLengthRear = amplitude3 * sin(angular_freq * m_totalTime + phaseRear) + dcOffset3;
    newLengthOuterTricep = outerTricepLength/2; 

    for (std::size_t i=0; i < Mid.size(); i++){
    	Mid[i]->setControlInput(newLengthMid);
        Rear[i]->setControlInput(newLengthRear);
        outerTricep[i]->setControlInput(newLengthOuterTricep);
    }
    for (std::size_t i=0; i < Front.size(); i++){
	Front[i]->setControlInput(newLengthFront);
    }

}
