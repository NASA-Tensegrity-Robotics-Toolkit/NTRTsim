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
 * @file BigPuppyController.cpp
 * @brief Implementing a hand-tuned controller for a quadruped based roughly on the Flemons BigPuppy model.
 * @author Dawn Hustig-Schultz
 * @date July 2015
 * @version 1.0.0
 * $Id$
 */

//This module
#include "BigPuppyController.h"
//This application
#include "BigPuppy.h"

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

BigPuppyController::BigPuppyController(double timestep) :
    m_totalTime(0.0),
    dt(timestep) {}

void BigPuppyController::onSetup(BigPuppy& subject){
    this->m_totalTime=0.0;

    
}
    
void BigPuppyController::onStep(BigPuppy& subject, double dt){

    // Update controller's internal time
    if (dt <= 0.0) { throw std::invalid_argument("dt is not positive"); }
    m_totalTime+=dt;    

    setBicepTargetLength(subject, dt);
    setFrontTricepTargetLength(subject, dt);
    setRearTricepTargetLength(subject, dt);
    setLegToAbdomenTargetLength(subject, dt);
    moveAllMotors(subject, dt);

    if(m_totalTime > 15) {
        m_totalTime = 0;
    }
}

void BigPuppyController::onTeardown(BigPuppy& subject) {

} 

void BigPuppyController::moveAllMotors(BigPuppy& subject, double dt){

    const std::vector<tgSpringCableActuator*> muscles = subject.getAllMuscles();
    for (size_t i = 0; i < muscles.size(); ++i) {
        tgBasicActuator * const pMuscle = tgCast::cast<tgSpringCableActuator, tgBasicActuator>(muscles[i]);
        assert(pMuscle != NULL);
        pMuscle->moveMotors(dt);
    }

}

void BigPuppyController::setBicepTargetLength(BigPuppy& subject, double dt){

    double newLengthMid = 0;
    double newLengthOuter = 0;
    const double bicepLength = 15.0;

    const double amplitude = bicepLength/2;
    const double angular_freq = 2;
    const double phaseMid = M_PI/2;
    const double phaseOuter = phaseMid + M_PI;
    const double dcOffset = bicepLength/2;

    const std::vector<tgBasicActuator*> bicepMid = subject.find<tgBasicActuator>("right mid bicep");
    const std::vector<tgBasicActuator*> bicepOuter = subject.find<tgBasicActuator>("right outer bicep");
    const std::vector<tgBasicActuator*> bicepInner = subject.find<tgBasicActuator>("right inner bicep");

    assert(bicepMid[0] != NULL);
    assert(bicepOuter[0] != NULL);
    assert(bicepInner[0] != NULL);

    newLengthMid = amplitude * sin(angular_freq * m_totalTime + phaseMid) + dcOffset;
    newLengthOuter = amplitude * sin(angular_freq * m_totalTime + phaseOuter) + 2*dcOffset;
    
    bicepMid[0]->setControlInput(newLengthMid, dt);
    bicepOuter[0]->setControlInput(newLengthOuter, dt);
    bicepInner[0]->setControlInput(newLengthOuter, dt);

}

void BigPuppyController::setFrontTricepTargetLength(BigPuppy& subject, double dt){

    double newLength = 0;
    const double frontTricepLength = sqrt(245.0);

    const double amplitude = frontTricepLength/2;
    const double angular_freq = 2;
    const double phase = 3*M_PI/2;
    const double dcOffset = frontTricepLength;

    const std::vector<tgBasicActuator*> frontTricep = subject.find<tgBasicActuator>("right front tricep");

    for (size_t i=0; i < frontTricep.size(); i++) {
        tgBasicActuator * const pMuscle = frontTricep[i];
        assert(pMuscle != NULL);
        newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;
        
        pMuscle->setControlInput(newLength, dt);
    }
    
}

void BigPuppyController::setRearTricepTargetLength(BigPuppy& subject, double dt){

    double newLength = 0;
    const double rearTricepLength = sqrt(165);

    const double amplitude = rearTricepLength/2; 
    const double angular_freq = 2;
    const double phase = 3*M_PI/2;
    const double dcOffset = rearTricepLength;

    const std::vector<tgBasicActuator*> rearTricep = subject.find<tgBasicActuator>("right tricep");

    for (size_t i=0; i < rearTricep.size(); i++) {
        tgBasicActuator * const pMuscle = rearTricep[i];
        assert(pMuscle != NULL);
        newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;

        pMuscle->setControlInput(newLength, dt);
    }
}

void BigPuppyController::setLegToAbdomenTargetLength(BigPuppy& subject, double dt){

    double newLength = 0;
    const double legToAbdomenLength = sqrt(66);

    const double amplitude = 0; //See if this needs to change much first before throwing in an amplitude.
    const double angular_freq = 2;
    const double phase = 3*M_PI/2;
    const double dcOffset = legToAbdomenLength;
 
    const std::vector<tgBasicActuator*> legToAbdomen = subject.find<tgBasicActuator>("right front abdomen");

    assert(legToAbdomen[0] != NULL);

    newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;

    legToAbdomen[0]->setControlInput(newLength, dt);

}

