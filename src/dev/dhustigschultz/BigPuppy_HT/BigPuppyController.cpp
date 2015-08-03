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
    setRightShoulderTargetLength(subject, dt);
    moveAllMotors(subject, dt);
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

    const double amplitude1 = bicepLength/2;
    const double amplitude2 = bicepLength/2;
    const double angular_freq = 2;
    const double phaseMid = 3*M_PI/2;
    const double phaseOuter = phaseMid - M_PI;
    const double dcOffset = bicepLength/2;

    const std::vector<tgBasicActuator*> bicepMid = subject.find<tgBasicActuator>("right mid bicep");
    const std::vector<tgBasicActuator*> bicepOuter = subject.find<tgBasicActuator>("right outer bicep");
    const std::vector<tgBasicActuator*> bicepInner = subject.find<tgBasicActuator>("right inner bicep");

    assert(bicepMid[0] != NULL);
    assert(bicepOuter[0] != NULL);
    assert(bicepInner[0] != NULL);

    newLengthMid = amplitude1 * sin(angular_freq * m_totalTime + phaseMid) + dcOffset;
    newLengthOuter = amplitude2 * sin(angular_freq * m_totalTime + phaseOuter) + 2*dcOffset;
    
    bicepMid[0]->setControlInput(newLengthMid);
    bicepOuter[0]->setControlInput(newLengthOuter);
    bicepInner[0]->setControlInput(newLengthOuter);

}

void BigPuppyController::setFrontTricepTargetLength(BigPuppy& subject, double dt){

    double newLengthInner = 0;
    double newLengthOuter = 0;
    const double frontTricepLength = sqrt(245.0);

    const double amplitude = frontTricepLength/4;
    const double angular_freq = 2;
    const double phase1 = M_PI/2;
    const double phase2 = 3*M_PI/2;
    const double dcOffset = frontTricepLength;


    const std::vector<tgBasicActuator*> frontOuterTricep = subject.find<tgBasicActuator>("outer right front tricep");
    const std::vector<tgBasicActuator*> frontInnerTricep = subject.find<tgBasicActuator>("inner right front tricep");

    assert(frontOuterTricep[0] != NULL);
    assert(frontInnerTricep[0] != NULL);
    
    newLengthInner = dcOffset - amplitude * sin(angular_freq * m_totalTime + phase1);
    newLengthOuter = dcOffset - amplitude * sin(angular_freq * m_totalTime + phase2);

    frontOuterTricep[0]->setControlInput(newLengthOuter);
    frontInnerTricep[0]->setControlInput(newLengthInner);

}

void BigPuppyController::setRearTricepTargetLength(BigPuppy& subject, double dt){

    double newLengthInner = 0;
    double newLengthOuter = 0;
    const double rearTricepLength = sqrt(165);

    const double amplitude = rearTricepLength/4; 
    const double angular_freq = 2;
    const double phase1 = 3*M_PI/2;
    const double phase2 = M_PI/2;
    const double dcOffset = rearTricepLength;

    const std::vector<tgBasicActuator*> rearOuterTricep = subject.find<tgBasicActuator>("outer right tricep");
    const std::vector<tgBasicActuator*> rearInnerTricep = subject.find<tgBasicActuator>("inner right tricep");

    assert(rearOuterTricep[0] != NULL);
    assert(rearInnerTricep[0] != NULL);
    
    newLengthInner = dcOffset - amplitude * sin(angular_freq * m_totalTime + phase1);
    newLengthOuter = dcOffset - amplitude * sin(angular_freq * m_totalTime + phase2);

    rearOuterTricep[0]->setControlInput(newLengthOuter);
    rearInnerTricep[0]->setControlInput(newLengthInner);

}

void BigPuppyController::setLegToAbdomenTargetLength(BigPuppy& subject, double dt){

    double newLength = 0;
    const double legToAbdomenLength = sqrt(66);

    const double amplitude = legToAbdomenLength/1; //See if this needs to change much first before throwing in an amplitude.
    const double angular_freq = 2;
    const double phase = 3*M_PI/2;
    const double dcOffset = legToAbdomenLength;
 
    const std::vector<tgBasicActuator*> legToAbdomen = subject.find<tgBasicActuator>("right front abdomen");

    assert(legToAbdomen[0] != NULL);

    newLength = amplitude * sin(angular_freq * m_totalTime + phase) + dcOffset;

    legToAbdomen[0]->setControlInput(newLength);

}


void BigPuppyController::setRightShoulderTargetLength(BigPuppy& subject, double dt){

    double newLengthFront = 0;
    double newLengthRear = 0;
    const double rightShoulderLength = sqrt(101);

    const double amplitude = rightShoulderLength/2; //See if this needs to change much first before throwing in an amplitude.
    const double angular_freq = 2;
    const double phaseRear = 3*M_PI/2;
    const double phaseFront = M_PI/2;
    const double dcOffset = rightShoulderLength;
 
    const std::vector<tgBasicActuator*> rightShoulderFront = subject.find<tgBasicActuator>("right shoulder front mid");
    const std::vector<tgBasicActuator*> rightShoulderRear = subject.find<tgBasicActuator>("right shoulder rear mid");

    assert(rightShoulderFront[0] != NULL);
    assert(rightShoulderRear[0] != NULL);

    newLengthFront = dcOffset - amplitude * sin(angular_freq * m_totalTime + phaseFront);
    newLengthRear = dcOffset - amplitude * sin(angular_freq * m_totalTime + phaseRear);

    rightShoulderFront[0]->setControlInput(newLengthFront);
    rightShoulderRear[0]->setControlInput(newLengthRear);

}
