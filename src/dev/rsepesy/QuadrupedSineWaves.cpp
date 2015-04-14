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
 * @file QuadrupedSineWaves.cpp
 * @brief Contains the implementation of class QuadrupedSineWaves
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// This module
#include "QuadrupedSineWaves.h"

// Its subject
#include "examples/learningSpines/BaseSpineModelLearning.h"
#include "Quadruped.h"

// NTRTSim
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"
#include "tgcreator/tgUtil.h"

QuadrupedSineWaves::QuadrupedSineWaves() :
    in_controller(new tgImpedanceController(100, 500, 50)),
    out_controller(new tgImpedanceController(100, 500, 100)),
    segments(1.0),
    insideLength(16.5),
    outsideLength(19.5),
    offsetSpeed(0.0),
    cpgAmplitude(50.0),
    cpgFrequency(2.51),
    bodyWaves(2.0),
    simTime(0.0),
    cycle(0.0),
    target(0.0)
{
    phaseOffsets.clear();
    phaseOffsets.push_back(M_PI/2);
    phaseOffsets.push_back(0);
    phaseOffsets.push_back(0);
}

QuadrupedSineWaves::~QuadrupedSineWaves()
{
	delete in_controller;
	delete out_controller;
}

void QuadrupedSineWaves::applyImpedanceControlInside(const std::vector<tgBasicActuator*> stringList, double dt)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        double setTension = in_controller->control(*(stringList[i]),
                                            dt,
                                            insideLength
                                            );
        #if (0) // Conditional compile for verbose control
        std::cout << "Inside String " << i << " tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }    
}

void QuadrupedSineWaves::applyImpedanceControlOutside(const std::vector<tgBasicActuator*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        cycle = sin(simTime * cpgFrequency + 2 * bodyWaves * M_PI * i / (segments) + phaseOffsets[phase]);
        target = offsetSpeed + cycle*cpgAmplitude;
        
        double setTension = out_controller->control(*(stringList[i]),
                                            dt,
                                            outsideLength,
                                            target
                                            );
        #if(0) // Conditional compile for verbose control
        std::cout << "Outside String " << i << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }    
}

void QuadrupedSineWaves::onStep(BaseSpineModelLearning& subject, double dt)
{
    simTime += dt;
    
    const Quadruped* qSubject = tgCast::cast<BaseSpineModelLearning, Quadruped>(subject);

    //segments = subject.getSegments();
    
    applyImpedanceControlInside(qSubject->getActuators("pull"), dt);
    /*
    applyImpedanceControlInside(subject.getActuators("inner left") , dt);
    applyImpedanceControlInside(subject.getActuators("inner right"), dt);
    */
    applyImpedanceControlOutside(qSubject->getActuators("pull"), dt, 0);
    /*
    applyImpedanceControlOutside(subject.getActuators("outer left"), dt, 1);
    applyImpedanceControlOutside(subject.getActuators("outer right"), dt, 2);
    */
}


