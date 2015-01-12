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
 * @file DuCTTMechTestController.cpp
 * @brief Contains the implementation of the class DuCTTMechTestController
 * @author Alexander Xydes
 * $Id$
 */

// This module
#include "DuCTTMechTestController.h"

// Its subject
#include "../robot/DuCTTRobotModel.h"
#include "../robot/tgPrismatic.h"
#include "../robot/tgTouchSensorSphereModel.h"

#include "core/abstractMarker.h"
#include "core/tgBasicActuator.h"
#include "controllers/tgImpedanceController.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>

DuCTTMechTestController::DuCTTMechTestController(double targetTime) :
    in_controller(new tgImpedanceController(100, 500, 50)),
    out_controller(new tgImpedanceController(0.01, 500, 10)),
    insideLength(6.5),
    simTime(0.0),
    offsetLength(0.5),
    cpgAmplitude(14.5),
    cpgFrequency(1.0),
    bodyWaves(1.0),
    cycle(0.0),
    target(0.0),
    targetTime(targetTime),
    recordedStartCOM(false),
    move(true)
{
    phaseOffsets.clear();
    phaseOffsets.push_back(0);
    phaseOffsets.push_back(M_PI/4);
    phaseOffsets.push_back(M_PI/2);
    phaseOffsets.push_back(3*M_PI/4);
    phaseOffsets.push_back(M_PI);
}

void DuCTTMechTestController::applyImpedanceControlInside(const std::vector<tgBasicActuator*> stringList, double dt)
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

void DuCTTMechTestController::applyImpedanceControlOutside(const std::vector<tgBasicActuator*> stringList,
                                                            double dt,
                                                            std::size_t phase)
{
    cycle = sin(simTime * cpgFrequency + 2 * bodyWaves * M_PI + phaseOffsets[phase]);
    target = offsetLength + cycle*cpgAmplitude;

    if (target < 0) target *= -1;

    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        double setTension = out_controller->control(*(stringList[i]), dt, target);
//        stringList[i]->setRestLength(target,dt);
//        double setTension = stringList[i]->getMuscle()->getTension();
        #if(0) // Conditional compile for verbose control
        std::cout << "Outside String " << i << ", target: " << target << " com tension " << setTension
        << " act tension " << stringList[i]->getMuscle()->getTension()
        << " length " << stringList[i]->getMuscle()->getActualLength() << std::endl;
        #endif
    }
}

void DuCTTMechTestController::onStep(DuCTTRobotModel& subject, double dt)
{

    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        simTime += dt;

        if (simTime < 5 || !move) return;
        else if (!recordedStartCOM)
        {
            startCOM = subject.getCOM();
            recordedStartCOM = true;
            std::vector<abstractMarker> markers = subject.getMarkers();
            for (size_t i = 0; i<markers.size(); i++)
            {
                fprintf(stderr,"X%lu,Y%lu,Z%lu,",i,i,i);
            }
            fprintf(stderr,"\n");
        }

        btVector3 com = subject.getCOM();
        double dist = startCOM.distance(com);

        //Record values of markers
        std::vector<abstractMarker> markers = subject.getMarkers();
        for (size_t i = 0; i<markers.size(); i++)
        {
            btVector3 pos = markers[i].getWorldPosition();
            fprintf(stderr,"%f,%f,%f,",pos.x(),pos.y(),pos.z());
        }
        fprintf(stderr,"\n");

        //Check for end condition
        if (targetTime < 0 || simTime < targetTime)
        {
            applyImpedanceControlOutside(subject.getAllMuscles(), dt, 1);
        }
        else if (simTime >= targetTime)
        {
            std::cout << "Total Dist moved: " << dist << std::endl;
            move = false;
        }
    }
}
