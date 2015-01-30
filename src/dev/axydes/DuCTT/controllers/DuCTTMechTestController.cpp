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

DuCTTMechTestController::DuCTTMechTestController(double targetTime, int _testCase) :
    impController(new tgImpedanceController(0.01, 5000, 10)),
    simTime(0.0),
    startTime(5.0),
    offsetLength(3.0),
    cpgAmplitude(5.0),
    cpgFrequency(40.0),
    bodyWaves(1.0),
    cycle(0.0),
    targetTime(targetTime),
    recordedStartCOM(false),
    move(true),
    testCase(_testCase),
    startedFile(false)
{
    phaseOffset.clear();
    phaseOffset.push_back(0);
    phaseOffset.push_back(M_PI/4);
    phaseOffset.push_back(M_PI/2);
    phaseOffset.push_back(3*M_PI/4);

    switch(testCase)
    {
        case 2:
            offsetLength = 3.0;
            cpgAmplitude = 30.0;
            cpgFrequency = 20.0;
            bodyWaves = 1.0;
            break;
        case 1:
            offsetLength = 3.0;
            cpgAmplitude = 15.0;
            cpgFrequency = 20.0;
            bodyWaves = 1.0;
            break;
        case 0:
        default:
            offsetLength = 3.0;
            cpgAmplitude = 5.0;
            cpgFrequency = 40.0;
            bodyWaves = 1.0;
            break;
    }
}

double DuCTTMechTestController::getCPGTarget(int phase)
{
    cycle = sin((simTime-startTime) * cpgFrequency + 2 * bodyWaves * M_PI + phaseOffset[phase]);
    double target = offsetLength + cycle*cpgAmplitude;
    if (target < offsetLength) target = offsetLength;
    return target;
}

void DuCTTMechTestController::applyImpedanceControl(tgBasicActuator* string, double dt, int phase)
{
    double target = getCPGTarget(phase);
    string->setControlInput(target, dt);
}

void DuCTTMechTestController::applyImpedanceControl(const std::vector<tgBasicActuator*> stringList,
                                                            double dt, int phase)
{
    double target = getCPGTarget(phase);

    for(std::size_t i = 0; i < stringList.size(); i++)
    {
        stringList[i]->setControlInput(target, dt);
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

        if (!startedFile)
        {
            std::vector<abstractMarker> markers = subject.getMarkers();
            for (size_t i = 0; i<markers.size(); i++)
            {
                fprintf(stderr,"X%lu,Y%lu,Z%lu,",i,i,i);
            }
            std::vector<tgBasicActuator*> muscles = subject.getAllMuscles();
            for (size_t i = 0; i<muscles.size(); i++)
            {
                fprintf(stderr,"Rest Length %lu,",i);
            }
            fprintf(stderr,"\n");
            startedFile = true;
        }
        //Record values of markers
        std::vector<abstractMarker> markers = subject.getMarkers();
        for (size_t i = 0; i<markers.size(); i++)
        {
            btVector3 pos = markers[i].getWorldPosition();
            fprintf(stderr,"%f,%f,%f,",pos.x(),pos.y(),pos.z());
        }

        //Record rest lengths of strings
        std::vector<tgBasicActuator*> vertMuscles = subject.getVertMuscles();
        for (size_t i = 0; i<vertMuscles.size(); i++)
        {
            fprintf(stderr,"%f,",vertMuscles[i]->getRestLength());
        }
        std::vector<tgBasicActuator*> saddleMuscles = subject.getSaddleMuscles();
        for (size_t i = 0; i<vertMuscles.size(); i++)
        {
            fprintf(stderr,"%f,",saddleMuscles[i]->getRestLength());
        }

        fprintf(stderr,"\n");

        if (simTime < startTime || !move) return;
        else if (!recordedStartCOM)
        {
            startCOM = subject.getCOM();
            recordedStartCOM = true;
        }

        btVector3 com = subject.getCOM();
        double dist = startCOM.distance(com);


        //Check for end condition
        if (targetTime < 0 || simTime < targetTime)
        {
            //Record values of markers
            std::vector<abstractMarker> markers = subject.getMarkers();
            for (size_t i = 0; i<markers.size(); i++)
            {
                btVector3 pos = markers[i].getWorldPosition();
                fprintf(stderr,"%f,%f,%f,",pos.x(),pos.y(),pos.z());
            }

            //Record rest lengths of strings
            std::vector<tgBasicActuator*> vertMuscles = subject.getVertMuscles();
            for (size_t i = 0; i<vertMuscles.size(); i++)
            {
                fprintf(stderr,"%f,",vertMuscles[i]->getRestLength());
            }
            std::vector<tgBasicActuator*> saddleMuscles = subject.getSaddleMuscles();
            for (size_t i = 0; i<vertMuscles.size(); i++)
            {
                fprintf(stderr,"%f,",saddleMuscles[i]->getRestLength());
            }

            fprintf(stderr,"\n");

            switch(testCase)
            {
            case 2:
                applyImpedanceControl(subject.getVertMuscles()[2], dt, 0);
                applyImpedanceControl(subject.getVertMuscles()[3], dt, 2);
                break;
            case 1:
                applyImpedanceControl(subject.getVertMuscles()[0], dt, 0);
                applyImpedanceControl(subject.getVertMuscles()[1], dt, 2);
                break;
            case 0:
            default:
                applyImpedanceControl(subject.getAllMuscles(), dt, 1);
                break;
            }
        }
        else if (simTime >= targetTime)
        {
            std::cout << "Total Dist moved: " << dist << std::endl;
            move = false;
        }
    }
}
