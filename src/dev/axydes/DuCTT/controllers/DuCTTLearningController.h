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

#ifndef TEST_CONTROLLER_H
#define TEST_CONTROLLER_H

/**
 * @file DuCTTSineWaves.h
 * @brief Contains the definition of the class DuCTTSineWaves.
 * @author Alexander Xydes
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class tgPrismatic;
class tgTouchSensorSphereModel;
class DuCTTRobotModel;

class DuCTTSineWaves : public tgObserver<DuCTTRobotModel>
{
public:

    DuCTTSineWaves();
    
    virtual void onStep(DuCTTRobotModel& subject, double dt);
    
    void applySineWave(tgPrismatic* prism, bool shouldPause, bool shouldUnPause, double dt);
    bool shouldPause(std::vector<tgTouchSensorSphereModel*> touchSensors);

private:
    double simTime;
    double cycle;
    double target;

    std::vector<double> phaseOffsets;
    const double offsetSpeed;
    const double cpgAmplitude;
    const double cpgFrequency;
    const double bodyWaves;
};

#endif // PRETENSION_CONTROLLER_H
