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

#ifndef DUCTT_SIMPLE_H
#define DUCTT_SIMPLE_H

/**
 * @file DuCTTSimple.h
 * @brief Contains the definition of the class DuCTTSimple.
 * @author Alexander Xydes
 * $Id$
 */

// This library
#include "core/tgObserver.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class tgBasicActuator;
class tgPrismatic;
class tgTouchSensorSphereModel;
class DuCTTRobotModel;
class tgImpedanceController;

class DuCTTSimple : public tgObserver<DuCTTRobotModel>
{
public:
    enum ROBOT_STATE{
        EXPAND_BOTTOM,
        RETRACT_TOP,
        PUSH_TOP,
        EXPAND_TOP,
        RETRACT_BOTTOM,
        PULL_BOTTOM
    };

    DuCTTSimple(double targetDist = -1);
    
    virtual void onStep(DuCTTRobotModel& subject, double dt);
    virtual void onSetup(DuCTTRobotModel& subject);
    virtual void onTeardown(DuCTTRobotModel& subject);

    void applySineWave(tgPrismatic* prism, bool shouldPause, bool shouldUnPause, double dt, int phase=0);
    bool shouldPause(std::vector<tgTouchSensorSphereModel*> touchSensors);
    bool checkPause(std::vector<tgTouchSensorSphereModel*> touchSensors, bool top);

    /**
     * Applies the impedance controllers using a velocity setpoint of 0.
     * Called during this classes onStep function.
     * @param[in] stringList a std::vector of strings taken from the
     * subject's MuscleMap
     * @param[in] dt - a timestep. Must be positive.
     */
    void applyImpedanceControlInside(const std::vector<tgBasicActuator*> stringList,
                                    double dt);
    /**
     * Applies the impedance controllers using a velocity setpoint determined.
     * by the phase parameter and
     * Called during this classes onStep function.
     * @param[in] stringList a std::vector of strings taken from the
     * subject's MuscleMap
     * @param[in] dt - a timestep. Must be positive.
     * @param[in] phase - reads the index out of the phaseOffsets vector
     */
    void applyImpedanceControlOutside(const std::vector<tgBasicActuator*> stringList,
                                    double dt,
                                    std::size_t phase);

    bool movePrism(tgPrismatic* prism, std::vector<tgTouchSensorSphereModel*> sensors, double goal, double dt);
    bool moveStrings(const std::vector<tgBasicActuator*> stringList, double goals, double dt);

private:
    tgImpedanceController* in_controller;
    tgImpedanceController* out_controller;

    double simTime;
    double cycle;
    double target;

    std::vector<double> phaseOffsets;
    const double offsetLength;
    const double cpgAmplitude;
    const double cpgFrequency;
    const double bodyWaves;

    double cyclePrism;
    double targetPrism;
    const double offsetLengthPrism;
    const double cpgAmplitudePrism;
    const double cpgFrequencyPrism;
    const double bodyWavesPrism;

    bool recordedStartCOM;
    btVector3 startCOM;

    const double insideLength;

    double targetDist;
    bool move;
    bool shouldBotPause;
    bool shouldTopPause;

    ROBOT_STATE state;
};

#endif // PRETENSION_CONTROLLER_H
