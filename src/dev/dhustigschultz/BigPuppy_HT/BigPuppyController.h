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

#ifndef BIGPUPPY_CONTROLLER_H
#define BIGPUPPY_CONTROLLER_H

/**
 * @file BigPuppyController.h
 * @brief Implementing a hand-tuned controller for a quadruped based roughly on the Flemons BigPuppy model.
 * @author Dawn Hustig-Schultz
 * @date July 2015
 * @version 1.0.0
 * $Id$
 */

//this library
#include "core/tgObserver.h"

#include "sensors/tgDataObserver.h"

//Forward Declarations
class BigPuppy;

class BigPuppyController : public tgObserver<BigPuppy>
{
public:

    //I really don't think I need a preferred length here, since there will be many different lengths, decided in onSetup()
    BigPuppyController(double timestep);

    ~BigPuppyController() {}

    virtual void onSetup(BigPuppy& subject);
    
    virtual void onStep(BigPuppy& subject, double dt);

    virtual void onTeardown(BigPuppy& subject);

private:
      
    double m_totalTime;
    double dt;

    //May need functions for setting target lengths of different muscle groups...
    virtual void moveAllMotors(BigPuppy& subject, double dt);

    //For setting muscle lengths:
    void setBicepTargetLength(BigPuppy& subject, double dt);
    void setFrontTricepTargetLength(BigPuppy& subject, double dt);
    void setRearTricepTargetLength(BigPuppy& subject, double dt);
    void setLegToAbdomenTargetLength(BigPuppy& subject, double dt);
    void setRightShoulderTargetLength(BigPuppy& subject, double dt);
 
};
#endif //BIGPUPPY_CONTROLLER_H
