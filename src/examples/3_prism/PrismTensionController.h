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

#ifndef PRISM_TENSION_CONTROLLER_H
#define PRISM_TENSION_CONTROLLER_H

// This library
#include "core/tgObserver.h"

// Forward declarations
class PrismModel;

class PrismTensionController : public tgObserver<PrismModel>
{
public:

    PrismTensionController(double tensionPct = .01);
    
    virtual void onStep(PrismModel& subject, double dt);
    
private:

    const double m_tension;
};

#endif // Prism_TENSION_CONTROLLER_H
