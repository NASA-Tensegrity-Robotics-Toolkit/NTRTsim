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

#ifndef PRETENSION_CONTROLLER_H
#define PRETENSION_CONTROLLER_H

/**
 * @file PretensionController.h
 * @brief Contains the definition of the class PretensonController.
 * @author Brian Tietz
 * $Id$
 */

// This library
#include "core/tgObserver.h"

// Forward declarations
class tgLinearString;

class PretensionController : public tgObserver<tgLinearString>
{
public:

    PretensionController(double tensionPct = .01);
    
    virtual void onStep(tgLinearString& subject, double dt);
    
private:

    const double m_tension;
};

#endif // PRETENSION_CONTROLLER_H
