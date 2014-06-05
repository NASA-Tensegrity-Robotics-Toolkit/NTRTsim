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

#ifndef CONTACT_TEST_CONTROLLER_H
#define CONTACT_TEST_CONTROLLER_H

#include "core/tgObserver.h"

//This should probably be a forward declaration
#include "ContactTestModel.h"
#include "dev/btietz/tgRBString.h"
#include "core/ImpedanceControl.h"

class ContactTestController : public tgObserver<ContactTestModel>
{
public:

    ContactTestController() : m_control(0.0, 200000.0, 0.0) 
    {
    }
    
    virtual void onStep(ContactTestModel& subject, double dt)
    {
        
        const std::vector<tgRBString*> muscles =
            subject.getAllMuscles();
            
        for(int i = 0; i < muscles.size(); i++)
        {
            std::cout << "Ctens: " << 
            m_control.control(muscles[i], dt, 10.0) << std::endl;
        }
    }
    
private:
    ImpedanceControl m_control;
};

#endif // MY_MODEL_CONTROLLER_H
