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

#ifndef TG_CORDE_MODEL
#define TG_CORDE_MODEL

#include "core/tgModel.h"

// Forward Declaration
class CordeModel;

class tgCordeModel : public tgModel
{
public:
    tgCordeModel();
    
    ~tgCordeModel();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
    
    virtual void step(double dt);
    
    /**
    * Call tgModelVisitor::render() on self and all descendants.
    * @param[in,out] r a reference to a tgModelVisitor
    */
    virtual void onVisit(const tgModelVisitor& r) const;
    
    /** @todo consider adding a toString method **/
    
    const CordeModel* const getModel() const
    {
        return testString;
    }
    
private:
    CordeModel* testString;
    
    std::vector<btVector3> generatePoints(btVector3& point1, btVector3& point2, std::size_t resolution);
};

#endif //TG_CORDE_MODEL
