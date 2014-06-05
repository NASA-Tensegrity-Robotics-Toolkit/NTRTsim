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

#ifndef FLEMONS_SPINE_MODEL_H
#define FLEMONS_SPINE_MODEL_H

#include "core/tgModel.h" 

#include "core/tgSubject.h"

#include <set>
#include <map>

class tgLinearString;
class tgWorld;
class CPGEquations;

class FlemonsSpineModel: public tgSubject<FlemonsSpineModel>, public tgModel
{
public: 

    FlemonsSpineModel(int segments);

    virtual ~FlemonsSpineModel()
    {}
    
    virtual void setup(tgWorld& world);
    
    virtual void step(double dt);
    
    const std::vector<double> getSegmentCOM(const int n) const;

    void changeMuscle (double length, double dt);
    
    const std::vector<tgLinearString*> getMuscles (std::string key)
    {
        return muscleMap[key];
    }
    
    const int getSegments() const
    {
        return m_segments;
    }
    
private:
    std::vector<tgLinearString*> allMuscles;
    std::vector<tgModel*> allSegments;
    std::map<std::string, std::vector<tgLinearString*> > muscleMap;

    const int m_segments;
};

#endif // FLEMONS_SPINE_MODEL_H
