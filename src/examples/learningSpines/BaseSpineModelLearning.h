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

#ifndef BASE_SPINE_MODEL_LEARNING_H
#define BASE_SPINE_MODEL_LEARNING_H

/**
 * @file BaseSpineModelLearning.h
 * @brief A template base class for a tensegrity spine
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

#include "core/tgModel.h" 
#include "core/tgSubject.h"

#include <map>
#include <set>
#include <string>
#include <vector>

class tgWorld;
class tgStructureInfo;
class tgSpringCableActuator;
class tgBaseRigid;
class btVector3;

/**
 * Provides all of the interfaces for a learning spine model, which
 * implements all of the functions except using  setup
 */
class BaseSpineModelLearning: public tgSubject<BaseSpineModelLearning>,
                                 public tgModel
{
public: 
	
	typedef std::map<std::string, std::vector<tgSpringCableActuator*> > MuscleMap;
	
    virtual ~BaseSpineModelLearning();
    
    virtual void setup(tgWorld& world);
    
    virtual void teardown();
        
    virtual void step(double dt);
    
    virtual std::vector<double> getSegmentCOM(const int n) const;
    
    virtual btVector3 getSegmentCOMVector(const int n) const;
      
    virtual const std::vector<tgSpringCableActuator*>& getMuscles(const std::string& key) const;
    
    virtual const std::vector<tgSpringCableActuator*>& getAllMuscles() const;
    
    virtual const std::vector<tgBaseRigid*> getAllRigids() const;
    
    virtual const int getSegments() const;
    
    virtual std::size_t getNumberofMuslces() const
    {
        return m_allMuscles.size();
    }
    
    double getSpineLength() const;
    
protected:
	
	BaseSpineModelLearning(int segments);
	
    std::vector<tgSpringCableActuator*> m_allMuscles;
	
    std::vector<tgModel*> m_allSegments;

    MuscleMap m_muscleMap;
    
    const std::size_t m_segments;
};

#endif // BASE_SPINE_MODEL_H
