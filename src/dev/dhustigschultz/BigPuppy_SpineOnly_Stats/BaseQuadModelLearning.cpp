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
 * @file BaseQuadModelLearning.cpp
 * @brief A template base class for a tensegrity quadruped
 * @author Brian Tietz, Dawn Hustig-Schultz
 * @version 1.0.0
 * @date Nov. 2015
 * $Id$
 */

// This module
#include "BaseQuadModelLearning.h"
// This library
#include "core/tgCast.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgBaseRigid.h"
#include "core/tgRod.h"
#include "core/tgString.h"
// The C++ Standard Library
#include <algorithm> // std::fill
#include <iostream>
#include <stdexcept>

BaseQuadModelLearning::BaseQuadModelLearning(int segments, int hips) : 
    m_segments(segments),  
    m_hips(hips),
    tgModel() 
{
    m_subStructures = m_hips + m_segments;
}

BaseQuadModelLearning::~BaseQuadModelLearning()
{
}

void BaseQuadModelLearning::setup(tgWorld& world)
{
	
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void BaseQuadModelLearning::teardown()
{
    notifyTeardown();
    
    tgModel::teardown();
      
    // These pointers will be freed by tgModel::teardown
    m_allMuscles.clear();
    m_allSegments.clear();
    m_muscleMap.clear();
}

void BaseQuadModelLearning::step(double dt)
{
    /* CPG update occurs in the controller so that we can decouple it
    * from the physics update
    */
    notifyStep(dt);
    
    tgModel::step(dt);  // Step any children
}

const std::vector<tgSpringCableActuator*>&
BaseQuadModelLearning::getMuscles (const std::string& key) const
{
    const MuscleMap::const_iterator it = m_muscleMap.find(key);
    if (it == m_muscleMap.end())
    {
        throw std::invalid_argument("Key '" + key + "' not found in muscle map");
    }
    else
    {
        return it->second;
    }
}

const std::vector<tgSpringCableActuator*>& BaseQuadModelLearning::getAllMuscles() const
{
    return m_allMuscles;
}

const std::vector<tgBaseRigid*> BaseQuadModelLearning::getAllRigids() const
{
	if (m_allSegments.size() != m_subStructures)
    {
        throw std::runtime_error("Not initialized");
    }
	
	std::vector<tgBaseRigid*> p_rods;
	
	for (std::size_t i = 0; i < m_allSegments.size(); i++)
	{
		std::vector<tgBaseRigid*> temp = tgCast::filter<tgModel, tgBaseRigid> (m_allSegments[i]->getDescendants());
        p_rods.insert(p_rods.end(), temp.begin(), temp.end());
	}
	
	return p_rods;
}

const int BaseQuadModelLearning::getSegments() const
{
    return m_segments;
}

std::vector<double> BaseQuadModelLearning::getCOM(const int n)
{
    if (m_allSegments.size() != m_subStructures)
    {
        throw std::runtime_error("Not initialized");
    }
    else if (n < 0) 
    {
        throw std::range_error("Negative segment number"); 
    }
    else if (n >= m_subStructures)
    {
        throw std::range_error(tgString("Segment number > ", m_segments));
    }

    std::vector<double> centerOfMass(3);
    for (size_t i = 0; i < 3; ++i) { centerOfMass[i] = 0; }
    double mass = 0.0;
    for (std::size_t i = n; i < m_subStructures; i++)
    {  
        std::vector<double> segmentCOM = getSegmentCOM(i);
	for (std::size_t j = 0; j < 3; j++)
	{
	    segmentCOM[j] *= segmentMasses[i];
	}
	for (std::size_t j = 0; j < 3; j++)
	{
	    centerOfMass[j] += segmentCOM[j];
	}
	mass += segmentMasses[i];
    }
    
    //assert(n == segmentMasses.size());

    for (std::size_t j = 0; j < 3; j++)
    {
	centerOfMass[j] /= mass;
    }
    
    return centerOfMass;
}
    
std::vector<double> BaseQuadModelLearning::getSegmentCOM(const int n)
{
    
    btVector3 segmentCenterOfMass = getSegmentCOMVector(n);
    
    // Copy to the result std::vector
    std::vector<double> result(3);
    for (size_t i = 0; i < 3; ++i) { result[i] = segmentCenterOfMass[i]; }
    
    return result;
}

btVector3 BaseQuadModelLearning::getSegmentCOMVector(const int n)
{
    if (m_allSegments.size() != m_subStructures)
    {
        throw std::runtime_error("Not initialized");
    }
    else if (n < 0) 
    {
        throw std::range_error("Negative segment number"); 
    }
    else if (n >= m_subStructures)
    {
        throw std::range_error(tgString("Segment number > ", m_segments));
    }
    
    std::vector<tgRod*> p_rods =
        tgCast::filter<tgModel, tgRod> (m_allSegments[n]->getDescendants());
    
    // Ensure our segments are being populated correctly
    assert(!p_rods.empty());

    btVector3 segmentCenterOfMass(0, 0, 0);
    double segmentMass = 0.0;
    for (std::size_t i = 0; i < p_rods.size(); i++)
    {
        const tgRod* const pRod = p_rods[i];
        assert(pRod != NULL);
        const double rodMass = pRod->mass();
        //std::cout << "mass " << rodMass;
        const btVector3 rodCenterOfMass = pRod->centerOfMass();
        segmentCenterOfMass += rodCenterOfMass * rodMass;
        segmentMass += rodMass;
    }
    
    // Check to make sure the rods actually had mass
    assert(segmentMass > 0.0);

    segmentMasses.push_back(segmentMass);
    
    segmentCenterOfMass /= segmentMass;

    return segmentCenterOfMass;
}

double BaseQuadModelLearning::getSpineLength()
{
    const btVector3 start = getSegmentCOMVector(0);
    const btVector3 end = getSegmentCOMVector(m_segments - 1);
    
    return (start - end).length();
}

