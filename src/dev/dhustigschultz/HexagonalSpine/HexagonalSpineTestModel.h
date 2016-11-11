
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

#ifndef HEXAGONAL_SPINE_TEST_MODEL_H
#define HEXAGONAL_SPINE_TEST_MODEL_H

/**
 * @file HexagonalSpineTestModel.h
 * @brief Contains the definition of class HexagonalSpineTestModel
 * @author Lauren Sharo
 * $Id$
 */

// This library
#include "core/tgModel.h" 
#include "core/tgSubject.h"
// The C++ Standard Library
#include <map>
#include <set>
#include <string>
#include <vector>

// Forward declarations
class tgBasicActuator;
class tgStructure;

/**
 * The HexagonalSpineTestModel implements a tensegrity with.....
 */
class HexagonalSpineTestModel: public tgSubject<HexagonalSpineTestModel>, public tgModel
{
public: 
	
	/**
	 * Used within this function to map segments to string keys
	 */
    typedef std::map<std::string, std::vector<tgBasicActuator*> > ActuatorMap;
	
	/**
	 * The only constructor. The model details are instantiated once
	 * setup is called typically when the model is added to a simulation.
	 * @param[in] segmentsLeft, a positive integer dictating how many
	 * rigid segments will be constructed on the left side.
	 * @param[in] segmentsRight, a positive integer dictating how many
	 * rigid segments will be constructed on the right side.
	 */
    HexagonalSpineTestModel(size_t segmentsLeft, size_t segmentsRight);
	
	/**
	 * Nothing to do. Most functions already handled by tgModel::teardown
	 */
    virtual ~HexagonalSpineTestModel()
    {}
    
    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);
	
	/**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(const double dt);
	
	/**
	 * Get a group of actuators according to the provided key. Groups are
	 * populated during setup.
	 * @param[in] key - a std:string* used as a key to a std::map from
	 * a string to a vector of actuators
	 * @return a std::vector of pointers to the actuators found by the key
	 */
    const std::vector<tgBasicActuator*>& getActuators (const std::string& key) const;
    
    /**
     * Return a std::size_t indicating the number of  left-hand segments in the
     * tetraSpine.
     * @return a std::size_t with the value of m_segmentsLeft
     */
    size_t getLeftSegments() const
    {
    	return m_segmentsLeft;
    }

    /**
     * Return a std::size_t indicating the number of  right-hand segments in the
     * tetraSpine.
     * @return a std::size_t with the value of m_segmentsRight
     */
    size_t getRightSegments() const
    {
    	return m_segmentsRight;
    }

    /**
     * Return a double indicating the total mass of the structure.
     */
    double getMass();

private:
	
	/**
	 * A std::vector containing all of the tgBasicActuators amongst
	 * the children of this model. Populated during setup
	 */
    std::vector<tgBasicActuator*> allActuators;
	
	/**
	 * A typdef of std::map from std::string to tgBasicActuator*. Contains
	 * mapped actuators, populated during setup.
	 */
    ActuatorMap actuatorMap;
    
    /**
     * The number of left-hand segments in the spine
     */
    const size_t m_segmentsLeft;

    /**
     * The number of right-hand segments in the spine
     */
    const size_t m_segmentsRight;
};

#endif
