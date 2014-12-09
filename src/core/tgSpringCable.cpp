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
 * @file Muscle2P.cpp
 * @brief Definitions of members of classes Muscle2P and MuscleAnchor
 * @todo Split so only one class is defined per file.
 * $Id$
 */

// This module
#include "tgSpringCable.h"
#include "tgSpringCableAnchor.h"

#include <iostream>
#include <stdexcept>

tgSpringCable::tgSpringCable( const std::vector<tgSpringCableAnchor*>& anchors,
				double coefK,
				double dampingCoefficient,
				double pretension) :
m_velocity(0.0),
m_damping(0.0),
m_coefK (coefK),
m_dampingCoefficient(dampingCoefficient)
{
	// Anchors will be stored in child classes
	assert(anchors.size() >= 2);
	assert(coefK > 0.0);
	assert(dampingCoefficient >= 0.0);
	
	std::size_t n = anchors.size();
	
	btVector3 pos1 = anchors[0]->getWorldPosition();
	btVector3 pos2 = anchors[n - 1]->getWorldPosition();
	
	m_restLength = pos1.distance(pos2) - pretension / coefK;
	
	if (m_restLength <= 0.0)
	{
		throw std::invalid_argument("Pretension causes string to shorten past rest length!");
	}
	
	m_prevLength = m_restLength;
}

tgSpringCable::~tgSpringCable()
{
}
