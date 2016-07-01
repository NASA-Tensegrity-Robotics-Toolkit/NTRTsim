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
 * @file utility.cpp
 * @brief Definitions of some utility functions.
 * @author Edward Zhu
 * @version 1.0.0
 * $Id$
 */

#ifndef _UTILITY_HPP
#define _UTILITY_HPP

#include <iostream>
#include <vector>

namespace utility {

/**
 * A function for printing out vectors of fundamental data types
 * @param[in] vect - Vector to be printed
 */
template <typename T> void printVector(std::vector<T> const& vect)
{
	std::cout << "[" << *(vect.begin());
	for (typename std::vector<T>::const_iterator it = vect.begin()+1; it != vect.end(); ++it) {
		std::cout << ", " << *it;
	}
	std::cout << "]" << std::endl;
}

}
#endif