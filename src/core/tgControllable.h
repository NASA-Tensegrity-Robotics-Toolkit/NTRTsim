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

#ifndef TG_CONTROLLABLE_H
#define TG_CONTROLLABLE_H

/**
 * @file tgControllable.h
 * @brief Definition of the tgControllable abstract base class
 * @author Brian Mirletz
 * @date December 2014
 * $Id$
 */


/**
 * An abstract mixin base class for classes that can be affected by
 * controllers. Defines the common control API. Controllers will likely
 * be observers, so they will know the details of their subjects/
 * controllables. That said, it is not worth making something this
 * low level a subject, as there are higher level details not
 * defined in this API (sensor data, etc)
 */
class tgControllable
{
public:

    /** The consructor has nothing to do. */
    tgControllable() { }

    /** The virtual destructor has nothing to do. */
    virtual ~tgControllable() { }
	
	/**
	 * Set the relevant control variable for this class, such as 
	 * commanded position or torque
     * @param[in] input, the  control input, units are application dependent
	 */
	virtual void setControlInput(double input) = 0;
	
	/**
	 * Secondary function for those classes which need to know how much
	 * time has elapsed since they last recieved input (such as 
	 * tgBasicActuator's moveMotors(dt) function)
	 * This will silently fail if it is called erroneously
     * @param[in] input, the desired control input
     * @param[in] dt, the time elapsed since the last call. Must be positive
     * (typically enforced in child class)
	 */
	virtual void setControlInput(double input, double dt) { }
	
};

#endif  // TG_CONTROLLABLE_H

