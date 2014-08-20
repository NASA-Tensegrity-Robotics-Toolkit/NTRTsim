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

#ifndef TG_OBSERVER_H
#define TG_OBSERVER_H

/**
 * @file tgObserver.h
 * @brief Definition of tgObserver class
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

/**
 * A mixin class which makes its derived class the Subject in the Obsever
 * design pattern. These are typically controllers.
 */
template <class Subject>
class tgObserver
{
public:

    /** A class with virtual member functions must have a virtual destructor. */
    virtual ~tgObserver() { }
    
    /**
     * Notify the observers when a step action has occurred.
     * @param[in,out] subject the subject being observed
     * @param[in] the number of seconds since the previous call; must be
     * positive
     */
    virtual void onStep(Subject& subject, double dt) = 0;
    
    /**
     * Notify the observers when an attach action has occurred.
     * Will only occur once, typically before setup
     * @param[in,out] subject the subject being observed
     */
    virtual void onAttach(Subject& subject) { }
    
    /**
     * Notify the observers when a setup action has occurred.
     * @param[in,out] subject the subject being observed
     */
    virtual void onSetup(Subject& subject) { }

    /**
     * Notify the observers when a teardown action has occurred.
     * @param[in,out] subject the subject being observed
     */    
    virtual void onTeardown(Subject& subject) { }
    
};
   
#endif
