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

#ifndef TG_SUBJECT_H
#define TG_SUBJECT_H

/**
 * @file tgSubject.h
 * @brief Definition of tgSubject class
 * @author Ryan Adams
 * @date March 2014
 * $Id$
 */

// This application
#include "tgObserver.h"
// The C++ standard library
#include <vector>

/**
 * A mixin base class for the subject in the observer design pattern.
 * Observers are attached to the subject, and their onStep() member functions
 * are called when notification is required. Used primarily for control
 * of tensegrity structures, a structure that needs to be controlled
 * will be a child of this class. This can either be the main model
 * or submodels such as a tgLinearString
 */
template <typename T>
class tgSubject
{
public:

    /** The consructor has nothing to do. */
    tgSubject() { }

    /** The virtual destructor has nothing to do. */
    virtual ~tgSubject() { }

    /**
     * Attach an observer to the subject of the observer.
     * @param[in,out] pObserver a pointer to an observer for the subject;
     * do nothing if the pointer is NULL
     */
    void attach(tgObserver<T>* pObserver);
    
    /**
     * Call tgObserver<T>::onStep() on all observers in the order in which they
     * were attached.
     * @param[in] dt the number of seconds since the previous call; do nothing
     * if not positive
     */
    void notifyStep(double dt);
    
    /**
     * Call tgObserver<T>::onSetup() on all observers in the order in which they
     * were attached.
     */
    void notifySetup();

    /**
     * Call tgObserver<T>::onTeardown() on all observers in the order in which they
     * were attached.
     */
    void notifyTeardown();
    
private:

    /**
     * A sequence of observers called in the order in which they were attached.
     * The subject does not own the observers and must not deallocate them.
     */
     std::vector<tgObserver<T> * > m_observers;
};

template <typename Subject>
void tgSubject<Subject>::attach(tgObserver<Subject>* pObserver)
{
    if (pObserver) { m_observers.push_back(pObserver); 
        pObserver->onAttach(static_cast<Subject&>(*this));}
}

template <typename Subject>
void tgSubject<Subject>::notifyStep(double dt)
{
    if (dt > 0)
    {
        const std::size_t n = m_observers.size();
    for (std::size_t i = 0; i < n; ++i) 
    {
        tgObserver<Subject>* const pObserver = m_observers[i];
        if (pObserver) { pObserver->onStep(static_cast<Subject&>(*this), dt); }
    }
    }
}

template <typename Subject>
void tgSubject<Subject>::notifySetup()
{
        const std::size_t n = m_observers.size();
    for (std::size_t i = 0; i < n; ++i) 
    {
        tgObserver<Subject>* const pObserver = m_observers[i];
        if (pObserver) { pObserver->onSetup(static_cast<Subject&>(*this)); }
    }
}

template <typename Subject> 
void tgSubject<Subject>::notifyTeardown()
{
        const std::size_t n = m_observers.size();
    for (std::size_t i = 0; i < n; ++i) 
    {
        tgObserver<Subject>* const pObserver = m_observers[i];
        if (pObserver) { pObserver->onTeardown(static_cast<Subject&>(*this)); }
    }
}
#endif  // TG_SUBJECT_H

