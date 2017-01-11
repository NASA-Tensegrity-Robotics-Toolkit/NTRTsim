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

#ifndef TG_DATA_MANAGER_H
#define TG_DATA_MANAGER_H

/**
 * @file tgDataManager.h
 * @brief Contains the definition of class tgDataManager.
 * @author Drew Sabelhaus
 * $Id$
 */

// This application
#include "core/tgSenseable.h" //not sure why this needs to be included vs. just declared...
// The C++ Standard Library
#include <string>
#include <sstream>
#include <iostream>
#include <vector>

// Forward declarations
class tgSensor;
class tgSensorInfo;

/**
 * Abstract class for objects that will manage data within NTRTsim.
 * These tgDataManagers are created in an app, and passed in to a tgSimulation.
 * An example of a data manager would be a data logger: it would create various
 * tgSensor objects, and pull from them at each timestep of the simulation.
 */
class tgDataManager
{
public: 

    /**
    * The default constructor. Nothing to do in this base class,
    * but subclasses might do something (e.g. set up a path name for
    * a data logger.)
    */  
    tgDataManager();

    /**
    * Destructor. Deletes the sensors and sensor infos, if 
    * not already deleted by teardown()
    */
    virtual ~tgDataManager();
    
    /**
     * Setup creates the sensors via sensor info classes,
     * and may do other things (ex., open a log file and put in a header.)
     */
    virtual void setup();
    
    /**
     * Deletes all objects inside this (undoes setup)
     */
    virtual void teardown();

    /**
     * Advance the simulation.
     * @param[in] dt the number of seconds since the previous call;
     * std::invalid_argument is thrown if dt is not positive
     * @throw std::invalid_argument if dt is not positive
     */
    virtual void step(double dt);

    /**
     * Add a tgSenseable object to this data manager.
     * These objects will be checked via the sensor infos, and sensors will
     * be assigned to them if appropriate.
     * @param[in] pSenseable a pointer to a tgSenseable object that will be
     * added to this data manager's list of senseable objects.
     */
    virtual void addSenseable(tgSenseable* pSenseable);

    /**
     * Add a sensor info object to the current list of sensor infos.
     * @param[in] pSensorInfo a pointer to a tgSensorInfo.
     */
    virtual void addSensorInfo(tgSensorInfo* pSensorInfo);
	
    /**
     * Returns some basic information about this tgDataManager,
     * like the type of data manager it is, the number of sensor infos
     * and sensors that are held in this object's lists.
     * @return informative string about this data manager
     */
    virtual std::string toString() const;

 private:

    /**
     * A helper function for setup. Since there will be a loop over
     * the sensor infos, this function abstracts it away.
     * @param[in] pSenseable a pointer to one of this object's senseables.
     */
    void addSensorsHelper(tgSenseable* pSenseable);

protected:

    // Integrity predicate.
    bool invariant() const;

    /**
     * A data manager has a list of sensors that it 
     * has created (during setup.)
     */
    std::vector<tgSensor*> m_sensors;

    /**
     * Data managers also have a list of the sensor infos that
     * have been passed in to it.
     * These are used to create the sensors
     */
    std::vector<tgSensorInfo*> m_sensorInfos;

    /**
     * A data manager will also have a list of tgSenseable objects 
     * (really, just tgModels most of the time) that it will collect data from.
     */
    std::vector<tgSenseable*> m_senseables;

};

/**
 * Overload operator<<() to handle tgDataManager
 * @param[in,out] os an ostream
 * @param[in] pair a tgDataManager
 * @return os
 * @todo Inlining this does no good; stream operations are slow.
 */
std::ostream&
operator<<(std::ostream& os, const tgDataManager& obj);

#endif // TG_DATA_MANAGER_H
