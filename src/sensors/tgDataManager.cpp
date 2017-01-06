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
 * @file tgDataManager.cpp
 * @brief Contains the implementations of parts of the abstract class tgDataManager
 * @author Drew Sabelhaus
 * $Id$
 */

// This module
#include "tgDataManager.h"
// This application
#include "tgSensor.h"
#include "core/tgSenseable.h"
//#include "tgSensorInfo.h"
// The C++ Standard Library
//#include <stdio.h> // for sprintf
#include <iostream>
#include <stdexcept>
#include <cassert>

/**
 * Nothing to do, in this abstract base class.
 */
tgDataManager::tgDataManager()
{
  //DEBUGGING
  //std::cout << "tgDataManager constructor." << std::endl;
  // Postcondition
  assert(invariant());
}

/**
 * In the destructor, we must be sure to delete everything in both the 
 * m_sensors and m_sensorInfos lists.
 * Ideally, this should be handled by teardown, so this is really a
 * double-check for memory leaks more than anything else.
 */
tgDataManager::~tgDataManager()
{
  //DEBUGGING
  //std::cout << "tgDataManager destructor." << std::endl;
  
  // First, delete everything in m_sensors.
  const size_t n_Sens = m_sensors.size();
  for (size_t i = 0; i < n_Sens; ++i)
  {
    // Pick out one sensor from the list:
    tgSensor* const pSensor = m_sensors[i];
    // TO-DO: check if we need this assert...
    // It is safe to delete NULL, but this is an invariant
    // assert(pChild != NULL); 
    delete pSensor;
  }
  // Next, delete the sensor infos.
  // TO-DO: fill in.

  // Note that the creation and deletion of the senseable objects, e.g.
  // the tgModels, is handled externally.
  // tgDataManagers should NOT destroy the objects they are sensing.
}

/**
 * In child classes, setup will probably do something interesting.
 * In the base class, nothing is being created.
 */
void tgDataManager::setup()
{
  //DEBUGGING
  //std::cout << "tgDataManager setup." << std::endl;
  // TO-DO: Should we create the sensors here in the setup method of the base?
  // What's the best way to maximize code re-use?
  // Postcondition
  assert(invariant());
}

/**
 * The teardown method has to remove all the sensors and sensor infos.
 * Note that this method could be redefined by subclasses, for example to 
 * do some final step with a mesage-passing algorithm before the simulation ends.
 * TO-DO: what's a good way to capture the deletion of sensors and sensorInfos
 * that doesn't require subclasses to copy-and-paste from this teardown method?
 */
void tgDataManager::teardown()
{
  //DEBUGGING
  //std::cout << "tgDataManager teardown." << std::endl;
  
  // First, delete the sensors.
  // Note that it's good practice to set deleted pointers to NULL here.
  for (std::size_t i = 0; i < m_sensors.size(); i++)
  {
    // Note that sensors don't have any teardown method.
    // The delete method will call their destructors.
    delete m_sensors[i];
  }
  // Clear the list so that the destructor for this class doesn't have to
  // do anything.
  m_sensors.clear();

  // Next, delete the sensor infos.
  // TO-DO: implement this.

  // Clear the list of senseable objects.
  // Since tgDataManagers don't ever change these objects,
  // leave it to other classes to create and delete them.
  // Just get rid of the pointers here.
  // TO-DO: maybe we don't want to do this? Will this make it so no logs
  // are created upon reset???
  // TO-DO: could this segfault? Will the pointers in m_senseables be changed
  // during reset at all?
  //m_senseables.clear();

  // Postcondition
  assert(invariant());
  assert(m_sensors.empty());
}

/**
 * The step method is where data is actually collected!
 * This base class won't do anything, but subclasses will re-implement this method.
 */
void tgDataManager::step(double dt) 
{
  //DEBUGGING
  //std::cout << "tgDataManager step." << std::endl;
  if (dt <= 0.0)
  {
    throw std::invalid_argument("dt is not positive");
  }
  else
  {
    // Nothing to do.
  }

  // Postcondition
  assert(invariant());
}

/**
 * This method adds sensor info objects to this data manager.
 * It takes in a pointer to a sensor info and pushes it to the
 * current list of sensor infos.
 */
/*
void tgDataManager::addSensorInfo(tgSensorInfo* pSensorInfo)
{
  // Precondition
  if (pSensorInfo == NULL)
  {
    throw std::invalid_argument("pSensorInfo is NULL inside tgDataManager::addSensorInfo");
  } 

  m_sensorInfos.push_back(pSensorInfo);

  // Postcondition
  assert(invariant());
  assert(!m_sensorInfos.empty());
}
*/

/**
 * This method adds sense-able objects to this data manager.
 * It takes in a pointer to a sense-able object and pushes it to the
 * current list of tgSenseables.
 */
void tgDataManager::addSenseable(tgSenseable* pSenseable)
{
  //DEBUGGING
  std::cout << "tgDataManager addSenseable." << std::endl;
  // Precondition
  if (pSenseable == NULL)
  {
    throw std::invalid_argument("pSenseable is NULL inside tgDataManager::addSenseable");
  } 

  m_senseables.push_back(pSenseable);

  // Postcondition
  assert(invariant());
  assert(!m_senseables.empty());
}


/**
 * The toString method for data managers should include a list of the number
 * of sensors and sensor Infos it has.
 */
std::string tgDataManager::toString() const
{
  std::string p = "  ";  
  std::ostringstream os;
  // Note that we're using sprintf here to convert an int to a string.
  // TO-DO: fix this!
  os << "tgDataManager" << std::endl;
    //   << " with " << sprintf("%d",m_sensors.size()) << " sensors."<< std::endl;

  /*
    os << prefix << p << "Children:" << std::endl;
  for(std::size_t i = 0; i < m_children.size(); i++) {
    os << m_children[i]->toString(prefix + p) << std::endl;
  }
  os << prefix << p << "Tags: [" << getTags() << "]" << std::endl;
  os << prefix << ")";
  */
  return os.str();
}


bool tgDataManager::invariant() const
{
  // TO-DO:
  // m_sensors and m_sensorInfos are sane, check somehow...?
  return true;
}

std::ostream&
operator<<(std::ostream& os, const tgDataManager& obj)
{
    os << obj.toString() << std::endl;
    return os;
}
