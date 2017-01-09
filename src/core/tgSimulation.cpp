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
 * @file tgSimulation.cpp
 * @brief Contains the definitions of members of class tgSimulation
 * @author Ryan Adams, Brian Mirletz, Drew Sabelhaus
 * $Id$
 */

// This module
#include "tgSimulation.h"
// This application
#include "tgModel.h"
#include "tgSimView.h"
#include "tgSimViewGraphics.h"
#include "tgWorld.h"
#include "sensors/tgDataManager.h" //for loggers etc.
// The Bullet Physics Library
#include "LinearMath/btQuickprof.h"

// The C++ Standard Library
#include <stdexcept>

tgSimulation::tgSimulation(tgSimView& view) :
  m_view(view)
{
        m_view.bindToSimulation(*this);

    m_view.setup();

    // Postcondition
    assert(invariant());
}

tgSimulation::~tgSimulation()
{
    teardown();
    m_view.releaseFromSimulation();
    for (std::size_t i = 0; i < m_models.size(); i++)
    {
        delete m_models[i];
    }
    // Delete the tgDataManagers here too.
    for (std::size_t i=0; i < m_dataManagers.size(); i++) {
      delete m_dataManagers[i];
    }
}

void tgSimulation::addModel(tgModel* pModel)
{
    // Precondition
    if (pModel == NULL)
    {
        throw std::invalid_argument("NULL pointer to tgModel");
    }
    else
    {

        pModel->setup(m_view.world());
        m_models.push_back(pModel);
    }

    // Postcondition
    assert(invariant());
    assert(!m_models.empty());
}

void tgSimulation::addObstacle(tgModel* pObstacle)
{
    // Precondition
    if (pObstacle == NULL)
    {
        throw std::invalid_argument("NULL pointer to tgModel");
    }
    else
    {

        pObstacle->setup(m_view.world());
        m_obstacles.push_back(pObstacle);
    }

    // Postcondition
    assert(invariant());
    assert(!m_obstacles.empty());
}

// Similar to models and obstacles, add a data manager.
void tgSimulation::addDataManager(tgDataManager* pDataManager)
{
  // Precondition
  if( pDataManager == NULL){
    throw std::invalid_argument("NULL pointer to data manager, in tgSimulation.");
  }
  else {
    // TO-DO: do data managers need knowledge of the world?
    //pDataManager->setup(m_view.world());
    pDataManager->setup();
    m_dataManagers.push_back(pDataManager);
  }
  // Postcondition
  assert(invariant());
  assert(!m_dataManagers.empty());
}

void tgSimulation::onVisit(const tgModelVisitor& r) const
{
#ifndef BT_NO_PROFILE 
    BT_PROFILE("tgSimulation::onVisit");
#endif //BT_NO_PROFILE	
        // Removed sending the visitor to the world since it wasn't used
        // Write a worldVisitor if its necessary
        for (std::size_t i = 0; i < m_models.size(); i++) {
            m_models[i]->onVisit(r);
        }
        for (std::size_t i = 0; i < m_obstacles.size(); i++) {
            m_obstacles[i]->onVisit(r);
        }
}

void tgSimulation::reset()
{

    teardown();

    m_view.setup();
    for (std::size_t i = 0; i != m_models.size(); i++)
    {
        
        m_models[i]->setup(m_view.world());
    }
    // Also, need to set up the data managers again.
    // Note that this MUST occur after calling setup on the models,
    // otherwise the data manager will not create any sensors
    // (since there are no tgRods, etc., inside the tgModel yet!)
    for (std::size_t i = 0; i < m_dataManagers.size(); i++) {
      // As in addDataManager: do the data managers need knowledge of the world?
      m_dataManagers[i]->setup();
    }
    
    // Don't need to set up obstacles since they will be added after this
}

void tgSimulation::reset(tgGround* newGround)
{

    teardown();
    
    // This will reset the world twice (once in teardown, once here), but that shouldn't hurt anything
    m_view.world().reset(newGround);
    
    m_view.setup();
    for (std::size_t i = 0; i != m_models.size(); i++)
    {
        
        m_models[i]->setup(m_view.world());
    }
    // Also, need to set up the data managers again.
    // Note that this MUST occur after calling setup on the models,
    // otherwise the data manager will not create any sensors
    // (since there are no tgRods, etc., inside the tgModel yet!)
    for (std::size_t i = 0; i < m_dataManagers.size(); i++) {
      // As in addDataManager: do the data managers need knowledge of the world?
      m_dataManagers[i]->setup();
    }
    
    // Don't need to set up obstacles since they were just added
}

/**
 * @note This is not inlined because it depends on the definition of tgSimView.
 */
tgWorld& tgSimulation::getWorld() const
{
    return m_view.world();
}

void tgSimulation::step(double dt) const
{
// Trying to profile here creates trouble for tgLinearString -  this is outside of the profile loop	
	
        if (dt <= 0)
    {
        throw std::invalid_argument("dt for step is not positive");
    }
    else
    {
        // Step the world.
        // This can be done before or after stepping the models.
        m_view.world().step(dt);

        // Step the models
        for (std::size_t i = 0; i < m_models.size(); i++)
        {
            m_models[i]->step(dt);
        }
        
        // Step the obstacles
        /// @todo determine if this is necessary
        for (std::size_t i = 0; i < m_obstacles.size(); i++)
        {
            m_obstacles[i]->step(dt);
        }

	// Step the data managers
	for (std::size_t i = 0; i < m_dataManagers.size(); i++) {
	  m_dataManagers[i]->step(dt);
	}
    }
}
  
void tgSimulation::teardown()
{
    const size_t n = m_models.size();
    for (std::size_t i = 0; i < n; i++)
    {
        tgModel * const pModel = m_models[i];
        assert(pModel != NULL);
        
        pModel->teardown();
    }
    
    while(m_obstacles.size() != 0)
    {
        tgModel * const pModel = m_obstacles.back();
        assert(pModel != NULL);
        
        pModel->teardown();
        
        // Remove and destroy element
        delete pModel;
        m_obstacles.pop_back();
    }
    assert(m_obstacles.empty());

    // Similar to the models and obstacles, tear down the data managers.
    const size_t num_DM = m_dataManagers.size(); //why not in the loop gaurd?...
    for (std::size_t i = 0; i < num_DM; i++) {
      tgDataManager* const pDataManager = m_dataManagers[i];
      assert(pDataManager != NULL);
      // perform the actual teardown
      pDataManager->teardown();
    }
    
    // Reset the world after the models - models need world info for
    // their onTeardown() functions
    m_view.world().reset();
    // Postcondition
    assert(invariant());
}

void tgSimulation::run() const
{
    m_view.run();
}

void tgSimulation::run(int steps) const
{    
    m_view.run(steps);
}

bool tgSimulation::invariant() const
{
  return true;
}   
