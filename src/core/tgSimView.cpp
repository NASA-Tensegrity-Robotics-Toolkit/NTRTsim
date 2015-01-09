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
 * @file tgSimView.cpp
 * @brief Contains the definitions of members of class tgSimView
 * @author Brian Mirletz, Ryan Adams
 * $Id$
 */

// This module
#include "tgSimulation.h"
// This application
#include "tgModelVisitor.h"
#include "tgSimView.h"
// The C++ Standard Library
#include <cassert>  
#include <iostream>
#include <stdexcept>

tgSimView::tgSimView(tgWorld& world,
             double stepSize,
             double renderRate) :
  m_world(world),
  m_pSimulation(NULL),
  m_pModelVisitor(NULL),
  m_stepSize(stepSize),
  m_renderRate(renderRate),         
  m_renderTime(0.0),
  m_initialized(false)
{
  if (m_stepSize < 0.0)
  {
    throw std::invalid_argument("stepSize is not positive");
  }
  else if (renderRate < m_stepSize)
  {
    throw std::invalid_argument("renderRate is less than stepSize");
  }
    
  // Postcondition
  assert(invariant());
  assert(m_pSimulation == NULL);
  assert(m_pModelVisitor == NULL);
  assert(m_stepSize == stepSize);
  assert(m_renderRate == renderRate);
  assert(m_renderTime == 0.0);
  assert(!m_initialized);
}

tgSimView::~tgSimView()
{
        if (m_pSimulation != NULL)
        {
            // The tgSimView has been passed to a tgSimulation
            teardown();
    }
    delete m_pModelVisitor;
}


void tgSimView::bindToSimulation(tgSimulation& simulation)
{
  if (m_pSimulation != NULL)
  {
      throw 
        std::invalid_argument("The view already belongs to a simulation.");
  }
  else
  {
          m_pSimulation = &simulation;
      tgWorld& world = simulation.getWorld();
      bindToWorld(world);
  }

  // Postcondition
  assert(invariant());
  assert(m_pSimulation == &simulation);
}

void tgSimView::releaseFromSimulation()
{
        // The destructor that calls this must not fail, so don't assert or throw
        // on a precondition
        m_pSimulation = NULL;
    // The destructor that calls this must not fail, so don't assert a
    // postcondition
}

void tgSimView::bindToWorld(tgWorld& world)
{
}

void tgSimView::setup()
{
  assert(m_pSimulation != NULL);
  // Just note that this function was called. 
  // tgSimViewGraphics needs to know for now.
  m_initialized = true;

  // Postcondition
  assert(invariant());
  assert(m_initialized);
}

void tgSimView::teardown()
{
  // Just note that this function was called.
  // tgSimViewGraphics needs to know for now.
  m_initialized = false;

  // Postcondition
  assert(invariant());
  assert(!m_initialized);
}

void tgSimView::run()
    {
        // This would normally run forever, but this is just for testing
        run(10);
    }

void tgSimView::run(int steps) 
{
    if (m_pSimulation != NULL)
    {
            // The tgSimView has been passed to a tgSimulation
        std::cout << "SimView::run("<<steps<<")" << std::endl;
        // This would normally run forever, but this is just for testing
        m_renderTime = 0;
        double totalTime = 0.0;
        for (int i = 0; i < steps; i++) {
            m_pSimulation->step(m_stepSize);    
            m_renderTime += m_stepSize;
            totalTime += m_stepSize;
            
            if (m_renderTime >= m_renderRate) {
                render();
                //std::cout << totalTime << std::endl;
                m_renderTime = 0;
            }
        }
    }
}

void tgSimView::render() const
{
	if ((m_pSimulation != NULL) && (m_pModelVisitor != NULL))
    {
            // The tgSimView has been passed to a tgSimulation
        m_pSimulation->onVisit(*m_pModelVisitor);
    }
}

void tgSimView::render(const tgModelVisitor& r) const
{
    if (m_pSimulation != NULL)
    {
            // The tgSimView has been passed to a tgSimulation
        m_pSimulation->onVisit(r);
    }
}

void tgSimView::reset() 
{
	if (m_pSimulation != NULL)
    {
            // The tgSimView has been passed to a tgSimulation
            m_pSimulation->reset();
    }
}
    
void tgSimView::setRenderRate(double renderRate)
{
	m_renderRate = (renderRate > m_stepSize) ? renderRate : m_stepSize;

    // Postcondition
    assert(invariant());
}
    
void tgSimView::setStepSize(double stepSize)
{
  if (stepSize <= 0.0)
  {
    throw std::invalid_argument("stepSize is not positive");
  }
  else
  {
      m_stepSize = stepSize;
      // Assure that the render rate is no less than the new step size
      setRenderRate(m_renderRate);
  }

  // Postcondition
  assert(invariant());
  assert((stepSize <= 0.0) || (m_stepSize == stepSize));
}

bool tgSimView::invariant() const
{
  return
    (m_stepSize >= 0.0) &&
    (m_renderRate >= m_stepSize) &&
    (m_renderTime >= 0.0);
}

