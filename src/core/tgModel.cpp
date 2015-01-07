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
 * @file tgModel.cpp
 * @brief Contains the definitions of members of class tgModel
 * $Id$
 */

// This module
#include "tgModel.h"
// This application
#include "tgModelVisitor.h"
#include "abstractMarker.h"
// The C++ Standard Library
#include <stdexcept>

tgModel::tgModel()
{
  // Postcondition
  assert(invariant());
}

tgModel::tgModel(const tgTags& tags) :
        tgTaggable(tags)
{
  assert(invariant());
}

tgModel::~tgModel()
{
  const size_t n = m_children.size();
  for (size_t i = 0; i < n; ++i)
  {
    tgModel * const pChild = m_children[i];
    // It is safe to delete NULL, but this is an invariant
    assert(pChild != NULL);
    delete pChild;
  }
}

void tgModel::setup(tgWorld& world)
{
  for (std::size_t i = 0; i < m_children.size(); i++)
  {
    m_children[i]->setup(world);
  }

  // Postcondition
  assert(invariant());
}

void tgModel::teardown()
{
  for (std::size_t i = 0; i < m_children.size(); i++)
  {
    m_children[i]->teardown();
    delete m_children[i];
  }
  m_children.clear();
  //Clear the markers
  this->m_markers.clear();

  // Postcondition
  assert(invariant());
  assert(m_children.empty());
}

void tgModel::step(double dt) 
{
  if (dt <= 0.0)
  {
    throw std::invalid_argument("dt is not positive");
  }
  else
  {
    // Note: You can adjust whether to step children before notifying 
    // controllers or the other way around in your model
    const size_t n = m_children.size();
    for (std::size_t i = 0; i < n; i++)
    {
      tgModel* const pChild = m_children[i];
      assert(pChild != NULL);
      pChild->step(dt);
    }
  }

  // Postcondition
  assert(invariant());
}

void tgModel::onVisit(const tgModelVisitor& r) const
{
        r.render(*this);

        // Call onRender for all children (if we have any)
    const size_t n = m_children.size();
        for (std::size_t i = 0; i < n; i++)
        {
        tgModel * const pChild = m_children[i];
        assert(pChild != NULL);
            pChild->onVisit(r);
        }

        // Postcondition
        assert(invariant());
}

void tgModel::addChild(tgModel* pChild)
{
  // Preconditoin
  if (pChild == NULL)
  {
    throw std::invalid_argument("child is NULL");
  }
  else if (pChild == this)
  {
    throw std::invalid_argument("child is this object");
  } 
  else 
  {
    const std::vector<tgModel*> descendants = getDescendants();
    if (std::find(descendants.begin(), descendants.end(), pChild) !=
    descendants.end())
    {
      throw std::invalid_argument("child is already a descendant");
    }
  }

  m_children.push_back(pChild);

  // Postcondition
  assert(invariant());
  assert(!m_children.empty());
  assert(std::find(m_children.begin(), m_children.end(), pChild) !=
     m_children.end());
}

std::string tgModel::toString(std::string prefix) const
{
  std::string p = "  ";  
  std::ostringstream os;
  os << prefix << "tgModel(" << std::endl;
  os << prefix << p << "Children:" << std::endl;
  for(std::size_t i = 0; i < m_children.size(); i++) {
    os << m_children[i]->toString(prefix + p) << std::endl;
  }
  os << prefix << p << "Tags: [" << getTags() << "]" << std::endl;
  os << prefix << ")";
  return os.str();
}

/**
 * @todo Unnecessary copying can be avoided by pasing the result
 * collection in the recursive step.
 */
std::vector<tgModel*> tgModel::getDescendants() const
{
  std::vector<tgModel*> result;
  const size_t n = m_children.size();
  for (std::size_t i = 0; i < n; i++)
  {
    tgModel* const pChild = m_children[i];
    assert(pChild != NULL);
    result.push_back(pChild);
    // Recursion
    const std::vector<tgModel*> cd = pChild->getDescendants();
    result.insert(result.end(), cd.begin(), cd.end());
  }
  return result;
}

const std::vector<abstractMarker>& tgModel::getMarkers() const {
    return m_markers;
}

void tgModel::addMarker(abstractMarker a){
    m_markers.push_back(a);
}

bool tgModel::invariant() const
{
  // No child is NULL
  // No child appears more than once in the tree
  return true;
}

std::ostream&
operator<<(std::ostream& os, const tgModel& obj)
{
    os << obj.toString() << std::endl;
    return os;
}
