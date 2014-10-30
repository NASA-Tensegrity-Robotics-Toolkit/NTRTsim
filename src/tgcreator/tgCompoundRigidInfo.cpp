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
 * @file tgCompoundRigidInfo.cpp
 * @brief Implementaton of class tgCompoundRigidInfo 
 * @author Ryan Adams
 * @date January 2014
 * $Id$
 */
#include "tgCompoundRigidInfo.h"

tgCompoundRigidInfo::tgCompoundRigidInfo() : m_compoundShape(NULL), tgRigidInfo()
{
}

tgModel* tgCompoundRigidInfo::createModel(tgWorld& world)
{
    // @todo: make tgModel a true composite and use that here
    tgModel* result = new tgModel();
    for(int i = 0; i < m_rigids.size(); i++) {
        result->addChild(m_rigids[i]->createModel(world));
    }
    return result;
}

void tgCompoundRigidInfo::addRigid(tgRigidInfo& rigid)
{
    m_rigids.push_back(&rigid);
}
    
btVector3 tgCompoundRigidInfo::getCenterOfMass() const
{
    btVector3 sum = btVector3(0.0, 0.0, 0.0);
    for (int ii = 0; ii < m_rigids.size(); ii++)
    {
        /* const */ tgRigidInfo * const rigid = m_rigids[ii];
        sum += (rigid->getCenterOfMass() * rigid->getMass());
    }
    const double totalMass = getMass();
    return
      (totalMass == 0.0) ? btVector3(0.0, 0.0, 0.0) : (sum / totalMass);
}
    

btCompoundShape* tgCompoundRigidInfo::createCompoundShape(tgWorld& world) const
{
    if (m_compoundShape == 0)
    {
        // Deallocated by the world implementation
        m_compoundShape = new btCompoundShape(&world);

        const btVector3 com = getCenterOfMass();

        for (int ii = 0; ii < m_rigids.size(); ii++)
        {
            tgRigidInfo* const rigid = m_rigids[ii];
            btTransform t = rigid->getTransform();
            t.setOrigin(t.getOrigin() - com);
            m_compoundShape->addChildShape(t, rigid->getCollisionShape(world));
        }        
        // Add the collision shape to the array so we can delete it later
        tgWorldBulletPhysicsImpl& bulletWorld =
          (tgWorldBulletPhysicsImpl&)world.implementation();
        bulletWorld.addCollisionShape(m_compoundShape);

    }
    return m_compoundShape;
}

btCollisionShape* tgCompoundRigidInfo::getCollisionShape(tgWorld& world) const
{
    if (m_compoundShape == 0)
    {
        m_compoundShape = createCompoundShape(world);
    }
    return m_compoundShape;
}

btTransform tgCompoundRigidInfo::getTransform() const
{
    btTransform t;
    t.setIdentity();
    t.setOrigin(getCenterOfMass());
    return t;
}
    
double tgCompoundRigidInfo::getMass() const
{
    /// @todo Use std::accumulate()
    double mass = 0.0;
    for (int ii = 0; ii < m_rigids.size(); ii++)
    {
        mass += m_rigids[ii]->getMass();
    }
    return mass;
}

btRigidBody* tgCompoundRigidInfo::getRigidBody()
{
	btRigidBody* rb = tgCast::cast<btCollisionObject, btRigidBody>(m_collisionObject);
	return rb;
}

const btRigidBody* tgCompoundRigidInfo::getRigidBody() const
{
	btRigidBody* rb = tgCast::cast<btCollisionObject, btRigidBody>(m_collisionObject);
	return rb;
}
    
void tgCompoundRigidInfo::setRigidBody(btRigidBody* const rigidBody)
{
    m_collisionObject = rigidBody;
    // Set the rigid body for all components
    /// @todo Use std::for_each()
    for (int ii = 0; ii < m_rigids.size(); ii++) {
        m_rigids[ii]->setRigidBody(rigidBody);
    }
}

void tgCompoundRigidInfo::setCollisionObject(btCollisionObject* collisionObject)
{
	m_collisionObject = collisionObject;
    // Set the rigid body for all components
    /// @todo Use std::for_each()
    for (int ii = 0; ii < m_rigids.size(); ii++) {
        m_rigids[ii]->setCollisionObject(collisionObject);
    }
}
      
    
std::set<tgRigidInfo*> tgCompoundRigidInfo::getLeafRigids()
{
    std::set<tgRigidInfo*> leaves;
    for (int ii = 0; ii < m_rigids.size(); ii++) {
        tgRigidInfo * const rigid = m_rigids[ii];
        if (rigid->isCompound())
        {
            // Insert the leaves from the compound recursively
            const std::set<tgRigidInfo*> compoundLeaves =
               rigid->getLeafRigids();
            leaves.insert(compoundLeaves.begin(), compoundLeaves.end());
        }
        else
        {
            leaves.insert(rigid);
        }
    }        
    return leaves;
}
    
bool tgCompoundRigidInfo::containsNode(const btVector3& nodeVector) const
{
    /// @todo Use std::find_if()
    for (int ii = 0; ii < m_rigids.size(); ii++)
    {
      if (m_rigids[ii]->containsNode(nodeVector))
        {
            return true;
        }
    }
    return false;
}
    
bool tgCompoundRigidInfo::sharesNodesWith(const tgRigidInfo& other) const
{
    /// @todo Use std::find_if()
    for (int ii = 0; ii < m_rigids.size(); ii++)
    {
        if (m_rigids[ii]->sharesNodesWith(other)) {
            return true;
        }
    }
    return false;
}
    
std::set<btVector3> tgCompoundRigidInfo::getContainedNodes() const
{
    /// @todo Use std::accumulate()
    std::set<btVector3> contained;
    for (int ii = 0; ii < m_rigids.size(); ii++) 
{
        const std::set<btVector3> nodes = m_rigids[ii]->getContainedNodes();
        contained.insert(nodes.begin(), nodes.end());
    }
    return contained;
}

