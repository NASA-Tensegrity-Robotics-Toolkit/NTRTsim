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
 * @file cordeCollisionObject.cpp
 * @brief Interface Between Corde Model and Bullet
 * @author Brian Mirletz
 * $Id$
 */

// This Module
#include "cordeCollisionObject.h"
#include "cordeCollisionShape.h"

// Core library
#include "core/tgWorld.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorldBulletPhysicsImpl.h"

// The Bullet Physics Library
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"


cordeCollisionObject::cordeCollisionObject(std::vector<btVector3>& centerLine, tgWorldBulletPhysicsImpl& world, CordeModel::Config& Config) :
CordeModel(centerLine, Config),
m_broadphase(world.getBroadphase()),
m_dispatcher(world.getDispatcher())
{
	// Enum from btCollisionObject
	m_internalType		=	CO_USER_TYPE;
	
	// Apparently a hack...
	m_collisionShape = new cordeCollisionShape(this);
	m_collisionShape->setMargin(0.25f);
	
	const btScalar		margin=getCollisionShape()->getMargin();
	
	// Get cordeModel data into collision object
	for (std::size_t i; i < m_massPoints.size(); i++)
	{
		CordePositionElement&	n = *m_massPoints[i];
		m_leaves.push_back( m_ndbvt.insert(btDbvtVolume::FromCR(n.pos, margin),&n) );
	}
	
	updateAABBBounds();
	stepPrerequisites();
///@todo examine how to reconfigure collision shape defaults (m_friction, etc)	
}
	
cordeCollisionObject::~cordeCollisionObject() 
{}

void cordeCollisionObject::predictMotion(btScalar dt)
{
	assert(m_massPoints.size() == m_leaves.size());
	
	/* Prepare				*/ 
	m_sst.sdt		=	dt;
	m_sst.isdt		=	1/m_sst.sdt;
	m_sst.velmrg	=	m_sst.sdt*3; ///@todo investigate
	m_sst.radmrg	=	getCollisionShape()->getMargin();
	m_sst.updmrg	=	m_sst.radmrg*(btScalar)0.25;
	
	computeInternalForces();
    unconstrainedMotion(dt);
	
	updateAABBBounds();	
	
	ATTRIBUTE_ALIGNED16(btDbvtVolume)	vol;
	for(std::size_t i=0, ni= m_massPoints.size(); i<ni; ++i)
	{
		CordePositionElement&	n = *m_massPoints[i];
		vol = btDbvtVolume::FromCR(n.pos, m_sst.radmrg);
		m_ndbvt.update(	m_leaves[i],
			vol,
			n.vel * m_sst.velmrg,
			m_sst.updmrg);
	}
	
	/* Optimize dbvt's		*/ 
	m_ndbvt.optimizeIncremental(1);
}

void cordeCollisionObject::integrateMotion (btScalar dt)
{
	constrainMotion(dt);
    simTime += dt;
	stepPrerequisites();
}

void cordeCollisionObject::updateAABBBounds()
{
	if(m_ndbvt.m_root)
	{
		const btVector3&	mins=m_ndbvt.m_root->volume.Mins();
		const btVector3&	maxs=m_ndbvt.m_root->volume.Maxs();
		const btScalar		csm=getCollisionShape()->getMargin();
		const btVector3		mrg=btVector3(	csm,
			csm,
			csm)*1; // ??? to investigate...
		m_bounds[0] = mins - mrg;
		m_bounds[1] = maxs + mrg;
		if(0 != getBroadphaseHandle())
		{					
			m_broadphase.setAabb(	getBroadphaseHandle(),
				m_bounds[0],
				m_bounds[1],
				&m_dispatcher);
		}
	}
	else
	{
		m_bounds[0]=
			m_bounds[1]=btVector3(0,0,0);
	}
}
