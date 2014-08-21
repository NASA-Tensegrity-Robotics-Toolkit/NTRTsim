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
 * 
This class is a modified version of btSoftBody
* from Bullet 2.82. A copy of the z-lib license from the Bullet Physics
* Library is provided below:

Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
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
#include "cordeColliders.h"

// Core library
#include "core/tgWorld.h"
#include "core/tgBulletUtil.h"

// The Bullet Physics Library
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"

//The C++ Standard Library
#include <iostream>

cordeCollisionObject::cordeCollisionObject(std::vector<btVector3>& centerLine, tgWorld& world, CordeModel::Config& Config) :
CordeModel(centerLine, Config),
m_broadphase(tgBulletUtil::worldToDynamicsWorld(world).getBroadphase()),
m_dispatcher(tgBulletUtil::worldToDynamicsWorld(world).getDispatcher())
{
	// Enum from btCollisionObject
	m_internalType		=	CO_USER_TYPE;
	m_collisionFlags =  CF_KINEMATIC_OBJECT;
	
	// Apparently a hack...
	m_collisionShape = new cordeCollisionShape(this);
	m_collisionShape->setMargin(0.25f);
	
	const btScalar		margin=getCollisionShape()->getMargin();
	
	// Get cordeModel data into collision object
	for (std::size_t i = 0; i < m_massPoints.size(); i++)
	{
		CordePositionElement&	n = *m_massPoints[i];
		m_leaves.push_back( m_ndbvt.insert(btDbvtVolume::FromCR(n.pos, margin),&n) );
	}
	
	updateAABBBounds();
	stepPrerequisites();
///@todo examine how to reconfigure collision shape defaults (m_friction, etc)	
}
	
cordeCollisionObject::~cordeCollisionObject() 
{
	delete m_collisionShape;	
}

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

void cordeCollisionObject::defaultCollisionHandler(const btCollisionObjectWrapper* collisionObjectWrap ) 
{ 
	std::cout << "Handling rigid collision!" << std::endl;
	
	cordeColliders::CollideSDF_RS	docollide;		
	btRigidBody*		prb1=(btRigidBody*) btRigidBody::upcast(pcoWrap->getCollisionObject());
	btTransform	wtr=pcoWrap->getWorldTransform();

	const btTransform	ctr=pcoWrap->getWorldTransform();
	const btScalar		timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
	const btScalar		basemargin=getCollisionShape()->getMargin();
	btVector3			mins;
	btVector3			maxs;
	ATTRIBUTE_ALIGNED16(btDbvtVolume)		volume;
	pcoWrap->getCollisionShape()->getAabb(	pcoWrap->getWorldTransform(),
		mins,
		maxs);
	volume=btDbvtVolume::FromMM(mins,maxs);
	volume.Expand(btVector3(basemargin,basemargin,basemargin));		
	docollide.psb		=	this;
	docollide.m_colObj1Wrap = pcoWrap;
	docollide.m_rigidBody = prb1;

	docollide.dynmargin	=	basemargin+timemargin;
	docollide.stamargin	=	basemargin;
	m_ndbvt.collideTV(m_ndbvt.m_root,volume,docollide);
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
			m_broadphase->setAabb(	getBroadphaseHandle(),
				m_bounds[0],
				m_bounds[1],
				m_dispatcher);
		}
	}
	else
	{
		m_bounds[0]=
			m_bounds[1]=btVector3(0,0,0);
	}
}
