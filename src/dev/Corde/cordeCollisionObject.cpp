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
#include "cordeAnchor.h"
#include "cordeCollisionObject.h"
#include "cordeCollisionShape.h"
#include "cordeColliders.h"

// Core library
#include "core/tgWorld.h"
#include "core/tgBulletUtil.h"

// Builder library (printing)
#include "tgcreator/tgUtil.h"

// The Bullet Physics Library
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/BroadphaseCollision/btDispatcher.h"
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

//The C++ Standard Library
#include <iostream>
#include <stdexcept>

cordeCollisionObject::cordeCollisionObject(std::vector<btVector3>& centerLine, tgWorld& world, CordeModel::Config& Config) :
CordeModel(centerLine, Config),
m_broadphase(tgBulletUtil::worldToDynamicsWorld(world).getBroadphase()),
m_dispatcher(tgBulletUtil::worldToDynamicsWorld(world).getDispatcher()),
m_gravity(tgBulletUtil::worldToDynamicsWorld(world).getGravity()) // Note, if gravity ever needs to change, we're screwed with this method
{
	// Enum from btCollisionObject
	//m_internalType		=	CO_USER_TYPE;
	//m_collisionFlags =  CF_KINEMATIC_OBJECT;
	
	// Apparently a hack...
	m_collisionShape = new cordeCollisionShape(this);
	m_collisionShape->setMargin(Config.radius);
	
	const btScalar		margin = getCollisionShape()->getMargin();
	
	// Get cordeModel data into collision object
	for (std::size_t i = 0, ni = m_massPoints.size(); i < ni; i++)
	{
		CordePositionElement&	n = *m_massPoints[i];
		
		int j = i;
		if (i == ni - 1)
		{
			j = i - 1;
		}
		btVector3 box = btVector3(margin, margin, linkLengths[j]/2.0);
			
		CordeQuaternionElement& q = *m_centerlines[j];
		btScalar angle = q.q_new.getAngle();
		btVector3 axis = q.q_new.getAxis();
		box.rotate(axis, angle);
		
		m_leaves.push_back( m_ndbvt.insert(btDbvtVolume::FromCE(n.pos, box),&n) );
	}
	
	/// In btSoftBody this was passsed in from outside and initialized in the world. Check for compatability
	m_sparsesdf.Initialize();
	
	updateAABBBounds();
	stepPrerequisites();
///@todo examine how to reconfigure collision shape defaults (m_friction, etc)	
}
	
cordeCollisionObject::~cordeCollisionObject() 
{
	delete m_collisionShape;	
	
}

void cordeCollisionObject::appendAnchor (std::size_t node,
											btRigidBody* body,
											btVector3 pos)
{
    if (node >= m_massPoints.size())
    {
        throw std::invalid_argument("Node index is greater than size of m_massPoints");
    }
    if (!body)
    {
		throw std::invalid_argument("Invalid pointer for rigid body");
	}
    
	cordeAnchor* anchor = new cordeAnchor(body, m_massPoints[node], pos);
	
	m_anchors.push_back(anchor);
}

void cordeCollisionObject::predictMotion(btScalar dt)
{
	assert(m_massPoints.size() == m_leaves.size());
	
	/* Prepare				*/ 
	m_sst.sdt		=	dt;
	m_sst.isdt		=	1.0 / m_sst.sdt;
	m_sst.velmrg	=	m_sst.sdt*3; ///@todo investigate
	m_sst.radmrg	=	getCollisionShape()->getMargin();
	m_sst.updmrg	=	m_sst.radmrg;
	
	computeInternalForces();
	
	applyUniformAcc(m_gravity);
	
    unconstrainedMotion(dt);
	
	updateAABBBounds();	
	
	ATTRIBUTE_ALIGNED16(btDbvtVolume)	vol;
	for(std::size_t i = 0, ni= m_massPoints.size(); i<ni; ++i)
	{
		CordePositionElement&	n = *m_massPoints[i];
		
		int j = i;
		if (i == ni - 1)
		{
			j = i - 1;
		}
		btVector3 box = btVector3(m_sst.radmrg, m_sst.radmrg, linkLengths[j]/2.0);
			
		CordeQuaternionElement& q = *m_centerlines[j];
		btScalar angle = q.q_new.getAngle();
		btVector3 axis = q.q_new.getAxis();
		box.rotate(axis, angle);
		
		vol = btDbvtVolume::FromCE(n.pos_new, box);
		m_ndbvt.update(	m_leaves[i], 
			vol,
			n.vel_new * m_sst.velmrg,
			m_sst.updmrg);
	}
	
	/* Clear Contacts */
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
	
	/* Optimize dbvt's		*/ 
	// Argument is number of passes
	m_ndbvt.optimizeIncremental(1);
}

void cordeCollisionObject::integrateMotion (btScalar dt)
{
	constrainMotion(dt);
	
#if (0)	
	if (simTime >= 1.0/10.0)
    {
        size_t n = m_massPoints.size();
        for (std::size_t i = 0; i < n; i++)
        {
            std::cout << "Position " << i << " " << m_massPoints[i]->pos << std::endl
					  << "Velocity " << i << " " << m_massPoints[i]->vel << std::endl
                      << "Force " << i << " " << m_massPoints[i]->force << std::endl;
            if (i < n - 1)
            {
            std::cout << "Quaternion " << i << " " << m_centerlines[i]->q << std::endl
                      << "Qdot " << i << " " << m_centerlines[i]->qdot << std::endl
                      << "Omega " << i << " " << m_centerlines[i]->omega << std::endl
                      << "Force " << i << " " << m_centerlines[i]->tprime << std::endl
                      << "Torque " << i << " " << m_centerlines[i]->torques << std::endl;
            }       
        }
        simTime = 0.0;
    }
#endif	
    simTime += dt;
	stepPrerequisites();
}

void cordeCollisionObject::solveConstraints(btScalar dt)
{
#if (0)
	if (m_rcontacts.size() > 0)
	{
		std::cout << "Collisions!" << std::endl;
	}
#endif
	// How would one iterate on this step such that constraints are not violated?
	// How are rigid bodies involved?
#if (1)
	solveAnchors(dt);
#endif
	solveRContacts();
	solveSContacts();
}

void cordeCollisionObject::defaultCollisionHandler(cordeCollisionObject* otherSoftBody)
{
	
	cordeColliders::CollideSDF_SS	docollide;
	//std::cout << "Soft soft collisions!" << std::endl;
	docollide.ProcessSoftSoft(this, otherSoftBody);
}

void cordeCollisionObject::defaultCollisionHandler(const btCollisionObjectWrapper* pcoWrap) 
{ 
	cordeColliders::CollideSDF_RS	docollide;		
	btRigidBody*		prb1=(btRigidBody*) btRigidBody::upcast(pcoWrap->getCollisionObject());
	btTransform	wtr = pcoWrap->getWorldTransform();

	const btTransform	ctr=pcoWrap->getWorldTransform();
	const btScalar		timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
	const btScalar		basemargin=getCollisionShape()->getMargin();
	btVector3			mins;
	btVector3			maxs;
	ATTRIBUTE_ALIGNED16(btDbvtVolume)		volume;
	pcoWrap->getCollisionShape()->getAabb(	pcoWrap->getWorldTransform(),
		mins,
		maxs);
	volume = btDbvtVolume::FromMM(mins,maxs);
	volume.Expand(btVector3(basemargin,basemargin,basemargin));		
	docollide.psb		=	this;
	docollide.m_colObj1Wrap = pcoWrap;
	docollide.m_rigidBody = prb1;

	docollide.dynmargin	=	basemargin + timemargin;
	docollide.stamargin	=	basemargin;
	m_ndbvt.collideTV(m_ndbvt.m_root,volume,docollide);
}

bool	cordeCollisionObject::checkContact(	const btCollisionObjectWrapper* colObjWrap,
											 const btVector3& x,
											 btScalar margin,
											 cordeCollisionObject::sCti& cti)
{
	btVector3 nrm;
	const btCollisionShape *shp = colObjWrap->getCollisionShape();

	const btTransform &wtr = colObjWrap->getWorldTransform();
	//todo: check which transform is needed here

	btScalar dst = 
		m_sparsesdf.Evaluate(	
			wtr.invXform(x),
			shp,
			nrm,
			margin);
	if(dst<0)
	{
		cti.m_dist 	 = -dst;
		cti.m_colObj = colObjWrap->getCollisionObject();
		cti.m_normal = wtr.getBasis()*nrm;
		cti.m_offset = -btDot( cti.m_normal, x - cti.m_normal * dst );
		return(true);
	}
	return(false);
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

void cordeCollisionObject::solveRContacts()
{
	const btScalar	idt = m_sst.isdt;
	const btScalar	mrg = getCollisionShape()->getMargin();
	for(int i = 0,ni = m_rcontacts.size(); i < ni; ++i)
	{
		const RContact&		c = m_rcontacts[i];
		const sCti&			cti = c.m_cti;	 
		
		// Normal * mass ratio * penetration distance
		btVector3 rSoft = cti.m_normal * c.m_c2 * cti.m_dist;
		btVector3 rRigid =  -cti.m_normal * (1.f - c.m_c2) * cti.m_dist;
		
		btVector3 fSoft = pow(idt, 2.0) * c.m_node->mass * rSoft;
		btScalar  magSoft = fSoft.length();
		c.m_node->applyForce(fSoft - c.m_c3 * magSoft);
		
		btRigidBody* rBody = (btRigidBody*) btRigidBody::upcast(cti.m_colObj);
		
		btScalar rMass = rBody->getInvMass() > 0.0 ? 1.0 / rBody->getInvMass() : 0.0;
		
		btVector3 fRigid = pow(idt, 1.0) * rMass * rRigid;
		btScalar  magRigid = fRigid.length();
		
		if (rBody)
		{
			rBody->applyImpulse(fRigid + c.m_c3 * magRigid, c.m_c1);
		}
	}
}

void cordeCollisionObject::solveSContacts()
{
	/// This is a compromise between Spillman's algorithm and Bullet's collision detection
	/// Ideally we would find the exact point of a collision. It may be possible that Bullet can do this
	const btScalar	idt = m_sst.isdt;
	for(int i = 0, ni = m_scontacts.size(); i < ni; ++i)
	{
		const SContact&		c = m_scontacts[i];
		
		// Normal * mass ratio * penetration distance
		btVector3 rA = -c.m_normal * c.m_massRatio * c.m_depth;
		btVector3 rB =  c.m_normal * (1.f - c.m_massRatio) * c.m_depth;
		
		btVector3 fA = pow(idt, 2.0) * c.m_nodea->mass * rA;
		btScalar  magA = fA.length();
		c.m_nodea->applyForce(fA - c.m_friction * magA);
	
		btVector3 fB = pow(idt, 2.0) * c.m_nodeb->mass * rB;
		btScalar  magB = fB.length();
		c.m_nodeb->applyForce(fB + c.m_friction * magB);			
	}
}

void cordeCollisionObject::solveAnchors(const double dt)
{
	for( std::size_t i = 0; i < m_anchors.size(); i++)
	{
		m_anchors[i]->solve(dt);
	}
}
