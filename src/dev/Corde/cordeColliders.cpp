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

This class is a modified version of btSoftColliders
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

#include "cordeColliders.h"

#include "cordeCollisionObject.h"

#include "tgcreator/tgUtil.h"

#include "BulletCollision/CollisionDispatch/btCollisionObjectWrapper.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

// The C++ Standard Library
#include <stdexcept>

void cordeColliders::CollideSDF_RS::Process(const btDbvtNode* leaf)
{
	CordeModel::CordePositionElement*	node=(CordeModel::CordePositionElement*)leaf->data;
	DoNode(*node);
}
void cordeColliders::CollideSDF_RS::DoNode(CordeModel::CordePositionElement& n) const
{
	if (n.mass <= 0.0)
    {
        throw std::runtime_error("Mass of element is not positive.");
    }
	
	const btScalar			m = n.mass > 0 ? dynmargin:stamargin;
	cordeCollisionObject::RContact	c;

	/// @todo original bullet code checked if this was an anchor too, consider restoring
	/// Check contact populates values of m_cti
	if(	psb->checkContact(m_colObj1Wrap, n.pos_new, m, c.m_cti) && !n.isAnchor)
	{
		const btScalar	ima = 1.0 / n.mass; // Already established mass is positive
		const btScalar	imb = m_rigidBody? m_rigidBody->getInvMass() : 0.f;
		const btScalar	ms = ima + imb;
		if(ms>0)
		{
			const btTransform&	wtr=m_rigidBody?m_rigidBody->getWorldTransform() : m_colObj1Wrap->getCollisionObject()->getWorldTransform();
			static const btMatrix3x3	iwiStatic(0,0,0,0,0,0,0,0,0);
			const btMatrix3x3&	iwi = m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
			const btVector3		ra = n.pos_new - wtr.getOrigin();
			
			/// A is the rigid body, B is the soft body
			const btVector3		va = m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra) * psb->m_sst.sdt : btVector3(0,0,0);
			const btVector3		vb = n.vel_new;	
			const btVector3		vr = vb - va;
			const btScalar		dn = btDot(vr,c.m_cti.m_normal); // Magnitude of normal
			btVector3			fv = vr - c.m_cti.m_normal * dn; // Orthogonal velocity?
			
			// Use lower of two friction values
			const btScalar 		f1 = psb->m_friction;
			const btScalar		f2 = m_colObj1Wrap->getCollisionObject()->getFriction();
			const btScalar		fc = f1 < f2 ? f1 : f2;
			
			c.m_node	=	&n;
			c.m_c0		=	tgUtil::ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
			c.m_c1		=	ra;
			
			// No mass means immobile object, so softbody gets all the force
			if (imb == 0)
			{
				c.m_c2	= 1.f;
			}
			else
			{
				// mr / (ms + mr)
				c.m_c2 = (1.0 / imb) * 1.0 / (1.0 / imb + n.mass);
			}
			
			if (fv.length() != 0.0)
			{
				c.m_c3		=	fv.normalize() * fc;
			}
			else
			{
				c.m_c3 		= btVector3(0.0, 0.0, 0.0);
			}
			
			psb->m_rcontacts.push_back(c);
			if (m_rigidBody)
				m_rigidBody->activate();
		}
	}
}

cordeColliders::CollideSDF_SS::CollideSDF_SS() :
erp(1),
idt(0.0),
m_margin(0.0),
friction(0.0),
threshold(0.0)
{

}

void cordeColliders::CollideSDF_SS::Process(const btDbvtNode* leafa, const btDbvtNode* leafb)
{
	/// @todo consider making these references
	CordeModel::CordePositionElement*	nodea = (CordeModel::CordePositionElement*)leafa->data;
	CordeModel::CordePositionElement*	nodeb = (CordeModel::CordePositionElement*)leafb->data;
	
	assert(nodea);
	assert(nodeb);
	
	if (nodea->mass <= 0.0)
    {
        throw std::runtime_error("Mass of element a is not positive.");
    }
    if (nodeb->mass <= 0.0)
    {
		throw std::runtime_error("Mass of element b is not positive.");
	}
	
	bool connected = false;
	
	// Determine if the nodes are connected by checking their indicies in m_massPoints. This is a friend class, so we have access
	if (bodies[0] == bodies[1])
	{
		int i = 0;
		const int n = bodies[0]->m_massPoints.size();
		while (nodea != bodies[0]->m_massPoints[i] && i < n)
		{
			i++;
		}
		if (i == n)
		{
			throw std::runtime_error("Didn't find collsion node, bodies must not be the same!");
		}
		else if(i == 0)
		{
			if (nodeb != bodies[0]->m_massPoints[i + 1])
			{
				connected = true;
			}
		}
		else if(i == n - 1)
		{
			if(nodeb != bodies[0]->m_massPoints[i - 1])
			{
				connected = true;
			}
		}
		else
		{
			if(nodeb != bodies[0]->m_massPoints[i + 1] && nodeb != bodies[0]->m_massPoints[i - 1])
			{
				connected = true;
			}
		}
	}
	
	if(!connected)
	{
		// Copy a bunch of data
		cordeCollisionObject::SContact	c;
		c.m_nodea = nodea;
		c.m_nodeb = nodeb;
		c.m_margin = m_margin;
		
		btVector3 dist = nodea->pos_new - nodeb->pos_new;
		
		c.m_depth = c.m_margin - dist.length();
		c.m_normal = dist.normalize();
		
		if (c.m_depth < 0) // Presumably this has already been checked...
		{
		
			const btScalar ma = nodea->mass;
			const btScalar mb = nodeb->mass;
			
			c.m_massRatio = mb / (ma + mb);
			
			const btVector3		va = nodea->vel_new;
			const btVector3		vb = nodeb->vel_new;
			const btVector3		vr = vb - va;
			btVector3			fv = vr - c.m_normal * c.m_depth; // Orthogonal velocity?
			
			if (fv.length() != 0.0)
			{
				c.m_friction		=	fv.normalize() * friction;
			}
			else
			{
				c.m_friction		= btVector3(0.0, 0.0, 0.0);
			}
			
			bodies[0]->m_scontacts.push_back(c);
		}
	}
}

void cordeColliders::CollideSDF_SS::ProcessSoftSoft(cordeCollisionObject* psa, cordeCollisionObject* psb)
{
	idt			=	psa->m_sst.isdt;
	//m_margin		=	(psa->getCollisionShape()->getMargin()+psb->getCollisionShape()->getMargin())/2;
	m_margin		=	(psa->getCollisionShape()->getMargin()+psb->getCollisionShape()->getMargin());
	friction	=	btMin(psa->getFriction(), psb->getFriction());
	bodies[0]	=	psa;
	bodies[1]	=	psb;
	psa->m_ndbvt.collideTT(psa->m_ndbvt.m_root,psb->m_ndbvt.m_root,*this);
}
