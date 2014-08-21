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

	if(	(!n.m_battach)&&
		psb->checkContact(m_colObj1Wrap, n.pos, m, c.m_cti))
	{
		const btScalar	ima = 1.0 / n.mass; // Already established mass is positive
		const btScalar	imb = m_rigidBody? m_rigidBody->getInvMass() : 0.f;
		const btScalar	ms = ima + imb;
		if(ms>0)
		{
			const btTransform&	wtr=m_rigidBody?m_rigidBody->getWorldTransform() : m_colObj1Wrap->getCollisionObject()->getWorldTransform();
			static const btMatrix3x3	iwiStatic(0,0,0,0,0,0,0,0,0);
			const btMatrix3x3&	iwi = m_rigidBody?m_rigidBody->getInvInertiaTensorWorld() : iwiStatic;
			const btVector3		ra = n.pos - wtr.getOrigin();
			const btVector3		va = m_rigidBody ? m_rigidBody->getVelocityInLocalPoint(ra) * psb->m_sst.sdt : btVector3(0,0,0);
			const btVector3		vb = n.vel;	
			const btVector3		vr = vb - va;
			const btScalar		dn = btDot(vr,c.m_cti.m_normal);
			const btVector3		fv = vr - c.m_cti.m_normal*dn;
			const btScalar		fc = psb->m_cfg.kDF*m_colObj1Wrap->getCollisionObject()->getFriction();
			c.m_node	=	&n;
			c.m_c0		=	ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
			c.m_c1		=	ra;
			c.m_c2		=	ima*psb->m_sst.sdt;
			c.m_c3		=	fv.length2()<(dn*fc*dn*fc)?0:1-fc;
			c.m_c4		=	m_colObj1Wrap->getCollisionObject()->isStaticOrKinematicObject()?psb->m_cfg.kKHR:psb->m_cfg.kCHR;
			psb->m_rcontacts.push_back(c);
			if (m_rigidBody)
				m_rigidBody->activate();
		}
	}
}
