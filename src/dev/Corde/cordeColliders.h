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

#ifndef CORDE_COLLIDERS
#define CORDE_COLLIDERS

#include "BulletCollision/BroadphaseCollision/btDbvt.h"
#include "CordeModel.h"

// Forward Declarations
class cordeCollisionObject;
class btCollisionObjectWrapper;
class btRigidBody;

struct cordeColliders
{

struct	CollideSDF_RS : btDbvt::ICollide
	{
		void		Process(const btDbvtNode* leaf);
		void		DoNode(CordeModel::CordePositionElement& n) const;
		
		cordeCollisionObject*		psb;
		const btCollisionObjectWrapper*	m_colObj1Wrap;
		btRigidBody*	m_rigidBody;
		btScalar		dynmargin;
		btScalar		stamargin;
	};

struct  CollideSDF_SS : btDbvt::ICollide
	{
		btScalar			erp;
		btScalar			idt;
		btScalar			m_margin;
		btScalar			friction;
		btScalar			threshold;
		
		CollideSDF_SS();
		
		cordeCollisionObject*	bodies[2];
		
		void		Process(const btDbvtNode* leafa, const btDbvtNode* leafb);
		
		void ProcessSoftSoft(cordeCollisionObject* psa, cordeCollisionObject* psb);
		
	};
	
};

#endif // CORDE_COLLIDERS
