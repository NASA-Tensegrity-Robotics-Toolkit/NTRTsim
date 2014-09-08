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

#ifndef CORDE_ANCHOR
#define CORDE_ANCHOR

#include "CordeModel.h"

// The Bullet Physics library
#include "LinearMath/btScalar.h"
#include "LinearMath/btVector3.h"

class btRigidBody;

class cordeAnchor
{
public:
    cordeAnchor(btRigidBody* body, CordeModel::CordePositionElement* element, btVector3 pos);
    
    ~cordeAnchor();
    
    btVector3 getWorldPosition();

    // Relative to the body
    btVector3 getRelativePosition();
	
	void solve(const double dt);  

private:
 
	cordeAnchor();

    btRigidBody* attachedBody;
	
	CordeModel::CordePositionElement* m_attachedElement;
	
    // Relative to the body when it is first constructed
    const btVector3 attachedRelativeOriginalPosition;

};

#endif //CORDE_ANCHOR
