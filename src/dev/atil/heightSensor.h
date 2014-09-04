/*
 * abstractMarker.h
 *
 *  Created on: Jul 15, 2014
 *      Author: atiliscen
 */

#ifndef HEIGHTSENSOR_H_
#define HEIGHTSENSOR_H_

#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "core/tgSubject.h"
#include "btBulletDynamicsCommon.h"
#include "core/tgWorld.h"
#include "core/tgBulletUtil.h"


/* ColoredMarkers are non-physical markers that are attached to a specific physical body.
 * They are  presented as colored spheres at the simulation.
 * Markers are considered to move with the physical body.
 * (they use orientation and translation vectors of the body)
 * Using the orientation of the physical body, it can return its absolute position when requested.
*/

class heightSensor : public tgSubject<heightSensor>
{
public:
        heightSensor();
        /*construct by attaching to the body using the absolute position pos, and highlight it with the given color.*/
        heightSensor(const btRigidBody *body,btVector3 pos, int nodeNumber, tgWorld& world);
        /* Returns the current world position by looking at the attached objects transformation quaternion */
        btVector3 getWorldPosition() const;
        /* Returns the relative position to the center of mass of the attached body */
		btVector3 getRelativePosition() const;
		double getHeight() const;

	int getNodeNumber() const {
		return nodeNumber;
	}

private:
        const btRigidBody *attachedBody;
        //Relative position to the body when it is first constructed
        btVector3 attachedRelativeOriginalPosition;
        int nodeNumber;
        btDynamicsWorld *btworld;
};

#endif /* HEIGHTSENSOR_H_ */
