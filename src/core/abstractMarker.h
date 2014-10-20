/*
 * abstractMarker.h
 *
 *  Created on: Jul 15, 2014
 *      Author: atiliscen
 */

#ifndef ABSTRACTMARKER_H_
#define ABSTRACTMARKER_H_

#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "tgSubject.h"


/* ColoredMarkers are non-physical markers that are attached to a specific physical body.
 * They are  presented as colored spheres at the simulation.
 * Markers are considered to move with the physical body.
 * (they use orientation and translation vectors of the body)
 * Using the orientation of the physical body, it can return its absolute position when requested.
*/

class abstractMarker : public tgSubject<abstractMarker>
{
public:
        abstractMarker();
        /*construct by attaching to the body using the absolute position pos, and highlight it with the given color.*/
        abstractMarker(const btRigidBody *body,btVector3 pos,btVector3 color, int nodeNumber);
        /* Returns the current world position by looking at the attached objects transformation quaternion */
        btVector3 getWorldPosition() const;
        /* Returns the relative position to the center of mass of the attached body */
		btVector3 getRelativePosition() const;

	const btVector3& getColor() const {
		return color;
	}

	int getNodeNumber() const {
		return nodeNumber;
	}

private:
        btVector3 color;
        const btRigidBody *attachedBody;
        //Relative position to the body when it is first constructed
        btVector3 attachedRelativeOriginalPosition;
        int nodeNumber;
};

#endif /* ABSTRACTMARKER_H_ */
