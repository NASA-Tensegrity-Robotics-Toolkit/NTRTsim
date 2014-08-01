#include "cordeAnchor.h"

// The Bullet Physics library
#include "BulletDynamics/Dynamics/btRigidBody.h"

cordeAnchor::cordeAnchor()
{
}

cordeAnchor::cordeAnchor(btRigidBody * body,
               btVector3 worldPos) :
  attachedBody(body),
  // Find relative position
  // This should give relative position in a default orientation.
  attachedRelativeOriginalPosition(attachedBody->getWorldTransform().inverse() *
                   worldPos)
{
}

cordeAnchor::~cordeAnchor()
{
    // World should delete this
    attachedBody = NULL;
    
}

// This returns current position relative to the rigidbody.
btVector3 cordeAnchor::getRelativePosition()
{
    const btTransform tr = attachedBody->getWorldTransform();
    const btVector3 worldPos = tr * attachedRelativeOriginalPosition;
    return worldPos-this->attachedBody->getCenterOfMassPosition();
}

btVector3 cordeAnchor::getWorldPosition()
{
    const btTransform tr = attachedBody->getWorldTransform();
    return tr * attachedRelativeOriginalPosition;
}
