/**
 * @file PrismModel.cpp
 * @brief Contains the definition of the members of the class PrismModel.
 * $Id$
 */

// This module
#include "Marker.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "core/abstractMarker.h" 
// The Bullet Physics library
#include "LinearMath/btVector3.h"
#include "core/tgWorldBulletPhysicsImpl.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
// The C++ Standard Library
#include <stdexcept>
#include <math.h>

//Debug Drawers
#include "GL_ShapeDrawer.h"
#include "LinearMath/btIDebugDraw.h"
#include "GLDebugDrawer.h"
#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"

#define PI 3.14159265

namespace
{
  /**
   * Configuration parameters so they're easily accessable.
   * All parameters must be positive.
   */

  double sf = 30; //scaling factor. Match with all models, app file and controller files
  btVector3 position;
  
  const struct Config
  {
    double density;
    double radius;
    double rod_length; 
    double friction;
    double rollFriction;
    double restitution;
   
  } c =
    {
      
      //Parameters
      100,    // density (kg / length^3) [calculated so 6 rods = 1.5 kg]
      0.10*sf,          // radius (length)
      0.66*sf,         // rod_length (length)
      0.99,             // friction (unitless)
      0.01,             // rollFriction (unitless)
      0.0,              // restitution (?)
    };
} // namespace

MarkerModel::MarkerModel(btVector3 Location) :
  tgModel() 
{
  position = Location;  
}

MarkerModel::~MarkerModel()
{
}


void MarkerModel::setup(tgWorld& world)
{
  // Create a structure that will hold the details of this model
  tgStructure s;

  const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

  // Pylon
  s.addNode(0, 0, 0); // 0
  s.addNode(0, 0.2*sf, 0); // 1
  s.addPair( 0,  1,"rod");
  
  s.move(position);
  
  // Create the build spec that uses tags to turn the structure into a real model
  tgBuildSpec spec;
  spec.addBuilder("rod", new tgRodInfo(rodConfig));

  // Create your structureInfo
  tgStructureInfo structureInfo(s, spec);
  structureInfo.buildInto(*this, world);

  // Notify controllers that setup has finished.
  notifySetup();
    
  // Actually setup the children
  tgModel::setup(world);

  std::cout << "Marker Setup Complete ~~~~~~~~~~~~" << std::endl;
}

void MarkerModel::step(double dt)
{        
      notifyStep(dt);
      tgModel::step(dt);  // Step any children
}


void MarkerModel::onVisit(tgModelVisitor& r)
{
  // Example: m_rod->getRigidBody()->dosomething()...
  tgModel::onVisit(r);
}

    
void MarkerModel::teardown()
{
  notifyTeardown();
  tgModel::teardown();
}













