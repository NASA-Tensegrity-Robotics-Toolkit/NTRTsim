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

  double sf = 30; //scaling factor. Match with App file and Controller file
  btVector3 position;
  
  const struct Config
  {
    double density;
    double radius;
    double stiffness;
    double damping;
    double rod_length; 
    double friction;
    double rollFriction;
    double restitution;
    double pretension;
    bool   hist;
    double maxTens;
    double targetVelocity;
  } c =
    {
      
      //TT3 Parameters
      2990/pow(sf,3),    // density (kg / length^3) [calculated so 6 rods = 1.5 kg]
      10*sf,          // radius (length)
      300.0,           // stiffness (kg / sec^2) was 1500
      20.0,            // damping (kg / sec)
      0.66*sf,         // rod_length (length)
      0.99,             // friction (unitless)
      0.01,             // rollFriction (unitless)
      0.0,              // restitution (?)
      17.5*sf,         // pretension (kg-m/s^2) -> set to 4 * 613, the previous value of the rest length controller
      0,                // History logging (boolean)
      10000*sf,         // maxTens (kg-m/s^2)
      0.25*sf,          // targetVelocity (m/s)
      
      /*
      //Superball Parameters
      688/pow(sf,3),    // density (kg / length^3)
      .031*sf,          // radius (length)
      3500.0,           // stiffness (kg / sec^2) was 1500
      200.0,            // damping (kg / sec)
      1.615*sf,         // rod_length (length)
      0.99,             // friction (unitless)
      0.01,             // rollFriction (unitless)
      0.0,              // restitution (?)
      300.0*sf,         // pretension (kg-m/s^2) -> set to 4 * 613, the previous value of the rest length controller
      0,                // History logging (boolean)
      10000*sf,         // maxTens (kg-m/s^2)
      0.5*sf,          // targetVelocity (m/s) 
      */
    };
} // namespace

MarkerModel::MarkerModel(btVector3 Location) :
  tgModel() 
{
  position = Location;
  //this->gDebugDraw = new GLDebugDrawer();
  
}

MarkerModel::~MarkerModel()
{
}


void MarkerModel::setup(tgWorld& world)
{

  //Get the dynamics world
  //tgWorldImpl& impl = world.implementation();
  //tgWorldBulletPhysicsImpl& bulletWorld = static_cast<tgWorldBulletPhysicsImpl&>(impl);
  //this->btWorld = &bulletWorld.dynamicsWorld();

  // Create a structure that will hold the details of this model
  tgStructure s;

  const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);

  // Pylon
  s.addNode(0, 0, 0); // 0
  s.addNode(0, 10*sf, 0); // 1
  s.addPair( 0,  1,"rod");
  
  //rotateToFace(s, 15);
  s.move(position);
  //s.move(btVector3(100, 1800, -100));
  
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













