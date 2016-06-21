/**
 * @file PrismModel.cpp
 * @brief Contains the definition of the members of the class PrismModel.
 * $Id$
 */

// This module
#include "PrismModel.h"
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

// LOCAL FUNCTIONS
/*
  void vertex(btVector3 &v)
  {
  glVertex3d(v.getX(), v.getY(), v.getZ());
  }

  void drawFrame(btTransform &tr)
  {
  const float fSize = 3.f;

  glBegin(GL_LINES);

  // x
  glColor3f(255.f,0,0);
  btVector3 vX = tr*btVector3(fSize,0,0);
  vertex(tr.getOrigin()); vertex(vX);

  // y
  glColor3f(0,255.f,0);
  btVector3 vY = tr*btVector3(0,fSize,0);
  vertex(tr.getOrigin()); vertex(vY);

  // z
  glColor3f(0,0,255.f);
  btVector3 vZ = tr*btVector3(0,0,fSize);
  vertex(tr.getOrigin()); vertex(vZ);

  glEnd();
  }
*/
/**
 * Anonomous namespace so we don't have to declare the config in
 * the header.
 */
namespace
{
  /**
   * Configuration parameters so they're easily accessable.
   * All parameters must be positive.
   */
  const struct Config
  {
    double density;
    double radius;
    double stiffness;
    double damping;
    double rod_length;
    double rod_space;    
    double friction;
    double rollFriction;
    double restitution;
    double pretension;
    bool   hist;
    double maxTens;
    double targetVelocity;
  } c =
    {
      0.688,    // density (kg / length^3)
      0.31,     // radius (length)
      613.0,   // stiffness (kg / sec^2) was 1500
      200.0,    // damping (kg / sec)
      16.84,     // rod_length (length)
      7.5,      // rod_space (length)
      0.99,      // friction (unitless)
      0.01,     // rollFriction (unitless)
      0.0,      // restitution (?)
      2452.0,        // pretension -> set to 4 * 613, the previous value of the rest length controller
      0,         // History logging (boolean)
      100000,   // maxTens
      10000,    // targetVelocity     
    };
} // namespace

PrismModel::PrismModel() :
  tgModel() 
{
  this->btWorld = NULL;
  this->thrusterTransform = NULL;
  this->gDebugDraw = new GLDebugDrawer();

  //Set Desired Orientation Here ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  this->goalAltitude = -15;
  this->goalYaw = 45;
  this->goalVector[0] = sin(goalYaw*M_PI/180);
  this->goalVector[1] = cos((goalAltitude+90)*M_PI/180);
  this->goalVector[2] = cos(goalYaw*M_PI/180);
  this->goalVector = goalVector.normalized();
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

PrismModel::~PrismModel()
{
}

void PrismModel::addRobot(tgStructure& s, int& offset){
  addNodes(s,offset);
  addRods(s,offset);
  addActuators(s,offset);
  //offset += 12;
}

void PrismModel::setup(tgWorld& world)
{

  //Get the dynamics world
  tgWorldImpl& impl = world.implementation();
  tgWorldBulletPhysicsImpl& bulletWorld = static_cast<tgWorldBulletPhysicsImpl&>(impl);
  this->btWorld = &bulletWorld.dynamicsWorld();
    
  /*DEBUG*/
  //this->btWorld->setDebugDrawer(this->gDebugDraw);


  double tankRadius = 1.0;

  // Define the configurations of the rods and strings
  // Note that pretension is defined for this string
  const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
  const tgRod::Config tankConfig(tankRadius, c.density/10.0, c.friction, 
				 c.rollFriction, c.restitution);
  const tgRod::Config linkConfig(c.radius/4.0, 0.0, c.friction, 
				 c.rollFriction, c.restitution);
  const tgRod::Config thrusterConfig(0.2, c.density, c.friction, 
				     c.rollFriction, c.restitution);
  const tgRod::Config gimbalConfig(0.05, c.density, c.friction, 
				   c.rollFriction, c.restitution);

  tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
				       c.maxTens, c.targetVelocity);
  tgBasicActuator::Config tankLinkConfig(c.stiffness/1.5, 100000, c.pretension, c.hist, 
					 c.maxTens, c.targetVelocity);

    
  // Create a structure that will hold the details of this model
  tgStructure s;

  int nPtsExtRing = 48;  //Must be divisible by four
  int nPtsIntRing = 48;  //Must be divisible by four

  double internalRadius = 0.6;
  double externalRadius = 1.0; 
  double tankToOuterRing = 1.0;
  double payloadLength = 1;

  //Incremented by reference assignment in each call below
  int globalOffset = 0;

  /*GIMBAL*/
  addRing(s,externalRadius,nPtsExtRing,globalOffset);
  addRing(s,internalRadius,nPtsIntRing,globalOffset);

  /*TANK (aka PAYLOAD)*/ 
  int baseStartLink = globalOffset; //Save the node position before adding new nodes
  addBottomStructure(s,externalRadius,payloadLength,tankToOuterRing,globalOffset);
    
  /*HINGES*/
  makeLinks(s,externalRadius,internalRadius,tankToOuterRing,tankRadius,nPtsExtRing,globalOffset,baseStartLink); //Adds 4 nodes
    
  /*THRUSTER*/
  int thrusterNode = globalOffset;
  addThruster(s,nPtsExtRing,globalOffset); 

  /*ROBOT*/
  //Save the node position
  int beforeRobot = globalOffset;
  //addRobot(s,globalOffset);
  //addStrings(s,baseStartLink,beforeRobot);

  // Move the structure so it doesn't start in the ground
  s.addRotation(btVector3(0,0,0), btVector3(0,0,1), M_PI);
  s.move(btVector3(0, 13, 0));
        
  // Create the build spec that uses tags to turn the structure into a real model
  tgBuildSpec spec;
  spec.addBuilder("rod", new tgRodInfo(rodConfig));
  spec.addBuilder("tank", new tgRodInfo(tankConfig));
  spec.addBuilder("gimbal", new tgRodInfo(gimbalConfig));
  spec.addBuilder("link", new tgRodInfo(linkConfig));
  spec.addBuilder("thruster", new tgRodInfo(thrusterConfig));
  spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
  spec.addBuilder("string", new tgBasicActuatorInfo(tankLinkConfig));

    
  // Create your structureInfo
  tgStructureInfo structureInfo(s, spec);
  structureInfo.buildInto(*this, world);

  // Get actuators
  allActuators = getAllActuators();

  //Get linking rods
  std::vector<tgRod *> linkingRods = find<tgRod>("link");
  std::cout << "[ ] -> There are " << linkingRods.size() << " links" << std::endl;
    
  //Get links for inner hinge
  tgRod* rod1 = linkingRods[0]; //Outer
  tgRod* rod2 = linkingRods[1]; //Inner
  btRigidBody* rigidbody1 = rod1->getPRigidBody();
  btRigidBody* rigidbody2 = rod2->getPRigidBody();
  altitudeHinge = new btHingeConstraint(*rigidbody2,*rigidbody1,btVector3(0,0,0),btVector3(0,0,0),btVector3(1,0,0),btVector3(1,0,0),false);
  
    
  //Get links for outer hinge
  tgRod* rod3 = linkingRods[2]; //Outer
  tgRod* rod4 = linkingRods[3]; //Inner
  btRigidBody* rigidbody3 = rod3->getPRigidBody();
  btRigidBody* rigidbody4 = rod4->getPRigidBody();
  //Important to set pivot location relative to orientation (i.e. flipping structure required -sign in from of pivot position)vvvvv
  yawHinge = new btHingeConstraint(*rigidbody4,*rigidbody3,btVector3(0,0,0),btVector3(0,-(-externalRadius-tankToOuterRing-tankToOuterRing/2.2),0),btVector3(0,-1,0),btVector3(0,-1,0),false);
    

  //Add the hinge constraints
  //altitudeHinge->setLimit(-M_PI/2,M_PI/2);
  this->btWorld->addConstraint(altitudeHinge); //Inner
  this->btWorld->addConstraint(yawHinge); //Outer
    
    
  //Debug thruster orientation
  std::vector<tgRod *> thrusterParts = PrismModel::find<tgRod>("thruster");
  //Get thruster transform
  tgRod* thrusterRod = thrusterParts[0]; //Outer
  btRigidBody* thrusterRigidBody = thrusterRod->getPRigidBody();
  ThrusterBodies.push_back(thrusterRigidBody);
  this->thrusterTransform = &(btTransform&)thrusterRigidBody->getCenterOfMassTransform();

  //Add Marker to Visualize Thruster Orientation
  btTransform inverseTransform = thrusterRigidBody->getWorldTransform().inverse();
  btVector3 pos = btVector3(0,0,7);
  abstractMarker thrust_dir=abstractMarker(thrusterRigidBody,pos,btVector3(1,.0,.0),thrusterNode); // body, position, color, node number
    
  this->addMarker(thrust_dir); //added thruster orientation marker

  //Remove gravity
  this->btWorld->setGravity(btVector3(0,0,0));
  
  // Notify controllers that setup has finished.
  notifySetup();
    
  // Actually setup the children
  tgModel::setup(world);
}

void PrismModel::step(double dt)
{


  double tolerance = 0.001; //Degree
  double speed = 0.5;

  // Precondition
  if (dt <= 0.0)
    {
      throw std::invalid_argument("dt is not positive");
    }
  else
    {

      //yawHinge->enableAngularMotor(true,1,10);

      //std::cout << 
      double altitudeAngle = altitudeHinge->getHingeAngle()*180.0/M_PI;
      double yawAngle = yawHinge->getHingeAngle()*180.0/M_PI;
      std::cout << "Goal Vector: " << goalVector << std::endl;
      std::cout << "Altitude Angle: " << altitudeAngle << ", Yaw Angle: " << yawAngle << std::endl;

      //std::cout << "Altitude angle: " << altitudeAngle << std::endl;
      //std::cout << "Yaw angle: " << yawAngle << std::endl;

      bool altitudeInPosition = false;
      bool yawInPosition = false;
      double altitude_corr;
      double yaw_corr;

      //Corrected Goal Altitude
      std::vector<tgRod *> tank = PrismModel::find<tgRod>("tank");
      tgRod* tankRod = tank[0]; //Outer
      btRigidBody* tankRigidBody = tankRod->getPRigidBody();
      btTransform world_corr_mat = tankRigidBody->getWorldTransform();
      btMatrix3x3 world_rot_mat = world_corr_mat.getBasis();

      btVector3  sol_vector = world_rot_mat.transpose()*goalVector;
      std::cout << sol_vector << std::endl;
      double beta = asin(-sol_vector.getY());
      double alpha;
      alpha = atan2(sol_vector.getX(),sol_vector.getZ());
      std::cout << "check: " << asin(sol_vector.getX()/cos(beta)) << " " <<  acos(sol_vector.getZ()/cos(beta)) << std::endl;

      //Set altitude
      //double deltaAltitude = goalAltitude - altitude_corr*180/M_PI - altitudeAngle;
      double deltaAltitude = beta*180/M_PI - altitudeAngle;
      //deltaAltitude = deltaAltitude 
      if (deltaAltitude > tolerance || deltaAltitude < -tolerance){
	altitudeHinge->enableAngularMotor(true,deltaAltitude*speed,10);
      }
      else{
	altitudeHinge->enableAngularMotor(true,0.0,10);
	altitudeInPosition = true;
      }

      //Set yaw
      //double deltaYaw = goalYaw - yaw_corr*180/M_PI - yawAngle; //yaw_corr positive because rotation axes are opposing
      double deltaYaw = alpha*180/M_PI - yawAngle;
      if (deltaYaw > tolerance || deltaYaw < -tolerance){
	yawHinge->enableAngularMotor(true,deltaYaw*speed,10);
      }
      else{
	yawHinge->enableAngularMotor(true,0.0,10);
	yawInPosition = true;
      }

      //std::cout << "[ ] Yaw error " << deltaYaw << " Altitude error " << deltaAltitude << std::endl;

      //Actual Thruster Heading
      
      btTransform rotation;
      std::vector<tgRod *> thrusters = PrismModel::find<tgRod>("thruster");
      tgRod* thrusterRod = thrusters[0]; //Outer
      btRigidBody* thrusterRigidBody = thrusterRod->getPRigidBody();
      btQuaternion orientation = thrusterRigidBody->getOrientation();
      rotation.setRotation(orientation);
      btVector3 unit = btVector3(0,0,1);
      unit = rotation*unit;
      //std::cout << "Thruster Heading: " << unit.x() << " " << unit.y() << " " << unit.z() << std::endl << std::endl;;
      std::cout << "Thruster Heading: Altitude - " << acos(unit.y())*180/M_PI << ", Yaw -  " << atan2(unit.z(),-unit.x())*180/M_PI << std::endl << std::endl;;
      /*
        if (yawInPosition && altitudeInPosition){
	goalYaw = std::rand() % 360 - 180;
	goalAltitude = std::rand() % 360 - 180;
	std::cout << "[X] Thruster in position, now aiming at (" << goalYaw << ";" << goalAltitude << ")"<< std::endl;
        }
      */
        
      notifyStep(dt);
      tgModel::step(dt);  // Step any children
    }
}

void PrismModel::onVisit(tgModelVisitor& r)
{
  // Example: m_rod->getRigidBody()->dosomething()...
  tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& PrismModel::getAllActuators() const
{
  return allActuators;
}
    
void PrismModel::teardown()
{
  notifyTeardown();
  tgModel::teardown();
}


void PrismModel::addRing(tgStructure& s, double radius, int nPts, int& pointOffset){
  //Add nPts nodes
  for (int i = 0; i < nPts; i++){
    s.addNode(radius*cos(i*2*PI/nPts),radius*sin(i*2*PI/nPts),0);
  }

  for (int i = pointOffset; i < pointOffset+nPts; i++){
    if (i==pointOffset+nPts-1){
      s.addPair(i,pointOffset,"gimbal");
    }
    else{
      s.addPair(i,i+1,"gimbal");
    }
  }

  pointOffset += nPts;
}

void PrismModel::addBottomStructure(tgStructure& s,double extRadius,double payLength, double tankToOuterRing, int& pointOffset){
  //BCera:6/13/16 - Edited to have the Payload initiate as vertical rod (2 nodes rather than 3)
  
  //Cross structure nodes

  //s.addNode(0,-extRadius-tankToOuterRing,0);//N
  //s.addNode(1,-extRadius-tankToOuterRing,0);//N
  //s.addNode(-1,-extRadius-tankToOuterRing,0);//N
  s.addNode(0,-extRadius-tankToOuterRing,0);//N
  s.addNode(0,-extRadius-payLength-tankToOuterRing,0);//N

  s.addPair(pointOffset,pointOffset+1,"tank");
  //s.addPair(pointOffset,pointOffset+2,"tank");

  pointOffset += 2;
}

void PrismModel::makeLinks(tgStructure& s, double extRadius, double intRadius, double tankToOuterRing, double tankRadius, int pointOffset1, int& pointOffset2, int baseStartLink){
  //BCera - everything is relative to the center of the thruster
  //BCera - if dividend below is 2, rods perfectly touch and are rigidly connected
  double offsetInt = (extRadius-intRadius)/2.2; 
  double offsetOut = (tankToOuterRing)/2.2;
  std::cout << offsetOut << std::endl;
  //offsetOut = 0.2;
  //Inner-Link node for outer ring
  s.addNode(extRadius-offsetInt,0,0); //2N
  //Outer-Link node for inner ring
  s.addNode(intRadius+offsetInt,0,0); //2N+1

  //Outer-Link node for outer ring
  s.addNode(0,-extRadius-offsetOut,0);//2N+2
  //Inner-link node for bottom structure
  s.addNode(0,-extRadius-tankToOuterRing+offsetOut,0);//2N+3

  //Make inner links
  s.addPair(0,pointOffset2,"link"); //pointOffset2 is global count of nodes at start of function call
  s.addPair(pointOffset1,pointOffset2+1,"link"); //pointOffset1 is #ofnodes for external ring

  //Make outer links
  s.addPair(3*pointOffset1/4,pointOffset2+2,"link");
  s.addPair(pointOffset2+3,baseStartLink,"link");

  pointOffset2 += 4;
}


void PrismModel::addThruster(tgStructure& s, int nExt, int& globalOffset){
  //Add thruster
  s.addNode(0,0,-0.5);//Start
  s.addNode(0,0,0);//Middle
  s.addNode(0,0,0.5);//End

  s.addPair(globalOffset,globalOffset+1,"thruster");
  s.addPair(globalOffset+1,globalOffset+2,"thruster");
  s.addPair(globalOffset+1,nExt,"link");

  globalOffset += 3;
}


//FUNCTIONS BELOW HERE ARE TO BUILD THE ROBOT#######################################
void PrismModel::addStrings(tgStructure& s, int baseStartLink, int nPtBeforeRobot){
  
  //Add the strings to hold the gimbal
  s.addPair(baseStartLink, nPtBeforeRobot + 12,  "string");
  s.addPair(baseStartLink, nPtBeforeRobot + 13,  "string");
  s.addPair(baseStartLink + 1, nPtBeforeRobot + 16,  "string");
  s.addPair(baseStartLink + 2, nPtBeforeRobot + 17,  "string");  
}


void PrismModel::addNodes(tgStructure& s, int offset)
{
  const double half_length = c.rod_length / 2;

  s.addNode(-c.rod_space,  -half_length, 0);            // 0 + offset
  s.addNode(-c.rod_space,   half_length, 0);            // 1 + ...

  s.addNode( c.rod_space,  -half_length, 0);            // 2
  s.addNode( c.rod_space,   half_length, 0);            // 3

  s.addNode(0,           -c.rod_space,   -half_length); // 4
  s.addNode(0,           -c.rod_space,    half_length); // 5

  s.addNode(0,            c.rod_space,   -half_length); // 6
  s.addNode(0,            c.rod_space,    half_length); // 7

  s.addNode(-half_length, 0,            c.rod_space);   // 8
  s.addNode( half_length, 0,            c.rod_space);   // 9

  s.addNode(-half_length, 0,           -c.rod_space);   // 10
  s.addNode( half_length, 0,           -c.rod_space);   // 11 + offset

  /*Middle nodes*/

  s.addNode(-c.rod_space,  0, 0);            // 12 + offset

  s.addNode( c.rod_space,  0, 0);            // 13

  s.addNode(0,           -c.rod_space,    0); // 14

  s.addNode(0,            c.rod_space,   0); // 15

  s.addNode(0, 0,            c.rod_space);   // 16

  s.addNode(0, 0,           -c.rod_space);   // 17 + offset
}

void PrismModel::addRods(tgStructure& s, int offset)
{
  s.addPair( offset + 0,  offset + 12, "rod");
  s.addPair( offset + 12,  offset + 1, "rod");

  s.addPair( offset + 2,  offset + 13, "rod");
  s.addPair( offset + 13,  offset + 3, "rod");

  s.addPair( offset + 4,  offset + 14, "rod");
  s.addPair( offset + 14,  offset + 5, "rod");

  s.addPair( offset + 6,  offset + 15, "rod");
  s.addPair( offset + 15,  offset + 7, "rod");

  s.addPair( offset + 8,  offset + 16, "rod");
  s.addPair( offset + 16,  offset + 9, "rod");

  s.addPair( offset + 10, offset + 17, "rod");
  s.addPair( offset + 17, offset + 11, "rod");

}

void PrismModel::addActuators(tgStructure& s, int offset)
{
  s.addPair(offset + 0, offset + 4,  "muscle");
  s.addPair(offset + 0, offset + 5,  "muscle");
  s.addPair(offset + 0, offset + 8,  "muscle");
  s.addPair(offset + 0, offset + 10, "muscle");

  s.addPair(offset + 1, offset + 6,  "muscle");
  s.addPair(offset + 1, offset + 7,  "muscle");
  s.addPair(offset + 1, offset + 8,  "muscle");
  s.addPair(offset + 1, offset + 10, "muscle");

  s.addPair(offset + 2, offset + 4,  "muscle");
  s.addPair(offset + 2, offset + 5,  "muscle");
  s.addPair(offset + 2, offset + 9,  "muscle");
  s.addPair(offset + 2, offset + 11, "muscle");

  s.addPair(offset + 3, offset + 7,  "muscle");
  s.addPair(offset + 3, offset + 6,  "muscle");
  s.addPair(offset + 3, offset + 9,  "muscle");
  s.addPair(offset + 3, offset + 11, "muscle");

  s.addPair(offset + 4, offset + 2,  "muscle");
  s.addPair(offset + 4, offset + 10, "muscle");
  s.addPair(offset + 4, offset + 11, "muscle");

  s.addPair(offset + 5, offset + 8,  "muscle");
  s.addPair(offset + 5, offset + 9,  "muscle");

  s.addPair(offset + 6, offset + 10, "muscle");
  s.addPair(offset + 6, offset + 11, "muscle");

  s.addPair(offset + 7, offset + 8,  "muscle");
  s.addPair(offset + 7, offset + 9,  "muscle");

}












