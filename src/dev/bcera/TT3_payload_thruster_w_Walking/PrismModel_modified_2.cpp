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

namespace
{
  /**
   * Configuration parameters so they're easily accessable.
   * All parameters must be positive.
   */

  double sf = 30; //scaling factor. Match with App file and Controller file
  
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
      .0127/2*sf,          // radius (length)
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

PrismModel::PrismModel() :
  tgModel() 
{
 
  this->btWorld = NULL;
  this->thrusterTransform = NULL;
  this->gDebugDraw = new GLDebugDrawer();

  rodDist = (-c.rod_length + sqrt(pow(c.rod_length,2)+4*pow(c.rod_length,2)))/2;

  // Nodes in the x-z plane
  node0 = btVector3(-rodDist/2, 0, c.rod_length/2); // 0
  node1 = btVector3(-rodDist/2, 0, -c.rod_length/2); // 1
  node2 = btVector3(rodDist/2, 0, -c.rod_length/2); // 2
  node3 = btVector3(rodDist/2, 0, c.rod_length/2); // 3

  // Nodes in the y-z plane
  node4 = btVector3(0, -c.rod_length/2, rodDist/2); // 4
  node5 = btVector3(0, c.rod_length/2, rodDist/2); // 5
  node6 = btVector3(0, c.rod_length/2, -rodDist/2); // 6
  node7 = btVector3(0, -c.rod_length/2, -rodDist/2); // 7

  // Nodes in the x-y plane
  node8 = btVector3(-c.rod_length/2, -rodDist/2, 0); // 8
  node9 = btVector3(-c.rod_length/2, rodDist/2, 0); // 9
  node10 = btVector3(c.rod_length/2, rodDist/2, 0); // 10
  node11 = btVector3(c.rod_length/2, -rodDist/2, 0); // 11

  // Find all edge vectors of closed triangles	// Actuator #
  face0Edge0 = node8 - node4;		// 16
  face0Edge1 = node0 - node8;		// 2
  face0Edge2 = node4 - node0;		// 0

  face2Edge0 = node9 - node0;		// 3
  face2Edge1 = node5 - node9;		// 18
  face2Edge2 = node0 - node5;		// 1

  face5Edge0 = node3 - node4;		// 12
  face5Edge1 = node11 - node3;	// 15
  face5Edge2 = node4 - node11;	// 17

  face7Edge0 = node5 - node3;		// 13
  face7Edge1 = node10 - node5;	// 19
  face7Edge2 = node3 - node10;	// 14

  face8Edge0 = node11 - node7;	// 23
  face8Edge1 = node2 - node11;	// 11
  face8Edge2 = node7 - node2;		// 9

  face10Edge0 = node10 - node2;	// 10
  face10Edge1 = node6 - node10;	// 21
  face10Edge2 = node2 - node6;	// 8

  face13Edge0 = node1 - node7;	// 7
  face13Edge1 = node8 - node1;	// 6
  face13Edge2 = node7 - node8;	// 22

  face15Edge0 = node6 - node1;	// 4
  face15Edge1 = node9 - node6;	// 20
  face15Edge2 = node1 - node9;	// 7

  // Find normal vectors to all faces
  face0Norm = (face0Edge0.cross(face0Edge2)).normalize();
  face1Norm = (face0Edge1.cross(face2Edge0)).normalize();
  face2Norm = (face2Edge0.cross(face2Edge2)).normalize();
  face3Norm = (face7Edge0.cross(face2Edge2)).normalize();
  face4Norm = (face0Edge2.cross(face5Edge0)).normalize();
  face5Norm = (face5Edge0.cross(face5Edge2)).normalize();
  face6Norm = (face7Edge2.cross(face5Edge1)).normalize();
  face7Norm = (face7Edge0.cross(face7Edge2)).normalize();

  face8Norm = (face8Edge0.cross(face8Edge2)).normalize();
  face9Norm = (face8Edge1.cross(face10Edge0)).normalize();
  face10Norm = (face10Edge0.cross(face10Edge2)).normalize();
  face11Norm = (face15Edge0.cross(face10Edge2)).normalize();
  face12Norm = (face8Edge2.cross(face13Edge0)).normalize();
  face13Norm = (face13Edge0.cross(face13Edge2)).normalize();
  face14Norm = (face15Edge2.cross(face13Edge1)).normalize();
  face15Norm = (face15Edge0.cross(face15Edge2)).normalize();

  face16Norm = (face0Edge0.cross(face13Edge2)).normalize();
  face17Norm = (face15Edge1.cross(face2Edge1)).normalize();
  face18Norm = (face7Edge1.cross(face10Edge1)).normalize();
  face19Norm = (face8Edge0.cross(face5Edge2)).normalize();

  // Place all normal vectors into vector
  normalVectors.push_back(face0Norm);
  normalVectors.push_back(face1Norm);
  normalVectors.push_back(face2Norm);
  normalVectors.push_back(face3Norm);
  normalVectors.push_back(face4Norm);
  normalVectors.push_back(face5Norm);
  normalVectors.push_back(face6Norm);
  normalVectors.push_back(face7Norm);
  normalVectors.push_back(face8Norm);
  normalVectors.push_back(face9Norm);
  normalVectors.push_back(face10Norm);
  normalVectors.push_back(face11Norm);
  normalVectors.push_back(face12Norm);
  normalVectors.push_back(face13Norm);
  normalVectors.push_back(face14Norm);
  normalVectors.push_back(face15Norm);
  normalVectors.push_back(face16Norm);
  normalVectors.push_back(face17Norm);
  normalVectors.push_back(face18Norm);
  normalVectors.push_back(face19Norm);
}

PrismModel::~PrismModel()
{
}

void PrismModel::addRobot(tgStructure& s, int& offset, double tankToOuterRing){
  addNodes(s,offset,tankToOuterRing);
  addRods(s,offset);
  addActuators(s,offset);
  offset += 12;
}

void PrismModel::setup(tgWorld& world)
{

  //Get the dynamics world
  tgWorldImpl& impl = world.implementation();
  tgWorldBulletPhysicsImpl& bulletWorld = static_cast<tgWorldBulletPhysicsImpl&>(impl);
  this->btWorld = &bulletWorld.dynamicsWorld();
  
  //TT3 Parameters
  double tankRadius = 0.05*sf;
  double internalRadius = 0.025*sf;//0.05*sf;
  double externalRadius = 0.03*sf;//0.055*sf; 
  double tankToOuterRing = 0.05*sf;
  double payloadLength = 0.05*sf;
  /*
  //Superball Parameters
  double tankRadius = 0.1*sf;
  double internalRadius = 0.06*sf;
  double externalRadius = 0.1*sf; 
  double tankToOuterRing = 0.1*sf;
  double payloadLength = 0.1*sf;
  */
  // Define the configurations of the rods and strings
  // Note that pretension is defined for this string
  const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollFriction, c.restitution);
  //std::cout << "Density Sanity Check: " << c.density << std::endl;
  const tgRod::Config tankConfig(tankRadius, 21645/pow(sf,3), c.friction, //density calculated so tank - 8.5 kg
				 c.rollFriction, c.restitution);
  
  //const tgRod::Config tankConfig(tankRadius, 2/pow(sf,3), c.friction, //density calculated so tank - 8.5 kg
  //				 c.rollFriction, c.restitution);
  const tgRod::Config linkConfig(c.radius/2.0, 0.0, c.friction, 
				 c.rollFriction, c.restitution);
  const tgRod::Config thrusterConfig(0.2, c.density/5, c.friction, 
				     c.rollFriction, c.restitution);
  const tgRod::Config gimbalConfig(0.05, c.density/5, c.friction, 
				   c.rollFriction, c.restitution);

  tgBasicActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension, c.hist, 
				       c.maxTens, c.targetVelocity);
  tgBasicActuator::Config tankLinkConfig(c.stiffness, c.damping, c.pretension*1.5, c.hist, 
					 c.maxTens, c.targetVelocity);

    
  // Create a structure that will hold the details of this model
  tgStructure s;
  
  int nPtsExtRing = 48;  //Must be divisible by four
  int nPtsIntRing = 48;  //Must be divisible by four

  //Incremented by reference assignment in each call below
  int globalOffset = 0;

  /*ROBOT*/
  //Save the node position
  int beforeRobot = globalOffset;
  ///*
  addRobot(s,globalOffset,tankToOuterRing+externalRadius); //**If you comment out the robot constructor, also comment out the string constructor below
  //Rotate Robot to sit on Closed Triangle
  /*
  double L = c.rod_length;
  double t = 1.2213*L/1.9912; //find exact geometric ratio later
  double r = t/(2*sin(36*M_PI/180)); 
  double theta = asin(r/t);
  btVector3 rotationPoint = btVector3(0, 0, 0); // origin
  btVector3 rotationAxis_Z = btVector3(0, 0, 1); 
  double rotationAngle_Z = -asin(2*cos(theta)/sqrt(3));
  btQuaternion rot_corr_Z(rotationAxis_Z,-rotationAngle_Z);
  btVector3 rotationAxis_Y = btVector3(0, 1, 0); 
  double rotationAngle_Y = 18*M_PI/180;
  btQuaternion rot_corr_Y(rotationAxis_Y,-rotationAngle_Y);
  s.addRotation(rotationPoint, rotationAxis_Z, -rotationAngle_Z);
  s.addRotation(rotationPoint, rotationAxis_Y, -rotationAngle_Y);
  */

  btQuaternion rotation = rotateToFace(s, 10);
  rotateNormVectors(rotation);
  /*
  for(int i=0;i<normalVectors.size();i++){
    //normalVectors_rotated.push_back(rotMat*normalVectors[i]);
    normalVectors_rotated[i] = normalVectors_rotated[i].rotate(btVector3(0,1,0),M_PI/6);
  }
  */
  double* COM = returnCOM(s,beforeRobot,12);
  s.move(btVector3(-COM[0], -COM[1]-tankToOuterRing-externalRadius-payloadLength/2, -COM[2]));
  std::cout << "COM: " << *COM << " " << *(COM+1) << " " << *(COM+2) << std::endl;
  //*/
  
  //GIMBAL
  int gimbalStart = globalOffset;
  addRing(s,externalRadius,nPtsExtRing,globalOffset);
  addRing(s,internalRadius,nPtsIntRing,globalOffset);
  std::cout << "After Rings: " << globalOffset << std::endl;
  
  //TANK (aka PAYLOAD) 
  int baseStartLink = globalOffset; //Save the node position before adding new nodes
  addBottomStructure(s,externalRadius,payloadLength,tankToOuterRing,globalOffset);
  std::cout << "After Tank: " << globalOffset << std::endl;
  
  //HINGES
  int hingeStart = globalOffset;
  makeLinks(s,externalRadius,internalRadius,tankToOuterRing,tankRadius,nPtsExtRing,globalOffset,baseStartLink,gimbalStart); //Adds 4 nodes
  std::cout << "After Hinges: " << globalOffset << std::endl;
  
  //THRUSTER
  int thrusterNode = globalOffset;
  addThruster(s,nPtsExtRing,globalOffset,gimbalStart); 
  std::cout << "After Thruster: " << globalOffset << std::endl;
  //Inner Payload Strings
  addStrings(s,baseStartLink,beforeRobot); //**Comment out robot constructor above as well
 
  // Move the structure so it doesn't start in the ground
  //Rotate so that payload faces up
  //btQuaternion norm_rotation = btQuaternion(btVector3(0,0,1), M_PI);
  //rotateNormVectors(norm_rotation);
  //s.addRotation(btVector3(0,0,0), btVector3(0,0,1), M_PI);
  
  s.move(btVector3(0, c.rod_length/1.5, 0));

  //rotateToFace(s, 15);
  s.move(btVector3(100, 700, -100));
  //s.move(btVector3(100, 1800, -100));
  
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
  //Important to set pivot location relative to orientation (i.e. flipping structure required neg. sign in from of pivot position)vvvvv
  yawHinge = new btHingeConstraint(*rigidbody4,*rigidbody3,btVector3(0,0,0),btVector3(0,(-externalRadius-tankToOuterRing-tankToOuterRing/2.2),0),btVector3(0,-1,0),btVector3(0,-1,0),false);

  //Add the hinge constraints
  this->btWorld->addConstraint(altitudeHinge); //Inner
  this->btWorld->addConstraint(yawHinge); //Outer
  //altitudeHinge->setLimit(-M_PI/2+M_PI/4,-M_PI/2-M_PI/4);
  //yawHinge->setLimit(-M_PI,M_PI);

  //Debug thruster orientation
  std::vector<tgRod *> thrusterParts = PrismModel::find<tgRod>("thruster");
  //Get thruster transform
  tgRod* thrusterRod = thrusterParts[0]; //Outer
  btRigidBody* thrusterRigidBody = thrusterRod->getPRigidBody();
  ThrusterBodies.push_back(thrusterRigidBody);
  this->thrusterTransform = &(btTransform&)thrusterRigidBody->getCenterOfMassTransform();


  //Get tank rigidbody for controller
  std::vector<tgRod *> tankParts = PrismModel::find<tgRod>("tank");
  tgRod* tankRod = tankParts[0]; //Outer
  btRigidBody* tankRigidBody = tankRod->getPRigidBody();
  TankBodies.push_back(tankRigidBody);

  //Add Marker to Visualize Thruster Orientation
  btTransform inverseTransform = thrusterRigidBody->getWorldTransform().inverse();
  btVector3 pos = btVector3(0,0,7);
  abstractMarker thrust_dir=abstractMarker(thrusterRigidBody,pos,btVector3(.0,1,.0),thrusterNode); // body, position, color, node number
  //this->addMarker(thrust_dir); //added thruster orientation marker

  
  // Get the rod rigid bodies for controller
  allRods = PrismModel::find<tgRod>("rod");
  
  btRigidBody* rodBody = allRods[0]->getPRigidBody();
  abstractMarker NODE0 = abstractMarker(rodBody,btVector3(0,-c.rod_length/2,0),btVector3(1,0,.0),thrusterNode); // body, position, color, node number
  //this->addMarker(NODE0); //added thruster orientation marker
  
  btRigidBody* rodBody0 = allRods[3]->getPRigidBody();
  abstractMarker NODE7  = abstractMarker(rodBody0,btVector3(0,-c.rod_length/2,0),btVector3(0,0,1),thrusterNode); // body, position, color, node number
  btRigidBody* rodBody1 = allRods[4]->getPRigidBody();
  abstractMarker NODE8 = abstractMarker(rodBody1,btVector3(0,-c.rod_length/2,0),btVector3(.0,1,.0),thrusterNode); // body, position, color, node number
  btRigidBody* rodBody2 = allRods[0]->getPRigidBody();
  abstractMarker NODE1 = abstractMarker(rodBody2,btVector3(0,c.rod_length/2,0),btVector3(.0,0,.1),thrusterNode); // body, position, color, node number
   btRigidBody* rodBody3 = allRods[2]->getPRigidBody();
  abstractMarker NODE4 = abstractMarker(rodBody3,btVector3(0,-c.rod_length/2,0),btVector3(.0,0,1),thrusterNode); // body, position, color, node number
  this->addMarker(NODE0); //added thruster orientation marker
  this->addMarker(NODE8); //added thruster orientation marker
  this->addMarker(NODE4); //added thruster orientation marker

  
  // Get the actuators for controller
  allActuators = PrismModel::find<tgBasicActuator>("muscle");
  
  // Notify controllers that setup has finished.
  notifySetup();
    
  // Actually setup the children
  tgModel::setup(world);

  std::cout << "Model Setup Complete ~~~~~~~~~~~~" << std::endl;
}

void PrismModel::step(double dt)
{        
      notifyStep(dt);
      tgModel::step(dt);  // Step any children
}


void PrismModel::onVisit(tgModelVisitor& r)
{
  // Example: m_rod->getRigidBody()->dosomething()...
  tgModel::onVisit(r);
}

std::vector<tgBasicActuator*>& PrismModel::getAllActuators() 
{
  return allActuators;
}

std::vector<tgRod*>& PrismModel::getAllRods()
{
	return allRods;
}

std::vector<btRigidBody*>& PrismModel::getTank()
{
	return TankBodies;
}

std::vector<btVector3>& PrismModel::getNormVects()
{
	return normalVectors_rotated;
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
  
  s.addNode(0,-extRadius-tankToOuterRing,0);//N
  s.addNode(0,-extRadius-payLength-tankToOuterRing,0);//N

  s.addPair(pointOffset,pointOffset+1,"tank");

  pointOffset += 2;
}

void PrismModel::makeLinks(tgStructure& s, double extRadius, double intRadius, double tankToOuterRing, double tankRadius, int pointOffset1, int& pointOffset2, int baseStartLink, int gimbalStart){
  //BCera - everything is relative to the center of the thruster
  //BCera - if dividend below is 2, rods perfectly touch and are rigidly connected
  double offsetInt = (extRadius-intRadius)/2.2; 
  double offsetOut = (tankToOuterRing)/2.2;
  std::cout << offsetOut << std::endl;

  //Inner-Link node for outer ring
  s.addNode(extRadius-offsetInt,0,0); //2N
  //Outer-Link node for inner ring
  s.addNode(intRadius+offsetInt,0,0); //2N+1

  //Outer-Link node for outer ring
  s.addNode(0,-extRadius-offsetOut,0);//2N+2
  //Inner-link node for bottom structure
  s.addNode(0,-extRadius-tankToOuterRing+offsetOut,0);//2N+3

  //Make inner links
  s.addPair(gimbalStart,pointOffset2,"link"); //pointOffset2 is global count of nodes at start of function call
  s.addPair(gimbalStart+pointOffset1,pointOffset2+1,"link"); //pointOffset1 is #ofnodes for external ring

  //Make outer links
  s.addPair(gimbalStart+3*pointOffset1/4,pointOffset2+2,"link");
  s.addPair(pointOffset2+3,baseStartLink,"link");

  pointOffset2 += 4;
}


void PrismModel::addThruster(tgStructure& s, int nExt, int& globalOffset, int gimbalStart){
  //Add thruster
  s.addNode(0,0,-0.5);//Start
  s.addNode(0,0,0);//Middle
  s.addNode(0,0,0.5);//End

  s.addPair(globalOffset,globalOffset+1,"thruster");
  s.addPair(globalOffset+1,globalOffset+2,"thruster");
  s.addPair(globalOffset+1,gimbalStart+nExt,"link");

  globalOffset += 3;
}


//FUNCTIONS BELOW HERE ARE TO BUILD THE ROBOT#######################################
void PrismModel::addStrings(tgStructure& s, int baseStartLink, int nPtBeforeRobot){
  
  //Add the strings to hold the gimbal
  // inner cables
  s.addPair( 0+nPtBeforeRobot,  baseStartLink, "string"); //24
  s.addPair( 4+nPtBeforeRobot,  baseStartLink, "string"); //25
  s.addPair( 8+nPtBeforeRobot,  baseStartLink, "string"); //26
  s.addPair( 6+nPtBeforeRobot, baseStartLink+1, "string"); //27
  s.addPair( 10+nPtBeforeRobot,  baseStartLink+1, "string"); //28
  s.addPair( 2+nPtBeforeRobot,  baseStartLink+1, "string"); //29 
}


void PrismModel::addNodes(tgStructure& s, int offset, double tankToOuterRing)
{
  /*
  double L = c.rod_length; //already scaled
  double t = 1.2213*L/1.9912; //find exact geometric ratio later
  double r = t/(2*sin(36*M_PI/180)); 
  double theta = asin(r/t)*180/M_PI;
  double m = sqrt(pow(t,2)+pow((2*t*sin(54*M_PI/180)),2)); //tip to tip of reg. icosahedron
  double var = m-t*cos(theta*M_PI/180);
  //Values below copied from Icosahderon_w_payload.m, with Y_ntrt = Z_matlab  and Z_ntrt = -Y_matlab
  //Coordinate transformation necessary as NTRT height direction is (0,1,0) Y unit vector
  s.addNode(0, -m/2-tankToOuterRing, 0);  // 0
  s.addNode(0, var-m/2-tankToOuterRing, r);  // 1
  s.addNode(0, t*cos(theta*M_PI/180)-m/2-tankToOuterRing, -r);  // 2
  s.addNode(0, m-m/2-tankToOuterRing, 0); // 3
  s.addNode(-r*cos(18*M_PI/180),   t*cos(theta*M_PI/180)-m/2-tankToOuterRing,    -r*sin(18*M_PI/180));   // 4
  s.addNode(r*cos(18*M_PI/180),   t*cos(theta*M_PI/180)-m/2-tankToOuterRing,    -r*sin(18*M_PI/180));  // 5
  s.addNode(-r*cos(18*M_PI/180),   var-m/2-tankToOuterRing,    r*sin(18*M_PI/180));  // 6
  s.addNode(r*cos(18*M_PI/180),   var-m/2-tankToOuterRing,    r*sin(18*M_PI/180));  // 7
  s.addNode(-r*sin(36*M_PI/180),   t*cos(theta*M_PI/180)-m/2-tankToOuterRing,    r*cos(36*M_PI/180));  // 8
  s.addNode(-r*sin(36*M_PI/180),   var-m/2-tankToOuterRing,    -r*cos(36*M_PI/180));  // 9
  s.addNode(r*sin(36*M_PI/180),   t*cos(theta*M_PI/180)-m/2-tankToOuterRing,    r*cos(36*M_PI/180));  // 10
  s.addNode(r*sin(36*M_PI/180),   var-m/2-tankToOuterRing,    -r*cos(36*M_PI/180));  // 11
  */

  // Calculate the space between two parallel rods based on the rod length from Config
  double rodSpace = (-c.rod_length + sqrt(pow(c.rod_length,2)+4*pow(c.rod_length,2)))/2;

  // Nodes in the x-z plane
  s.addNode(-rodSpace/2, 0, c.rod_length/2); // 0
  s.addNode(-rodSpace/2, 0, -c.rod_length/2); // 1
  s.addNode(rodSpace/2, 0, -c.rod_length/2); // 2
  s.addNode(rodSpace/2, 0, c.rod_length/2); // 3

  // Nodes in the y-z plane
  s.addNode(0, -c.rod_length/2, rodSpace/2); // 4
  s.addNode(0, c.rod_length/2, rodSpace/2); // 5
  s.addNode(0, c.rod_length/2, -rodSpace/2); // 6
  s.addNode(0, -c.rod_length/2, -rodSpace/2); // 7

  // Nodes in the x-y plane
  s.addNode(-c.rod_length/2, -rodSpace/2, 0); // 8
  s.addNode(-c.rod_length/2, rodSpace/2, 0); // 9
  s.addNode(c.rod_length/2, rodSpace/2, 0); // 10
  s.addNode(c.rod_length/2, -rodSpace/2, 0); // 11

}

double* PrismModel::returnCOM(tgStructure &s, int firstNode, int numberofNodes){
  //Returns COM of rods (assumes mass of each rod is identical)

  tgNodes rodNodes = s.getNodes();    
  
  static double rodNodesCOM[3] = {0.0, 0.0, 0.0};
  for ( size_t i = firstNode; i < firstNode + numberofNodes; i++ )
    {
      rodNodesCOM[0] += rodNodes[i][0];
      rodNodesCOM[1] += rodNodes[i][1];
      rodNodesCOM[2] += rodNodes[i][2];
    }
  rodNodesCOM[0] /= 12;
  rodNodesCOM[1] /= 12;
  rodNodesCOM[2] /= 12;

  return rodNodesCOM;
}

void PrismModel::addRods(tgStructure& s, int offset)
{
  /*
  s.addPair( 0+offset,  1+offset, "rod");
  s.addPair( 2+offset,  3+offset, "rod");
  s.addPair( 4+offset,  5+offset, "rod");
  s.addPair( 6+offset,  7+offset, "rod");
  s.addPair( 8+offset,  9+offset, "rod");
  s.addPair( 10+offset, 11+offset, "rod");
  */
  s.addPair(0+offset, 1+offset, "rod"); // 0
  s.addPair(3+offset, 2+offset, "rod"); // 1
  s.addPair(4+offset, 5+offset, "rod"); // 2
  s.addPair(7+offset, 6+offset, "rod"); // 3
  s.addPair(8+offset, 11+offset, "rod"); // 4
  s.addPair(9+offset, 10+offset, "rod"); // 5
  
}

void PrismModel::addActuators(tgStructure& s, int offset)
{
  /*
  // outer cables	
  s.addPair( 0+offset, 4+offset,  "muscle");	//0	
  s.addPair( 0+offset, 5+offset,  "muscle");	//1
  s.addPair( 0+offset, 8+offset,  "muscle");	//2
  s.addPair( 0+offset, 10+offset, "muscle");	//3	
  s.addPair( 1+offset, 8+offset,  "muscle");	//4
  s.addPair( 1+offset, 10+offset, "muscle");	//5
  s.addPair( 1+offset, 6+offset,  "muscle");	//6
  s.addPair( 1+offset, 7+offset,  "muscle");	//7
  s.addPair( 2+offset, 4+offset,  "muscle");	//8
  s.addPair( 2+offset, 5+offset,  "muscle");	//9
  s.addPair( 2+offset, 9+offset,  "muscle");	//10
  s.addPair( 2+offset, 11+offset, "muscle");	//11
  s.addPair( 3+offset, 6+offset,  "muscle");	//12
  s.addPair( 3+offset, 7+offset,  "muscle");	//13
  s.addPair( 3+offset, 9+offset,  "muscle");	//14
  s.addPair( 3+offset, 11+offset, "muscle");	//15
  s.addPair( 4+offset, 8+offset,  "muscle");	//16
  s.addPair( 4+offset, 9+offset,  "muscle");	//17
  s.addPair( 5+offset, 10+offset, "muscle");	//18
  s.addPair( 5+offset, 11+offset, "muscle");	//19
  s.addPair( 6+offset, 8+offset,  "muscle");	//20
  s.addPair( 6+offset, 9+offset,  "muscle");	//21
  s.addPair( 7+offset, 10+offset, "muscle");	//22
  s.addPair( 7+offset, 11+offset, "muscle");    //23
  */

  s.addPair(0+offset, 4+offset, "muscle"); // 0
  s.addPair(0+offset, 5+offset, "muscle"); // 1
  s.addPair(0+offset, 8+offset, "muscle"); // 2
  s.addPair(0+offset, 9+offset, "muscle"); // 3

  s.addPair(1+offset, 6+offset, "muscle"); // 4
  s.addPair(1+offset, 7+offset, "muscle"); // 5
  s.addPair(1+offset, 8+offset, "muscle"); // 6
  s.addPair(1+offset, 9+offset, "muscle"); // 7

  s.addPair(2+offset, 6+offset, "muscle"); // 8
  s.addPair(2+offset, 7+offset, "muscle"); // 9
  s.addPair(2+offset, 10+offset, "muscle"); // 10
  s.addPair(2+offset, 11+offset, "muscle"); // 11

  s.addPair(3+offset, 4+offset, "muscle"); // 12
  s.addPair(3+offset, 5+offset, "muscle"); // 13
  s.addPair(3+offset, 10+offset, "muscle"); // 14
  s.addPair(3+offset, 11+offset, "muscle"); // 15

  s.addPair(4+offset, 8+offset, "muscle"); // 16
  s.addPair(4+offset, 11+offset, "muscle"); // 17

  s.addPair(5+offset, 9+offset, "muscle"); // 18
  s.addPair(5+offset, 10+offset, "muscle"); // 19

  s.addPair(6+offset, 9+offset, "muscle"); // 20
  s.addPair(6+offset, 10+offset, "muscle"); // 21

  s.addPair(7+offset, 8+offset, "muscle"); // 22
  s.addPair(7+offset, 11+offset, "muscle"); // 23

}

btQuaternion PrismModel::rotateToFace(tgStructure& s, int face)
{
	btVector3 faceNorm = normalVectors[face];
	btVector3 goalDir = btVector3(0, -1, 0);
	double dotProd = faceNorm.dot(goalDir);
	double theta = acos(dotProd);
	btVector3 crossProd = faceNorm.cross(goalDir);
	btQuaternion rotation = btQuaternion (crossProd, theta);
	std::cout << "crosProd1: " << crossProd << std::endl;
	std::cout << "theta1: " << theta << std::endl; 
	s.addRotation(btVector3(0,0,0), crossProd, theta);

	return rotation;
}

void PrismModel::rotateNormVectors(btQuaternion rotation)
{
  btVector3 axis = rotation.getAxis();
  btScalar theta = rotation.getAngle();
  for(int i=0;i<normalVectors.size();i++){
    //normalVectors_rotated.push_back(rotMat*normalVectors[i]);
    normalVectors_rotated.push_back(normalVectors[i].rotate(axis,theta));
    //std::cout << normalVectors[i] << std::endl;
  }
}

void PrismModel::changeRobotState(int state)
{
  robotState = state;
}
