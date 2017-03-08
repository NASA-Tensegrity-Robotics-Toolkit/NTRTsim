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

 /**
 * @file MGSingleLeg.cpp
 * @brief Implementing Mountain Goat 
 * Trying to stiffen up leg actuators more, to prevent falling.
 * @author Dawn Hustig-Schultz
 * @date Aug 2016
 * @version 1.1.0
 * $Id$
 */


//This application
#include "MGSingleLeg.h"

// This library
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/tgBasicActuator.h"
#include "tgcreator/tgKinematicActuatorInfo.h"
#include "tgcreator/tgKinematicContactCableInfo.h"
#include "core/tgRod.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <iostream>
#include <stdexcept>

//#define USE_KINEMATIC
//#define PASSIVE_STRUCTURE

MGSingleLeg::MGSingleLeg(int segments, int hips, int legs) :
BaseQuadModelLearning(segments, hips),
m_legs(legs)
{
    m_subStructures = hips + legs;
}

MGSingleLeg::~MGSingleLeg()
{
}

void MGSingleLeg::addNodesLeg(tgStructure& s, double r){ 
    s.addNode(0,0,0); //0: Bottom Center of lower leg segment
    s.addNode(0,r,0);  //1: Center of lower leg segment
    s.addNode(r,r,0); //2: Right of lower leg segment
    s.addNode(-r,r,0);  //3: Left of lower leg segment
    s.addNode(0,2*r,0);  //4: Top of lower leg segment
    s.addNode(0,-r/2,0);  //5: Leg segment extension for connections to foot.

    //Extra nodes, for supporting rod structures on bottom
    s.addNode(0,-r/2,-r/2); //6
    s.addNode(0,-r/2,r/2); //7
    s.addNode(r/2,-r/2,0); //8
    s.addNode(-r/2,-r/2,0); //9 
}

void MGSingleLeg::addRodsLeg(tgStructure& s){
    s.addPair(0,1,"rod");
    s.addPair(1,2,"rod");
    s.addPair(1,3,"rod");
    s.addPair(1,4,"rod");
    s.addPair(0,5,"rod");

    //Extra rods, for support
    s.addPair(5,6,"rod");
    s.addPair(5,7,"rod");
    s.addPair(5,8,"rod");
    s.addPair(5,9,"rod");
}

void MGSingleLeg::addNodesHip(tgStructure& s, double r){
    s.addNode(0,0,0); //Node 0 
    s.addNode(0,r,r); //Node 1 
    s.addNode(0,-r,-r); //Node 2
    s.addNode(0,-r,r); //Node 3
}

void MGSingleLeg::addRodsHip(tgStructure& s){
    s.addPair(0,1,"rod");
    s.addPair(0,2,"rod");
    s.addPair(0,3,"rod");
}

void MGSingleLeg::addNodesVertebra(tgStructure& s, double r){
    s.addNode(0,0,0); //Node 0 
    s.addNode(r,0,r); //Node 1 
    s.addNode(r,0,-r); //Node 2
    s.addNode(-r,0,-r); //Node 3
    s.addNode(-r,0,r); //Node 4
}

void MGSingleLeg::addRodsVertebra(tgStructure& s){
    s.addPair(0,1,"masslessRod");
    s.addPair(0,2,"masslessRod");
    s.addPair(0,3,"masslessRod");
    s.addPair(0,4,"masslessRod");
}

void MGSingleLeg::addSegments(tgStructure& goat, tgStructure& vertebra, tgStructure& hip, tgStructure& leg, double r){ 
    const double offsetDist = r+1; 
    const double offsetDist2 = offsetDist*6; 
    const double offsetDist3 = offsetDist2+2;
    const double yOffset_leg = -(2*r+1); 
    const double yOffset_foot = -(2*r+6);

    //Vertebrae
    btVector3 offset(offsetDist,0.0,0);
    //Hips
    btVector3 offset1(offsetDist*2,0.0,offsetDist);
    btVector3 offset2(offsetDist2,0.0,offsetDist);
    btVector3 offset3(offsetDist*2,0.0,-offsetDist);
    btVector3 offset4(offsetDist2,0.0,-offsetDist);
    //Lower legs
    btVector3 offset5(offsetDist3,yOffset_leg,offsetDist);
    btVector3 offset6(offsetDist3,yOffset_leg,-offsetDist);
    btVector3 offset7(r*2,yOffset_leg,offsetDist);
    btVector3 offset8(r*2,yOffset_leg,-offsetDist);
    //Feet
    btVector3 offset9(offsetDist3+1,yOffset_foot,offsetDist);
    btVector3 offset10(offsetDist3+1,yOffset_foot,-offsetDist);
    btVector3 offset11(r*2+1,yOffset_foot,offsetDist);
    btVector3 offset12(r*2+1,yOffset_foot,-offsetDist);

    for(std::size_t i = 0; i < m_segments; i++) { //Connect segments for spine of goat
        tgStructure* t = new tgStructure (vertebra);
        t->addTags(tgString("spine segment2 num", i + 1));
        t->move((i + 1)*offset);

        if (i % 2 == 1){

            t->addRotation(btVector3((i + 1) * offsetDist, 0.0, 0.0), btVector3(1, 0, 0), 0.0);

        }
        else{

            t->addRotation(btVector3((i + 1) * offsetDist, 0.0, 0.0), btVector3(1, 0, 0), M_PI/2.0);

        }

        goat.addChild(t); //Add a vertebral segment to the goat
    }

	//place a hip on the structure
        tgStructure* t2 = new tgStructure (hip);
        t2->addTags(tgString("segment num", m_segments));
 
        t2->move(offset1);
        t2->addRotation(btVector3(offsetDist*2, 0.0, offsetDist), btVector3(0, 1, 0), M_PI);

        goat.addChild(t2); //Add the hip

        //place lower leg on the hip
        tgStructure* t3 = new tgStructure (leg);
        t3->addTags(tgString("segment num", m_segments + 1));

        t3->move(offset7);
        t3->addRotation(btVector3(r*2, yOffset_leg, offsetDist), btVector3(0, 1, 0), M_PI);

        goat.addChild(t3); //Add lower leg 
}

void MGSingleLeg::addMuscles(tgStructure& goat){ 
    //Time to add the muscles to the structure. 
    //A note about tags: if want to identify a muscle by multiple words, the multiple words will be pulled out in any order,
    //not the order in which they are written. So, put underscores, use camel case, or use unique, individual words to pull out muscles! 
    std::vector<tgStructure*> children = goat.getChildren();
    

    tgNodes n0 = children[0]->getNodes();
    tgNodes n1 = children[1]->getNodes();
    tgNodes n2 = children[2]->getNodes();

    tgNodes n3 = children[3]->getNodes();
    tgNodes n4 = children[4]->getNodes();


	//Muscles from vertebra 0 to vertebra 1. 
	goat.addPair(n0[3], n1[3], tgString("spine front upper right muscleAct1 seg", 0) + tgString(" seg", 1));
	goat.addPair(n0[3], n1[4], tgString("spine front upper left muscleAct1 seg", 0) + tgString(" seg", 1));
	goat.addPair(n0[4], n1[3], tgString("spine front lower right muscleAct2 seg", 0) + tgString(" seg", 1));
	goat.addPair(n0[4], n1[4], tgString("spine front lower left muscleAct2 seg", 0) + tgString(" seg", 1));

	goat.addPair(n0[1], n1[3], tgString("spine rear upper right muscleAct1 seg", 0) + tgString(" seg", 1));
	goat.addPair(n0[1], n1[4], tgString("spine rear upper left muscleAct1 seg", 0) + tgString(" seg", 1));
	goat.addPair(n0[2], n1[3], tgString("spine rear lower right muscleAct2 seg", 0) + tgString(" seg", 1));
	goat.addPair(n0[2], n1[4], tgString("spine rear lower left muscleAct2 seg", 0) + tgString(" seg", 1));


	// Muscles from vertebra 1 to vertebra 2:
        goat.addPair(n1[1], n2[3], tgString("spine front lower left muscleAct2 seg", 1) + tgString(" seg", 2));
        goat.addPair(n1[2], n2[3], tgString("spine front lower right muscleAct2 seg", 1) + tgString(" seg", 2));
        goat.addPair(n1[1], n2[4], tgString("spine front upper left muscleAct1 seg", 1) + tgString(" seg", 2));
        goat.addPair(n1[2], n2[4], tgString("spine front upper right muscleAct1 seg", 1) + tgString(" seg", 2));

        goat.addPair(n1[1], n2[2], tgString("spine rear lower left muscleAct2 seg", 1) + tgString(" seg", 2));
        goat.addPair(n1[2], n2[2], tgString("spine rear lower right muscleAct2 seg", 1) + tgString(" seg", 2));
        goat.addPair(n1[1], n2[1], tgString("spine rear upper left muscleAct1 seg", 1) + tgString(" seg", 2));
        goat.addPair(n1[2], n2[1], tgString("spine rear upper right muscleAct1 seg", 1) + tgString(" seg", 2));

    //Now add muscles to the hip.... 

    //Left shoulder muscles
    goat.addPair(n3[1], n1[1], tgString("all left_shoulder rear upper muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n3[1], n1[4], tgString("all left_shoulder front upper muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n3[1], n0[2], tgString("left_shoulder front top muscleAct1 seg", 7) + tgString(" seg", 0));
    goat.addPair(n3[1], n2[3], tgString("left_shoulder rear top muscleAct1 seg", 7) + tgString(" seg", 2));

    goat.addPair(n3[3], n1[1], tgString("all left_shoulder rear lower muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n3[3], n1[4], tgString("all left_shoulder front lower muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n3[3], n0[1], tgString("left_shoulder front bottom muscleAct1 seg", 7) + tgString(" seg", 0));
    goat.addPair(n3[3], n2[4], tgString("left_shoulder rear bottom muscleAct1 seg", 7) + tgString(" seg", 2));

    //Extra muscles, to move left shoulder forward and back:
    goat.addPair(n3[0], n1[1], tgString("all left_shoulder rear mid muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n3[0], n1[4], tgString("all left_shoulder front mid muscleAct1 seg", 7) + tgString(" seg", 1));

    //Leg/hip connections:

    //Left front leg/shoulder
    goat.addPair(n4[4], n3[3], tgString("all left_foreleg outer bicep muscle seg", 11) + tgString(" seg", 7));
    goat.addPair(n4[4], n3[2], tgString("all left_foreleg inner bicep muscle seg", 11) + tgString(" seg", 7));
    goat.addPair(n4[4], n1[4], tgString("all left_foreleg front abdomen connection muscle seg", 11) + tgString(" seg", 1));
    goat.addPair(n4[3], n1[1], tgString("all left_foreleg frontAbdomen connection muscle3 seg", 11) + tgString(" seg", 1)); //Active
    goat.addPair(n4[2], n1[4], tgString("all left_foreleg rear_abdomen connection muscle3 seg", 11) + tgString(" seg", 1)); //Active

    goat.addPair(n4[3], n3[3], tgString("all left_foreleg outer_tricep muscle seg", 11) + tgString(" seg", 7));
    goat.addPair(n4[3], n3[2], tgString("all left_foreleg inner_tricep muscle seg", 11) + tgString(" seg", 7));

    goat.addPair(n4[2], n3[3], tgString("all left_foreleg outer front tricep muscle seg", 11) + tgString(" seg", 7));
    goat.addPair(n4[2], n3[2], tgString("all left_foreleg inner front tricep muscle seg", 11) + tgString(" seg", 7));

    //Adding muscle to pull up on right front leg:
    goat.addPair(n4[4], n3[1], tgString("all left_foreleg mid_bicep muscle3 seg", 11) + tgString(" seg", 7)); //Active
       
}

void MGSingleLeg::setup(tgWorld& world)
{
    //Rod and Muscle configuration. 
    const double density = 4.2/300.0; //Note: this needs to be high enough or things fly apart...
    const double densityMassless = 0;
    const double radius = 0.5;
    const double rod_space = 10.0;
    const double rod_space2 = 8.0;
    const double friction = 0.5;
    const double rollFriction = 0.0;
    const double restitution = 0.0;

    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);
    const tgRod::Config masslessRodConfig(radius, densityMassless, friction, rollFriction, restitution);

    const double stiffness = 1000.0;
    const double stiffnessPassive = 4000.0; //4000
    const double stiffnessPassive2 = 4000.0;
    const double stiffnessPassive3 = 10000.0;
    const double damping = .01*stiffness;
    const double pretension = 0.0;
    const bool   history = true;
    const double maxTens = 7000.0;
    const double maxSpeed = 12.0;

    const double passivePretension = 1000; 
    const double passivePretension2 = 3500; 
    const double passivePretension3 = 3500; 
    const double passivePretension4 = 4000.0;

#ifdef USE_KINEMATIC

    const double mRad = 1.0;
    const double motorFriction = 10.0;
    const double motorInertia = 1.0;
    const bool backDrivable = false;
    #ifdef PASSIVE_STRUCTURE
        tgKinematicActuator::Config motorConfig(stiffness, 20, passivePretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);
	tgKinematicActuator::Config motorConfigOther(stiffnessPassive, damping, passivePretension2,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 

	tgKinematicActuator::Config motorConfigStomach(stiffnessPassive2, damping, passivePretension4,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 
	tgKinematicActuator::Config motorConfigLegs(stiffnessPassive3, damping, passivePretension3,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);
    #else
        tgKinematicActuator::Config motorConfigSpine(stiffness, damping, pretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 

	tgKinematicActuator::Config motorConfigOther(stiffnessPassive, damping, passivePretension2,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 

	tgKinematicActuator::Config motorConfigStomach(stiffnessPassive2, damping, passivePretension4,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 
	tgKinematicActuator::Config motorConfigLegs(stiffnessPassive3, damping, passivePretension3,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);
    #endif

#else
    
    #ifdef PASSIVE_STRUCTURE
        tgSpringCableActuator::Config muscleConfig(2000, 20, passivePretension);
	tgSpringCableActuator::Config muscleConfigOther(stiffnessPassive, damping, passivePretension2);
	tgSpringCableActuator::Config muscleConfigStomach(stiffnessPassive2, damping, passivePretension4); 
	tgSpringCableActuator::Config muscleConfigLegs(stiffnessPassive, damping, passivePretension3);

    #else
        tgSpringCableActuator::Config muscleConfigSpine(stiffness, damping, pretension, history, maxTens, 2*maxSpeed);
	tgSpringCableActuator::Config muscleConfigOther(stiffnessPassive, damping, passivePretension2, history);
	tgSpringCableActuator::Config muscleConfigStomach(stiffnessPassive2, damping, passivePretension4, history); 
	tgSpringCableActuator::Config muscleConfigLegs(stiffnessPassive, damping, passivePretension3, history);
    #endif

#endif

    //Leg:
    tgStructure leg;
    addNodesLeg(leg,rod_space);
    addRodsLeg(leg);

    //Create the basic unit of the goat
    tgStructure vertebra;
    addNodesVertebra(vertebra,rod_space);
    addRodsVertebra(vertebra);

    //Create the basic unit for the hips/shoulders. 
    tgStructure hip;
    addNodesHip(hip,rod_space);
    addRodsHip(hip);

    //Build the goat
    tgStructure goat;

    const double yOffset_foot = -(2*rod_space+6) - 2;

    addSegments(goat,vertebra,hip,leg,rod_space); //,m_segments,m_hips,m_legs,m_feet

    goat.move(btVector3(0.0,-yOffset_foot,0.0));

    addMuscles(goat); //,m_segments,m_hips,m_legs,m_feet

    std::vector<tgStructure*> children = goat.getChildren();
   
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("masslessRod", new tgRodInfo(masslessRodConfig));

#ifdef USE_KINEMATIC

    #ifdef PASSIVE_STRUCTURE
        spec.addBuilder("muscleAct1", new tgKinematicContactCableInfo(motorConfig));
	spec.addBuilder("muscle ", new tgKinematicContactCableInfo(motorConfigOther));
	spec.addBuilder("muscleAct2 ", new tgKinematicContactCableInfo(motorConfigStomach));
	spec.addBuilder("muscle3 ", new tgKinematicContactCableInfo(motorConfigLegs));
    #else 
	spec.addBuilder("muscleAct1", new tgKinematicContactCableInfo(motorConfigSpine));
	spec.addBuilder("muscle ", new tgKinematicContactCableInfo(motorConfigOther));
	spec.addBuilder("muscleAct2 ", new tgKinematicContactCableInfo(motorConfigStomach));
	spec.addBuilder("muscle3 ", new tgKinematicContactCableInfo(motorConfigLegs));
	
    #endif

#else
    #ifdef PASSIVE_STRUCTURE
   	spec.addBuilder("muscleAct1", new tgBasicActuatorInfo(muscleConfig));
	spec.addBuilder("muscle " , new tgBasicActuatorInfo(muscleConfigOther));
	spec.addBuilder("muscleAct2 " , new tgBasicActuatorInfo(muscleConfigStomach));
	spec.addBuilder("muscle3 " , new tgBasicActuatorInfo(muscleConfigLegs));
    #else 
	spec.addBuilder("muscleAct1" , new tgBasicActuatorInfo(muscleConfigSpine));
	spec.addBuilder("muscle " , new tgBasicActuatorInfo(muscleConfigOther));
	spec.addBuilder("muscleAct2 " , new tgBasicActuatorInfo(muscleConfigStomach));
	spec.addBuilder("muscle3 " , new tgBasicActuatorInfo(muscleConfigLegs));
    #endif
    
#endif


    
    // Create your structureInfo
    tgStructureInfo structureInfo(goat, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    m_allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    m_allSegments = this->find<tgModel> ("segment ");
    
    // Actually setup the children, notify controller that the setup has finished
    BaseQuadModelLearning::setup(world);

    children.clear();
}

void MGSingleLeg::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
	BaseQuadModelLearning::step(dt);
    }
}

void MGSingleLeg::teardown()
{
    BaseQuadModelLearning::teardown();
}
