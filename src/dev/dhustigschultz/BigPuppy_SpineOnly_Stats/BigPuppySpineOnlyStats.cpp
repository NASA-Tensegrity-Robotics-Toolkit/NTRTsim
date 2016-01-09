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
 * @file BigPuppySpineOnlyStats.cpp
 * @brief Implementing the spine of a quadruped based off the Flemons BigPuppy model.
 * @author Dawn Hustig-Schultz
 * @date July 2015
 * @version 1.1.0
 * $Id$
 */


//This application
#include "BigPuppySpineOnlyStats.h"

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

BigPuppySpineOnlyStats::BigPuppySpineOnlyStats(int segments, int hips) :
BaseQuadModelLearning(segments, hips)
{
    
}

BigPuppySpineOnlyStats::~BigPuppySpineOnlyStats()
{
}

void BigPuppySpineOnlyStats::addNodesHip(tgStructure& s, double r){
    s.addNode(0,0,0); //Node 0 
    s.addNode(0,r,r); //Node 1 
    s.addNode(0,-r,-r); //Node 2
    s.addNode(0,-r,r); //Node 3
    s.addNode(0,-r-2,-r-2); //node 4
    //s.addNode(0,-r-2,r+2); //node 5
}

void BigPuppySpineOnlyStats::addRodsHip(tgStructure& s){
    s.addPair(0,1,"rod");
    s.addPair(0,2,"rod");
    s.addPair(0,3,"rod");
    s.addPair(2,4,"rod");
    //s.addPair(3,5,"rod");
}

void BigPuppySpineOnlyStats::addNodesVertebra(tgStructure& s, double r){
    s.addNode(0,0,0); //Node 0 
    s.addNode(r,0,r); //Node 1 
    s.addNode(r,0,-r); //Node 2
    s.addNode(-r,0,-r); //Node 3
    s.addNode(-r,0,r); //Node 4
}

void BigPuppySpineOnlyStats::addRodsVertebra(tgStructure& s){
    s.addPair(0,1,"rod");
    s.addPair(0,2,"rod");
    s.addPair(0,3,"rod");
    s.addPair(0,4,"rod");
}

void BigPuppySpineOnlyStats::addSegments(tgStructure& puppy, tgStructure& vertebra, tgStructure& hip, double r){ 
    const double offsetDist = r+1; 
    const double offsetDist2 = offsetDist*6; 
    const double offsetDist3 = offsetDist2+2;

    //Vertebrae
    btVector3 offset(offsetDist,0.0,0);
    //Hips
    btVector3 offset1(offsetDist*2,0.0,offsetDist);
    btVector3 offset2(offsetDist2,0.0,offsetDist);
    btVector3 offset3(offsetDist*2,0.0,-offsetDist);
    btVector3 offset4(offsetDist2,0.0,-offsetDist);


    for(std::size_t i = 0; i < m_segments; i++) { //Connect segments for spine of puppy
        tgStructure* t = new tgStructure (vertebra);
        t->addTags(tgString("spine segment num", i + 1));
        t->move((i + 1)*offset);

        if (i % 2 == 1){

            t->addRotation(btVector3((i + 1) * offsetDist, 0.0, 0.0), btVector3(1, 0, 0), 0.0);

        }
        else{

            t->addRotation(btVector3((i + 1) * offsetDist, 0.0, 0.0), btVector3(1, 0, 0), M_PI/2.0);

        }

        puppy.addChild(t); //Add a segment to the puppy
    }

     for(std::size_t i = m_segments; i < (m_segments + 2); i++) {//deal with right hip and shoulder first
        tgStructure* t = new tgStructure (hip);
        t->addTags(tgString("segment num", i + 1));
 
        if(i % 2 == 0){
            t->move(offset2);
            t->addRotation(btVector3(offsetDist2, 0.0, offsetDist), btVector3(0, 1, 0), M_PI);

        }
        else{
            t->move(offset1);
            t->addRotation(btVector3(offsetDist*2, 0.0, offsetDist), btVector3(0, 1, 0), M_PI);
        }

        puppy.addChild(t); //Add a segment to the puppy
    }

    for(std::size_t i = (m_segments + 2); i < (m_segments + m_hips); i++) {//deal with left hip and shoulder now
        tgStructure* t = new tgStructure (hip);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset4);
        }
        else{
            t->move(offset3);
        }

        puppy.addChild(t); //Add a segment to the puppy

    }
}

void BigPuppySpineOnlyStats::addMuscles(tgStructure& puppy){ 
        //Time to add the muscles to the structure. Todo: try to clean this up some more.
    std::vector<tgStructure*> children = puppy.getChildren();
    for(std::size_t i = 2; i < (children.size() - (m_hips)); i++) { 

        tgNodes n0 = children[i-2]->getNodes();
        tgNodes n1 = children[i-1]->getNodes();
        tgNodes n2 = children[i]->getNodes();


        if(i==2){
            //Extra muscles, to keep front vertebra from swinging. 
            puppy.addPair(n0[3], n1[3], tgString("spine front upper right muscleAct seg", i-2) + tgString(" seg", i-1));
            puppy.addPair(n0[3], n1[4], tgString("spine front upper left muscleAct seg", i-2) + tgString(" seg", i-1));

            puppy.addPair(n0[4], n1[3], tgString("spine front lower right muscleAct seg", i-2) + tgString(" seg", i-1));
            puppy.addPair(n0[4], n1[4], tgString("spine front lower left muscleAct seg", i-2) + tgString(" seg", i-1));


        }        

        //Add muscles to the puppy
        if(i < 3){
            if(i % 2 == 0){ //front
                puppy.addPair(n0[1], n1[3], tgString("spine front lower right muscleAct seg", i-2) + tgString(" seg", i-1));
                puppy.addPair(n0[1], n1[4], tgString("spine front lower left muscleAct seg", i-2) + tgString(" seg", i-1));
                puppy.addPair(n0[2], n1[3], tgString("spine front upper right muscleAct seg", i-2) + tgString(" seg", i-1));
                puppy.addPair(n0[2], n1[4], tgString("spine front upper left muscleAct seg", i-2) + tgString(" seg", i-1));
            }
            else{ //rear
                puppy.addPair(n0[1], n1[3], tgString("spine rear upper left muscleAct seg", i-2) + tgString(" seg", i-1));
                puppy.addPair(n0[1], n1[4], tgString("spine rear lower left muscleAct seg", i-2) + tgString(" seg", i-1));
                puppy.addPair(n0[2], n1[3], tgString("spine rear upper right muscleAct seg", i-2) + tgString(" seg", i-1));
                puppy.addPair(n0[2], n1[4], tgString("spine rear lower right muscleAct seg", i-2) + tgString(" seg", i-1)); 
            }
        }
        if(i < 7){//Was 6
            if(i % 2 == 0){
                puppy.addPair(n0[1], n2[4], tgString("spine2 bottom muscleAct seg", i-2) + tgString(" seg", i-1));
                puppy.addPair(n0[2], n2[3], tgString("spine2 top muscleAct seg", i-2) + tgString(" seg", i-1));
            }
            else{
                puppy.addPair(n0[1], n2[4], tgString("spine2 lateral left muscleAct seg", i-2) + tgString(" seg", i-1));
                puppy.addPair(n0[2], n2[3], tgString("spine2 lateral right muscleAct seg", i-2) + tgString(" seg", i-1));

            }
        }
        if(i > 0 && i < 7){
            if(i % 2 == 0){//rear
                puppy.addPair(n1[1], n2[3], tgString("spine rear upper left muscleAct seg", i-1) + tgString(" seg", i));
                puppy.addPair(n1[1], n2[4], tgString("spine rear lower left muscleAct seg", i-1) + tgString(" seg", i));
                puppy.addPair(n1[2], n2[3], tgString("spine rear upper right muscleAct seg", i-1) + tgString(" seg", i));
                puppy.addPair(n1[2], n2[4], tgString("spine rear lower right muscleAct seg", i-1) + tgString(" seg", i));
            }
            else{//front

                puppy.addPair(n1[1], n2[3], tgString("spine front lower right muscleAct seg", i-1) + tgString(" seg", i));
                puppy.addPair(n1[1], n2[4], tgString("spine front lower left muscleAct seg", i-1) + tgString(" seg", i));
                puppy.addPair(n1[2], n2[3], tgString("spine front upper right muscleAct seg", i-1) + tgString(" seg", i));
                puppy.addPair(n1[2], n2[4], tgString("spine front upper left muscleAct seg", i-1) + tgString(" seg", i));
            }
        }
        if(i == 6){
            //rear
            puppy.addPair(n1[1], n2[2], tgString("spine rear lower left muscleAct seg", i-1) + tgString(" seg", i));
            puppy.addPair(n1[2], n2[2], tgString("spine rear lower right muscleAct seg", i-1) + tgString(" seg", i));
            puppy.addPair(n1[1], n2[1], tgString("spine rear upper left muscleAct seg", i-1) + tgString(" seg", i));
            puppy.addPair(n1[2], n2[1], tgString("spine rear upper right muscleAct seg", i-1) + tgString(" seg", i));  
        }
        
    }


    //Now add muscles to hips.... 
    tgNodes n0 = children[0]->getNodes();
    tgNodes n1 = children[1]->getNodes();
    tgNodes n2 = children[2]->getNodes();
    tgNodes n3 = children[3]->getNodes();
    tgNodes n4 = children[4]->getNodes();
    tgNodes n5 = children[5]->getNodes();
    tgNodes n6 = children[6]->getNodes();
    tgNodes n7 = children[7]->getNodes();
    tgNodes n8 = children[8]->getNodes();
    tgNodes n9 = children[9]->getNodes();
    tgNodes n10 = children[10]->getNodes();

    
    //Left shoulder muscles
    puppy.addPair(n7[1], n1[1], tgString("left shoulder rear upper muscleAct seg", 6) + tgString(" seg", 1));
    puppy.addPair(n7[1], n1[4], tgString("left shoulder front upper muscleAct seg", 6) + tgString(" seg", 1));
    puppy.addPair(n7[1], n0[2], tgString("left shoulder front top muscleAct seg", 6) + tgString(" seg", 0));
    puppy.addPair(n7[1], n2[3], tgString("left shoulder rear top muscleAct seg", 6) + tgString(" seg", 2));

    puppy.addPair(n7[3], n1[1], tgString("left shoulder rear lower muscleAct seg", 6) + tgString(" seg", 1));
    puppy.addPair(n7[3], n1[4], tgString("left shoulder front lower muscleAct seg", 6) + tgString(" seg", 1));
    puppy.addPair(n7[3], n0[1], tgString("left shoulder front bottom muscleAct seg", 6) + tgString(" seg", 0));
    puppy.addPair(n7[3], n2[4], tgString("left shoulder rear bottom muscleAct seg", 6) + tgString(" seg", 2));

    //Extra muscles, to move left shoulder forward and back:
    puppy.addPair(n7[0], n1[1], tgString("left shoulder rear mid muscleAct seg", 6) + tgString(" seg", 1));
    puppy.addPair(n7[0], n1[4], tgString("left shoulder front mid muscleAct seg", 6) + tgString(" seg", 1));

    //Left hip muscles
    puppy.addPair(n8[1], n5[1], tgString("left hip rear upper muscleAct seg", 7) + tgString(" seg", 5));
    puppy.addPair(n8[1], n5[4], tgString("left hip front upper muscleAct seg", 7) + tgString(" seg", 5));
    puppy.addPair(n8[1], n4[2], tgString("left hip front top muscleAct seg", 7) + tgString(" seg", 4));
    puppy.addPair(n8[1], n6[3], tgString("left hip rear top muscleAct seg", 7) + tgString(" seg", 4));

    puppy.addPair(n8[3], n5[1], tgString("left hip rear lower muscleAct seg", 7) + tgString(" seg", 5));
    puppy.addPair(n8[3], n5[4], tgString("left hip front lower muscleAct seg", 7) + tgString(" seg", 5));
    puppy.addPair(n8[3], n4[1], tgString("left hip front bottom muscleAct seg", 7) + tgString(" seg", 4));
    puppy.addPair(n8[3], n6[4], tgString("left hip front bottom muscleAct seg", 7) + tgString(" seg", 6));

    //Extra muscles, to move left hip forward and back:
    puppy.addPair(n8[0], n5[1], tgString("left hip rear mid muscleAct seg", 7) + tgString(" seg", 3)); //could also be n3[3]
    puppy.addPair(n8[0], n5[4], tgString("left hip front mid muscleAct seg", 7) + tgString(" seg", 5));

    //Right shoulder muscles
    puppy.addPair(n9[1], n1[2], tgString("right shoulder rear upper muscleAct seg", 8) + tgString(" seg", 1));
    puppy.addPair(n9[1], n1[3], tgString("right shoulder front upper muscleAct seg", 8) + tgString(" seg", 1));
    puppy.addPair(n9[1], n0[2], tgString("right shoulder front top muscleAct seg", 8) + tgString(" seg", 0));
    puppy.addPair(n9[1], n2[3], tgString("right shoulder rear top muscleAct seg", 8) + tgString(" seg", 2));

    puppy.addPair(n9[3], n1[2], tgString("right shoulder rear lower muscleAct seg", 8) + tgString(" seg", 1));
    puppy.addPair(n9[3], n1[3], tgString("right shoulder front lower muscleAct seg", 8) + tgString(" seg", 1));
    puppy.addPair(n9[3], n0[1], tgString("right shoulder front bottom muscleAct seg", 8) + tgString(" seg", 0));
    puppy.addPair(n9[3], n2[4], tgString("right shoulder rear bottom muscleAct seg", 8) + tgString(" seg", 2));

    //Extra muscles, to move right shoulder forward and back:
    puppy.addPair(n9[0], n1[2], tgString("right shoulder rear mid muscleAct seg", 8) + tgString(" seg", 1));
    puppy.addPair(n9[0], n1[3], tgString("right shoulder front mid muscleAct seg", 8) + tgString(" seg", 1));

    //Right hip muscles
    puppy.addPair(n10[1], n5[2], tgString("right hip rear upper muscleAct seg", 9) + tgString(" seg", 5));
    puppy.addPair(n10[1], n5[3], tgString("right hip front upper muscleAct seg", 9) + tgString(" seg", 5));
    puppy.addPair(n10[1], n4[2], tgString("right hip front top muscleAct seg", 9) + tgString(" seg", 4));
    puppy.addPair(n10[1], n6[3], tgString("right hip rear top muscleAct seg", 9) + tgString(" seg", 4));

    puppy.addPair(n10[3], n5[2], tgString("right hip rear lower muscleAct seg", 9) + tgString(" seg", 5));
    puppy.addPair(n10[3], n5[3], tgString("right hip front lower muscleAct seg", 9) + tgString(" seg", 5));
    puppy.addPair(n10[3], n4[1], tgString("right hip bottom muscleAct seg", 9) + tgString(" seg", 4));  
    puppy.addPair(n10[3], n6[4], tgString("right hip bottom muscleAct seg", 9) + tgString(" seg", 6));  

    //Extra muscles, to move right hip forward and back:
    puppy.addPair(n10[0], n5[2], tgString("right hip rear mid muscleAct seg", 9) + tgString(" seg", 3)); //could also be n3[3]
    puppy.addPair(n10[0], n5[3], tgString("right hip front mid muscleAct seg", 9) + tgString(" seg", 5));

}

void BigPuppySpineOnlyStats::setup(tgWorld& world)
{
    //Rod and Muscle configuration. 
    const double density = 4.2/300.0; //Note: this needs to be high enough or things fly apart...
    const double radius = 0.5;
    const double rod_space = 10.0;
    const double rod_space2 = 8.0;
    const double friction = 0.5;
    const double rollFriction = 0.0;
    const double restitution = 0.0;

    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);

    const double stiffness = 1000.0;
    const double damping = .01*stiffness;
    const double pretension = 0.0;
    const bool   history = true;
    const double maxTens = 7000.0;
    const double maxSpeed = 12.0;

    const double passivePretension = 1000; // 5 N. Todo: make this a #ifdef

#ifdef USE_KINEMATIC

    const double mRad = 1.0;
    const double motorFriction = 10.0;
    const double motorInertia = 1.0;
    const bool backDrivable = false;
    #ifdef PASSIVE_STRUCTURE
        tgKinematicActuator::Config motorConfig(2000, 20, passivePretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);
    #else
        tgKinematicActuator::Config motorConfigSpine(stiffness, damping, pretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 

	tgKinematicActuator::Config motorConfigOther(2000, damping*2, passivePretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 
    #endif

#else
    
    #ifdef PASSIVE_STRUCTURE
        tgSpringCableActuator::Config muscleConfig(2000, 20, passivePretension);
    #else
        tgSpringCableActuator::Config muscleConfigSpine(stiffness, damping, pretension, history, maxTens, 2*maxSpeed);
	tgSpringCableActuator::Config muscleConfigOther(2000, damping*2, passivePretension); //May need different configs for different muscles in legs/feet... but using one to start with.
    #endif

#endif

    //Create the basic unit of the puppy
    tgStructure vertebra;
    addNodesVertebra(vertebra,rod_space);
    addRodsVertebra(vertebra);

    //Create the basic unit for the hips/shoulders. 
    tgStructure hip;
    addNodesHip(hip,rod_space);
    addRodsHip(hip);

    //Build the puppy
    tgStructure puppy;

    //const double yOffset_foot = -(2*rod_space+6);

    addSegments(puppy,vertebra,hip,rod_space); //,m_segments,m_hips,m_legs,m_feet

    puppy.move(btVector3(0.0,12.0,0.0));

    addMuscles(puppy); //,m_segments,m_hips,m_legs,m_feet

    //Time to add the muscles to the structure. Todo: make this a function; also try to clean this up.
    std::vector<tgStructure*> children = puppy.getChildren();
   
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));

#ifdef USE_KINEMATIC

    #ifdef PASSIVE_STRUCTURE
        spec.addBuilder("muscle", new tgKinematicContactCableInfo(motorConfig));
    #else 
	spec.addBuilder("muscleAct", new tgKinematicContactCableInfo(motorConfigSpine));
	spec.addBuilder("muscle ", new tgKinematicContactCableInfo(motorConfigOther));
    #endif

#else
    #ifdef PASSIVE_STRUCTURE
   	spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    #else 
	spec.addBuilder("muscleAct" , new tgBasicActuatorInfo(muscleConfigSpine));
	spec.addBuilder("muscle " , new tgBasicActuatorInfo(muscleConfigOther));
    #endif
    
#endif


    
    // Create your structureInfo
    tgStructureInfo structureInfo(puppy, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    m_allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    m_allSegments = this->find<tgModel> ("segment");
    
    // Actually setup the children, notify controller that the setup has finished
    BaseQuadModelLearning::setup(world);

    children.clear();
}

void BigPuppySpineOnlyStats::step(double dt)
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

void BigPuppySpineOnlyStats::teardown()
{
    BaseQuadModelLearning::teardown();
}
