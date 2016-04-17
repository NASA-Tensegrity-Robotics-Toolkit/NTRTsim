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
 * @file BigPuppyNoFeet.cpp
 * @brief Implementing a quadruped based off the Flemons BigPuppy model.
 * @author Dawn Hustig-Schultz
 * @date July 2015
 * @version 1.1.0
 * $Id$
 */


//This application
#include "BigPuppyNoFeet.h"

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
#define PASSIVE_STRUCTURE

BigPuppyNoFeet::BigPuppyNoFeet(int segments, int hips, int legs, int feet) :
BaseSpineModelLearning(segments),
m_hips(hips),
m_legs(legs),
m_feet(feet)
{
}

BigPuppyNoFeet::~BigPuppyNoFeet()
{
}

void BigPuppyNoFeet::addNodesFoot(tgStructure& s, double r1, double r2){
    s.addNode(r2,0,r2);//0 
    s.addNode(r2,0,-r2);//1
    s.addNode(-r2,0,-r2);//2 
    s.addNode(-r2,0,r2);//3 
    s.addNode(r2/2,r1/2,0);//4
    s.addNode(0,r1/2,-r2/2);//5
    s.addNode(-r2/2,r1/2,0);//6
    s.addNode(0,r1/2,r2/2);//7        
}

void BigPuppyNoFeet::addRodsFoot(tgStructure& s){
    s.addPair(0,6,"rod");
    s.addPair(1,7,"rod");
    s.addPair(2,4,"rod");
    s.addPair(3,5,"rod");
}

void BigPuppyNoFeet::addNodesLeg(tgStructure& s, double r){ 
    s.addNode(0,0,0); //0: Bottom Center of lower leg segment
    s.addNode(0,r,0);  //1: Center of lower leg segment
    s.addNode(r,r,0); //2: Right of lower leg segment
    s.addNode(-r,r,0);  //3: Left of lower leg segment
    s.addNode(0,2*r,0);  //4: Top of lower leg segment
    s.addNode(0,-r/2,0);  //5: Leg segment extension for connections to foot.
}

void BigPuppyNoFeet::addRodsLeg(tgStructure& s){
    s.addPair(0,1,"rod");
    s.addPair(1,2,"rod");
    s.addPair(1,3,"rod");
    s.addPair(1,4,"rod");
    s.addPair(0,5,"rod");
}

void BigPuppyNoFeet::addNodesHip(tgStructure& s, double r){
    s.addNode(0,0,0); //Node 0 
    s.addNode(0,r,r); //Node 1 
    s.addNode(0,-r,-r); //Node 2
    s.addNode(0,-r,r); //Node 3
}

void BigPuppyNoFeet::addRodsHip(tgStructure& s){
    s.addPair(0,1,"rod");
    s.addPair(0,2,"rod");
    s.addPair(0,3,"rod");
}

void BigPuppyNoFeet::addNodesVertebra(tgStructure& s, double r){
    s.addNode(0,0,0); //Node 0 
    s.addNode(r,0,r); //Node 1 
    s.addNode(r,0,-r); //Node 2
    s.addNode(-r,0,-r); //Node 3
    s.addNode(-r,0,r); //Node 4
}

void BigPuppyNoFeet::addRodsVertebra(tgStructure& s){
    s.addPair(0,1,"rod");
    s.addPair(0,2,"rod");
    s.addPair(0,3,"rod");
    s.addPair(0,4,"rod");
}

void BigPuppyNoFeet::addSegments(tgStructure& puppy, tgStructure& vertebra, tgStructure& hip, tgStructure& leg,
                 double r){ 
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

     for(std::size_t i = (m_segments + m_hips); i < (m_segments + m_hips + 2); i++) {//right front and back legs
        tgStructure* t = new tgStructure (leg);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset5);
            t->addRotation(btVector3(offsetDist3, yOffset_leg, offsetDist), btVector3(0, 1, 0), M_PI);

        }
        else{
            t->move(offset7);
            t->addRotation(btVector3(r*2, yOffset_leg, offsetDist), btVector3(0, 1, 0), M_PI);
            //the rotations for the legs are a remnant of the earlier design. Removing them now 
            //would mean changing all my muscle attachments. I will do this someday. 

        }

        puppy.addChild(t); //Add a segment to the puppy
    }

    for(std::size_t i = (m_segments + m_hips + 2); i < (m_segments + m_hips + m_legs); i++) {//left front and back legs
        tgStructure* t = new tgStructure (leg);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset6);
            t->addRotation(btVector3(offsetDist3, yOffset_leg, -offsetDist), btVector3(0, 1, 0), M_PI);

        }
        else{
            t->move(offset8);
            t->addRotation(btVector3(r*2, yOffset_leg, -offsetDist), btVector3(0, 1, 0), M_PI);
        }

        puppy.addChild(t); //Add a segment to the puppy
    }

    /*for(std::size_t i = (m_segments + m_hips + m_legs); i < (m_segments + m_hips + m_legs + 2); i++) {//right front and back feet
        tgStructure* t = new tgStructure (foot);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset9);
            t->addRotation(btVector3(offsetDist3+1, yOffset_foot, offsetDist), btVector3(0, 1, 0), 0.0);

        }
        else{
            t->move(offset11);
            t->addRotation(btVector3(r*2+1, yOffset_foot, offsetDist), btVector3(0, 1, 0), 0.0);
        }

        puppy.addChild(t); //Add a segment to the puppy
    }

    for(std::size_t i = (m_segments + m_hips + m_legs + 2); i < (m_segments + m_hips + m_legs + m_feet); i++) {//left front and back feet
        tgStructure* t = new tgStructure (foot);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset10);
            t->addRotation(btVector3(offsetDist3+1, yOffset_foot, -offsetDist), btVector3(0, 1, 0), 0.0);

        }
        else{
            t->move(offset12);
            t->addRotation(btVector3(r*2+1, yOffset_foot, -offsetDist), btVector3(0, 1, 0), 0.0);
        }

        puppy.addChild(t); //Add a segment to the puppy
    }*/
}

void BigPuppyNoFeet::addMuscles(tgStructure& puppy){ 
        //Time to add the muscles to the structure. Todo: try to clean this up some more.
    std::vector<tgStructure*> children = puppy.getChildren();
    for(std::size_t i = 2; i < (children.size() - (m_hips + m_legs)); i++) { 

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
    tgNodes n11 = children[11]->getNodes();
    tgNodes n12 = children[12]->getNodes();
    tgNodes n13 = children[13]->getNodes();
    tgNodes n14 = children[14]->getNodes();
    
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

    //Leg/hip connections:

    //Left front leg/shoulder
    puppy.addPair(n11[4], n7[3], tgString("right outer bicep muscle seg", 10) + tgString(" seg", 6));
    puppy.addPair(n11[4], n7[2], tgString("right inner bicep muscle seg", 10) + tgString(" seg", 6));
    puppy.addPair(n11[4], n1[4], tgString("right front abdomen connection muscle seg", 10) + tgString(" seg", 1));

    puppy.addPair(n11[3], n7[3], tgString("right outer tricep muscle seg", 10) + tgString(" seg", 6));
    puppy.addPair(n11[3], n7[2], tgString("right inner tricep muscle seg", 10) + tgString(" seg", 6));

    puppy.addPair(n11[2], n7[3], tgString("right outer front tricep muscle seg", 10) + tgString(" seg", 6));
    puppy.addPair(n11[2], n7[2], tgString("right inner front tricep muscle seg", 10) + tgString(" seg", 6));

    //Adding muscle to pull up on right front leg:
    puppy.addPair(n11[4], n7[1], tgString("right mid bicep muscle3 seg", 10) + tgString(" seg", 6));
    
    //Right front leg/shoulder
    puppy.addPair(n13[4], n9[2], tgString("left inner bicep muscle seg", 12) + tgString(" seg", 8));
    puppy.addPair(n13[4], n9[3], tgString("left outer bicep muscle seg", 12) + tgString(" seg", 8));
    puppy.addPair(n13[4], n1[3], tgString("left front abdomen connection muscle seg", 12) + tgString(" seg", 1)); //Was n1[2]

    puppy.addPair(n13[3], n9[2], tgString("left inner tricep muscle seg", 12) + tgString(" seg", 8));
    puppy.addPair(n13[3], n9[3], tgString("left outer tricep muscle seg", 12) + tgString(" seg", 8));

    puppy.addPair(n13[2], n9[2], tgString("left inner front tricep muscle seg", 12) + tgString(" seg", 8));
    puppy.addPair(n13[2], n9[3], tgString("left outer front tricep muscle seg", 12) + tgString(" seg", 8));

    //Adding muscle to pull up on left front leg:
    puppy.addPair(n13[4], n9[1], tgString("left mid bicep muscle3 seg", 12) + tgString(" seg", 8));

    //Left rear leg/hip
    puppy.addPair(n12[4], n8[3], tgString("right outer thigh muscle seg", 11) + tgString(" seg", 7)); 
    puppy.addPair(n12[4], n8[2], tgString("right inner thigh muscle seg", 11) + tgString(" seg", 7));

    puppy.addPair(n12[4], n3[1],tgString("right rear abdomen connection muscle seg", 11) + tgString(" seg", 3)); 
    puppy.addPair(n12[3], n5[1],tgString("right rear abdomen connection muscle seg", 11) + tgString(" seg", 5)); 

    puppy.addPair(n12[3], n8[3], tgString("right outer calf muscle seg", 11) + tgString(" seg", 7));
    puppy.addPair(n12[3], n8[2], tgString("right inner calf muscle seg", 11) + tgString(" seg", 7));

    puppy.addPair(n12[2], n8[3], tgString("right outer front calf muscle seg", 11) + tgString(" seg", 7));
    puppy.addPair(n12[2], n8[2], tgString("right inner front calf muscle seg", 11) + tgString(" seg", 7));

    //Adding muscle to pull rear right leg up:
    puppy.addPair(n12[4], n8[1], tgString("right central thigh muscle3 seg", 11) + tgString(" seg", 7));

    //Right rear leg/hip
    puppy.addPair(n14[4], n10[2], tgString("left inner thigh muscle seg", 13) + tgString(" seg", 9)); 
    puppy.addPair(n14[4], n10[3], tgString("left outer thigh muscle seg", 13) + tgString(" seg", 9));

    puppy.addPair(n14[4], n3[2], tgString("left rear abdomen connection muscle seg", 13) + tgString(" seg", 3)); 
    puppy.addPair(n14[3], n5[2], tgString("left rear abdomen connection muscle seg", 13) + tgString(" seg", 5)); 

    puppy.addPair(n14[3], n10[2], tgString("left inner calf muscle seg", 13) + tgString(" seg", 9));
    puppy.addPair(n14[3], n10[3], tgString("left outer calf muscle seg", 13) + tgString(" seg", 9));

    puppy.addPair(n14[2], n10[2], tgString("left inner front calf muscle seg", 13) + tgString(" seg", 9));
    puppy.addPair(n14[2], n10[3], tgString("left outer front calf muscle seg", 13) + tgString(" seg", 9));

    //Adding muscle to pull rear left leg up:
    puppy.addPair(n14[4], n10[1], tgString("left central thigh muscle3 seg", 13) + tgString(" seg", 9));

    //Populate feet with muscles. Todo: think up names to differentiate each!
    /*for(std::size_t i = (m_segments + m_hips + m_legs); i < children.size(); i++) { 
        tgNodes ni = children[i]->getNodes();
        tgNodes ni4 = children[i-4]->getNodes();
        
        puppy.addPair(ni[0],ni[1],tgString("foot muscle seg", i));
        puppy.addPair(ni[0],ni[3],tgString("foot muscle seg", i));
        puppy.addPair(ni[1],ni[2],tgString("foot muscle seg", i));
        puppy.addPair(ni[2],ni[3],tgString("foot muscle seg", i));
        puppy.addPair(ni[0],ni[7],tgString("foot muscle2 seg", i));
        puppy.addPair(ni[1],ni[4],tgString("foot muscle2 seg", i));
        puppy.addPair(ni[2],ni[5],tgString("foot muscle2 seg", i));
        puppy.addPair(ni[3],ni[6],tgString("foot muscle2 seg", i));
        puppy.addPair(ni[4],ni[5],tgString("foot muscle2 seg", i));
        puppy.addPair(ni[4],ni[7],tgString("foot muscle2 seg", i));
        puppy.addPair(ni[5],ni[6],tgString("foot muscle2 seg", i));
        puppy.addPair(ni[6],ni[7],tgString("foot muscle2 seg", i));
        
        //Connect feet to legs:
        puppy.addPair(ni4[5],ni[0],tgString("foot muscle2 seg", i) + tgString(" seg", i-4));
        puppy.addPair(ni4[5],ni[1],tgString("foot muscle2 seg", i) + tgString(" seg", i-4));
        puppy.addPair(ni4[5],ni[2],tgString("foot muscle2 seg", i) + tgString(" seg", i-4));
        puppy.addPair(ni4[5],ni[3],tgString("foot muscle2 seg", i) + tgString(" seg", i-4));

        puppy.addPair(ni4[0],ni[4],tgString("foot muscle2 seg", i) + tgString(" seg", i-4));
        puppy.addPair(ni4[0],ni[5],tgString("foot muscle2 seg", i) + tgString(" seg", i-4));
        puppy.addPair(ni4[0],ni[6],tgString("foot muscle2 seg", i) + tgString(" seg", i-4));
        puppy.addPair(ni4[0],ni[7],tgString("foot muscle2 seg", i) + tgString(" seg", i-4));

    }*/

}

void BigPuppyNoFeet::setup(tgWorld& world)
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
    const double stiffnessPassive = 3000.0;
    const double damping = .01*stiffness;
    const double pretension = 0.0; 
    const bool   history = true;
    const double maxTens = 7000.0;
    const double maxSpeed = 12.0;

    const double passivePretension = 1000; 
    const double passivePretension2 = 2500;
    const double passivePretension3 = 2500;

#ifdef USE_KINEMATIC

    const double mRad = 1.0;
    const double motorFriction = 10.0;
    const double motorInertia = 1.0;
    const bool backDrivable = false;
    #ifdef PASSIVE_STRUCTURE
        tgKinematicActuator::Config motorConfig(2000, 20, passivePretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);

	tgKinematicActuator::Config motorConfigOther(stiffnessPassive, damping, passivePretension2,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 

	tgKinematicActuator::Config motorConfigFeet(stiffnessPassive, damping, passivePretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 
	tgKinematicActuator::Config motorConfigLegs(stiffnessPassive, 20, passivePretension3,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);

    #else
        tgKinematicActuator::Config motorConfigSpine(stiffness, damping, pretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 

	tgKinematicActuator::Config motorConfigOther(stiffnessPassive, damping, passivePretension2,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 

	tgKinematicActuator::Config motorConfigFeet(stiffnessPassive, damping, passivePretension,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed); 
	tgKinematicActuator::Config motorConfigLegs(stiffnessPassive, damping, passivePretension3,
                                            mRad, motorFriction, motorInertia, backDrivable,
                                            history, maxTens, maxSpeed);
    #endif

#else
    
    #ifdef PASSIVE_STRUCTURE
        tgSpringCableActuator::Config muscleConfig(2000, 20, passivePretension);
	tgSpringCableActuator::Config muscleConfigOther(stiffnessPassive, damping, passivePretension2);
	tgSpringCableActuator::Config muscleConfigFeet(stiffnessPassive, damping, passivePretension); 
	tgSpringCableActuator::Config muscleConfigLegs(stiffnessPassive, damping, passivePretension3);

    #else
        tgSpringCableActuator::Config muscleConfigSpine(stiffness, damping, pretension, history, maxTens, 2*maxSpeed);
	tgSpringCableActuator::Config muscleConfigOther(stiffnessPassive, damping, passivePretension2);
	tgSpringCableActuator::Config muscleConfigFeet(stiffnessPassive, damping, passivePretension); 
	tgSpringCableActuator::Config muscleConfigLegs(stiffnessPassive, damping, passivePretension3);
    #endif

#endif

    //Foot:
    //tgStructure foot;
    //addNodesFoot(foot,rod_space,rod_space2);
    //addRodsFoot(foot);

    //Leg:
    tgStructure leg;
    addNodesLeg(leg,rod_space);
    addRodsLeg(leg);

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

    const double yOffset_foot = -(2*rod_space+6);

    addSegments(puppy,vertebra,hip,leg,rod_space); //,m_segments,m_hips,m_legs,m_feet

    puppy.move(btVector3(0.0,26,0.0));

    addMuscles(puppy); //,m_segments,m_hips,m_legs,m_feet

    //Time to add the muscles to the structure. Todo: make this a function; also try to clean this up.
    std::vector<tgStructure*> children = puppy.getChildren();
   
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));

#ifdef USE_KINEMATIC

    #ifdef PASSIVE_STRUCTURE
        spec.addBuilder("muscleAct", new tgKinematicContactCableInfo(motorConfig));
	spec.addBuilder("muscle ", new tgKinematicContactCableInfo(motorConfigOther));
	//spec.addBuilder("muscle2 ", new tgKinematicContactCableInfo(motorConfigFeet));
	spec.addBuilder("muscle3 ", new tgKinematicContactCableInfo(motorConfigLegs));
    #else 
	spec.addBuilder("muscleAct", new tgKinematicContactCableInfo(motorConfigSpine));
	spec.addBuilder("muscle ", new tgKinematicContactCableInfo(motorConfigOther));
	//spec.addBuilder("muscle2 ", new tgKinematicContactCableInfo(motorConfigFeet));
	spec.addBuilder("muscle3 ", new tgKinematicContactCableInfo(motorConfigLegs));
	
    #endif

#else
    #ifdef PASSIVE_STRUCTURE
   	spec.addBuilder("muscleAct", new tgBasicActuatorInfo(muscleConfig));
	spec.addBuilder("muscle " , new tgBasicActuatorInfo(muscleConfigOther));
	//spec.addBuilder("muscle2 " , new tgBasicActuatorInfo(muscleConfigFeet));
	spec.addBuilder("muscle3 " , new tgBasicActuatorInfo(muscleConfigLegs));
    #else 
	spec.addBuilder("muscleAct" , new tgBasicActuatorInfo(muscleConfigSpine));
	spec.addBuilder("muscle " , new tgBasicActuatorInfo(muscleConfigOther));
	//spec.addBuilder("muscle2 " , new tgBasicActuatorInfo(muscleConfigFeet));
	spec.addBuilder("muscle3 " , new tgBasicActuatorInfo(muscleConfigLegs));
    #endif
    
#endif


    
    // Create your structureInfo
    tgStructureInfo structureInfo(puppy, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    m_allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    m_allSegments = this->find<tgModel> ("spine segment");
    
    // Actually setup the children, notify controller that the setup has finished
    BaseSpineModelLearning::setup(world);

    children.clear();
}

void BigPuppyNoFeet::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
	BaseSpineModelLearning::step(dt);
    }
}

void BigPuppyNoFeet::teardown()
{
    BaseSpineModelLearning::teardown();
}
