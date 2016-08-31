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
 * @file MountainGoatAchilles.cpp
 * @brief Implementing Mountain Goat 
 * Trying to implement a longer achilles tendon muscle.
 * @author Dawn Hustig-Schultz
 * @date April 2016
 * @version 1.1.0
 * $Id$
 */


//This application
#include "MountainGoatAchilles.h"

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

MountainGoatAchilles::MountainGoatAchilles(int segments, int hips, int legs) :
BaseQuadModelLearning(segments, hips),
m_legs(legs)
{
    m_subStructures = segments + hips + legs;
}

MountainGoatAchilles::~MountainGoatAchilles()
{
}

void MountainGoatAchilles::addNodesLeg(tgStructure& s, double r){ 
    s.addNode(0,0,0); //0: Bottom Center of lower leg segment
    s.addNode(0,r,0);  //1: Center of lower leg segment
    s.addNode(r,r,0); //2: Right of lower leg segment
    s.addNode(-r,r,0);  //3: Left of lower leg segment
    s.addNode(0,2*r-2,0);  //4: Top of lower leg segment
    s.addNode(0,-r/2,0);  //5: Leg segment extension for connections to foot.

    //Extra nodes, for supporting rod structures on bottom
    s.addNode(0,-r/2,-r/2); //6
    s.addNode(0,-r/2,r/2); //7
    s.addNode(r/2,-r/2,0); //8
    s.addNode(-r/2,-r/2,0); //9 
}

void MountainGoatAchilles::addRodsLeg(tgStructure& s){
    s.addPair(0,1,"rod2");
    s.addPair(1,2,"rod2");
    s.addPair(1,3,"rod2");
    s.addPair(1,4,"rod2");
    //s.addPair(0,5,"rod");

    //Extra rods, for support
    //s.addPair(5,6,"rod");
    //s.addPair(5,7,"rod");
    //s.addPair(5,8,"rod");
    //s.addPair(5,9,"rod");

    s.addPair(0,6,"rod2");
    s.addPair(0,7,"rod2");
    s.addPair(0,8,"rod2");
    s.addPair(0,9,"rod2");
    
}

void MountainGoatAchilles::addNodesHip(tgStructure& s, double r){
    s.addNode(0,0,0); //Node 0 
    s.addNode(0,r,r); //Node 1 
    s.addNode(0,-r,-r); //Node 2
    s.addNode(0,-r,r); //Node 3
    s.addNode(0,r/2,0); //Node4
}

void MountainGoatAchilles::addRodsHip(tgStructure& s){
    s.addPair(4,1,"rod");
    s.addPair(0,2,"rod");
    s.addPair(0,3,"rod");
    s.addPair(0,4,"rod");
}

void MountainGoatAchilles::addNodesVertebra(tgStructure& s, double r){
    s.addNode(0,0,0); //Node 0 
    s.addNode(r,0,r); //Node 1 
    s.addNode(r,0,-r); //Node 2
    s.addNode(-r,0,-r); //Node 3
    s.addNode(-r,0,r); //Node 4
}

void MountainGoatAchilles::addRodsVertebra(tgStructure& s){
    s.addPair(0,1,"rod");
    s.addPair(0,2,"rod");
    s.addPair(0,3,"rod");
    s.addPair(0,4,"rod");
}

void MountainGoatAchilles::addSegments(tgStructure& goat, tgStructure& vertebra, tgStructure& hip, tgStructure& leg, double r){ 
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
        t->addTags(tgString("spine segment num", i + 1));
        t->move((i + 1)*offset);

        if (i % 2 == 1){

            t->addRotation(btVector3((i + 1) * offsetDist, 0.0, 0.0), btVector3(1, 0, 0), 0.0);

        }
        else{

            t->addRotation(btVector3((i + 1) * offsetDist, 0.0, 0.0), btVector3(1, 0, 0), M_PI/2.0);

        }

        goat.addChild(t); //Add a segment to the goat
    }

     for(std::size_t i = m_segments; i < (m_segments + 2); i++) {//deal with left hip and shoulder first
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

        goat.addChild(t); //Add a segment to the goat
    }

    for(std::size_t i = (m_segments + 2); i < (m_segments + m_hips); i++) {//deal with right hip and shoulder now
        tgStructure* t = new tgStructure (hip);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset4);
        }
        else{
            t->move(offset3);
        }

        goat.addChild(t); //Add a segment to the goat

    }

     for(std::size_t i = (m_segments + m_hips); i < (m_segments + m_hips + 2); i++) {//left front and back legs
        tgStructure* t = new tgStructure (leg);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset5);
            t->addRotation(btVector3(offsetDist3, yOffset_leg, offsetDist), btVector3(0, 1, 0), 0);

        }
        else{
            t->move(offset7);
            t->addRotation(btVector3(r*2, yOffset_leg, offsetDist), btVector3(0, 1, 0), 0);

        }

        goat.addChild(t); //Add a segment to the goat
    }

    for(std::size_t i = (m_segments + m_hips + 2); i < (m_segments + m_hips + m_legs); i++) {//right front and back legs
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

        goat.addChild(t); //Add a segment to the goat
    }

}

void MountainGoatAchilles::addMuscles(tgStructure& goat){ 
        //Time to add the muscles to the structure. 
	//A note about tags: if want to identify a muscle by multiple words, the multiple words will be pulled out in any order,
	//not the order in which they are written. So, put underscores, use camel case, or use unique, individual words to pull out muscles! 
    std::vector<tgStructure*> children = goat.getChildren();
    for(std::size_t i = 2; i < (children.size() - (m_hips + m_legs)); i++) { 

        tgNodes n0 = children[i-2]->getNodes();
        tgNodes n1 = children[i-1]->getNodes();
        tgNodes n2 = children[i]->getNodes();


        if(i==2){
            //Extra muscles, to keep front vertebra from swinging. 
            goat.addPair(n0[3], n1[3], tgString("spine all main front upper right muscleAct1 seg", i-2) + tgString(" seg", i-1));
            goat.addPair(n0[3], n1[4], tgString("spine all main front upper left muscleAct1 seg", i-2) + tgString(" seg", i-1));

            goat.addPair(n0[4], n1[3], tgString("spine all main front lower right muscleAct2 seg", i-2) + tgString(" seg", i-1));
            goat.addPair(n0[4], n1[4], tgString("spine all main front lower left muscleAct2 seg", i-2) + tgString(" seg", i-1));


        }        

        //Add muscles to the goat
        if(i < 3){
            if(i % 2 == 0){ //front
                goat.addPair(n0[1], n1[3], tgString("spine all main front lower right muscleAct2 seg", i-2) + tgString(" seg", i-1));
                goat.addPair(n0[1], n1[4], tgString("spine all main front lower left muscleAct2 seg", i-2) + tgString(" seg", i-1));
                goat.addPair(n0[2], n1[3], tgString("spine all main front upper right muscleAct1 seg", i-2) + tgString(" seg", i-1));
                goat.addPair(n0[2], n1[4], tgString("spine all main front upper left muscleAct1 seg", i-2) + tgString(" seg", i-1));
            }
            else{ //rear
                goat.addPair(n0[1], n1[3], tgString("spine all main rear upper left muscleAct1 seg", i-2) + tgString(" seg", i-1));
                goat.addPair(n0[1], n1[4], tgString("spine all main rear lower left muscleAct2 seg", i-2) + tgString(" seg", i-1));
                goat.addPair(n0[2], n1[3], tgString("spine all main rear upper right muscleAct1 seg", i-2) + tgString(" seg", i-1));
                goat.addPair(n0[2], n1[4], tgString("spine all main rear lower right muscleAct2 seg", i-2) + tgString(" seg", i-1)); 
            }
        }
        if(i < 7){//Was 6
            if(i % 2 == 0){
                goat.addPair(n0[1], n2[4], tgString("spine2 bottom muscleAct2 seg", i-2) + tgString(" seg", i-1));
                goat.addPair(n0[2], n2[3], tgString("spine2 top muscleAct1 seg", i-2) + tgString(" seg", i-1));
            }
            else{
                goat.addPair(n0[1], n2[4], tgString("spine2 lateral left muscleAct1 seg", i-2) + tgString(" seg", i-1));
                goat.addPair(n0[2], n2[3], tgString("spine2 lateral right muscleAct1 seg", i-2) + tgString(" seg", i-1));

            }
        }
        if(i > 0 && i < 7){
            if(i % 2 == 0){//rear
                goat.addPair(n1[1], n2[3], tgString("spine all main rear upper left muscleAct1 seg", i-1) + tgString(" seg", i));
                goat.addPair(n1[1], n2[4], tgString("spine all main rear lower left muscleAct2 seg", i-1) + tgString(" seg", i));
                goat.addPair(n1[2], n2[3], tgString("spine all main rear upper right muscleAct1 seg", i-1) + tgString(" seg", i));
                goat.addPair(n1[2], n2[4], tgString("spine all main rear lower right muscleAct2 seg", i-1) + tgString(" seg", i));
            }
            else{//front

                goat.addPair(n1[1], n2[3], tgString("spine all main front lower right muscleAct2 seg", i-1) + tgString(" seg", i));
                goat.addPair(n1[1], n2[4], tgString("spine all main front lower left muscleAct2 seg", i-1) + tgString(" seg", i));
                goat.addPair(n1[2], n2[3], tgString("spine all main front upper right muscleAct1 seg", i-1) + tgString(" seg", i));
                goat.addPair(n1[2], n2[4], tgString("spine all main front upper left muscleAct1 seg", i-1) + tgString(" seg", i));
            }
        }
	if (i >= 2 && i < 7){
	    goat.addPair(n1[3], n2[3], tgString("spine all spiral muscleAct1 seg", i-1) + tgString(" seg", i));
	    goat.addPair(n1[4], n2[3], tgString("spine all spiral muscleAct1 seg", i-1) + tgString(" seg", i));
	    goat.addPair(n1[3], n2[4], tgString("spine all spiral muscleAct1 seg", i-1) + tgString(" seg", i));
	    goat.addPair(n1[4], n2[4], tgString("spine all spiral muscleAct1 seg", i-1) + tgString(" seg", i));
	}
        if(i == 6){
            //rear
            goat.addPair(n1[1], n2[2], tgString("spine all rear lower left muscleAct2 seg", i-1) + tgString(" seg", i));
            goat.addPair(n1[2], n2[2], tgString("spine all rear lower right muscleAct2 seg", i-1) + tgString(" seg", i));
            goat.addPair(n1[1], n2[1], tgString("spine all rear upper left muscleAct1 seg", i-1) + tgString(" seg", i));
            goat.addPair(n1[2], n2[1], tgString("spine all rear upper right muscleAct1 seg", i-1) + tgString(" seg", i));  
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

    //Adding long muscles to spine, for bending/arching:
    //goat.addPair(n0[2], n6[3], tgString("spine secondary top arching muscleAct seg", 0) + tgString(" seg", 6)); //Change these to something other than "spine " or "spine2" when it's time to implement new code for them!
    //goat.addPair(n0[1], n6[4], tgString("spine bottom arching muscleAct seg", 0) + tgString(" seg", 6));
    //goat.addPair(n1[4], n5[1], tgString("spine right lateral arching muscleAct seg", 1) + tgString(" seg", 5));
    //goat.addPair(n1[3], n5[2], tgString("spine left lateral arching muscleAct seg", 1) + tgString(" seg", 5));
    
    //Left shoulder muscles
    goat.addPair(n7[1], n1[1], tgString("all left_shoulder hip rear upper muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n7[1], n1[4], tgString("all left_shoulder hip front upper muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n7[1], n0[2], tgString("hip front top muscleAct1 seg", 7) + tgString(" seg", 0)); //all left_shoulder
    goat.addPair(n7[1], n2[3], tgString("hip rear top muscleAct1 seg", 7) + tgString(" seg", 2));

    goat.addPair(n7[3], n1[1], tgString("all left_shoulder hip rear lower muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n7[3], n1[4], tgString("all left_shoulder hip front lower muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n7[3], n0[1], tgString("front bottom muscleAct1 seg", 7) + tgString(" seg", 0));
    goat.addPair(n7[3], n2[4], tgString("hip rear bottom muscleAct1 seg", 7) + tgString(" seg", 2));

    //Extra muscles, to move left shoulder forward and back:
    goat.addPair(n7[0], n1[1], tgString("all left_shoulder hip rear mid muscleAct1 seg", 7) + tgString(" seg", 1));
    goat.addPair(n7[0], n1[4], tgString("all left_shoulder hip front mid muscleAct1 seg", 7) + tgString(" seg", 1));

    //Left hip muscles
    goat.addPair(n8[1], n5[1], tgString("all left_hip rear upper muscleAct1 seg", 8) + tgString(" seg", 5));
    goat.addPair(n8[1], n5[4], tgString("all left_hip front upper muscleAct1 seg", 8) + tgString(" seg", 5));
    goat.addPair(n8[1], n4[2], tgString("front top muscleAct1 seg", 8) + tgString(" seg", 4));
    goat.addPair(n8[1], n6[3], tgString("rear top muscleAct1 seg", 8) + tgString(" seg", 6));

    goat.addPair(n8[3], n5[1], tgString("all left_hip rear lower muscleAct1 seg", 8) + tgString(" seg", 5));
    goat.addPair(n8[3], n5[4], tgString("all left_hip front lower muscleAct1 seg", 8) + tgString(" seg", 5));
    goat.addPair(n8[3], n4[1], tgString("front bottom muscleAct1 seg", 8) + tgString(" seg", 4));
    goat.addPair(n8[3], n6[4], tgString("front bottom muscleAct1 seg", 8) + tgString(" seg", 6)); //all left_hip 

    //Extra muscles, to move left hip forward and back:
    goat.addPair(n8[0], n5[1], tgString("all left_hip rear mid muscleAct1 seg", 8) + tgString(" seg", 5)); 
    goat.addPair(n8[0], n5[4], tgString("all left_hip front mid muscleAct1 seg", 8) + tgString(" seg", 5));

    //Right shoulder muscles
    goat.addPair(n9[1], n1[2], tgString("all right_shoulder hip rear upper muscleAct1 seg", 9) + tgString(" seg", 1));
    goat.addPair(n9[1], n1[3], tgString("all right_shoulder hip front upper muscleAct1 seg", 9) + tgString(" seg", 1));
    goat.addPair(n9[1], n0[2], tgString("hip front top muscleAct1 seg", 9) + tgString(" seg", 0));
    goat.addPair(n9[1], n2[3], tgString("hip rear top muscleAct1 seg", 9) + tgString(" seg", 2));

    goat.addPair(n9[3], n1[2], tgString("all right_shoulder hip rear lower muscleAct1 seg", 9) + tgString(" seg", 1));
    goat.addPair(n9[3], n1[3], tgString("all right_shoulder hip front lower muscleAct1 seg", 9) + tgString(" seg", 1));
    goat.addPair(n9[3], n0[1], tgString("hip front bottom muscleAct1 seg", 9) + tgString(" seg", 0));
    goat.addPair(n9[3], n2[4], tgString("hip rear bottom muscleAct1 seg", 9) + tgString(" seg", 2));

    //Extra muscles, to move right shoulder forward and back:
    goat.addPair(n9[0], n1[2], tgString("all right_shoulder hip rear mid muscleAct1 seg", 9) + tgString(" seg", 1));
    goat.addPair(n9[0], n1[3], tgString("all right_shoulder hip front mid muscleAct1 seg", 9) + tgString(" seg", 1));

    //Right hip muscles
    goat.addPair(n10[1], n5[2], tgString("all right_hip rear upper muscleAct1 seg", 10) + tgString(" seg", 5));
    goat.addPair(n10[1], n5[3], tgString("all right_hip front upper muscleAct1 seg", 10) + tgString(" seg", 5));
    goat.addPair(n10[1], n4[2], tgString("front top muscleAct1 seg", 10) + tgString(" seg", 4)); //all right_hip 
    goat.addPair(n10[1], n6[3], tgString("rear top muscleAct1 seg", 10) + tgString(" seg", 6));

    goat.addPair(n10[3], n5[2], tgString("all right_hip rear lower muscleAct1 seg", 10) + tgString(" seg", 5));
    goat.addPair(n10[3], n5[3], tgString("all right_hip front lower muscleAct1 seg", 10) + tgString(" seg", 5));
    goat.addPair(n10[3], n4[1], tgString("bottom muscleAct1 seg", 10) + tgString(" seg", 4));  
    goat.addPair(n10[3], n6[4], tgString("bottom muscleAct1 seg", 10) + tgString(" seg", 6));  

    //Extra muscles, to move right hip forward and back:
    goat.addPair(n10[0], n5[2], tgString("all right_hip rear mid muscleAct1 seg", 10) + tgString(" seg", 5)); 
    goat.addPair(n10[0], n5[3], tgString("all right_hip front mid muscleAct1 seg", 10) + tgString(" seg", 5));

    //Leg/hip connections:

    //Left front leg/shoulder
    goat.addPair(n11[4], n7[3], tgString("all left_foreleg outer bicep muscle seg", 11) + tgString(" seg", 7));
    goat.addPair(n11[4], n7[2], tgString("all left_foreleg inner bicep muscle seg", 11) + tgString(" seg", 7));
    goat.addPair(n11[4], n1[4], tgString("all left_foreleg front_horizontal abdomen connection muscle seg", 11) + tgString(" seg", 1));
    goat.addPair(n11[4], n1[1], tgString("all left_foreleg rear_horizontal abdomen connection muscle seg", 11) + tgString(" seg", 1));
    goat.addPair(n11[2], n1[1],tgString("all left_foreleg front abdomen connection muscle3 seg", 11) + tgString(" seg", 1)); //Active
    goat.addPair(n11[3], n1[4],tgString("all left_foreleg rear abdomen connection muscle3 seg", 11) + tgString(" seg", 1)); //Active

    goat.addPair(n11[3], n7[3], tgString("outer tricep muscle seg", 11) + tgString(" seg", 7));
    goat.addPair(n11[3], n7[2], tgString("inner tricep muscle seg", 11) + tgString(" seg", 7));

    goat.addPair(n11[2], n7[3], tgString("outer front tricep muscle seg", 11) + tgString(" seg", 7));
    goat.addPair(n11[2], n7[2], tgString("inner front tricep muscle seg", 11) + tgString(" seg", 7));

    //Adding muscle to pull up on right front leg:
    goat.addPair(n11[4], n7[1], tgString("all left_foreleg mid bicep muscle3 seg", 11) + tgString(" seg", 7)); //Active

    //Achilles Tendon Muscle:
    goat.addPair(n11[8], n7[4], tgString("all left_front_achilles_tendon only muscle seg", 11) + tgString(" seg", 7)); //0-4
    goat.addPair(n11[9], n7[4], tgString("all left_front_achilles_tendon only muscle seg", 11) + tgString(" seg", 7)); //0-0
    //goat.addPair(n11[0], n8[4], tgString("all achilles_tendon muscle seg", 11) + tgString(" seg", 8));
    
    //Right front leg/shoulder
    goat.addPair(n13[4], n9[2], tgString("all right_foreleg inner bicep muscle seg", 13) + tgString(" seg", 9));
    goat.addPair(n13[4], n9[3], tgString("all right_foreleg outer bicep muscle seg", 13) + tgString(" seg", 9));
    goat.addPair(n13[4], n1[3], tgString("all right_foreleg front_horizontal abdomen connection muscle seg", 13) + tgString(" seg", 1));
    goat.addPair(n13[4], n1[2], tgString("all right_foreleg rear_horizontal abdomen connection muscle seg", 13) + tgString(" seg", 1));
    goat.addPair(n13[3], n1[2], tgString("all right_foreleg front abdomen connection muscle3 seg", 13) + tgString(" seg", 1)); //Active
    goat.addPair(n13[2], n1[3], tgString("all right_foreleg rear abdomen connection muscle3 seg", 13) + tgString(" seg", 1)); //Active


    goat.addPair(n13[3], n9[2], tgString("inner tricep muscle seg", 13) + tgString(" seg", 9));
    goat.addPair(n13[3], n9[3], tgString("outer tricep muscle seg", 13) + tgString(" seg", 9));

    goat.addPair(n13[2], n9[2], tgString("inner front tricep muscle seg", 13) + tgString(" seg", 9));
    goat.addPair(n13[2], n9[3], tgString("outer front tricep muscle seg", 13) + tgString(" seg", 9));

    //Adding muscle to pull up on left front leg:
    goat.addPair(n13[4], n9[1], tgString("all right_foreleg mid bicep muscle3 seg", 13) + tgString(" seg", 9)); //Active

    //Achilles Tendon Muscle:
    goat.addPair(n13[8], n9[4], tgString("all right_front_achilles_tendon only muscle seg", 13) + tgString(" seg", 9)); //0-4
    goat.addPair(n13[9], n9[4], tgString("all right_front_achilles_tendon only muscle seg", 13) + tgString(" seg", 9)); //0-0
    //goat.addPair(n13[0], n10[4], tgString("all right_front_achilles_tendon muscle seg", 13) + tgString(" seg", 10));

    //Left rear leg/hip
    goat.addPair(n12[4], n8[3], tgString("all left_hindleg outer thigh muscle seg", 12) + tgString(" seg", 8)); 
    goat.addPair(n12[4], n8[2], tgString("all left_hindleg inner thigh muscle seg", 12) + tgString(" seg", 8));
    goat.addPair(n12[4], n3[1],tgString("all left_hindleg front_horizontal abdomen connection muscle seg", 12) + tgString(" seg", 3));
    goat.addPair(n12[4], n5[1],tgString("all left_hindleg rear_horizontal abdomen connection muscle seg", 12) + tgString(" seg", 3));  
    goat.addPair(n12[2], n5[1],tgString("all left_hindleg front abdomen connection muscle3 seg", 12) + tgString(" seg", 5)); //Active
    goat.addPair(n12[3], n5[4],tgString("all left_hindleg rear abdomen connection muscle3 seg", 12) + tgString(" seg", 5)); //Active

    goat.addPair(n12[3], n8[3], tgString("outer calf muscle seg", 12) + tgString(" seg", 8));
    goat.addPair(n12[3], n8[2], tgString("inner calf muscle seg", 12) + tgString(" seg", 8));

    goat.addPair(n12[2], n8[3], tgString("outer front calf muscle seg", 12) + tgString(" seg", 8));
    goat.addPair(n12[2], n8[2], tgString("inner front calf muscle seg", 12) + tgString(" seg", 8));

    //Adding muscle to pull rear right leg up:
    goat.addPair(n12[4], n8[1], tgString("all left_hindleg central thigh muscle3 seg", 12) + tgString(" seg", 8)); //Active

    //Achilles Tendon Muscle:
    goat.addPair(n12[8], n8[4], tgString("all left_rear_achilles_tendon only muscle seg", 12) + tgString(" seg", 8)); //0-4
    goat.addPair(n12[9], n8[4], tgString("all left_rear_achilles_tendon only muscle seg", 12) + tgString(" seg", 8)); //0-0
    //goat.addPair(n12[0], n7[4], tgString("all left_rear_achilles_tendon muscle seg", 12) + tgString(" seg", 7));

    //Right rear leg/hip
    goat.addPair(n14[4], n10[2], tgString("all right_hindleg inner thigh muscle seg", 14) + tgString(" seg", 10)); 
    goat.addPair(n14[4], n10[3], tgString("all right_hindleg outer thigh muscle seg", 14) + tgString(" seg", 10));
    goat.addPair(n14[4], n3[2], tgString("all right_hindleg front_horizontal abdomen connection muscle seg", 14) + tgString(" seg", 3));
    goat.addPair(n14[4], n5[2], tgString("all right_hindleg rear_horizontal abdomen connection muscle seg", 14) + tgString(" seg", 3));  
    goat.addPair(n14[3], n5[2], tgString("all right_hindleg front abdomen connection muscle3 seg", 14) + tgString(" seg", 5)); 
    goat.addPair(n14[2], n5[3], tgString("all right_hindleg rear abdomen connection muscle3 seg", 14) + tgString(" seg", 5)); 


    goat.addPair(n14[3], n10[2], tgString("inner calf muscle seg", 14) + tgString(" seg", 10));
    goat.addPair(n14[3], n10[3], tgString("outer calf muscle seg", 14) + tgString(" seg", 10));

    goat.addPair(n14[2], n10[2], tgString("inner front calf muscle seg", 14) + tgString(" seg", 10));
    goat.addPair(n14[2], n10[3], tgString("outer front calf muscle seg", 14) + tgString(" seg", 10));

    //Adding muscle to pull rear left leg up:
    goat.addPair(n14[4], n10[1], tgString("all right_hindleg central thigh muscle3 seg", 14) + tgString(" seg", 10)); 

    //Achilles Tendon Muscle:
    goat.addPair(n14[8], n10[4], tgString("all right_rear_achilles_tendon only muscle seg", 14) + tgString(" seg", 10)); //0-4
    goat.addPair(n14[9], n10[4], tgString("all right_rear_achilles_tendon only muscle seg", 14) + tgString(" seg", 10)); //0-0
    //goat.addPair(n14[0], n9[4], tgString("all right_rear_achilles_tendon muscle seg", 14) + tgString(" seg", 9));

}

void MountainGoatAchilles::setup(tgWorld& world)
{
    //Rod and Muscle configuration. 
    const double density = 4.2/300.0; //Note: this needs to be high enough or things fly apart...

    const double density2 = 4.2/300.0; //For legs, might try to modify this a bit. 

    const double radius = 0.5;
    const double rod_space = 10.0;
    const double rod_space2 = 8.0;
    const double friction = 0.5;
    const double rollFriction = 0.0;
    const double restitution = 0.0;

    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);
    const tgRod::Config rodConfig2(radius, density2, friction, rollFriction, restitution);


    const double stiffness = 1000.0;
    const double stiffnessPassive = 4000.0; //4000
    const double stiffnessPassive2 = 4000.0;
    const double stiffnessPassive3 = 10000.0;
    const double damping = .01*stiffness;
    const double pretension = 0.0;
    const bool   history = true;
    const double maxTens = 10000.0; //7000.0
    const double maxSpeed = 12.0;	

    const double maxTens2 = 1000000.0; //7000.0
    const double maxSpeed2 = 20.0;	//12.0

    const double passivePretension = 1000; 
    const double passivePretension2 = 3500; 
    const double passivePretension3 = 3500; 
    const double passivePretension4 = 5000.0;

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
	tgSpringCableActuator::Config muscleConfigOther(stiffnessPassive, damping, passivePretension2, history, maxTens, 2*maxSpeed);
	tgSpringCableActuator::Config muscleConfigStomach(stiffnessPassive2, damping, passivePretension4, history, maxTens, 2*maxSpeed); 
	tgSpringCableActuator::Config muscleConfigLegs(stiffnessPassive, damping, passivePretension3, history, maxTens2, 2*maxSpeed2);
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

    const double yOffset_foot = -(2*rod_space+6);

    addSegments(goat,vertebra,hip,leg,rod_space); //,m_segments,m_hips,m_legs,m_feet

    goat.move(btVector3(0.0,-yOffset_foot,0.0));

    addMuscles(goat); //,m_segments,m_hips,m_legs,m_feet

    std::vector<tgStructure*> children = goat.getChildren();
   
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("rod2", new tgRodInfo(rodConfig2));

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

    m_allSegments = this->find<tgModel> ("segment");
    
    // Actually setup the children, notify controller that the setup has finished
    BaseQuadModelLearning::setup(world);

    children.clear();
}

void MountainGoatAchilles::step(double dt)
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

void MountainGoatAchilles::teardown()
{
    BaseQuadModelLearning::teardown();
}
