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
 * @file BigPuppy.cpp
 * @brief Implementing the Flemons quadruped model.
 * @author Dawn Hustig-Schultz
 * @date April 2015
 * @version 1.0.0
 * $Id$
 */


//This application
#include "BigPuppy.h"

// This library
#include "core/tgModel.h"
#include "core/tgSimView.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "core/tgBasicActuator.h"
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

BigPuppy::BigPuppy() :
tgModel() 
{
}

BigPuppy::~BigPuppy()
{
}

void BigPuppy::setup(tgWorld& world)
{
    //Rod and Muscle configuration

    const double density = 4.2/300.0;	//Note: this needs to be high enough or things fly apart...
    const double radius = 0.5;
    const double friction = 0.5;
    const double rollFriction = 0.0;
    const double restitution = 0.0;
    const tgRod::Config rodConfig(radius, density, friction, rollFriction, restitution);

    const double radius2 = 0.15;
    const double density2 = 1;	// Note: This needs to be high enough or things fly apart...
    const tgRod::Config rodConfig2(radius2, density2);

    const double stiffness = 1000.0;
    const double damping = .01*stiffness;
    const double pretension = 0.0;

    const tgSpringCableActuator::Config stringConfig(stiffness, damping, pretension, false, 7000, 24);
    
    
    const double passivePretension = 700; // 5 N
    tgSpringCableActuator::Config muscleConfig(2000, 20, passivePretension);
    
    // Calculations for the flemons spine model
    double v_size = 10.0;

    //Create basic unit for right leg
    tgStructure rightLeg;

    //Right Leg nodes: 
    rightLeg.addNode(0,0,0); //0: Bottom Center of lower leg segment
    rightLeg.addNode(0,10,0);  //1: Center of lower leg segment
    rightLeg.addNode(10,10,0); //2: Right of lower leg segment
    rightLeg.addNode(-10,10,0);  //3: Left of lower leg segment
    rightLeg.addNode(0,20,0);  //4: Top of lower leg segment
    rightLeg.addNode(12,2,3);  //5: Big toe; was y=0, z=5
    rightLeg.addNode(12,1.5,-3.5); //6: Little toe; was y=0, z=-5
    rightLeg.addNode(0,10,5);  //7: Outer ankle
    rightLeg.addNode(0,10,-5); //8: Inner ankle
 
    //Adding some extra nodes to stabilize base of leg:
    rightLeg.addNode(4,0,0); //9
    rightLeg.addNode(-4,0,0);  //10
    rightLeg.addNode(0,0,4); //11
    rightLeg.addNode(0,0,-4);  //12
 
    //Adding a toe extension
    rightLeg.addNode(15,0,5); //13: Big toe
    rightLeg.addNode(14,0,-5); //14: Little toe

    //Add rods for right leg:
    rightLeg.addPair(0,1,"rod");
    rightLeg.addPair(1,2,"rod");
    rightLeg.addPair(1,3,"rod");
    rightLeg.addPair(1,4,"rod");
    rightLeg.addPair(5,8,"rod");
    rightLeg.addPair(6,7,"rod");
    //Toe extension rod
    rightLeg.addPair(5,13,"rod");
    rightLeg.addPair(6,14,"rod");

    rightLeg.addPair(0,9,"rod");
    rightLeg.addPair(0,10,"rod");
    rightLeg.addPair(0,11,"rod");
    rightLeg.addPair(0,12,"rod");

    //Create basic unit for left leg
    tgStructure leftLeg;

    //Left Leg nodes: 
    leftLeg.addNode(0,0,0); //0: Bottom Center of lower leg segment
    leftLeg.addNode(0,10,0);  //1: Center of lower leg segment
    leftLeg.addNode(10,10,0); //2: Right of lower leg segment
    leftLeg.addNode(-10,10,0);  //3: Left of lower leg segment
    leftLeg.addNode(0,20,0);  //4: Top of lower leg segment
    leftLeg.addNode(12,2,3);  //5: Little toe; was y=0, z=5
    leftLeg.addNode(12,1.5,-3.5); //6: Big toe; was y=0, z=-5
    leftLeg.addNode(0,10,5);  //7: Inner ankle
    leftLeg.addNode(0,10,-5); //8: Outer ankle
 
    //Adding some extra nodes to stabilize base of leg:
    leftLeg.addNode(4,0,0); //9 
    leftLeg.addNode(-4,0,0);  //10
    leftLeg.addNode(0,0,4); //11
    leftLeg.addNode(0,0,-4);  //12
 
    //Adding a toe extension
    leftLeg.addNode(15,0,5); //13: Little toe
    leftLeg.addNode(14,0,-5); //14: Big toe

    //Add rods for left leg:
    leftLeg.addPair(0,1,"rod");
    leftLeg.addPair(1,2,"rod");
    leftLeg.addPair(1,3,"rod");
    leftLeg.addPair(1,4,"rod");
    leftLeg.addPair(5,8,"rod");
    leftLeg.addPair(6,7,"rod");
    //Toe extension rod
    leftLeg.addPair(5,13,"rod");
    leftLeg.addPair(6,14,"rod");

    leftLeg.addPair(0,9,"rod");
    leftLeg.addPair(0,10,"rod");
    leftLeg.addPair(0,11,"rod");
    leftLeg.addPair(0,12,"rod");

    //Create the basic unit of the spine
    tgStructure tetra;

    //Add the nodes
    tetra.addNode(0,0,0); //Node 0 
    tetra.addNode(v_size, 0, v_size); //Node 1 
    tetra.addNode(v_size, 0, -v_size); //Node 2
    tetra.addNode(-v_size, 0, -v_size); //Node 3
    tetra.addNode(-v_size, 0, v_size); //Node 4

    tetra.addPair(0,1,"rod");    
    tetra.addPair(0,2,"rod");    
    tetra.addPair(0,3,"rod");    
    tetra.addPair(0,4,"rod");    

    //Create the basic unit for the hips/shoulders:
    tgStructure lHip;

    lHip.addNode(0,0,0); //Node 0 
    lHip.addNode(0, v_size, v_size); //Node 1 
    lHip.addNode(0, -v_size, -v_size); //Node 2
    lHip.addNode(0, -v_size, v_size); //Node 3
 
    lHip.addPair(0,1,"rod");    
    lHip.addPair(0,2,"rod");    
    lHip.addPair(0,3,"rod"); 

    tgStructure rHip;

    rHip.addNode(0,0,0); //Node 0 
    rHip.addNode(0, v_size, -v_size); //Node 1 
    rHip.addNode(0, -v_size, -v_size); //Node 2
    rHip.addNode(0, -v_size, v_size); //Node 3
 
    rHip.addPair(0,1,"rod");    
    rHip.addPair(0,2,"rod");    
    rHip.addPair(0,3,"rod"); 

    //Build the spine
    tgStructure spine;
    const double offsetDist = v_size + 1; //So rod ends don't touch, may need to adjust
    const double offsetDist2 = v_size*5 + 5 + 3.3; 
    const double offsetDist3 = v_size*6; 
    std::size_t m_segments = 6;
    std::size_t m_hips = 4;
    std::size_t m_legs = 4; 
    btVector3 offset(offsetDist,0.0,0);
    btVector3 offset1(offsetDist*2,0.0,offsetDist);
    btVector3 offset2(offsetDist2,0.0,offsetDist);
    btVector3 offset3(offsetDist*2,0.0,-offsetDist);
    btVector3 offset4(offsetDist2,0.0,-offsetDist);
    btVector3 offset5(offsetDist3,-21.0,offsetDist);
    btVector3 offset6(offsetDist3,-21.0,-offsetDist);
    btVector3 offset7(v_size*2,-21.0,offsetDist);
    btVector3 offset8(v_size*2,-21.0,-offsetDist);
    
    for(std::size_t i = 0; i < m_segments; i++) { //Connect segments for spine
        tgStructure* t = new tgStructure (tetra);
        t->addTags(tgString("segment num", i + 1));
        t->move((i + 1)*offset);

        if (i % 2 == 1){

            t->addRotation(btVector3((i + 1) * offsetDist, 0.0, 0.0), btVector3(1, 0, 0), 0.0); 

        }
        else{

            t->addRotation(btVector3((i + 1) * offsetDist, 0.0, 0.0), btVector3(1, 0, 0), M_PI/2.0); 

        }

        spine.addChild(t); //Add a segment to the spine
    }

    for(std::size_t i = m_segments; i < (m_segments + 2); i++) {//deal with right hip and shoulder first
        tgStructure* t = new tgStructure (rHip);
        t->addTags(tgString("segment num", i + 1));
 
        if(i % 2 == 0){
            t->move(offset1);
            t->addRotation(btVector3(offsetDist*2, 0.0, offsetDist), btVector3(1, 0, 0), 0.0);
        }
        else{
            t->move(offset2);
            t->addRotation(btVector3(offsetDist2, 0.0, offsetDist), btVector3(0, 0, 1), M_PI*1/8);
        }

        spine.addChild(t); //Add a segment to the spine
    }

    for(std::size_t i = (m_segments + 2); i < (m_segments + m_hips); i++) {//deal with left hip and shoulder now
        tgStructure* t = new tgStructure (lHip);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){ 
            t->move(offset3);
            t->addRotation(btVector3(offsetDist*2, 0.0, -offsetDist), btVector3(1, 0, 0), 0.0);
        }
        else{
            t->move(offset4);
            t->addRotation(btVector3(offsetDist2, 0.0, -offsetDist), btVector3(0, 0, 1), M_PI*1/8);
        }

        spine.addChild(t); //Add a segment to the spine

    }

    for(std::size_t i = (m_segments + m_hips); i < (m_segments + m_hips + 2); i++) {//right front and back legs
        tgStructure* t = new tgStructure (rightLeg);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset7);
            t->addRotation(btVector3(v_size*2, -21.0, offsetDist), btVector3(0, 1, 0), M_PI);
        }
        else{
            t->move(offset5);
            t->addRotation(btVector3(offsetDist3, -21.0, offsetDist), btVector3(0, 1, 0), M_PI);
        }

        spine.addChild(t); //Add a segment to the spine
    }

    for(std::size_t i = (m_segments + m_hips + 2); i < (m_segments + m_hips + m_legs); i++) {//left front and back legs
        tgStructure* t = new tgStructure (leftLeg);
        t->addTags(tgString("segment num", i + 1));

        if(i % 2 == 0){
            t->move(offset8);
            t->addRotation(btVector3(v_size*2, -21.0, -offsetDist), btVector3(0, 1, 0), M_PI);
        }
        else{
            t->move(offset6);
            t->addRotation(btVector3(offsetDist3, -21.0, -offsetDist), btVector3(0, 1, 0), M_PI);
        }

        spine.addChild(t); //Add a segment to the spine
    }

    spine.move(btVector3(0.0,21.0,0.0));


    std::vector<tgStructure*> children = spine.getChildren();
    for(std::size_t i = 2; i < (children.size() - (m_hips + m_legs)); i++) { 

        tgNodes n0 = children[i-2]->getNodes();
        tgNodes n1 = children[i-1]->getNodes();
        tgNodes n2 = children[i]->getNodes();
        

        //Add muscles to the spine
        if(i < 3){
            if(i % 2 == 0){ //front
                spine.addPair(n0[1], n1[3], tgString("spine front lower right muscle seg", i-2) + tgString(" seg", i-1));
                spine.addPair(n0[1], n1[4], tgString("spine front lower left muscle seg", i-2) + tgString(" seg", i-1));
                spine.addPair(n0[2], n1[3], tgString("spine front upper right muscle seg", i-2) + tgString(" seg", i-1));
                spine.addPair(n0[2], n1[4], tgString("spine front upper left muscle seg", i-2) + tgString(" seg", i-1));
            }
            else{ //rear
                spine.addPair(n0[1], n1[3], tgString("spine rear upper left muscle seg", i-2) + tgString(" seg", i-1));
                spine.addPair(n0[1], n1[4], tgString("spine rear lower left muscle seg", i-2) + tgString(" seg", i-1));
                spine.addPair(n0[2], n1[3], tgString("spine rear upper right muscle seg", i-2) + tgString(" seg", i-1));
                spine.addPair(n0[2], n1[4], tgString("spine rear lower right muscle seg", i-2) + tgString(" seg", i-1)); 
            }
        }
        if(i < 6){
            if(i % 2 == 0){
                spine.addPair(n0[1], n2[4], tgString("spine bottom muscle seg", i-2) + tgString(" seg", i-1));
                spine.addPair(n0[2], n2[3], tgString("spine top muscle seg", i-2) + tgString(" seg", i-1));
            }
            else{
                spine.addPair(n0[1], n2[4], tgString("spine lateral left muscle seg", i-2) + tgString(" seg", i-1));
                spine.addPair(n0[2], n2[3], tgString("spine lateral right muscle seg", i-2) + tgString(" seg", i-1));

            }
        }
        if(i > 0 && i < 5){
            if(i % 2 == 0){//rear
                spine.addPair(n1[1], n2[3], tgString("spine rear upper left muscle seg", i-1) + tgString(" seg", i));
                spine.addPair(n1[1], n2[4], tgString("spine rear lower left muscle seg", i-1) + tgString(" seg", i));
                spine.addPair(n1[2], n2[3], tgString("spine rear upper right muscle seg", i-1) + tgString(" seg", i));
                spine.addPair(n1[2], n2[4], tgString("spine rear lower right muscle seg", i-1) + tgString(" seg", i));
            }
            else{//front

                spine.addPair(n1[1], n2[3], tgString("spine front lower right muscle seg", i-1) + tgString(" seg", i));
                spine.addPair(n1[1], n2[4], tgString("spine front lower left muscle seg", i-1) + tgString(" seg", i));
                spine.addPair(n1[2], n2[3], tgString("spine front upper right muscle seg", i-1) + tgString(" seg", i));
                spine.addPair(n1[2], n2[4], tgString("spine front upper left muscle seg", i-1) + tgString(" seg", i));
            }
        }
        if(i == 5){
            //rear
            spine.addPair(n1[1], n2[1], tgString("spine rear lower left muscle seg", i-1) + tgString(" seg", i));
            spine.addPair(n1[1], n2[2], tgString("spine rear lower right muscle seg", i-1) + tgString(" seg", i));
            spine.addPair(n1[2], n2[1], tgString("spine rear upper left muscle seg", i-1) + tgString(" seg", i));
            spine.addPair(n1[2], n2[2], tgString("spine rear upper right muscle seg", i-1) + tgString(" seg", i));   
            //front
            spine.addPair(n1[1], n2[3], tgString("spine front lower right muscle seg", i-1) + tgString(" seg", i));
            spine.addPair(n1[1], n2[4], tgString("spine front lower left muscle seg", i-1) + tgString(" seg", i));
            spine.addPair(n1[2], n2[3], tgString("spine front upper right muscle seg", i-1) + tgString(" seg", i));
            spine.addPair(n1[2], n2[4], tgString("spine front upper left muscle seg", i-1) + tgString(" seg", i));
 
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
    
    //Left shoulder muscles
    spine.addPair(n6[1], n1[1], tgString("left shoulder rear upper muscle seg", 6) + tgString(" seg", 1));
    spine.addPair(n6[1], n1[4], tgString("left shoulder front upper muscle seg", 6) + tgString(" seg", 1));
    spine.addPair(n6[1], n0[2], tgString("left shoulder front top muscle seg", 6) + tgString(" seg", 0));
    spine.addPair(n6[1], n2[3], tgString("left shoulder rear top muscle seg", 6) + tgString(" seg", 2));

    spine.addPair(n6[2], n1[1], tgString("left shoulder rear lower muscle seg", 6) + tgString(" seg", 1));
    spine.addPair(n6[2], n1[4], tgString("left shoulder front lower muscle seg", 6) + tgString(" seg", 1));
    spine.addPair(n6[2], n0[1], tgString("left shoulder front bottom muscle seg", 6) + tgString(" seg", 0));
    spine.addPair(n6[2], n2[4], tgString("left shoulder rear bottom muscle seg", 6) + tgString(" seg", 2));

    //Extra muscles, to move left shoulder forward and back:
    spine.addPair(n6[0], n1[1], tgString("left shoulder rear mid muscle seg", 6) + tgString(" seg", 1));
    spine.addPair(n6[0], n1[4], tgString("left shoulder front mid muscle seg", 6) + tgString(" seg", 1));

    //Left hip muscles
    spine.addPair(n7[1], n5[1], tgString("left hip rear upper muscle seg", 7) + tgString(" seg", 5));
    spine.addPair(n7[1], n5[4], tgString("left hip front upper muscle seg", 7) + tgString(" seg", 5));
    spine.addPair(n7[1], n4[2], tgString("left hip rear top muscle seg", 7) + tgString(" seg", 4));
    spine.addPair(n7[1], n4[3], tgString("left hip front top muscle seg", 7) + tgString(" seg", 4));

    spine.addPair(n7[2], n5[1], tgString("left hip rear lower muscle seg", 7) + tgString(" seg", 5));
    spine.addPair(n7[2], n5[4], tgString("left hip front lower muscle seg", 7) + tgString(" seg", 5));
    spine.addPair(n7[2], n4[1], tgString("left hip bottom muscle seg", 7) + tgString(" seg", 4));

    //Extra muscles, to move left hip forward and back:
    spine.addPair(n7[0], n3[1], tgString("left hip rear mid muscle seg", 7) + tgString(" seg", 3)); //could also be n3[3]
    spine.addPair(n7[0], n5[4], tgString("left hip front mid muscle seg", 7) + tgString(" seg", 5));

    //Inter-hip connector muscle
    spine.addPair(n7[2], n9[3], tgString("inter-hip bottom muscle seg", 7) + tgString(" seg", 9)); //inter-hip bottom muscle

    //Right shoulder muscles
    spine.addPair(n8[1], n1[2], tgString("right shoulder rear upper muscle seg", 8) + tgString(" seg", 1));
    spine.addPair(n8[1], n1[3], tgString("right shoulder front upper muscle seg", 8) + tgString(" seg", 1));
    spine.addPair(n8[1], n0[2], tgString("right shoulder front top muscle seg", 8) + tgString(" seg", 0));
    spine.addPair(n8[1], n2[3], tgString("right shoulder rear top muscle seg", 8) + tgString(" seg", 2));

    spine.addPair(n8[3], n1[2], tgString("right shoulder rear lower muscle seg", 8) + tgString(" seg", 1));
    spine.addPair(n8[3], n1[3], tgString("right shoulder front lower muscle seg", 8) + tgString(" seg", 1));
    spine.addPair(n8[3], n0[1], tgString("right shoulder front bottom muscle seg", 8) + tgString(" seg", 0));
    spine.addPair(n8[3], n2[4], tgString("right shoulder rear bottom muscle seg", 8) + tgString(" seg", 2));

    //Extra muscles, to move right shoulder forward and back:
    spine.addPair(n8[0], n1[2], tgString("right shoulder rear mid muscle seg", 8) + tgString(" seg", 1));
    spine.addPair(n8[0], n1[3], tgString("right shoulder front mid muscle seg", 8) + tgString(" seg", 1));

    //Right hip muscles
    spine.addPair(n9[1], n5[2], tgString("right hip rear upper muscle seg", 9) + tgString(" seg", 5));
    spine.addPair(n9[1], n5[3], tgString("right hip front upper muscle seg", 9) + tgString(" seg", 5));
    spine.addPair(n9[1], n4[2], tgString("right hip rear top muscle seg", 9) + tgString(" seg", 4));
    spine.addPair(n9[1], n4[3], tgString("right hip front top muscle seg", 9) + tgString(" seg", 4));

    spine.addPair(n9[3], n5[2], tgString("right hip rear lower muscle seg", 9) + tgString(" seg", 5));
    spine.addPair(n9[3], n5[3], tgString("right hip front lower muscle seg", 9) + tgString(" seg", 5));
    spine.addPair(n9[3], n4[1], tgString("right hip bottom muscle seg", 9) + tgString(" seg", 4));  

    //Extra muscles, to move right hip forward and back:
    spine.addPair(n9[0], n3[2], tgString("right hip rear mid muscle seg", 9) + tgString(" seg", 3)); //could also be n3[3]
    spine.addPair(n9[0], n5[3], tgString("right hip front mid muscle seg", 9) + tgString(" seg", 5));

    //Leg/hip connections:

    //Right front leg/shoulder
    spine.addPair(n10[4], n6[2], tgString("right outer bicep muscle seg", 10) + tgString(" seg", 6));
    spine.addPair(n10[4], n6[3], tgString("right inner bicep muscle seg", 10) + tgString(" seg", 6));
    //spine.addPair(n10[4], n1[1], tgString("right front abdomen connection muscle seg", 10) + tgString(" seg", 1));

    spine.addPair(n10[3], n6[2], tgString("right outer tricep muscle seg", 10) + tgString(" seg", 6));
    spine.addPair(n10[3], n6[3], tgString("right inner tricep muscle seg", 10) + tgString(" seg", 6));

    spine.addPair(n10[2], n6[2], tgString("right outer front tricep muscle seg", 10) + tgString(" seg", 6));
    spine.addPair(n10[2], n6[3], tgString("right inner front tricep muscle seg", 10) + tgString(" seg", 6));

    //Adding muscle to pull up on right front leg:
    spine.addPair(n10[4], n6[1], tgString("right mid bicep muscle seg", 10) + tgString(" seg", 6));
    
    //Left front leg/shoulder
    spine.addPair(n12[4], n8[2], tgString("left inner bicep muscle seg", 12) + tgString(" seg", 8));
    spine.addPair(n12[4], n8[3], tgString("left outer bicep muscle seg", 12) + tgString(" seg", 8));
    //spine.addPair(n12[4], n1[2], tgString("left front abdomen connection muscle seg", 12) + tgString(" seg", 1)); //Was n1[2]

    spine.addPair(n12[3], n8[2], tgString("left inner tricep muscle seg", 12) + tgString(" seg", 8));
    spine.addPair(n12[3], n8[3], tgString("left outer tricep muscle seg", 12) + tgString(" seg", 8));

    spine.addPair(n12[2], n8[2], tgString("left inner front tricep muscle seg", 12) + tgString(" seg", 8));
    spine.addPair(n12[2], n8[3], tgString("left outer front tricep muscle seg", 12) + tgString(" seg", 8));

    //Adding muscle to pull up on left front leg:
    spine.addPair(n12[4], n8[1], tgString("left mid bicep muscle seg", 12) + tgString(" seg", 8));

    //Right rear leg/hip
    spine.addPair(n11[4], n7[2], tgString("right outer thigh muscle seg", 11) + tgString(" seg", 7)); 
    spine.addPair(n11[4], n7[3], tgString("right inner thigh muscle seg", 11) + tgString(" seg", 7));
    //spine.addPair(n11[4], n3[1],tgString("right rear abdomen connection muscle seg", 11) + tgString(" seg", 4)); 

    spine.addPair(n11[3], n7[2], tgString("right outer calf muscle seg", 11) + tgString(" seg", 7));
    spine.addPair(n11[3], n7[3], tgString("right inner calf muscle seg", 11) + tgString(" seg", 7));

    spine.addPair(n11[2], n7[2], tgString("right outer front calf muscle seg", 11) + tgString(" seg", 7));
    spine.addPair(n11[2], n7[3], tgString("right inner front calf muscle seg", 11) + tgString(" seg", 7));

    //Adding muscle to pull rear right leg up:
    spine.addPair(n11[4], n7[1], tgString("right central thigh muscle seg", 11) + tgString(" seg", 7));

    //Left rear leg/hip
    spine.addPair(n13[4], n9[2], tgString("left inner thigh muscle seg", 13) + tgString(" seg", 9)); 
    spine.addPair(n13[4], n9[3], tgString("left outer thigh muscle seg", 13) + tgString(" seg", 9));
    //spine.addPair(n13[4], n3[2], tgString("left rear abdomen connection muscle seg", 13) + tgString(" seg", 4)); 

    spine.addPair(n13[3], n9[2], tgString("left inner calf muscle seg", 13) + tgString(" seg", 9));
    spine.addPair(n13[3], n9[3], tgString("left outer calf muscle seg", 13) + tgString(" seg", 9));

    spine.addPair(n13[2], n9[2], tgString("left inner front calf muscle seg", 13) + tgString(" seg", 9));
    spine.addPair(n13[2], n9[3], tgString("left outer front calf muscle seg", 13) + tgString(" seg", 9));

    //Adding muscle to pull rear left leg up:
    spine.addPair(n13[4], n9[1], tgString("left central thigh muscle seg", 13) + tgString(" seg", 9));

    //Populate Legs with muscles
    for(std::size_t i = (m_segments + m_hips); i < children.size(); i++) { 
        
        tgNodes ni = children[i]->getNodes();

        if(i < (m_segments + m_legs + 2)){
            
            spine.addPair(ni[5], ni[7], tgString("right leg big outer muscle seg", i));
            spine.addPair(ni[6], ni[8], tgString("right leg little inner muscle seg", i));
            spine.addPair(ni[5], ni[6], tgString("right leg big little muscle seg", i));
            spine.addPair(ni[0], ni[7], tgString("right leg lower outer muscle seg", i));
            spine.addPair(ni[0], ni[8], tgString("right leg lower inner muscle seg", i));
            spine.addPair(ni[2], ni[7], tgString("right leg front outer muscle seg", i));
            spine.addPair(ni[2], ni[8], tgString("right leg front inner muscle seg", i));
            spine.addPair(ni[3], ni[7], tgString("right leg heel outer muscle seg", i));
            spine.addPair(ni[3], ni[8], tgString("right leg heel inner muscle seg", i));
            spine.addPair(ni[4], ni[7], tgString("right leg upper outer muscle seg", i));
            spine.addPair(ni[4], ni[8], tgString("right leg upper inner muscle seg", i));

        }
        else{

            spine.addPair(ni[5], ni[7], tgString("left leg litte inner muscle seg", i));
            spine.addPair(ni[6], ni[8], tgString("left leg big outer muscle seg", i));
            spine.addPair(ni[5], ni[6], tgString("left leg big little muscle seg", i));
            spine.addPair(ni[0], ni[7], tgString("left leg lower inner muscle seg", i));
            spine.addPair(ni[0], ni[8], tgString("left leg lower outer muscle seg", i));
            spine.addPair(ni[2], ni[7], tgString("left leg front inner muscle seg", i));
            spine.addPair(ni[2], ni[8], tgString("left leg front outer muscle seg", i));
            spine.addPair(ni[3], ni[7], tgString("left leg heel inner muscle seg", i));
            spine.addPair(ni[3], ni[8], tgString("left leg heel outer muscle seg", i));
            spine.addPair(ni[4], ni[7], tgString("left leg upper inner muscle seg", i));
            spine.addPair(ni[4], ni[8], tgString("left leg upper outer muscle seg", i));

        }        

    }

    //Extra muscles, to keep front vertebra from swinging. May need to change label to "fascia". Also need to fold this into loop above, for consistency
    spine.addPair(n0[3], n1[3], tgString("spine front upper right muscle seg", 0) + tgString(" seg", 1));
    spine.addPair(n0[3], n1[4], tgString("spine front upper left muscle seg", 0) + tgString(" seg", 1));


    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(spine, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);

    children.clear();
}

void BigPuppy::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

const std::vector<tgSpringCableActuator*>& BigPuppy::getAllActuators() const
{
    return allActuators;
}

void BigPuppy::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
