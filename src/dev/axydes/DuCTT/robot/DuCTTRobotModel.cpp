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
 * @file DuCTTRobotModel.cpp
 * @brief Contains the definition of the members of the class DuCTTModel.
 * $Id$
 */

// This module
#include "DuCTTRobotModel.h"
#include "tgDuCTTHingeInfo.h"
#include "tgPrismaticInfo.h"
#include "tgTouchSensorRodInfo.h"
#include "tgTouchSensorSphereInfo.h"
#include "tgTouchSensorModel.h"

// The NTRT Core Libary
#include "core/abstractMarker.h"
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "core/tgSphere.h"

// The NTRT Creator Libary
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgSphereInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"

// The Bullet Physics library
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"

// The C++ Standard Library
#include <stdexcept>

DuCTTRobotModel::Config::Config(
    //robot params
    btVector3 startPos,
    btVector3 startRotAxis,
    double startRotAngle,
    //tetra prams
    double triangle_length,
    double duct_distance,
    double duct_height,
    //rod params
    double prismRadius,
    double prismExtent,
    double prismDensity,
    double vertRodRadius,
    double vertDensity,
    double innerRodRadius,
    double innerDensity,
    //sphere tip params
    double tipRad,
    double tipDens,
    double tipFric,
    //string params
    double stiffness,
    double damping,
    double pretension,
    double maxVertStringVel,
    double maxSaddleStringVel,
    double maxStringForce,
    double maxStringAcc,
    double minStringRestLength,
    bool storeStringHist,
    bool useCylinderEndCaps,
    bool debug
    ) :
m_startPos(startPos),
m_startRotAxis(startRotAxis),
m_startRotAngle(startRotAngle),
m_triangle_length(triangle_length),
m_duct_distance(duct_distance),
m_duct_height(duct_height),
m_prismDensity(prismDensity),
m_prismRadius(prismRadius),
m_prismExtent(prismExtent),
m_vertRodRadius(vertRodRadius),
m_vertDensity(vertDensity),
m_innerRodRadius(innerRodRadius),
m_innerDensity(innerDensity),
m_tipRad(tipRad),
m_tipDens(tipDens),
m_tipFric(tipFric),
m_stiffness(stiffness),
m_damping(damping),
m_pretension(pretension),
m_maxVertStringVel(maxVertStringVel),
m_maxSaddleStringVel(maxSaddleStringVel),
m_maxStringForce(maxStringForce),
m_maxStringAcc(maxStringAcc),
m_minStringRestLength(minStringRestLength),
m_storeStringHist(storeStringHist),
m_useCylinderEndCaps(useCylinderEndCaps),
m_debug(debug)
{
}

DuCTTRobotModel::DuCTTRobotModel() :
    m_config(DuCTTRobotModel::Config()),
    tgModel()
{
}

DuCTTRobotModel::DuCTTRobotModel(DuCTTRobotModel::Config &config) :
    m_config(config),
    tgModel()
{
}

DuCTTRobotModel::~DuCTTRobotModel()
{
}

void DuCTTRobotModel::addNodes(tgStructure& tetra,
                            double edge,
                            double distance,
                            double height,
                            bool isGhost,
                            double distBtHinges,
                            double distBtNodes
                               )
{
    //Main nodes
    btVector3 bottomRight = btVector3(edge/2.0, distance, 0);
    btVector3 bottomLeft = btVector3(-edge/2.0, distance, 0);

    btVector3 topFront = btVector3(0, distance+height, edge/2.0);
    btVector3 topBack = btVector3(0, distance+height, -edge/2.0);

    //Prismatic nodes
    btVector3 bottomMidRight = btVector3(0.01, distance, 0);
    btVector3 bottomMidLeft = btVector3(-0.01, distance, 0);

    btVector3 topMidFront = btVector3(0, distance+height, 0.01);
    btVector3 topMidBack = btVector3(0, distance+height, -0.01);

    //touch sensor rod nodes
    btVector3 bottomRightTouch = bottomRight;
    bottomRightTouch.setX(bottomRight.x()+m_config.m_tipRad);
    btVector3 bottomLeftTouch = bottomLeft;
    bottomLeftTouch.setX(bottomLeft.x()-m_config.m_tipRad);

    btVector3 topFrontTouch = topFront;
    topFrontTouch.setZ(topFront.z()+m_config.m_tipRad);
    btVector3 topBackTouch = topBack;
    topBackTouch.setZ(topBack.z()-m_config.m_tipRad);

    if (isGhost)
    {
        bottomRightTouch.setX(bottomRightTouch.x()+0.01);
        bottomLeftTouch.setX(bottomLeftTouch.x()-0.01);
        topFrontTouch.setZ(topFrontTouch.z()+0.01);
        topBackTouch.setZ(topBackTouch.z()-0.01);
    }

    //bottom Hinge nodes
    btVector3 RightFrontSlope = topFront - bottomRight;
    btVector3 LeftFrontSlope = topFront - bottomLeft;
    btVector3 RightBackSlope = topBack - bottomRight;
    btVector3 LeftBackSlope = topBack - bottomLeft;

    btVector3 bottomRightFront = bottomRight + distBtHinges*RightFrontSlope;
    btVector3 bottomLeftFront = bottomLeft + distBtHinges*LeftFrontSlope;
    btVector3 bottomRightBack = bottomRight + distBtHinges*RightBackSlope;
    btVector3 bottomLeftBack = bottomLeft + distBtHinges*LeftBackSlope;
    bottomRight = bottomRight - btVector3(distBtNodes,0,0);
    bottomLeft = bottomLeft + btVector3(distBtNodes,0,0);

    //top Hinge nodes
    btVector3 topRightFront = topFront - distBtHinges*RightFrontSlope;
    btVector3 topLeftFront = topFront - distBtHinges*LeftFrontSlope;
    btVector3 topRightBack = topBack - distBtHinges*RightBackSlope;
    btVector3 topLeftBack = topBack - distBtHinges*LeftBackSlope;
    topFront = topFront - btVector3(0,0,distBtNodes);
    topBack = topBack + btVector3(0,0,distBtNodes);

    if (distance == 0)
    {
        if (m_config.m_useCylinderEndCaps)
        {
            tetra.addNode(bottomRight); // 0
            tetra.addNode(bottomLeft); // 1
        }
        else
        {
            tetra.addNode(bottomRight.x(), bottomRight.y(), bottomRight.z(), "sphere right touch bottom"); // 0
            tetra.addNode(bottomLeft.x(), bottomLeft.y(), bottomLeft.z(), "sphere left touch bottom"); // 1
        }

        tetra.addNode(topBack); // 2
        tetra.addNode(topFront); // 3
    }
    else
    {
        tetra.addNode(bottomRight); // 0
        tetra.addNode(bottomLeft); // 1

        if (m_config.m_useCylinderEndCaps)
        {
            tetra.addNode(topBack); // 2
            tetra.addNode(topFront); // 3
        }
        else
        {
            tetra.addNode(topBack.x(), topBack.y(), topBack.z(), "sphere back touch top"); // 2
            tetra.addNode(topFront.x(), topFront.y(), topFront.z(), "sphere front touch top"); // 3
        }
    }

    tetra.addNode(bottomMidRight); // 4
    tetra.addNode(bottomMidLeft); // 5

    tetra.addNode(topMidBack); // 6
    tetra.addNode(topMidFront); // 7

    tetra.addNode(bottomRightFront); // 8
    tetra.addNode(bottomLeftFront); // 9

    tetra.addNode(bottomRightBack); // 10
    tetra.addNode(bottomLeftBack); // 11

    tetra.addNode(topRightFront); // 12
    tetra.addNode(topLeftFront); // 13

    tetra.addNode(topRightBack); // 14
    tetra.addNode(topLeftBack); // 15

    //touch sensor nodes
    tetra.addNode(bottomRightTouch); // 16
    tetra.addNode(bottomLeftTouch); // 17
    tetra.addNode(topBackTouch); // 18
    tetra.addNode(topFrontTouch); // 19
}

void DuCTTRobotModel::addPairs(tgStructure& s, int startNode)
{
    std::string tetra = " bottom";
    if (startNode != 0)
        tetra = " top";
    // for one tetra
    //right rods
    s.addPair( startNode+8, startNode+12, "vert rod"+tetra);
    s.addPair( startNode+10, startNode+14, "vert rod"+tetra);

    //left rods
    s.addPair( startNode+9, startNode+13, "vert rod"+tetra);
    s.addPair( startNode+11, startNode+15, "vert rod"+tetra);

    if (startNode == 0)
    {
        //bottom tetra
        // bottom rods
        s.addPair( startNode+0, startNode+4, "prism rod"+tetra);
        s.addPair( startNode+5, startNode+1, "prism rod"+tetra);

        //top rods
        s.addPair( startNode+2, startNode+3, "inner rod"+tetra);

        s.addPair( startNode+4, startNode+5, "prismatic"+tetra);

        //bottom right hinges
        s.addPair( startNode+0, startNode+8, "hinge"+tetra);
        s.addPair( startNode+0, startNode+10, "hinge"+tetra);

        //bottom left hinges
        s.addPair( startNode+1, startNode+9, "hinge"+tetra);
        s.addPair( startNode+1, startNode+11, "hinge"+tetra);

        //top front hinges
        s.addPair( startNode+3, startNode+12, "hinge3"+tetra);
        s.addPair( startNode+3, startNode+13, "hinge3"+tetra);

        //top back hinges
        s.addPair( startNode+2, startNode+14, "hinge3"+tetra);
        s.addPair( startNode+2, startNode+15, "hinge3"+tetra);

        if (m_config.m_useCylinderEndCaps)
        {
            //touch sensor rods
            s.addPair( startNode+16, startNode+0, "rod right touch"+tetra);
            s.addPair( startNode+1, startNode+17, "rod left touch"+tetra);
        }
    }
    else
    {
        //top tetra
        // bottom rods
        s.addPair( startNode+0, startNode+1, "inner rod"+tetra);

        //top rods
        s.addPair( startNode+2, startNode+6, "prism rod"+tetra);
        s.addPair( startNode+7, startNode+3, "prism rod"+tetra);

        s.addPair( startNode+6, startNode+7, "prismatic"+tetra);

        //bottom right hinges
        s.addPair( startNode+0, startNode+8, "hinge3"+tetra);
        s.addPair( startNode+0, startNode+10, "hinge3"+tetra);

        //bottom left hinges
        s.addPair( startNode+1, startNode+9, "hinge3"+tetra);
        s.addPair( startNode+1, startNode+11, "hinge3"+tetra);

        //top front hinges
        s.addPair( startNode+3, startNode+12, "hinge2"+tetra);
        s.addPair( startNode+3, startNode+13, "hinge2"+tetra);

        //top back hinges
        s.addPair( startNode+2, startNode+14, "hinge2"+tetra);
        s.addPair( startNode+2, startNode+15, "hinge2"+tetra);

        if (m_config.m_useCylinderEndCaps)
        {
            //touch sensor rods
            s.addPair( startNode+18, startNode+2, "rod back touch"+tetra);
            s.addPair( startNode+3, startNode+19, "rod front touch"+tetra);
        }
    }
}

/**
void DuCTTRobotModel::addMuscles(tgStructure& s, int topNodesStart)
{
    //vertical strings
    s.addPair(0, topNodesStart+0,  "vert string cluster1");
    s.addPair(1, topNodesStart+1,  "vert string cluster2");
    s.addPair(2, topNodesStart+2,  "vert string cluster3");
    s.addPair(3, topNodesStart+3,  "vert string cluster4");

    //saddle strings
    s.addPair(3, topNodesStart+0,  "saddle string cluster5");
    s.addPair(2, topNodesStart+0,  "saddle string cluster6");
    s.addPair(3, topNodesStart+1,  "saddle string cluster7");
    s.addPair(2, topNodesStart+1,  "saddle string cluster8");
}
/**/
void DuCTTRobotModel::addMuscles(tgStructure& s, int topNodesStart)
{
    //vertical strings
    s.addPair(0, topNodesStart+0,  "vert string cluster1");
    s.addPair(1, topNodesStart+1,  "vert string cluster1");
    s.addPair(2, topNodesStart+2,  "vert string cluster1");
    s.addPair(3, topNodesStart+3,  "vert string cluster1");
    
    //saddle strings
    s.addPair(3, topNodesStart+0,  "saddle string cluster2");
    s.addPair(2, topNodesStart+0,  "saddle string cluster2");
    s.addPair(3, topNodesStart+1,  "saddle string cluster2");
    s.addPair(2, topNodesStart+1,  "saddle string cluster2");
}
/**/

void DuCTTRobotModel::setup(tgWorld& world)
{
    setupStructure(world);
    setupGhostStructure(world);
    setupVariables();
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void DuCTTRobotModel::setupStructure(tgWorld &world)
{
    // Define the configurations of the rods and strings
    // rodConfigB has density of 0 so it stays fixed in simulator
    const tgRod::Config prismRodConfig(m_config.m_prismRadius, m_config.m_prismDensity);
    const tgRod::Config vertRodConfig(m_config.m_vertRodRadius, m_config.m_vertDensity);
    const tgRod::Config innerRodConfig(m_config.m_innerRodRadius, m_config.m_innerDensity);
    const tgRod::Config touchRodConfig(m_config.m_tipRad, m_config.m_tipDens, m_config.m_tipFric);

    const tgBasicActuator::Config vertStringConfig(m_config.m_stiffness, m_config.m_damping, m_config.m_pretension,
                                                  m_config.m_storeStringHist, m_config.m_maxStringForce, m_config.m_maxVertStringVel,
                                                  m_config.m_minStringRestLength, m_config.m_minStringRestLength, 0);
    const tgBasicActuator::Config saddleStringConfig(m_config.m_stiffness, m_config.m_damping, m_config.m_pretension,
                                                    m_config.m_storeStringHist, m_config.m_maxStringForce, m_config.m_maxSaddleStringVel,
                                                    m_config.m_minStringRestLength, m_config.m_minStringRestLength, 0);

    btVector3 prismAxisBottom(0,0,1);
    btVector3 prismAxisTop(0,1,0);

    prismAxisBottom = prismAxisBottom.rotate(m_config.m_startRotAxis, m_config.m_startRotAngle);
    prismAxisTop = prismAxisTop.rotate(m_config.m_startRotAxis, m_config.m_startRotAngle);

    tgPrismatic::Config prismConfigBottom(prismAxisBottom, 0, 0.1, m_config.m_prismExtent, 133.45, 0.01016, 0.0254);
    tgPrismatic::Config prismConfigTop(prismAxisTop, M_PI/2.0, 0.1, m_config.m_prismExtent, 133.45, 0.01016, 0.0254);

    if ((m_config.m_startRotAxis == btVector3(0,1,0)) && fabs(m_config.m_startRotAngle - (45*SIMD_RADS_PER_DEG)) < 0.01)
    {
        prismAxisBottom = btVector3(0,1,0);
        prismConfigBottom.m_axis = prismAxisBottom;
        prismConfigBottom.m_rotation = M_PI/4.0;
        prismConfigTop.m_rotation = 3.0*M_PI/4.0;
    }
    else if ((m_config.m_startRotAxis == btVector3(0,1,0)) && fabs(m_config.m_startRotAngle - (-45*SIMD_RADS_PER_DEG)) < 0.01)
    {
        prismAxisBottom = btVector3(0,1,0);
        prismConfigBottom.m_axis = prismAxisBottom;
        prismConfigBottom.m_rotation = -M_PI/4.0;
        prismConfigTop.m_rotation = M_PI/4.0;
    }

    const tgSphere::Config sphereConfig(m_config.m_tipRad, m_config.m_tipDens, m_config.m_tipFric, 0, 0.01);

    //used for mechanical test & validation
    const tgRod::Config staticPrismRodConfig(m_config.m_prismRadius, 0);
    const tgRod::Config staticVertRodConfig(m_config.m_vertRodRadius, 0);
    const tgRod::Config staticInnerRodConfig(m_config.m_innerRodRadius, 0);
    const tgRod::Config staticTouchRodConfig(m_config.m_tipRad, 0, m_config.m_tipFric);
    const tgSphere::Config topsphereConfig(m_config.m_tipRad, m_config.m_tipDens, m_config.m_tipFric, 0, 0.01);
    const tgSphere::Config botsphereConfig(m_config.m_tipRad, 0, m_config.m_tipFric, 0, 0.01);

    btVector3 hinge1Axis(0,0,1);
    btVector3 hinge2Axis(1,0,0);
    btVector3 hinge3Axis(0,1,0);

    hinge1Axis = hinge1Axis.rotate(m_config.m_startRotAxis, m_config.m_startRotAngle);
//    hinge2Axis = hinge2Axis.rotate(m_config.m_startRotAxis, m_config.m_startRotAngle);
//    hinge3Axis = hinge3Axis.rotate(m_config.m_startRotAxis, m_config.m_startRotAngle);

    const tgDuCTTHinge::Config hingeConfig(-SIMD_PI, SIMD_PI, hinge1Axis, false, 0.01, 20, 0.2, 0.9, 0.9, 0);
    const tgDuCTTHinge::Config hingeConfig2(-SIMD_PI, SIMD_PI, hinge2Axis, false, 0.01, 20, 0.2, 0.9, 0.9, 0);
    const tgDuCTTHinge::Config hingeConfig3(-SIMD_PI, SIMD_PI, hinge3Axis, false, 0.01, 20, 0.2, 0.9, 0.9, 0);

    // Create a structure that will hold the details of this model
    tgStructure s;

    // Add nodes to the bottom tetrahedron
    addNodes(s, m_config.m_triangle_length, 0, m_config.m_duct_height);

    // Add rods to the bottom tetrahedron
    addPairs(s);

    // Add nodes to top tetrahedron
    addNodes(s, m_config.m_triangle_length, m_config.m_duct_distance, m_config.m_duct_height);

    // Add rods to the top tetrahedron
    addPairs(s, 20);

    // Add muscles to the structure
    addMuscles(s, 20);

    // Move the structure so it doesn't start in the ground
    s.addRotation(btVector3(0,0,0), m_config.m_startRotAxis, m_config.m_startRotAngle);
    s.move(m_config.m_startPos);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("prism rod ", new tgRodInfo(prismRodConfig));
    spec.addBuilder("vert rod ", new tgRodInfo(vertRodConfig));
    spec.addBuilder("inner rod ", new tgRodInfo(innerRodConfig));

    spec.addBuilder("vert string", new tgBasicActuatorInfo(vertStringConfig));
    spec.addBuilder("saddle string", new tgBasicActuatorInfo(saddleStringConfig));

    spec.addBuilder("prismatic bottom", new tgPrismaticInfo(prismConfigBottom));
    spec.addBuilder("prismatic top", new tgPrismaticInfo(prismConfigTop));

    if (m_config.m_useCylinderEndCaps)
    {
        spec.addBuilder("touch", new tgRodInfo(touchRodConfig));
    }
    else
    {
        spec.addBuilder("touch", new tgSphereInfo(sphereConfig));
    }

    //used for mechanical test & validation
//    spec.addBuilder("prism rod ", new tgRodInfo(staticPrismRodConfig));
//    spec.addBuilder("vert rod ", new tgRodInfo(staticVertRodConfig));
//    spec.addBuilder("inner rod ", new tgRodInfo(staticInnerRodConfig));
//    spec.addBuilder("touch", new tgRodInfo(staticTouchRodConfig));
//    spec.addBuilder("top sphere", new tgSphereInfo(topsphereConfig));
//    spec.addBuilder("bottom sphere", new tgSphereInfo(botsphereConfig));

    spec.addBuilder("hinge", new tgDuCTTHingeInfo(hingeConfig));
    spec.addBuilder("hinge2", new tgDuCTTHingeInfo(hingeConfig2));
    spec.addBuilder("hinge3", new tgDuCTTHingeInfo(hingeConfig3));

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    if (m_config.m_debug)
    {
        std::vector<tgRigidInfo*> allRodInfo = structureInfo.getAllRigids();
        std::cout << "Num rod infos: " << allRodInfo.size() << std::endl;
        for (size_t i=0; i<allRodInfo.size(); i++)
        {
            std::cout << "Rod " << allRodInfo[i]->getTagStr() << " mass: " << allRodInfo[i]->getMass() << std::endl;
        }
    }
}

void DuCTTRobotModel::setupGhostStructure(tgWorld &world)
{
    const tgRod::Config touchRodConfig(m_config.m_tipRad, m_config.m_tipDens, m_config.m_tipFric);
    const tgSphere::Config sphereConfig(m_config.m_tipRad, m_config.m_tipDens, m_config.m_tipFric, 0, 0.01);

    // Create a structure that will hold the details of this model
    tgStructure s;

    // Add nodes to the bottom tetrahedron
    addNodes(s, m_config.m_triangle_length, 0, m_config.m_duct_height, true);

    addPairs(s);

    // Add nodes to top tetrahedron
    addNodes(s, m_config.m_triangle_length, m_config.m_duct_distance, m_config.m_duct_height, true);

    addPairs(s,20);

    // Move the structure so it doesn't start in the ground
    s.move(m_config.m_startPos);
    s.addRotation(m_config.m_startPos, m_config.m_startRotAxis, m_config.m_startRotAngle);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    if (m_config.m_useCylinderEndCaps)
    {
        spec.addBuilder("touch", new tgTouchSensorRodInfo(touchRodConfig));
    }
    else
    {
        spec.addBuilder("touch", new tgTouchSensorSphereInfo(sphereConfig));
    }

    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);
}

void DuCTTRobotModel::setupVariables()
{
    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control.
    allMuscles = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());
    allPrisms = tgCast::filter<tgModel, tgPrismatic> (getDescendants());
    allRods = tgCast::filter<tgModel, tgRod> (getDescendants());

    bottomRods = find<tgRod>("rod bottom");
    topRods = find<tgRod>("rod top");
    prismRods = find<tgRod>("prism rod");
    vertMuscles = find<tgBasicActuator>("vert string");
    saddleMuscles = find<tgBasicActuator>("saddle string");

    allTouchSensors = find<tgTouchSensorModel>("touch");
    bottomTouchSensors = find<tgTouchSensorModel>("touch bottom");
    topTouchSensors = find<tgTouchSensorModel>("touch top");

    if (m_config.m_useCylinderEndCaps)
    {
        std::vector<tgRod*> bottomTouchRods = find<tgRod>("touch bottom");
        std::vector<tgRod*> topTouchRods = find<tgRod>("touch top");

        //attach touch sensors to the appropriate rods on the actual robot model
        //add the appropriate rods to the touch sensor ignore list
        for (size_t i=0; i<bottomTouchSensors.size(); i++)
        {
            if (m_config.m_debug)
                std::cerr << bottomTouchSensors[i]->toString() << std::endl;

            btVector3 offset = bottomTouchSensors[i]->centerOfMass() - bottomTouchRods[i]->centerOfMass();
            abstractMarker marker (bottomTouchRods[i]->getPRigidBody(), offset, btVector3(255,0,0), 0);
            addMarker(marker);
            bottomTouchSensors[i]->addMarker(marker);
            for (size_t j=0; j<bottomRods.size(); j++)
            {
                bottomTouchSensors[i]->addIgnoredObject(bottomRods[j]->getPRigidBody());
            }
        }
        for (size_t i=0; i<topTouchSensors.size(); i++)
        {
            if (m_config.m_debug)
                std::cerr << topTouchSensors[i]->toString() << std::endl;

            btVector3 offset = topTouchSensors[i]->centerOfMass() - topTouchRods[i]->centerOfMass();
            abstractMarker marker (topTouchRods[i]->getPRigidBody(), offset, btVector3(255,0,0), 0);
            addMarker(marker);
            topTouchSensors[i]->addMarker(marker);
            for (size_t j=0; j<topRods.size(); j++)
            {
                topTouchSensors[i]->addIgnoredObject(topRods[j]->getPRigidBody());
            }
        }
    }
    else
    {
        std::vector<tgSphere*> bottomTouchSpheres = find<tgSphere>("touch bottom");
        std::vector<tgSphere*> topTouchSpheres = find<tgSphere>("touch top");

        //attach touch sensors to the appropriate Spheres on the actual robot model
        //add the appropriate Spheres to the touch sensor ignore list
        for (size_t i=0; i<bottomTouchSensors.size(); i++)
        {
            if (m_config.m_debug)
                std::cerr << bottomTouchSensors[i]->toString() << std::endl;

            btVector3 offset = bottomTouchSensors[i]->centerOfMass() - bottomTouchSpheres[i]->centerOfMass();
            abstractMarker marker (bottomTouchSpheres[i]->getPRigidBody(), offset, btVector3(255,0,0), 0);
            addMarker(marker);
            bottomTouchSensors[i]->addMarker(marker);
            for (size_t j=0; j<bottomRods.size(); j++)
            {
                bottomTouchSensors[i]->addIgnoredObject(bottomRods[j]->getPRigidBody());
            }
        }
        for (size_t i=0; i<topTouchSensors.size(); i++)
        {
            if (m_config.m_debug)
                std::cerr << topTouchSensors[i]->toString() << std::endl;

            btVector3 offset = topTouchSensors[i]->centerOfMass() - topTouchSpheres[i]->centerOfMass();
            abstractMarker marker (topTouchSpheres[i]->getPRigidBody(), offset, btVector3(255,0,0), 0);
            addMarker(marker);
            topTouchSensors[i]->addMarker(marker);
            for (size_t j=0; j<topRods.size(); j++)
            {
                topTouchSensors[i]->addIgnoredObject(topRods[j]->getPRigidBody());
            }
        }
    }

    if (m_IgnoredObjs.size() > 0)
    {
        for (size_t i=0; i<m_IgnoredObjs.size(); i++)
        {
            addIgnoredObject(m_IgnoredObjs[i], allTouchSensors);
        }
        m_IgnoredObjs.clear();
    }

    std::vector<tgPrismatic*> bottomPrisms = find<tgPrismatic>("prismatic bottom");
    if (bottomPrisms.size() == 1)
    {
        m_pBottomPrismatic = bottomPrisms[0];
    }

    std::vector<tgPrismatic*> topPrisms = find<tgPrismatic>("prismatic top");
    if (topPrisms.size() == 1)
    {
        m_pTopPrismatic = topPrisms[0];
    }

    std::vector<tgRod*> innerRods = find<tgRod>("inner rod");
    for (size_t i=0; i<innerRods.size(); i++)
    {
        double offsetDist = innerRods[i]->length()/2;
        abstractMarker marker (innerRods[i]->getPRigidBody(), btVector3(0,offsetDist,0), btVector3(255,0,0), 0);
        addMarker(marker);
        abstractMarker marker2 (innerRods[i]->getPRigidBody(), btVector3(0,-offsetDist,0), btVector3(255,0,0), 0);
        addMarker(marker2);
    }

    if (m_config.m_debug)
    {
        std::cout << "Num rods: " << allRods.size() << std::endl;
        double totalMass = 0;
        for (size_t i=0; i<allRods.size(); i++)
        {
            std::cout << "Rod " << allRods[i]->toString() << " mass: " << allRods[i]->mass() << std::endl;
            totalMass += allRods[i]->mass();
        }
        std::cout << "Top Mass: " << getTetraMass(false) << std::endl;
        std::cout << "Bottom Mass: " << getTetraMass(true) << std::endl;
        std::cout << "Total Mass: " << totalMass << std::endl;
    }
}

void DuCTTRobotModel::step(double dt)
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

void DuCTTRobotModel::onVisit(tgModelVisitor& r)
{
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& DuCTTRobotModel::getAllMuscles() const
{
    return allMuscles;
}

const std::vector<tgBasicActuator*>& DuCTTRobotModel::getVertMuscles() const
{
    return vertMuscles;
}

const std::vector<tgBasicActuator*>& DuCTTRobotModel::getSaddleMuscles() const
{
    return saddleMuscles;
}

const std::vector<tgPrismatic*>& DuCTTRobotModel::getAllPrismatics() const
{
    return allPrisms;
}

const std::vector<tgTouchSensorModel*>& DuCTTRobotModel::getAllTouchSensors() const
{
    return allTouchSensors;
}

tgPrismatic* DuCTTRobotModel::getBottomPrismatic()
{
    return m_pBottomPrismatic;
}

tgPrismatic* DuCTTRobotModel::getTopPrismatic()
{
    return m_pTopPrismatic;
}

btVector3 DuCTTRobotModel::getCOM()
{
    std::vector<tgRod*> rods = find<tgRod>("rod");
    assert(!rods.empty());

    btVector3 tetraCenterOfMass(0, 0, 0);
    double tetraMass = 0.0;
    for (std::size_t i = 0; i < rods.size(); i++) {
        const tgRod* const rod = rods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        tetraCenterOfMass += rodCenterOfMass * rodMass;
        tetraMass += rodMass;
    }

    assert(tetraMass > 0.0);
    tetraCenterOfMass /= tetraMass;

    return tetraCenterOfMass;
}

btVector3 DuCTTRobotModel::getTetraCOM(bool bottom)
{
    std::vector<tgRod*> rods;
    if (bottom)
    {
        rods = bottomRods;
    }
    else
    {
        rods = topRods;
    }
    assert(!rods.empty());

    btVector3 tetraCenterOfMass(0, 0, 0);
    double tetraMass = 0.0;
    for (std::size_t i = 0; i < rods.size(); i++) {
        const tgRod* const rod = rods[i];
        assert(rod != NULL);
        const double rodMass = rod->mass();
        const btVector3 rodCenterOfMass = rod->centerOfMass();
        tetraCenterOfMass += rodCenterOfMass * rodMass;
        tetraMass += rodMass;
    }

    assert(tetraMass > 0.0);
    tetraCenterOfMass /= tetraMass;

    return tetraCenterOfMass;
}

double DuCTTRobotModel::mass()
{
    assert(!allRods.empty());
    double totalMass = 0;
    for (size_t i=0; i<allRods.size(); i++)
    {
        totalMass += allRods[i]->mass();
    }
    return totalMass;
}

double DuCTTRobotModel::getTetraMass(bool bottom)
{
    std::vector<tgRod*> rods;
    if (bottom)
    {
        rods = bottomRods;
    }
    else
    {
        rods = topRods;
    }
    assert(!rods.empty());

    double totalMass = 0;
    for (size_t i=0; i<rods.size(); i++)
    {
        totalMass += rods[i]->mass();
    }
    return totalMass;
}

bool DuCTTRobotModel::addIgnoredObject(const btCollisionObject* obj)
{
    if (allTouchSensors.size() > 0)
    {
        addIgnoredObject(obj, allTouchSensors);
    }
    else
    {
        m_IgnoredObjs.push_back(obj);
    }
}

bool DuCTTRobotModel::addIgnoredObject(const btCollisionObject* obj, std::vector<tgTouchSensorModel*> touchSensors)
{
    for (size_t i=0; i<touchSensors.size(); i++)
    {
        touchSensors[i]->addIgnoredObject(obj);
    }
}

void DuCTTRobotModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();

    allMuscles.clear();
    vertMuscles.clear();
    saddleMuscles.clear();
    allPrisms.clear();
    allRods.clear();
    bottomRods.clear();
    topRods.clear();
    prismRods.clear();
    allTouchSensors.clear();
    bottomTouchSensors.clear();
    topTouchSensors.clear();
}
