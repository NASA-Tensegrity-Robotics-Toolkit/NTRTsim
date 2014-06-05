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

#include "tgcreator/tgNode.h"
#include "tgcreator/tgNodes.h"
#include "tgcreator/tgPair.h"
#include "tgcreator/tgPairs.h"

void testNode(const btVector3& fixedPoint, const btVector3& axis, double angle)
{
    tgNode n(0,1,0);

    std::cout << "[tgNode]:" << std::endl;
    std::cout << "- before rotation: " << n << std::endl;

    n.addRotation(fixedPoint, axis, angle);
    
    std::cout << "- after rotation: " << n << std::endl;
}

void testNodes(const btVector3& fixedPoint, const btVector3& axis, double angle)
{
    std::cout << "[tgNodes]:" << std::endl;
    tgNodes nodes;
    nodes.addNode(0,0,0);
    nodes.addNode(1,0,0);
    nodes.addNode(0,1,0);
    nodes.addNode(0,0,1);
    nodes.addNode(1,1,1);
    
    std::cout << "- before rotation: " << nodes << std::endl;

    nodes.addRotation(fixedPoint, axis, angle);
    
    std::cout << "- after rotation: " << nodes << std::endl;
}

void testPair(const btVector3& fixedPoint, const btVector3& axis, double angle)
{
    tgPair p(tgNode(0,1,0), tgNode(1,0,1));

    std::cout << "[tgPair]:" << std::endl;
    std::cout << "- before rotation: " << p << std::endl;

    p.addRotation(fixedPoint, axis, angle);
    
    std::cout << "- after rotation: " << p << std::endl;
}

void testPairs(const btVector3& fixedPoint, const btVector3& axis, double angle)
{
    std::cout << "[tgPairs]:" << std::endl;
    tgNodes nodes;
    nodes.addNode(0,0,0);
    nodes.addNode(1,0,0);
    nodes.addNode(0,1,0);
    nodes.addNode(0,0,1);
    nodes.addNode(1,1,1);
    nodes.addNode(2,2,2);
    
    tgPairs pairs;
    pairs.addPair(nodes.pair(0, 1));
    pairs.addPair(nodes.pair(2, 3));
    pairs.addPair(nodes.pair(4, 5));

    std::cout << "- before rotation: " << pairs << std::endl;

    pairs.addRotation(fixedPoint, axis, angle);
    
    std::cout << "- after rotation: " << pairs << std::endl;
}



int main(int argc, char** argv)
{

    btVector3 fixedPoint(0,0,0);
    btVector3 axis(1,0,0);
    double angle = M_PI/2.0;

    std::cout << "Fixed point:  " << fixedPoint << std::endl;
    std::cout << "Axis:         " << axis << std::endl;
    std::cout << "Angle:        " << angle << " radians" << std::endl;
    
    testNode(fixedPoint, axis, angle);
    testNodes(fixedPoint, axis, angle);
    
    testPair(fixedPoint, axis, angle);
    testPairs(fixedPoint, axis, angle);
    
}
