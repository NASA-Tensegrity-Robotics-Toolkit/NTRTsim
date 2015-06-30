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
 * @file PrismModel.cpp
 * @brief Contains the definition of the members of the class AATModel.
 * $Id$
 */

// This module
#include "AATModel.h"
// This library
#include "tgBallJoint.h"
#include "core/tgKinematicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "tgBallJointInfo.h"
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgKinematicActuatorInfo.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgBasicContactCableInfo.h" 
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include <vector>
#include <cmath>

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
        double radius_ml;
        double spacing_ml;
        double stiffness;
        double damping;
        double pretension;
        double triangle_length;
        double triangle_height;
        double height_ml;
        double rod_length;
        double cap_rod_spacing;
        double hex_ratio;
        double motor_radius;
        double motor_friction;
        double motor_inertia;
        bool   backDrivable;
        bool   hist;
        double maxTens;
        double targetVelocity;
    } c =
   {
       0.2,      // density (mass / length^3)
       0.31,     // radius (length)
       1.5,     // radius of the fixed (massless) cap (length)
       1.3,      // spacing of the fixed (massless) cap (length)  
       1000.0,   // stiffness (mass / sec^2)
       500.0,     // damping (mass / sec)
       10000.0,    // pretension (mass * length / sec^2)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       40.0,     // height of the fixed (massless) cap (length)
       10.0,     // rod_length (length)
       0.5,      // cap_rod_spacing (length)
       15.0,      // hex_ratio    
       0.09,    // motor_radius // Spindle radius (length)
       4.24e-5, // motor_friction (kg*(length)^2/sec)
       2.749e-4, // motor_inertia (kg*(length)^2) // Inertia of motor, gearbox, and spindle all together
       0,       // Not backDrivable
       0,           // History logging (boolean)
       100000,   // maxTens
       4,    // targetVelocity  
  };
} // namespace

AATModel::AATModel() :
tgModel()
{
}

AATModel::~AATModel()
{
}

void AATModel::addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{


    // 6 Rod Structure

    // const double half_radius_ml = c.radius_ml / 2;

    // Rods

    // Top nodes

    // s.addNode(c.radius_ml, c.height_ml - c.cap_rod_spacing, 0);
    // s.addNode(half_radius_ml, c.height_ml - c.cap_rod_spacing, c.spacing_ml);
    // s.addNode(-half_radius_ml, c.height_ml - c.cap_rod_spacing, c.spacing_ml);
    // s.addNode(-c.radius_ml, c.height_ml - c.cap_rod_spacing, 0);
    // s.addNode(-half_radius_ml, c.height_ml - c.cap_rod_spacing, -c.spacing_ml);
    // s.addNode(half_radius_ml, c.height_ml - c.cap_rod_spacing, -c.spacing_ml);

    // // Bottom nodes

    // s.addNode(c.hex_ratio * c.radius_ml, c.height_ml - c.cap_rod_spacing - height_str, 0);
    // s.addNode(c.hex_ratio * half_radius_ml, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * c.spacing_ml);
    // s.addNode(c.hex_ratio * -half_radius_ml, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * c.spacing_ml);
    // s.addNode(c.hex_ratio * -c.radius_ml, c.height_ml - c.cap_rod_spacing - height_str, 0);
    // s.addNode(c.hex_ratio * -half_radius_ml, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * -c.spacing_ml);
    // s.addNode(c.hex_ratio * half_radius_ml, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * -c.spacing_ml);


    // // Cap

    // s.addNode(c.radius_ml, c.height_ml, 0);
    // s.addNode(half_radius_ml, c.height_ml, c.spacing_ml);
    // s.addNode(-half_radius_ml, c.height_ml, c.spacing_ml);
    // s.addNode(-c.radius_ml, c.height_ml, 0);
    // s.addNode(-half_radius_ml, c.height_ml, -c.spacing_ml);
    // s.addNode(half_radius_ml, c.height_ml, -c.spacing_ml);

    // 8 Rod Structure
    
    const double spacing = sqrt(2) * c.radius_ml / 2;
    const double height_str = (c.hex_ratio - 1) * c.radius_ml * tan(0.349066);
    //const double cable_ratio = (c.hex_ratio + 3) / 4;
    // const double gap = sqrt(2) * (c.radius_ml + height_str) / 2;

    // Top nodes

    s.addNode(c.radius_ml, c.height_ml - c.cap_rod_spacing, 0);
    s.addNode(spacing, c.height_ml - c.cap_rod_spacing, spacing);
    s.addNode(0, c.height_ml - c.cap_rod_spacing, c.radius_ml);
    s.addNode(-spacing, c.height_ml - c.cap_rod_spacing, spacing);
    s.addNode(-c.radius_ml, c.height_ml - c.cap_rod_spacing, 0);
    s.addNode(-spacing, c.height_ml - c.cap_rod_spacing, -spacing);
    s.addNode(0, c.height_ml - c.cap_rod_spacing, -c.radius_ml);
    s.addNode(spacing, c.height_ml - c.cap_rod_spacing, -spacing);

    // Bottom nodes

    s.addNode(c.hex_ratio * c.radius_ml, c.height_ml - c.cap_rod_spacing - height_str, 0);
    s.addNode(c.hex_ratio * spacing, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * spacing);
    s.addNode(0, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * c.radius_ml);
    s.addNode(c.hex_ratio * -spacing, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * spacing);
    s.addNode(c.hex_ratio * -c.radius_ml, c.height_ml - c.cap_rod_spacing - height_str, 0);
    s.addNode(c.hex_ratio * -spacing, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * -spacing);
    s.addNode(0, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * -c.radius_ml);
    s.addNode(c.hex_ratio * spacing, c.height_ml - c.cap_rod_spacing - height_str, c.hex_ratio * -spacing);

    // s.addNode(c.radius_ml + height_str, c.height_ml - c.cap_rod_spacing, 0);
    // s.addNode(gap, c.height_ml - c.cap_rod_spacing, gap);
    // s.addNode(0, c.height_ml - c.cap_rod_spacing, c.radius_ml + height_str);
    // s.addNode(-gap, c.height_ml - c.cap_rod_spacing, gap);
    // s.addNode(-c.radius_ml - height_str, c.height_ml - c.cap_rod_spacing, 0);
    // s.addNode(-gap, c.height_ml - c.cap_rod_spacing, -gap);
    // s.addNode(0, c.height_ml - c.cap_rod_spacing, -c.radius_ml - height_str);
    // s.addNode(gap, c.height_ml - c.cap_rod_spacing, -gap);

    // Cap

    s.addNode(c.radius_ml, c.height_ml, 0);
    s.addNode(spacing, c.height_ml, spacing);
    s.addNode(0, c.height_ml, c.radius_ml);
    s.addNode(-spacing, c.height_ml, spacing);
    s.addNode(-c.radius_ml, c.height_ml, 0);
    s.addNode(-spacing, c.height_ml, -spacing);
    s.addNode(0, c.height_ml, -c.radius_ml);
    s.addNode(spacing, c.height_ml, -spacing);

    const double radius_mlo = 3 * c.radius_ml;
    const double spacing_mlo = 3 * spacing; 

    s.addNode(radius_mlo, c.height_ml, 0);
    s.addNode(spacing_mlo, c.height_ml, spacing_mlo);
    s.addNode(0, c.height_ml, radius_mlo);
    s.addNode(-spacing_mlo, c.height_ml, spacing_mlo);
    s.addNode(-radius_mlo, c.height_ml, 0);
    s.addNode(-spacing_mlo, c.height_ml, -spacing_mlo);
    s.addNode(0, c.height_ml, -radius_mlo);
    s.addNode(spacing_mlo, c.height_ml, -spacing_mlo);

    //Cable Nodes

    // s.addNode(cable_ratio * c.radius_ml, c.height_ml - c.cap_rod_spacing - height_str / 4, 0);
    // s.addNode(cable_ratio * spacing, c.height_ml - c.cap_rod_spacing - height_str / 4, cable_ratio * spacing);
    // s.addNode(0, c.height_ml - c.cap_rod_spacing - height_str / 4, cable_ratio * c.radius_ml);
    // s.addNode(cable_ratio * -spacing, c.height_ml - c.cap_rod_spacing - height_str / 4, cable_ratio * spacing);
    // s.addNode(cable_ratio * -c.radius_ml, c.height_ml - c.cap_rod_spacing - height_str / 4, 0);
    // s.addNode(cable_ratio * -spacing, c.height_ml - c.cap_rod_spacing - height_str / 4, cable_ratio * -spacing);
    // s.addNode(0, c.height_ml - c.cap_rod_spacing - height_str / 4, cable_ratio * -c.radius_ml);
    // s.addNode(cable_ratio * spacing, c.height_ml - c.cap_rod_spacing - height_str / 4, cable_ratio * -spacing);


}

void AATModel::addRods(tgStructure& s)
{
    
    // // 6 Rod Struecture

    // // Rods

    // s.addPair( 0,  7, "rod");
    // s.addPair( 1,  8, "rod");
    // s.addPair( 2,  9, "rod");
    // s.addPair( 3,  10, "rod");
    // s.addPair( 4,  11, "rod");
    // s.addPair( 5,  6, "rod");

    // // Cap

    // s.addPair( 12,  13, "cap");
    // s.addPair( 13,  14, "cap");
    // s.addPair( 14,  15, "cap");
    // s.addPair( 15,  16, "cap");
    // s.addPair( 16,  17, "cap");
    // s.addPair( 17,  12, "cap");

    // 8 Rod Structure

    // Rods

    // Normal Structure

    s.addPair( 0, 8, "rod");
    s.addPair( 1, 9, "rod");
    s.addPair( 2, 10, "rod");
    s.addPair( 3, 11, "rod");
    s.addPair( 4, 12, "rod");
    s.addPair( 5, 13, "rod");
    s.addPair( 6, 14, "rod");
    s.addPair( 7, 15, "rod");

    // For the structure with cables attached to the middle of rods

    // s.addPair( 0, 24, "rod");
    // s.addPair( 1, 25, "rod");
    // s.addPair( 2, 26, "rod");
    // s.addPair( 3, 27, "rod");
    // s.addPair( 4, 28, "rod");
    // s.addPair( 5, 29, "rod");
    // s.addPair( 6, 30, "rod");
    // s.addPair( 7, 31, "rod");

    // s.addPair( 24, 8, "rod");
    // s.addPair( 25, 9, "rod");
    // s.addPair( 26, 10, "rod");
    // s.addPair( 27, 11, "rod");
    // s.addPair( 28, 12, "rod");
    // s.addPair( 29, 13, "rod");
    // s.addPair( 30, 14, "rod");
    // s.addPair( 31, 15, "rod");

    // Cap

    s.addPair( 16, 17, "cap");
    s.addPair( 17, 18, "cap");
    s.addPair( 18, 19, "cap");
    s.addPair( 19, 20, "cap");
    s.addPair( 20, 21, "cap");
    s.addPair( 21, 22, "cap");
    s.addPair( 22, 23, "cap");
    s.addPair( 23, 16, "cap");

    s.addPair( 24, 25, "cap");
    s.addPair( 25, 26, "cap");
    s.addPair( 26, 27, "cap");
    s.addPair( 27, 28, "cap");
    s.addPair( 28, 29, "cap");
    s.addPair( 29, 30, "cap");
    s.addPair( 30, 31, "cap");
    s.addPair( 31, 24, "cap");

}

void AATModel::addActuators(tgStructure& s)
{
    // // 6 Rod Structure

    // s.addPair( 0,  6, "actuated");
    // s.addPair( 1,  7, "actuated");
    // s.addPair( 2,  8, "actuated");
    // s.addPair( 3,  9, "actuated");
    // s.addPair( 4,  10, "actuated");
    // s.addPair( 5,  11, "actuated");

    // s.addPair( 0,  1, "passive");
    // s.addPair( 1,  2, "passive");
    // s.addPair( 2,  3, "passive");
    // s.addPair( 3,  4, "passive");
    // s.addPair( 4,  5, "passive");
    // s.addPair( 5,  0, "passive");

    // s.addPair( 6,  7, "passive");
    // s.addPair( 7,  8, "passive");
    // s.addPair( 8,  9, "passive");
    // s.addPair( 9,  10, "passive");
    // s.addPair( 10, 11, "passive");
    // s.addPair( 11, 6, "passive");

    // 8 Rod Structure

    // Normal Structure

    s.addPair( 8,  24, "actuated");
    s.addPair( 9,  25, "actuated");
    s.addPair( 10,  26, "actuated");
    s.addPair( 11,  27, "actuated");
    s.addPair( 12,  28, "actuated");
    s.addPair( 13,  29, "actuated");
    s.addPair( 14,  30, "actuated");
    s.addPair( 15,  31, "actuated");

    // For the structure with cables attached to the middle of rods

    // s.addPair( 8,  25, "actuated");
    // s.addPair( 9,  26, "actuated");
    // s.addPair( 10,  27, "actuated");
    // s.addPair( 11,  28, "actuated");
    // s.addPair( 12,  29,  "actuated");
    // s.addPair( 13,  30, "actuated");
    // s.addPair( 14,  31, "actuated");
    // s.addPair( 15,  24, "actuated");

    // s.addPair( 0,  1, "passive");
    // s.addPair( 1,  2, "passive");
    // s.addPair( 2,  3, "passive");
    // s.addPair( 3,  4, "passive");
    // s.addPair( 4,  5, "passive");
    // s.addPair( 5,  6, "passive");
    // s.addPair( 6,  7, "passive");
    // s.addPair( 7,  0, "passive");

    s.addPair( 8,  9, "actuated");
    s.addPair( 9,  10, "actuated");
    s.addPair( 10,  11, "actuated");
    s.addPair( 11,  12, "actuated");
    s.addPair( 12,  13, "actuated");
    s.addPair( 13,  14, "actuated");
    s.addPair( 14,  15, "actuated");
    s.addPair( 15,  8, "actuated");

}

void AATModel::addBallJoints(tgStructure& s)
{
    // // 6 Rod Structure

    // s.addPair( 0,  12, "balljoint");
    // s.addPair( 1,  13, "balljoint");
    // s.addPair( 2,  14, "balljoint");
    // s.addPair( 3,  15, "balljoint");
    // s.addPair( 4,  16, "balljoint");
    // s.addPair( 5,  17, "balljoint");

    // 8 Rod Structure

    s.addPair( 0, 16, "balljoint");
    s.addPair( 1, 17, "balljoint"); 
    s.addPair( 2, 18, "balljoint"); 
    s.addPair( 3, 19, "balljoint"); 
    s.addPair( 4, 20, "balljoint");     
    s.addPair( 5, 21, "balljoint"); 
    s.addPair( 6, 22, "balljoint"); 
    s.addPair( 7, 23, "balljoint"); 

}

void AATModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    //const tgRod::Config CapConfig(c.radius, 0);
    const tgRod::Config RodConfig(c.radius, c.density);

    const tgRod::Config CapConfig(c.radius, 0);
    
    const tgBasicActuator::Config passiveCableConfig(c.stiffness, c.damping, 50, c.hist,
                        c.maxTens, c.targetVelocity, 0.1, 0.001);
    // // This part is added by Ali to make a more accurate model of SuperBall's Rods
    // tgKinematicActuator::Config motorConfig(998., c.damping, c.pretension, c.motor_radius, c.motor_friction,
    //                     c.motor_inertia, c.backDrivable, c.hist, c.maxTens, c.targetVelocity);
    const tgBasicActuator::Config actuatedCableConfig(c.stiffness, c.damping, c.pretension, c.hist,
                        c.maxTens, c.targetVelocity, 0.1, 0.001);
    const tgBallJoint::Config balljointConfig(0.01, 20, 0.01, 0.9, 0.3, 1.0);

    // Create a structure that will hold the details of this model
    tgStructure s;

    // Add nodes to the structure
    addNodes(s, c.radius_ml, c.spacing_ml, c.height_ml);

    // Add rods to the structure
    addRods(s);

    // Add muscles to the structure
    addActuators(s);

    // Add Ball Joints

    addBallJoints(s);

    // Move the structure so it doesn't start in the ground
    //s.move(btVector3(0, 2, 0));

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    //spec.addBuilder("cap", new tgRodInfo(CapConfig));
    spec.addBuilder("rod", new tgRodInfo(RodConfig));
    spec.addBuilder("cap", new tgRodInfo(CapConfig));
    spec.addBuilder("actuated", new tgBasicActuatorInfo(actuatedCableConfig));
    //spec.addBuilder("passive", new tgBasicActuatorInfo(passiveCableConfig));
    spec.addBuilder("balljoint", new tgBallJointInfo(balljointConfig));
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control.
    allActuators = tgCast::filter<tgModel, tgBasicActuator> (getDescendants());

    // Notify controllers that setup has finished.
    notifySetup();

    // Actually setup the children
    tgModel::setup(world);
}

void AATModel::step(double dt)
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

void AATModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgBasicActuator*>& AATModel::getAllActuators() const
{
    return allActuators;
}



void AATModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
