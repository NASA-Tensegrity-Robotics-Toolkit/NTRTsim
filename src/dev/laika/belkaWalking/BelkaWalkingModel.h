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

#ifndef BELKA_WALKING_MODEL_H
#define BELKA_WALKING_MODEL_H

/**
 * @file BelkaWalkingModel.h
 * @brief Contains the definition of class BelkaWalkingModel
 * @author Drew Sabelhaus
 * $Id$
 */

// This library
#include "yamlbuilder/TensegrityModel.h"
// The C++ Standard Library
#include <map>
#include <set>
#include <string>
#include <vector>
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h" // for hinge hack

// Forward declarations
//class tgWorld; // will we need this for adding the btHingeConstraint?

// This class will inherit from TensegrityModel, and we don't really
// need to do much else besides add some extra methods to get the rigid bodies
// and cables.
class BelkaWalkingModel: public tgSubject<BelkaWalkingModel>, public TensegrityModel
{
public: 
	
    /**
     * The two constructors. Will just pass in to the parent
     */
    BelkaWalkingModel(const std::string& structurePath);
    BelkaWalkingModel(const std::string& structurePath, bool debugging);
	
    /**
     * Nothing to do. Most functions already handled by tgModel::teardown
     */
    virtual ~BelkaWalkingModel()
    {}
    
    /**
     * Create the model. Call the parent's method,
     * then add a little extra in.
     */
    virtual void setup(tgWorld& world);
	
    /**
     * Step the model, its children. Notifies controllers of step.
     * NOT OVERLOADED HERE since the parent class does all we need (steps the controller for us)
     * @param[in] dt, the timestep. Must be positive.
     */
    //virtual void step(const double dt);

    /**
     * Helper to hand off the leg joints to the controller.
     */
    std::vector<btHingeConstraint*> getLegHinges();

    /**
     * Overriding tgModel's keyboard callback to manually control the robot.
     * Presumably this is through the GLUT interface to Bullet Physics.
     */
    virtual void keyboardCallback(unsigned char key, int x, int y);

    /**
     * For the controller: control input vector.
     */
    std::vector<double> getU(){return u_in;}
    
private:
	
    // the leg hinges
    std::vector<btHingeConstraint*> legHinges;

    /**
     * Frustratingly enough, there isn't a good method for event-based notification of the controller
     * when passing in new information. Instead, we'll need to store the control input vector
     * in the model, and have the controller ask for it. 
     */
    std::vector<double> u_in;

    /**
     * Helper that will update the control input (given to the controller) given a key press.
     */
    void updateU(unsigned char key);

    // Amount to increment the leg angles and spine cable retraction percentages.
    // Might want to eventually change this later programatically so is a variable not a #define.
    double ang_incr = 5.0; // degrees
    double cbl_incr = 0.01; // percent

    // A little helper to roll over the joint angle, should stay between -180 < 0 <180.
    double adjTheta(double theta);
};

#endif
