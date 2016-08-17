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

#ifndef FORCE_PLATE_MODEL_H
#define FORCE_PLATE_MODEL_H

/**
 * @file ForcePlateModel.h
 * @brief Contains the definition of class ForcePlateModel.
 * @author Drew Sabelhaus
 * @copyright Copyright (C) 2016 NASA Ames Research Center
 * $Id$
 */

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"
// The C++ Standard Library
#include <vector>

// Forward declarations
class tgUnidirectionalCompressionSpringActuator;
class tgModelVisitor;
class tgStructure;
class tgWorld;

/**
 * This tgModel creates a 'force plate' that can sense forces in x, y, and z.
 * It creates a frame structure with a separate plate in the middle, and a
 * series of springs supporting it around the sides and bottom.
 * These springs (should) have a high stiffness, so that when a force is
 * applied to the plate, the plate moves a very small distance, and then
 * the force can be back-calculated through the deflection of the springs.
 * To use this model, be sure to choose appropriate dimensions and spring
 * constants. The spring constant and coefficient
 * of damping will need to be chosen based on the types of forces that should
 * be sensed, so run tests with different parameters to see what works best.
 * See the PDF document, and Solidworks model, that accompany this file
 * (called NTRT Force Plate) for a graphical view of all the dimensions.
 * In order to take data using this force plate, a tgDataObserver/Logger must
 * be attached to this model.
 */
class TwoBoxesModel : public tgSubject<TwoBoxesModel>, public tgModel
{
public: 
	
    /**
     * The only constructor. Utilizes default constructor of tgModel
     * Configuration parameters are within the .cpp file in this case,
     * not passed in. 
     */
    TwoBoxesModel();
	
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~TwoBoxesModel();
    
    /**
     * Create the model. Place the boxess and springs into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);
    
    /**
     * Undoes setup. Deletes child models. Called automatically on
     * reset and end of simulation. Notifies controllers of teardown
     */
    void teardown();
    
    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(double dt);
	
    /**
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself 
     */
    virtual void onVisit(tgModelVisitor& r);
    
    /**
     * Return a vector of all muscles for the controllers to work with.
     * @return A vector of all of the muscles
     */
    const std::vector<tgCompressionSpringActuator*>& getAllActuators() const;
    //const std::vector<tgBasicActuator*>& getAllActuators() const;
    
private:
	
	/**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] s: A tgStructure that we're building into
     */
    static void addNodes(tgStructure& s);
	
	/**
     * A function called during setup that creates boxes from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s: A tgStructure that we're building into
     */
    static void addBoxes(tgStructure& s);
	
	/**
     * A function called during setup that creates muscles (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s: A tgStructure that we're building into
     */
    static void addActuators(tgStructure& s);

private:
	
	/**
     * A list of all of the muscles. Will be empty until most of the way
     * through setup
     */
    std::vector<tgCompressionSpringActuator*> allActuators;
    //std::vector<tgBasicActuator*> allActuators;
};

#endif  // TWO_BOXES_MODEL_H
