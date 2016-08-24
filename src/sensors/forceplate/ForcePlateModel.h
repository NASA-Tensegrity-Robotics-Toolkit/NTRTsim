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
// The Bullet Physics library
#include "LinearMath/btVector3.h"

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
class ForcePlateModel : public tgSubject<ForcePlateModel>, public tgModel
{
public: 

  /**
   * The configuration struct for this force plate.
   * See the Solidworks model files for a graphical representation of
   * the geometric parameters here.
   */
  struct Config
  {
  public:
    /**
     * This is the definition of the config struct. It's 'constructor' is in 
     * the cpp file.
     * The 'constructor' for this struct assigns variables if they're passed in,
     * otherwise, defaults are chosen.
     * Unlike with some of the other models, such as tgSpringCable Actuator,
     * where abbreviations are used for the dummy variables and longer names
     * are used for the actual variables in the struct, this is reversed:
     * since there will be a LOT of calculations using these names, typing out
     * a whole word will be very cumbersome as I'm writing this class.
     * Just use this class in the same way as done with the demo, and it should
     * work fine. 
     *
     * This model also takes a vector as input, below, specifying the location of
     * the center of the bottom of the housing. This is preferable to taking
     * two points like a tgBox, since that would involve validating that the 
     * two points were along a horizontal line (this class will not support 
     * force plates at angles, at least for now.)
     *
     * So, the config struct is for parameters related to the actual construction,
     * not the location, of the force plate.
     */
    Config( double length = 5.0,
	    double width = 5.0,
	    double height = 2.0,
	    double thickness = 0.1,
	    double platethickness = 1.0,
	    double wallGap = 0.2,
	    double bottomGap = 0.5,
	    double lateralStiffness = 500.0,
	    double verticalStiffness = 1000.0,
	    double lateralDamping = 50.0,
	    double verticalDamping = 100.0,
	    double lateralRestLength = 0.2,
	    double verticalRestLength = 0.5,
	    double springAnchorOffset = 0.1);

    /**
     * Length of the whole force plate assembly.
     * Use a capital letter here since lower-case "l" is difficult to read.
     * Units: length.
     */
    double L;

    /**
     * Width of the whole force plate assembly
     * Units: length.
     */
    double w;
    
    /**
     * Height of the whole force plate assembly
     * Units: length.
     */
    double h;
    
    /**
     * Thickness of the walls of the force plate assembly. This is
     * the distance between the outer edge of housing to the inner edge
     * of the housing, along the lateral directions.
     * Must be greater than zero, and less than (TO-DO)
     * Units: length.
     */
    double t;
    
    /**
     * Thickness of the force plate itself. The force plate will be
     * this "tall", inside the housing.  Must be greater than zero,
     * and less than (TO-DO)
     * Units: length.
     */
    double pt;

    /**
     * Thickness of the gap between the housing and the sides of the force plate 
     * (in the lateral directions.) This is the displacement of those lateral springs
     * by default. If equal to lat_RL, springs will apply zero force when
     * the plate is unloaded.
     * Units: length.
     */
    double wgap;
    
    /**
     * Thickness of the gap between the housing and the bottom of the force plate.
     * If equal to vert_RL, springs will only have a force equal to the gravitational
     * force of the weight of the plate itself, when unloaded.
     * Units: length.
     */
    double bgap;
    
    /**
     * Stiffness of the springs between the housing and the sides of the force plate 
     * (in the lateral directions.) This applies to all 8 of the lateral springs.
     * Units: spring stiffness, Force / length.
     */
    double latK;
    
    /**
     * Stiffness of the springs between the housing and the bottom of the force
     * plate. This applies to all 4 of the vertical springs.
     * Units: spring stiffness, Force / length.
     */
    double vertK;

    /**
     * Damping constant of the springs between the housing and the sides of the 
     * force plate (in the lateral directions.) This applies to all 8 of the 
     * lateral springs.
     * Units: spring damping, Force / velocity.
     */
    double latD;

    /**
     * Damping constant of the springs between the housing and the bottom of the 
     * force plate. This applies to all 4 of the vertical springs.
     * Units: spring damping, Force / velocity.
     */
    double vertD;

    /**
     * Rest length of the springs between the housing and the sides of the 
     * force plate (in the lateral directions.) This applies to all 8 of the 
     * lateral springs.
     * Units: length.
     */
    double latRL;

    /**
     * Rest length of the springs between the housing and the bottom of the force
     * plate. This applies to all 4 of the vertical springs.
     * Units: length.
     */
    double vertRL;

    /**
     * Spring anchor offset amount. This is the distance between the edges
     * of the force plate and the spring, for both lateral and horizontal.
     * For example, a lateral spring is attached this far away from a plate edge.
     */
    double sOff;
    
  };

    /**
     * Helper function for the constructor. Validates all the passed-in parameters.
     */
    void constructorAux();
  
    /**
     * The first constructor. Takes a config struct and a location to put
     * the force plate at.
     * @param[in] config, a struct as defined above
     * @param[in] location, the location of the center point of 
     * the bottom of the force plate housing. A vector, assumed to start
     * from (0, 0, 0).
     */
    ForcePlateModel(const ForcePlateModel::Config& config, btVector3& location);

    /**
     * Another constructor. Takes a config struct and a location to put
     * the force plate at, as well as tgTags to mass in to tgModel.
     * @param[in] tags as passed through tgStructure and tgStructureInfo
     */
    //ForcePlateModel(ForcePlateModel::Config config, btVector3 location,
    //		    const tgTags& tags);

    /**
     * Another constructor that allows for a debugging on/off flag to be passed in.
     * @param[in] debugging: a boolean that controls the output of 
     * various debugging information to the terminal.
     */
    ForcePlateModel(const ForcePlateModel::Config& config, btVector3& location,
		    bool debugging);
	
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~ForcePlateModel();
    
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
    //const std::vector<tgCompressionSpringActuator*>& getAllActuators() const;
    //const std::vector<tgBasicActuator*>& getAllActuators() const;

protected:

    /**
     * The config struct for this specific force plate.
     */
    Config m_config;

    /**
     * The node positions for this specific force plate.
     * This is populated in setup by the calculateNodePositions function.
     */
    btVector3 a1;
    btVector3 a2;
    btVector3 b1;
    btVector3 b2;
    btVector3 c1;
    btVector3 c2;
    btVector3 d1;
    btVector3 d2;

    /**
     * The spring anchor positions on the force plate, for the lateral springs.
     * These are denoted as s_(node1)(node2), where for example s_ab is the 
     * spring anchor location on the ab face that's closest to point a, while
     * s_ba is the anchor location on the ab face closest to point b.
     */
    btVector3 s_ab;
    btVector3 s_ba;
    btVector3 s_bc;
    btVector3 s_cb;
    btVector3 s_cd;
    btVector3 s_dc;
    btVector3 s_da;
    btVector3 s_ad;

    /**
     * The spring anchor positions on the bottom side of the force plate,
     * for the vertical springs. There are only four bottom springs,
     * so the variable name denotes which corner of the box is closest to that 
     * spring anchor location.
     */
    btVector3 s_bot_a;
    btVector3 s_bot_b;
    btVector3 s_bot_c;
    btVector3 s_bot_d;
    
    /**
     * The btVector3 location of this specific force plate.
     */
    btVector3 m_location;

    /**
     * Flag for debugging. If on, information about the force plate construction
     * is output to the terminal.
     */
    bool m_debugging;
    
private:
	
    /**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] s: A tgStructure that we're building into
     */
    static void addNodes(tgStructure& s);
	
    /**
     * A function called during setup that creates boxes for the force plate,
     * the ones that will be used for the lateral springs.
     * @param[in] s: A tgStructure that we're building into
     */
    static void addLateralPlateBoxes(tgStructure& s);
	
    /**
     * A function called during setup that creates springs from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s: A tgStructure that we're building into
     */
    static void addSprings(tgStructure& s);    
    
    /**
     * Calculates all the node positions for this force plate.
     * Uses m_config.
     */
    void calculatePlateNodePositions();
	
    /**
     * A list of all of the muscles. Will be empty until most of the way
     * through setup
     */
    //std::vector<tgCompressionSpringActuator*> allActuators;
    //std::vector<tgBasicActuator*> allActuators;

    // The "invariant" function just checks the integrity of the member
    // variables of this class.
    bool invariant() const;
};

#endif  // FORCE_PLATE_MODEL_H
