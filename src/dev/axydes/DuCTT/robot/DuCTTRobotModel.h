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

#ifndef DUCTT_ROBOT_MODEL_H
#define DUCTT_ROBOT_MODEL_H

/**
 * @file DuCTTRobotModel.h
 * @brief Defines a 3 strut 9 string tensegrity model
 * @author Brian Tietz
 * @version 1.0.0
 * $Id$
 */

// The NTRT Core Libary
#include "core/tgModel.h"
#include "core/tgSubject.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"

// The C++ Standard Library
#include <vector>

// Forward declarations
class tgModelVisitor;
class tgStructure;
class tgWorld;
class tgBasicActuator;
class tgPrismatic;
class tgRod;
class tgSphere;
class tgTouchSensorModel;
class btCollisionObject;

/**
 * A class that constructs a three bar tensegrity DuCTT using the tools
 * in tgcreator.
 */
class DuCTTRobotModel : public tgSubject<DuCTTRobotModel>, public tgModel
{
public: 
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     //
     // see tgBaseString.h for a descripton of some of these rod parameters
     // (specifically, those related to the motor moving the strings.)
     //
     // NOTE that any parameter that depends on units of length will scale
     // with the current gravity scaling. E.g., with gravity as 981,
     // the length units below are in centimeters.
     */
    struct Config
    {
        Config(
                btVector3 startPos = btVector3(0,10,0),//so it doesn't start in the floor
                btVector3 startRotAxis = btVector3(0,0,1),
                double startRotAngle = 0,
                double triangle_length = 32,
                double duct_distance = 15,
                double duct_height = 23,
                double prismRadius = 1.524,
                double prismExtent = 10.16,
                double prismDensity = 0.001943,
                double vertRodRadius = 1.27,
                double vertDensity = 0.000895,
                double innerRodRadius = 2.0955,
                double innerDensity = 0.001359,
                double tipRad = 1.524,
                double tipDens = 0.00001943,
                double tipFric = 10.0,
                double stiffness = 5000.0,
                double damping = 25.0,
                double pretension = 2000.0,
                double maxVertStringVel = 25.4,
                double maxSaddleStringVel = 8.5,
                double maxStringForce = 5000.0,
                double maxStringAcc = 100000.0,
                double minStringRestLength = 1.2,
                bool storeStringHist = false,
                bool useCylinderEndCaps = false,
                bool debug = false
              );

        //robot params
        btVector3 m_startPos;
        btVector3 m_startRotAxis;
        double m_startRotAngle;
        //tetra prams
        double m_triangle_length;     // triangle_length (length) 30 cm
        double m_duct_distance;     // duct_distance (distance between tetrahedrons) 15 cm
        double m_duct_height;    // duct_height (length)
        //rod params
        double m_prismRadius; // prismatic joint radius 1.524 cm
        double m_prismExtent; // prismatic joint max extension 10.16 cm
        double m_prismDensity;     // density (mass / (pi * radius^2 * length)) kg/cm^3 0.453592 / (pi * 1.524^2 * 32) = 0.001943
        double m_vertRodRadius; // vertical rod radius 1.27 cm
        double m_vertDensity;     // density (mass / (pi * radius^2 * length)) kg/cm^3 0.127 / (pi * 1.27^2 * 28) = 0.000895
        double m_innerRodRadius; // inner rod radius 2.0955 cm
        double m_innerDensity;     // density (mass / (pi * radius^2 * length)) kg/cm^3 0.6 / (pi * 2.0955^2 * 32) = 0.001359
        //sphere tip params
        double m_tipRad; // prismatic joint tip radius 1.524 cm
        double m_tipDens; // prismatic joint tip density (mas / length^3) kg/cm^3
        double m_tipFric; // prismatic joint tip friction
        //string params
        double m_stiffness;   // stiffness (mass / sec^2) vectran string
        double m_damping;     // damping (mass / sec)
        double m_pretension;     // Pretension Force (Newtons?)
        double m_maxVertStringVel; // max velocity of vertical string motors (cm/s) 25.4cm/s
        double m_maxSaddleStringVel; // max velocity of saddle string motors (cm/s) 8.5cm/s
        double m_maxStringForce; // max force to exert on all strings (Newtons) 50 N = 5000 kg*cm/s^2
        double m_maxStringAcc; // NOT USED: max acceleration of string motors (length/s^2) 10000.0
        double m_minStringRestLength; // rest length below which motor ceases to function 1.2cm
        bool m_storeStringHist; // store string history, useful for learning applications
        bool m_useCylinderEndCaps; // true=use cylinders for end caps of linear actuators, false=use half-spheres
        bool m_debug; //print debugging info
    };

    DuCTTRobotModel();
    DuCTTRobotModel(DuCTTRobotModel::Config &config);
    
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    virtual ~DuCTTRobotModel();
    
    /**
     * Create the model. Place the rods and strings into the world
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
    virtual void teardown();
    
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
    const std::vector<tgBasicActuator*>& getAllMuscles() const;

    /**
     * Return a vector of all the vertical muscles for the controllers to work with.
     * @return A vector of all of the vertical muscles
     */
    const std::vector<tgBasicActuator*>& getVertMuscles() const;

    /**
     * Return a vector of all saddle muscles for the controllers to work with.
     * @return A vector of all of the saddle muscles
     */
    const std::vector<tgBasicActuator*>& getSaddleMuscles() const;

    /**
     * Return a vector of all prismatic joints for the controllers to work with.
     * @return A vector of all of the prismatic joints
     */
    const std::vector<tgPrismatic*>& getAllPrismatics() const;

    /**
     * Return a vector of all touch sensors for the controllers to work with.
     * @return A vector of all of the touch sensors
     */
    const std::vector<tgTouchSensorModel*>& getAllTouchSensors() const;

    /**
     * @return the bottom tetra's prismatic actuator
     */
    tgPrismatic* getBottomPrismatic();

    /**
     * @return the top tetra's prismatic actuator
     */
    tgPrismatic* getTopPrismatic();

    /**
     * Calculate the Center of Mass (COM) for the entire model.
     * @return btVector3 COM of the model
     */
    btVector3 getCOM();

    /**
     * Calculate the Center of Mass (COM) for one of the tetras.
     * @param bottom true if you want the COM for the bottom tetra, false if you want it for the top
     * @return btVector3 COM of the specified tetra
     */
    btVector3 getTetraCOM(bool bottom = true);

    /**
     * Calculate the mass for the entire model.
     * @return double mass of the model
     */
    double mass();

    /**
     * Calculate the mass for one of the tetras.
     * @param bottom true if you want the COM for the bottom tetra, false if you want it for the top
     * @return double mass of the specified tetra
     */
    double getTetraMass(bool bottom = true);

    bool addIgnoredObject(const btCollisionObject* obj);
    bool addIgnoredObject(const btCollisionObject* obj, std::vector<tgTouchSensorModel*> touchSensors);

    std::vector<tgTouchSensorModel*> allTouchSensors;
    std::vector<tgTouchSensorModel*> bottomTouchSensors;
    std::vector<tgTouchSensorModel*> topTouchSensors;

private:
    /**
     * Sets up the tgStructure containing the actual model of the robot.
     */
    virtual void setupStructure(tgWorld &world);

    /**
     * Sets up the tgStructure containing the Ghost model of the robot (just touch sensors).
     */
    virtual void setupGhostStructure(tgWorld &world);

    /**
     * Sets up the internal variables used to make accessing parts of the robot easier.
     */
    virtual void setupVariables();

    /**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] tetra: A tgStructure that we're building into
     * @param[in] edge: the X distance of the base points
     * @param[in] width: the Z distance of the base triangle
     * @param[in] height: the Y distance along the axis of the DuCTT
     */
    void addNodes(tgStructure& tetra,
                            double edge,
                            double width,
                            double height,
                            bool isGhost = false,
                            double distBtHinges = 0.1,
                            double distBtNodes = 0.1
                  );

    /**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     * @param[in] startNode Number of node to start with
     */
    void addPairs(tgStructure& s, int startNode = 0);
    
    /**
     * A function called during setup that creates muscles (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    void addMuscles(tgStructure& s, int topNodesStart);
    
    /**
     * A list of all of the muscles. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgBasicActuator*> allMuscles;

    /**
     * A list of all of the vertical muscles. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgBasicActuator*> vertMuscles;

    /**
     * A list of all of the saddle muscles. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgBasicActuator*> saddleMuscles;

    /**
     * A list of all of the prismatic joints. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgPrismatic*> allPrisms;

    std::vector<tgRod*> allRods;
    std::vector<tgRod*> bottomRods;
    std::vector<tgRod*> topRods;
    std::vector<tgRod*> prismRods;

    DuCTTRobotModel::Config m_config;

    tgPrismatic *m_pBottomPrismatic;
    tgPrismatic *m_pTopPrismatic;
    std::vector<const btCollisionObject*> m_IgnoredObjs;
};

#endif  // DuCTT_MODEL_H
