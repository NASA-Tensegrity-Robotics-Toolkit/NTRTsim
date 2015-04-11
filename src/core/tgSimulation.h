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

#ifndef TG_SIMULATION_H
#define TG_SIMULATION_H

/**
 * @file tgSimulation.h
 * @brief Contains the definition of class tgSimulation
 * @author Ryan Adams, Brian Mirletz
 * $Id$
 */

// The C++ Standard Library
#include <iostream>
#include <vector>

// Forward declarations
class tgModel;
class tgModelVisitor;
class tgSimView;
class tgWorld;
class tgGround;

/**
 * Holds objects necessary for simulation, a world, a view
 * and a list of models.
 */
class tgSimulation
{
public:

    /**
     * The only constructor.
     * @param[in,out] view the way the world and its models are rendered.
     */
    tgSimulation(tgSimView& view);

    ~tgSimulation();

    /**
     * Advance the simulation.
     * @param[in] dt the number of seconds since the previous call;
     * throw an exception if not positive
     * @throw std::invalid_argument if dt is not positive
     */
    void step(double dt) const; 

    /**
     * Run until stopped by user. Calls tgSimView.run()
     */   
    void run() const;

    /**
     * Run for a specific number of steps. Calls tgSimView.run(int steps)
     * @param[in] steps the number of steps to update the graphics
     * @todo Make steps of type size_t.
     */
    void run(int steps) const;

    /**
     * Add a Tensegrity to the simulation.
     * @param[in] pModel a pointer to a tgModel representing a Tensegrity;
     * an exception is thrown if it is NULL
     * @throw std::invalid_argument if pModel is NULL
     */

    void addModel(tgModel* pModel);
    /**
     * Add an obstacle to the simulation.
     * Obstacles are deleted upon reset.
     * @param[in] pObstacle a pointer to a tgModel representing an obstacle;
     * an exception is thrown if it is NULL
     * @throw std::invalid_argument if pModel is NULL
     */
    void addObstacle(tgModel* pObstacle);
    
    /**
     * Pass the tgModelVisitor to all of the models
     */
    void onVisit(const tgModelVisitor& r) const;
    
    /**
     * Calls teardown, then calls setup on the view, finally
     * calls setup on the models
     * Will delete and remake the dynamics world
     */
    void reset();

    /**
     * Calls teardown, then sets the ground of the world to newGround
     * then calls setup on the view, finally
     * calls setup on the models
     * Will delete and remake the dynamics world, the previous
     * ground will be deleted
     */
    void reset(tgGround* newGround);
    
    /**
     * Returns a reference to the world
     */
    tgWorld& getWorld() const;

 private:
    
    /**
     * Calls teardown on all of the models and reset on the world
     */
    void teardown();

    /** Integrity predicate. */
    bool invariant() const;

private:

    /** The way the world and its models are rendered. */
    tgSimView& m_view;

    /**
     * The Tensegrities.
     * All pointers are non-NULL.
     * @todo Should this be std::set?
     */
    std::vector<tgModel*> m_models;
    
    /**
     * Obstacles are models that are deleted after one simulation
     * This allows their presence or absence to be controlled by
     * main.
     * All pointers should be non-NULL
     */
    std::vector<tgModel*> m_obstacles;
};

#endif  // TG_SIMULATION_H

