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

#ifndef TG_SIM_VIEW_H
#define TG_SIM_VIEW_H

/**
 * @file tgSimView.h
 * @brief Contains the definition of class tgSimView
 * @author Brian Mirletz, Ryan Adams
 * $Id$
 */

// Forward declarations
class tgModelVisitor;
class tgSimulation;
class tgWorld;

class tgSimView
{

  /** 
   * Allow tgSimulation to set tgSimView::m_pSimulation to be a back pointer to
   * itself when the tgSimView is passed to the tgSimulation.
   */
  friend class tgSimulation;

public:

    /**
     * The only constructor..
     * @param[in] world a reference to the tgWorld being simulated.
     * @param[in] stepSize the time interval for advancing the simulation;
     * std::invalid_argument is thrown if not positive
     * @param[in] renderRate the time interval for updating the graphics;
     * std::invalid_argument is thrown if less than stepSize
     * @throw std::invalid_argument if stepSize is not positive or renderRate is
     * less than stepSize
     */
    tgSimView(tgWorld& world,
          double stepSize = 1.0/1000.0,
          double renderRate = 1.0/60.0);

    virtual ~tgSimView();

    /**
     * Return a reference to the tgWorld being simulated.
     * @return a reference to the tgWorld being simulated
     */
    tgWorld& world() { return m_world; }

    /**
     * Sets m_initialized to true. Proxy for higher level functions
     * in tgSimViewGraphics
     */
    virtual void setup();
   
    /**
     * Sets m_initialized to false. Proxy for higher level functions
     * in tgSimViewGraphics
     */
    virtual void teardown();
    
    /**
     * Run until stopped by user
     */
    virtual void run();
	
	/**
	 * Run for a specific number of steps
	 */
    virtual void run(int steps);
    
    /**
     * Send the tgModelVisitor to the simulation
     */
    virtual void render() const;
    
    /**
     * Send the tgModelVisitor that was passed in to the simulation
     */
    virtual void render(const tgModelVisitor& r) const;
    
    /**
     * Resets the simulation using simulation->reset()
     * the simulation will call setup and teardown on this as appropreate
     */
    virtual void reset();

    /**
     * Set the interval in seconds at which the graphics are to be rendered.
     * It is set to the minimum of the renderRate argument and the current
     * step size.
     * @param[in] renderRate the interval in seconds at which the graphics are to
     * be rendered
     */
    void setRenderRate(double renderRate);
    
    /**
     * Return the interval in seconds at which the graphics are rendered.
     * @return the interval in seconds at which the graphics are rendered
     */
    double getRenderRate() const { return m_renderRate; }
    
    /**
     * Set the interval in seconds at which the simulation is advanced.
     * The render rate is adjusted to be no less than stepSize.
     * @param[in] stepSize the interval in seconds at which the simulation is
     * advanced; std::invalid_argument is thrown if stepSize is not positive
     * @throw std::invalid_argument if stepSize is not positive
     */
    void setStepSize(double stepSize);
    
    /**
     * Return the interval in seconds at which the graphics are rendered.
     * @return the interval in seconds at which the graphics are rendered
     */
    double getStepSize() const { return m_stepSize; }
    
protected:

    /**
     * Called by a constructor of friend class tgSimulation when an instance of
     * this class is passed as argument to the constructor.
     * Cache a back pointer to the tgSimulation
     * @param[in,out] simulation a reference to the tgSimulation being
     * constructed
     * @throw std::invalid_argument if the tgSimView already has a back pointer.
     */
    void bindToSimulation(tgSimulation& simulation);

    /**
     * Called by the destructor of friend class tgSimulation.
     * Assure that m_pView has a NULL back pointer to its tgSimulation.
     * This allows the tgSimView to be re-used.
     */
    void releaseFromSimulation();

    /**
     * When bound to a tgSimulation, a tgWorld becomes available for the first
     * time.
     * @param[in,out] world a reference to a newly-available tgWorld
     */
    void bindToWorld(tgWorld& world);

    /** @todo Get rid of this. May only be possible once we're no longer using GLUT*/
    bool isInitialzed() const { return m_initialized; }
    
protected:

    /**
     * A back pointer to the simulation that owns this view.
     * Made protected to allow tgSimulatorWithGraphics to inherit it.
     * The constructor initializes it to NULL and it becomes non-NULL
     * when added to a simulation.
     */
    tgSimulation* m_pSimulation;

    /**
     * An object that knows how to visit the various models for
     * rendering or data logging.
     * @note This is a pointer to avoid #including the header file.
     */
    tgModelVisitor * m_pModelVisitor;

    /**
     * The interval in seconds at which the simulation is advanced.
     * It must be positive and it must be less than or equal to
     * m_renderRate. Typically between 1/1000 and 1/4000 will run
     * close to real time. Slower simulations will run faster but
     * be less accurate.
     */
    double m_stepSize;

    /**
     * The interval in seconds at which the graphics are rendered.
     * It must be be greater than or equal to m_stepSize.
     * Note that Bullet is set to render at 60 hz (60 fps), so changing
     * this from 60 may cause the simulation to run faster or slower
     * than real time.
     * @todo See about maintaining real time simulation with a different
     * rendering rate
     */
    double m_renderRate;
    
    /**
     * The time in seconds when the next render should be.
     * It must be non-negative.
     */
    double m_renderTime;
    
private:

    /** Integrity predicate. */
    bool invariant() const;

private:

    /** A reference to the tgWorld being simulated. */
    tgWorld& m_world;

    /** Ensures the world has been initialized before running */
    bool m_initialized;
};

#endif  // TG_SIM_VIEW_H
