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

#ifndef NESTED_STRUCTURE_SERIALIZED_SINE_WAVES_H
#define NESTED_STRUCTURE_SERIALIZED_SINE_WAVES_H

/**
 * @file SerializedSineWaves.h
 * @brief A Sine Wave controller for TetraSpine using JSON serialization
 * @author Brian Mirletz
 * @version 1.0.0
 * $Id$
 */

// NTRTSim
#include "core/tgObserver.h"

// The C++ Standard Library
#include <vector>
#include <string>

// Forward Declarations
class tgImpedanceController;
class tgSpringCableActuator;
class BaseSpineModelLearning;

/**
 * Control the TetraSpineLearningModel with a series of sine waves
 * and local impedance controllers
 */
class SerializedSineWaves : public tgObserver<BaseSpineModelLearning>
{
public:

struct Config
{
	Config(std::string fileName);
	
	~Config();
	
	/**
	 * Pointers to impedance controllers 
	 */
    tgImpedanceController* in_controller;
    tgImpedanceController* out_controller;
	
	 /**
     * Muscle Length Parameters
     */
    
    double insideLength;
    double outsideLength;
    
    /**
     * CPG related parameters.
     */
    double offsetSpeed;
    double cpgAmplitude;
    double cpgFrequency;
    double bodyWaves;
    
    /**
     * How the inside strings re-interpret the signal
     */
    double insideMod;
    
    /**
     * Phase offsets for the CPGs.
     * Units are radians
     */
    std::vector<double> phaseOffsets;
    
    /**
     * How often to send a sine wave signal
     */
     double updateFrequency;
	
};
	
public:
	
	/**
	 * Construct the controller. Typically occurs in the main function.
	 * The controller will need to be attached to a subject (model)
	 * Parameters are currently set in the initalizer lists.
	 */
    SerializedSineWaves(std::string fileName);
    
    /**
     * Destructor. Frees the tgImpedanceController pointers
     */
    ~SerializedSineWaves();
    
    /**
     * Applies the impedance controllers using a velocity setpoint of 0.
     * Called during this classes onStep function.
     * @param[in] stringList a std::vector of strings taken from the
     * subject's MuscleMap
     * @param[in] dt - a timestep. Must be positive.
     * @param[in] phase - reads the index out of the phaseOffsets vector
     */
    void applyImpedanceControlInside(const std::vector<tgSpringCableActuator*> stringList,
                                                            double dt,
                                                            std::size_t phase);
    /**
     * Applies the impedance controllers using a velocity setpoint determined.
     * by the phase parameter and 
     * Called during this classes onStep function.
     * @param[in] stringList a std::vector of strings taken from the
     * subject's MuscleMap
     * @param[in] dt - a timestep. Must be positive.
     * @param[in] phase - reads the index out of the phaseOffsets vector
     */                                    
    void applyImpedanceControlOutside(const std::vector<tgSpringCableActuator*> stringList,
                                    double dt,
                                    std::size_t phase);
    
    /**
     * Apply the sineWave controller. Called my notifyStep(dt) of its
     * subject. Calls the applyImpedanceControl functions of this class
     * @param[in] subject - the TetraSpineLearningModel that is being 
     * Subject must have a MuscleMap populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(BaseSpineModelLearning& subject, double dt);
    
private:
    
    const Config m_config;
    
    std::size_t segments;
    
    /**
     * Parameters that consolidate the sine wave computations within
     * the update code. Cycle deals with sin(theta), and target handles
     * offset + amplitude * cycle
     */
    double simTime;
    double updateTime;
    double cycle;
    double target;
};

#endif // MY_MODEL_CONTROLLER_H
