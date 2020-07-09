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

#ifndef SERIALIZED_SPINE_CONTROL
#define SERIALIZED_SPINE_CONTROL

/**
 * @file SerializedSpineControl.h
 * @brief A Sine Wave controller for TetraSpine using JSON serialization
 * @author Brian Mirletz
 * @version 1.0.0
 * $Id$
 */

// NTRTSim
#include "core/tgObserver.h"
#include "sensors/tgDataObserver.h"

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
class SerializedSpineControl : public tgObserver<BaseSpineModelLearning>
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
    tgImpedanceController* top_controller;
	
	/**
	 * Definitions to find muscle length parameters
	 */
	double rod_edge;
	double rod_front;
	double rod_offset;
	
	
	 /**
     * Muscle Length Parameters
     */
    
    std::vector<double> insideTopLength;
    std::vector<double> insideLeftLength;
    std::vector<double> insideRightLength;
    std::vector<double> outsideTopLength;
    std::vector<double> outsideLeftLength;
    std::vector<double> outsideRightLength;
    
	 /**
     * Muscle Tension Parameters
     */
    
    std::vector<double> insideTopTens;
    std::vector<double> insideLeftTens;
    std::vector<double> insideRightTens;
    std::vector<double> outsideTopTens;
    std::vector<double> outsideLeftTens;
    std::vector<double> outsideRightTens;
    
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
    SerializedSpineControl(std::string fileName);
    
    /**
     * Destructor. Frees the tgImpedanceController pointers
     */
    ~SerializedSpineControl();
    

    
    virtual void onSetup(BaseSpineModelLearning& subject);
    
    /**
     * Apply the sineWave controller. Called my notifyStep(dt) of its
     * subject. Calls the applyImpedanceControl functions of this class
     * @param[in] subject - the TetraSpineLearningModel that is being 
     * Subject must have a MuscleMap populated
     * @param[in] dt, current timestep must be positive
     */
    virtual void onStep(BaseSpineModelLearning& subject, double dt);
    
private:
   
    /**
     * Because sometimes you give up and specify everything
     */
    void applyImpedanceControlGeneric(tgImpedanceController* controller,	
										const std::vector<tgSpringCableActuator*> stringList,
										const std::vector<double> stringLengths,
										const std::vector<double> tensions,
										double dt,
										std::size_t phase);
    
    Config m_config;
    
    std::size_t segments;
    
   tgDataObserver m_dataObserver;
    
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
