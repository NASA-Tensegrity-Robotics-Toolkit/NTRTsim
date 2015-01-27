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

#ifndef ESCAPECONTROLLER
#define ESCAPECONTROLLER

/**
 * @file EscapeController.h
 * @brief Contains the definition of class EscapeController.
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

#include <vector>

#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"
#include "learning/Configuration/configuration.h"
#include "learning/AnnealEvolution/AnnealEvolution.h"

// Forward declarations
class EscapeModel;
class tgBasicActuator;

/** Escape Controller for T6 */
class EscapeController : public tgObserver<EscapeModel>
{
    public:
        // Note that currently this is calibrated for decimeters.
        EscapeController(const double prefLength=5.0,
                            std::string args = "_Escape",
                            std::string resourcePath = "",
                            std::string config = "Config.ini");

        /** Nothing to delete, destructor must be virtual */
        virtual ~EscapeController() { }

        virtual void onSetup(EscapeModel& subject);

        virtual void onStep(EscapeModel& subject, double dt);

        virtual void onTeardown(EscapeModel& subject);

    protected:
        virtual std::vector< std::vector <double> > transformActions(std::vector< std::vector <double> > act);

        virtual void applyActions(EscapeModel& subject, std::vector< std::vector <double> > act);

    private:
        std::vector<double> initPosition; // Initial position of model
        const double m_initialLengths;
        double m_totalTime;
        double const maxStringLengthFactor; // Proportion of string's initial length by which a given actuator can increase/decrease

        // Evolution and Adapter
        AnnealAdapter evolutionAdapter;
        std::vector< std::vector<double> > actions; // For modifications between episodes

        // Muscle Clusters
        int nClusters;
        int musclesPerCluster;
        /** A vector clusters, each of which contains a vector of muscles */
        std::vector<std::vector<tgBasicActuator*> > clusters; 

        // Sine Wave Data
        double* amplitude;
        double* angularFrequency;
        double* phaseChange;
        double* dcOffset;
        
        // Configuration strings
        std::string suffix;
        std::string configPath;
        std::string configName;

        /** Initialize the evolution adapter as well as its own parameters */
        void setupAdapter();

        /** Returns amount of energy spent by each muscle in subject */
        double totalEnergySpent(EscapeModel& subject);

        /** Sets target lengths for each muscle */
        void setPreferredMuscleLengths(EscapeModel& subject, double dt);

        /** Divides the 24 muscles of an EscapeModel 
         * into 8 clusters of 3 muscles */
        void populateClusters(EscapeModel& subject);

        /** Sets the amplitude, angularFrequency, phase change, and dcOffset 
         * for each sine wave used in muscle actuation */
        void initializeSineWaves();

        /** Difference in position between initPosition and finalPosition
         * of subject */
        double displacement(EscapeModel& subject);

        /** Select action paramters from a comma-separated line in a file */
        std::vector<double> readManualParams(int lineNumber, std::string filename);

        void printSineParams();
};

#endif // ESCAPECONTROLLER
