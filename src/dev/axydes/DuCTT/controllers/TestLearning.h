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

#ifndef ESCAPE_T6CONTROLLER
#define ESCAPE_T6CONTROLLER

/**
 * @file DuCTTRobotController.h
 * @brief Contains the definition of class DuCTTRobotController.
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

#include <vector>

#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"

// Forward declarations
class DuCTTRobotModel;
class tgLinearString;
class tgPrismatic;

//namespace std for vectors
using namespace std;

/** Escape Controller for T6 */
class DuCTTRobotController : public tgObserver<DuCTTRobotModel>
{
    public:
        DuCTTRobotController(const double prefLength=5.0,
                             const bool useManualParams=false,
                             const string manParamFile="");

        /** Nothing to delete, destructor must be virtual */
        virtual ~DuCTTRobotController() { }

        virtual void onSetup(DuCTTRobotModel& subject);

        virtual void onStep(DuCTTRobotModel& subject, double dt);

        virtual void onTeardown(DuCTTRobotModel& subject);

    protected:
        virtual vector< vector <double> > transformActions(vector< vector <double> > act);

        virtual void applyActions(DuCTTRobotModel& subject, vector< vector <double> > act);

    private:
        /** Initialize the evolution adapter as well as its own parameters */
        void setupAdapter();

        /** Returns amount of energy spent by each muscle in subject */
        double totalEnergySpent(DuCTTRobotModel& subject);

        /** Sets target lengths for each muscle */
        void setPreferredMuscleLengths(DuCTTRobotModel& subject, double dt);

        /** Divides the 24 muscles of an DuCTTRobotModel
         * into 8 clusters of 3 muscles */
        void populateClusters(DuCTTRobotModel& subject);

        /** Sets the amplitude, angularFrequency, phase change, and dcOffset
         * for each sine wave used in muscle actuation */
        void initializeSineWaves();

        /** Difference in position between initPosition and finalPosition
         * of subject */
        double displacement(DuCTTRobotModel& subject);

        /** Select action paramters from a comma-separated line in a file */
        std::vector<double> readManualParams(int lineNumber, string filename);

        void printSineParams();

        btVector3 initPosition; // Initial position of model
        const double m_initialLengths;
        double m_totalTime;
        double const maxStringLengthFactor; // Proportion of string's initial length by which a given actuator can increase/decrease
        const bool m_usingManualParams;
        const string m_manualParamFile;

        // Evolution and Adapter
        AnnealAdapter evolutionAdapter;
        vector< vector<double> > actions; // For modifications between episodes

        // Muscle Clusters
        int nClusters;
        int musclesPerCluster;
        /** A vector clusters, each of which contains a vector of muscles */
        vector<vector<tgLinearString*> > clusters; 

        // Sine Wave Data
        double* amplitude;
        double* angularFrequency;
        double* phaseChange;
        double* dcOffset;
};

#endif // ESCAPE_T6CONTROLLER
