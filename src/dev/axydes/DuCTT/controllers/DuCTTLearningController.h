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

#ifndef DUCTT_LEARNING_CONTROLLER
#define DUCTT_LEARNING_CONTROLLER

/**
 * @file DuCTTLearningController.h
 * @brief Contains the definition of class DuCTTLearningController.
 * @author Alexander Xydes
 * @version 1.0.0
 * $Id$
 */

#include <vector>

#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"
#include "learning/Adapters/NeuroAdapter.h"

// The Bullet Physics library
#include "LinearMath/btVector3.h"

// Forward declarations
class AnnealEvolution;
class NeuroEvolution;
class DuCTTRobotModel;
class tgBasicActuator;
class tgPrismatic;
class tgImpedanceController;
class tgTouchSensorModel;

//namespace std for vectors
using namespace std;

class DuCTTLearningController : public tgObserver<DuCTTRobotModel>
{
    public:
        DuCTTLearningController(const double prefLength=5.0,
                                const bool useManualParams=false,
                                const string manParamFile="",
                                int axis=1,
                                bool neuro=false,
                                string resourcePath="",
                                string suffix="_DuCTT",
                                string evoConfigFilename="Config.ini"
                                );

        /** Nothing to delete, destructor must be virtual */
        virtual ~DuCTTLearningController() { }

        virtual void onSetup(DuCTTRobotModel& subject);

        virtual void onStep(DuCTTRobotModel& subject, double dt);

        virtual void onTeardown(DuCTTRobotModel& subject);

    protected:
        virtual vector< vector <double> > transformActions(vector< vector <double> > act);

        virtual void applyActions(DuCTTRobotModel& subject, vector< vector <double> > act);

    private:
        /** Returns amount of energy spent by each muscle in subject */
        double totalEnergySpent(DuCTTRobotModel& subject);

        void moveMotors(DuCTTRobotModel &subject, double dt);
        /** Sets target lengths for each muscle */
        void setPreferredMuscleLengths(DuCTTRobotModel& subject, double dt);
        /** Sets target lengths for each prismatic joint */
        void setPrismaticLengths(DuCTTRobotModel& subject, double dt);

        bool shouldPause(std::vector<tgTouchSensorModel*> touchSensors);
        bool isLocked(DuCTTRobotModel& subject, bool isTop);

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
        const double m_initialLength;
        double m_totalTime;
        double const maxStringLengthFactor; // Proportion of string's initial length by which a given actuator can increase/decrease
        const bool m_usingManualParams;
        const string m_manualParamFile;
        int m_axis; //axis of movement. x=0, y=1, z=2, all=3

        // Evolution Params
        bool m_isLearning;
        string m_evoConfigFilename;
        configuration m_evoConfig;

        // Anneal adapter
        AnnealEvolution m_evolution;
        AnnealAdapter m_evolutionAdapter;

        // Neuro adapter
        NeuroEvolution m_NeuroEvolution;
        NeuroAdapter m_neuroAdapter;
        bool m_bUseNeuro;

        vector< vector<double> > m_actions; // For modifications between episodes

        // Muscle Clusters
        int nClusters;
        int musclesPerCluster;
        /** A vector clusters, each of which contains a vector of muscles */
        vector<vector<tgBasicActuator*> > clusters;
        int nPrisms;
        vector<tgPrismatic*> prisms;
        int nActions;

        bool m_bIgnoreTouchSensors;
        bool m_bRecordedStart;

        // Sine Wave Data
        double* amplitude;
        double* angularFrequency;
        double* phaseChange;
        double* dcOffset;

        tgImpedanceController* imp_controller;
        bool m_bBadRun;

        bool m_bBottomPaused;
        bool m_bTopPaused;
        int bottomCounter;
        int topCounter;
        double m_dHistorisisSeconds;
};

#endif // ESCAPE_T6CONTROLLER
