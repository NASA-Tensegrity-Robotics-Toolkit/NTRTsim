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

#ifndef DUCTT_LEARNING
#define DUCTT_LEARNING

/**
 * @file DuCTTLearning.h
 * @brief Contains the definition of class DuCTTLearning.
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

class DuCTTLearning : public tgObserver<DuCTTRobotModel>
{
    public:
        DuCTTLearning(const double prefLength=5.0,
                        int axis=1,
                        bool neuro=false,
                        string resourcePath="",
                        string suffix="DuCTT",
                        string evoConfigFilename="Config.ini",
                        bool useManualParams=false,
                        string manualParamFile=""
                        );

        /** Nothing to delete, destructor must be virtual */
        virtual ~DuCTTLearning() { }

        virtual void onSetup(DuCTTRobotModel& subject);

        virtual void onStep(DuCTTRobotModel& subject, double dt);

        virtual void onTeardown(DuCTTRobotModel& subject);

    protected:
        //Returns the class name, used for printouts
        virtual string getName() = 0;

        //Any initializing needing to be done before learning adapter is initialized
        virtual void initBeforeAdapter(DuCTTRobotModel &subject) = 0;

        //Any initializing needing to be done after learning adapter is initialized
        virtual void initAfterAdapter(DuCTTRobotModel &subject) = 0;

        //Transform learning adapter parameters into local parameters
        virtual vector< vector <double> > transformActions(vector< vector <double> > act) = 0;

        //Apply local parameters to their controllers
        virtual void applyActions(DuCTTRobotModel& subject, vector< vector <double> > act) = 0;

        //Anything needing to be done before the recorded start
        virtual void stepBeforeStart(DuCTTRobotModel& subject, double dt) = 0;

        //Normal step function, moveMotors will be called right after this
        virtual void stepBeforeMove(DuCTTRobotModel& subject, double dt) = 0;

        //Anything needing to be done after moveMotors is called
        virtual void stepAfterMove(DuCTTRobotModel& subject, double dt) = 0;

        //local teardown function
        virtual void teardownEnd(DuCTTRobotModel &subject) = 0;

        //Moves the subjects motors
        void moveMotors(DuCTTRobotModel &subject, double dt);

        //Return the first score to be recorded for this run
        //Defaults to displacement
        //Override to use your own
        double getFirstScore(DuCTTRobotModel &subject);

        //Return the second score to be recorded for this run
        //Defaults to totalEnergySpent
        //Override to use your own
        double getSecondScore(DuCTTRobotModel &subject);

        /** Returns amount of energy spent by each muscle in subject */
        double totalEnergySpent(DuCTTRobotModel& subject);

        /** Difference in position between initPosition and finalPosition
         * of subject */
        double displacement(DuCTTRobotModel& subject);

        std::vector<double> readManualParams(int lineNumber);

        btVector3 initPosition; // Initial position of model
        const double m_initialLength;
        double m_totalTime;
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

        bool m_bRecordedStart;

        tgImpedanceController* imp_controller;

        bool m_bBadRun;
        bool m_bUseManualParams;
        string m_ManualParamFile;
        string m_ResourcePath;
};

#endif // ESCAPE_T6CONTROLLER
