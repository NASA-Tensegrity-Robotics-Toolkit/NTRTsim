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

#ifndef DUCTT_LEARN_STATE_MACHINE
#define DUCTT_LEARN_STATE_MACHINE

/**
 * @file DuCTTLearnStateMachine.h
 * @brief Contains the definition of class DuCTTLearnStateMachine.
 * @author Alexander Xydes
 * @version 1.0.0
 * $Id$
 */

#include "DuCTTLearning.h"

class DuCTTLearnStateMachine : public DuCTTLearning
{
    public:
        enum ROBOT_STATE{
            EXPAND_BOTTOM,
            RETRACT_TOP,
            PUSH_TOP,
            EXPAND_TOP,
            RETRACT_BOTTOM,
            PULL_BOTTOM
        };

        DuCTTLearnStateMachine(const double prefLength=5.0,
                        int axis=1,
                        bool neuro=false,
                        string resourcePath="",
                        string suffix="DuCTT",
                        string evoConfigFilename="Config.ini",
                        bool useManualParams=false,
                        string manualParamFile=""
                        );

        /** Nothing to delete, destructor must be virtual */
        virtual ~DuCTTLearnStateMachine() { }

    protected:
        virtual string getName() {return "DuCTTLearnStateMachine";}

        virtual void initBeforeAdapter(DuCTTRobotModel &subject);

        /** Sets the amplitude, angularFrequency, phase change, and dcOffset
         * for each sine wave used in muscle actuation */
        virtual void initAfterAdapter(DuCTTRobotModel &subject);

        virtual vector< vector <double> > transformActions(vector< vector <double> > act);

        virtual void applyActions(DuCTTRobotModel& subject, vector< vector <double> > act);

        virtual void stepBeforeStart(DuCTTRobotModel &subject, double dt);

        virtual void stepBeforeMove(DuCTTRobotModel &subject, double dt);

        virtual void stepAfterMove(DuCTTRobotModel &subject, double dt) {}

        virtual void teardownEnd(DuCTTRobotModel &subject);

        bool movePrism(tgPrismatic* prism, std::vector<tgTouchSensorModel*> sensors, double goal, double dt);
        bool moveStrings(const std::vector<tgBasicActuator*> stringList, double goals, double dt);

        bool shouldPause(std::vector<tgTouchSensorModel*> touchSensors);

        bool isLocked(DuCTTRobotModel& subject, bool isTop);

        void printSineParams();

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

        bool m_bBottomPaused;
        bool m_bTopPaused;
        int bottomCounter;
        int topCounter;
        double m_dHistorisisSeconds;
        bool m_bTilting;

        ROBOT_STATE state;
        double maxStringLength;
        double minStringLength;
        double stringLengthEPS;
};

#endif // ESCAPE_T6CONTROLLER
