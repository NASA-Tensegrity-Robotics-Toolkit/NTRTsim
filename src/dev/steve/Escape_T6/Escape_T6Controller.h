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
 * @file Escape_T6Controller.h
 * @brief Contains the definition of class Escape_T6Controller.
 * @author Steven Lessard
 * @version 1.0.0
 * $Id$
 */

// This library
#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"
#include <vector>

// Forward declarations
class Escape_T6Model;
class tgLinearString;

//namespace std for vectors
using namespace std;

/**
 * Escape Controller for T6.
 */
class Escape_T6Controller : public tgObserver<Escape_T6Model>
{
    public:

        /**
         * Construct a T6PrefLengthController with the initial preferred length.
         */

        // Note that currently this is calibrated for decimeters.
        Escape_T6Controller(const double prefLength=5.0);

        /**
         * Nothing to delete, destructor must be virtual
         */
        virtual ~Escape_T6Controller() { }

        virtual void onSetup(Escape_T6Model& subject);

        virtual void onStep(Escape_T6Model& subject, double dt);

        virtual void onTeardown(Escape_T6Model& subject);

    protected:

        virtual vector< vector <double> > transformActions(vector< vector <double> > act);

        virtual void applyActions (Escape_T6Model& subject, vector< vector <double> > act);

    private:
        double m_initialLengths;
        double m_totalTime;
        AnnealAdapter evolutionAdapter;

        /** Initialize the evolution adapter as well as its own parameters */
        void setupAdapter();

        /** Returns amount of energy spent by each muscle in subject */
        double totalEnergySpent(Escape_T6Model& subject);

        /** Sets target lengths for each muscle */
        void setPreferredMuscleLengths(vector<tgLinearString*> muscles, double dt);
};

#endif // ESCAPE_T6CONTROLLER
