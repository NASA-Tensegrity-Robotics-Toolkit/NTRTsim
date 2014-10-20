/*
 * Copyright © 2014, United States Government, as represented by the
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

/**
 * @file tgPort.cpp
 * @brief Provides portability of tensegrities via JSON
 * $Id$
 */

// Sample application
#include "dev/steve/T12SuperBallPayload.h"
#include "dev/steve/T12SuperBallPayloadController.h"
// This library
#include "core/tgModel.h"
// The C++ Standard Library
#include <iostream>
#include <string>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */

// Exports JSON of tg to file
void exportJSON(string filename, tgModel* tg);

// Returns tgModel stored in provided file
// Assumes file is in JSON format
tgModel *importJSON(string filename);

int main(int argc, char** argv)
{
    T12SuperBallPayload* myModel = new T12SuperBallPayload(); //Note: model can no longer be const

    SuperBallPrefLengthController* pTC = new SuperBallPrefLengthController(9); //Note: controller can no longer be const
    myModel->attach(pTC);

    // DEMO
    string filename = "tgData.json";
    myModel = (T12SuperBallPayload *) importJSON(filename); //TODO: copy constructor?
    exportJSON(filename, myModel);

    return 0;
}

void exportJSON(string filename, tgModel* tg) {
    cout << "\"Exporting JSON of tg to " + filename + "\"" << endl;
}

tgModel *importJSON(string filename) {
    cout << "\"Importing JSON from " + filename + "\"" << endl;

    return NULL;
}

