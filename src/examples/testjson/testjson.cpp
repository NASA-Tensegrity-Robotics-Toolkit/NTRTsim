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

/**
 * @file testjson.cpp
 * @brief Contains a brief example illustrating JsonCPP's usage.
 * $Id$
 */

#include <iostream>
#include <json/json.h>
#include "helpers/FileHelpers.h"

using namespace std;

int main() {
    cout<<"Hello"<<endl;
    string fileStr = FileHelpers::getFileString("data.txt");

    Json::Value root; // will contains the root value after parsing.
    Json::Reader reader;
    bool parsingSuccessful = reader.parse( fileStr, root );
    if ( !parsingSuccessful )
    {
        // report to the user the failure and their locations in the document.
        std::cout << "Failed to parse configuration\n"
            << reader.getFormattedErrorMessages();
        return 1;
    }
    string myName = root.get("name", "UTF-8").asString();
    cout <<"My name is "<<myName<<endl;
}
