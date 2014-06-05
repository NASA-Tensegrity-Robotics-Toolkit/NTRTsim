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
 * @file configuration.cpp
 * @brief Contains the implementation of class configuration.
 * Allows for configuration of learning in a text file
 * @date April 2013
 * @author Atil Iscen
 * $Id$
 */

#include <iostream>
#include <sstream>
#include <fstream>
#include "configuration.h"

using namespace std;

configuration::configuration(){}
configuration::~configuration(){}

int configuration::getintvalue( const std::string& key )
{
	if (!iskey( key )){
		std::cout<<"Cannot find the key in the config file, Key: "<<key<<endl;
		throw 0;
	}
	std::istringstream ss( this->data.operator [] ( key ) );
	int result;
	ss >> result;
	if (!ss.eof())
	{
		std::cout<<"Problematic key: "<<key<<endl;
		std::cout<<"Error reading configuration file"<<endl;
		throw 1;
	}
	return result;
}


double configuration::getDoubleValue(const std::string& key )
{
	if (!iskey( key )) throw 0;
	std::istringstream ss( this->data.operator [] ( key ) );
	double result;
	ss >> result;
	if (!ss.eof()) throw 1;
	return result;
}

std::string configuration::getStringValue(const std::string& key )
{
	if (!iskey( key )) throw 0;
	std::istringstream ss( this->data.operator [] ( key ) );
	string result;
	ss >> result;
	if (!ss.eof()) throw 1;
	return result;
}

void configuration::readFile(const std::string filename)
{
	std::string s, key, value;
	std::ifstream confFile(&filename[0]);
	if(!confFile.is_open())
	{
		std::cout<<"Warning! Config.ini file not found!"<<std::endl;
		return;
	}

	// For each (key, value) pair in the file
	while (std::getline( confFile, s ))
	{
		std::string::size_type begin = s.find_first_not_of( " \f\t\v" );

		// Skip blank lines
		if (begin == std::string::npos) continue;

		// Skip commentary
		if (std::string( "#;" ).find( s[ begin ] ) != std::string::npos) continue;

		// Extract the key value
		std::string::size_type end = s.find( '=', begin );
		key = s.substr( begin, end - begin );

		// (No leading or trailing whitespace allowed)
		key.erase( key.find_last_not_of( " \f\t\v" ) + 1 );

		// No blank keys allowed
		if (key.empty()) continue;

		// Extract the value (no leading or trailing whitespace allowed)
		begin = s.find_first_not_of( " \f\n\r\t\v", end + 1 );
		end   = s.find_last_not_of(  " \f\n\r\t\v" ) + 1;

		value = s.substr( begin, end - begin );

		// Insert the properly extracted (key, value) pair into the map
		this->data[ key ] = value;
	}
	confFile.close();
	return;
}


void configuration::writeToFile(const std::string filename)
{
	ofstream confFile(&filename[0]);
	for (std::map <std::string, std::string>::const_iterator iter = this->data.begin(); iter != this->data.end(); iter++)
		confFile << iter->first << " = " << iter->second << endl;
	confFile.close();
	return;
}
