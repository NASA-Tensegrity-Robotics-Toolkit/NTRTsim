/**
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
 * @file AppElevationMap.cpp
 * @brief Contains a script for extracting the elevation of the terrain
 * @author Edward Zhu
 * $Id$
 */

// C++ Libraries
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>
#include <vector>

using namespace std;

int main(int argc, char* argv[]) {
	// Check for number of arguments
	if (argc != 3) {
		cout << "Incorrect number of arguments, please use ./AppElevationMap [path/filename_in] [path/filename_out]" << endl;
		exit(EXIT_FAILURE);
	}
	else {
		// Get filenames from argv
		string filename_in = argv[1];
		string filename_out = argv[2];

		// Check for valid file extension
		if (filename_in.find(".txt") == string::npos) {
			cout << "Incorrect filetype, application for .txt files only" << endl;
			exit(EXIT_FAILURE);
		}
		else {
			cout << "File to parse: " << filename_in << endl;
		}

		// Create filestream objects
		fstream file_in;
		fstream file_out;

		// Open input file
		file_in.open(filename_in.c_str(), fstream::in);
		
		// Check if input file opened successfully
		if (!file_in.is_open()) {
			cout << "Failed to open input file, please check filename" << endl;
			exit(EXIT_FAILURE);
		}
		else {
			cout << "Input file opened successfully" << endl;
		}

		//out_path = out_path + filename_out;
		cout << "Elevation data writing to: " << filename_out << endl;

		// Open output file
		file_out.open(filename_out.c_str(), fstream::out);
		
		// Check if output file opened successfully
		if (!file_out.is_open()) {
			cout << "Failed to open output file" << endl;
			exit(EXIT_FAILURE);
		}
		else {
			cout << "Output file opened successfully" << endl;
			file_out << "XAvg, YAvg, ZAvg" << endl;
		}

		while (file_in.good()) {
			
			vector<double> v0, v1, v2;

			string line_in;

			getline(file_in, line_in);
			
			size_t found_left_brac_last = -1, found_right_brac_last = -1, found_comma_last = -1;
        	
        	bool validTriangle = true;

	        // Iterate over 3 sets of vertecies
	        for (int i = 0; i < 3; i++) {
	            size_t found_left_brac = line_in.find_first_of("[", found_left_brac_last + 1);
	            size_t found_right_brac = line_in.find_first_of("]", found_right_brac_last + 1);
	            size_t found_comma_1 = line_in.find_first_of(",", found_comma_last + 1);
	            size_t found_comma_2 = line_in.find_first_of(",", found_comma_1 + 1);

	            // Extract x value
	            string x_str = line_in.substr(found_left_brac + 1, found_comma_1 - 1 - found_left_brac);
	            double x = atof(x_str.c_str());

	            // Extract y value
	            string y_str = line_in.substr(found_comma_1 + 1, found_comma_2 - 1 - found_comma_1);
	            double y = atof(y_str.c_str());

	            // Extrack z value
	            string z_str = line_in.substr(found_comma_2 + 1, found_right_brac - 1 - found_comma_2);
	            double z = atof(z_str.c_str());

	            // Swap y and z values to match NTRT coordinate convention (y is height)
	            double temp = y;
	            y = z;
	            z = -temp;

	            // Update last found positions
	            found_left_brac_last = found_left_brac;
	            found_right_brac_last = found_right_brac;
	            found_comma_last = found_comma_2;

				switch (i) {
                case 0:
                    v0.push_back(x);
                    v0.push_back(y);
                    v0.push_back(z);
                    break;
                case 1:
                    v1.push_back(x);
                    v1.push_back(y);
                    v1.push_back(z);
                    break;
                case 2:
                    v2.push_back(x);
                    v2.push_back(y);
                    v2.push_back(z);
                    break;
            	}
			}

			if (v0[1] <= 0 || v1[1] <= 0 || v2[1] <= 0) {
				validTriangle = false;
			}

			if (validTriangle) {
				double x_avg = (v0[0] + v1[0] + v2[0]) / 3;
				double y_avg = (v0[1] + v1[1] + v2[1]) / 3;
				double z_avg = (v0[2] + v1[2] + v2[2]) / 3;
				cout << x_avg << ", " << y_avg << ", " << z_avg << endl;
				file_out << x_avg << ", " << y_avg << ", " << z_avg << endl;
			}
			
		}

		if (file_in.eof()) {
			cout << "Reached end of file, closing files" << endl;
		}
		else if (file_in.fail()) {
			cout << "Read failed" << endl;
		}
		file_in.close();
		file_out.close();	
	}
	return 0;
}