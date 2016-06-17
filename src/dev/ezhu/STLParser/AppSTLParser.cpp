#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

using namespace std;

int main(int argc, char* argv[]) {
	// Check for number of arguments
	if (argc <= 2) {
		cout << "Not enough arguments, please use ./AppSTLParser [path/filename_in] [path/filename_out]" << endl;
		exit(EXIT_FAILURE);
	}
	else if (argc > 3) {
		cout << "Too many arguments, please use ./AppSTLParser [path/filename_in] [path/filename_out]" << endl;
		exit(EXIT_FAILURE);
	}
	else {
		// Get filenames from argv
		string filename_in = argv[1];
		string filename_out = argv[2];
		//string  out_path = "/home/edward/NTRTsim/src/dev/ezhu/STLParser/";
		// Check for valid file extension
		if (strstr(filename_in.c_str(), ".stl") == NULL) {
			cout << "Incorrect filetype, application for ASCII STL files only" << endl;
			exit(EXIT_FAILURE);
		}
		else {
			cout << "File to parse: " << filename_in << endl;
		}
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
		cout << "Parsed info writing to: " << filename_out << endl;
		// Open output file
		file_out.open(filename_out.c_str(), fstream::out);
		// Check if output file opened successfully
		if (!file_out.is_open()) {
			cout << "Failed to open output file" << endl;
			exit(EXIT_FAILURE);
		}
		else {
			cout << "Output file opened successfully" << endl;
		}
		// file_out << "Test 1 2 3" << endl;
		string key = "vertex";
		// Parse triangle verticies from .stl file
		for (int i = 0; i < 20; i++) {
			if (file_in.good()) {
				string line;
				getline(file_in, line);
				//cout << line << endl;
				if (strstr(line.c_str(), key.c_str()) != NULL) {
					cout << line << endl;
				}
			}
			else {
				// Check for read failures or end of file
				if (file_in.fail()) {
					cout << "Read failed" << endl;
				}
				else if (file_in.eof()) {
					cout << "Reached end of file, closing input file" << endl;
				}
			}
		}
		

		file_in.close();
		file_out.close();
	}
	
	return 0;
}
