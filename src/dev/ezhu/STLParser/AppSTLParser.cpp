#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <math.h>

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
		if (filename_in.find(".stl") == string::npos) {
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
		string line_out;
		int vertex_index = 0;
		int vertex_count = 0;
		int triangle_count = 0;
		int element_index = 0;
		int line_index = 0;
		// Parse triangle verticies from .stl file
		//for (int i = 0; i < 204; i++) {
		while (1) {
			if (file_in.good()) {
				string line_in;
				getline(file_in, line_in);
				line_index += 1;
				cout << "Lines processed: " << line_index << endl;
				//cout << line << endl;
				size_t found_key = line_in.find(key);
				// Look for "vertex" tag
				if (found_key != string::npos) {
					//cout << line_in << endl;
					// Coordinates are space delimited
					size_t found_char_last = line_in.find_first_of(" ", found_key);
					size_t found_char_curr;
					while (found_char_last != string::npos) {
						found_char_curr = line_in.find_first_of(" ", found_char_last + 1);
						string element;
						if (found_char_curr == string::npos) {
							element = line_in.substr(found_char_last + 1, line_in.size()-found_char_last);
							//cout << element << endl;
						}
						else {
							element = line_in.substr(found_char_last + 1, found_char_curr - 1 - found_char_last);
							//cout << element << endl;
						}
						size_t found_E = element.find_first_of("E");
						string coeff = element.substr(0, found_E);
						string exp = element.substr(found_E + 1, element.size() - found_E);
						//cout << coeff << " " << exp << endl;
						double coeff_num = atof(coeff.c_str());
						double exp_num = atof(exp.c_str());
						//cout << exp_num << endl;
						double num = coeff_num * pow(10.0, exp_num);
						//cout << num << endl;

						switch (element_index) {
							case 0:
								file_out << "[" << num << ", ";
								element_index += 1;
								break;
							case 1:
								file_out << num << ", ";
								element_index += 1;
								break;
							case 2:
								file_out << num << "] ";
								element_index = 0;
								break;
						}
						found_char_last = found_char_curr;
					}
					vertex_index += 1;
					vertex_count += 1;
					if (vertex_index >= 3) {
						file_out << endl;
						vertex_index = 0;
						triangle_count += 1;
					}
				}

			}
			else {
				// Check for read failures or end of file
				if (file_in.eof()) {
					cout << "Reached end of file, closing files" << endl;
					cout << "Number of verticies extracted: " << vertex_count << endl;
					cout << "Number of triangles: " << triangle_count << endl;
					break;
				}
				else if (file_in.fail()) {
					cout << "Read failed" << endl;
					break;
				}
				
			}
		}
		file_in.close();
		file_out.close();
	}
	
	return 0;
}
