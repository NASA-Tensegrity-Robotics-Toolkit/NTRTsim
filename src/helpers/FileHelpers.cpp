#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "FileHelpers.h"

using namespace std;

std::string FileHelpers::getFileString(std::string fileName) {
    std::ifstream fileInput(fileName.c_str());
    stringstream buffer;
    buffer << fileInput.rdbuf();
    return buffer.str();
}
