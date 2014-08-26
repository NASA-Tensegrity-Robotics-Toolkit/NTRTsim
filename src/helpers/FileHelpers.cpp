#include <iostream>
#include <string>
#include <fstream>
#include <stringstream>

using namespace std;

void FileHelpers::getFileString(string fileName) {
    ifStream fileInput(fileName);
    stringstream buffer;
    buffer << fileInput.rdbuf()
    return buffer.str();
}
