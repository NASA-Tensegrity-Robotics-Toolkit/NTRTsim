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
