#include <iostream>
#include <json/json.h>
#include "helpers/FileHelpers.h"

using namespace std;

int main() {
    cout<<"Hello"<<endl;
    cout<<FileHelpers::getFileString("data.txt")<<endl;
}
