#include <iostream>
#include <json/json.h>

using namespace std;

int main() {
    cout<<"Hello"<<endl;
    Json::Value val;
    val = Json::Value(30);
    cout<<"Value is "<<val.asUInt()<<endl;
    cout<<"It works!"<<endl;
}
