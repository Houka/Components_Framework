#include "ROS_PARAM.h"

Document document;

bool string_to_bool(string input){
    if (input == "TRUE") return 1;
    else return 0;
}

Recovery string_to_recovery(string input){
    if (input == "DISABLE") return DISABLE;
    else if (input == "KILL_ROVER") return KILL_ROVER;
    else return RECONNECT;
}

LogLevel string_to_log(string input){
    if (input == "LOG") return LOG;
    else if (input == "ERROR") return ERROR;
    else return NONE;
}

void json_init(string file){
    //Load the JSON file into a string
    ifstream t(file.c_str());
    string str((std::istreambuf_iterator<char>(t)), istreambuf_iterator<char>());
    //Parse the JSON file
    document.Parse(str.c_str());
}

list<string> get_dependents(string name){
    //Create the list to return
    list<string> children;
    const Value& a = document[name.c_str()];
    const Value& b = a["dependents"];
    for (rapidjson::SizeType i = 0; i < b.Size(); i++){
        children.push_back(b[i].GetString());
    }
    return children;
}

bool get_default_enabled(string name){
    return string_to_bool(document[name.c_str()]["enabled"].GetString());
}

Recovery get_recovery(string name){
    return string_to_recovery(document[name.c_str()]["on_disconnect"].GetString());
}

LogLevel get_log_level(string name){
    return string_to_log(document[name.c_str()]["log_level"].GetString());
}