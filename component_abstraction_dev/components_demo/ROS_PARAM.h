#include <list>
#include <string>
#include <fstream>
#include <streambuf>

#include "rapidjson/include/document.h"
#include "rapidjson/include/filereadstream.h"

using namespace std;
using namespace rapidjson;

#pragma once

enum Recovery {RECONNECT, DISABLE, KILL_ROVER};
enum LogLevel {NONE, LOG, ERROR};

bool string_to_bool(string input);
Recovery string_to_recovery(string input);
LogLevel string_to_log(string input);

void json_init(string file);
list<string> get_dependents(string file);
bool get_default_enabled(string name);
Recovery get_recovery(string name);
LogLevel get_log_level(string name);