#include "component.h"
#include "exceptions/cmr_exceptions.h"
#include <stdarg.h>
#include <stdio.h>
#include <ros/ros.h>

using namespace std;

#define DEFAULT_ENABLE false
#define DEFAULT_RECOVERY RECONNECT
#define DEFAULT_LOG_LEVEL NONE


Component::Component(string name, shared_ptr<ComponentGraphManager> graph){
    this->name = name;
    this->graph = graph;
    //Register ourselves with the component graph manager
    this->graph->add(*this);
    enabled = true;
    connected = true;
    recovery = get_recovery();
    log_level = get_log_level();
    num_parents_disabled = 0;

    log(DEBUG, name, "loaded");
}

bool Component::set_connect(bool new_val){
    //We just connected
    if (new_val && !connected){
        log(WARN, name, "connected");
    }
    //We just disconnected
    if (!new_val && connected){
        log(WARN, name, "disconnected");
        //We disconnected and want to disable this component 
        if (recovery == DISABLE){
            log(ERROR, name, "disabling");
            enable(false);
        }
        //We disconnected and want to kill the rover
        if (recovery == KILL_ROVER){
            log(FATAL, name, "killing rover");
            throw cmr::cmr_error("Rover died");
        }
    }
    connected = new_val;
}

void Component::log(LogLevel level, string name, const char *fmt, ...){
    // converts printf args into string
    string msg;
    char buffer[256];
    va_list args;
    va_start (args, fmt);
    vsprintf (buffer,fmt, args);
    va_end (args);
    msg = "("+name+") "+(string)buffer;
    
    // logs at appropriate level
    switch(level){
        case INFO:
            if (log_level <= INFO) ROS_INFO_STREAM_NAMED(name.c_str(), msg.c_str());
            break;
        case DEBUG:
            if (log_level <= DEBUG) ROS_DEBUG_STREAM_NAMED(name.c_str(), msg.c_str());
            break;
        case WARN:
            if (log_level <= WARN) ROS_WARN_STREAM_NAMED(name.c_str(), msg.c_str());
            break;
        case ERROR:
            if (log_level <= ERROR) ROS_ERROR_STREAM_NAMED(name.c_str(), msg.c_str());
            break;
        case FATAL:
            if (log_level <= FATAL) ROS_FATAL_STREAM_NAMED(name.c_str(), msg.c_str());
            break;
        default:
            break;
    }
}

bool Component::is_connected(){
    return connected;
}
/** Are we enabled? */
bool Component::Component::is_enabled(){
    return enabled;
}
/** Are we available to interact with? */
bool Component::is_active(){
    return is_connected() && is_enabled();
}

void Component::set_to_default(){
    list<Component> children = get_dependents();
    for (std::list<Component>::const_iterator iterator = children.begin(), end = children.end(); iterator != end; ++iterator) {
        Component child = *iterator;
        child.set_to_default();
    }
    enable(get_default_enabled());
}

/** Set whether the rover is enabled or not. */
void Component::enable(bool new_val){
    //If we're being newly enabled
    if (new_val && !enabled && num_parents_disabled == 0) {
        //Propogate enable on to children
        list<Component> children = get_dependents();
        for (std::list<Component>::const_iterator iterator = children.begin(), end = children.end(); iterator != end; ++iterator) {
            Component child = *iterator;
            child.num_parents_disabled--;
            //Reset child to its defalt
            child.enable(child.get_default_enabled());
        }
        on_enable();
    }
    //If we're being newly disabled
    else if (!new_val && enabled){
        //Propogate disable on to children
        list<Component> children = get_dependents();
        for (std::list<Component>::const_iterator iterator = children.begin(), end = children.end(); iterator != end; ++iterator) {
            Component child = *iterator;
            child.num_parents_disabled++;
            child.enable(false);
        }
        on_disable();
    }
    enabled = new_val;
}

/** What to do when this component is called to read */
void Component::read(){
    try{
        if (is_enabled()){
            on_read();
            //If read succeeds, reconnect was successful
            set_connect(true);
        }
    }catch(cmr::cmr_error& e){ 
        set_connect(false);
    }
};

/** What to do when this component is called to write */
void Component::write(){
    try{
        if (is_enabled()){
            on_write();
            //If write succeeds, reconnect was successful
            set_connect(true);
        }
    }catch(cmr::cmr_error& e){ 
        set_connect(false);   
    }
};

/** Some helpful conversion functions for enum types. */
Component::Recovery Component::string_to_recovery(string input){
    if (input == "DISABLE") return DISABLE;
    else if (input == "KILL_ROVER") return KILL_ROVER;
    else return DEFAULT_RECOVERY;
}

Component::LogLevel Component::string_to_log(string input){
    if (input == "INFO") return INFO;
    if (input == "DEBUG") return DEBUG;
    if (input == "WARN") return WARN;
    if (input == "ERROR") return ERROR;
    if (input == "FATAL") return FATAL;
    return DEFAULT_LOG_LEVEL;
}

list<Component> Component::get_dependents(){
    list<Component> children;
    char buf[50 + strlen(name.c_str())];
    sprintf(buf, "components/%s/dependents", name.c_str());
    vector<string> children_names;
    if (!graph->get_handle().getParam(buf, children_names)){
        log(ERROR, name, "Error loading dependents from data file");
        return children;
    }
    for (vector<string>::iterator it = children_names.begin(); it != children_names.end(); ++it){
        string child_id = *it;
        children.push_back(graph->get(child_id));
    }
    return children;
}

bool Component::get_default_enabled(){
    bool enabled;
    char buf[50 + strlen(name.c_str())];
    sprintf(buf, "components/%s/enabled", name.c_str());
    if (!graph->get_handle().getParam(buf, enabled)){
        log(ERROR, name, "Error loading default enabled from data file");
        return DEFAULT_ENABLE;
    }
    if (enabled) return true;
    else return false;
}

Component::Recovery Component::get_recovery(){
    string recovery;
    char buf[50 + strlen(name.c_str())];
    sprintf(buf, "components/%s/on_disconnect", name.c_str());
    if (!graph->get_handle().getParam(buf, recovery)){
        log(ERROR, name, "Error loading recovery option from data file");
        return DEFAULT_RECOVERY;
    }
    return string_to_recovery(recovery);
}

Component::LogLevel Component::get_log_level(){
    string log_level;
    char buf[50 + strlen(name.c_str())];
    sprintf(buf, "components/%s/log_level", name.c_str());
    if (!graph->get_handle().getParam(buf, log_level)){
        log(ERROR, name, "Error loading log level from data file");
        return DEFAULT_LOG_LEVEL;
    }
    return string_to_log(log_level);
}

string Component::get_name() const{
    return name;
}

/* This function is required by ComponentGraphManager and 
   ROS's resource manager. DO NOT DELETE OR RENAME IT. */
string Component::getName() const{
    return get_name();
}
