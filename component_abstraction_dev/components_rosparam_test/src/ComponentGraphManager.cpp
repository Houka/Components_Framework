#include <ros/ros.h>
#include "ComponentGraphManager.h"

using namespace std;

ComponentGraphManager::ComponentGraphManager(){
    ros::NodeHandle nh("/components");
    component_handle = nh;
    //Verify consistency
    if (!check_consistency()){
        ROS_ERROR_STREAM("Component YAML file consistency check failed.");
    }
}

void ComponentGraphManager::add(Component& c){
    registerHandle(c);
}
Component ComponentGraphManager::get(const string id){
    return getHandle(id);
}

ros::NodeHandle ComponentGraphManager::get_handle(){
    return component_handle;
}

bool ComponentGraphManager::check_consistency(){
    return check_valid_formatting() && 
        check_cyclic_dependency();
}

bool ComponentGraphManager::check_valid_formatting(){
    bool valid = true;
    vector<string> components;
    component_handle.getParam("components", components);
    for (vector<string>::iterator it = components.begin(); it != components.end(); ++it){
        string child_id = *it;
        string val;
        char buf[50 + strlen(child_id.c_str())];
        sprintf(buf,"components/%s", child_id.c_str());
        //Verify enabled is a valid boolean
        char en_buf[50 + strlen(child_id.c_str())];
        sprintf(en_buf, "%s/enabled", buf);
        component_handle.getParam(en_buf, val);
        if (!valid_bool(val)){
            ROS_ERROR_STREAM(child_id << " component has bad enabled (" << val << ")");
            valid = false;
        }
        //Verify that on_disconnect is valid
        char dis_buf[50 + strlen(child_id.c_str())];
        sprintf(dis_buf, "%s/on_disconnect", buf);
        component_handle.getParam(dis_buf, val);
        if (!valid_recovery(val)){
            ROS_ERROR_STREAM(child_id << " component has bad recovery option (" << val << ")");
            valid = false;
        }
        //Verify that log level is valid
        char log_buf[50 + strlen(child_id.c_str())];
        sprintf(log_buf, "%s/log_level", buf);
        component_handle.getParam(log_buf, val);
        if (!valid_log_level(val)){
            ROS_ERROR_STREAM(child_id << " component has bad log level option (" << val << ")");
            valid = false;
        }
    }
    return valid;
}

bool ComponentGraphManager::valid_bool(string input){
    return input == "true" || input == "false";
}

bool ComponentGraphManager::valid_log_level(string input){
    return (input == "DEBUG" || input == "INFO" || input == "WARN"
        || input == "ERROR" || input == "FATAL" || input == "NONE");
}

bool ComponentGraphManager::valid_recovery(string input){
    return (input == "RECONNECT" || input == "DISABLE"
        || input == "KILL_ROVER");
}

/*  Checks whether we have a cyclic dependency from the graph rooted at parent.
    Will also fail if there is an invalid child name. 
    Assumes that visited is the set of nodes visited between the graph's topmost root
    and parent. */
bool ComponentGraphManager::check_cyclic_dependency_from(string parent, unordered_set<string> visited){
    //Add the parent to visited
    visited.insert(parent);
    //Get all the parent's children
    char buf[50 + strlen(parent.c_str())];
    sprintf(buf, "components/%s/dependents", parent.c_str());
    vector<string> children_names;
    if (!component_handle.getParam(buf, children_names)){
        ROS_ERROR_STREAM(parent << " component was referenced as a dependent but does not exist.");
        return false;
    }
    bool okay = true;
    for (vector<string>::iterator it = children_names.begin(); it != children_names.end(); ++it){
        string child_id = *it;
        //Verify that we've never seen the child before
        unordered_set<string>::const_iterator found = visited.find(child_id);
        if (found != visited.end()){
            ROS_ERROR_STREAM(parent << " has child " << child_id << " which is a cyclic dependency.");
            okay = false;
        }
        //Look at its children recursively
        else{
            okay &= check_cyclic_dependency_from(child_id, unordered_set<string>(visited));
        }   
    }
    return okay;
}

bool ComponentGraphManager::check_cyclic_dependency(){
    unordered_set<string> visited;
    return check_cyclic_dependency_from("Rover", visited);
}
