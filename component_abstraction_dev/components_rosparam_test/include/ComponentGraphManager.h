#pragma once

#include <map>
#include <list>
#include <string>
#include <unordered_set>
#include <ros/ros.h>    
#include <hardware_interface/internal/resource_manager.h>

#include "component.h"

using namespace std;

//Avoid cyclic dependency
class Component;

/* A ComponentGraphManager stores a map from component names to the objects
   representing these components. This is designed to be used internally to 
   component.cpp for things such as propogating errors, disables, etc to children.
   It also provides access to a NodeHandle to access the YAML component file. */
class ComponentGraphManager : public hardware_interface::ResourceManager<Component>
{
protected:
    /* Stores a reference to the node handle for our YAML file. */
    ros::NodeHandle component_handle;

    /* Checks the consistency of the YAML file provided. Returns true
        if OK, false otherwise. In case of error, logs it. */
    bool check_consistency();
public:
    /* Create a new graph manager. */
    ComponentGraphManager();

    /* Register component c with the graph. c MUST have a function
        called getName() which returns a string identifier for it,
        as required by ROS resource manager. */
    void add(Component& c);

    /* Get the component assigned with a given ID from the graph. */
    Component get(const string id);

    /* Get the node handle. */
    ros::NodeHandle get_handle();
private:
    /* Runtime check to verify if the YAML graph has any cyclic dependencies. 
       Returns true if OK, false if dependency found. Will log error to console
       if error found. */
    bool check_cyclic_dependency();
    /*  Helper function for check_cyclic_dependency().
        Checks whether we have a cyclic dependency from the graph rooted at parent.
        Will also fail if there is an invalid child name. 
        Assumes that visited is the set of nodes visited between the graph's topmost root
        and parent. */
    bool check_cyclic_dependency_from(string parent, unordered_set<string> visited);

    /* Runtime check to make sure that all the YAML is has valid formatting.
       Returns true if OK, false otherwise. Logs error to console if error found. */
    bool check_valid_formatting();

    /* Checks whether the loglevel is valid. Doesn't log anything. */
    bool valid_log_level(string input);
    /* Checks whether the recovery option is valid. Doesn't log anything. */
    bool valid_recovery(string input);
    /* Checks whether the input is a valid boolean. Doesn't log anything. */
    bool valid_bool(string input);
};