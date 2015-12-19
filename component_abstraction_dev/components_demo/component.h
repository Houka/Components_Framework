#pragma once

#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <cstdio>
#include <string>
#include <fstream>
#include <streambuf>

#include "ROS_PARAM.h"
#include "ComponentGraphManager.h"

using namespace std;

#define RESET   "\033[0m"
#define RED     "\033[31m"      /* Red */

//Avoid cyclic dependencies
class ComponentGraphManager;


/* An abstract device class. Any device should inheret from this class. */
class Component{
 protected:

    string name;        //unique identifier for this component

    bool enabled;       //whether the user wants the component enabled or not
    bool connected;     //whether the component is currently functioning or not
    
    Recovery recovery;  //what should we do if the component disconnects?
    LogLevel log_level; //what types of messages should we log for this component?

    int num_parents_disabled; //how many of our parents are disabled? (must be 0 before we can enable)

    ComponentGraphManager* graph;

    /** Construct a new device. */
    Component(string name, ComponentGraphManager* graph);

    /** The device has disconnected or reconnected. */
    bool set_connect(bool new_val);

    void log_info(string message);
    void log_error(string message);

    virtual void on_read(){};
    virtual void on_write(){};

    /** interface methods*/
    virtual void on_enable(){};
    virtual void on_disable(){}; 

 public:
    /** Are we connected? */
    bool is_connected();
    /** Are we enabled? */
    bool is_enabled();
    /** Are we available to interact with? */
    bool is_active();

    /** Set whether the rover is enabled or not. */
    void enable(bool new_val);

    /** Sets enable/disable status of entire rover to default. */
    void set_to_default();

    /** What to do when this component is called to read */
    void read();

    /** What to do when this component is called to write */
    void write();
};