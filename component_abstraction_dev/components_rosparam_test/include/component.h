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

#include "ComponentGraphManager.h"

using namespace std;

//Avoid cyclic dependencies
class ComponentGraphManager;

/* An abstract Component class. All components should extend
   from this class. This provides functionalities such as automatic
   error handling, logging, and error recovery.  */
class Component{
 public:

    /* Options on how to try and recover from errors. 
        RECONNECT  - continuously attempt to reconnect to device
        DISABLE    - disable the device and forget about it
        KILL_ROVER - controlled shutdown of the entire rover */
    enum Recovery {RECONNECT, DISABLE, KILL_ROVER};

    /* Options on how to log messages from this device. 
       These map one-to-one with the log levels ROS provides. */
    enum LogLevel {DEBUG, INFO, WARN, ERROR, FATAL, NONE};

    /* Checks if the current device is connected. This
       stores whether our last attempt to communicate
       with the rover was successful or not. */
    bool is_connected();

    /* Checks if the current device is enabled. This is a boolean
       flag that can be set by the user.  */
    bool is_enabled();

    /* Is a device active and able to be interacted with. This is true
       only if the device is both connected and enabled. */
    bool is_active();

    /* Set the component's enabled status. */
    void enable(bool new_val);

    /* Set's the enable/disable status of this component
       and all its children back to their default. */
    void set_to_default();

    /* This device's read functionality. Contains both
       Component overhead read functionality and 
       logic custom to this device. */
    void read();

    /* This device's write functionality. Contains both
       Component overhead write functionality and 
       logic custom to this device. */
    void write();

    /* Returns a string representing the name of this component.
        This function MUST exist and be called getName() exactly
        because the ComponentGraphManager uses ROS's resource manager
        which requires a function with this name exist. */
    string getName() const;
    /* Alternative name for getName() function that can be used internally .*/
    string get_name() const;

 protected:
    /* Construct a new device. Requires a unique string identifier which
       matches its name in the YAML file and the component graph manager. */
    Component(string name, shared_ptr<ComponentGraphManager> graph);

    /* Log a message for this device. LogLevel takes the level the message is being logged at,
       and msg is the message. Will only be logged if this device is configured to log
       messages at the specified level. */
    void log(LogLevel level, string name, const char *fmt, ...);

    /* Subclasses can override to specify custom read functionality
       on top of the component functions. */
    virtual void on_read(){};

    /* Subclasses can override to specify custom write functionality
       on top of the component functions. */
    virtual void on_write(){};

    /* Subclasses can override to specify custom enable functionality
       on top of the component functions. */
    virtual void on_enable(){};

    /* Subclasses can override to specify custom disable functionality
       on top of the component functions. */
    virtual void on_disable(){}; 

    /* Reference to the component graph manager.
       Required to pass to new component constructors. */
    shared_ptr<ComponentGraphManager> graph;

private:
    /* Unique identifier for this component. */
    string name;
    /* Did the user set this component to enabled or not? */
    bool enabled;
    /* Was the last attempt at communicating with this component successful or not? */
    bool connected;
    /* How this component should handle disconnects */
    Recovery recovery; 
    /* What type of messages should we log for this component? */
    LogLevel log_level;
    /* How many parents are disabled? (must be 0 before we can be enabled) */
    int num_parents_disabled;

    /* The device has disconnected or reconnected. Should only be called
       after trying to connect and seeing what happened, not manually
       by the user. */
    bool set_connect(bool new_val);

    /* Converts a string to Recovery type. */
    Recovery string_to_recovery(string input);
    /* Converts a string to LogLevel type. */
    LogLevel string_to_log(string input);

     /* Parse the YAML file to get a list of Components
        that depend on us. */
    list<Component> get_dependents();
    /* Parse the YAML file to learn whether we're enabled by default or not. */
    bool get_default_enabled();
    /* Parse the YAML file to learn what our plan for recovery is. */
    Recovery get_recovery();
    /* Parse the YAML file to learn what level of logging we do. */
    LogLevel get_log_level();
};