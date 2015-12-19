#include "component.h"

using namespace std;


Component::Component(string name, ComponentGraphManager* graph){
    this->name = name;
    this->graph = graph;
    //Register ourselves with the component graph manager
    this->graph->add(name, this);
    enabled = true;
    connected = true;
    recovery = get_recovery(name);
    log_level = get_log_level(name);
    num_parents_disabled = 0;
}

bool Component::set_connect(bool new_val){
    //We just connected
    if (new_val && !connected){
        log_error("connected");
    }
    //We just disconnected
    if (!new_val && connected){
        log_error("disconnected");
        //We disconnected and want to disable this component 
        if (recovery == DISABLE){
            log_error("disabling");
            enable(false);
        }
        //We disconnected and want to kill the rover
        if (recovery == KILL_ROVER){
            log_error("killing rover");
            throw logic_error("Rover died\n");
        }
    }
    connected = new_val;
}

//logging stuff
//TODO: replace printf with ros log
void Component::log_info(string message){
    if (log_level == LOG) printf("%s: %s\n", name.c_str(), message.c_str());
}
void Component::log_error(string message){
    if (log_level == LOG || log_level == ERROR) printf(RED "%s ERROR: %s\n" RESET, name.c_str(), message.c_str());
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
    list<Component*> children = graph->get_children(name);
    for (std::list<Component*>::const_iterator iterator = children.begin(), end = children.end(); iterator != end; ++iterator) {
        Component* child = *iterator;
        child->set_to_default();
    }
    enable(get_default_enabled(name));
}

/** Set whether the rover is enabled or not. */
void Component::enable(bool new_val){
    //If we're being newly enabled
    if (new_val && !enabled && num_parents_disabled == 0) {
        //Propogate enable on to children
        list<Component*> children = graph->get_children(name);
        for (std::list<Component*>::const_iterator iterator = children.begin(), end = children.end(); iterator != end; ++iterator) {
            Component* child = *iterator;
            child->num_parents_disabled--;
            //Reset child to its defalt
            child->enable(get_default_enabled(child->name));
        }
        on_enable();
    }
    //If we're being newly disabled
    else if (!new_val && enabled){
        //Propogate disable on to children
        list<Component*> children = graph->get_children(name);
        for (std::list<Component*>::const_iterator iterator = children.begin(), end = children.end(); iterator != end; ++iterator) {
            Component* child = *iterator;
            child->num_parents_disabled++;
            child->enable(false);
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
    }catch(const invalid_argument& e){  //TODO: change this to cmr error
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
    }catch(const invalid_argument& e){  //TODO: change this to cmr error
        set_connect(false);   
    }
};
