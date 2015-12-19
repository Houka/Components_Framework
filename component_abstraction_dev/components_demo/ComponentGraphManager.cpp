#include "ComponentGraphManager.h"
#include "ROS_PARAM.h"
#include "ph_sensor.h"


void ComponentGraphManager::add(string id, Component* c){
    graph.insert(graph.begin(), pair<string, Component*>(id, c));
}
Component* ComponentGraphManager::get(string id){
    return graph[id];
}

/** Returns a list of its children*/
list<Component*> ComponentGraphManager::get_children(string id){
    list<Component*> children;
    list<string> childNames = get_dependents(id);
    for (auto s : childNames) {
        children.push_back(get(s));
    }
    return children;
}

bool check_consistency(){
    return false;
}

bool check_cyclic_dependency(){
    return false;
}