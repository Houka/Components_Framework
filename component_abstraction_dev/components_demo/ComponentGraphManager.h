#pragma once

#include <map>
#include <list>
#include <string>

#include "component.h"

using namespace std;

//Avoid cyclic dependency
class Component;

class ComponentGraphManager
{
private:
    bool check_cyclic_dependency();
protected:
    map<string, Component*> graph;
    bool check_consistency();
public:
    void add(string id, Component* c);
    Component* get(string id);
    list<Component*> get_children(string id);
};