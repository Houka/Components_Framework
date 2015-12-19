#pragma once

#include <component.h>
#include <ComponentGraphManager.h>
#include <stdlib.h>
#include <string>

using namespace std;

class Motor : public Component{
 protected:

    /** Read the value from the PhSensor */
    void on_write(){
        log(INFO, get_name(),"writing");
    }

 public:
    Motor(string file, shared_ptr<ComponentGraphManager> cgm) : Component(file, cgm) {
    }
};