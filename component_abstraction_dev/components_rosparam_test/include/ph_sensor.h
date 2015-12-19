#pragma once

#include "component.h"
#include "ComponentGraphManager.h"
#include "diolan.h"
#include <stdlib.h>
#include <string>

using namespace std;

class PhSensor : public Component{
 protected:

    /** Read the value from the PhSensor */
    void on_read(){
        log(INFO,get_name(), "reading");
    }

 public:
    PhSensor(string file, shared_ptr<ComponentGraphManager> cgm) : Component(file, cgm) {
    }
};