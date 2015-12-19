#pragma once

#include "component.h"
#include "ComponentGraphManager.h"
#include "diolan.h"
#include <stdlib.h>
#include <string>

using namespace std;

class IMU : public Component{
 protected:

    /** Read the value from the PhSensor */
    void on_read(){
        Diolan* diolan = (Diolan*)graph->get("Diolan");      
        char* temp = diolan->i2c_read();
        string tempstr ="reading... value: ";
        tempstr.append(temp);
        log_info(tempstr);
    }

 public:
    IMU(string file, ComponentGraphManager *cgm) : Component(file, cgm) {
    }
};