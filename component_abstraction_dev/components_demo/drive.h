#pragma once

#include "component.h"
#include "ComponentGraphManager.h"
#include "i2c_interface.h"
#include "motor.h"
#include <stdlib.h>
#include <string>

using namespace std;

class Drive : public Component{
 protected:
    Motor* m;

    void on_write(){
       log_info("running");
       m->write();
    }

 public:
    Drive(string file, ComponentGraphManager *cgm) : Component(file, cgm) {
        m = new Motor("Motor", cgm);
    }
};