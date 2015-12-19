#pragma once

#include <component.h>
#include <ComponentGraphManager.h>
#include <motor.h>
#include <stdlib.h>
#include <string>

using namespace std;

class Drive : public Component{
 protected:
    Motor* m;

    void on_write(){
       log(INFO, get_name(),"writing");
       m->write();
    }

 public:
    Drive(string file, shared_ptr<ComponentGraphManager> cgm) : Component(file, cgm) {
        m = new Motor("Motor", graph);
    }
};