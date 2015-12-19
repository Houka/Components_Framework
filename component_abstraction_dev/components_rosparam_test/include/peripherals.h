#pragma once

#include <component.h>
#include <ComponentGraphManager.h>
#include <ph_sensor.h>
#include <stdlib.h>
#include <string>

using namespace std;

class Peripherals : public Component{
 protected:
    PhSensor* ph;

    void on_read(){
       log(INFO, get_name(),"reading");
       ph->read();
    }

 public:
    Peripherals(string file, shared_ptr<ComponentGraphManager> cgm) : Component(file, cgm) {
        ph = new PhSensor("PHSensor", graph);
    }
};