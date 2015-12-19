#pragma once

#include "component.h"
#include "ComponentGraphManager.h"
#include "i2c_interface.h"
#include <stdlib.h>
#include <string>

using namespace std;

class Diolan : public Component{
 protected:
    I2CInterface* i2c;

 public:
    Diolan(string file, shared_ptr<ComponentGraphManager> cgm) : Component(file, cgm) {
        i2c = new I2CInterface();
    }
    char* i2c_read(){
        return i2c->read(0);
    }
    char* i2c_write(){
        return i2c->read(0);
    }
};