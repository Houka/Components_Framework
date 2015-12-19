#pragma once

#include <stdlib.h>
#include <string>

#include "component.h"
#include "ComponentGraphManager.h"
#include "i2c_interface.h"
#include "drive.h"
#include "diolan.h"
#include "pca.h"
#include "peripherals.h"

using namespace std;

class Rover : public Component{
 protected:
    Peripherals* sensors;
    Drive* drive;
    Diolan* diolan;
    PCA* pca;

    void on_read(){
       log_info("reading");
       sensors->read();
    }
    void on_write(){
		log_info("writing");
		drive->write();
    }

 public:
    Rover(string file, ComponentGraphManager *cgm) : Component(file, cgm) {
        sensors = new Peripherals("Peripherals", cgm);
    	drive = new Drive("Drive", cgm);
        diolan = new Diolan("Diolan", cgm);
        pca = new PCA("PCA", cgm);
        set_to_default();
    }
};