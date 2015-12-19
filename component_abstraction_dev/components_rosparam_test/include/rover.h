#pragma once

#include <stdlib.h>
#include <string>

#include <component.h>
#include <ComponentGraphManager.h>
#include <drive.h>
#include <peripherals.h>
#include <diolan.h>
#include <ros/ros.h>

using namespace std;

class Rover : public Component{
 protected:
    Peripherals* sensors;
    Drive* drive;
    Diolan* diolan;

    void on_read(){
        //diolan shit
        //just for random error throwing

        char* temp = diolan->i2c_read();
        log(INFO, get_name(), "reading... value: %s", temp);

        
        log(INFO, get_name(),"reading");
        sensors->read();
    }
    void on_write(){
		log(INFO, get_name(),"writing");
		drive->write();
    }

 public:
    Rover(string file, shared_ptr<ComponentGraphManager> cgm) : Component(file, cgm) {
        
        sensors = new Peripherals("Peripherals", graph);
        log(DEBUG, get_name(),"%s made", "Sensors");

    	drive = new Drive("Drive", graph);
        log(DEBUG, get_name(),"%s made", "Drive");

        diolan = new Diolan("Diolan", graph);

        set_to_default(); // needed here so all children will set themselves to default enabled
        log(DEBUG, get_name(),"setting enabled to default value");
    }
};