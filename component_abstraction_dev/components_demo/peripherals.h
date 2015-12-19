#pragma once

#include "component.h"
#include "ComponentGraphManager.h"
#include "ph_sensor.h"
#include "imu.h"
#include <stdlib.h>
#include <string>

using namespace std;

class Peripherals : public Component{
 protected:
    PhSensor* ph;
    IMU* imu;

    void on_read(){
       log_info("running");
       ph->read();
       imu->read();
    }

 public:
    Peripherals(string file, ComponentGraphManager *cgm) : Component(file, cgm) {
        ph = new PhSensor("PHSensor", cgm);
        imu = new IMU("IMU", cgm);
    }
};