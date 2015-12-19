#pragma once

#include "component.h"
#include "ComponentGraphManager.h"
#include "pca.h"
#include <stdlib.h>
#include <string>

using namespace std;

class Motor : public Component{
 protected:

    /** Read the value from the PhSensor */
    void on_write(){
        PCA* pca = (PCA*)graph->get("PCA"); 
        char* temp = pca->pca_write();
        string tempstr ="writing... value: ";
        tempstr.append(temp);
        log_info(tempstr);
    }

 public:
    Motor(string file, ComponentGraphManager *cgm) : Component(file, cgm) {
    }
};