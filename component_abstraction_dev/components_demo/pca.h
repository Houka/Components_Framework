#pragma once

#include "component.h"
#include "ComponentGraphManager.h"
#include "diolan.h"
#include <stdlib.h>
#include <string>

using namespace std;

class PCA : public Component{
 protected:

 public:
    PCA(string file, ComponentGraphManager *cgm) : Component(file, cgm) {
    }

    char* pca_write(){
        Diolan* diolan = (Diolan*)graph->get("Diolan");
        return diolan->i2c_read();
    }
};