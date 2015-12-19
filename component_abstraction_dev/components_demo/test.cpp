#include "ph_sensor.h"
#include "rover.h"
#include <unistd.h>
#include "ComponentGraphManager.h"
#include "ROS_PARAM.h"

/*
    1) What should happen if data file is bad (parent disabled but child enabled or cyclic dependency)
    2) Cyclic dependency of header files and our fix
    3) Is there a better way to handle disable from multiple parents
    4) What is the best data struct for returning a list of pointers? list<Component*>? vector<Component*>? Component**?
    5) How to update default enabled/disabled without using second pass?
*/

int main(void){
    json_init("components_small.json");

    ComponentGraphManager graph;
    Rover* r = new Rover("Rover", &graph);

    while (1){
        r->read();
        r->write();
        usleep(100000);
    }
    return 0;
}