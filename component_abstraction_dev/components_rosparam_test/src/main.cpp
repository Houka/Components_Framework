#include <ph_sensor.h>
#include <rover.h>
#include <unistd.h>
#include <ComponentGraphManager.h>
#include <ros/ros.h>

/*
    1) What should happen if data file is bad (parent disabled but child enabled or cyclic dependency)
    2) Cyclic dependency of header files and our fix
    3) Is there a better way to handle disable from multiple parents
    4) What is the best data struct for returning a list of pointers? list<Component*>? vector<Component*>? Component**?
    5) How to update default enabled/disabled without using second pass?
*/

int main(int argc, char** argv){
    int loop_rate = 10;

    ros::init(argc, argv, "components_test");
    ros::AsyncSpinner spinner(1);
 
    Component* robot = new Rover("Rover", shared_ptr<ComponentGraphManager>(new ComponentGraphManager()));


    try {

        ros::Time previous = ros::Time::now();
        spinner.start();

        ros::Rate r(loop_rate);

        while (ros::ok()) {
            robot->read();
            previous = ros::Time::now();
            robot->write();
            r.sleep();
        }
    } catch (std::runtime_error e) {
        ROS_ERROR_STREAM(e.what());
        ros::shutdown();
    }

    // while (1){
    //     r->read();
    //     r->write();
    //     usleep(100000);
    // }
    return 0;
}