#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <exception>
#include <stdexcept>
#include <sstream>

#include "component.h"

using namespace std;

class I2CInterface{
 int num_to_connect;
 public:
    I2CInterface() { 
        srand(time(NULL));
        num_to_connect = 0;
    }
    void write(uint8_t address, char* message){

    }

    char* int_to_char(int i){
        stringstream ss;
        ss << i;
        string temp_str = ss.str();
        return (char*) temp_str.c_str();
    }
    char* read(uint8_t address){
        if (num_to_connect > 0){
            num_to_connect--;
            throw invalid_argument("Read failed!");
        }
        //Random chance of failure
        int result = rand() % 10;
        char * c_result = int_to_char(result);
        if (result == 0){
            num_to_connect = rand() % 20 + 10;
            throw invalid_argument("Read failed!");
        }
        else{
            return c_result;
        }
    }
};