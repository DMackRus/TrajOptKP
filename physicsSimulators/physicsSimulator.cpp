//
// Created by dave on 01/03/23.
//

#include "physicsSimulator.h"


physicsSimulator::physicsSimulator(vector<robot> _robots, vector<string> _bodies){
    // populate robots
    for(int i = 0; i < _robots.size(); i++){
        robots.push_back(_robots[i]);
    }

    // populate bodies
    for(int i = 0; i < _bodies.size(); i++){
        bodies.push_back(_bodies[i]);
    }
}