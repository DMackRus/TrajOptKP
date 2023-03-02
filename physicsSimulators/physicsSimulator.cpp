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

bool physicsSimulator::isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName){
    std::cout << "called sis valid robot name in base class" << std::endl;
    return false;
}

bool physicsSimulator::setRobotJointsPositions(string robotName, vector<double> jointPositions) {
    std::cout << "called set robot joints in base class" << std::endl;
    return false;
}

bool physicsSimulator::setRobotJointsVelocities(string robotName, vector<double> jointVelocities) {
    std::cout << "called set robot joints in base class" << std::endl;
    return false;
}

// Do the same for the other functions
bool physicsSimulator::setRobotJointsControls(string robotName, vector<double> jointControls) {
    std::cout << "called set robot controls in base class" << std::endl;
    return false;
}

bool physicsSimulator::getRobotJointsPositions(string robotName, vector<double> &jointPositions) {
    std::cout << "called get robot joints in base class" << std::endl;
    return false;
}

bool physicsSimulator::getRobotJointsVelocities(string robotName, vector<double> &jointVelocities) {
    std::cout << "called get robot joints velocities in base class" << std::endl;
    return false;
}

bool physicsSimulator::getRobotJointsControls(string robotName, vector<double> &joinsControls) {
    std::cout << "called get robot joint controls in base class" << std::endl;
    return false;
}


void physicsSimulator::initSimulator() {
    std::cout << "Hello, World!" << std::endl;
}
