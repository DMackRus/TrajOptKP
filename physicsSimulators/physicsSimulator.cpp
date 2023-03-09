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

bool physicsSimulator::isValidBodyName(string bodyName, int &bodyIndex) {
    std::cout << "called is valid body name in base class" << std::endl;
    return false;
}

bool physicsSimulator::setBodyPose_quat(string bodyName, pose_7 pose) {
    std::cout << "called set body pose in base class" << std::endl;
    return false;
}

bool physicsSimulator::setBodyPose_angle(string bodyName, pose_6 pose) {
    std::cout << "called set body pose in base class" << std::endl;
    return false;
}

bool physicsSimulator::setBodyVelocity(string bodyName, pose_6 velocity) {
    std::cout << "called set body velocity in base class" << std::endl;
    return false;
}

bool physicsSimulator::getBodyPose_quat(string bodyName, pose_7 &pose) {
    std::cout << "called get body pose in base class" << std::endl;
    return false;
}

bool physicsSimulator::getBodyPose_angle(string bodyName, pose_6 &pose) {
    std::cout << "called get body pose in base class" << std::endl;
    return false;
}

bool physicsSimulator::getBodyVelocity(string bodyName, pose_6 &velocity) {
    std::cout << "called get body velocity in base class" << std::endl;
    return false;
}

bool physicsSimulator::stepSimulator(int steps){
    std::cout << "called step simulator in base class" << std::endl;
    return false;
}

void physicsSimulator::initSimulator() {
    std::cout << "Hello, World!" << std::endl;
}

void physicsSimulator::initVisualisation(){
    std::cout << "called init visualisation in base class" << std::endl;
}

void physicsSimulator::updateScene(GLFWwindow *window){
    std::cout << "called update scene in base class" << std::endl;
}

void physicsSimulator::mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window){
    std::cout << "called mouse move in base class" << std::endl;
}

void physicsSimulator::scroll(double yoffset){
    std::cout << "called scroll in base class" << std::endl;
}

bool physicsSimulator::appendCurrentSystemStateToEnd(){
    std::cout << "called append state in base class" << std::endl;
}

bool physicsSimulator::saveSystemStateToIndex(int listIndex){
    std::cout << "called save state in base class" << std::endl;
}

bool physicsSimulator::loadSystemStateFromIndex(int listIndex){
    std::cout << "called load state in base class" << std::endl;
}

bool physicsSimulator::deleteSystemStateFromIndex(int listIndex){
    std::cout << "called delete state in base class" << std::endl;
}

bool physicsSimulator::clearSystemStateList(){
    std::cout << "called clear state list in base class" << std::endl;
}
