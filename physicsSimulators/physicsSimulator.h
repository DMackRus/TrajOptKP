//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H
#define PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H

#include "../stdInclude/stdInclude.h"
#include <GLFW/glfw3.h>

struct robot{
    string name;
    vector<string> jointNames;
    int numActuators;
    vector<double> jointPosCosts;
    vector<double> jointVelCosts;
    vector<double> jointControlCosts;
};

struct pose_7{
    m_point position;
    m_quat quat;
};

struct pose_6{
    m_point position;
    m_point orientation;
};



class physicsSimulator {
public:
    // Constructor
    physicsSimulator(vector<robot> _robots, vector<string> _bodies);

    // ----- Robot functions -----
    virtual bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName) = 0;
    virtual bool setRobotJointsPositions(string robotName, vector<double> jointPositions) = 0;
    virtual bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities) = 0;
    virtual bool setRobotJointsControls(string robotName, vector<double> jointControls) = 0;

    virtual bool getRobotJointsPositions(string robotName, vector<double> &jointPositions) = 0;
    virtual bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities) = 0;
    virtual bool getRobotJointsAccelerations(string robotName, vector<double> &jointsAccelerations) = 0;
    virtual bool getRobotJointsControls(string robotName, vector<double> &jointsControls) = 0;

    // ----- Body functions -----
    virtual bool isValidBodyName(string bodyName, int &bodyIndex) = 0;
    virtual bool setBodyPose_quat(string bodyName, pose_7 pose) = 0;
    virtual bool setBodyPose_angle(string bodyName, pose_6 pose) = 0;
    virtual bool setBodyVelocity(string bodyName, pose_6 velocity) = 0;

    virtual bool getBodyPose_quat(string bodyName, pose_7 &pose) = 0;
    virtual bool getBodyPose_angle(string bodyName, pose_6 &pose) = 0;
    virtual bool getBodyVelocity(string bodyName, pose_6 &velocity) = 0;
    virtual bool getBodyAcceleration(string bodyName, pose_6 &acceleration) = 0;

    // ----- Loading and saving system states -----
    virtual bool appendCurrentSystemStateToEnd() = 0;
    virtual bool saveSystemStateToIndex(int listIndex) = 0;
    virtual bool loadSystemStateFromIndex(int listIndex) = 0;
    virtual bool deleteSystemStateFromIndex(int listIndex) = 0;
    virtual bool clearSystemStateList() = 0;

    // ------------------------------- Visualisation -----------------------------------------
    virtual void initVisualisation() = 0;
    virtual void updateScene(GLFWwindow *window) = 0;
    virtual void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window) = 0;
    virtual void scroll(double yoffset) = 0;

    virtual void initSimulator(double timeStep, const char* fileName) = 0;
    virtual bool stepSimulator(int steps) = 0;


protected:
    vector<robot> robots;
    vector<string> bodies;

};

#endif //PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H
