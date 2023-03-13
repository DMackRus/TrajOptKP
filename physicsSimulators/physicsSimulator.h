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
    virtual bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName);
    virtual bool setRobotJointsPositions(string robotName, vector<double> jointPositions);
    virtual bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities);
    virtual bool setRobotJointsControls(string robotName, vector<double> jointControls);

    virtual bool getRobotJointsPositions(string robotName, vector<double> &jointPositions);
    virtual bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities);
    virtual bool getRobotJointsControls(string robotName, vector<double> &joinsControls);

    // ----- Body functions -----
    virtual bool isValidBodyName(string bodyName, int &bodyIndex);
    virtual bool setBodyPose_quat(string bodyName, pose_7 pose);
    virtual bool setBodyPose_angle(string bodyName, pose_6 pose);
    virtual bool setBodyVelocity(string bodyName, pose_6 velocity);

    virtual bool getBodyPose_quat(string bodyName, pose_7 &pose);
    virtual bool getBodyPose_angle(string bodyName, pose_6 &pose);
    virtual bool getBodyVelocity(string bodyName, pose_6 &velocity);

    // ----- Loading and saving system states -----
    virtual bool appendCurrentSystemStateToEnd();
    virtual bool saveSystemStateToIndex(int listIndex);
    virtual bool loadSystemStateFromIndex(int listIndex);
    virtual bool deleteSystemStateFromIndex(int listIndex);
    virtual bool clearSystemStateList();



    // ------------------------------- Visualisation -----------------------------------------
    virtual void initVisualisation();
    virtual void updateScene(GLFWwindow *window);
    virtual void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window);
    virtual void scroll(double yoffset);

    virtual void initSimulator(double timeStep, const char* fileName);
    virtual bool stepSimulator(int steps);


protected:
    vector<robot> robots;
    vector<string> bodies;

};

#endif //PHYSICSSIMSWITCHING_PHYSICSSIMULATOR_H
