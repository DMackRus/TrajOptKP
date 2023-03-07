//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_MUJOCOHELPER_H
#define PHYSICSSIMSWITCHING_MUJOCOHELPER_H

#include "physicsSimulator.h"
#include "mujoco.h"
#include "glfw3.h"

class MuJoCoHelper : public physicsSimulator {
public:
    // Constructor
    MuJoCoHelper(vector<robot> robots, vector<string> _bodies);

    // Utility functions -- robots
    bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName) override;
    bool setRobotJointsPositions(string robotName, vector<double> jointPositions) override;
    bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities) override;
    bool setRobotJointsControls(string robotName, vector<double> jointControls) override;

    bool getRobotJointsPositions(string robotName, vector<double> &jointPositions) override;
    bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities) override;
    bool getRobotJointsControls(string robotName, vector<double> &joinsControls) override;

    // Utility functions -- bodies
    bool isValidBodyName(string bodyName, int &bodyIndex) override;
    bool setBodyPose_quat(string bodyName, pose_7 pose) override;
    bool setBodyPose_angle(string bodyName, pose_6 pose) override;
    bool setBodyVelocity(string bodyName, pose_6 velocity) override;

    bool getBodyPose_quat(string bodyName, pose_7 &pose) override;
    bool getBodyPose_angle(string bodyName, pose_6 &pose) override;
    bool getBodyVelocity(string bodyName, pose_6 &velocity) override;

    // ------------------------------- Visualisation -----------------------------------------
    void initVisualisation() override;
    void updateScene(GLFWwindow *window) override;
    void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window) override;
    void scroll(double yoffset) override;
    mjvCamera cam;                   // abstract camera
    mjvScene scn;                    // abstract scene
    mjvOption opt;			        // visualization options
    mjrContext con;				    // custom GPU context

    bool stepSimulator(int steps) override;
    void setupMuJoCoWorld(double timestep, const char* fileName);

    bool setBodyPosition(string bodyName, m_point position);

private:
    mjData *mdata;                   // MuJoCo data
    mjModel *model;                  // MuJoCo model
};

#endif //PHYSICSSIMSWITCHING_MUJOCOHELPER_H
