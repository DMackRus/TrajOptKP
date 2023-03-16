//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_MUJOCOHELPER_H
#define PHYSICSSIMSWITCHING_MUJOCOHELPER_H

#include "physicsSimulator.h"
#include "mujoco.h"
#include <GLFW/glfw3.h>

class MuJoCoHelper : public physicsSimulator {
public:
    // Constructor
    MuJoCoHelper(vector<robot> robots, vector<string> _bodies);

    // Utility functions -- robots
    bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName) override;
    bool setRobotJointsPositions(string robotName, vector<double> jointPositions, int dataIndex) override;
    bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities, int dataIndex) override;
    bool setRobotJointsControls(string robotName, vector<double> jointControls, int dataIndex) override;

    bool getRobotJointsPositions(string robotName, vector<double> &jointPositions, int dataIndex) override;
    bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities, int dataIndex) override;
    bool getRobotJointsAccelerations(string robotName, vector<double> &jointsAccelerations, int dataIndex) override;
    bool getRobotJointsControls(string robotName, vector<double> &jointsControls, int dataIndex) override;

    // Utility functions -- bodies
    bool isValidBodyName(string bodyName, int &bodyIndex) override;
    bool setBodyPose_quat(string bodyName, pose_7 pose, int dataIndex) override;
    bool setBodyPose_angle(string bodyName, pose_6 pose, int dataIndex) override;
    bool setBodyVelocity(string bodyName, pose_6 velocity, int dataIndex) override;

    bool getBodyPose_quat(string bodyName, pose_7 &pose, int dataIndex) override;
    bool getBodyPose_angle(string bodyName, pose_6 &pose, int dataIndex) override;
    bool getBodyVelocity(string bodyName, pose_6 &velocity, int dataIndex) override;
    bool getBodyAcceleration(string bodyName, pose_6 &acceleration, int dataIndex) override;

    // ----- Loading and saving system states -----
    bool appendSystemStateToEnd(int dataIndex) override;
    bool saveSystemStateToIndex(int saveDataIndex, int listIndex) override;
    bool loadSystemStateFromIndex(int loadDataIndex, int listIndex) override;
    bool deleteSystemStateFromIndex(int listIndex) override;
    bool clearSystemStateList() override;

    void cpMjData(const mjModel* m, mjData* d_dest, const mjData* d_src);

    // ------------------------------- Visualisation -----------------------------------------
    void initVisualisation() override;
    void updateScene(GLFWwindow *window) override;
    void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window) override;
    void scroll(double yoffset) override;
    mjvCamera cam;                   // abstract camera
    mjvScene scn;                    // abstract scene
    mjvOption opt;			        // visualization options
    mjrContext con;				    // custom GPU context

    void initSimulator(double timestep, const char* fileName) override;
    bool stepSimulator(int steps, int dataIndex) override;
    void setupMuJoCoWorld(double timestep, const char* fileName);

    bool setBodyPosition(string bodyName, m_point position);

    vector<mjData*> savedSystemStatesList;
    mjData *mdata;                   // MuJoCo data
    mjModel *model;                  // MuJoCo model

private:
   
    

};

#endif //PHYSICSSIMSWITCHING_MUJOCOHELPER_H
