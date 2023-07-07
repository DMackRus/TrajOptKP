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
    bool getRobotJointsGravityCompensaionControls(string robotName, vector<double> &jointsControls, int dataIndex) override;

    // Utility functions -- bodies
    bool isValidBodyName(string bodyName, int &bodyIndex) override;
    bool setBodyPose_quat(string bodyName, pose_7 pose, int dataIndex) override;
    bool setBodyPose_angle(string bodyName, pose_6 pose, int dataIndex) override;
    bool setBodyVelocity(string bodyName, pose_6 velocity, int dataIndex) override;

    bool getBodyPose_quat(string bodyName, pose_7 &pose, int dataIndex) override;
    bool getBodyPose_angle(string bodyName, pose_6 &pose, int dataIndex) override;
    bool getBodyVelocity(string bodyName, pose_6 &velocity, int dataIndex) override;
    bool getBodyAcceleration(string bodyName, pose_6 &acceleration, int dataIndex) override;

    // Extras
    Eigen::MatrixXd calculateJacobian(std::string bodyName, int dataIndex) override;
    int checkSystemForCollisions(int dataIndex) override;
    bool checkBodyForCollisions(string bodyName, int dataIndex) override;

    // ----- Loading and saving system states -----
    bool appendSystemStateToEnd(int dataIndex) override;
    bool checkIfDataIndexExists(int dataIndex) override;
    bool copySystemState(int dataDestinationIndex, int dataSourceIndex) override;
    bool deleteSystemStateFromIndex(int listIndex) override;
    bool clearSystemStateList() override;

    void cpMjData(const mjModel* m, mjData* d_dest, const mjData* d_src);
    mjData* returnDesiredDataState(int dataIndex);

    // ------------------------------- Visualisation -----------------------------------------
    void initVisualisation() override;
    void updateScene(GLFWwindow *window, const char* label) override;
    void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window) override;
    void scroll(double yoffset) override;
    mjvCamera cam;                   // abstract camera
    mjvScene scn;                    // abstract scene
    mjvOption opt;			        // visualization options
    mjrContext con;				    // custom GPU context

    void initSimulator(double timestep, const char* fileName) override;
    bool stepSimulator(int steps, int dataIndex) override;
    bool forwardSimulator(int dataIndex) override;

    void setupMuJoCoWorld(double timestep, const char* fileName);

    bool setBodyPosition(string bodyName, m_point position);

    vector<mjData*> savedSystemStatesList;
    mjData *d_master_reset;
    mjData *mdata;                   // main MuJoCo data
    mjModel *model;                  // MuJoCo model
    std::vector<mjData *> fd_data;   // Finite differencing MuJoCo data - instantiated with the number of cores on the pc

private:

};

#endif //PHYSICSSIMSWITCHING_MUJOCOHELPER_H
