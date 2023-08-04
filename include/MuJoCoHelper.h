//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_MUJOCOHELPER_H
#define PHYSICSSIMSWITCHING_MUJOCOHELPER_H

#include "stdInclude.h"
#include "mujoco.h"
#include <GLFW/glfw3.h>
#include <thread>

#define MAIN_DATA_STATE     -1
#define MASTER_RESET_DATA   -2

struct pose_7{
    m_point position;
    m_quat quat;
};

struct pose_6{
    m_point position;
    m_point orientation;
};

class MuJoCoHelper{
public:
    // Constructor
    MuJoCoHelper(vector<robot> robots, vector<string> _bodies);

    // Utility functions -- robots
    bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName);

    bool setRobotJointsPositions(string robotName, vector<double> jointPositions, std::shared_ptr<mjData> d);
    bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities, std::shared_ptr<mjData> d);
    bool setRobotJointsControls(string robotName, vector<double> jointControls, std::shared_ptr<mjData> d);

    bool getRobotJointsPositions(string robotName, vector<double> &jointPositions, std::shared_ptr<mjData> d);
    bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities, std::shared_ptr<mjData> d);
    bool getRobotJointsAccelerations(string robotName, vector<double> &jointsAccelerations, std::shared_ptr<mjData> d);
    bool getRobotJointsControls(string robotName, vector<double> &jointsControls, std::shared_ptr<mjData> d);
    bool getRobotJointsGravityCompensaionControls(string robotName, vector<double> &jointsControls, std::shared_ptr<mjData> d);

    // Utility functions -- bodies
    bool isValidBodyName(string bodyName, int &bodyIndex);
    bool setBodyPose_quat(string bodyName, pose_7 pose, std::shared_ptr<mjData> d);
    bool setBodyPose_angle(string bodyName, pose_6 pose, std::shared_ptr<mjData> d);
    bool setBodyVelocity(string bodyName, pose_6 velocity, std::shared_ptr<mjData> d);

    bool getBodyPose_quat(string bodyName, pose_7 &pose, std::shared_ptr<mjData> d);
    bool getBodyPose_angle(string bodyName, pose_6 &pose, std::shared_ptr<mjData> d);
    bool getBodyVelocity(string bodyName, pose_6 &velocity, std::shared_ptr<mjData> d);
    bool getBodyAcceleration(string bodyName, pose_6 &acceleration, std::shared_ptr<mjData> d);

    // Extras
    Eigen::MatrixXd calculateJacobian(std::string bodyName, std::shared_ptr<mjData> d);
    int checkSystemForCollisions(std::shared_ptr<mjData> d);
    bool checkBodyForCollisions(string bodyName, std::shared_ptr<mjData> d);

    // ----- Loading and saving system states -----
    bool appendSystemStateToEnd(std::shared_ptr<mjData> d);
    bool checkIfDataIndexExists(int dataIndex);
//    bool copySystemState(int dataDestinationIndex, int dataSourceIndex);
    bool deleteSystemStateFromIndex(int listIndex);
    bool clearSystemStateList();

    void cpMjData(const std::shared_ptr<mjModel> m, std::shared_ptr<mjData> d_dest, const std::shared_ptr<mjData> d_src);
    std::shared_ptr<mjData> returnDesiredDataState(std::shared_ptr<mjData> d);

    // ------------------------------- Visualisation -----------------------------------------
    void initVisualisation();
    void updateScene(GLFWwindow *window, const char* label);
    void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window);
    void scroll(double yoffset);
    mjvCamera cam;                   // abstract camera
    mjvScene scn;                    // abstract scene
    mjvOption opt;			        // visualization options
    mjrContext con;				    // custom GPU context

    void initSimulator(double timestep, const char* fileName);
    bool stepSimulator(int steps, std::shared_ptr<mjData> d);
    bool forwardSimulator(std::shared_ptr<mjData> d);
    bool forwardSimulatorWithSkip(std::shared_ptr<mjData> d, int skipStage, int skipSensor);

    void setupMuJoCoWorld(double timestep, const char* fileName);

    bool setBodyPosition(string bodyName, m_point position);

    void initModelForFiniteDifferencing();
    void resetModelAfterFiniteDifferencing();

    double returnModelTimeStep();

    double* sensorState(std::shared_ptr<mjData> d, std::string sensorName);

    vector<std::shared_ptr<mjData>> mjDataTrajectory;      // List of saved system states
    std::shared_ptr<mjData> mjDataMaster;                     // Master reset mujoco data
    std::shared_ptr<mjData> mjDataMain;                              // main MuJoCo data

    std::shared_ptr<mjModel> model;                             // MuJoCo model

    vector<robot> robots;
    vector<string> bodies;

private:
    int save_iterations;
    mjtNum save_tolerance;

};

#endif //PHYSICSSIMSWITCHING_MUJOCOHELPER_H
