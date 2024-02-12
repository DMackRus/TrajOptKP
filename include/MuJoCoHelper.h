//
// Created by dave on 01/03/23.
//

#pragma once

#include "StdInclude.h"
#include "mujoco.h"
#include <GLFW/glfw3.h>
#include <thread>

#define MAIN_DATA_STATE     -1
#define MASTER_RESET_DATA   -2
#define VISUALISATION_DATA  -3

struct pose_7{
    m_point position;
    m_quat quat;
};

struct pose_6{
    m_point position;
    m_point orientation;
};

class MuJoCoHelper {
public:
    // Constructor
    MuJoCoHelper(vector<robot> robots, vector<string> _bodies);

    // Utility functions -- robots
    bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName);
    bool setRobotJointsPositions(string robotName, vector<double> jointPositions, int dataIndex);
    bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities, int dataIndex);
    bool setRobotJointsControls(string robotName, vector<double> jointControls, int dataIndex);

    bool getRobotJointsPositions(string robotName, vector<double> &jointPositions, int dataIndex);
    bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities, int dataIndex);
    bool getRobotJointsAccelerations(string robotName, vector<double> &jointsAccelerations, int dataIndex);
    bool getRobotJointsControls(string robotName, vector<double> &jointsControls, int dataIndex);
    bool getRobotJointsGravityCompensaionControls(string robotName, vector<double> &jointsControls, int dataIndex);

    // Utility functions -- bodies
    bool isValidBodyName(string bodyName, int &bodyIndex);
    bool setBodyPose_quat(string bodyName, pose_7 pose, int dataIndex);
    bool setBodyPose_angle(string bodyName, pose_6 pose, int dataIndex);
    bool setBodyVelocity(string bodyName, pose_6 velocity, int dataIndex);

    bool getBodyPose_quat(string bodyName, pose_7 &pose, int dataIndex);
    bool getBodyPose_angle(string bodyName, pose_6 &pose, int dataIndex);
    bool getBodyVelocity(string bodyName, pose_6 &velocity, int dataIndex);
    bool getBodyAcceleration(string bodyName, pose_6 &acceleration, int dataIndex);

    bool getBodyPose_quat_ViaXpos(string bodyName, pose_7 &pose, int dataIndex);
    bool getBodyPose_angle_ViaXpos(string bodyName, pose_6 &pose, int dataIndex);

    // Extras
    Eigen::MatrixXd calculateJacobian(std::string bodyName, int dataIndex);
    int checkSystemForCollisions(int dataIndex);
    bool checkBodyForCollisions(string bodyName, int dataIndex);

    // ----- Loading and saving system states -----
    bool appendSystemStateToEnd(int dataIndex);
    bool checkIfDataIndexExists(int dataIndex);
    bool copySystemState(int dataDestinationIndex, int dataSourceIndex);
    bool deleteSystemStateFromIndex(int listIndex);
    bool clearSystemStateList();
    void saveDataToRolloutBuffer(int dataIndex, int rolloutIndex);
    void copyRolloutBufferToSavedSystemStatesList();

    void cpMjData(const std::shared_ptr<mjModel> m, std::shared_ptr<mjData> d_dest, const std::shared_ptr<mjData> d_src);
    std::shared_ptr<mjData> returnDesiredDataState(int dataIndex);

    void _mjdTransitionFD();


    // ------------------------------- Visualisation -----------------------------------------
    void initVisualisation();
    void updateScene(GLFWwindow *window, const char* label);
    void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window);
    void scroll(double yoffset);

    void initSimulator(double timestep, const char* fileName);
    bool stepSimulator(int steps, int dataIndex);
    bool forwardSimulator(int dataIndex);
    bool forwardSimulatorWithSkip(int dataIndex, int skipStage, int skipSensor);

    void setupMuJoCoWorld(double timestep, const char* fileName);

    bool setBodyPosition(string bodyName, m_point position);

    void initModelForFiniteDifferencing();
    void resetModelAfterFiniteDifferencing();

    double returnModelTimeStep();

    double* sensorState(int dataIndex, std::string sensorName);

    vector<std::shared_ptr<mjData>> savedSystemStatesList;      // List of saved system states
    vector<std::shared_ptr<mjData>> fp_rollout_data;            // forwards pass rollout data
    std::shared_ptr<mjData> d_master_reset;                     // Master reset mujoco data
    std::shared_ptr<mjData> mdata;                              // main MuJoCo data
    std::shared_ptr<mjData> vis_data;                           // Visualisation MuJoCo data
    std::shared_ptr<mjModel> model;                             // MuJoCo model
    std::vector<std::shared_ptr<mjData>> fd_data;               // Finite differencing MuJoCo data - instantiated with the number of cores on the pc

    mjvCamera cam;                   // abstract camera
    mjvScene scn;                    // abstract scene
    mjvOption opt;			        // visualization options
    mjrContext con;				    // custom GPU context

private:
    int save_iterations;
    mjtNum save_tolerance;

    vector<robot> robots;
    vector<string> bodies;

};