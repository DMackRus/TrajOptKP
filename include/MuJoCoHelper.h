//
// Created by dave on 01/03/23.
//

#ifndef PHYSICSSIMSWITCHING_MUJOCOHELPER_H
#define PHYSICSSIMSWITCHING_MUJOCOHELPER_H

#include "PhysicsSimulator.h"
#include "mujoco.h"
#include <GLFW/glfw3.h>

class MuJoCoHelper : public PhysicsSimulator {
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
    void saveDataToRolloutBuffer(int dataIndex, int rolloutIndex) override;
    void copyRolloutBufferToSavedSystemStatesList() override;

    void cpMjData(const std::shared_ptr<mjModel> m, std::shared_ptr<mjData> d_dest, const std::shared_ptr<mjData> d_src);
    std::shared_ptr<mjData> returnDesiredDataState(int dataIndex);

    void _mjdTransitionFD();


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
    bool forwardSimulatorWithSkip(int dataIndex, int skipStage, int skipSensor) override;

    void setupMuJoCoWorld(double timestep, const char* fileName);

    bool setBodyPosition(string bodyName, m_point position);

    void initModelForFiniteDifferencing() override;
    void resetModelAfterFiniteDifferencing() override;

    double returnModelTimeStep() override;

    double* sensorState(int dataIndex, std::string sensorName) override;

    vector<std::shared_ptr<mjData>> savedSystemStatesList;      // List of saved system states
    vector<std::shared_ptr<mjData>> fp_rollout_data;            // forwards pass rollout data
    std::shared_ptr<mjData> d_master_reset;                     // Master reset mujoco data
    std::shared_ptr<mjData> mdata;                              // main MuJoCo data
    std::shared_ptr<mjData> vis_data;                           // Visualisation MuJoCo data
    std::shared_ptr<mjModel> model;                             // MuJoCo model
    std::vector<std::shared_ptr<mjData>> fd_data;               // Finite differencing MuJoCo data - instantiated with the number of cores on the pc

private:
    int save_iterations;
    mjtNum save_tolerance;

};

#endif //PHYSICSSIMSWITCHING_MUJOCOHELPER_H
