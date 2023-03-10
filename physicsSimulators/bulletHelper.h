//
// Created by dave on 07/03/23.
//

#ifndef PHYSICSSIMSWITCHING_BULLETHELPER_H
#define PHYSICSSIMSWITCHING_BULLETHELPER_H

#include "physicsSimulator.h"
// #include "btBulletDynamicsCommon.h"

//class bulletHelper : public physicsSimulator {
//public:
//    // Constructor
//    bulletHelper(vector<robot> robots, vector<string> _bodies);
//
//    // Utility functions -- robots
//    bool isValidRobotName(string robotName, int &robotIndex, string &robotBaseJointName) override;
//    bool setRobotJointsPositions(string robotName, vector<double> jointPositions) override;
//    bool setRobotJointsVelocities(string robotName, vector<double> jointVelocities) override;
//    bool setRobotJointsControls(string robotName, vector<double> jointControls) override;
//
//    bool getRobotJointsPositions(string robotName, vector<double> &jointPositions) override;
//    bool getRobotJointsVelocities(string robotName, vector<double> &jointVelocities) override;
//    bool getRobotJointsControls(string robotName, vector<double> &joinsControls) override;
//
//    // ------------------------------- Visualisation -----------------------------------------
//    void initVisualisation() override;
//    void updateScene(GLFWwindow *window) override;
//    void mouseMove(double dx, double dy, bool button_left, bool button_right,  GLFWwindow *window) override;
//    void scroll(double yoffset) override;
//
//
//private:
//
//    btDefaultCollisionConfiguration* collisionConfiguration;
//    btCollisionDispatcher* dispatcher;
//    btBroadphaseInterface* overlappingPairCache;
//    btSequentialImpulseConstraintSolver* solver;
//    btDiscreteDynamicsWorld* dynamicsWorld;
//
//    btAlignedObjectArray<btCollisionShape*> collisionShapes;
//
//};

#endif //PHYSICSSIMSWITCHING_BULLETHELPER_H
