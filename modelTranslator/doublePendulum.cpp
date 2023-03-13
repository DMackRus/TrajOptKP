//
// Created by dave on 09/03/23.
//

#include "doublePendulum.h"

doublePendulum::doublePendulum(): modelTranslator(){
    filePath = "/home/davidrussell/catkin_ws/src/physicsSimSwitching/Franka-emika-panda-arm/Acrobot.xml";
    pendulumNumCtrl = 2;
    vector<robot> robots;
    robot doublePendulum;
    doublePendulum.name = "doublePendulum";
    doublePendulum.jointNames = {"shoulder", "elbow"};
    doublePendulum.numActuators = 2;
    doublePendulum.jointPosCosts = {1, 1};
    doublePendulum.jointVelCosts = {0.1, 0.1};
    doublePendulum.jointControlCosts = {0.01, 0.01};
    robots.push_back(doublePendulum);

    vector<bodyStateVec> bodies;

    initModelTranslator(filePath, pendulumNumCtrl, robots, bodies);

    X_desired << 0, 3.14, 0, 0;

    std::cout << "initialise double pendulum model translator" << std::endl;
}

double doublePendulum::costFunction(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last){
    double cost = 0.0f;

    MatrixXd X_diff = Xt - X_desired;
    MatrixXd temp;

    temp = ((X_diff.transpose() * Q * X_diff)) + (Ut.transpose() * R * Ut);

    cost = temp(0);

    return cost;
}