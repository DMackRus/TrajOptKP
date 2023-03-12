//
// Created by dave on 09/03/23.
//

#include "doublePendulum.h"

doublePendulum::doublePendulum(): modelTranslator(){
    filePath = "/home/davidrussell/catkin_ws/src/physicsSimSwitching/Franka-emika-panda-arm/Acrobot.xml";
    pendulumDOF = 2;
    pendulumNumCtrl = 2;
    vector<robot> robots;
    robot doublePendulum;
    doublePendulum.name = "doublePendulum";
    doublePendulum.jointNames = {"shoulder", "elbow"};
    doublePendulum.numActuators = 2;
    robots.push_back(doublePendulum);


    vector<string> bodies;

    initModelTranslator(filePath, pendulumDOF, pendulumNumCtrl, robots, bodies);
    std::cout << "initialise double pendulum model translator" << std::endl;
}

double doublePendulum::costFunction(MatrixXd Xt, MatrixXd Ut, MatrixXd X_last, MatrixXd U_last){
    double cost = 0.0f;



    return cost;
}