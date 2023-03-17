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
    analyticalCostDerivatives = true;

    X_desired << 0, 3.14, 0, 0;

    std::cout << "initialise double pendulum model translator" << std::endl;
}

MatrixXd doublePendulum::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    float arm1Pos = randFloat(-2, 2);
    float arm2Pos = randFloat(-2, 2);

    randomStartState << arm1Pos, arm2Pos, 0, 0;

    return randomStartState;
}

MatrixXd doublePendulum::returnRandomGoalState(){
    MatrixXd randomGoalState(stateVectorSize, 1);

    float randomNum = randFloat(0, 1);
    // stable down position
    if(randomNum > 0.5){
        randomGoalState << 3.1415, 0, 0, 0;
    }
    // Unstable up position
    else{
        randomGoalState << 0, 0, 0, 0;
    }

    return randomGoalState;
}