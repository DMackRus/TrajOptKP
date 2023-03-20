//
// Created by dave on 09/03/23.
//

#include "doublePendulum.h"

doublePendulum::doublePendulum(): modelTranslator(){
    std::string yamlFilePath = "/home/davidrussell/catkin_ws/src/physicsSimSwitching/pendulumConfig.yaml";

    filePath = "/home/davidrussell/catkin_ws/src/physicsSimSwitching/Franka-emika-panda-arm/Acrobot.xml";
    vector<robot> robots;
    vector<bodyStateVec> bodies;

    loadRobotsandBodiesFromYAML(yamlFilePath, robots, bodies);




    // robot doublePendulum;
    // doublePendulum.name = "doublePendulum";
    // doublePendulum.jointNames = {"shoulder", "elbow"};
    // doublePendulum.numActuators = 2;
    // doublePendulum.jointPosCosts = {1, 1};
    // doublePendulum.jointVelCosts = {0, 0};
    // doublePendulum.jointControlCosts = {0.1, 0.1};
    // robots.push_back(doublePendulum);



    

    initModelTranslator(filePath, 2, robots, bodies);
    analyticalCostDerivatives = true;

    // Pendulum down stable position
    //X_desired << 3.1415, 0, 0, 0;

    // Pendulum unstable up configuration
    //X_desired << 0, 0, 0, 0;

    // Pendulum half untable position
    X_desired << 3.1415, 3.1415, 0, 0;

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