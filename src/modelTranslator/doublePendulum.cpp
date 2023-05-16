//
// Created by dave on 09/03/23.
//

#include "doublePendulum.h"

doublePendulum::doublePendulum(): modelTranslator(){
    std::string yamlFilePath = "/taskConfigs/pendulumConfig.yaml";
    initModelTranslator(yamlFilePath);
}

bool doublePendulum::taskComplete(int dataIndex){
    double diff = 0.0f;

    MatrixXd Xt = returnStateVector(dataIndex);

    for(int i = 0; i < dof; i++){
        diff += abs(X_desired(i) - Xt(i));
    }

    if(diff < 0.01){
        return true;
    }
    return false;
}

MatrixXd doublePendulum::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    float arm1Pos = randFloat(0, 3);
    float arm2Pos = randFloat(0, 3);

    randomStartState << arm1Pos, arm2Pos, 0, 0;

    return randomStartState;
}

MatrixXd doublePendulum::returnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(stateVectorSize, 1);

    float randomNum = randFloat(0, 1);
    // stable down position
    if(randomNum < 0.33){
        randomGoalState << 3.1415, 0, 0, 0;
    }
    // Half up unstable
    else if(randomNum > 0.33 && randomNum < 0.66){
        randomGoalState << 3.1415, 3.1415, 0, 0;
    }
    // Unstable up position
    else{
        randomGoalState << 0, 0, 0, 0;
    }

    return randomGoalState;
}