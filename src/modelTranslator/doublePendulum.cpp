//
// Created by dave on 09/03/23.
//

#include "doublePendulum.h"

doublePendulum::doublePendulum(): modelTranslator(){
    std::string yamlFilePath = "/taskConfigs/pendulumConfig.yaml";
    initModelTranslator(yamlFilePath);
    analyticalCostDerivatives = true;
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