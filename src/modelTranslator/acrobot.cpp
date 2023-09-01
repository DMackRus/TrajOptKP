#include "acrobot.h"

acrobot::acrobot(): modelTranslator(){
    std::string yamlFilePath = "/taskConfigs/acrobotConfig.yaml";
    initModelTranslator(yamlFilePath);
}

bool acrobot::taskComplete(int dataIndex, double &dist){
    double diff = 0.0f;

    MatrixXd Xt = returnStateVector(dataIndex);

    for(int i = 0; i < dof; i++){
        diff += abs(X_desired(i) - Xt(i));
    }

    dist = diff;

    if(diff < 0.01){
        return true;
    }
    return false;
}

void acrobot::generateRandomGoalAndStartState() {
    X_start.resize(stateVectorSize, 1);
    X_desired.resize(stateVectorSize, 1);

    float arm1Pos = randFloat(0, 3);
    float arm2Pos = randFloat(0, 3);

    X_start << arm1Pos, arm2Pos, 0, 0;

    float randomNum = randFloat(0, 1);
    // stable down position
    if(randomNum < 0.33){
        X_desired << 3.1415, 0, 0, 0;
    }
        // Half up unstable
    else if(randomNum > 0.33 && randomNum < 0.66){
        X_desired << 3.1415, 3.1415, 0, 0;
    }
        // Unstable up position
    else{
        X_desired << 0, 0, 0, 0;
    }

}

MatrixXd acrobot::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    float arm1Pos = randFloat(0, 3);
    float arm2Pos = randFloat(0, 3);

    randomStartState << arm1Pos, arm2Pos, 0, 0;

    return randomStartState;
}

MatrixXd acrobot::returnRandomGoalState(MatrixXd X0){
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
