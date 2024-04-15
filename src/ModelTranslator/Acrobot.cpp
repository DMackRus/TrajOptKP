#include "Acrobot.h"

Acrobot::Acrobot(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/acrobotConfig.yaml";
    InitModelTranslator(yamlFilePath);
}

// TODO fix
bool Acrobot::TaskComplete(mjData *d, double &dist){
    double diff = 0.0f;

    MatrixXd Xt = ReturnStateVector(d);

    for(int i = 0; i < dof; i++){
//        diff += abs(X_desired(i) - Xt(i));
    }

    dist = diff;

    if(diff < 0.01){
        return true;
    }
    return false;
}

//MatrixXd Acrobot::ReturnRandomStartState(){
//    MatrixXd randomStartState(state_vector_size, 1);
//
//    float arm1Pos = randFloat(0, 3);
//    float arm2Pos = randFloat(0, 3);
//
//    randomStartState << arm1Pos, arm2Pos, 0, 0;
//
//    return randomStartState;
//}
//
//MatrixXd Acrobot::ReturnRandomGoalState(MatrixXd X0){
//    MatrixXd randomGoalState(state_vector_size, 1);
//
//    float randomNum = randFloat(0, 1);
//    // stable down position
//    if(randomNum < 0.33){
//        randomGoalState << 3.1415, 0, 0, 0;
//    }
//        // Half up unstable
//    else if(randomNum > 0.33 && randomNum < 0.66){
//        randomGoalState << 3.1415, 3.1415, 0, 0;
//    }
//        // Unstable up position
//    else{
//        randomGoalState << 0, 0, 0, 0;
//    }
//
//    return randomGoalState;
//}
