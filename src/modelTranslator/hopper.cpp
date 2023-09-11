#include "hopper.h"

hopper::hopper(): modelTranslator(){
    std::string yamlFilePath = "/taskConfigs/hopperConfig.yaml";
    initModelTranslator(yamlFilePath);

}

bool hopper::taskComplete(int dataIndex, double &dist){
    return false;
}

void hopper::generateRandomGoalAndStartState() {
    X_start.resize(stateVectorSize, 1);
    X_desired.resize(stateVectorSize, 1);

}

MatrixXd hopper::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    return randomStartState;
}

MatrixXd hopper::returnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(stateVectorSize, 1);

    return randomGoalState;
}
