#include "Hopper.h"

Hopper::Hopper(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/hopperConfig.yaml";
    InitModelTranslator(yamlFilePath);

}

bool Hopper::TaskComplete(int dataIndex, double &dist){
    return false;
}

void Hopper::GenerateRandomGoalAndStartState() {
    X_start.resize(state_vector_size, 1);
    X_desired.resize(state_vector_size, 1);

}

MatrixXd Hopper::ReturnRandomStartState(){
    MatrixXd randomStartState(state_vector_size, 1);

    return randomStartState;
}

MatrixXd Hopper::ReturnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(state_vector_size, 1);

    return randomGoalState;
}
