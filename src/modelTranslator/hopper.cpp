#include "hopper.h"

hopper::hopper(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/hopperConfig.yaml";
    InitModelTranslator(yamlFilePath);

}

bool hopper::TaskComplete(int dataIndex, double &dist){
    return false;
}

void hopper::GenerateRandomGoalAndStartState() {
    X_start.resize(state_vector_size, 1);
    X_desired.resize(state_vector_size, 1);

}

MatrixXd hopper::ReturnRandomStartState(){
    MatrixXd randomStartState(state_vector_size, 1);

    return randomStartState;
}

MatrixXd hopper::ReturnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(state_vector_size, 1);

    return randomGoalState;
}
