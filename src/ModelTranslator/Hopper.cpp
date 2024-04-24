#include "ModelTranslator/Hopper.h"

Hopper::Hopper(): ModelTranslator(){
    std::string yamlFilePath = "/taskConfigs/hopperConfig.yaml";
    InitModelTranslator(yamlFilePath);

}

//MatrixXd Hopper::ReturnRandomStartState(){
//    MatrixXd randomStartState(state_vector_size, 1);
//
//    return randomStartState;
//}
//
//MatrixXd Hopper::ReturnRandomGoalState(MatrixXd X0){
//    MatrixXd randomGoalState(state_vector_size, 1);
//
//    return randomGoalState;
//}
