#include "ModelTranslator/ArticulatedContact.h"

ArticulatedContact::ArticulatedContact(): ModelTranslator(){
    std::string yamlFilePath = "/TaskConfigs/toys/articulated_contact.yaml";
    InitModelTranslator(yamlFilePath);
}

bool ArticulatedContact::TaskComplete(mjData *d, double &dist){
    // Not implemented
    return false;
}

void ArticulatedContact::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;
    // --------------- Residual 0: Tip position -----------------
    residuals(resid_index++, 0) = 0;

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

void ArticulatedContact::ReturnRandomStartState(){
//    float arm1Pos = randFloat(0, 3);
//    float arm2Pos = randFloat(0, 3);
//    full_state_vector.robots[0].start_pos[0] = arm1Pos;
//    full_state_vector.robots[0].start_pos[1] = arm2Pos;
}

void ArticulatedContact::ReturnRandomGoalState(){
//    float randomNum = randFloat(0, 1);
//    float shoulder, elbow;
//    // stable down position
//    if(randomNum < 0.33){
//        shoulder = PI;
//        elbow = 0;
//    }
//        // Half up unstable
//    else if(randomNum > 0.33 && randomNum < 0.66){
//        shoulder = PI;
//        elbow = PI;
//    }
//        // Unstable up position
//    else{
//        shoulder = 0.0f;
//        elbow = 0.0f;
//    }
}

void ArticulatedContact::SetGoalVisuals(mjData *d) {
    // Not implemented
    // This function is supposed to set the visuals for the goal state in the MuJoCo simulation.
    // It can be used to visualize the target positions or orientations of the articulated contact.
}