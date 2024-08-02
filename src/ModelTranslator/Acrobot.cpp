#include "ModelTranslator/Acrobot.h"

Acrobot::Acrobot(): ModelTranslator(){
    std::string yamlFilePath = "/TaskConfigs/toys/acrobot.yaml";
    InitModelTranslator(yamlFilePath);
}

bool Acrobot::TaskComplete(mjData *d, double &dist){

    dist = 0.0;
    std::vector<double> acrobot_joints;
    MuJoCo_helper->GetRobotJointsPositions("acrobot", acrobot_joints, d);

    for(int i = 0; i < full_state_vector.dof; i++){
        std::cout << "joint pos " << acrobot_joints[i] << "\n";
        dist += abs(residual_list[i].target[0] - acrobot_joints[i]);
    }

    if(dist < 0.01){
        return true;
    }

    return false;
}

void Acrobot::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    std::vector<double> acrobot_joints;
    std::vector<double> acrobot_velocities;
    std::vector<double> acrobot_control;
    MuJoCo_helper->GetRobotJointsPositions("acrobot", acrobot_joints, d);
    MuJoCo_helper->GetRobotJointsVelocities("acrobot", acrobot_velocities, d);
    MuJoCo_helper->GetRobotJointsControls("acrobot", acrobot_control, d);

    // --------------- Residual 0: Joint 0 position -----------------
    residuals(resid_index++, 0) = acrobot_joints[0] - residual_list[0].target[0];

    // --------------- Residual 1: Joint 1 position -----------------
    residuals(resid_index++, 0) = acrobot_joints[1] - residual_list[1].target[0];

    // --------------- Residual 2: Joint 0 velocity -----------------
    residuals(resid_index++, 0) = acrobot_velocities[0] - residual_list[2].target[0];

    // --------------- Residual 3: Joint 1 velocity -----------------
    residuals(resid_index++, 0) = acrobot_velocities[1] - - residual_list[3].target[0];

    // --------------- Residual 4: Joint 0 control -----------------
    residuals(resid_index++, 0) = acrobot_control[0] - residual_list[4].target[0];

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

void Acrobot::ReturnRandomStartState(){
    float arm1Pos = randFloat(0, 3);
    float arm2Pos = randFloat(0, 3);
    full_state_vector.robots[0].start_pos[0] = arm1Pos;
    full_state_vector.robots[0].start_pos[1] = arm2Pos;
}

void Acrobot::ReturnRandomGoalState(){

    float randomNum = randFloat(0, 1);
    float shoulder, elbow;
    // stable down position
    if(randomNum < 0.33){
        shoulder = PI;
        elbow = 0;
    }
    // Half up unstable
    else if(randomNum > 0.33 && randomNum < 0.66){
        shoulder = PI;
        elbow = PI;
    }
    // Unstable up position
    else{
        shoulder = 0.0f;
        elbow = 0.0f;
    }

    // Positions of the acrobot joints
    residual_list[0].target[0] = shoulder;
    residual_list[1].target[0] = elbow;

    // Velocities of the acrobot joints
    residual_list[2].target[0] = 0.0;
    residual_list[3].target[0] = 0.0;

    // Control of the acrobot motor
    residual_list[4].target[0] = 0.0;


}
