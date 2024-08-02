//
// Created by dave on 26/06/23.
//
#include "ModelTranslator/Walker.h"

walker::walker(int terrain, int locomotion_type): ModelTranslator(){
    std::string yaml_file_path;

    if(locomotion_type == WALK){
        low_bound_velocity = 0.1;
        high_bound_velocity = 0.6;
        if(terrain == PLANE)
            yaml_file_path = "/TaskConfigs/locomotion/walk_plane.yaml";
        else if(terrain == UNEVEN)
            yaml_file_path = "/TaskConfigs/locomotion/walk_uneven.yaml";

    }
    else if(locomotion_type == RUN){
        low_bound_velocity = 0.9;
        high_bound_velocity = 1.3;
        yaml_file_path = "/TaskConfigs/locomotion/run_plane.yaml";
    }

    InitModelTranslator(yaml_file_path);
}

bool walker::TaskComplete(mjData *d, double &dist){
    dist = 0.0;
    return false;
}

void walker::ReturnRandomStartState(){

    double start_config[9] = {0, 0, 0, 1, -1, 0.2, 0, 0, 0};

    for(int i = 0; i < 9; i++){
        current_state_vector.robots[0].start_pos[i] = start_config[i];
    }
}

void walker::ReturnRandomGoalState(){

    // Stand at normal height
    residual_list[0].target[0] = 0.0;

    // Be upright
    residual_list[1].target[0] = 0.0;

    // Body velocity
    float rand_body_vel = randFloat(low_bound_velocity, high_bound_velocity);
    residual_list[2].target[0] = rand_body_vel;

    // Joint controls always want to be close to zero
    for(int i = 0; i < 6; i++){
        residual_list[3+i].target[0] = 0.0;
    }

}

void walker::Residuals(mjData *d, MatrixXd &residuals){
//    MatrixXd residuals(residual_list.size(), 1);
    int resid_index = 0;

    std::vector<double> walker_joints;
    std::vector<double> walker_velocities;
    std::vector<double> walker_controls;
    MuJoCo_helper->GetRobotJointsPositions("walker", walker_joints, d);
    MuJoCo_helper->GetRobotJointsVelocities("walker", walker_velocities, d);
    MuJoCo_helper->GetRobotJointsControls("walker", walker_controls, d);

    // --------------- Residual 0: Body height -----------------
    residuals(resid_index++, 0) = walker_joints[0] - residual_list[0].target[0];

    // --------------- Residual 1: Body rotation ---------------
    residuals(resid_index++, 0) = walker_joints[2] - residual_list[1].target[0];

    // --------------- Residual 2: Body velocity ---------------
    residuals(resid_index++, 0) = walker_velocities[1] - residual_list[2].target[0];

    // --------------- Residual 3: Joints controls -------------
    for(int i = 0; i < walker_controls.size(); i++){
        residuals(resid_index++, 0) = walker_controls[i] - residual_list[3+i].target[0];
    }

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }

//    return residuals;
}