#include "ModelTranslator/PistonBlock.h"

PistonBlock::PistonBlock(){
    std::string task_config = "/TaskConfigs/toys/piston_box.yaml";
    InitModelTranslator(task_config);
}

void PistonBlock::ReturnRandomStartState(){
    current_state_vector.robots[0].start_pos[0] = 0.0;

    // Initialise the body
    current_state_vector.rigid_bodies[0].start_linear_pos[0] = 0.0;
    current_state_vector.rigid_bodies[0].start_linear_pos[1] = 0.9;
    current_state_vector.rigid_bodies[0].start_linear_pos[2] = 0.1;

    current_state_vector.rigid_bodies[0].start_angular_pos[0] = 0.0;
    current_state_vector.rigid_bodies[0].start_angular_pos[1] = 0.0;
    current_state_vector.rigid_bodies[0].start_angular_pos[2] = 0.0;
}

void PistonBlock::ReturnRandomGoalState(){

    // Box position
    residual_list[0].target[0] = randFloat(1.0, 2.0);

    // Box velocity
    residual_list[1].target[0] = 0.0;
}

std::vector<MatrixXd> PistonBlock::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> init_controls;

    // Create controls where we move forward to contact the box
    for(int t = 0; t < horizonLength; t++){
        MatrixXd control(current_state_vector.num_ctrl, 1);
        control(0) = 0.2;
        init_controls.push_back(control);
    }

    return init_controls;
}

void PistonBlock::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    pose_6 box_pose;
    pose_6 box_vel;

    MuJoCo_helper->GetBodyPoseAngle("goal", box_pose, d);
    MuJoCo_helper->GetBodyVelocity("goal", box_vel, d);

    // --------------- Residual 0: Cube x positions -----------------
    residuals(resid_index++, 0) = box_pose.position(1) - residual_list[0].target[0];

    // --------------- Residual 1: Cube x velocity -----------------
    residuals(resid_index++, 0) = box_vel.position[1] - residual_list[1].target[0];

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

void PistonBlock::SetGoalVisuals(mjData *d){

    pose_6 goal_pose;

    goal_pose.position[0] = 0.0;
    goal_pose.position[1] = residual_list[0].target[0];
    goal_pose.position[2] = 0.0;

    MuJoCo_helper->SetBodyPoseAngle("display_goal", goal_pose, d);
}