#include "ModelTranslator/Piston.h"

Piston::Piston(){
    std::string task_config = "/TaskConfigs/toys/piston.yaml";
    InitModelTranslator(task_config);
}

void Piston::ReturnRandomStartState(){
    current_state_vector.robots[0].start_pos[0] = 0.0;
}

void Piston::ReturnRandomGoalState(){

    // Box position
    residual_list[0].target[0] = randFloat(1.0, 2.0);

    // Box velocity
    residual_list[1].target[0] = 0.0;
}

//std::vector<MatrixXd> Piston::CreateInitOptimisationControls(int horizonLength){
//    std::vector<MatrixXd> init_controls;
//
//    // Create controls where we move forward to contact the box
//    for(int t = 0; t < horizonLength; t++){
//        MatrixXd control(current_state_vector.num_ctrl, 1);
//        control(0) = 0.1;
//        init_controls.push_back(control);
//    }
//
//    return init_controls;
//}

void Piston::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

//    pose_6 box_pose;
//    pose_6 box_vel;
//
//    MuJoCo_helper->GetBodyPoseAngle("goal", box_pose, d);
//    MuJoCo_helper->GetBodyVelocity("goal", box_vel, d);

    std::vector<double> piston_joints, piston_velocities;
    MuJoCo_helper->GetRobotJointsPositions("piston", piston_joints, d);
    MuJoCo_helper->GetRobotJointsVelocities("piston", piston_velocities, d);

    // --------------- Residual 0: Cube x positions -----------------
    residuals(resid_index++, 0) = piston_joints[0] - residual_list[0].target[0];

    // --------------- Residual 1: Cube x velocity -----------------
    residuals(resid_index++, 0) = piston_velocities[0] - residual_list[1].target[0];

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

void Piston::SetGoalVisuals(mjData *d){

    pose_6 goal_pose;

    MuJoCo_helper->GetBodyPoseAngle("display_goal", goal_pose, d);

    goal_pose.position[0] = 0.0;
    goal_pose.position[1] = residual_list[0].target[0];
    goal_pose.position[2] = 0.0;

    MuJoCo_helper->SetBodyPoseAngle("display_goal", goal_pose, d);
}