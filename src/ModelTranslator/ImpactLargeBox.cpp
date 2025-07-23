#include "ModelTranslator/ImpactLargeBox.h"

ImpactLargeBox::ImpactLargeBox(){
    std::string yamlFilePath = "/TaskConfigs/impact_manipulation/impact_large_box.yaml";
    InitModelTranslator(yamlFilePath);
}

void ImpactLargeBox::ReturnRandomStartState(){
    double robot_config[7] = {-1, 0.7, -0.0593, -1.73, 0, 0.722, -1.6};

    // Franka Panda starting configuration
    for(int i = 0; i < 7; i++){
        full_state_vector.robots[0].start_pos[i] = robot_config[i];
    }

    // Large box configuration
    for(int i = 0; i < 3; i++){
        full_state_vector.rigid_bodies[0].start_linear_pos[i] = 0.0;
        full_state_vector.rigid_bodies[0].start_angular_pos[i] = 0.0;
    }

    // Set X position for big box
    full_state_vector.rigid_bodies[0].start_linear_pos[0] = 0.65;

    // Set Y position for big box
    full_state_vector.rigid_bodies[0].start_linear_pos[1] = 0.3;

    // Set Z position for big box
    full_state_vector.rigid_bodies[0].start_linear_pos[2] = 0.08;

}

void ImpactLargeBox::ReturnRandomGoalState(){
    float upperBoundX = 0.7;
    float lowerBoundX = 0.6;
    float upperBoundY = 0.9;
    float lowerBoundY = 0.8;

    float randX = randFloat(lowerBoundX, upperBoundX);
    float randY = randFloat(lowerBoundY, upperBoundY);

    // Box goal position
    residual_list[0].target[0] = randX;
    residual_list[0].target[1] = randY;

    // EE position towards goal object
    residual_list[1].target[0] = 0;
}

void ImpactLargeBox::SetGoalVisuals(mjData *d){
    pose_6 box_goal;
    MuJoCo_helper->GetBodyPoseAngle("display_goal", box_goal, d);

    box_goal.position(0) = residual_list[0].target[0];
    box_goal.position(1) = residual_list[0].target[1];
    box_goal.position(2) = 0.08;

    MuJoCo_helper->SetBodyPoseAngle("display_goal", box_goal, d);
}

std::vector<MatrixXd> ImpactLargeBox::CreateInitOptimisationControls(int horizonLength) {
    std::vector<MatrixXd> init_opt_controls;
    int num_ctrl = current_state_vector.num_ctrl;
    vector<double> gravCompensation;
    MatrixXd control(num_ctrl, 1);

    for(int i = 0; i < horizonLength; i++){

        MuJoCo_helper->GetRobotJointsGravityCompensationControls(current_state_vector.robots[0].name, gravCompensation, MuJoCo_helper->main_data);

        for(int j = 0; j < num_ctrl; j++){
            control(j) = gravCompensation[j];
        }

        SetControlVector(control, MuJoCo_helper->main_data, full_state_vector);
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);
        init_opt_controls.push_back(control);
    }

    return init_opt_controls;
}

void ImpactLargeBox::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    // Compute kinematics chain to compute site poses
    mj_kinematics(MuJoCo_helper->model, d);

    pose_6 goal_pose;
    pose_6 goal_vel;
    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);
    MuJoCo_helper->GetBodyVelocity("goal", goal_vel, d);

    // --------------- Residual 0: Body goal position -----------------
    double diff_x = goal_pose.position(0) - residual_list[0].target[0];
    double diff_y = goal_pose.position(1) - residual_list[0].target[1];
    residuals(resid_index++, 0) = sqrt(pow(diff_x, 2)
                                       + pow(diff_y, 2));

    // --------------- Residual 1: Body goal velocity -----------------
//    diff_x = goal_vel.position(0) - residual_list[1].target[0];
//    diff_y = goal_vel.position(1) - residual_list[1].target[1];
//    residuals(resid_index++, 0) = sqrt(pow(diff_x, 2)
//                                       + pow(diff_y, 2));

    // --------------- Residual 2: EE position towards goal object -----------------
    pose_7 EE_pose;
    MuJoCo_helper->GetBodyPoseQuatViaXpos("franka_gripper", EE_pose, d);
    diff_x = EE_pose.position(0) - goal_pose.position(0);
    diff_y = EE_pose.position(1) - goal_pose.position(1);
    double diff_z = EE_pose.position(2) - goal_pose.position(2);
    double dist = sqrt(pow(diff_x, 2)
                       + pow(diff_y, 2)
                       + pow(diff_z, 2));
    residuals(resid_index++, 0) = dist - residual_list[1].target[0];

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}