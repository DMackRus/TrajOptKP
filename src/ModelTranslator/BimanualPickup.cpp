#include "ModelTranslator/BimanualPickup.h"

BimanualPickup::BimanualPickup() : ModelTranslator() {
    InitModelTranslator("/TaskConfigs/rigid_body_manipulation/bimanual_pickup.yaml");
}

void BimanualPickup::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    // Compute kinematics chain to compute site poses
    mj_kinematics(MuJoCo_helper->model, d);

    pose_6 goal_pose;
    pose_6 goal_vel;
    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);
    MuJoCo_helper->GetBodyVelocity("goal", goal_vel, d);

    double diff_x, diff_y, diff_z, dist;

//    std::cout << "body goal pose: " << goal_pose.position.transpose() << "\n";

//    // --------------- Residual 0: Body goal position -----------------
    diff_x = goal_pose.position(0) - residual_list[0].target[0];
    diff_y = goal_pose.position(1) - residual_list[0].target[1];
    diff_z = goal_pose.position(2) - residual_list[0].target[2];
    dist = sqrt(pow(diff_x, 2)
                       + pow(diff_y, 2)
                       + pow(diff_z, 2));
    residuals(resid_index++, 0) = dist; // - residual_list[1].target[0];

//    diff_z = goal_pose.position(2) - residual_list[0].target[0];
//    dist = sqrt(pow(diff_x, 2)
//                + pow(diff_y, 2)
//                + pow(diff_z, 2));
//    residuals(resid_index++, 0) = diff_z; // - residual_list[1].target[0];

    // --------------- Residual 1: Gripper 1 to box -----------------
    pose_7 EE_pose;
    MuJoCo_helper->GetBodyPoseQuatViaXpos("panda0_gripper", EE_pose, d);
    diff_x = EE_pose.position(0) - goal_pose.position(0);
    diff_y = EE_pose.position(1) - goal_pose.position(1);
    diff_z = EE_pose.position(2) - goal_pose.position(2);
    dist = sqrt(pow(diff_x, 2)
                       + pow(diff_y, 2)
                       + pow(diff_z, 2));
    residuals(resid_index++, 0) = dist - residual_list[1].target[0];

    // --------------- Residual 2: Gripper 2 to box -----------------
    MuJoCo_helper->GetBodyPoseQuatViaXpos("panda1_gripper", EE_pose, d);
    diff_x = EE_pose.position(0) - goal_pose.position(0);
    diff_y = EE_pose.position(1) - goal_pose.position(1);
    diff_z = EE_pose.position(2) - goal_pose.position(2);
    dist = sqrt(pow(diff_x, 2)
                       + pow(diff_y, 2)
                       + pow(diff_z, 2));
    residuals(resid_index++, 0) = dist - residual_list[2].target[0];

    // --------------- Residual 1: Body goal velocity -----------------
//    diff_x = goal_vel.position(0) - residual_list[1].target[0];
//    diff_y = goal_vel.position(1) - residual_list[1].target[1];
//    residuals(resid_index++, 0) = sqrt(pow(diff_x, 2)
//                                       + pow(diff_y, 2));
//    residuals(resid_index++, 0) = 0.0;

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

void BimanualPickup::SetGoalVisuals(mjData *d){
    pose_6 box_goal;
    MuJoCo_helper->GetBodyPoseAngle("display_goal", box_goal, d);

//    box_goal.position(0) = residual_list[0].target[0];
//    box_goal.position(1) = residual_list[0].target[1];
//    box_goal.position(2) = residual_list[0].target[2];

    box_goal.position(0) = residual_list[0].target[0];
    box_goal.position(1) = residual_list[0].target[1];
    box_goal.position(2) = residual_list[0].target[2];

    MuJoCo_helper->SetBodyPoseAngle("display_goal", box_goal, d);
}

void BimanualPickup::ReturnRandomStartState(){
    std::cerr << "BimanualPickup::ReturnRandomStartState() not implemented yet.\n";
    exit(1);
}

void BimanualPickup::ReturnRandomGoalState(){

}

bool BimanualPickup::TaskComplete(mjData *d, double &dist){
    return false;
}