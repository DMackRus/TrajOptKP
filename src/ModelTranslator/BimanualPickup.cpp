#include "ModelTranslator/BimanualPickup.h"

BimanualPickup::BimanualPickup() : ModelTranslator() {
    InitModelTranslator("/TaskConfigs/rigid_body_manipulation/bimanual_pickup.yaml");
}

void BimanualPickup::Residuals(mjData *d, MatrixXd &residuals){
    int resid_index = 0;

    // Compute kinematics chain to compute site poses
    mj_kinematics(MuJoCo_helper->model, d);

//    mj_sensorPos(MuJoCo_helper->model, d);
//    int panda_0_ee_site_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SENSOR, "panda_0_EE");
//    int panda_1_ee_site_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SENSOR, "panda_1_EE");
//    const mjtNum* EE_pos_1 = d->sensordata + MuJoCo_helper->model->sensor_adr[panda_0_ee_site_id];
//    const mjtNum* EE_pos_2 = d->sensordata + MuJoCo_helper->model->sensor_adr[panda_1_ee_site_id];

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
    residuals(resid_index++, 0) = dist;

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

    // --------------- Residual 1: Gripper 1 to box -----------------
//    pose_7 EE_pose;
//    MuJoCo_helper->GetBodyPoseQuatViaXpos("panda0_gripper", EE_pose, d);
//    std::cout << "EE_pos_1: " << EE_pos_1[0] << ", " << EE_pos_1[1] << ", " << EE_pos_1[2] << "\n";
//    diff_x = EE_pos_1[0] - goal_pose.position(0);
//    diff_y = EE_pos_1[1] - goal_pose.position(1);
//    diff_z = EE_pos_1[2] - goal_pose.position(2);
//    dist = sqrt(pow(diff_x, 2)
//                       + pow(diff_y, 2)
//                       + pow(diff_z, 2));
//    residuals(resid_index++, 0) = dist - residual_list[1].target[0];
//
//    // --------------- Residual 2: Gripper 2 to box -----------------
////    MuJoCo_helper->GetBodyPoseQuatViaXpos("panda1_gripper", EE_pose, d);
//    diff_x = EE_pos_2[0] - goal_pose.position(0);
//    diff_y = EE_pos_2[1] - goal_pose.position(1);
//    diff_z = EE_pos_2[2] - goal_pose.position(2);
//    dist = sqrt(pow(diff_x, 2)
//                       + pow(diff_y, 2)
//                       + pow(diff_z, 2));
//    residuals(resid_index++, 0) = dist - residual_list[2].target[0];

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

    double robot_1_config[7] = {0.2, 0.53, 0, -1.5, 0, 0, 1.55};
    double robot_2_config[7] = {-0.2, 0.53, 0, -1.5, 0, 0, -1.55};

    for(int i = 0; i < 7; i++){
        full_state_vector.robots[0].start_pos[i] = robot_1_config[i];
        full_state_vector.robots[1].start_pos[i] = robot_2_config[i];
    }

    // Static position for the box
    pose_7 box_pose;
    MuJoCo_helper->GetBodyPoseQuat("goal", box_pose, MuJoCo_helper->master_reset_data);

    box_pose.position(0) = 0.4;
    box_pose.position(1) = 0.0;
    box_pose.position(2) = 0.05;
}

void BimanualPickup::ReturnRandomGoalState(){

    // Residual 0 - Box goal position
    residual_list[0].target[0] = randFloat(0.3, 0.5);
    residual_list[0].target[1] = 0.0;
    residual_list[0].target[2] = randFloat(0.4, 0.8);

    // Residual 1 - Robot 1 EE to Goal
    residual_list[1].target[0] = 0.015;

    // Residual 2 - Robot 2 EE to Goal
    residual_list[2].target[0] = 0.015;

}

bool BimanualPickup::TaskComplete(mjData *d, double &dist){
    return false;
}