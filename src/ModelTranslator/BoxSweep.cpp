#include "ModelTranslator/BoxSweep.h"

BoxSweep::BoxSweep() : PushBaseClass("franka_gripper", "goal"){

    std::string yamlFilePath = "/TaskConfigs/rigid_body_manipulation/box_sweep.yaml";

    InitModelTranslator(yamlFilePath);
}

void BoxSweep::ReturnRandomStartState(){
    double robot_config[7] = {-0.178, 0.7, -0.0593, -1.73, 0, 0.722, -1.6};

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

    // Set Z position for big box
    full_state_vector.rigid_bodies[0].start_linear_pos[2] = 0.16;

}

void BoxSweep::ReturnRandomGoalState(){
    float upperBoundX = 0.7;
    float lowerBoundX = 0.6;
    float upperBoundY = 0.5;
    float lowerBoundY = 0.3;

    float randX = randFloat(lowerBoundX, upperBoundX);
    float randY = randFloat(lowerBoundY, upperBoundY);

    // Box goal position
    residual_list[0].target[0] = randX;
    residual_list[0].target[1] = randY;

    // Box goal velocity
    residual_list[1].target[0] = 0.0;
    residual_list[1].target[1] = 0.0;

    // EE position towards goal object
    residual_list[2].target[0] = 0.05;
}

void BoxSweep::SetGoalVisuals(mjData *d){
    pose_6 box_goal;
    box_goal.position(0) = residual_list[0].target[0];
    box_goal.position(1) = residual_list[0].target[1];

    box_goal.position(2) = 0.0;

    MuJoCo_helper->SetBodyPoseAngle("display_goal", box_goal, d);
}

std::vector<MatrixXd> BoxSweep::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);

    return initSetupControls;
}

std::vector<MatrixXd> BoxSweep::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> init_controls;

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = residual_list[0].target[0];
    goal_pos(1) = residual_list[0].target[1];
    EEWayPointsPush(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Compute angle of push based on goal - start
    pose_7 goal_obj_start;
    MuJoCo_helper->GetBodyPoseQuat(body_name, goal_obj_start, MuJoCo_helper->master_reset_data);
    double diff_x = goal_pos(0) - goal_obj_start.position[0];
    double diff_y =  goal_pos(1) - goal_obj_start.position[1];
    double angle_EE_push = atan2(diff_y, diff_x);

    // Step 3 - follow the points via the jacobian
    init_controls = JacobianEEControl(allWayPoints, angle_EE_push);

    return init_controls;
}

void BoxSweep::Residuals(mjData *d, MatrixXd &residuals){
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
    diff_x = goal_vel.position(0) - residual_list[1].target[0];
    diff_y = goal_vel.position(1) - residual_list[1].target[1];
    residuals(resid_index++, 0) = sqrt(pow(diff_x, 2)
                                       + pow(diff_y, 2));

    // --------------- Residual 2: EE position towards goal object -----------------
    pose_7 EE_pose;
    MuJoCo_helper->GetBodyPoseQuatViaXpos("franka_gripper", EE_pose, d);
    diff_x = EE_pose.position(0) - goal_pose.position(0);
    diff_y = EE_pose.position(1) - goal_pose.position(1);
    double diff_z = EE_pose.position(2) - goal_pose.position(2);
    double dist = sqrt(pow(diff_x, 2)
             + pow(diff_y, 2)
             + pow(diff_z, 2));
    residuals(resid_index++, 0) = dist - residual_list[2].target[0];

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

bool BoxSweep::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);

    double x_diff = goal_pose.position(0) - residual_list[0].target[0];
    double y_diff = goal_pose.position(1) - residual_list[0].target[1];

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.035){
        taskComplete = true;
    }

    return taskComplete;
}