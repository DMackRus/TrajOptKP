#include "ModelTranslator/BoxSweep.h"

BoxSweep::BoxSweep() : PushBaseClass("franka_gripper", "goal"){

    std::string yamlFilePath = "/TaskConfigs/rigid_body_manipulation/box_sweep.yaml";

    InitModelTranslator(yamlFilePath);
}

void BoxSweep::ReturnRandomStartState(){
    double robot_config[7] = {-0.178, 0.7, -0.0593, -1.73, 0, 0.722, -1.6};

    // Franka Panda starting cofniguration
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

    // Franka Panda goal configuration is unimportant
    for(int i = 0; i < 7; i++){
        full_state_vector.robots[0].goal_pos[i] = 0.0;
        full_state_vector.robots[0].goal_vel[i] = 0.0;
    }

    // Large box configuration
    for(int i = 0; i < 3; i++){
        full_state_vector.rigid_bodies[0].goal_linear_pos[i] = 0.0;
        full_state_vector.rigid_bodies[0].goal_angular_pos[i] = 0.0;
    }

    // Set goal location for big box
    full_state_vector.rigid_bodies[0].goal_linear_pos[0] = randX;
    full_state_vector.rigid_bodies[0].goal_linear_pos[1] = randY;

}

std::vector<MatrixXd> BoxSweep::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);

    return initSetupControls;
}

std::vector<MatrixXd> BoxSweep::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> init_controls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_7 display_goal_pose;
    display_goal_pose.position[0] = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    display_goal_pose.position[1] = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
    display_goal_pose.position[2] = 0.0f;

    m_point desired_eul = {current_state_vector.rigid_bodies[0].goal_angular_pos[0],
                           current_state_vector.rigid_bodies[0].goal_angular_pos[1],
                           current_state_vector.rigid_bodies[0].goal_angular_pos[2]};

    display_goal_pose.quat = eul2Quat(desired_eul);

    MuJoCo_helper->SetBodyPoseQuat(goalMarkerName, display_goal_pose, MuJoCo_helper->master_reset_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    goal_pos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
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

bool BoxSweep::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);

    double x_diff = goal_pose.position(0) - current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    double y_diff = goal_pose.position(1) - current_state_vector.rigid_bodies[0].goal_linear_pos[1];

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.035){
        taskComplete = true;
    }


    return taskComplete;
}
