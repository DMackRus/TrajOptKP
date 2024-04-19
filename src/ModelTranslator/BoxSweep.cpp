#include "BoxSweep.h"

BoxSweep::BoxSweep() : PushBaseClass("franka_gripper", "goal"){

    std::string yamlFilePath = "/taskConfigs/boxSweep.yaml";

    InitModelTranslator(yamlFilePath);
}

void BoxSweep::ReturnRandomStartState(){
    MatrixXd randomStartState(state_vector_size, 1);

    double robot_config[7] = {-0.178, 0.7, -0.0593, -1.73, 0, 0.722, -1.6};

    // Franka Panda starting cofniguration
    for(int i = 0; i < 7; i++){
        active_state_vector.robots[0].startPos[i] = robot_config[i];
    }

    // Large box configuration
    for(int i = 0; i < 3; i++){
        active_state_vector.bodiesStates[0].startLinearPos[i] = 0.0;
        active_state_vector.bodiesStates[0].startAngularPos[i] = 0.0;
    }

    // Set X position for big box
    active_state_vector.bodiesStates[0].startLinearPos[0] = 0.65;

    // Set Z position for big box
    active_state_vector.bodiesStates[0].startLinearPos[2] = 0.16;

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
        active_state_vector.robots[0].goalPos[i] = 0.0;
        active_state_vector.robots[0].goalVel[i] = 0.0;
    }

    // Large box configuration
    for(int i = 0; i < 3; i++){
        active_state_vector.bodiesStates[0].goalLinearPos[i] = 0.0;
        active_state_vector.bodiesStates[0].goalAngularPos[i] = 0.0;
    }

    // Set goal location for big box
    active_state_vector.bodiesStates[0].goalLinearPos[0] = randX;
    active_state_vector.bodiesStates[0].goalLinearPos[1] = randY;

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
    display_goal_pose.position[0] = active_state_vector.bodiesStates[0].goalLinearPos[0];
    display_goal_pose.position[1] = active_state_vector.bodiesStates[0].goalLinearPos[1];
    display_goal_pose.position[2] = 0.0f;

    m_point desired_eul = {active_state_vector.bodiesStates[0].goalAngularPos[0],
                           active_state_vector.bodiesStates[0].goalAngularPos[1],
                           active_state_vector.bodiesStates[0].goalAngularPos[2]};

    std::cout << "deisred eul " << desired_eul << "\n";

    display_goal_pose.quat = eul2Quat(desired_eul);

    MuJoCo_helper->SetBodyPoseQuat(goalMarkerName, display_goal_pose, MuJoCo_helper->master_reset_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = active_state_vector.bodiesStates[0].goalLinearPos[0];
    goal_pos(1) = active_state_vector.bodiesStates[0].goalLinearPos[1];
    EEWayPointsPush(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    init_controls = JacobianEEControl(goal_pos, allWayPoints);

    return init_controls;
}

bool BoxSweep::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    MatrixXd currentState = ReturnStateVector(d);

    double x_diff = currentState(7) - active_state_vector.bodiesStates[0].goalLinearPos[0];
    double y_diff = currentState(8) - active_state_vector.bodiesStates[0].goalLinearPos[1];

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.035){
        taskComplete = true;
    }


    return taskComplete;
}
