#include "BoxSweep.h"

BoxSweep::BoxSweep() : PushBaseClass("franka_gripper", "bigBox"){

    std::string yamlFilePath = "/taskConfigs/boxSweep.yaml";

    InitModelTranslator(yamlFilePath);
}

void BoxSweep::GenerateRandomGoalAndStartState() {
    X_start = ReturnRandomStartState();
    X_desired = ReturnRandomGoalState(X_start);
}

MatrixXd BoxSweep::ReturnRandomStartState(){
    MatrixXd randomStartState(state_vector_size, 1);

    randomStartState << -0.178, 0.7, -0.0593, -1.73, 0, 0.722, -1.6,
                        0.65, 0,
                        0, 0, 0, 0, 0, 0, 0,
                        0, 0;

    return randomStartState;
}

MatrixXd BoxSweep::ReturnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(state_vector_size, 1);

    float upperBoundX = 0.7;
    float lowerBoundX = 0.6;
    float upperBoundY = 0.5;
    float lowerBoundY = 0.3;

    float randX = randFloat(lowerBoundX, upperBoundX);
    float randY = randFloat(lowerBoundY, upperBoundY);

    randomGoalState << 0, 0, 0, 0, 0, 0, 0,
                        randX, randY,
                        0, 0, 0, 0, 0, 0, 0,
                        0, 0;

    return randomGoalState;
}

std::vector<MatrixXd> BoxSweep::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->copySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->forwardSimulator(MuJoCo_helper->main_data);

    return initSetupControls;
}

std::vector<MatrixXd> BoxSweep::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> init_controls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 display_goal_pose;
    display_goal_pose.position[0] = X_desired(7);
    display_goal_pose.position[1] = X_desired(8);
    display_goal_pose.position[2] = 0.0f;
    MuJoCo_helper->setBodyPose_angle(goalMarkerName, display_goal_pose, MuJoCo_helper->master_reset_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = X_desired(7);
    goal_pos(1) = X_desired(8);
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

    double x_diff = currentState(7) - X_desired(7);
    double y_diff = currentState(8) - X_desired(8);

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.035){
        taskComplete = true;
    }


    return taskComplete;
}
