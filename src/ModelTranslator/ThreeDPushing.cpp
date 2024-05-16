#include "ModelTranslator/ThreeDPushing.h"

ThreeDPushing::ThreeDPushing() : PushBaseClass("franka_gripper", "goal"){

    std::string yamlFilePath = "/taskConfigs/pushThreeDConfig.yaml";

    InitModelTranslator(yamlFilePath);
}

//MatrixXd ThreeDPushing::ReturnRandomStartState(){
//    MatrixXd randomStartState(state_vector_size, 1);
//
//    float startX;
//    float startY;
//    float goalX;
//    float goalY;
//
//    startX = 0.4;
//    startY = randFloat(-0.1, 0.1);
//
//    float randAngle = randFloat(-PI/4, PI/4);
//    float randDist = randFloat(0.28, 0.3);
//
//    goalX = startX + randDist * cos(randAngle);
//    goalY = startY + randDist * sin(randAngle);
//
//    // Set start position of pushed object
//    pose_6 pushedObjectStartPose;
//    MuJoCo_helper->GetBodyPoseAngle("goal", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
//    pushedObjectStartPose.position(0) = startX;
//    pushedObjectStartPose.position(1) = startY;
//    pushedObjectStartPose.position(2) = 0.032;
//    MuJoCo_helper->SetBodyPoseAngle("goal", pushedObjectStartPose, MuJoCo_helper->main_data);
//    MuJoCo_helper->SetBodyPoseAngle("goal", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
//    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
//    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->master_reset_data);
//
//
//    randomGoalX = goalX;
//    randomGoalY = goalY;
//
//    std::vector<double> objectXPos;
//    std::vector<double> objectYPos;
//
//    // TODO - this has correct number of elements but all zeros for rotaiton
//    // might not be desired.
//    randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//            startX, startY, 0, 0, 0, 0,
//            0, 0, 0, 0, 0, 0, 0,
//            0, 0, 0, 0, 0, 0;
//
//
//    return randomStartState;
//}
//
//MatrixXd ThreeDPushing::ReturnRandomGoalState(MatrixXd X0){
//    MatrixXd randomGoalState(state_vector_size, 1);
//
//    // TODO - Right number of elements, but all zeros rotation might not be desired.
//    randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                randomGoalX, randomGoalY, 0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0;
//
//
//
//    return randomGoalState;
//}

std::vector<MatrixXd> ThreeDPushing::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);

    // Pushing create init controls borken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = current_state_vector.bodies[0].goal_linear_pos[0];
    goal_pos(1) = current_state_vector.bodies[0].goal_linear_pos[1];
    EEWayPointsSetup(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << "setup mainwaypoint 0: " << mainWayPoints[0] << endl;
//    cout << "setup mainWayPoint 1: " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initSetupControls = JacobianEEControl(goal_pos, allWayPoints);

    return initSetupControls;
}

std::vector<MatrixXd> ThreeDPushing::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    displayBodyPose.position[0] = current_state_vector.bodies[0].goal_linear_pos[0];
    displayBodyPose.position[1] = current_state_vector.bodies[0].goal_linear_pos[1];
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->SetBodyPoseAngle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goal_pos(0) = current_state_vector.bodies[0].goal_linear_pos[0];
    goal_pos(1) = current_state_vector.bodies[0].goal_linear_pos[1];
    EEWayPointsPush(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << mainWayPoints.size() << " waypoints created" << endl;
//    cout << "mainwaypoint 0: " << mainWayPoints[1] << endl;
//    cout << "mainWayPoint 1: " << mainWayPoints[2] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initControls = JacobianEEControl(goal_pos, allWayPoints);

    return initControls;
}

bool ThreeDPushing::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);

    double x_diff = goal_pose.position(0) - current_state_vector.bodies[0].goal_linear_pos[0];
    double y_diff = goal_pose.position(1) - current_state_vector.bodies[0].goal_linear_pos[1];

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.025){
        taskComplete = true;
    }

    return taskComplete;
}