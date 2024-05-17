#include "ModelTranslator/TwoDPushing.h"

TwoDPushing::TwoDPushing(int _clutterLevel): PushBaseClass("franka_gripper", "goal"){

    clutterLevel = _clutterLevel;
    std::string yamlFilePath = "/taskConfigs/twoDPushingConfig.yaml";
    if(clutterLevel == noClutter){
        yamlFilePath = "/taskConfigs/twoDPushingConfig.yaml";
    }
    else if(clutterLevel == lowClutter){
        yamlFilePath = "/taskConfigs/twoDPushingLowClutterConfig.yaml";
    }
    else if(clutterLevel == heavyClutter){
        yamlFilePath = "/taskConfigs/twoDPushingHeavyClutterConfig.yaml";
    }
    else if(clutterLevel == constrainedClutter){
        yamlFilePath = "/taskConfigs/twoDPushingConstrainedClutterConfig.yaml";
    }
    else if(clutterLevel == clutter_realWorld){
        yamlFilePath = "/taskConfigs/twoDPushingRealWorldConfig.yaml";
    }
    else{
        cout << "ERROR: Invalid clutter level" << endl;
    }

    cost_reach.setZero();
    cost_reach(0, 0) = 1;
    cost_reach(1, 1) = 1;

    InitModelTranslator(yamlFilePath);
}

void TwoDPushing::ReturnRandomStartState(){

    float startX;
    float startY;
    float goalX;
    float goalY;

    if(clutterLevel == constrainedClutter){
        startX = randFloat(0.45, 0.46);
        startY = randFloat(-0.05, 0.05);

        goalX = randFloat(0.6, 0.65);
        goalY = randFloat(-0.2, 0.2);

    }
    else{
        startX = 0.4;
        startY = randFloat(-0.1, 0.1);

        float randAngle = randFloat(-PI/4, PI/4);
        float randDist = randFloat(0.28, 0.3);

        goalX = startX + randDist * cos(randAngle);
        goalY = startY + randDist * sin(randAngle);
    }

    // Set start position of pushed object
    pose_6 pushedObjectStartPose;
    MuJoCo_helper->GetBodyPoseAngle("goal", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
    pushedObjectStartPose.position(0) = startX;
    pushedObjectStartPose.position(1) = startY;
    pushedObjectStartPose.position(2) = 0.032;
    MuJoCo_helper->SetBodyPoseAngle("goal", pushedObjectStartPose, MuJoCo_helper->main_data);
    MuJoCo_helper->SetBodyPoseAngle("goal", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->master_reset_data);

    randomGoalX = goalX;
    randomGoalY = goalY;

    std::vector<std::string> object_names;

    if(clutterLevel == lowClutter  || clutterLevel == constrainedClutter){
        object_names.emplace_back("obstacle_1");
        object_names.emplace_back("obstacle_2");
        object_names.emplace_back("obstacle_3");

        int validObjectCounter = 0;

        for(const auto & objectName : object_names){
            bool validPlacement = false;
            float sizeX = 0.01;
            float sizeY = 0.05;
            while(!validPlacement){
                sizeX += 0.0005;
                sizeY += 0.0001;

                float randX, randY;

                if(clutterLevel == constrainedClutter){
                    randX = randFloat(startX, goalX + 0.1f);
                    randY = randFloat(goalY - sizeY, goalY + sizeY);
                }
                else{
                    randX = randFloat(goalX - sizeX, goalX);
                    randY = randFloat(goalY - sizeY, goalY + sizeY);
                }

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                MuJoCo_helper->GetBodyPoseAngle(objectName, objectCurrentPose, MuJoCo_helper->main_data);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->main_data);
                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->master_reset_data);

                if(MuJoCo_helper->CheckBodyForCollisions(objectName, MuJoCo_helper->main_data)){
//                    cout << "invalid placement of " << objectName << " at : " << randX << ", " << randY << endl;
                }
                else{
                    validPlacement = true;
                }
            }
            validObjectCounter++;
        }
    }
    else if(clutterLevel == heavyClutter){
        object_names.emplace_back("obstacle_1");
        object_names.emplace_back("obstacle_2");
        object_names.emplace_back("obstacle_3");
        object_names.emplace_back("obstacle_4");
        object_names.emplace_back("obstacle_5");
        object_names.emplace_back("obstacle_6");
        object_names.emplace_back("obstacle_7");


        cout << "goal position: " << goalX << ", " << goalY << endl;

        for(const auto & objectName : object_names){
            bool validPlacement = false;
            float sizeX = 0.08;
            float sizeY = 0.04;
            while(!validPlacement){
                sizeX += 0.001;
                sizeY += 0.0005;

                float randX = randFloat(goalX - sizeX, goalX + (0.5f * sizeX));
                float randY = randFloat(goalY - sizeY, goalY + sizeY);

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                MuJoCo_helper->GetBodyPoseAngle(objectName, objectCurrentPose, MuJoCo_helper->master_reset_data);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->main_data);
                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->master_reset_data);

                if(MuJoCo_helper->CheckBodyForCollisions(objectName, MuJoCo_helper->main_data)){
                    cout << "invalid placement at : " << randX << ", " << randY << endl;
                }
                else{
                    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
                    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->master_reset_data);
                    validPlacement = true;
                }
            }
        }
    }

    // Robot start configuration
    double robot_start_config[7] = {0, -0.183, 0, -3.1, 0, 1.34, 0};

    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
        full_state_vector.robots[0].start_pos[i] = robot_start_config[i];
    }

    full_state_vector.rigid_bodies[0].start_linear_pos[0] = startX;
    full_state_vector.rigid_bodies[0].start_linear_pos[1] = startY;
    full_state_vector.rigid_bodies[0].start_linear_pos[2] = 0.032;

    full_state_vector.rigid_bodies[0].start_angular_pos[0] = 0.0;
    full_state_vector.rigid_bodies[0].start_angular_pos[1] = 0.0;
    full_state_vector.rigid_bodies[0].start_angular_pos[2] = 0.0;

    // Distractor body poses
    for(int i = 0; i < object_names.size(); i++){
        std::cout << "object name: " << object_names[i] << "\n";
        pose_6 obstacle_pose;
        MuJoCo_helper->GetBodyPoseAngle(object_names[i], obstacle_pose, MuJoCo_helper->master_reset_data);

        for(int j = 0; j < 3; j++){
            full_state_vector.rigid_bodies[i + 1].start_linear_pos[j] = obstacle_pose.position[j];
            full_state_vector.rigid_bodies[i + 1].start_angular_pos[j] = obstacle_pose.orientation[j];
        }
    }
    std::cout << "in generation \n";
    std::cout << "body " << full_state_vector.rigid_bodies[2].name << " x: " << full_state_vector.rigid_bodies[2].start_linear_pos[0] << " y: " << full_state_vector.rigid_bodies[2].start_linear_pos[1] << std::endl;
}

void TwoDPushing::ReturnRandomGoalState(){

    // Robot configuration doesnt matter for this task
    for(int i = 0; i < full_state_vector.robots[0].joint_names.size(); i++){
        full_state_vector.robots[0].goal_pos[i] = 0.0;
        full_state_vector.robots[0].goal_vel[i] = 0.0;
    }

    // Goal object body
    std::cout << "goal x" << randomGoalX << "goal y: " << randomGoalY << std::endl;
    full_state_vector.rigid_bodies[0].goal_linear_pos[0] = randomGoalX;
    full_state_vector.rigid_bodies[0].goal_linear_pos[1] = randomGoalY;
    full_state_vector.rigid_bodies[0].goal_linear_pos[2] = 0.0;

    full_state_vector.rigid_bodies[0].goal_angular_pos[0] = 0.0;
    full_state_vector.rigid_bodies[0].goal_angular_pos[1] = 0.0;
    full_state_vector.rigid_bodies[0].goal_angular_pos[2] = 0.0;

    // Distractor objects
    for(int i = 1; i < full_state_vector.rigid_bodies.size(); i++){

        full_state_vector.rigid_bodies[i].goal_linear_pos[0] = full_state_vector.rigid_bodies[i].start_linear_pos[0];
        full_state_vector.rigid_bodies[i].goal_linear_pos[1] = full_state_vector.rigid_bodies[i].start_linear_pos[1];
        full_state_vector.rigid_bodies[i].goal_linear_pos[2] = full_state_vector.rigid_bodies[i].start_linear_pos[2];

        full_state_vector.rigid_bodies[i].goal_angular_pos[0] = full_state_vector.rigid_bodies[i].start_angular_pos[0];
        full_state_vector.rigid_bodies[i].goal_angular_pos[1] = full_state_vector.rigid_bodies[i].start_angular_pos[1];
        full_state_vector.rigid_bodies[i].goal_angular_pos[2] = full_state_vector.rigid_bodies[i].start_angular_pos[2];
    }
}

std::vector<MatrixXd> TwoDPushing::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);

    // Pushing create init controls borken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goalPos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    goalPos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
    goalPos(2) = 0.0;
    EEWayPointsSetup(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << "setup mainwaypoint 0: " << mainWayPoints[0] << endl;
//    cout << "setup mainWayPoint 1: " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initSetupControls = JacobianEEControl(goalPos, allWayPoints);

    return initSetupControls;
}

std::vector<MatrixXd> TwoDPushing::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    MuJoCo_helper->GetBodyPoseAngle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);
    displayBodyPose.position[0] = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    displayBodyPose.position[1] = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->SetBodyPoseAngle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goalPos(0) = current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    goalPos(1) = current_state_vector.rigid_bodies[0].goal_linear_pos[1];
    EEWayPointsPush(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << mainWayPoints.size() << " waypoints created" << endl;
//    cout << "mainwaypoint 0: " << mainWayPoints[1] << endl;
//    cout << "mainWayPoint 1: " << mainWayPoints[2] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initControls = JacobianEEControl(goalPos, allWayPoints);

    return initControls;
}

// New - testing it out
//double TwoDPushing::CostFunction(mjData *d, bool terminal){
//    double cost;
//    MatrixXd Xt = ReturnStateVector(d);
//    MatrixXd Ut = ReturnControlVector(d);
//
//    // General cost function for the difference between desired and actual state
//    MatrixXd X_diff = Xt - X_desired;
//    MatrixXd temp;
//
//    if(terminal){
//        temp = ((X_diff.transpose() * Q_terminal * X_diff)) + (Ut.transpose() * R * Ut);
//    }
//    else{
//        temp = ((X_diff.transpose() * Q * X_diff)) + (Ut.transpose() * R * Ut);
//    }
//
//    cost = temp(0);
//
//    // Reach cost function - difference between EE and goal object.
////    pose_7 EE_pose;
////    MuJoCo_helper->getBodyPose_quat_ViaXpos("franka_gripper", EE_pose, d);
////
////    cost += pow(EE_pose.position(0) - X_desired(7), 2) * 1;
////    cost += pow(EE_pose.position(1) - X_desired(8), 2) * 1;
//
//    return cost;
//}

//void TwoDPushing::CostDerivatives(mjData *d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
//    MatrixXd Xt = ReturnStateVector(d);
//    MatrixXd Ut = ReturnControlVector(d);
//
//    MatrixXd X_diff = Xt - X_desired;
//
//    // Size cost derivatives appropriately
//    l_x.resize(state_vector_size, 1);
//    l_xx.resize(state_vector_size, state_vector_size);
//
//    l_u.resize(num_ctrl, 1);
//    l_uu.resize(num_ctrl, num_ctrl);
//
//    if(terminal){
//        l_x = 2 * Q_terminal * X_diff;
//        l_xx = 2 * Q_terminal;
//    }
//    else{
//        l_x = 2 * Q * X_diff;
//        l_xx = 2 * Q;
//    }
//
//    l_u = 2 * R * Ut;
//    l_uu = 2 * R;
//
////    // Reach gradients
////    MatrixXd Jac;
////    MatrixXd joints = Xt.block(0, 0, 7, 1);
////
////    // Not ideal this being here, computationally expensive
////    MuJoCo_helper->forwardSimulator(data_index);
////
////    Jac = MuJoCo_helper->calculateJacobian("franka_gripper", data_index);
////
////    std::cout << "Jac " << Jac << std::endl;
////    std::cout << "joints transpose " << joints << std::endl;
////
////    MatrixXd joints_2_EE(6, 1);
////    MatrixXd EE_x(7, 1);
////    MatrixXd EE_xx(7, 7);
////    joints_2_EE = Jac * joints;
////    std::cout << "jointes_2_ee " << EE_x << std::endl;
////
////    EE_x = 2 * joints_2_EE * cost_reach;
////    std::cout << "EE_x " << EE_x << std::endl;
////    EE_xx = 2 * cost_reach;
////
////    // Add the reach gradients to the cost derivatives
////    l_x.block(7, 0, 7, 1) += EE_x;
////    l_xx.block(7, 7, 7, 7) += EE_xx;
//}

bool TwoDPushing::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);

    double x_diff = goal_pose.position(0) - current_state_vector.rigid_bodies[0].goal_linear_pos[0];
    double y_diff = goal_pose.position(1) - current_state_vector.rigid_bodies[0].goal_linear_pos[1];

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.025){
        taskComplete = true;
    }


    return taskComplete;
}