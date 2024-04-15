#include "TwoDPushing.h"

TwoDPushing::TwoDPushing(int _clutterLevel): PushBaseClass("franka_gripper", "blueTin"){

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

//MatrixXd TwoDPushing::ReturnRandomStartState(){
//    MatrixXd randomStartState(state_vector_size, 1);
//
//    float startX;
//    float startY;
//    float goalX;
//    float goalY;
//
//    if(clutterLevel == constrainedClutter){
//        startX = randFloat(0.45, 0.46);
//        startY = randFloat(-0.05, 0.05);
//
//        goalX = randFloat(0.6, 0.65);
//        goalY = randFloat(-0.2, 0.2);
//
//    }
//    else{
//
//        startX = 0.4;
//        startY = randFloat(-0.1, 0.1);
//
//        float randAngle = randFloat(-PI/4, PI/4);
//        float randDist = randFloat(0.28, 0.3);
//
//        goalX = startX + randDist * cos(randAngle);
//        goalY = startY + randDist * sin(randAngle);
//    }
//
//    // Set start position of pushed object
//    pose_6 pushedObjectStartPose;
//    MuJoCo_helper->GetBodyPoseAngle("blueTin", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
//    pushedObjectStartPose.position(0) = startX;
//    pushedObjectStartPose.position(1) = startY;
//    pushedObjectStartPose.position(2) = 0.032;
//    MuJoCo_helper->SetBodyPoseAngle("blueTin", pushedObjectStartPose, MuJoCo_helper->main_data);
//    MuJoCo_helper->SetBodyPoseAngle("blueTin", pushedObjectStartPose, MuJoCo_helper->master_reset_data);
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
//    if(clutterLevel == noClutter){
//        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                startX, startY,
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0;
//    }
//    else if(clutterLevel == lowClutter  || clutterLevel == constrainedClutter){
//
//        std::string objectNames[3] = {"bigBox", "smallBox","tallCylinder"};
//        int validObjectCounter = 0;
//
//        for(const auto & objectName : objectNames){
//            bool validPlacement = false;
//            float sizeX = 0.01;
//            float sizeY = 0.05;
//            while(!validPlacement){
//                sizeX += 0.0005;
//                sizeY += 0.0001;
//
//                float randX, randY;
//
//                if(clutterLevel == constrainedClutter){
//                    randX = randFloat(startX, goalX + 0.1f);
//                    randY = randFloat(goalY - sizeY, goalY + sizeY);
//                }
//                else{
//                    randX = randFloat(goalX - sizeX, goalX);
//                    randY = randFloat(goalY - sizeY, goalY + sizeY);
//                }
//
//                pose_6 objectCurrentPose;
//                pose_6 newObjectPose;
//
//                MuJoCo_helper->GetBodyPoseAngle(objectName, objectCurrentPose, MuJoCo_helper->main_data);
//                newObjectPose = objectCurrentPose;
//                newObjectPose.position(0) = randX;
//                newObjectPose.position(1) = randY;
//                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->main_data);
//                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->master_reset_data);
//
//                if(MuJoCo_helper->CheckBodyForCollisions(objectName, MuJoCo_helper->main_data)){
//                    cout << "invalid placement at : " << randX << ", " << randY << endl;
//                }
//                else{
//                    validPlacement = true;
//                    objectXPos.push_back(randX);
//                    objectYPos.push_back(randY);
//                }
//            }
//            validObjectCounter++;
//        }
//
//        //1, -0.07, 0, -3, 0.232, 1.34, 3, 0.232,
//        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                startX, startY, objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1], objectXPos[2], objectYPos[2],
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0, 0, 0;
//
//    }
//    else if(clutterLevel == heavyClutter){
//
//        bool validPlacement = false;
//        float sizeX = 0.08;
//        float sizeY = 0.04;
//        cout << "goal position: " << goalX << ", " << goalY << endl;
//        while(!validPlacement){
//            sizeX += 0.001;
//            sizeY += 0.0005;
//
//            float randX = randFloat(goalX + 0.1f, goalX +  + 0.1f + sizeX);
//            float randY = randFloat(goalY - sizeY, goalY + sizeY);
//
//            pose_6 objectCurrentPose;
//            pose_6 newObjectPose;
//
//            MuJoCo_helper->GetBodyPoseAngle("obstacle5", objectCurrentPose, MuJoCo_helper->master_reset_data);
//            newObjectPose = objectCurrentPose;
//            newObjectPose.position(0) = randX;
//            newObjectPose.position(1) = randY;
//            newObjectPose.position(2) = objectCurrentPose.position(2);
//            MuJoCo_helper->SetBodyPoseAngle("obstacle5", newObjectPose, MuJoCo_helper->main_data);
//            MuJoCo_helper->SetBodyPoseAngle("obstacle5", newObjectPose, MuJoCo_helper->master_reset_data);
//
//            if(MuJoCo_helper->CheckBodyForCollisions("obstacle5", MuJoCo_helper->main_data)){
//                cout << "first object invalid placement : " << randX << ", " << randY << endl;
//            }
//            else{
//                MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
//                MuJoCo_helper->ForwardSimulator(MuJoCo_helper->master_reset_data);
//                validPlacement = true;
//                objectXPos.push_back(randX);
//                objectYPos.push_back(randY);
//            }
//        }
//
//        std::string objectNames[6] = {"mediumCylinder", "bigBox", "obstacle1","obstacle2", "obstacle3", "obstacle4"};
//
//        for(const auto & objectName : objectNames){
//            while(!validPlacement){
//                sizeX += 0.001;
//                sizeY += 0.0005;
//
//                float randX = randFloat(goalX - sizeX, goalX + (0.5f * sizeX));
//                float randY = randFloat(goalY - sizeY, goalY + sizeY);
//
//                pose_6 objectCurrentPose;
//                pose_6 newObjectPose;
//
//                MuJoCo_helper->GetBodyPoseAngle(objectName, objectCurrentPose, MuJoCo_helper->master_reset_data);
//                newObjectPose = objectCurrentPose;
//                newObjectPose.position(0) = randX;
//                newObjectPose.position(1) = randY;
//                newObjectPose.position(2) = objectCurrentPose.position(2);
//                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->main_data);
//                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->master_reset_data);
//
//                if(MuJoCo_helper->CheckBodyForCollisions(objectName, MuJoCo_helper->main_data)){
//                    cout << "invalid placement at : " << randX << ", " << randY << endl;
//                }
//                else{
//                    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);
//                    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->master_reset_data);
//                    validPlacement = true;
//                    objectXPos.push_back(randX);
//                    objectYPos.push_back(randY);
//                }
//            }
//        }
//
//        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                startX, startY, objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1],
//                objectXPos[2], objectYPos[2], objectXPos[3], objectYPos[3],
//                objectXPos[4], objectYPos[4], objectXPos[5], objectYPos[5], objectXPos[6], objectYPos[6],
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0;
//    }
//
//    return randomStartState;
//}
//
//MatrixXd TwoDPushing::ReturnRandomGoalState(MatrixXd X0){
//    MatrixXd randomGoalState(state_vector_size, 1);
//
//    if(clutterLevel == noClutter){
//        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                randomGoalX, randomGoalY,
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0;
//    }
//    else if(clutterLevel == lowClutter || clutterLevel == constrainedClutter){
//        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                randomGoalX, randomGoalY, X0(9), X0(10), X0(11), X0(12), X0(13), X0(14),
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0, 0, 0;
//    }
//    else if(clutterLevel == heavyClutter){
//        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                randomGoalX, randomGoalY, X0(9), X0(10), X0(11), X0(12),
//                X0(13), X0(14), X0(15), X0(16),
//                X0(17), X0(18), X0(19), X0(20), X0(21), X0(22),
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0;
//
//    }
//
//
//    return randomGoalState;
//}

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
    goalPos(0) = active_state_vector.bodiesStates[0].goalLinearPos[0];
    goalPos(1) = active_state_vector.bodiesStates[0].goalLinearPos[1];
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
    displayBodyPose.position[0] = active_state_vector.bodiesStates[0].goalLinearPos[0];
    displayBodyPose.position[1] = active_state_vector.bodiesStates[0].goalLinearPos[1];
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->SetBodyPoseAngle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goalPos(0) = active_state_vector.bodiesStates[0].goalLinearPos[0];
    goalPos(1) = active_state_vector.bodiesStates[0].goalLinearPos[1];
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

    MatrixXd currentState = ReturnStateVector(d);

    double x_diff = currentState(7) - active_state_vector.bodiesStates[0].goalLinearPos[0];
    double y_diff = currentState(8) - active_state_vector.bodiesStates[0].goalLinearPos[1];

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.025){
        taskComplete = true;
    }


    return taskComplete;
}