#include "ModelTranslator/BoxFlick.h"

BoxFlick::BoxFlick(int _clutterLevel) : PushBaseClass("franka_gripper", "goal"){

    clutterLevel = _clutterLevel;
    std::string yamlFilePath = "/taskConfigs/boxFlickConfig.yaml";

    if(clutterLevel == noClutter){
        yamlFilePath = "/taskConfigs/boxFlickConfig.yaml";
    }
    else if(clutterLevel == lowClutter){
        yamlFilePath = "/taskConfigs/boxFlickLowClutterConfig.yaml";
    }
    else if(clutterLevel == heavyClutter){
        yamlFilePath = "/taskConfigs/boxFlickHeavyClutterConfig.yaml";
    }
    else{
        cout << "ERROR: Invalid clutter level" << endl;
    }

    InitModelTranslator(yamlFilePath);
}

//MatrixXd BoxFlick::ReturnRandomStartState(){
//    MatrixXd randomStartState(state_vector_size, 1);
//
//    float startX = randFloat(0.4, 0.55);
//    float startY = randFloat(-0.2, 0.2);
//
//    pose_6 stackedObjectPose;
//    stackedObjectPose.position(0) = startX;
//    stackedObjectPose.position(1) = startY;
//    stackedObjectPose.position(2) = 0.2;
//
//    std::vector<double> objectXPos;
//    std::vector<double> objectYPos;
//
//    pose_6 objectCurrentPose;
//    MuJoCo_helper->GetBodyPoseAngle("goal", objectCurrentPose, MuJoCo_helper->master_reset_data);
//    objectCurrentPose.position(0) = startX;
//    objectCurrentPose.position(1) = startY;
//    MuJoCo_helper->SetBodyPoseAngle("goal", objectCurrentPose, MuJoCo_helper->main_data);
//    MuJoCo_helper->SetBodyPoseAngle("goal", objectCurrentPose, MuJoCo_helper->master_reset_data);
//
//    MuJoCo_helper->GetBodyPoseAngle("mainObstacle", objectCurrentPose, MuJoCo_helper->master_reset_data);
//    objectCurrentPose.position(0) = startX;
//    objectCurrentPose.position(1) = startY;
//    objectCurrentPose.position(2) = 0.2;
//    MuJoCo_helper->SetBodyPoseAngle("mainObstacle", objectCurrentPose, MuJoCo_helper->main_data);
//    MuJoCo_helper->SetBodyPoseAngle("mainObstacle", objectCurrentPose, MuJoCo_helper->master_reset_data);
//
//
//    if(clutterLevel == noClutter){
//        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                startX, startY, stackedObjectPose.position(0), stackedObjectPose.position(1), stackedObjectPose.position(2),
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0;
//    }
//    else if(clutterLevel == lowClutter){
//
//        std::string objectNames[2] = {"obstacle1", "obstacle2"};
//        int validObjectCounter = 0;
//
//        for(const auto & objectName : objectNames){
//            bool validPlacement = false;
//            float sizeX = 0.08;
//            float sizeY = 0.04;
//            while(!validPlacement){
//                sizeX += 0.005;
//                sizeY += 0.005;
//
//                float randX = randFloat(startX, startX + sizeX);
//                float randY = randFloat(startY - sizeY, startY + sizeY);
//
//                pose_6 newObjectPose;
//
//                MuJoCo_helper->GetBodyPoseAngle(objectName, objectCurrentPose, MuJoCo_helper->master_reset_data);
//                newObjectPose = objectCurrentPose;
//                newObjectPose.position(0) = randX;
//                newObjectPose.position(1) = randY;
//                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->main_data);
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
//        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                startX, startY, stackedObjectPose.position(0), stackedObjectPose.position(1), stackedObjectPose.position(2),
//                objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1],
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0,
//                0, 0, 0, 0;
//
//    }
//    else if(clutterLevel == heavyClutter){
//
//        std::string objectNames[6] = {"obstacle1", "obstacle2", "obstacle3", "obstacle4", "obstacle5", "obstacle6"};
//        int validObjectCounter = 0;
//
//        for(const auto & objectName : objectNames){
//            bool validPlacement = false;
//            float sizeX = 0.08;
//            float sizeY = 0.05;
//            while(!validPlacement){
//                sizeX += 0.002;
//                sizeY += 0.001;
//
//                float randX = randFloat(startX + 0.04f, startX+ 0.04f + sizeX);
//                float randY = randFloat(startY - sizeY, startY + sizeY);
//
//                pose_6 newObjectPose;
//
//                MuJoCo_helper->GetBodyPoseAngle(objectName, objectCurrentPose, MuJoCo_helper->master_reset_data);
//                newObjectPose = objectCurrentPose;
//                newObjectPose.position(0) = randX;
//                newObjectPose.position(1) = randY;
//                MuJoCo_helper->SetBodyPoseAngle(objectName, newObjectPose, MuJoCo_helper->main_data);
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
//        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                startX, startY, stackedObjectPose.position(0), stackedObjectPose.position(1), stackedObjectPose.position(2),
//                objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1],
//                objectXPos[2], objectYPos[2], objectXPos[3], objectYPos[3], objectXPos[4], objectYPos[4], objectXPos[5], objectYPos[5],
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0,
//                0, 0, 0, 0,
//                0 ,0, 0, 0, 0, 0, 0, 0;
//
//    }
//    else{
//        cout << "ERROR: Invalid clutter level" << endl;
//    }
//
//    return randomStartState;
//}
//
//MatrixXd BoxFlick::ReturnRandomGoalState(MatrixXd X0){
//    MatrixXd randomGoalState(state_vector_size, 1);
//
//    if(clutterLevel == noClutter){
//        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                X0(7), X0(8), X0(9), X0(10), X0(11),
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0;
//    }
//    else if(clutterLevel == lowClutter){
//        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                X0(7), X0(8), X0(9), X0(10), X0(11),
//                X0(12), X0(13), X0(14), X0(15),
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0,
//                0, 0, 0, 0;
//    }
//    else if(clutterLevel == heavyClutter){
//        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
//                X0(7), X0(8), X0(9), X0(10), X0(11),
//                X0(12), X0(13), X0(14), X0(15),
//                X0(16), X0(17), X0(18), X0(19), X0(20), X0(21), X0(22), X0(23),
//                0, 0, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0,
//                0, 0, 0, 0,
//                0, 0, 0, 0, 0, 0, 0, 0;
//    }
//    else{
//        cout << "ERROR: Invalid clutter level" << endl;
//    }
//
//    return randomGoalState;
//}

//double BoxFlick::CostFunction(mjData *d, bool terminal){
//    double cost;
////    MatrixXd Xt = ReturnStateVector(d);
////    MatrixXd Ut = ReturnControlVector(d);
////
////    MatrixXd X_diff = Xt - X_desired;
////    MatrixXd temp;
////
////    double obstacleDistCost;
////
////    double objectsDiffX = Xt(9) - boxStartX;
////    double objectsDiffY = Xt(10) - boxStartY;
////
////    obstacleDistCost = (A * exp(-(pow(objectsDiffX,2)/sigma))) + (A * exp(-(pow(objectsDiffY,2)/sigma)));
////
//////    obstacleDistCost = 0.01/(pow(objectsDiffX,2) + 0.1) + 0.01/(pow(objectsDiffY,2) + 0.1);
////
//////    if(terminal){
//////        obstacleDistCost = 1/(pow(hypotenuseDiff,2) + 0.1);
//////    }
////
////    if(terminal){
////        temp = ((X_diff.transpose() * Q_terminal * X_diff)) + (Ut.transpose() * R * Ut);
////    }
////    else{
////        temp = ((X_diff.transpose() * Q * X_diff)) + (Ut.transpose() * R * Ut);
////    }
////
////    cost = temp(0) + obstacleDistCost;
//
//    return cost;
//}
//
//void BoxFlick::CostDerivatives(mjData *d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
////    MatrixXd Xt = ReturnStateVector(d);
////    MatrixXd Ut = ReturnControlVector(d);
////    MatrixXd X_diff = Xt - X_desired;
////
////    // Special elemetns for gaussian distance of obstacle1 to goal
////    double objectsDiffX = Xt(9) - boxStartX;
////    double objectsDiffY = Xt(10) - boxStartY;
////
////    double exponentialX = exp(-(pow(objectsDiffX,2)/sigma));
////    double exponentialY = exp(-(pow(objectsDiffY,2)/sigma));
////
////    double l_x_X_add = -(2 * A * objectsDiffX * exponentialX)/sigma;
////    double l_x_Y_add = -(2 * A * objectsDiffY * exponentialY)/sigma;
////
////    double l_xx_X_add = (-2 * A * objectsDiffX * exponentialX)/sigma + (4 * pow(A, 2) * pow(objectsDiffX,2) * exponentialX)/pow(sigma,2);
////    double l_xx_y_add = (-2 * A * objectsDiffY * exponentialY)/sigma + (4 * pow(A, 2) * pow(objectsDiffY,2) * exponentialY)/pow(sigma,2);
////
////    // Size cost derivatives appropriately
////    l_x.resize(state_vector_size, 1);
////    l_xx.resize(state_vector_size, state_vector_size);
////
////    l_u.resize(num_ctrl, 1);
////    l_uu.resize(num_ctrl, num_ctrl);
////
////    if(terminal){
////        l_x = 2 * Q_terminal * X_diff;
////        l_xx = 2 * Q_terminal;
////    }
////    else{
////        l_x = 2 * Q * X_diff;
////        l_xx = 2 * Q;
////    }
////
////    l_x(9) = l_x(9) + l_x_X_add;
////    l_x(10) = l_x(10) + l_x_Y_add;
////
////    l_xx(9,9) = l_xx(9,9) + l_xx_X_add;
////    l_xx(10,10) = l_xx(10,10) + l_xx_y_add;
////
////    l_u = 2 * R * Ut;
////    l_uu = 2 * R;
//}

std::vector<MatrixXd> BoxFlick::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    MuJoCo_helper->ForwardSimulator(MuJoCo_helper->main_data);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
//    goal_pos(0) = X_desired(7) + 0.2;
//    goal_pos(1) = X_desired(8) + 0.01;

    EEWayPointsSetup(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);

//    boxStartX = X_desired(7);
//    boxStartY = X_desired(8);

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initSetupControls = JacobianEEControl(goal_pos, allWayPoints);

    return initSetupControls;
}

std::vector<MatrixXd> BoxFlick::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> init_controls;

    // Set the goal position so that we can see where we are pushing to


    // Pushing create init controls borken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goal_pos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
//    goal_pos(0) = X_desired(7) + 0.2;
//    goal_pos(1) = X_desired(8) + 0.01;

    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    displayBodyPose.position[0] = goal_pos(0);
    displayBodyPose.position[1]  = goal_pos(1);
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->SetBodyPoseAngle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);
    EEWayPointsSetup(goal_pos, mainWayPoints, mainWayPointsTimings, horizonLength);
    cout << "mainwaypoint 0: " << mainWayPoints[0] << endl;
    cout << "mainWayPoint " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = CreateAllEETransitPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    init_controls = JacobianEEControl(goal_pos, allWayPoints);

    return init_controls;
}

bool BoxFlick::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("goal", goal_pose, d);

    double x_diff = goal_pose.position(0) - full_state_vector.rigid_bodies[0].goal_linear_pos[0];
    double y_diff = goal_pose.position(1) - full_state_vector.rigid_bodies[0].goal_linear_pos[1];

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    std::cout << "distance is: " << dist << std::endl;

    if(dist > 0.1){
        taskComplete = true;
    }


    return taskComplete;
}