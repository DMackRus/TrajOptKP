#include "BoxFlick.h"

BoxFlick::BoxFlick(int _clutterLevel){

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

void BoxFlick::GenerateRandomGoalAndStartState() {
    X_start = ReturnRandomStartState();
    X_desired = ReturnRandomGoalState(X_start);
}

MatrixXd BoxFlick::ReturnRandomStartState(){
    MatrixXd randomStartState(state_vector_size, 1);

    float randStartAngle = randFloat(0, PI);
    float randStartDist = randFloat(0.05, 0.2);

    float startX = randFloat(0.4, 0.55);
    float startY = randFloat(-0.2, 0.2);

    pose_6 stackedObjectPose;
    stackedObjectPose.position(0) = startX;
    stackedObjectPose.position(1) = startY;
    stackedObjectPose.position(2) = 0.2;

    std::vector<double> objectXPos;
    std::vector<double> objectYPos;

    pose_6 objectCurrentPose;
    MuJoCo_helper->getBodyPose_angle("goal", objectCurrentPose, MASTER_RESET_DATA);
    objectCurrentPose.position(0) = startX;
    objectCurrentPose.position(1) = startY;
    MuJoCo_helper->setBodyPose_angle("goal", objectCurrentPose, MAIN_DATA_STATE);
    MuJoCo_helper->setBodyPose_angle("goal", objectCurrentPose, MASTER_RESET_DATA);

    MuJoCo_helper->getBodyPose_angle("mainObstacle", objectCurrentPose, MASTER_RESET_DATA);
    objectCurrentPose.position(0) = startX;
    objectCurrentPose.position(1) = startY;
    objectCurrentPose.position(2) = 0.2;
    MuJoCo_helper->setBodyPose_angle("mainObstacle", objectCurrentPose, MAIN_DATA_STATE);
    MuJoCo_helper->setBodyPose_angle("mainObstacle", objectCurrentPose, MASTER_RESET_DATA);


    if(clutterLevel == noClutter){
        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY, stackedObjectPose.position(0), stackedObjectPose.position(1), stackedObjectPose.position(2),
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0;
    }
    else if(clutterLevel == lowClutter){

        std::string objectNames[2] = {"obstacle1", "obstacle2"};
        int validObjectCounter = 0;

        for(int i = 0; i < 2; i++){
            bool validPlacement = false;
            float sizeX = 0.08;
            float sizeY = 0.04;
            while(!validPlacement){
                sizeX += 0.005;
                sizeY += 0.005;

                float randX = randFloat(startX, startX + sizeX);
                float randY = randFloat(startY - sizeY, startY + sizeY);

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                MuJoCo_helper->getBodyPose_angle(objectNames[i], objectCurrentPose, MASTER_RESET_DATA);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                MuJoCo_helper->setBodyPose_angle(objectNames[i], newObjectPose, MAIN_DATA_STATE);

                if(MuJoCo_helper->checkBodyForCollisions(objectNames[i], MAIN_DATA_STATE)){
                    cout << "invalid placement at : " << randX << ", " << randY << endl;
                }
                else{
                    validPlacement = true;
                    objectXPos.push_back(randX);
                    objectYPos.push_back(randY);
                }
            }
            validObjectCounter++;
        }

        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY, stackedObjectPose.position(0), stackedObjectPose.position(1), stackedObjectPose.position(2),
                objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1],
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0;

    }
    else if(clutterLevel == heavyClutter){

        std::string objectNames[6] = {"obstacle1", "obstacle2", "obstacle3", "obstacle4", "obstacle5", "obstacle6"};
        int validObjectCounter = 0;

        for(int i = 0; i < 6; i++){
            bool validPlacement = false;
            float sizeX = 0.08;
            float sizeY = 0.05;
            while(!validPlacement){
                sizeX += 0.002;
                sizeY += 0.001;

                float randX = randFloat(startX + 0.04, startX + 0.04 + sizeX);
                float randY = randFloat(startY - sizeY, startY + sizeY);

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                MuJoCo_helper->getBodyPose_angle(objectNames[i], objectCurrentPose, MASTER_RESET_DATA);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                MuJoCo_helper->setBodyPose_angle(objectNames[i], newObjectPose, MAIN_DATA_STATE);

                if(MuJoCo_helper->checkBodyForCollisions(objectNames[i], MAIN_DATA_STATE)){
                    cout << "invalid placement at : " << randX << ", " << randY << endl;
                }
                else{
                    validPlacement = true;
                    objectXPos.push_back(randX);
                    objectYPos.push_back(randY);
                }
            }
            validObjectCounter++;
        }

        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY, stackedObjectPose.position(0), stackedObjectPose.position(1), stackedObjectPose.position(2),
                objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1],
                objectXPos[2], objectYPos[2], objectXPos[3], objectYPos[3], objectXPos[4], objectYPos[4], objectXPos[5], objectYPos[5],
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0,
                0 ,0, 0, 0, 0, 0, 0, 0;

    }
    else{
        cout << "ERROR: Invalid clutter level" << endl;
    }

    return randomStartState;
}

MatrixXd BoxFlick::ReturnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(state_vector_size, 1);

    if(clutterLevel == noClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                X0(7), X0(8), X0(9), X0(10), X0(11),
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0;
    }
    else if(clutterLevel == lowClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                X0(7), X0(8), X0(9), X0(10), X0(11),
                X0(12), X0(13), X0(14), X0(15),
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0;
    }
    else if(clutterLevel == heavyClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                X0(7), X0(8), X0(9), X0(10), X0(11),
                X0(12), X0(13), X0(14), X0(15),
                X0(16), X0(17), X0(18), X0(19), X0(20), X0(21), X0(22), X0(23),
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0;
    }
    else{
        cout << "ERROR: Invalid clutter level" << endl;
    }

    return randomGoalState;
}

double BoxFlick::CostFunction(int dataIndex, bool terminal){
    double cost = 0.0f;
    MatrixXd Xt = ReturnStateVector(dataIndex);
    MatrixXd Ut = ReturnControlVector(dataIndex);

    MatrixXd X_diff = Xt - X_desired;
    MatrixXd temp;

    double obstacleDistCost = 0.0f;

    double objectsDiffX = Xt(9) - boxStartX;
    double objectsDiffY = Xt(10) - boxStartY;

    obstacleDistCost = (A * exp(-(pow(objectsDiffX,2)/sigma))) + (A * exp(-(pow(objectsDiffY,2)/sigma)));

//    obstacleDistCost = 0.01/(pow(objectsDiffX,2) + 0.1) + 0.01/(pow(objectsDiffY,2) + 0.1);

//    if(terminal){
//        obstacleDistCost = 1/(pow(hypotenuseDiff,2) + 0.1);
//    }

    if(terminal){
        temp = ((X_diff.transpose() * Q_terminal * X_diff)) + (Ut.transpose() * R * Ut);
    }
    else{
        temp = ((X_diff.transpose() * Q * X_diff)) + (Ut.transpose() * R * Ut);
    }

    cost = temp(0) + obstacleDistCost;

    cost = obstacleDistCost;

    return cost;
}

void BoxFlick::CostDerivatives(int dataIndex, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){
    MatrixXd Xt = ReturnStateVector(dataIndex);
    MatrixXd Ut = ReturnControlVector(dataIndex);
    MatrixXd X_diff = Xt - X_desired;

    // Special elemetns for gaussian distance of obstacle1 to goal
    double objectsDiffX = Xt(9) - boxStartX;
    double objectsDiffY = Xt(10) - boxStartY;

    double exponentialX = exp(-(pow(objectsDiffX,2)/sigma));
    double exponentialY = exp(-(pow(objectsDiffY,2)/sigma));

    double l_x_X_add = -(2 * A * objectsDiffX * exponentialX)/sigma;
    double l_x_Y_add = -(2 * A * objectsDiffY * exponentialY)/sigma;

    double l_xx_X_add = (-2 * A * objectsDiffX * exponentialX)/sigma + (4 * pow(A, 2) * pow(objectsDiffX,2) * exponentialX)/pow(sigma,2);
    double l_xx_y_add = (-2 * A * objectsDiffY * exponentialY)/sigma + (4 * pow(A, 2) * pow(objectsDiffY,2) * exponentialY)/pow(sigma,2);

    // Size cost derivatives appropriately
    l_x.resize(state_vector_size, 1);
    l_xx.resize(state_vector_size, state_vector_size);

    l_u.resize(num_ctrl, 1);
    l_uu.resize(num_ctrl, num_ctrl);

    if(terminal){
        l_x = 2 * Q_terminal * X_diff;
        l_xx = 2 * Q_terminal;
    }
    else{
        l_x = 2 * Q * X_diff;
        l_xx = 2 * Q;
    }

    l_x(9) = l_x(9) + l_x_X_add;
    l_x(10) = l_x(10) + l_x_Y_add;

    l_xx(9,9) = l_xx(9,9) + l_xx_X_add;
    l_xx(10,10) = l_xx(10,10) + l_xx_y_add;

    l_u = 2 * R * Ut;
    l_uu = 2 * R;
}

std::vector<MatrixXd> BoxFlick::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    MuJoCo_helper->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    MuJoCo_helper->forwardSimulator(MAIN_DATA_STATE);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goalPos(0) = X_desired(7) + 0.2;
    goalPos(1) = X_desired(8) + 0.01;

    initControls_mainWayPoints_setup(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);

    boxStartX = X_desired(7);
    boxStartY = X_desired(8);

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = initControls_createAllWayPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initSetupControls = generate_initControls_fromWayPoints(allWayPoints);

    return initSetupControls;
}

void BoxFlick::initControls_mainWayPoints_setup(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon){
    const std::string goalObject = "mainObstacle";
    const std::string EE_name = "franka_gripper";

    pose_6 EE_startPose;
    pose_6 goalobj_startPose;
    MuJoCo_helper->getBodyPose_angle(EE_name, EE_startPose, MAIN_DATA_STATE);
    MuJoCo_helper->getBodyPose_angle(goalObject, goalobj_startPose, MAIN_DATA_STATE);

    m_point mainWayPoint;
    // First waypoint - where the end-effector is currently
    mainWayPoint << EE_startPose.position(0), EE_startPose.position(1), EE_startPose.position(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    // Calculate the angle of approach - from goal position to object start position
    float angle_EE_push;
    float x_diff = desiredObjectEnd(0) - goalobj_startPose.position(0);
    float y_diff = desiredObjectEnd(1) - goalobj_startPose.position(1);
    angle_EE_push = atan2(y_diff, x_diff);

    // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
//    float cylinder_radius = 0.08;
    float cylinder_radius = 0.1;
    float x_cylinder0ffset = cylinder_radius * cos(angle_EE_push);
    float y_cylinder0ffset = cylinder_radius * sin(angle_EE_push);

    float endPointX;
    float endPointY;
    float intermediatePointY = goalobj_startPose.position(1);
    float intermediatePointX = goalobj_startPose.position(0);

    intermediatePointX = intermediatePointX - 0.15*cos(angle_EE_push);
    intermediatePointY = intermediatePointY - 0.15*sin(angle_EE_push);
//    if(desiredObjectEnd(1) - goalobj_startPose.position(1) > 0){
//        intermediatePointY = intermediatePointY + y_cylinder0ffset;
//    }
//    else{
//        intermediatePointY = intermediatePointY - y_cylinder0ffset;
//    }

    std::string goalMarkerName = "display_intermediate";
    pose_6 displayBodyPose;
    displayBodyPose.position[0] = intermediatePointX;
//    displayBodyPose.position[0] = 100.0f;
    displayBodyPose.position[1] = intermediatePointY;
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->setBodyPose_angle(goalMarkerName, displayBodyPose, MASTER_RESET_DATA);

    mainWayPoint(0) = intermediatePointX;
    mainWayPoint(1) = intermediatePointY;
    mainWayPoint(2) = 0.27f;
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(horizon - 1);

}

std::vector<MatrixXd> BoxFlick::CreateInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Set the goal position so that we can see where we are pushing to


    // Pushing create init controls borken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goalPos(0) = X_desired(7) + 0.2;
    goalPos(1) = X_desired(8) + 0.01;

    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    displayBodyPose.position[0] = goalPos(0);
    displayBodyPose.position[1]  = goalPos(1);
//    displayBodyPose.position[1] = 100.0f;
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->setBodyPose_angle(goalMarkerName, displayBodyPose, MASTER_RESET_DATA);
    initControls_mainWayPoints_optimisation(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
    cout << "mainwaypoint 0: " << mainWayPoints[0] << endl;
    cout << "mainWayPoint " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = initControls_createAllWayPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initControls = generate_initControls_fromWayPoints(allWayPoints);

    return initControls;

}

void BoxFlick::initControls_mainWayPoints_optimisation(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon){
    const std::string goalObject = "mainObstacle";
    const std::string EE_name = "franka_gripper";

    pose_6 EE_startPose;
    pose_6 goalobj_startPose;
    MuJoCo_helper->getBodyPose_angle(EE_name, EE_startPose, MAIN_DATA_STATE);
    MuJoCo_helper->getBodyPose_angle(goalObject, goalobj_startPose, MAIN_DATA_STATE);

    m_point mainWayPoint;
    // First waypoint - where the end-effector is currently
    mainWayPoint << EE_startPose.position(0), EE_startPose.position(1), EE_startPose.position(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    // Calculate the angle of approach - from goal position to object start position
    float angle_EE_push;
    float x_diff = desiredObjectEnd(0) - goalobj_startPose.position(0);
    float y_diff = desiredObjectEnd(1) - goalobj_startPose.position(1);
    angle_EE_push = atan2(y_diff, x_diff);

    // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
//    float cylinder_radius = 0.08;
    float cylinder_radius = 0.1;
    float x_cylinder0ffset = cylinder_radius * cos(angle_EE_push);
    float y_cylinder0ffset = cylinder_radius * sin(angle_EE_push);

    float desired_endPointX = desiredObjectEnd(0) - x_cylinder0ffset;
    float desired_endPointY;

    float endPointX;
    float endPointY;
    if(desiredObjectEnd(1) - goalobj_startPose.position(1) > 0){
        desired_endPointY = desiredObjectEnd(1) + y_cylinder0ffset;
    }
    else{
        desired_endPointY = desiredObjectEnd(1) - y_cylinder0ffset;
    }

    float intermediatePointY = goalobj_startPose.position(1);
    float intermediatePointX = goalobj_startPose.position(0);

    // // Setting this up so we can visualise where the intermediate point is located
    // intermediatePoint(0) = intermediatePointX;
    // intermediatePoint(1) = intermediatePointY;

    float maxDistTravelled = 0.05 * ((5.0f/6.0f) * horizon * MuJoCo_helper->returnModelTimeStep());
    // float maxDistTravelled = 0.05 * ((5.0f/6.0f) * horizon * MUJOCO_DT);
//    cout << "max EE travel dist: " << maxDistTravelled << endl;
    float desiredDistTravelled = sqrt(pow((desired_endPointX - intermediatePointX),2) + pow((desired_endPointY - intermediatePointY),2));
    float proportionOfDistTravelled = maxDistTravelled / desiredDistTravelled;
//    cout << "proportion" << proportionOfDistTravelled << endl;
    if(proportionOfDistTravelled > 1){
        endPointX = desired_endPointX;
        endPointY = desired_endPointY;
    }
    else{
        endPointX = intermediatePointX + ((desired_endPointX - intermediatePointX) * proportionOfDistTravelled);
        endPointY = intermediatePointY + ((desired_endPointY - intermediatePointY) * proportionOfDistTravelled);
    }

    mainWayPoint(0) = endPointX;
    mainWayPoint(1) = endPointY;
    mainWayPoint(2) = 0.27f;

    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(horizon - 1);
}

std::vector<m_point> BoxFlick::initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming){
    int numMainWayPoints = mainWayPoints.size();
    std::vector<m_point> initPath;

    initPath.push_back(mainWayPoints[0]);
    wayPointsTiming[0]--;

    // should only be MUJ_STEPS_HORIZON_LENGTH number of controls
    int counter = 1;
    for(int i = 0; i < numMainWayPoints - 1; i++){
        float x_diff = mainWayPoints[i + 1](0) - mainWayPoints[i](0);
        float y_diff = mainWayPoints[i + 1](1) - mainWayPoints[i](1);
        float z_diff = mainWayPoints[i + 1](2) - mainWayPoints[i](2);
        for(int j = 0; j < wayPointsTiming[i + 1]; j++){
            initPath.push_back(m_point());
            initPath[counter](0) = initPath[counter - 1](0) + (x_diff / wayPointsTiming[i + 1]);
            initPath[counter](1) = initPath[counter - 1](1) + (y_diff / wayPointsTiming[i + 1]);
            initPath[counter](2) = initPath[counter - 1](2) + (z_diff / wayPointsTiming[i + 1]);

            counter++;
        }
    }

    return initPath;
}

std::vector<MatrixXd> BoxFlick::generate_initControls_fromWayPoints(std::vector<m_point> initPath){
    std::vector<MatrixXd> initControls;
    std::string goalObjName = "goal";
    std::string EEName = "franka_gripper";

    pose_7 EE_start_pose;
    pose_6 goalobj_startPose;
    MuJoCo_helper->getBodyPose_quat(EEName, EE_start_pose, MAIN_DATA_STATE);
    MuJoCo_helper->getBodyPose_angle(goalObjName, goalobj_startPose, MAIN_DATA_STATE);

    float angle_EE_push;
    float x_diff = 0.2;
    float y_diff = 0.01;
    angle_EE_push = atan2(y_diff, x_diff);

    if(angle_EE_push < 0){
        angle_EE_push = angle_EE_push + (2*PI);
//        cout << "converted angle is: " << convertedAngle << endl;
    }

    double convertedAngle = angle_EE_push - (PI/4);

    // Setup the desired rotation matrix for the end-effector
    m_point xAxis, yAxis, zAxis;
    xAxis << cos(convertedAngle), sin(convertedAngle), 0;
    zAxis << 0, 0, -1;
    yAxis = crossProduct(zAxis, xAxis);
    // xAxis << 0, 0, -1;
    // zAxis << cos(convertedAngle), sin(convertedAngle), 0;
    // yAxis = crossProduct(zAxis, xAxis);

    Eigen::Matrix3d rotMat;
    rotMat << xAxis(0), yAxis(0), zAxis(0),
            xAxis(1), yAxis(1), zAxis(1),
            xAxis(2), yAxis(2), zAxis(2);

    m_quat desiredQuat = rotMat2Quat(rotMat);

    MatrixXd currentControl(num_ctrl, 1);
    if(active_state_vector.robots[0].torqueControlled){
        MatrixXd robotPos = returnPositionVector(MAIN_DATA_STATE);
        for(int i = 0; i < num_ctrl; i++){
            currentControl(i) = robotPos(i);
        }
    }

    for(int i = 0; i < initPath.size(); i++){
        pose_7 currentEEPose;
        MuJoCo_helper->getBodyPose_quat(EEName, currentEEPose, MAIN_DATA_STATE);
        m_quat currentEEQuat, invertedQuat, quatDiff;
        currentEEQuat(0) = currentEEPose.quat(0);
        currentEEQuat(1) = currentEEPose.quat(1);
        currentEEQuat(2) = currentEEPose.quat(2);
        currentEEQuat(3) = currentEEPose.quat(3);
        invertedQuat = invQuat(currentEEQuat);
        quatDiff = multQuat(desiredQuat, invertedQuat);


        m_point axisDiff = quat2Axis(quatDiff);
        MatrixXd differenceFromPath(6, 1);
        float gainsTorque[6] = {100, 100, 100, 500, 500, 500};
        float gainsPositionControl[6] = {10000, 10000, 30000, 5000, 5000, 5000};

        for(int j = 0; j < 3; j++){
            differenceFromPath(j) = initPath[i](j) - currentEEPose.position(j);
            differenceFromPath(j + 3) = axisDiff(j);
        }

        MatrixXd Jac, JacInv;

        Jac = MuJoCo_helper->calculateJacobian(EEName, MAIN_DATA_STATE);
        JacInv = Jac.completeOrthogonalDecomposition().pseudoInverse();

        MatrixXd desiredEEForce(6, 1);
        MatrixXd desiredControls(num_ctrl, 1);

        if(active_state_vector.robots[0].torqueControlled){
            for(int j = 0; j < 6; j++) {
                desiredEEForce(j) = differenceFromPath(j) * gainsTorque[j];
            }
            desiredControls = JacInv * desiredEEForce;

            std::vector<double> gravCompensation;
            MatrixXd gravCompControl(num_ctrl, 1);
            MuJoCo_helper->getRobotJointsGravityCompensaionControls(active_state_vector.robots[0].name, gravCompensation, MAIN_DATA_STATE);
            for(int j = 0; j < num_ctrl; j++){
                gravCompControl(j) = gravCompensation[j];
            }
            desiredControls += gravCompControl;
        }
            // Position control
        else{
            for(int j = 0; j < 6; j++) {
                desiredEEForce(j) = differenceFromPath(j) * gainsPositionControl[j] * 0.000001;
            }
            desiredControls += JacInv * desiredEEForce;
            //cout << "desired controls: " << desiredControls << endl;
        }


        initControls.push_back(desiredControls);

        SetControlVector(desiredControls, MAIN_DATA_STATE);
        MuJoCo_helper->stepSimulator(1, MAIN_DATA_STATE);

    }

    return initControls;
}

bool BoxFlick::TaskComplete(int dataIndex, double &dist){
    bool taskComplete = false;

    MatrixXd currentState = ReturnStateVector(dataIndex);

    float x_diff = currentState(9) - currentState(7);
    float y_diff = currentState(10) - currentState(8);

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
    std::cout << "distance is: " << dist << std::endl;

    if(dist > 0.1){
        taskComplete = true;
    }


    return taskComplete;
}