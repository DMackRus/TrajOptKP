#include "twoDPushing.h"

twoDPushing::twoDPushing(int _clutterLevel){

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
    else{
        cout << "ERROR: Invalid clutter level" << endl;
    }

    initModelTranslator(yamlFilePath);
}

void twoDPushing::generateRandomGoalAndStartState() {
    X_start = returnRandomStartState();
    X_desired = returnRandomGoalState(X_start);
}

MatrixXd twoDPushing::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

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
//        float randStartAngle = randFloat(0, PI);
//        float randStartDist = randFloat(0.05, 0.1);

        startX = 0.4;
        startY = randFloat(-0.1, 0.1);

        float randAngle = randFloat(-PI/4, PI/4);
        float randDist = randFloat(0.28, 0.3);

        goalX = startX + randDist * cos(randAngle);
        goalY = startY + randDist * sin(randAngle);
    }

    // Set start position of pushed object
    pose_6 pushedObjectStartPose;
    activePhysicsSimulator->getBodyPose_angle("blueTin", pushedObjectStartPose, MASTER_RESET_DATA);
    pushedObjectStartPose.position(0) = startX;
    pushedObjectStartPose.position(1) = startY;
    pushedObjectStartPose.position(2) = 0.032;
    activePhysicsSimulator->setBodyPose_angle("blueTin", pushedObjectStartPose, MAIN_DATA_STATE);
    activePhysicsSimulator->setBodyPose_angle("blueTin", pushedObjectStartPose, MASTER_RESET_DATA);
    activePhysicsSimulator->forwardSimulator(MAIN_DATA_STATE);
    activePhysicsSimulator->forwardSimulator(MASTER_RESET_DATA);


    randomGoalX = goalX;
    randomGoalY = goalY;

    std::vector<double> objectXPos;
    std::vector<double> objectYPos;

    if(clutterLevel == noClutter){
        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY,
                0, 0, 0, 0, 0, 0, 0,
                0, 0;
    }
    else if(clutterLevel == lowClutter  || clutterLevel == constrainedClutter){

        std::string objectNames[3] = {"bigBox", "smallBox","tallCylinder"};
        int validObjectCounter = 0;

        for(int i = 0; i < 3; i++){
            bool validPlacement = false;
            float sizeX = 0.01;
            float sizeY = 0.05;
            while(!validPlacement){
                sizeX += 0.0005;
                sizeY += 0.0001;

                float randX, randY;

                if(clutterLevel == constrainedClutter){
                    randX = randFloat(startX, goalX + 0.1);
                    randY = randFloat(goalY - sizeY, goalY + sizeY);
                }
                else{
                    randX = randFloat(goalX - sizeX, goalX);
                    randY = randFloat(goalY - sizeY, goalY + sizeY);
                }

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                activePhysicsSimulator->getBodyPose_angle(objectNames[i], objectCurrentPose, MAIN_DATA_STATE);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                activePhysicsSimulator->setBodyPose_angle(objectNames[i], newObjectPose, MAIN_DATA_STATE);
                activePhysicsSimulator->setBodyPose_angle(objectNames[i], newObjectPose, MASTER_RESET_DATA);

                if(activePhysicsSimulator->checkBodyForCollisions(objectNames[i], MAIN_DATA_STATE)){
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

        //1, -0.07, 0, -3, 0.232, 1.34, 3, 0.232,
        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY, objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1], objectXPos[2], objectYPos[2],
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0;

    }
    else if(clutterLevel == heavyClutter){

        bool validPlacement = false;
        float sizeX = 0.08;
        float sizeY = 0.04;
        cout << "goal position: " << goalX << ", " << goalY << endl;
        while(!validPlacement){
            sizeX += 0.001;
            sizeY += 0.0005;

            float randX = randFloat(goalX + 0.1, goalX +  + 0.1 + sizeX);
            float randY = randFloat(goalY - sizeY, goalY + sizeY);

            pose_6 objectCurrentPose;
            pose_6 newObjectPose;

            activePhysicsSimulator->getBodyPose_angle("obstacle5", objectCurrentPose, MASTER_RESET_DATA);
            newObjectPose = objectCurrentPose;
            newObjectPose.position(0) = randX;
            newObjectPose.position(1) = randY;
            newObjectPose.position(2) = objectCurrentPose.position(2);
            activePhysicsSimulator->setBodyPose_angle("obstacle5", newObjectPose, MAIN_DATA_STATE);
            activePhysicsSimulator->setBodyPose_angle("obstacle5", newObjectPose, MASTER_RESET_DATA);

            if(activePhysicsSimulator->checkBodyForCollisions("obstacle5", MAIN_DATA_STATE)){
                cout << "first object invalid placement : " << randX << ", " << randY << endl;
            }
            else{
                activePhysicsSimulator->forwardSimulator(MAIN_DATA_STATE);
                activePhysicsSimulator->forwardSimulator(MASTER_RESET_DATA);
                validPlacement = true;
                objectXPos.push_back(randX);
                objectYPos.push_back(randY);
            }
        }

        std::string objectNames[6] = {"mediumCylinder", "bigBox", "obstacle1","obstacle2", "obstacle3", "obstacle4"};

        for(int i = 0; i < 6; i++){
            bool validPlacement = false;
            float sizeX = 0.08;
            float sizeY = 0.04;
            while(!validPlacement){
                sizeX += 0.001;
                sizeY += 0.0005;

                float randX = randFloat(goalX - sizeX, goalX + (0.5 * sizeX));
                float randY = randFloat(goalY - sizeY, goalY + sizeY);

                pose_6 objectCurrentPose;
                pose_6 newObjectPose;

                activePhysicsSimulator->getBodyPose_angle(objectNames[i], objectCurrentPose, MASTER_RESET_DATA);
                newObjectPose = objectCurrentPose;
                newObjectPose.position(0) = randX;
                newObjectPose.position(1) = randY;
                newObjectPose.position(2) = objectCurrentPose.position(2);
                activePhysicsSimulator->setBodyPose_angle(objectNames[i], newObjectPose, MAIN_DATA_STATE);
                activePhysicsSimulator->setBodyPose_angle(objectNames[i], newObjectPose, MASTER_RESET_DATA);

                if(activePhysicsSimulator->checkBodyForCollisions(objectNames[i], MAIN_DATA_STATE)){
                    cout << "invalid placement at : " << randX << ", " << randY << endl;
                }
                else{
                    activePhysicsSimulator->forwardSimulator(MAIN_DATA_STATE);
                    activePhysicsSimulator->forwardSimulator(MASTER_RESET_DATA);
                    validPlacement = true;
                    objectXPos.push_back(randX);
                    objectYPos.push_back(randY);
                }
            }
        }

        randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                startX, startY, objectXPos[0], objectYPos[0], objectXPos[1], objectYPos[1],
                objectXPos[2], objectYPos[2], objectXPos[3], objectYPos[3],
                objectXPos[4], objectYPos[4], objectXPos[5], objectYPos[5], objectXPos[6], objectYPos[6],
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0, 0, 0;
    }

    return randomStartState;
}

MatrixXd twoDPushing::returnRandomGoalState(MatrixXd X0){
    MatrixXd randomGoalState(stateVectorSize, 1);

    if(clutterLevel == noClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                randomGoalX, randomGoalY,
                0, 0, 0, 0, 0, 0, 0,
                0, 0;
    }
    else if(clutterLevel == lowClutter || clutterLevel == constrainedClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                randomGoalX, randomGoalY, X0(9), X0(10), X0(11), X0(12), X0(13), X0(14),
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0;
    }
    else if(clutterLevel == heavyClutter){
        randomGoalState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
                randomGoalX, randomGoalY, X0(9), X0(10), X0(11), X0(12),
                X0(13), X0(14), X0(15), X0(16),
                X0(17), X0(18), X0(19), X0(20), X0(21), X0(22),
                0, 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0,
                0, 0, 0, 0, 0, 0;

    }


    return randomGoalState;
}

std::vector<MatrixXd> twoDPushing::createInitSetupControls(int horizonLength){
    std::vector<MatrixXd> initSetupControls;

    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    activePhysicsSimulator->forwardSimulator(MAIN_DATA_STATE);

    // Pushing create init controls borken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goalPos(0) = X_desired(7);
    goalPos(1) = X_desired(8);
    initControls_mainWayPoints_setup(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << "setup mainwaypoint 0: " << mainWayPoints[0] << endl;
//    cout << "setup mainWayPoint 1: " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = initControls_createAllWayPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initSetupControls = generate_initControls_fromWayPoints(allWayPoints);

    return initSetupControls;
}

void twoDPushing::initControls_mainWayPoints_setup(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon){
    const std::string goalObject = "blueTin";
    const std::string EE_name = "franka_gripper";
//    const std::string EE_name = "hand";

    pose_6 EE_startPose;
    pose_6 goalobj_startPose;
    activePhysicsSimulator->getBodyPose_angle(EE_name, EE_startPose, MAIN_DATA_STATE);
    activePhysicsSimulator->getBodyPose_angle(goalObject, goalobj_startPose, MAIN_DATA_STATE);
//    cout << "goalobj_startPose: " << goalobj_startPose.position(0) << ", " << goalobj_startPose.position(1) << endl;

    m_point mainWayPoint;
    // First waypoint - where the end-effector is currently
    mainWayPoint << EE_startPose.position(0), EE_startPose.position(1), EE_startPose.position(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    // Calculate the angle of approach - from goal position to object start position
    float angle_EE_push;
    float x_diff = X_desired(7) - goalobj_startPose.position(0);
    float y_diff = X_desired(8) - goalobj_startPose.position(1);
    angle_EE_push = atan2(y_diff, x_diff);

    // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
//    float cylinder_radius = 0.08;
    float cylinder_radius = 0.05;
    float x_cylinder0ffset = cylinder_radius * cos(angle_EE_push);
    float y_cylinder0ffset = cylinder_radius * sin(angle_EE_push);

    float endPointX;
    float endPointY;
    float intermediatePointY = goalobj_startPose.position(1);
    float intermediatePointX = goalobj_startPose.position(0);

    intermediatePointX = intermediatePointX - 0.05*cos(angle_EE_push);
    intermediatePointY = intermediatePointY - 0.05*sin(angle_EE_push);
//    if(desiredObjectEnd(1) - goalobj_startPose.position(1) > 0){
//        intermediatePointY = intermediatePointY + y_cylinder0ffset;
//    }
//    else{
//        intermediatePointY = intermediatePointY - y_cylinder0ffset;
//    }

    mainWayPoint(0) = intermediatePointX;
    mainWayPoint(1) = intermediatePointY;
    mainWayPoint(2) = 0.25f;
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(horizon - 1);

}

std::vector<MatrixXd> twoDPushing::createInitOptimisationControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    displayBodyPose.position[0] = X_desired(7);
    displayBodyPose.position[1] = X_desired(8);
    displayBodyPose.position[2] = 0.0f;
    activePhysicsSimulator->setBodyPose_angle(goalMarkerName, displayBodyPose, MASTER_RESET_DATA);

    // Pushing create init controls broken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goalPos(0) = X_desired(7);
    goalPos(1) = X_desired(8);
    initControls_mainWayPoints_optimisation(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
//    cout << mainWayPoints.size() << " waypoints created" << endl;
//    cout << "mainwaypoint 0: " << mainWayPoints[1] << endl;
//    cout << "mainWayPoint 1: " << mainWayPoints[2] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = initControls_createAllWayPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initControls = generate_initControls_fromWayPoints(allWayPoints);

    return initControls;
}

void twoDPushing::initControls_mainWayPoints_optimisation(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon){
    const std::string goalObject = "blueTin";
    const std::string EE_name = "franka_gripper";
//    const std::string EE_name = "hand";

    pose_6 EE_startPose;
    pose_6 goalobj_startPose;
    activePhysicsSimulator->getBodyPose_angle(EE_name, EE_startPose, MAIN_DATA_STATE);
    activePhysicsSimulator->getBodyPose_angle(goalObject, goalobj_startPose, MAIN_DATA_STATE);

    m_point mainWayPoint;
    // First waypoint - where the end-effector is currently
    mainWayPoint << EE_startPose.position(0), EE_startPose.position(1), EE_startPose.position(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    // Calculate the angle of approach - from goal position to object start position
    float angle_EE_push;
    float x_diff = X_desired(7) - goalobj_startPose.position(0);
    float y_diff = X_desired(8) - goalobj_startPose.position(1);
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
//     intermediatePoint(0) = intermediatePointX;
//     intermediatePoint(1) = intermediatePointY;

//    mainWayPoint(0) = intermediatePointX;
//    mainWayPoint(1) = intermediatePointY;
//    mainWayPoint(2) = 0.25f;
//    mainWayPoints.push_back(mainWayPoint);
//    wayPointsTiming.push_back(3 * horizon / 4);

    float maxDistTravelled = 0.05 * ((5.0f/6.0f) * horizon * activePhysicsSimulator->returnModelTimeStep());
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
    mainWayPoint(2) = 0.25f;

    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(horizon - 1);
}

std::vector<m_point> twoDPushing::initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming){
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

std::vector<MatrixXd> twoDPushing::generate_initControls_fromWayPoints(std::vector<m_point> initPath){
    std::vector<MatrixXd> initControls;
    std::string goalObjName = "blueTin";
    std::string EEName = "franka_gripper";

    pose_7 EE_start_pose;
    pose_6 goalobj_startPose;
    activePhysicsSimulator->getBodyPose_quat(EEName, EE_start_pose, MAIN_DATA_STATE);
    activePhysicsSimulator->getBodyPose_angle(goalObjName, goalobj_startPose, MAIN_DATA_STATE);

    float angle_EE_push;
    float x_diff = X_desired(7) - goalobj_startPose.position(0);
    float y_diff = X_desired(8) - goalobj_startPose.position(1);
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
    if(myStateVector.robots[0].torqueControlled){
        MatrixXd robotPos = returnPositionVector(MAIN_DATA_STATE);
        for(int i = 0; i < num_ctrl; i++){
            currentControl(i) = robotPos(i);
        }
    }

    for(int i = 0; i < initPath.size(); i++){
        pose_7 currentEEPose;
        activePhysicsSimulator->getBodyPose_quat(EEName, currentEEPose, MAIN_DATA_STATE);
        m_quat currentEEQuat, invertedQuat, quatDiff;
        currentEEQuat(0) = currentEEPose.quat(0);
        currentEEQuat(1) = currentEEPose.quat(1);
        currentEEQuat(2) = currentEEPose.quat(2);
        currentEEQuat(3) = currentEEPose.quat(3);
        invertedQuat = invQuat(currentEEQuat);
        quatDiff = multQuat(desiredQuat, invertedQuat);

        
        m_point axisDiff = quat2Axis(quatDiff);
        MatrixXd differenceFromPath(6, 1);
        float gainsTorque[6] = {100, 100, 200, 100, 100, 100};
        float gainsPositionControl[6] = {10000, 10000, 30000, 5000, 5000, 5000};

        for(int j = 0; j < 3; j++){
            differenceFromPath(j) = initPath[i](j) - currentEEPose.position(j);
            differenceFromPath(j + 3) = axisDiff(j);
        }

        MatrixXd Jac, JacInv;

        Jac = activePhysicsSimulator->calculateJacobian(EEName, MAIN_DATA_STATE);
        JacInv = Jac.completeOrthogonalDecomposition().pseudoInverse();

        MatrixXd desiredEEForce(6, 1);
        MatrixXd desiredControls(num_ctrl, 1);

        if(myStateVector.robots[0].torqueControlled){
            for(int j = 0; j < 6; j++) {
                desiredEEForce(j) = differenceFromPath(j) * gainsTorque[j];
            }
            desiredControls = JacInv * desiredEEForce;

            std::vector<double> gravCompensation;
            MatrixXd gravCompControl(num_ctrl, 1);
            activePhysicsSimulator->getRobotJointsGravityCompensaionControls(myStateVector.robots[0].name, gravCompensation, MAIN_DATA_STATE);
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

        setControlVector(desiredControls, MAIN_DATA_STATE);
        activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

    }

    return initControls;
}

bool twoDPushing::taskComplete(int dataIndex, double &dist){
    bool taskComplete = false;

    MatrixXd currentState = returnStateVector(dataIndex);

    float x_diff = currentState(7) - X_desired(7);
    float y_diff = currentState(8) - X_desired(8);

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.035){
        taskComplete = true;
    }


    return taskComplete;
}