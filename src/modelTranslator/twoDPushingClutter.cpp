//
// Created by davidrussell on 3/28/23.
//

#include "twoDPushingClutter.h"

twoDPushingClutter::twoDPushingClutter(){
    std::string yamlFilePath = "/home/davidrussell/catkin_ws/src/autoTOTask/taskConfigs/twoDPushingClutterConfig.yaml";

    initModelTranslator(yamlFilePath);
    analyticalCostDerivatives = true;

    X_desired << 1, 1.5, 2, -2, 0, 0.6, 1,
            0.6, 0.3, 0.6, 0.25, 0.7, 0.3,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;
}

MatrixXd twoDPushingClutter::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    float cubeX = randFloat(0.45, 0.55);
    float cubeY = randFloat(-0.1, 0.1);

    cubeX = 0.5;
    cubeY = 0.2;
    randomStartState << 0, -0.183, 0, -3.1, 0, 1.34, 0,
            cubeX, cubeY, 0.48, 0.3, 0.6, 0.3,
            0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0;

    return randomStartState;
}

MatrixXd twoDPushingClutter::returnRandomGoalState(){
    MatrixXd randomGoalState(stateVectorSize, 1);

    return randomGoalState;
}

std::vector<MatrixXd> twoDPushingClutter::createInitControls(int horizonLength){
    std::vector<MatrixXd> initControls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    displayBodyPose.position[0] = X_desired(7);
    displayBodyPose.position[1] = X_desired(8);
    displayBodyPose.position[2] = 0.0f;
    activePhysicsSimulator->setBodyPose_angle(goalMarkerName, displayBodyPose, 0);

    // Pushing create init controls borken into three main steps
    // Step 1 - create main waypoints we want to end-effector to pass through
    m_point goalPos;
    std::vector<m_point> mainWayPoints;
    std::vector<int> mainWayPointsTimings;
    std::vector<m_point> allWayPoints;
    goalPos(0) = X_desired(7);
    goalPos(1) = X_desired(8);
    initControls_mainWayPoints(goalPos, mainWayPoints, mainWayPointsTimings, horizonLength);
    cout << "mainwaypoint 0: " << mainWayPoints[0] << endl;
    cout << "mainWayPoint " << mainWayPoints[1] << endl;

    // Step 2 - create all subwaypoints over the entire trajectory
    allWayPoints = initControls_createAllWayPoints(mainWayPoints, mainWayPointsTimings);

    // Step 3 - follow the points via the jacobian
    initControls = generate_initControls_fromWayPoints(allWayPoints);

    return initControls;
}

void twoDPushingClutter::initControls_mainWayPoints(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon){
    const std::string goalObject = "blueTin";
    const std::string EE_name = "franka_gripper";

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
    // intermediatePoint(0) = intermediatePointX;
    // intermediatePoint(1) = intermediatePointY;

    float maxDistTravelled = 0.05 * ((5.0f/6.0f) * horizon * MUJOCO_DT);
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
    wayPointsTiming.push_back(horizon);
}

std::vector<m_point> twoDPushingClutter::initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming){
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

std::vector<MatrixXd> twoDPushingClutter::generate_initControls_fromWayPoints(std::vector<m_point> initPath){
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
        float gainsTorque[6] = {100, 100, 100, 500, 500, 500};
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
