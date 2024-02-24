#include "BoxSweep.h"

BoxSweep::BoxSweep(){

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

    double upperBoundX = 0.7;
    double lowerBoundX = 0.6;
    double upperBoundY = 0.5;
    double lowerBoundY = 0.3;

    double randX = randFloat(lowerBoundX, upperBoundX);
    double randY = randFloat(lowerBoundY, upperBoundY);

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
    std::vector<MatrixXd> initControls;

    // Set the goal position so that we can see where we are pushing to
    std::string goalMarkerName = "display_goal";
    pose_6 displayBodyPose;
    displayBodyPose.position[0] = X_desired(7);
    displayBodyPose.position[1] = X_desired(8);
    displayBodyPose.position[2] = 0.0f;
    MuJoCo_helper->setBodyPose_angle(goalMarkerName, displayBodyPose, MuJoCo_helper->master_reset_data);

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

    return initControls;
}

void BoxSweep::initControls_mainWayPoints_optimisation(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon){
    const std::string goalObject = "bigBox";
    const std::string EE_name = "franka_gripper";

    pose_6 EE_startPose;
    pose_6 goalobj_startPose;
    MuJoCo_helper->getBodyPose_angle_ViaXpos(EE_name, EE_startPose, MuJoCo_helper->main_data);
    MuJoCo_helper->getBodyPose_angle(goalObject, goalobj_startPose, MuJoCo_helper->main_data);

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

    float maxDistTravelled = 0.01 * ((5.0f/6.0f) * horizon * MuJoCo_helper->returnModelTimeStep());
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

std::vector<m_point> BoxSweep::initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming){
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

std::vector<MatrixXd> BoxSweep::generate_initControls_fromWayPoints(std::vector<m_point> initPath){
    std::vector<MatrixXd> initControls;
    std::string goalObjName = "bigBox";
    std::string EEName = "franka_gripper";

    pose_7 EE_start_pose;
    pose_6 goalobj_startPose;
    MuJoCo_helper->getBodyPose_quat_ViaXpos(EEName, EE_start_pose, MuJoCo_helper->main_data);
    MuJoCo_helper->getBodyPose_angle(goalObjName, goalobj_startPose, MuJoCo_helper->main_data);

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
    if(active_state_vector.robots[0].torqueControlled){
        MatrixXd robotPos = returnPositionVector(MuJoCo_helper->main_data);
        for(int i = 0; i < num_ctrl; i++){
            currentControl(i) = robotPos(i);
        }
    }

    for(int i = 0; i < initPath.size(); i++){
        pose_7 currentEEPose;
        MuJoCo_helper->getBodyPose_quat_ViaXpos(EEName, currentEEPose, MuJoCo_helper->main_data);
        m_quat currentEEQuat, invertedQuat, quatDiff;
        currentEEQuat(0) = currentEEPose.quat(0);
        currentEEQuat(1) = currentEEPose.quat(1);
        currentEEQuat(2) = currentEEPose.quat(2);
        currentEEQuat(3) = currentEEPose.quat(3);
        invertedQuat = invQuat(currentEEQuat);
        quatDiff = multQuat(desiredQuat, invertedQuat);


        m_point axisDiff = quat2Axis(quatDiff);
        MatrixXd differenceFromPath(6, 1);
        float gainsTorque[6] = {20, 20, 40, 100, 100, 100};
        float gainsPositionControl[6] = {10000, 10000, 30000, 5000, 5000, 5000};

        for(int j = 0; j < 3; j++){
            differenceFromPath(j) = initPath[i](j) - currentEEPose.position(j);
            differenceFromPath(j + 3) = axisDiff(j);
        }

        MatrixXd Jac, JacInv;

        Jac = MuJoCo_helper->calculateJacobian(EEName, MuJoCo_helper->main_data);
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
            MuJoCo_helper->getRobotJointsGravityCompensaionControls(active_state_vector.robots[0].name, gravCompensation, MuJoCo_helper->main_data);
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

        SetControlVector(desiredControls, MuJoCo_helper->main_data);
        MuJoCo_helper->stepSimulator(1, MuJoCo_helper->main_data);

    }

    return initControls;
}

bool BoxSweep::TaskComplete(mjData *d, double &dist){
    bool taskComplete = false;

    MatrixXd currentState = ReturnStateVector(d);

    float x_diff = currentState(7) - X_desired(7);
    float y_diff = currentState(8) - X_desired(8);

    dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2));

    if(dist < 0.035){
        taskComplete = true;
    }


    return taskComplete;
}
