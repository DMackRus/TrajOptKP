#include "twoDPushing.h"

twoDPushing::twoDPushing(){
    std::string yamlFilePath = "/home/davidrussell/catkin_ws/src/autoTOTask/taskConfigs/twoDPushingConfig.yaml";

    initModelTranslator(yamlFilePath);
    analyticalCostDerivatives = true;

    X_desired << 1, 1.5, 2, -2, 0, 0.6, 1, 
                 0.8, 0,
                 0, 0, 0, 0, 0, 0, 0,
                 0, 0;

}

MatrixXd twoDPushing::returnRandomStartState(){
    MatrixXd randomStartState(stateVectorSize, 1);

    randomStartState << 1, 1.5, 2, -2, 0, 0.6, 1, 
                        0.6, 0,
                        0, 0, 0, 0, 0, 0, 0,
                        0, 0;

    return randomStartState;
}

MatrixXd twoDPushing::returnRandomGoalState(){
    MatrixXd randomGoalState(stateVectorSize, 1);

    return randomGoalState;
}

std::vector<MatrixXd> twoDPushing::createInitControls(int horizonLength){

    std::vector<MatrixXd> initControls;

    if(myStateVector.robots[0].torqueControlled){

        MatrixXd control(num_ctrl, 1);
        vector<double> gravCompensation;
        for(int i = 0; i < horizonLength; i++){

            activePhysicsSimulator->getRobotJointsGravityCompensaionControls(myStateVector.robots[0].name, gravCompensation, MAIN_DATA_STATE);
            for(int i = 0; i < num_ctrl; i++){
                control(i) = gravCompensation[i];
            }
            initControls.push_back(control);
        }
    }
    return initControls;

}

// // Generate initial controls to be optimised
// std::vector<m_ctrl> taskTranslator::initOptimisationControls(mjData *d, mjData *d_init) {

//     std::vector<m_ctrl> initControls;
//     cpMjData(model, d, d_init);

//     std::vector<m_point> mainWayPoints;
//     std::vector<int> wayPoints_timings;

//     m_state X0 = returnState(d_init);

//     m_point desiredObjectEnd;
//     desiredObjectEnd(0) = X_desired(7);
//     desiredObjectEnd(1) = X_desired(8);
//     m_point objectStart;
//     objectStart(0) = X0(7);
//     objectStart(1) = X0(8);

//     double angle_EE_push = atan2(desiredObjectEnd(1) - objectStart(1), desiredObjectEnd(0) - objectStart(0));
//     cout << "angle_EE_push: " << angle_EE_push << endl;

//     initControls_MainWayPoints_Optimise(d, model, desiredObjectEnd, angle_EE_push, mainWayPoints, wayPoints_timings);
//     std::vector<m_point> initPath = initControls_createAllWayPoints(mainWayPoints, wayPoints_timings);

//     initControls = initControls_generateAllControls(d, model, initPath, angle_EE_push);

//     return initControls;
// }

// void initControls_MainWayPoints_Optimise(mjData *d, mjModel *model, m_point desiredObjectEnd, double angle_EE_push, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming){
//     const std::string goalName = "goal";

//     const std::string EE_name = "franka_gripper";

//     int EE_id = mj_name2id(model, mjOBJ_BODY, EE_name.c_str());
//     int goal_id = mj_name2id(model, mjOBJ_BODY, goalName.c_str());

//     m_pose startPose = globalMujocoController->returnBodyPose(model, d, EE_id);

//     m_point mainWayPoint;
//     mainWayPoint << startPose(0), startPose(1), startPose(2);
//     mainWayPoints.push_back(mainWayPoint);
//     wayPointsTiming.push_back(0);

//     // TODO hard coded - get it programmatically? - also made it slightly bigger so trajectory has room to improve
// //    float cylinder_radius = 0.08;
//     float cylinder_radius = 0.1;
//     float x_cylinder0ffset = cylinder_radius * cos(angle_EE_push);
//     float y_cylinder0ffset = cylinder_radius * sin(angle_EE_push);

//     float desired_endPointX = desiredObjectEnd(0) - x_cylinder0ffset;
//     float desired_endPointY;

//     float endPointX;
//     float endPointY;
//     if(desiredObjectEnd(1) - startPose(1) > 0){
//         desired_endPointY = desiredObjectEnd(1) + y_cylinder0ffset;
//     }
//     else{
//         desired_endPointY = desiredObjectEnd(1) - y_cylinder0ffset;
//     }

//     float intermediatePointY = startPose(1);
//     float intermediatePointX = startPose(0);

//     // Setting this up so we can visualise where the intermediate point is located
//     intermediatePoint(0) = intermediatePointX;
//     intermediatePoint(1) = intermediatePointY;

//     float maxDistTravelled = 0.05 * ((5.0f/6.0f) * MUJ_STEPS_HORIZON_LENGTH * MUJOCO_DT);
// //    cout << "max EE travel dist: " << maxDistTravelled << endl;
//     float desiredDistTravelled = sqrt(pow((desired_endPointX - intermediatePointX),2) + pow((desired_endPointY - intermediatePointY),2));
//     float proportionOfDistTravelled = maxDistTravelled / desiredDistTravelled;
// //    cout << "proportion" << proportionOfDistTravelled << endl;
//     if(proportionOfDistTravelled > 1){
//         endPointX = desired_endPointX;
//         endPointY = desired_endPointY;
//     }
//     else{
//         endPointX = intermediatePointX + ((desired_endPointX - intermediatePointX) * proportionOfDistTravelled);
//         endPointY = intermediatePointY + ((desired_endPointY - intermediatePointY) * proportionOfDistTravelled);
//     }

//     mainWayPoint(0) = endPointX;
//     mainWayPoint(1) = endPointY;
//     mainWayPoint(2) = 0.27f;

//     mainWayPoints.push_back(mainWayPoint);
//     wayPointsTiming.push_back(2000);

// }

// std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming){
//     int numMainWayPoints = mainWayPoints.size();
//     std::vector<m_point> initPath;

//     initPath.push_back(mainWayPoints[0]);
//     wayPointsTiming[0]--;

//     // should only be MUJ_STEPS_HORIZON_LENGTH number of controls
//     int counter = 1;
//     for(int i = 0; i < numMainWayPoints - 1; i++){
//         float x_diff = mainWayPoints[i + 1](0) - mainWayPoints[i](0);
//         float y_diff = mainWayPoints[i + 1](1) - mainWayPoints[i](1);
//         float z_diff = mainWayPoints[i + 1](2) - mainWayPoints[i](2);
//         for(int j = 0; j < wayPointsTiming[i + 1]; j++){
//             initPath.push_back(m_point());
//             initPath[counter](0) = initPath[counter - 1](0) + (x_diff / wayPointsTiming[i + 1]);
//             initPath[counter](1) = initPath[counter - 1](1) + (y_diff / wayPointsTiming[i + 1]);
//             initPath[counter](2) = initPath[counter - 1](2) + (z_diff / wayPointsTiming[i + 1]);

//             counter++;
//             if(counter > MUJ_STEPS_HORIZON_LENGTH){
//                 cout << "ERROR, TOO MANY POINTS IN INIT PATH" << endl;
//             }
//         }
//     }

//     return initPath;
// }

// std::vector<m_ctrl> initControls_generateAllControls(mjData *d, mjModel *model, std::vector<m_point> initPath, double angle_EE_push){
//     std::vector<m_ctrl> initControls;

//     taskTranslator tempModelTranslator;
//     tempModelTranslator.init(model);
//     int EE_id = mj_name2id(model, mjOBJ_BODY, tempModelTranslator.EE_name.c_str());
//     m_quat startQuat = globalMujocoController->returnBodyQuat(model, d, EE_id);

//     m_pose startPose = globalMujocoController->returnBodyPose(model, d, EE_id);
//     globalMujocoController->quat2RotMat(startQuat);


// //    cout << "converted angle is: " << convertedAngle << endl;
//     if(angle_EE_push < 0){
//         angle_EE_push = angle_EE_push + (2*PI);
// //        cout << "converted angle is: " << convertedAngle << endl;
//     }

//     double convertedAngle = angle_EE_push - (PI/4);


//     m_point xAxis, yAxis, zAxis;
//     xAxis << cos(convertedAngle), sin(convertedAngle), 0;
//     zAxis << 0, 0, -1;
//     yAxis = globalMujocoController->crossProduct(zAxis, xAxis);



//     Eigen::Matrix3d rotMat;
//     rotMat << xAxis(0), yAxis(0), zAxis(0),
//             xAxis(1), yAxis(1), zAxis(1),
//             xAxis(2), yAxis(2), zAxis(2);

//     m_quat desiredQuat = globalMujocoController->rotMat2Quat(rotMat);
//     //cout << "angle EE push is: " << angle_EE_push << endl;
//     //cout << "x contribution:" << cos(convertedAngle) << "y contribution: " << sin(convertedAngle) << endl;
//     //cout << "yAxis: " << yAxis << endl;

//     //cout << "desired quat: " << desiredQuat << endl;

//     m_ctrl desiredControls;

//     // Calculate the initial position control to be the starting position
//     if(!TORQUE_CONTROL){
//         m_state startState = tempModelTranslator.returnState(d);
//         for(int i = 0; i < NUM_CTRL; i++){
//             desiredControls(i) = startState(i);
//         }
//     }

//     for (int i = 0; i < initPath.size(); i++) {

//         m_pose currentEEPose = globalMujocoController->returnBodyPose(model, d, EE_id);
//         m_quat currentQuat = globalMujocoController->returnBodyQuat(model, d, EE_id);
//         m_quat invQuat = globalMujocoController->invQuat(currentQuat);
//         m_quat quatDiff = globalMujocoController->multQuat(desiredQuat, invQuat);

//         m_point axisDiff = globalMujocoController->quat2Axis(quatDiff);

//         m_pose differenceFromPath;
//         float gains[6] = {10000, 10000, 30000, 5000, 5000, 5000};
//         for (int j = 0; j < 3; j++) {
//             differenceFromPath(j) = initPath[i](j) - currentEEPose(j);
//             differenceFromPath(j + 3) = axisDiff(j);
//         }

//         // Calculate jacobian inverse
//         MatrixXd Jac = globalMujocoController->calculateJacobian(model, d, EE_id);
//         MatrixXd Jac_inv = Jac.completeOrthogonalDecomposition().pseudoInverse();

//         m_pose desiredEEForce;

//         if(TORQUE_CONTROL){
//             for (int j = 0; j < 6; j++) {
//                 desiredEEForce(j) = differenceFromPath(j) * gains[j];
//             }
//             desiredControls = Jac_inv * desiredEEForce;
//         }
//         else{
//             // Position control
//             for (int j = 0; j < 6; j++) {
//                 desiredEEForce(j) = differenceFromPath(j) * gains[j] * 0.000001;
//             }
//             desiredControls += Jac_inv * desiredEEForce;
//             //cout << "desired controls: " << desiredControls << endl;
//         }


//         //cout << "desiredEEForce " << desiredEEForce << endl;


//         initControls.push_back(m_ctrl());


//         for (int k = 0; k < NUM_CTRL; k++) {

//             initControls[i](k) = desiredControls(k);
//         }

//         tempModelTranslator.setControls(d, initControls[i], false);

//         tempModelTranslator.stepModel(d, 1);

//     }

//     return initControls;
// }