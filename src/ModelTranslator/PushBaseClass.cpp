#include "ModelTranslator/PushBaseClass.h"

PushBaseClass::PushBaseClass(std::string EE_name, std::string body_name){
    this->EE_name = EE_name;
    this->body_name = body_name;
}

void PushBaseClass::EEWayPointsSetup(m_point desiredObjectEnd,
                                     std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon){

    pose_6 EE_startPose;
    pose_6 goalobj_startPose;
    MuJoCo_helper->GetBodyPoseAngleViaXpos(EE_name, EE_startPose, MuJoCo_helper->main_data);
    MuJoCo_helper->GetBodyPoseAngle(body_name, goalobj_startPose, MuJoCo_helper->main_data);

    m_point mainWayPoint;
    // First waypoint - where the end-effector is currently
    mainWayPoint << EE_startPose.position(0), EE_startPose.position(1), EE_startPose.position(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    // Calculate the angle of approach - from goal position to object start position
    double angle_EE_push;
    double x_diff = desiredObjectEnd(0) - goalobj_startPose.position(0);
    double y_diff = desiredObjectEnd(1) - goalobj_startPose.position(1);
    angle_EE_push = atan2(y_diff, x_diff);

    double intermediatePointY = goalobj_startPose.position(1);
    double intermediatePointX = goalobj_startPose.position(0);

    // Place EE behind the object, this might need reworking slightly
    intermediatePointX = intermediatePointX - 0.05*cos(angle_EE_push);
    intermediatePointY = intermediatePointY - 0.05*sin(angle_EE_push);

    mainWayPoint(0) = intermediatePointX;
    mainWayPoint(1) = intermediatePointY;
    // TODO - not great this is hardcoded
    mainWayPoint(2) = 0.28f;

    // Push the waypoint and time it should happen at
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(horizon - 1);
}

void PushBaseClass::EEWayPointsPush(m_point desiredObjectEnd,
                                    std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon){

    pose_6 EE_startPose;
    pose_6 goalobj_startPose;
    MuJoCo_helper->GetBodyPoseAngleViaXpos(EE_name, EE_startPose, MuJoCo_helper->main_data);
    MuJoCo_helper->GetBodyPoseAngle(body_name, goalobj_startPose, MuJoCo_helper->main_data);

    m_point mainWayPoint;
    // First waypoint - where the end-effector is currently
    mainWayPoint << EE_startPose.position(0), EE_startPose.position(1), EE_startPose.position(2);
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(0);

    // Calculate the angle of approach - from goal position to object start position
    double angle_EE_push;
    double x_diff = desiredObjectEnd(0) - goalobj_startPose.position(0);
    double y_diff = desiredObjectEnd(1) - goalobj_startPose.position(1);
//    double x_diff = desiredObjectEnd(0) - EE_startPose.position(0);
//    double y_diff = desiredObjectEnd(1) - EE_startPose.position(1);
    angle_EE_push = atan2(y_diff, x_diff);

    // TODO hard coded - get it programmatically?
    double cylinder_radius = 0.01;
    double x_cylinder0ffset = cylinder_radius * cos(angle_EE_push);
    double y_cylinder0ffset = cylinder_radius * sin(angle_EE_push);

    double desired_endPointX = desiredObjectEnd(0) - x_cylinder0ffset;
    double desired_endPointY;

    double endPointX;
    double endPointY;
    if(desiredObjectEnd(1) - goalobj_startPose.position(1) > 0){
        desired_endPointY = desiredObjectEnd(1) + y_cylinder0ffset;
    }
    else{
        desired_endPointY = desiredObjectEnd(1) - y_cylinder0ffset;
    }

    double intermediatePointY = goalobj_startPose.position(1);
    double intermediatePointX = goalobj_startPose.position(0);
//    double intermediatePointY = EE_startPose.position(1);
//    double intermediatePointX = EE_startPose.position(0);

    // Max speed could be a parameter
    double maxDistTravelled = 0.1 * ((5.0f/6.0f) * horizon * MuJoCo_helper->ReturnModelTimeStep());
    // float maxDistTravelled = 0.05 * ((5.0f/6.0f) * horizon * MUJOCO_DT);
//    cout << "max EE travel dist: " << maxDistTravelled << endl;
    double desiredDistTravelled = sqrt(pow((desired_endPointX - intermediatePointX),2) + pow((desired_endPointY - intermediatePointY),2));
    double proportionOfDistTravelled = maxDistTravelled / desiredDistTravelled;
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
    // TODO not great this is hard coded
    mainWayPoint(2) = 0.28f;

    // Push the waypoint and when it should occur
    mainWayPoints.push_back(mainWayPoint);
    wayPointsTiming.push_back(horizon - 1);
}

std::vector<m_point> PushBaseClass::CreateAllEETransitPoints(const std::vector<m_point> &mainWayPoints, const std::vector<int> &wayPointsTiming){
    std::vector<m_point> EE_path;

    EE_path.push_back(mainWayPoints[0]);

    int path_counter = 1;
    for(int i = 0; i < mainWayPoints.size(); i++){
        double x_diff = mainWayPoints[i + 1](0) - mainWayPoints[i](0);
        double y_diff = mainWayPoints[i + 1](1) - mainWayPoints[i](1);
        double z_diff = mainWayPoints[i + 1](2) - mainWayPoints[i](2);
        for(int j = 0; j < wayPointsTiming[i + 1]; j++){
            m_point path_point;
            path_point(0) = EE_path[path_counter - 1](0) + (x_diff / wayPointsTiming[i + 1]);
            path_point(1) = EE_path[path_counter - 1](1) + (y_diff / wayPointsTiming[i + 1]);
            path_point(2) = EE_path[path_counter - 1](2) + (z_diff / wayPointsTiming[i + 1]);
            EE_path.push_back(path_point);

            path_counter++;
        }
    }

    return EE_path;
}

std::vector<MatrixXd> PushBaseClass::JacobianEEControl(const std::vector<m_point> &EE_path, double EE_angle){
    std::vector<MatrixXd> init_controls;

    // Aliases
    int num_ctrl = current_state_vector.num_ctrl;

    pose_7 EE_start_pose;
    MuJoCo_helper->GetBodyPoseQuatViaXpos(EE_name, EE_start_pose, MuJoCo_helper->main_data);

    EE_angle -= (PI / 4);

    if(EE_angle < -(PI/2)){
        EE_angle = (2 * PI) + EE_angle;
    }

    double convertedAngle = EE_angle;

    // Setup the desired rotation matrix for the end-effector
    m_point xAxis, yAxis, zAxis;
    xAxis << cos(convertedAngle), sin(convertedAngle), 0;
    zAxis << 0, 0, -1;
    yAxis = crossProduct(zAxis, xAxis);

    // End effector parallel to table
    // xAxis << 0, 0, -1;
    // zAxis << cos(convertedAngle), sin(convertedAngle), 0;
    // yAxis = crossProduct(zAxis, xAxis);

    Eigen::Matrix3d rotMat;
    rotMat << xAxis(0), yAxis(0), zAxis(0),
            xAxis(1), yAxis(1), zAxis(1),
            xAxis(2), yAxis(2), zAxis(2);

    m_quat desiredQuat = rotMat2Quat(rotMat);

    MatrixXd currentControl(num_ctrl, 1);

    bool quaternion_check = false;

    for(const auto & i : EE_path){
        pose_7 currentEEPose;
        MuJoCo_helper->GetBodyPoseQuatViaXpos(EE_name, currentEEPose, MuJoCo_helper->main_data);
        m_quat currentEEQuat, invertedQuat, quatDiff;
        currentEEQuat(0) = currentEEPose.quat(0);
        currentEEQuat(1) = currentEEPose.quat(1);
        currentEEQuat(2) = currentEEPose.quat(2);
        currentEEQuat(3) = currentEEPose.quat(3);

        if(!quaternion_check){
            quaternion_check = true;
            // calculate dot produce between quaternios
            double dotProduct = 0;
            for(int j = 0; j < 4; j++){
                dotProduct += currentEEQuat(j) * desiredQuat(j);
            }

            // Invert the quaternion
            if(dotProduct < 0){
                desiredQuat(0) = -desiredQuat(0);
                desiredQuat(1) = -desiredQuat(1);
                desiredQuat(2) = -desiredQuat(2);
                desiredQuat(3) = -desiredQuat(3);
            }
        }
        invertedQuat = invQuat(currentEEQuat);
        quatDiff = multQuat(desiredQuat, invertedQuat);


        m_point axisDiff = quat2Axis(quatDiff);
        MatrixXd differenceFromPath(6, 1);
        float gainsTorque[6] = {100, 100, 200, 80, 80, 80};
        float gainsPositionControl[6] = {10000, 10000, 30000, 5000, 5000, 5000};

        for(int j = 0; j < 3; j++){
            differenceFromPath(j) = i(j) - currentEEPose.position(j);
            differenceFromPath(j + 3) = axisDiff(j);
        }

        MatrixXd Jac, JacInv;

        Jac = MuJoCo_helper->GetJacobian(EE_name, MuJoCo_helper->main_data);
        JacInv = Jac.completeOrthogonalDecomposition().pseudoInverse();

        MatrixXd desiredEEForce(6, 1);
        MatrixXd desiredControls(num_ctrl, 1);

        for(int j = 0; j < 6; j++) {
            desiredEEForce(j) = differenceFromPath(j) * gainsTorque[j];
        }
        desiredControls = JacInv * desiredEEForce;

        std::vector<double> gravCompensation;
        MatrixXd gravCompControl(num_ctrl, 1);
        MuJoCo_helper->GetRobotJointsGravityCompensationControls(current_state_vector.robots[0].name, gravCompensation, MuJoCo_helper->main_data);
        for(int j = 0; j < num_ctrl; j++){
            gravCompControl(j) = gravCompensation[j];
        }
        desiredControls += gravCompControl;



        init_controls.push_back(desiredControls);

        SetControlVector(desiredControls, MuJoCo_helper->main_data, current_state_vector);
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

    }

    return init_controls;
}
