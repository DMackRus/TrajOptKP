#include "ModelTranslator/PlaceObject.h"

PlaceObject::PlaceObject(std::string EE_name, std::string body_name){
    this->EE_name = EE_name;
    this->body_name = body_name;

    std::string yamlFilePath = "/TaskConfigs/rigid_body_manipulation/place_single.yaml";
    InitModelTranslator(yamlFilePath);
}

std::vector<MatrixXd> PlaceObject::CreateInitOptimisationControls(int horizonLength) {
    std::vector<MatrixXd> init_opt_controls;
    int num_ctrl = current_state_vector.num_ctrl;
    vector<double> gravCompensation;
    MatrixXd control(num_ctrl, 1);

    for(int i = 0; i < horizonLength; i++){

        MuJoCo_helper->GetRobotJointsGravityCompensationControls(current_state_vector.robots[0].name, gravCompensation, MuJoCo_helper->main_data);

        for(int j = 0; j < num_ctrl; j++){
            control(j) = gravCompensation[j];
        }

        SetControlVector(control, MuJoCo_helper->main_data, full_state_vector);
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);
        init_opt_controls.push_back(control);
    }

    return init_opt_controls;
}

//std::vector<MatrixXd> PlaceObject::CreateInitSetupControls(int horizonLength) {
//
//}

void PlaceObject::Residuals(mjData *d, MatrixXd &residuals) {
    int resid_index = 0;

    // Compute kinematics chain to compute site poses
    mj_kinematics(MuJoCo_helper->model, d);
//    mj_forwardSkip(MuJoCo_helper->model, d, mjSTAGE_NONE, 1);

    pose_7 goal_pose;
    pose_6 goal_velocity;
    pose_6 ee_pose;
    MuJoCo_helper->GetBodyPoseQuat(body_name, goal_pose, d);
    MuJoCo_helper->GetBodyVelocity(body_name, goal_velocity, d);

    int site_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SITE, EE_name.c_str());
    for(int i = 0; i < 3; i++){
        ee_pose.position(i) = d->site_xpos[site_id * 3 + i];
    }

    // --------------- Residual 0: Body goal position x -----------------
    double diff_x = goal_pose.position(0) - residual_list[0].target[0];
    residuals(resid_index++, 0) = diff_x;

    // --------------- Residual 1: Body goal position y -----------------
    double diff_y = goal_pose.position(1) - residual_list[1].target[0];
    residuals(resid_index++, 0) = diff_y;

    // --------------- Residual 2: Body goal position z -----------------
    double diff_z = goal_pose.position(2) - residual_list[2].target[0];
    residuals(resid_index++, 0) = diff_z;

    // ------------- Residual 3: Body orientation upright ---------------

    // Convert quat to eul and eul to rotation matrix
    m_point temp_eul = quat2Eul(goal_pose.quat);
    Eigen::Matrix3d current_rot_mat = eul2RotMat(temp_eul);

    // TODO - Temporary, unit quaternion (z facing down)
    m_quat desired = {0, 0, 1, 0};
    temp_eul = quat2Eul(desired);
    Eigen::Matrix3d desired_rot_mat = eul2RotMat(temp_eul);

    double dot_x, dot_y, dot_z;
    dot_x = acos(current_rot_mat(0, 0) * desired_rot_mat(0, 0)
            + current_rot_mat(1, 0) * desired_rot_mat(1, 0)
            + current_rot_mat(2, 0) * desired_rot_mat(2, 0));
    dot_y = acos(current_rot_mat(0, 1) * desired_rot_mat(0, 1)
            + current_rot_mat(1, 1) * desired_rot_mat(1, 1)
            + current_rot_mat(2, 1) * desired_rot_mat(2, 1));
    dot_z = acos(current_rot_mat(0, 2) * desired_rot_mat(0, 2)
            + current_rot_mat(1, 2) * desired_rot_mat(1, 2)
            + current_rot_mat(2, 2) * desired_rot_mat(2, 2));

//    std::cout << "dot_z: " << dot_z << std::endl;

//    residuals(resid_index++, 0) = pow(dot_z,2);
    residuals(resid_index++, 0) = dot_z;


//    Eigen::Matrix3d desired_rot_mat = eul2RotMat(desired_eul);

//    m_point desired_axis;
//    desired_axis(0) = 0;
//    desired_axis(1) = 0;
//    desired_axis(2) = 0;
//
//    m_quat current, desired, inv_current, diff;
//    current = axis2Quat(goal_pose.orientation);
//    desired = axis2Quat(desired_axis);
//
//    inv_current = invQuat(current);
//    diff = multQuat(inv_current, desired);
//
//    // temp mujoco quat2vel methodology?
//    double axis[3] = {diff[1], diff[2], diff[3]};
//    double sin_a_2 = sin(sqrt(pow(axis[0], 2) + pow(axis[1], 2) + pow(axis[2], 2)));
//    double speed = 2 * atan2(sin_a_2, diff[0]);
//
//    // When axis angle > pi rot is in other direction
//    if(speed > PI) speed -= 2*PI;
//
//    axis[0] *= speed;
//    axis[1] *= speed;
//    axis[2] *= speed;
//
//    m_point axis_diff = quat2Axis(diff);


//    std::cout << "axis_diff: " << axis_diff(0) << ", " << axis_diff(1) << ", " << axis_diff(2) << std::endl;

//    residuals(resid_index++, 0) = acos(dot_x);
//    residuals(resid_index++, 0) = axis_diff(2); // Was axis_diff(2) before

    // Residual 4: Grasped object velocity
    pose_6 body_vel;
    MuJoCo_helper->GetBodyVelocity(body_name, body_vel, d);
    double total_vel = sqrt(pow(body_vel.position(0),2)
            + pow(body_vel.position(1),2) + pow(body_vel.position(2),2));
//    std::cout << "total vel: " << total_vel << std::endl;
    residuals(resid_index++, 0) = total_vel;

    if(resid_index != residual_list.size()){
        std::cerr << "Error: Residuals size mismatch\n";
        exit(1);
    }
}

bool PlaceObject::TaskComplete(mjData *d, double &dist) {

    // Get the pose of the goal object
    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle(body_name, goal_pose, d);

    // Compute distance to the target
    double diffx, diffy, diffz;
    diffx = goal_pose.position(0) - residual_list[0].target[0];
    diffy = goal_pose.position(1) - residual_list[1].target[0];
    diffz = goal_pose.position(2) - residual_list[2].target[0];

    dist = sqrt(pow(diffx,2) + pow(diffy,2) + pow(diffz,2));
//    std::cout << "dist: " << dist << "\n";

    if (dist < 0.015){
        return true;
    }

    return false;
}

void PlaceObject::SetGoalVisuals(mjData *d) {
    pose_6 goal_pose;
    MuJoCo_helper->GetBodyPoseAngle("target", goal_pose, d);

    // Set the goal object position
    goal_pose.position(0) = residual_list[0].target[0];
    goal_pose.position(1) = residual_list[1].target[0];
    goal_pose.position(2) = residual_list[2].target[0];
    MuJoCo_helper->SetBodyPoseAngle("target", goal_pose, d);

    // TODO - not sure about this
    // Activate pump adhesion - Pump adhesion is not a decision variable currently in optimisation
//    int pump_id = mj_name2id(MuJoCo_helper->model, mjOBJ_ACTUATOR, "adhere_pump");
//    d->ctrl[pump_id] = 5.0;
}