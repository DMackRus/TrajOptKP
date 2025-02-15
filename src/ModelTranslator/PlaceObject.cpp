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
//    mj_sensorVel(MuJoCo_helper->model, d);
//    mj_fwdVelocity(MuJoCo_helper->model, d);
//    mj_forwardSkip(MuJoCo_helper->model, d, mjSTAGE_NONE, 1);


    // TODO - new dea using sensors
    mj_sensorPos(MuJoCo_helper->model, d);
    mj_sensorVel(MuJoCo_helper->model, d);
    int site_pos_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SENSOR, "site_position_sensor");
    int site_quat_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SENSOR, "site_orientation_sensor");
    int site_velp_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SENSOR, "site_linear_velocity_sensor");
//    int site_velr_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SENSOR, "site_angular_velocity_sensor");

    const mjtNum* site_pos = d->sensordata + MuJoCo_helper->model->sensor_adr[site_pos_id];
    const mjtNum* site_quat = d->sensordata + MuJoCo_helper->model->sensor_adr[site_quat_id];
    const mjtNum* site_velp = d->sensordata + MuJoCo_helper->model->sensor_adr[site_velp_id];
//    const mjtNum* site_velr = d->sensordata + MuJoCo_helper->model->sensor_adr[site_velr_id];

    pose_7 goal_pose;
    pose_6 goal_velocity;
    pose_6 ee_pose;
//    MuJoCo_helper->GetBodyPoseQuatViaXpos(body_name, goal_pose, d);
////    MuJoCo_helper->GetBodyVelocity(body_name, goal_velocity, d);
//
//    int body_id = mj_name2id(MuJoCo_helper->model, mjOBJ_BODY, body_name.c_str());
//    for(int i = 0; i < 3; i++){
//        goal_velocity.position(i) = d->cvel[body_id * 6 + i];
//        goal_velocity.orientation(i) = d->cvel[body_id * 6 + 3 + i];
//    }

    for(int i = 0; i < 3; i++){
        goal_pose.position(i) = site_pos[i];
        goal_velocity.position(i) = site_velp[i];
        goal_velocity.orientation(i) = 0.0;
    }

    goal_pose.quat(0) = site_quat[0];
    goal_pose.quat(1) = site_quat[1];
    goal_pose.quat(2) = site_quat[2];
    goal_pose.quat(3) = site_quat[3];

//    std::cout << "body velocity: " << goal_velocity.position(0) << ", " << goal_velocity.position(1) << ", " << goal_velocity.position(2) << std::endl;

    // TODO - temp code to not consider a grapsed object
//    int site_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SITE, "force_sensor");
//    if (site_id == -1) {
//        std::cerr << "Error: Site " << "force_sensor" << " not found in the model.\n";
//        return;
//    }
//
//    double diff_x = d->site_xpos[site_id * 3] - residual_list[0].target[0];
//    residuals(resid_index++, 0) = diff_x;
//    double diff_y = d->site_xpos[site_id * 3 + 1] - residual_list[1].target[0];
//    residuals(resid_index++, 0) = diff_y;
//    double diff_z = d->site_xpos[site_id * 3 + 2] - residual_list[2].target[0];
//    residuals(resid_index++, 0) = diff_z;
//    // --------------------------------------------------------------


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

    // TODO - Temporary, unit quaternion
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

    mj_kinematics(MuJoCo_helper->model, d);
    mj_sensorPos(MuJoCo_helper->model, d);
    int site_pos_id = mj_name2id(MuJoCo_helper->model, mjOBJ_SENSOR, "site_position_sensor");
    const mjtNum* site_pos = d->sensordata + MuJoCo_helper->model->sensor_adr[site_pos_id];

    // Compute distance to the target
    double diffx, diffy, diffz;
    diffx = site_pos[0] - residual_list[0].target[0];
    diffy = site_pos[1] - residual_list[1].target[0];
    diffz = site_pos[2] - residual_list[2].target[0];

    dist = sqrt(pow(diffx,2) + pow(diffy,2) + 0.5 * pow(diffz,2));
    std::cout << "dist: " << dist << "\n";

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