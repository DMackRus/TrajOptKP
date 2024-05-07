#include "ModelTranslator/ModelTranslator.h"

void ModelTranslator::GenerateRandomGoalAndStartState() {
    ReturnRandomStartState();
    ReturnRandomGoalState();
}

void ModelTranslator::ReturnRandomGoalState() {
    std::cerr << "Generate random goal state not overrided for " << model_name << "model, exiting \n";
    exit(1);
}

void ModelTranslator::ReturnRandomStartState() {
    std::cerr << "Generate random start state not overrided for " << model_name << "model, exiting \n";
    exit(1);
}

void ModelTranslator::InitModelTranslator(const std::string& yamlFilePath){
    task taskConfig;

    FileHandler yamlReader;
    yamlReader.readModelConfigFile(yamlFilePath, taskConfig);
    model_file_path = taskConfig.modelFilePath;

    model_name = taskConfig.modelName;
    min_N = taskConfig.minN;
    max_N = taskConfig.maxN;
    keypoint_method = taskConfig.keypointMethod;
    auto_adjust = taskConfig.auto_adjust;
    iterative_error_threshold = taskConfig.iterativeErrorThreshold;
    const char* _modelPath = model_file_path.c_str();

    openloop_horizon = taskConfig.openloop_horizon;
    MPC_horizon = taskConfig.mpc_horizon;

    // Initialise physics simulator
    vector<string> bodyNames;
    for(auto & robot : taskConfig.robots){
        bodyNames.push_back(robot.name);
        for(int j = 0; j < robot.jointNames.size(); j++){
            jerk_thresholds.push_back(robot.jointJerkThresholds[j]);
            // TODO fix this duplicate jerk thresholds
            accel_thresholds.push_back(robot.jointJerkThresholds[j]);
            velocity_change_thresholds.push_back(robot.magVelThresholds[j]);
        }
    }

    for(auto & bodiesState : taskConfig.bodiesStates){
        bodyNames.push_back(bodiesState.name);
        for(int j = 0; j < 3; j++){
            jerk_thresholds.push_back(bodiesState.linearJerkThreshold[j]);
            jerk_thresholds.push_back(bodiesState.angularJerkThreshold[j]);

            // TODO fix this duplicate jerk thresholds
            accel_thresholds.push_back(bodiesState.linearJerkThreshold[j]);
            accel_thresholds.push_back(bodiesState.angularJerkThreshold[j]);

            velocity_change_thresholds.push_back(bodiesState.linearMagVelThreshold[j]);
            velocity_change_thresholds.push_back(bodiesState.angularMagVelThreshold[j]);
        }
    }

    MuJoCo_helper = std::make_shared<MuJoCoHelper>(taskConfig.robots, bodyNames);
    MuJoCo_helper->InitSimulator(taskConfig.modelTimeStep, _modelPath);

    full_state_vector.robots = taskConfig.robots;
    full_state_vector.bodiesStates = taskConfig.bodiesStates;

    // Clear optimiser dof and num ctrl so matrices are properly sized
    ResetSVR();

    std::cout << "full state vector names: ";
    for(const auto & state_vector_name : full_state_vector.state_names){
        std::cout << state_vector_name << " ";
    }
    std::cout << "\n";
}

void ModelTranslator::UpdateCurrentStateVector(std::vector<std::string> state_vector_names, bool add_extra_states){

    // state vector names -
    // robot joint names
    // bodies - {body_name}_x, {body_name}_y, {body_name}_z, {body_name}_roll, {body_name}_pitch, {body_name}_yaw

    // TODO - make this more robust, with how we handle robot joints
    // Checks if any state names contain panda, ignore these
    for (auto it = state_vector_names.begin(); it != state_vector_names.end(); /* no increment here */) {
        // Check if the string contains "panda"
        if (it->find("panda") != std::string::npos) {
            // Remove the string
            it = state_vector_names.erase(it); // erase() returns the iterator to the next valid position
        } else {
            // Move to the next string
            ++it;
        }
    }


    // TODO (DMackRus) - This is an assumption but should be fine
    if(add_extra_states){
        current_state_vector.dof += static_cast<int>(state_vector_names.size());
    }
    else{
        current_state_vector.dof -= static_cast<int>(state_vector_names.size());
    }

    // Keep track of elements not inside current state vector
    if(!add_extra_states){
        for(const auto & state_vector_name : state_vector_names){
            unused_state_vector_elements.push_back(state_vector_name);
        }
    }
    else{
        for(const auto & state_vector_name : state_vector_names){
            for(int i = 0; i < unused_state_vector_elements.size(); i++){
                if(unused_state_vector_elements[i] == state_vector_name){
                    // remove that element from this list
                    unused_state_vector_elements.erase(unused_state_vector_elements.begin() + i);
                    break;
                }
            }
        }
    }

    for(auto & robot : current_state_vector.robots){
        for(int joint = 0; joint < robot.jointNames.size(); joint++){

            for(int i = 0; i < state_vector_names.size(); i++){
                // TODO (DMackRus) - Need to add ability to activate / deactivate joints from state vector
            }
        }
    }

    // Remove or add elements for bodies in the state vector
    for(auto & bodiesState : current_state_vector.bodiesStates){
        std::string body_name = bodiesState.name;
        for(auto & state_vector_name : state_vector_names){
            size_t found = state_vector_name.find(body_name);

            if (found != std::string::npos) {
                state_vector_name.erase(found, body_name.length());
                if(state_vector_name == "_x"){
                    bodiesState.activeLinearDOF[0] = add_extra_states;
                }
                else if(state_vector_name == "_y"){
                    bodiesState.activeLinearDOF[1] = add_extra_states;
                }
                else if(state_vector_name == "_z"){
                    bodiesState.activeLinearDOF[2] = add_extra_states;
                }
                else if(state_vector_name == "_roll"){
                    bodiesState.activeAngularDOF[0] = add_extra_states;
                }
                else if(state_vector_name == "_pitch"){
                    bodiesState.activeAngularDOF[1] = add_extra_states;
                }
                else if(state_vector_name == "_yaw"){
                    bodiesState.activeAngularDOF[2] = add_extra_states;
                }
            }
        }
    }

    // Update the number of dofs in the state vector
    current_state_vector.Update();
    UpdateSceneVisualisation();
}

std::vector<std::string> ModelTranslator::RandomSampleUnusedDofs(int num_dofs) const{
    std::vector<std::string> dofs_names;
    std::vector<std::string> copy_unused = unused_state_vector_elements;

    // If no unused elements, return empty list
    if(unused_state_vector_elements.empty()){
        return dofs_names;
    }

    // Clamp number resample to number of unused elements
    if(unused_state_vector_elements.size() < num_dofs){
        num_dofs = static_cast<int>(unused_state_vector_elements.size());
    }

    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(copy_unused.begin(), copy_unused.end(), g);

    for(int i = 0; i < num_dofs; i++){
        dofs_names.push_back(copy_unused[i]);
    }

    return dofs_names;
}

void ModelTranslator::UpdateSceneVisualisation(){
    // Using the current state vector, update the geoms in the scene dependant how many dofs are active

    for(const auto &body : current_state_vector.bodiesStates){
        // count the number of dofs for this body
        int dof_for_body = 0;
        for(int i = 0; i < 3; i++){
            if(body.activeLinearDOF[i]){
                dof_for_body++;
            }
            if(body.activeAngularDOF[i]){
                dof_for_body++;
            }
        }

        // compute color
        color body_color{};
        if(body.name == "goal"){
            body_color = goal_colors[dof_for_body];
        }
        else{
            body_color = distractor_colors[dof_for_body];
        }

        float color[4] = {
                body_color.r,
                body_color.g,
                body_color.b,
                body_color.a
        };

        // Set color
        MuJoCo_helper->SetBodyColor(body.name, color);
    }
}

double ModelTranslator::CostFunction(mjData* d, const struct stateVectorList &state_vector, bool terminal){
    double cost = 0.0f;

    // Loop through robots in simulator
    for(auto & robot : state_vector.robots){
        int robot_num_joints = static_cast<int>(robot.jointNames.size());

        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_controls;

        MuJoCo_helper->GetRobotJointsPositions(robot.name, joint_positions, d);
        MuJoCo_helper->GetRobotJointsVelocities(robot.name, joint_velocities, d);
        MuJoCo_helper->GetRobotJointsControls(robot.name, joint_controls, d);

        // Loop through the robot joints
        for(int j = 0; j < robot_num_joints; j++) {

            double cost_pos, cost_vel;

            if(terminal){
                cost_pos = robot.terminalJointPosCosts[j];
                cost_vel = robot.terminalJointVelCosts[j];
            }
            else{
                cost_pos = robot.jointPosCosts[j];
                cost_vel = robot.jointVelCosts[j];
            }

            // Position costs
            cost += cost_pos * pow((joint_positions[j] - robot.goalPos[j]), 2);

            // Velocity costs
            cost += cost_vel * pow(joint_velocities[j] - robot.goalVel[j], 2);

            // Control costs
            cost += robot.jointControlCosts[j] * pow(joint_controls[j], 2);
        }
    }

    // Loop through the bodies in simulation
    for(const auto &body : state_vector.bodiesStates){
        cost += CostFunctionBody(body, d, terminal);
    }

    return cost;
}

double ModelTranslator::CostFunctionBody(const bodyStateVec& body, mjData *d, bool terminal){
    double cost = 0.0;

    pose_6 body_pose;
    pose_6 body_vel;
    MuJoCo_helper->GetBodyPoseAngle(body.name, body_pose, d);
    MuJoCo_helper->GetBodyVelocity(body.name, body_vel, d);

    // Linear costs
    for(int j = 0; j < 3; j++){
        if(body.activeLinearDOF[j]){

            if(terminal){
                // Position cost
                cost += body.terminalLinearPosCost[j] * pow((body_pose.position[j] - body.goalLinearPos[j]), 2);

                // Velocity cost
                cost += body.terminalLinearVelCost[j] * pow(body_vel.position[j], 2);

            }
            else{
                // Position cost
                cost += body.linearPosCost[j] * pow((body_pose.position[j] - body.goalLinearPos[j]), 2);

                // Velocity cost
                cost += body.linearVelCost[j] * pow(body_vel.position[j], 2);
            }
        }
    }

    // Angular cost
    // Compute rotation matrix of body in world frame
//    Eigen::Matrix3d current_rot_mat = eul2RotMat(body_pose.orientation);
////        std::cout << "rot mat: \n" << current_rot_mat << "\n";
//
//    // Convert desired orientation to rotation matrix
    m_point desired_axis;
    desired_axis(0) = body.goalAngularPos[0];
    desired_axis(1) = body.goalAngularPos[1];
    desired_axis(2) = body.goalAngularPos[2];
//    Eigen::Matrix3d desired_rot_mat = eul2RotMat(desired_eul);
//    std::cout << "desired eul: " << desired_eul << "\n";

    m_quat current, desired, inv_current, diff;
    current = axis2Quat(body_pose.orientation);
    desired = axis2Quat(desired_axis);
//    std::cout << "current: " << current << "\n";
//    std::cout << "desired: " << desired << "\n";
    inv_current = invQuat(current);
    diff = multQuat(inv_current, desired);

    // temp mujoco quat2vel methodology?
    double axis[3] = {diff[1], diff[2], diff[3]};
    double sin_a_2 = sin(sqrt(pow(axis[0], 2) + pow(axis[1], 2) + pow(axis[2], 2)));
    double speed = 2 * atan2(sin_a_2, diff[0]);

    // When axis angle > pi rot is in other direction
    if(speed > PI) speed -= 2*PI;

    axis[0] *= speed;
    axis[1] *= speed;
    axis[2] *= speed;

    m_point axis_diff = quat2Axis(diff);

//    std::cout << "axis diff" << axis_diff.transpose() << "\n";
//    std::cout << "axis " << axis[0] << " " << axis[1] << " " << axis[2] << "\n";


//    for(int i = 0; i < 3; i++){
//        if(terminal) {
//            cost += body.terminalAngularPosCost[i] * pow(axis_diff(i), 2);
//        }
//        else{
//            cost += body.angularPosCost[i] * pow(axis_diff(i), 2);
//        }
//    }

    double dot_x, dot_y, dot_z = 0.0f;

    // Axis X
//    if(body.activeAngularDOF[0]){
//        dot_x = current_rot_mat(0, 0) * desired_rot_mat(0, 0) +
//                current_rot_mat(1, 0) * desired_rot_mat(1, 0) +
//                current_rot_mat(2, 0) * desired_rot_mat(2, 0);
//
//        if(terminal) {
//            cost += body.terminalAngularPosCost[0] * acos(dot_x);
//        }
//        else{
//            cost += body.angularPosCost[0] * acos(dot_x);
//        }
//    }
//
//    // Axis Y
//    if(body.activeAngularDOF[1]){
//        dot_y = current_rot_mat(0, 1) * desired_rot_mat(0, 1) +
//                current_rot_mat(1, 1) * desired_rot_mat(1, 1) +
//                current_rot_mat(2, 1) * desired_rot_mat(2, 1);
//
//        if(terminal) {
//            cost += body.terminalAngularPosCost[1] * acos(dot_y);
//        }
//        else{
//            cost += body.angularPosCost[1] * acos(dot_y);
//        }
//    }
//
//    // Axis Z
//    if(body.activeAngularDOF[2]){
//        dot_z = current_rot_mat(0, 2) * desired_rot_mat(0, 2) +
//                current_rot_mat(1, 2) * desired_rot_mat(1, 2) +
//                current_rot_mat(2, 2) * desired_rot_mat(2, 2);
//
//        if(terminal) {
//            cost += body.terminalAngularPosCost[2] * acos(dot_z);
//        }
//        else{
//            cost += body.angularPosCost[2] * acos(dot_z);
//        }
//    }

//    std::cout << "dot x: " << dot_x << " dot y: " << dot_y << " dot z: " << dot_z << std::endl;

    for(int j = 0; j < 3; j++){
        if(body.activeAngularDOF[j]){

            if(terminal){
                // Velocity cost
                cost += body.terminalAngularVelCost[j] * pow(body_vel.orientation[j], 2);

            }
            else{
                // Velocity cost
                cost += body.angularVelCost[j] * pow(body_vel.orientation[j], 2);
            }
        }
    }

    return cost;
}

void ModelTranslator::CostDerivatives(mjData* d, const struct stateVectorList &state_vector,
                                        MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal){

    // Sanity check that size of matrices are correct for this state vector
    if(l_u.rows() != state_vector.num_ctrl){
        std::cerr << "mismatch in ctrl size between cost derivatives and passed state vector, exiting \n";
        exit(1);
    }

    if(l_x.rows() != state_vector.dof * 2){
        std::cerr << "mismatch in dof size between cost derivatives and passed state vector, exiting \n";
        exit(1);
    }

    // Aliases
    int dof = state_vector.dof;
    int num_ctrl = state_vector.num_ctrl;

    // Set matrices to zero as these matrices should mostly be sparse.
    l_u.setZero();
    l_uu.setZero();
    l_x.setZero();
    l_xx.setZero();

    int Q_index = 0;
    int R_index = 0;
    for(auto & robot : state_vector.robots){
        int robot_num_joints = static_cast<int>(robot.jointNames.size());
        int robot_num_actuators = static_cast<int>(robot.actuatorNames.size());

        std::vector<double> joint_positions;
        std::vector<double> joint_velocities;
        std::vector<double> joint_controls;

        MuJoCo_helper->GetRobotJointsPositions(robot.name, joint_positions, d);
        MuJoCo_helper->GetRobotJointsVelocities(robot.name, joint_velocities, d);
        MuJoCo_helper->GetRobotJointsControls(robot.name, joint_controls, d);

        // Loop through the robot joints (position and velocity)
        for(int i = 0; i < robot_num_joints; i++) {

            double cost_pos, cost_vel;

            if(terminal){
                cost_pos = robot.terminalJointPosCosts[i];
                cost_vel = robot.terminalJointVelCosts[i];
            }
            else{
                cost_pos = robot.jointPosCosts[i];
                cost_vel = robot.jointVelCosts[i];
            }

            // l_x = 2 * cost * diff
            // position and velocity
            l_x(Q_index + i) = 2 * cost_pos * (joint_positions[i] - robot.goalPos[i]);
            l_x(Q_index + i + dof) = 2 * cost_vel * (joint_velocities[i] - robot.goalVel[i]);

            // l_xx = 2 * cost
            l_xx(Q_index + i, Q_index + i) = 2 * cost_pos;
            l_xx(Q_index + i + dof, Q_index + i + dof) = 2 * cost_vel;

        }

        for(int i = 0; i < robot_num_actuators; i++){
            // l_u = 2 * cost * diff
            l_u(R_index + i) = 2 * robot.jointControlCosts[i] * joint_controls[i];

            // l_uu = 2 * cost
            l_uu(R_index, R_index) = 2 * robot.jointControlCosts[i];
        }

        // Increment index of Q and R (inside state vector and control vector repsectively)
        Q_index += robot_num_joints;
        R_index += robot_num_actuators;
    }

    for(const auto& body : state_vector.bodiesStates){
        pose_6 body_pose;
        pose_6 body_vel;
        MuJoCo_helper->GetBodyPoseAngle(body.name, body_pose, d);
        MuJoCo_helper->GetBodyVelocity(body.name, body_vel, d);

        // Linear cost derivatives
        for(int j = 0; j < 3; j++) {
            if (body.activeLinearDOF[j]) {

                if (terminal) {
                    // Position cost derivatives
                    l_x(Q_index) = 2 * body.terminalLinearPosCost[j] * (body_pose.position[j] - body.goalLinearPos[j]);
                    l_xx(Q_index, Q_index) = 2 * body.terminalLinearPosCost[j];

                    // Velocity cost derivatives
                    l_x(Q_index + dof) = 2 * body.terminalLinearVelCost[j] * (body_vel.position[j]);
                    l_xx(Q_index + dof, Q_index + dof) = 2 * body.terminalLinearVelCost[j];
                } else {
                    // Position cost derivatives
                    l_x(Q_index) = 2 * body.linearPosCost[j] * (body_pose.position[j] - body.goalLinearPos[j]);
                    l_xx(Q_index, Q_index) = 2 * body.linearPosCost[j];

                    // Velocity cost derivatives
                    l_x(Q_index + dof) = 2 * body.linearVelCost[j] * (body_vel.position[j]);
                    l_xx(Q_index + dof, Q_index + dof) = 2 * body.linearVelCost[j];
                }

                Q_index++;
            }
        }

        // TODO - if cost is 0,  we should also probbaly skip computation
        // If no angular dofs active for this body, skip!
        if(!body.activeAngularDOF[0] && !body.activeAngularDOF[1] && !body.activeAngularDOF[2]){
            continue;
        }

        // Angular cost derivatives
        // Angular pos cost = w1*acos(dot_x) + w2*acos(dot_y) + w3*acos(dot_z)

        //Compute 3D rotation matrix of current body orientation as well as the desired orientation
//        Eigen::Matrix3d current_rot_mat = eul2RotMat(body_pose.orientation);
//
//        // Convert desired orientation to rotation matrix
//        m_point desired;
//        desired(0) = body.goalAngularPos[0];
//        desired(1) = body.goalAngularPos[1];
//        desired(2) = body.goalAngularPos[2];
//        Eigen::Matrix3d desired_rot_mat = eul2RotMat(desired);
//
//        double dot_prods[3];
//        for(int j = 0; j < 3; j++){
//            dot_prods[j] = current_rot_mat(0, j) * desired_rot_mat(0, j) +
//                    current_rot_mat(1, j) * desired_rot_mat(1, j) +
//                    current_rot_mat(2, j) * desired_rot_mat(2, j);
//        }
//
//        double dot_x, dot_y, dot_z;
//        dot_x = dot_prods[0];
//        dot_y = dot_prods[1];
//        dot_z = dot_prods[2];
//
//        // Compute and store sin and cosine values for
//        // object roll, pitch, yaw (r, p, y)
//        // as well as goal roll, pitch and yaw (gr, gp, gy)
//        double sr, sp, sy, cr, cp, cy;
//        double sgr, sgp, sgy, cgr, cgp, cgy;
//
//        sr = sin(body_pose.orientation[0]);
//        sp = sin(body_pose.orientation[1]);
//        sy = sin(body_pose.orientation[2]);
//        cr = cos(body_pose.orientation[0]);
//        cp = cos(body_pose.orientation[1]);
//        cy = cos(body_pose.orientation[2]);
//
//        sgr = sin(body.goalAngularPos[0]);
//        sgp = sin(body.goalAngularPos[1]);
//        sgy = sin(body.goalAngularPos[2]);
//        cgr = cos(body.goalAngularPos[0]);
//        cgp = cos(body.goalAngularPos[1]);
//        cgy = cos(body.goalAngularPos[2]);
//
//        // first order deriv for dot_z, w.r.t roll, pitch, yaw
//        double ddot_z_dr, ddot_z_dp, ddot_z_dy;
//        // Storing values from computation of derivative dependant on desired goal angles
//        double F, G, H;
//
//        F = cgy*sgp*cgr + sgy*sgr;
//        G = sgy*sgp*cgr + sgy*sgr;
//        H = cgp*cgr;
//
//        ddot_z_dr = F*(-cy*sp*cr + sy*cr) + G*(-sy*sp*cr + sy*cr) + H*(-cp*sr);
//        ddot_z_dy = F*(-sy*sp*cr + cy*sr) + G*(cy*sp*cr + cy*sr);
//        ddot_z_dp = F*(cy*cp*cr) + G*(sy*cp*cr) + H*(-sp*cr);
//
////        std::cout << "ddot_z_dy: " << ddot_z_dy << std::endl;
//
//
//        double dacos_dot_z;
//        double dl_dr, dl_dp, dl_dy;
//
//        // This comes from deriv of acos(u)
//        dacos_dot_z = -1 / (sqrt(1 - pow(dot_z, 2)));
//
//        // TODO - is this the correct cost weighting???
//        if(terminal){
//            dl_dr = body.terminalAngularPosCost[2] * dacos_dot_z * ddot_z_dr;
//            dl_dp = body.terminalAngularPosCost[2] * dacos_dot_z * ddot_z_dp;
//            dl_dy = body.terminalAngularPosCost[2] * dacos_dot_z * ddot_z_dy;
//        }
//        else{
//            dl_dr = body.angularPosCost[2] * dacos_dot_z * ddot_z_dr;
//            dl_dp = body.angularPosCost[2] * dacos_dot_z * ddot_z_dp;
//            dl_dy = body.angularPosCost[2] * dacos_dot_z * ddot_z_dy;
//        }
//
//        // Use these values to plug into l_x
//
//        // TODO Q indexing code????
////        std::cout << "dl_dr" << dl_dr << " dl_dp: " << dl_dp << " dl_dy: " << dl_dy << std::endl;
//        l_x(Q_index) = dl_dr;
//        l_x(Q_index + 1) = dl_dp;
//        l_x(Q_index + 2) = dl_dy;
//
//        l_xx(Q_index, Q_index) = dl_dr*dl_dr;
//        l_xx(Q_index + 1, Q_index + 1) = dl_dp*dl_dp;
//        l_xx(Q_index + 2, Q_index + 2) = dl_dy*dl_dy;


        // TODO - second order l_xx derivs, also add computation from x and y vectors.......

        // use internal F.D of costfunction body to compute derivatives????
//        double cost_dec, cost_inc;
//        double eps = 1e-5;
//        // Perturb 3 orientations
//
//        for(int j = 0; j < 3; j++){
////            std::cout << "index: " << j << "\n";
//
//            body_pose.orientation[j] += eps;
//            // TODO - possible issue here with using the data object itself.
//            MuJoCo_helper->SetBodyPoseAngle(body.name, body_pose, d);
//
//            cost_inc = CostFunctionBody(body, d, terminal);
////            std::cout << "cost inc: " << cost_inc << "\n";
//
//            // Undo perturbation and apply negative perturb
//            body_pose.orientation[j] -= (2 * eps);
//
//            MuJoCo_helper->SetBodyPoseAngle(body.name, body_pose, d);
//
//            cost_dec = CostFunctionBody(body, d, terminal);
////            std::cout << "cost dec: " << cost_dec << "\n";
//
//            //Undo perturbation
//            body_pose.orientation[j] += eps;
//
//            MuJoCo_helper->SetBodyPoseAngle(body.name, body_pose, d);
//
//            l_x(Q_index) = (cost_inc - cost_dec) / (2 * eps);
//
//            l_xx(Q_index, Q_index) = l_x(Q_index) * l_x(Q_index);
//
//            Q_index ++;
//        }
    }

//    std::cout << "l_x \n" << l_x << std::endl;
}

bool ModelTranslator::TaskComplete(mjData* d, double &dist){
    dist = 0.0;
    return false;
}

std::vector<MatrixXd> ModelTranslator::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> emptyInitSetupControls;
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    return emptyInitSetupControls;
}

MatrixXd ModelTranslator::ReturnStateVector(mjData* d, const struct stateVectorList &state_vector){
    MatrixXd position_vector(state_vector.dof, 1);
    MatrixXd velocity_vector(state_vector.dof, 1);
    MatrixXd state_vector_values(state_vector.dof*2, 1);

    position_vector = ReturnPositionVector(d, state_vector);
    velocity_vector = ReturnVelocityVector(d, state_vector);

    state_vector_values.block(0, 0, state_vector.dof, 1) =
            position_vector.block(0, 0, state_vector.dof, 1);

    state_vector_values.block(state_vector.dof, 0, state_vector.dof, 1) =
            velocity_vector.block(0, 0, state_vector.dof, 1);

    return state_vector_values;
}

MatrixXd ModelTranslator::ReturnStateVectorQuaternions(mjData *d, const struct stateVectorList &state_vector){

    MatrixXd position_vector_quat(state_vector.dof_quat, 1);
    MatrixXd velocity_vector(state_vector.dof, 1);
    MatrixXd state_vector_quat(state_vector.dof_quat + state_vector.dof, 1);

    position_vector_quat = ReturnPositionVectorQuat(d, state_vector);
    velocity_vector = ReturnVelocityVector(d, state_vector);

    state_vector_quat.block(0, 0, state_vector.dof_quat, 1) = position_vector_quat;

    state_vector_quat.block(state_vector.dof_quat, 0, state_vector.dof, 1) = velocity_vector;

    return state_vector_quat;
}

bool ModelTranslator::SetStateVector(MatrixXd state_vector_values, mjData* d, const struct stateVectorList &state_vector){

    if(state_vector_values.rows() != state_vector.dof*2){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    MatrixXd position_vector(state_vector.dof, 1);
    MatrixXd velocity_vector(state_vector.dof, 1);

    position_vector = state_vector_values.block(0, 0, state_vector.dof, 1);
    velocity_vector = state_vector_values.block(state_vector.dof, 0, state_vector.dof, 1);

    SetPositionVector(position_vector, d, state_vector);
    SetVelocityVector(velocity_vector, d, state_vector);

    return true;
}

MatrixXd ModelTranslator::ReturnControlVector(mjData* d, const struct stateVectorList &state_vector){
    MatrixXd controlVector(state_vector.num_ctrl, 1);
    int current_control_index = 0;

    // loop through all the present robots
    for(auto & robot : state_vector.robots){
        vector<double> jointControls;
        MuJoCo_helper->GetRobotJointsControls(robot.name, jointControls, d);
        for(int j = 0; j < robot.actuatorNames.size(); j++){

            controlVector(current_control_index + j, 0) = jointControls[j];
        }

        current_control_index += static_cast<int>(robot.actuatorNames.size());
    }

    return controlVector;
}

MatrixXd ModelTranslator::ReturnControlLimits(const struct stateVectorList &state_vector){
    MatrixXd control_limits(state_vector.num_ctrl*2, 1);
    int current_control_index = 0;

    // loop through all the present robots
    for(auto & robot : state_vector.robots){
        vector<double> robot_control_limits;
        MuJoCo_helper->GetRobotControlLimits(robot.name, robot_control_limits);
        for(int j = 0; j < robot.actuatorNames.size() * 2; j++){
            control_limits(current_control_index + j, 0) = robot_control_limits[j];
        }

        current_control_index += static_cast<int>(robot.actuatorNames.size());
    }

    return control_limits;
}

bool ModelTranslator::SetControlVector(MatrixXd control_vector, mjData* d, const struct stateVectorList &state_vector){
    if(control_vector.rows() != state_vector.num_ctrl){
        cout << "ERROR: control vector size does not match the size of the control vector in the model translator" << endl;
        return false;
    }

    int current_control_index = 0;

    // loop through all the present robots
    for(auto & robot : state_vector.robots){
        vector<double> jointControls;
        for(int j = 0; j < robot.actuatorNames.size(); j++){

            jointControls.push_back(control_vector(current_control_index + j));
        }

        MuJoCo_helper->SetRobotJointsControls(robot.name, jointControls, d);

        current_control_index += static_cast<int>(robot.actuatorNames.size());
    }

    return true;
}

MatrixXd ModelTranslator::ReturnPositionVector(mjData* d, const struct stateVectorList &state_vector){
    MatrixXd position_vector(state_vector.dof, 1);

    int current_state_index = 0;

    // Loop through all robots in the state vector
    for(auto & robot : state_vector.robots){
        vector<double> jointPositions;
        MuJoCo_helper->GetRobotJointsPositions(robot.name, jointPositions, d);

        for(int j = 0; j < robot.jointNames.size(); j++){
            position_vector(j, 0) = jointPositions[j];
        }

        // Increment the current state index by the number of joints in the robot
        current_state_index += static_cast<int>(robot.jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 body_pose;
        MuJoCo_helper->GetBodyPoseAngle(bodiesState.name, body_pose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                position_vector(current_state_index, 0) = body_pose.position[j];
                current_state_index++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                position_vector(current_state_index, 0) = body_pose.orientation[j];
                current_state_index++;
            }
        }
    }

    return position_vector;
}

// TODO - perhaps this could be compressed, very similar to ReturnPositionVector
MatrixXd ModelTranslator::ReturnPositionVectorQuat(mjData *d, const struct stateVectorList &state_vector) {
    MatrixXd position_vector(state_vector.dof_quat, 1);

    int current_state_index = 0;

    // Loop through all robots in the state vector
    for(auto & robot : state_vector.robots){
        vector<double> jointPositions;
        MuJoCo_helper->GetRobotJointsPositions(robot.name, jointPositions, d);

        for(int j = 0; j < robot.jointNames.size(); j++){
            position_vector(j, 0) = jointPositions[j];
        }

        // Increment the current state index by the number of joints in the robot
        current_state_index += static_cast<int>(robot.jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_7 body_pose;
        MuJoCo_helper->GetBodyPoseQuat(bodiesState.name, body_pose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                position_vector(current_state_index, 0) = body_pose.position[j];
                current_state_index++;
            }
        }
        bool angular_dof_considered = false;
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                angular_dof_considered = true;

            }
        }

        if(angular_dof_considered){
            position_vector(current_state_index,     0) = body_pose.quat[0];
            position_vector(current_state_index + 1, 0) = body_pose.quat[1];
            position_vector(current_state_index + 2, 0) = body_pose.quat[2];
            position_vector(current_state_index + 3, 0) = body_pose.quat[3];
            current_state_index += 4;
        }
    }

    return position_vector;
}

MatrixXd ModelTranslator::ReturnVelocityVector(mjData* d, const struct stateVectorList &state_vector){
    MatrixXd velocity_vector(state_vector.dof, 1);
    int current_state_index = 0;

    // Loop through all robots in the state vector
    for(auto & robot : state_vector.robots){
        vector<double> joint_velocities;
        MuJoCo_helper->GetRobotJointsVelocities(robot.name, joint_velocities, d);

        for(int j = 0; j < robot.jointNames.size(); j++){
            velocity_vector(j, 0) = joint_velocities[j];
        }

        // Increment the current state index by the number of joints in the robot
        current_state_index += static_cast<int>(robot.jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 body_velocities;
        MuJoCo_helper->GetBodyVelocity(bodiesState.name, body_velocities, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                velocity_vector(current_state_index, 0) = body_velocities.position[j];
                current_state_index++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                velocity_vector(current_state_index, 0) = body_velocities.orientation[j];
                current_state_index++;
            }
        }
    }

    return velocity_vector;
}

//MatrixXd ModelTranslator::returnAccelerationVector(mjData* d){
//    MatrixXd accel_vector(dof, 1);
//
//    int currentStateIndex = 0;
//
//    // Loop through all robots in the state vector
//    for(auto & robot : active_state_vector.robots){
//        vector<double> jointAccelerations;
//        MuJoCo_helper->getRobotJointsAccelerations(robot.name, jointAccelerations, d);
//
//        for(int j = 0; j < robot.jointNames.size(); j++){
//            accel_vector(j, 0) = jointAccelerations[j];
//        }
//
//        // Increment the current state index by the number of joints in the robot
//        currentStateIndex += static_cast<int>(robot.jointNames.size());
//    }
//
//    // Loop through all bodies in the state vector
//    for(auto & bodiesState : active_state_vector.bodiesStates){
//        // Get the body's position and orientation
//        pose_6 bodyAccelerations;
//        MuJoCo_helper->getBodyAcceleration(bodiesState.name, bodyAccelerations, d);
//
//        for(int j = 0; j < 3; j++) {
//            // Linear positions
//            if (bodiesState.activeLinearDOF[j]) {
//                accel_vector(currentStateIndex, 0) = bodyAccelerations.position[j];
//                currentStateIndex++;
//            }
//        }
//        for(int j = 0; j < 3; j++) {
//            // angular positions
//            if(bodiesState.activeAngularDOF[j]){
//                accel_vector(currentStateIndex, 0) = bodyAccelerations.orientation[j];
//                currentStateIndex++;
//            }
//        }
//    }
//
//    return accel_vector;
//}

bool ModelTranslator::SetPositionVector(MatrixXd position_vector, mjData* d, const struct stateVectorList &state_vector){
    if(position_vector.rows() != state_vector.dof){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int current_state_index = 0;

    // Loop through all robots in the state vector
    for(auto & robot : state_vector.robots){
        vector<double> joint_positions;

        for(int j = 0; j < robot.jointNames.size(); j++){
            joint_positions.push_back(position_vector(j, 0));
        }

        MuJoCo_helper->SetRobotJointPositions(robot.name, joint_positions, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        current_state_index += static_cast<int>(robot.jointNames.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 body_pose;
        MuJoCo_helper->GetBodyPoseAngle(bodiesState.name, body_pose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                body_pose.position[j] = position_vector(current_state_index, 0);
                current_state_index++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                body_pose.orientation[j] = position_vector(current_state_index, 0);
                current_state_index++;
            }
        }

        MuJoCo_helper->SetBodyPoseAngle(bodiesState.name, body_pose, d);
    }

    return true;
}

bool ModelTranslator::SetVelocityVector(MatrixXd velocity_vector, mjData* d, const struct stateVectorList &state_vector){
    if(velocity_vector.rows() != state_vector.dof){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int current_state_index = 0;

    // Loop through all robots in the state vector
    for(auto & robot : state_vector.robots){
        vector<double> joint_velocities;

        for(int j = 0; j < robot.jointNames.size(); j++){
            joint_velocities.push_back(velocity_vector(j, 0));
        }
        
        MuJoCo_helper->SetRobotJointsVelocities(robot.name, joint_velocities, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        current_state_index += static_cast<int>(robot.jointNames.size());
    }


    // Loop through all bodies in the state vector
    for(auto & bodiesState : state_vector.bodiesStates){
        // Get the body's position and orientation
        pose_6 body_velocity;
        MuJoCo_helper->GetBodyVelocity(bodiesState.name, body_velocity, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.activeLinearDOF[j]) {
                body_velocity.position[j] = velocity_vector(current_state_index, 0);
                current_state_index++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.activeAngularDOF[j]){
                body_velocity.orientation[j] = velocity_vector(current_state_index, 0);
                current_state_index++;
            }
        }

        MuJoCo_helper->SetBodyVelocity(bodiesState.name, body_velocity, d);
    }

    return true;
}

int ModelTranslator::StateIndexToQposIndex(int state_index, const struct stateVectorList &state_vector){
    std::string state_name = state_vector.state_names[state_index];

    bool found_body_tag = false;
    int joint_index;
    int body_index_offset = 0;
    std::string joint_name;

    std::string body_tags[6]{"_x", "_y", "_z", "_roll", "_pitch", "_yaw"};
    for(int i = 0; i < 6; i++){
        found_body_tag = endsWith(state_name, body_tags[i]);

        if(found_body_tag){
            // Remove body tag from string
            state_name.erase(state_name.length() - body_tags[i].length(), body_tags[i].length());
            body_index_offset = i;
            break;
        }
    }

    if(found_body_tag){
        int bodyId = mj_name2id(MuJoCo_helper->model, mjOBJ_BODY, state_name.c_str());
        joint_index = MuJoCo_helper->model->jnt_dofadr[MuJoCo_helper->model->body_jntadr[bodyId]];
    }
    else{
        joint_index = mj_name2id(MuJoCo_helper->model, mjOBJ_JOINT, state_name.c_str());
    }

    return joint_index + body_index_offset;
}

void ModelTranslator::InitialiseSystemToStartState(mjData *d) {

    // Reset time of simulation
    d->time = 0.0;

    // Initialise robot positions to start configuration
    for(auto & robot : full_state_vector.robots){
        std::vector<double> zero_robot_velocities(robot.jointNames.size(), 0.0);
        MuJoCo_helper->SetRobotJointPositions(robot.name, robot.startPos, d);
        MuJoCo_helper->SetRobotJointsVelocities(robot.name, zero_robot_velocities, d);
    }

    // Initialise body poses to start configuration
    for(auto & bodiesState : full_state_vector.bodiesStates){

        pose_6 body_pose;
        pose_6 body_vel;

        for(int i = 0; i < 3; i++){
            body_pose.position[i] = bodiesState.startLinearPos[i];
            body_pose.orientation[i] = bodiesState.startAngularPos[i];

            body_vel.position[i] = 0.0;
            body_vel.orientation[i] = 0.0;
        }

        MuJoCo_helper->SetBodyPoseAngle(bodiesState.name, body_pose, d);
        MuJoCo_helper->SetBodyVelocity(bodiesState.name, body_vel, d);
    }

    // If we have a goal body in the model, lets set it to the goal pose
    // TODO - add this in task config yaml
    // TODO - also this assumes first body is the goal body.
    int body_id;
    if(MuJoCo_helper->BodyExists("display_goal",  body_id)){
        pose_7 goal_body;
        goal_body.position[0] = full_state_vector.bodiesStates[0].goalLinearPos[0];
        goal_body.position[1] = full_state_vector.bodiesStates[0].goalLinearPos[1];
        goal_body.position[2] = full_state_vector.bodiesStates[0].goalLinearPos[2];

        m_point desired_eul = {full_state_vector.bodiesStates[0].goalAngularPos[0],
                               full_state_vector.bodiesStates[0].goalAngularPos[1],
                               full_state_vector.bodiesStates[0].goalAngularPos[2]};

        goal_body.quat = eul2Quat(desired_eul);
        std::cout << "goal body pos: " << goal_body.position[0] << " " << goal_body.position[1] << " " << goal_body.position[2] << "\n";

        MuJoCo_helper->SetBodyPoseQuat("display_goal", goal_body, d);
    }
}

std::vector<MatrixXd> ModelTranslator::CreateInitOptimisationControls(int horizon_length) {
    std::vector<MatrixXd> init_controls;

    int num_ctrl = full_state_vector.num_ctrl;

    for(int i = 0; i < horizon_length; i++){
        MatrixXd emptyControl(num_ctrl, 1);
        for(int j = 0; j < num_ctrl; j++){
            emptyControl(j) = 0.0f;
        }
        init_controls.push_back(emptyControl);
    }

    return init_controls;
}