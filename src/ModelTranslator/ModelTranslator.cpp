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
    yamlReader.ReadModelConfigFile(yamlFilePath, taskConfig);
    model_file_path = taskConfig.model_filepath;

    model_name = taskConfig.model_name;
    min_N = taskConfig.minN;
    max_N = taskConfig.maxN;
    keypoint_method = taskConfig.keypointMethod;
    auto_adjust = taskConfig.auto_adjust;
    iterative_error_threshold = taskConfig.iterativeErrorThreshold;
    const char* _modelPath = model_file_path.c_str();

    openloop_horizon = taskConfig.openloop_horizon;
    MPC_horizon = taskConfig.mpc_horizon;

    // Residuals
    residual_list = taskConfig.residuals;

    // Initialise physics simulator
    vector<string> bodyNames;
    for(auto & robot : taskConfig.robots){
        bodyNames.push_back(robot.name);
        int root_offset = 0;
        if(robot.root_name != "-"){
            root_offset = 6;
        }
        for(int j = 0; j < robot.joint_names.size() + root_offset; j++){
            jerk_thresholds.push_back(robot.jerk_thresholds[j]);
            // TODO fix this duplicate jerk thresholds
            accel_thresholds.push_back(robot.jerk_thresholds[j]);
            velocity_change_thresholds.push_back(robot.vel_change_thresholds[j]);
        }
    }

    for(auto & bodiesState : taskConfig.rigid_bodies){
        bodyNames.push_back(bodiesState.name);
        for(int j = 0; j < 3; j++){
            if(bodiesState.active_linear_dof[j]){
                jerk_thresholds.push_back(bodiesState.linear_jerk_threshold[j]);
                // TODO fix this duplicate jerk thresholds
                accel_thresholds.push_back(bodiesState.linear_jerk_threshold[j]);
                velocity_change_thresholds.push_back(bodiesState.linear_vel_change_threshold[j]);
            }

            if(bodiesState.active_angular_dof[j]){
                jerk_thresholds.push_back(bodiesState.angular_jerk_threshold[j]);
                // TODO fix this duplicate jerk thresholds
                accel_thresholds.push_back(bodiesState.angular_jerk_threshold[j]);
                velocity_change_thresholds.push_back(bodiesState.angular_vel_change_threshold[j]);
            }
        }
    }

    MuJoCo_helper = std::make_shared<MuJoCoHelper>(taskConfig.robots, bodyNames);

    full_state_vector.robots = taskConfig.robots;
    full_state_vector.rigid_bodies = taskConfig.rigid_bodies;
    full_state_vector.soft_bodies = taskConfig.soft_bodies;

    bool use_plugins = false;
    if(!full_state_vector.soft_bodies.empty()){
        use_plugins = true;
    }

    // Init simulator, make xml, make data, init plugins if required
    MuJoCo_helper->InitSimulator(taskConfig.model_time_step, _modelPath, use_plugins);

    // Setup colors of all rigid bodies
    for(auto &body: full_state_vector.rigid_bodies){
        std::array<double, 4> base_color = MuJoCo_helper->ReturnBodyColor(body.name);
        body.base_color[0] = base_color[0];
        body.base_color[1] = base_color[1];
        body.base_color[2] = base_color[2];
        body.base_color[3] = base_color[3];
    }

    // Clear optimiser dof and num ctrl so matrices are properly sized
    ResetSVR();
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
        for(int joint = 0; joint < robot.joint_names.size(); joint++){

            for(int i = 0; i < state_vector_names.size(); i++){
                // TODO (DMackRus) - Need to add ability to activate / deactivate joints from state vector
            }
        }
    }

    // Remove or add elements for rigid bodies in the state vector
    for(auto & rigid_body : current_state_vector.rigid_bodies){
        std::string body_name = rigid_body.name;
        for(auto & state_vector_name : state_vector_names){
            size_t found = state_vector_name.find(body_name);

            if (found != std::string::npos) {
                state_vector_name.erase(found, body_name.length());
                if(state_vector_name == "_x"){
                    rigid_body.active_linear_dof[0] = add_extra_states;
                }
                else if(state_vector_name == "_y"){
                    rigid_body.active_linear_dof[1] = add_extra_states;
                }
                else if(state_vector_name == "_z"){
                    rigid_body.active_linear_dof[2] = add_extra_states;
                }
                else if(state_vector_name == "_roll"){
                    rigid_body.active_angular_dof[0] = add_extra_states;
                }
                else if(state_vector_name == "_pitch"){
                    rigid_body.active_angular_dof[1] = add_extra_states;
                }
                else if(state_vector_name == "_yaw"){
                    rigid_body.active_angular_dof[2] = add_extra_states;
                }
            }
        }
    }

    // Remove or add states for sodt bodies in the simulator
    for( auto & soft_body : current_state_vector.soft_bodies){
        std::string body_name = soft_body.name;
        for(auto & state_vector_name : state_vector_names){
            size_t found = state_vector_name.find(body_name);

            if (found != std::string::npos) {
                // Erases soft body name
                state_vector_name.erase(found, body_name.length());

                // String should now be in form "_{i}_{x, y, or z}
                // we want number i as its the vertex number

                // Find vertex number
                int vertex_number;

                size_t firstUnderscorePos = state_vector_name.find('_V');

                // Find the position of the second underscore
                size_t secondUnderscorePos = state_vector_name.find('_', firstUnderscorePos + 2);

                std::string numberString = state_vector_name.substr(firstUnderscorePos + 1, secondUnderscorePos - (firstUnderscorePos + 1));
                vertex_number = std::atoi(numberString.c_str());

                // Find x, y or z suffix
                state_vector_name.erase(firstUnderscorePos - 1, secondUnderscorePos - (firstUnderscorePos - 1));

                if(state_vector_name == "_x"){
                    soft_body.vertices[vertex_number].active_linear_dof[0] = add_extra_states;
                }
                else if(state_vector_name == "_y"){
                    soft_body.vertices[vertex_number].active_linear_dof[1] = add_extra_states;
                }
                else if(state_vector_name == "_z"){
                    soft_body.vertices[vertex_number].active_linear_dof[2] = add_extra_states;
                }
            }
        }
    }

    // Update the number of dofs in the state vector
    current_state_vector.Update();
    // Compute state vector address indices
    state_dof_adr_indices.clear();
    ComputeStateDofAdrIndices(MuJoCo_helper->master_reset_data, full_state_vector);

    // if readding dofs, dont update scene vis yet
    if(!add_extra_states){
        UpdateSceneVisualisation();
    }
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

    for(int  i = 0; i < current_state_vector.rigid_bodies.size(); i++){

        // count the number of dofs for this body
        int current_dof_for_body = 0;
        int full_dof_for_body = 0;
        for(int j = 0; j < 3; j++){
            // Current state vector
            if(current_state_vector.rigid_bodies[i].active_linear_dof[j]){
                current_dof_for_body++;
            }
            if(current_state_vector.rigid_bodies[i].active_angular_dof[j]){
                current_dof_for_body++;
            }

            // Full state vector
            if(full_state_vector.rigid_bodies[i].active_linear_dof[j]){
                full_dof_for_body++;
            }
            if(full_state_vector.rigid_bodies[i].active_angular_dof[j]){
                full_dof_for_body++;
            }
        }

        float percentage_dofs_used = (float)current_dof_for_body / (float)full_dof_for_body;

        // Color = Percentage * base_color
        double color[4];
        color[0] = percentage_dofs_used * full_state_vector.rigid_bodies[i].base_color[0];
        color[1] = percentage_dofs_used * full_state_vector.rigid_bodies[i].base_color[1];
        color[2] = percentage_dofs_used * full_state_vector.rigid_bodies[i].base_color[2];
        color[3] = full_state_vector.rigid_bodies[i].base_color[3];

        // Set color
        MuJoCo_helper->SetBodyColor(current_state_vector.rigid_bodies[i].name, color);
    }
}

void ModelTranslator::Residuals(mjData *d, MatrixXd &residual) {
    std::cerr << "residuals not implemented for this model, exiting \n";
    exit(1);
}

double ModelTranslator::CostFunction(const MatrixXd &residuals, const struct stateVectorList &state_vector, bool terminal){
    double cost = 0.0;

    for(int i = 0; i < residual_list.size(); i++){
        // Hardcoded norm function is pow(r, 2)
        if(terminal){
            cost += residual_list[i].weight_terminal * pow(residuals(i), 2);
        }
        else{
            cost += residual_list[i].weight * pow(residuals(i), 2);
        }
    }

    return cost;
}

void ModelTranslator::CostDerivativesFromResiduals(const struct stateVectorList &state_vector,
                                  MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu,
                                  const MatrixXd &residuals, const vector<MatrixXd> r_x, const vector<MatrixXd> r_u, bool terminal){
    l_x.setZero();
    l_xx.setZero();
    l_u.setZero();
    l_uu.setZero();

    double weight_term;

    for(int i = 0; i < residual_list.size(); i++){

        if(terminal){
            weight_term = residual_list[i].weight_terminal;
        }
        else{
            weight_term = residual_list[i].weight;
        }

        // Hardcoded norm function is pow(r, 2)
        // l_x = w_i * dn/dr * dr/dx (dn/dr = 2r, as n = r^2)
        l_x += weight_term * 2 * residuals(i) * r_x[i];

        // l_xx = w_i * dn2/dr2 * dr2/dx2 (dr/dx * dr/dx^T) Gauss newton approximation
        l_xx += weight_term * 2 * r_x[i] * r_x[i].transpose();

        // l_u = w_i * dn/dr * dr/du
        l_u += weight_term * 2 * residuals(i) * r_u[i];
        // l_uu = w_i * dn2/dr2 * dr2/du2( dr/du * dr/du^T) Gauss newton approximation
        l_uu += 2 * weight_term * r_u[i] * r_u[i].transpose();
    }
}

// Default task complete function, task is never complete
bool ModelTranslator::TaskComplete(mjData* d, double &dist){
    dist = 0.0;
    return false;
}

std::vector<MatrixXd> ModelTranslator::CreateInitSetupControls(int horizonLength){
    std::vector<MatrixXd> emptyInitSetupControls;
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->master_reset_data);
    mj_forward(MuJoCo_helper->model, MuJoCo_helper->main_data);
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

bool ModelTranslator::SetStateVectorQuat(MatrixXd state_vector_values, mjData* d, const struct stateVectorList &state_vector){
    if(state_vector_values.rows() != state_vector.dof_quat + state_vector.dof){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    MatrixXd position_vector(state_vector.dof_quat, 1);
    MatrixXd velocity_vector(state_vector.dof, 1);

    position_vector = state_vector_values.block(0, 0, state_vector.dof_quat, 1);
    velocity_vector = state_vector_values.block(state_vector.dof_quat, 0, state_vector.dof, 1);

    SetPositionVectorQuat(position_vector, d, state_vector);
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
        for(int j = 0; j < robot.actuator_names.size(); j++){

            controlVector(current_control_index + j, 0) = jointControls[j];
        }

        current_control_index += static_cast<int>(robot.actuator_names.size());
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
        for(int j = 0; j < robot.actuator_names.size() * 2; j++){
            control_limits(current_control_index + j, 0) = robot_control_limits[j];
        }

        current_control_index += static_cast<int>(robot.actuator_names.size());
    }

    return control_limits;
}

bool ModelTranslator::SetControlVector(MatrixXd control_vector, mjData* d, const struct stateVectorList &state_vector){
    if(control_vector.rows() != state_vector.num_ctrl){
        cout << "ERROR: control vector size " << control_vector.rows() << " does not match the size of the control vector in the model translator: " << state_vector.num_ctrl << endl;
        cout << "control vector: " << control_vector.transpose() << "\n";
        return false;
    }

    int current_control_index = 0;

    // loop through all the present robots
    for(auto & robot : state_vector.robots){
        vector<double> jointControls;
        for(int j = 0; j < robot.actuator_names.size(); j++){

            jointControls.push_back(control_vector(current_control_index + j));
        }

        MuJoCo_helper->SetRobotJointsControls(robot.name, jointControls, d);

        current_control_index += static_cast<int>(robot.actuator_names.size());
    }

    return true;
}

MatrixXd ModelTranslator::ReturnPositionVector(mjData* d, const struct stateVectorList &state_vector){
    MatrixXd position_vector(state_vector.dof, 1);

    int current_state_index = 0;

    // Loop through all robots in the state vector
    for(auto &robot : state_vector.robots){
        if(robot.root_name != "-"){
            pose_6 root_position;
            MuJoCo_helper->GetBodyPoseAngle(robot.root_name, root_position, d);

            position_vector(current_state_index, 0) = root_position.position[0];
            position_vector(current_state_index + 1, 0) = root_position.position[1];
            position_vector(current_state_index + 2, 0) = root_position.position[2];
            position_vector(current_state_index + 3, 0) = root_position.orientation[0];
            position_vector(current_state_index + 4, 0) = root_position.orientation[1];
            position_vector(current_state_index + 5, 0) = root_position.orientation[2];

            current_state_index += 6;

        }
        vector<double> jointPositions;
        MuJoCo_helper->GetRobotJointsPositions(robot.name, jointPositions, d);

        for(int j = 0; j < robot.joint_names.size(); j++){
            position_vector(current_state_index + j, 0) = jointPositions[j];
        }

        // Increment the current state index by the number of joints in the robot
        current_state_index += static_cast<int>(robot.joint_names.size());
    }

    // ------------------- Rigid body position elements --------------------
    for(auto & bodiesState : state_vector.rigid_bodies){
        // Get the body's position and orientation
        pose_6 body_pose;
        MuJoCo_helper->GetBodyPoseAngle(bodiesState.name, body_pose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.active_linear_dof[j]) {
                position_vector(current_state_index, 0) = body_pose.position[j];
                current_state_index++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.active_angular_dof[j]){
                position_vector(current_state_index, 0) = body_pose.orientation[j];
                current_state_index++;
            }
        }
    }

    //  ------------------ Soft body position elements -----------------------------------
    for(auto & soft_body : state_vector.soft_bodies){
        // Get the body's position and orientation
        pose_6 body_pose;

        for(int i = 0; i < soft_body.num_vertices; i++){

            MuJoCo_helper->GetSoftBodyVertexPos(soft_body.name, i, body_pose, d);
            for(int j = 0; j < 3; j++){
                if(soft_body.vertices[i].active_linear_dof[j]){
                    position_vector(current_state_index, 0) = body_pose.position[j];
                    current_state_index++;
                }
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
        if(robot.root_name != "-"){
            pose_7 root_position;
            MuJoCo_helper->GetBodyPoseQuat(robot.root_name, root_position, d);

            position_vector(current_state_index, 0) = root_position.position[0];
            position_vector(current_state_index + 1, 0) = root_position.position[1];
            position_vector(current_state_index + 2, 0) = root_position.position[2];
            position_vector(current_state_index + 3, 0) = root_position.quat[0];
            position_vector(current_state_index + 4, 0) = root_position.quat[1];
            position_vector(current_state_index + 5, 0) = root_position.quat[2];
            position_vector(current_state_index + 6, 0) = root_position.quat[3];

            current_state_index += 7;

        }
        vector<double> jointPositions;
        MuJoCo_helper->GetRobotJointsPositions(robot.name, jointPositions, d);

        for(int j = 0; j < robot.joint_names.size(); j++){
            position_vector(current_state_index + j, 0) = jointPositions[j];
        }

        // Increment the current state index by the number of joints in the robot
        current_state_index += static_cast<int>(robot.joint_names.size());
    }

    // Loop through all bodies in the state vector
    for(auto & bodiesState : state_vector.rigid_bodies){
        // Get the body's position and orientation
        pose_7 body_pose;
        MuJoCo_helper->GetBodyPoseQuat(bodiesState.name, body_pose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.active_linear_dof[j]) {
                position_vector(current_state_index, 0) = body_pose.position[j];
                current_state_index++;
            }
        }
        bool angular_dof_considered = false;
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.active_angular_dof[j]){
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

    //  ------------------ Soft body position elements -----------------------------------
    for(auto & soft_body : state_vector.soft_bodies){
        // Get the body's position and orientation
        pose_6 body_pose;

        for(int i = 0; i < soft_body.num_vertices; i++){

            MuJoCo_helper->GetSoftBodyVertexPos(soft_body.name, i, body_pose, d);
            for(int j = 0; j < 3; j++){
                if(soft_body.vertices[i].active_linear_dof[j]){
                    position_vector(current_state_index, 0) = body_pose.position[j];
                    current_state_index++;
                }
            }
        }
    }

    return position_vector;
}

MatrixXd ModelTranslator::ReturnVelocityVector(mjData* d, const struct stateVectorList &state_vector){
    MatrixXd velocity_vector(state_vector.dof, 1);
    int current_state_index = 0;

    // Loop through all robots in the state vector
    for(auto & robot : state_vector.robots){
        if(robot.root_name != "-"){
            pose_6 root_velocity;
            MuJoCo_helper->GetBodyVelocity(robot.root_name, root_velocity, d);

            velocity_vector(current_state_index, 0) = root_velocity.position[0];
            velocity_vector(current_state_index + 1, 0) = root_velocity.position[1];
            velocity_vector(current_state_index + 2, 0) = root_velocity.position[2];
            velocity_vector(current_state_index + 3, 0) = root_velocity.orientation[0];
            velocity_vector(current_state_index + 4, 0) = root_velocity.orientation[1];
            velocity_vector(current_state_index + 5, 0) = root_velocity.orientation[2];

            current_state_index += 6;
        }

        vector<double> joint_velocities;
        MuJoCo_helper->GetRobotJointsVelocities(robot.name, joint_velocities, d);

        for(int j = 0; j < robot.joint_names.size(); j++){
            velocity_vector(current_state_index + j, 0) = joint_velocities[j];
        }

        // Increment the current state index by the number of joints in the robot
        current_state_index += static_cast<int>(robot.joint_names.size());

    }

    // ------------------- Rigid body velocity elements -------------------
    for(auto & bodiesState : state_vector.rigid_bodies){
        // Get the body's position and orientation
        pose_6 body_velocities;
        MuJoCo_helper->GetBodyVelocity(bodiesState.name, body_velocities, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.active_linear_dof[j]) {
                velocity_vector(current_state_index, 0) = body_velocities.position[j];
                current_state_index++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.active_angular_dof[j]){
                velocity_vector(current_state_index, 0) = body_velocities.orientation[j];
                current_state_index++;
            }
        }
    }

    //  ------------------ Soft body velocity elements -----------------------------------
    for(auto & soft_body : state_vector.soft_bodies){
        // Get the body's position and orientation
        pose_6 body_velocities;

        for(int i = 0; i < soft_body.num_vertices; i++){

            MuJoCo_helper->GetSoftBodyVertexVel(soft_body.name, i, body_velocities, d);
            for(int j = 0; j < 3; j++){
                if(soft_body.vertices[i].active_linear_dof[j]){
                    velocity_vector(current_state_index, 0) = body_velocities.position[j];
                    current_state_index++;
                }
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
        if(robot.root_name != "-"){
            pose_6 root_position;

            root_position.position[0] = position_vector(current_state_index, 0);
            root_position.position[1] = position_vector(current_state_index + 1, 0);
            root_position.position[2] = position_vector(current_state_index + 2, 0);
            root_position.orientation[0] = position_vector(current_state_index + 3, 0);
            root_position.orientation[1] = position_vector(current_state_index + 4, 0);
            root_position.orientation[2] = position_vector(current_state_index + 5, 0);

            MuJoCo_helper->SetBodyPoseAngle(robot.root_name, root_position, d);

            current_state_index += 6;
        }
        vector<double> joint_positions;

        for(int j = 0; j < robot.joint_names.size(); j++){
            joint_positions.push_back(position_vector(current_state_index + j, 0));
        }

        MuJoCo_helper->SetRobotJointPositions(robot.name, joint_positions, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        current_state_index += static_cast<int>(robot.joint_names.size());
    }

    // -------------------- rigid body position elements ---------------------------
    for(auto & rigid_body : state_vector.rigid_bodies){
        // Get the body's position and orientation
        pose_6 body_pose;
        MuJoCo_helper->GetBodyPoseAngle(rigid_body.name, body_pose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (rigid_body.active_linear_dof[j]) {
                body_pose.position[j] = position_vector(current_state_index, 0);
                current_state_index++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(rigid_body.active_angular_dof[j]){
                body_pose.orientation[j] = position_vector(current_state_index, 0);
                current_state_index++;
            }
        }

        MuJoCo_helper->SetBodyPoseAngle(rigid_body.name, body_pose, d);
    }

    //  ------------------ Soft body position elements -----------------------------------
    for(auto & soft_body : state_vector.soft_bodies){
        // Get the body's position and orientation
        pose_6 body_pose;


        for(int i = 0; i < soft_body.num_vertices; i++){

            MuJoCo_helper->GetSoftBodyVertexPos(soft_body.name, i, body_pose, d);
            for(int j = 0; j < 3; j++){
                if(soft_body.vertices[i].active_linear_dof[j]){
                    body_pose.position[j] = position_vector(current_state_index, 0);
                    current_state_index++;
                }

            }
            MuJoCo_helper->SetSoftBodyVertexPos(soft_body.name, i, body_pose, d);
        }
    }

    return true;
}

bool ModelTranslator::SetPositionVectorQuat(MatrixXd position_vector, mjData* d, const struct stateVectorList &state_vector){
    if(position_vector.rows() != state_vector.dof_quat){
        cout << "ERROR: state vector size does not match the size of the state vector in the model translator" << endl;
        return false;
    }

    int current_state_index = 0;

    // Loop through all robots in the state vector
    for(auto & robot : state_vector.robots){
        if(robot.root_name != "-"){
            pose_7 root_position;

            root_position.position[0] = position_vector(current_state_index, 0);
            root_position.position[1] = position_vector(current_state_index + 1, 0);
            root_position.position[2] = position_vector(current_state_index + 2, 0);
            root_position.quat[0] = position_vector(current_state_index + 3, 0);
            root_position.quat[1] = position_vector(current_state_index + 4, 0);
            root_position.quat[2] = position_vector(current_state_index + 5, 0);
            root_position.quat[3] = position_vector(current_state_index + 6, 0);

            MuJoCo_helper->SetBodyPoseQuat(robot.root_name, root_position, d);

            current_state_index += 7;
        }

        vector<double> joint_positions;

        for(int j = 0; j < robot.joint_names.size(); j++){
            joint_positions.push_back(position_vector(current_state_index + j, 0));
        }

        MuJoCo_helper->SetRobotJointPositions(robot.name, joint_positions, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        current_state_index += static_cast<int>(robot.joint_names.size());
    }

    // -------------------- rigid body position elements ---------------------------
    for(auto & rigid_body : state_vector.rigid_bodies){
        // Get the body's position and orientation
        pose_7 body_pose;
        MuJoCo_helper->GetBodyPoseQuat(rigid_body.name, body_pose, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (rigid_body.active_linear_dof[j]) {
                body_pose.position[j] = position_vector(current_state_index, 0);
                current_state_index++;
            }
        }

        bool angular_dof_considered = false;
        for(bool j : rigid_body.active_angular_dof) {
            // angular positions
            if(j){
                angular_dof_considered = true;
            }
        }

        if(angular_dof_considered){
            body_pose.quat[0] = position_vector(current_state_index,     0);
            body_pose.quat[1] = position_vector(current_state_index + 1, 0);
            body_pose.quat[2] = position_vector(current_state_index + 2, 0);
            body_pose.quat[3] = position_vector(current_state_index + 3, 0);
        }

        MuJoCo_helper->SetBodyPoseQuat(rigid_body.name, body_pose, d);
    }

    //  ------------------ Soft body position elements -----------------------------------
    for(auto & soft_body : state_vector.soft_bodies){
        // Get the body's position and orientation
        pose_6 body_pose;


        for(int i = 0; i < soft_body.num_vertices; i++){

            MuJoCo_helper->GetSoftBodyVertexPos(soft_body.name, i, body_pose, d);
            for(int j = 0; j < 3; j++){
                if(soft_body.vertices[i].active_linear_dof[j]){
                    body_pose.position[j] = position_vector(current_state_index, 0);
                    current_state_index++;
                }

            }
            MuJoCo_helper->SetSoftBodyVertexPos(soft_body.name, i, body_pose, d);
        }
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
        if(robot.root_name != "-"){
            pose_6 root_velocity;

            root_velocity.position[0] = velocity_vector(current_state_index, 0);
            root_velocity.position[1] = velocity_vector(current_state_index + 1, 0);
            root_velocity.position[2] = velocity_vector(current_state_index + 2, 0);
            root_velocity.orientation[0] = velocity_vector(current_state_index + 3, 0);
            root_velocity.orientation[1] = velocity_vector(current_state_index + 4, 0);
            root_velocity.orientation[2] = velocity_vector(current_state_index + 5, 0);

            MuJoCo_helper->SetBodyVelocity(robot.root_name, root_velocity, d);

            current_state_index += 6;
        }

        vector<double> joint_velocities;

        for(int j = 0; j < robot.joint_names.size(); j++){
            joint_velocities.push_back(velocity_vector(current_state_index + j, 0));
        }
        
        MuJoCo_helper->SetRobotJointsVelocities(robot.name, joint_velocities, d);

        // Increment the current state index by the number of joints in the robot x 2 (for positions and velocities)
        current_state_index += static_cast<int>(robot.joint_names.size());
    }


    // -------------------- rigid body velocity elemenets --------------------
    for(auto & bodiesState : state_vector.rigid_bodies){
        // Get the body's position and orientation
        pose_6 body_velocity;
        MuJoCo_helper->GetBodyVelocity(bodiesState.name, body_velocity, d);

        for(int j = 0; j < 3; j++) {
            // Linear positions
            if (bodiesState.active_linear_dof[j]) {
                body_velocity.position[j] = velocity_vector(current_state_index, 0);
                current_state_index++;
            }
        }
        for(int j = 0; j < 3; j++) {
            // angular positions
            if(bodiesState.active_angular_dof[j]){
                body_velocity.orientation[j] = velocity_vector(current_state_index, 0);
                current_state_index++;
            }
        }

        MuJoCo_helper->SetBodyVelocity(bodiesState.name, body_velocity, d);
    }

    //  ------------------ Soft body velocity elements -----------------------------------
    for(auto & soft_body : state_vector.soft_bodies){
        // Get the body's position and orientation
        pose_6 body_pose;

        for(int i = 0; i < soft_body.num_vertices; i++){
            MuJoCo_helper->GetSoftBodyVertexVel(soft_body.name, i, body_pose, d);
            for(int j = 0; j < 3; j++){
                if(soft_body.vertices[i].active_linear_dof[j]){
                    body_pose.position[j] = velocity_vector(current_state_index, 0);
                    current_state_index++;
                }
            }
            MuJoCo_helper->SetSoftBodyVertexVel(soft_body.name, i, body_pose, d);
        }
    }

    return true;
}

void ModelTranslator::ComputeStateDofAdrIndices(mjData* d, const struct stateVectorList &state_vector){

    std::string lin_suffixes[3] = {"_x", "_y", "_z"};
    std::string ang_suffixes[3] = {"_roll", "_pitch", "_yaw"};

    for(auto & robot : state_vector.robots){
        // Check if robot has a free root joint?
        if(robot.root_name != "-"){

            // Compute joint id
            int joint_id = mj_name2id(MuJoCo_helper->model, mjOBJ_JOINT, robot.root_name.c_str());
            int dof_adr = MuJoCo_helper->model->jnt_dofadr[joint_id];

            // TODO - enable this to be programatic, i.e not all elements of root in the state vector
            for(int i = 0; i < 6; i++){
                state_dof_adr_indices.push_back(dof_adr + i);
            }
        }

        // Loop through slide / hinge joints
        for(const auto & joint_name : robot.joint_names){
            int joint_id = mj_name2id(MuJoCo_helper->model, mjOBJ_JOINT, joint_name.c_str());
            int dof_adr = MuJoCo_helper->model->jnt_dofadr[joint_id];
            state_dof_adr_indices.push_back(dof_adr);
        }
    }

    // Loop through all bodies in the state vector
    for(auto & body : state_vector.rigid_bodies) {
        int body_id = mj_name2id(MuJoCo_helper->model, mjOBJ_BODY, body.name.c_str());
        int joint_id = MuJoCo_helper->model->body_jntadr[body_id];
        int dof_adr = MuJoCo_helper->model->jnt_dofadr[joint_id];
        for(int i = 0; i < 3; i++) {
            if(body.active_linear_dof[i]) {
                state_dof_adr_indices.push_back(dof_adr + i);
            }
        }

        for(int i = 0; i < 3; i++){
            if(body.active_angular_dof[i]){
                state_dof_adr_indices.push_back(dof_adr + 3 + i);
            }
        }
    }

    for(auto & soft_body: state_vector.soft_bodies){
        for(int i = 0; i < soft_body.num_vertices; i++){
            for(int j = 0; j < 3; j++){
                if(soft_body.vertices[i].active_linear_dof[j]){

                    // TODO - validate this is correct, hastily written
                    int flex_id = mj_name2id(MuJoCo_helper->model, mjOBJ_FLEX, soft_body.name.c_str());
                    int first_vertex_adr = MuJoCo_helper->model->flex_vertadr[flex_id];

                    int body_id = MuJoCo_helper->model->flex_vertbodyid[first_vertex_adr + i];
                    int joint_index = MuJoCo_helper->model->body_jntadr[body_id];
                    const int start = MuJoCo_helper->model->jnt_dofadr[joint_index];

                    state_dof_adr_indices.push_back(start + j);
                }
            }
        }
    }
//
//    std::string state_name = state_vector.state_names[state_index];
//    int joint_index;
//    int body_index_offset = 0;
//    std::string rigid_body_tags[6]{"_x", "_y", "_z", "_roll", "_pitch", "_yaw"};
//    bool found_rigid_body_tag = false;
//
//    // Determine whether the state_name is a robot, rigid body or soft body
//
//    // -------------------------- Soft body checker ---------------------------------
//    // First check for '_V', which determins whether its a soft body.
//    // TODO - fix this, it seems to be working, but im not sure
//    size_t found = state_name.find('_V');
//    // If we find _V, its a soft body
//    if(found != std::string::npos){
//
//        // Remove _V_{x,y,z}
//        std::string flex_name = state_name.substr(0, found);
//
//        // --------- Compute vertex number -------------
//        // Find the position of the second underscore
//        size_t secondUnderscorePos = state_name.find('_', found + 2);
//
//        std::string numberString = state_name.substr(found + 1, secondUnderscorePos - (found + 1));
//        int vertex_number = std::atoi(numberString.c_str());
//
//        // Get flex id
//        int flex_id = mj_name2id(MuJoCo_helper->model, mjOBJ_FLEX, flex_name.c_str());
//        int first_vertex_adr = MuJoCo_helper->model->flex_vertadr[flex_id];
//
//        int body_id = MuJoCo_helper->model->flex_vertbodyid[first_vertex_adr + vertex_number];
//        joint_index = MuJoCo_helper->model->body_jntadr[body_id];
//        const int start = MuJoCo_helper->model->jnt_dofadr[joint_index];
//
//        // offset index {x, y, z}
//        for(int i = 0; i < 3; i++){
//            found_rigid_body_tag = endsWith(state_name, rigid_body_tags[i]);
//
//            if(found_rigid_body_tag){
//                body_index_offset = i;
//                break;
//            }
//        }
//
//        return start + body_index_offset;
//    }
//
//
//    // ----------------------------- Rigid body checker --------------------------------
//    for(int i = 0; i < 6; i++){
//        found_rigid_body_tag = endsWith(state_name, rigid_body_tags[i]);
//
//        if(found_rigid_body_tag){
//            // Remove body tag from string
//            state_name.erase(state_name.length() - rigid_body_tags[i].length(), rigid_body_tags[i].length());
//            body_index_offset = i;
//            break;
//        }
//    }
//
//    if(found_rigid_body_tag){
//        // Remove body tag suffix
//        int bodyId = mj_name2id(MuJoCo_helper->model, mjOBJ_BODY, state_name.c_str());
//        joint_index = MuJoCo_helper->model->jnt_dofadr[MuJoCo_helper->model->body_jntadr[bodyId]];
//
//        return joint_index + body_index_offset;
//    }
//
//    // if not a soft or rigid body, its a robot joint
//    return mj_name2id(MuJoCo_helper->model, mjOBJ_JOINT, state_name.c_str());


//    std::string joint_name;
//
//    for(int i = 0; i < 6; i++){
//        found_rigid_body_tag = endsWith(state_name, body_tags[i]);
//
//        if(found_rigid_body_tag){
//            // Remove body tag from string
//            state_name.erase(state_name.length() - body_tags[i].length(), body_tags[i].length());
//            body_index_offset = i;
//            break;
//        }
//    }
//
//    if(found_rigid_body_tag){
//        int bodyId = mj_name2id(MuJoCo_helper->model, mjOBJ_BODY, state_name.c_str());
//        joint_index = MuJoCo_helper->model->jnt_dofadr[MuJoCo_helper->model->body_jntadr[bodyId]];
//    }
//    else{
//        joint_index = mj_name2id(MuJoCo_helper->model, mjOBJ_JOINT, state_name.c_str());
//    }
//
//    return joint_index + body_index_offset;
}

int ModelTranslator::StateIndexToQposIndex(int state_index, const struct stateVectorList &state_vector){
    return state_dof_adr_indices[state_index];
}

void ModelTranslator::InitialiseSystemToStartState(mjData *d) {

    // ----------- Reset other variables of the simulation to zero ----------------
    // Reset time of simulation
    d->time = 0.0;

    for(int i = 0; i < MuJoCo_helper->model->nq; i++){
        d->qpos[i] = 0.0;
    }

    for(int i = 0; i < MuJoCo_helper->model->nv; i++){
        d->qvel[i] = 0.0;
        d->qacc[i] = 0.0;
        d->qacc_warmstart[i] = 0.0;
        d->qfrc_applied[i] = 0.0;
    }

    // Reset the control vector to zero
    for(int i = 0; i < MuJoCo_helper->model->nu; i++){
        d->ctrl[i] = 0.0;
    }

    for(int i = 0; i < 6*MuJoCo_helper->model->nbody; i++){
        d->xfrc_applied[i] = 0.0;
    }
    // -------------------------------------------------------------------------

    // Set goal visuals, functions needs to be overwritten by task implementation file if desired
    SetGoalVisuals(d);

    // Initialise robot positions to start configuration
    for(auto & robot : full_state_vector.robots){
        std::vector<double> zero_robot_velocities(robot.joint_names.size(), 0.0);
        MuJoCo_helper->SetRobotJointPositions(robot.name, robot.start_pos, d);
        MuJoCo_helper->SetRobotJointsVelocities(robot.name, zero_robot_velocities, d);
    }

    // Initialise rigid body poses to start configuration
    for(auto & rigid_body : full_state_vector.rigid_bodies){

        pose_6 body_pose;
        pose_6 body_vel;

        for(int i = 0; i < 3; i++){
            body_pose.position[i] = rigid_body.start_linear_pos[i];
            body_pose.orientation[i] = rigid_body.start_angular_pos[i];

            body_vel.position[i] = 0.0;
            body_vel.orientation[i] = 0.0;
        }

        MuJoCo_helper->SetBodyPoseAngle(rigid_body.name, body_pose, d);
        MuJoCo_helper->SetBodyVelocity(rigid_body.name, body_vel, d);
    }

    // Initialise soft body poses to start configuration
//    for(auto & soft_body : full_state_vector.soft_bodies){
//        pose_6 body_pose;
//
//        for(int i = 0; i < 3; i++){
//            body_pose.position[i] = soft_body.start_linear_pos[i];
//            body_pose.orientation[i] = soft_body.start_angular_pos[i];
//        }
//        std::cout << "soft body x: " << full_state_vector.soft_bodies[0].start_linear_pos[0] << " y: " << full_state_vector.soft_bodies[0].start_linear_pos[1] << "\n";
//
//        // TODO - better way to do this where we use the spacing information and transforms
//        for(int i = 0; i < soft_body.num_vertices; i++){
//            MuJoCo_helper->SetSoftBodyVertexPos(soft_body.name, i, body_pose, d);
//        }
//    }
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