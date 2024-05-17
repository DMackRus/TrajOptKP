#include "FileHandler.h"

FileHandler::FileHandler(){
    // Init Controls
    project_run_mode = "Init_controls";
    // Pendulum
    taskName = "double_pendulum";
    // interpolated iLQR
    optimiser = "interpolated_iLQR";

    minIter = 2;
    maxIter = 5;

    // Get project parent path
    projectParentPath = __FILE__;
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));

}

void FileHandler::readModelConfigFile(const std::string& yamlFilePath, task &_taskConfig){
    YAML::Node node = YAML::LoadFile(projectParentPath + yamlFilePath);

    // General task settings
    _taskConfig.modelFilePath = projectParentPath + node["modelFile"].as<std::string>();
    _taskConfig.modelName = node["modelName"].as<std::string>();

    // model timestep
    if(node["timeStep"]){
        _taskConfig.modelTimeStep = node["timeStep"].as<double>();
    }
    else{
        _taskConfig.modelTimeStep = 0.004;
    }

    // Open loop horizon
    if(node["openloop_horizon"]){
        _taskConfig.openloop_horizon = node["openloop_horizon"].as<int>();
    }
    else{
        _taskConfig.openloop_horizon = 3000;
    }

    // MPC horizon
    if(node["mpc_horizon"]){
        _taskConfig.mpc_horizon = node["mpc_horizon"].as<int>();
    }
    else{
        _taskConfig.mpc_horizon = 100;
    }

    _taskConfig.keypointMethod = node["keypointMethod"].as<std::string>();
    if(node["auto_adjust"]){
        _taskConfig.auto_adjust = node["auto_adjust"].as<bool>();
    }
    else{
        _taskConfig.auto_adjust = false;
    }

    // Min N
    if(node["minN"]){
        _taskConfig.minN = node["minN"].as<int>();
    }
    else{
        _taskConfig.minN = 1;
    }

    // Max error
    if(node["maxN"]){
        _taskConfig.maxN = node["maxN"].as<int>();
    }
    else{
        _taskConfig.maxN = 10;
    }

    if(node["iterativeErrorThreshold"]){
        _taskConfig.iterativeErrorThreshold = node["iterativeErrorThreshold"].as<double>();
    }
    else{
        _taskConfig.iterativeErrorThreshold = 10;
    }

    // Loop through robots
    for(YAML::const_iterator robot_it=node["robots"].begin(); robot_it!=node["robots"].end(); ++robot_it){
        robot tempRobot;
        string robotName;
        vector<string> jointNames;
        vector<string> actuatorNames;
        bool torqueControlled;
        vector<double> torqueLimits;
        vector<double> startPos;
        vector<double> goalPos;
        vector<double> goalVel;
        vector<double> jointPosCosts;
        vector<double> jointVelCosts;
        vector<double> terminalJointPosCosts;
        vector<double> terminalJointVelCosts;
        vector<double> jointControlCosts;
        vector<double> jointJerkThresholds;
        vector<double> magVelThresholds;

        robotName = robot_it->first.as<string>();

        for(int i = 0; i < robot_it->second["jointNames"].size(); i++){
            jointNames.push_back(robot_it->second["jointNames"][i].as<std::string>());
        }

        for(int i = 0; i < robot_it->second["actuatorNames"].size(); i++){
            actuatorNames.push_back(robot_it->second["actuatorNames"][i].as<std::string>());
        }

        torqueControlled = robot_it->second["torqueControl"].as<bool>();

        for(int i = 0; i < robot_it->second["torqueLimits"].size(); i++){
            torqueLimits.push_back(robot_it->second["torqueLimits"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["startPos"].size(); i++){
            startPos.push_back(robot_it->second["startPos"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["goalPos"].size(); i++){
            goalPos.push_back(robot_it->second["goalPos"][i].as<double>());
            // Always make this the same size as goal pos
            goalVel.push_back(0.0);
        }

        if(robot_it->second["goalVel"]){
            for(int i = 0; i < robot_it->second["goalVel"].size(); i++){
                goalVel[i] = robot_it->second["goalVel"][i].as<double>();
            }
        }


        for(int i = 0; i < robot_it->second["jointPosCosts"].size(); i++){
            jointPosCosts.push_back(robot_it->second["jointPosCosts"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["jointVelCosts"].size(); i++){
            jointVelCosts.push_back(robot_it->second["jointVelCosts"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["terminalJointPosCosts"].size(); i++){
            terminalJointPosCosts.push_back(robot_it->second["terminalJointPosCosts"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["terminalJointVelCosts"].size(); i++){
            terminalJointVelCosts.push_back(robot_it->second["terminalJointVelCosts"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["jointControlCosts"].size(); i++){
            jointControlCosts.push_back(robot_it->second["jointControlCosts"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["jointJerkThresholds"].size(); i++){
            jointJerkThresholds.push_back(robot_it->second["jointJerkThresholds"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["magVelThresholds"].size(); i++){
            magVelThresholds.push_back(robot_it->second["magVelThresholds"][i].as<double>());
        }

        tempRobot.name = robotName;
        tempRobot.joint_names = jointNames;
        tempRobot.actuator_names = actuatorNames;
        tempRobot.torque_controlled = torqueControlled;
        tempRobot.torque_limits = torqueLimits;
        tempRobot.start_pos = startPos;
        tempRobot.goal_pos = goalPos;
        tempRobot.goal_vel = goalVel;
        tempRobot.joint_pos_costs = jointPosCosts;
        tempRobot.jointVelCosts = jointVelCosts;
        tempRobot.terminal_joint_pos_costs = terminalJointPosCosts;
        tempRobot.terminal_joint_vel_costs = terminalJointVelCosts;
        tempRobot.joint_controls_costs = jointControlCosts;
        tempRobot.jerk_thresholds = jointJerkThresholds;
        tempRobot.vel_change_thresholds = magVelThresholds;

        _taskConfig.robots.push_back(tempRobot);
    }

    // Loop through rigid bodies
    for(YAML::const_iterator body_it=node["bodies"].begin(); body_it!=node["bodies"].end(); ++body_it) {
        rigid_body _rigid_body;
        std::string bodyName;
        bool activeLinearDOF[3];
        bool activeAngularDOF[3];
        double startLinearPos[3];
        double startAngularPos[3];
        double goalLinearPos[3];
        double goalAngularPos[3];
        double linearPosCosts[3];
        double terminalLinearPosCosts[3];
        double linearVelCosts[3];
        double terminalLinearVelCosts[3];
        double angularPosCosts[3];
        double terminalAngularPosCosts[3];
        double angularVelCosts[3];
        double terminalAngularVelCosts[3];
        double linearJerkThreshold[3];
        double angularJerkThreshold[3];
        double linearMagVelThreshold[3];
        double angularMagVelThreshold[3];

        bodyName = body_it->first.as<string>();

        for(int i = 0; i < body_it->second["activeLinearDOF"].size(); i++){
            activeLinearDOF[i] = body_it->second["activeLinearDOF"][i].as<bool>();
        }

        for(int i = 0; i < body_it->second["activeAngularDOF"].size(); i++){
            activeAngularDOF[i] = body_it->second["activeAngularDOF"][i].as<bool>();
        }

        for(int i = 0; i < body_it->second["startLinearPos"].size(); i++){
            startLinearPos[i] = body_it->second["startLinearPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["startAngularPos"].size(); i++){
            startAngularPos[i] = body_it->second["startAngularPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["goalLinearPos"].size(); i++){
            goalLinearPos[i] = body_it->second["goalLinearPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["goalAngularPos"].size(); i++){
            goalAngularPos[i] = body_it->second["goalAngularPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearPosCost"].size(); i++){
            linearPosCosts[i] = body_it->second["linearPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalLinearPosCost"].size(); i++){
            terminalLinearPosCosts[i] = body_it->second["terminalLinearPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearVelCost"].size(); i++){
            linearVelCosts[i] = body_it->second["linearVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalLinearVelCost"].size(); i++){
            terminalLinearVelCosts[i] = body_it->second["terminalLinearVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularPosCost"].size(); i++){
            angularPosCosts[i] = body_it->second["angularPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalAngularPosCost"].size(); i++){
            terminalAngularPosCosts[i] = body_it->second["terminalAngularPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularVelCost"].size(); i++){
            angularVelCosts[i] = body_it->second["angularVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalAngularVelCost"].size(); i++){
            terminalAngularVelCosts[i] = body_it->second["terminalAngularVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearJerkThreshold"].size(); i++){
            linearJerkThreshold[i] = body_it->second["linearJerkThreshold"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularJerkThreshold"].size(); i++){
            angularJerkThreshold[i] = body_it->second["angularJerkThreshold"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearMagVelThreshold"].size(); i++){
            linearMagVelThreshold[i] = body_it->second["linearMagVelThreshold"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularMagVelThreshold"].size(); i++){
            angularMagVelThreshold[i] = body_it->second["angularMagVelThreshold"][i].as<double>();
        }

        _rigid_body.name = bodyName;
        for(int i = 0; i < 3; i++){
            _rigid_body.active_linear_dof[i] = activeLinearDOF[i];
            _rigid_body.active_angular_dof[i] = activeAngularDOF[i];
            _rigid_body.start_linear_pos[i] = startLinearPos[i];
            _rigid_body.start_angular_pos[i] = startAngularPos[i];
            _rigid_body.goal_linear_pos[i] = goalLinearPos[i];
            _rigid_body.goal_angular_pos[i] = goalAngularPos[i];
            _rigid_body.linearPosCost[i] = linearPosCosts[i];
            _rigid_body.terminal_linear_pos_cost[i] = terminalLinearPosCosts[i];
            _rigid_body.linear_vel_cost[i] = linearVelCosts[i];
            _rigid_body.terminal_linear_vel_cost[i] = terminalLinearVelCosts[i];
            _rigid_body.angular_pos_cost[i] = angularPosCosts[i];
            _rigid_body.terminal_angular_pos_cost[i] = terminalAngularPosCosts[i];
            _rigid_body.angular_vel_cost[i] = angularVelCosts[i];
            _rigid_body.terminal_angular_vel_cost[i] = terminalAngularVelCosts[i];
            _rigid_body.linear_jerk_threshold[i] = linearJerkThreshold[i];
            _rigid_body.angular_jerk_threshold[i] = angularJerkThreshold[i];
            _rigid_body.linear_vel_change_threshold[i] = linearMagVelThreshold[i];
            _rigid_body.angular_vel_change_threshold[i] = angularMagVelThreshold[i];
        }
        _taskConfig.rigid_bodies.push_back(_rigid_body);
    }

    // Loop through soft bodies
    for(YAML::const_iterator body_it=node["soft_bodies"].begin(); body_it!=node["soft_bodies"].end(); ++body_it) {
        soft_body _soft_body;
        std::string bodyName;
        int num_vertices;
        std::vector<vertex> vertices;
        double startLinearPos[3];
        double startAngularPos[3];
        double goalLinearPos[3];
        double goalAngularPos[3];
        double linearPosCosts[3];
        double terminalLinearPosCosts[3];
        double linearVelCosts[3];
        double terminalLinearVelCosts[3];
        double angularPosCosts[3];
        double terminalAngularPosCosts[3];
        double angularVelCosts[3];
        double terminalAngularVelCosts[3];

        bodyName = body_it->first.as<string>();
        num_vertices = body_it->second["num_vertices"].as<int>();

        for(int i = 0; i < num_vertices; i++){
            vertex _vertex{};
            for(int j = 0; j < 3; j++){
                _vertex.active_linear_dof[j] = body_it->second["activeLinearDOF"][j].as<bool>();

                _vertex.linear_jerk_threshold[j] = body_it->second["linearMagVelThreshold"][j].as<double>();
                _vertex.linear_vel_change_threshold[j] = body_it->second["angularMagVelThreshold"][j].as<double>();
            }

            vertices.push_back(_vertex);
        }

        for(int i = 0; i < body_it->second["startLinearPos"].size(); i++){
            startLinearPos[i] = body_it->second["startLinearPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["startAngularPos"].size(); i++){
            startAngularPos[i] = body_it->second["startAngularPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["goalLinearPos"].size(); i++){
            goalLinearPos[i] = body_it->second["goalLinearPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["goalAngularPos"].size(); i++){
            goalAngularPos[i] = body_it->second["goalAngularPos"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearPosCost"].size(); i++){
            linearPosCosts[i] = body_it->second["linearPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalLinearPosCost"].size(); i++){
            terminalLinearPosCosts[i] = body_it->second["terminalLinearPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["linearVelCost"].size(); i++){
            linearVelCosts[i] = body_it->second["linearVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalLinearVelCost"].size(); i++){
            terminalLinearVelCosts[i] = body_it->second["terminalLinearVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularPosCost"].size(); i++){
            angularPosCosts[i] = body_it->second["angularPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalAngularPosCost"].size(); i++){
            terminalAngularPosCosts[i] = body_it->second["terminalAngularPosCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["angularVelCost"].size(); i++){
            angularVelCosts[i] = body_it->second["angularVelCost"][i].as<double>();
        }

        for(int i = 0; i < body_it->second["terminalAngularVelCost"].size(); i++){
            terminalAngularVelCosts[i] = body_it->second["terminalAngularVelCost"][i].as<double>();
        }

        _soft_body.name = bodyName;
        _soft_body.num_vertices = num_vertices;

        // Soft body vertices
        for(int i = 0; i < num_vertices; i++){
            _soft_body.vertices.push_back(vertices[i]);
        }

        // General centroid of soft body qualities
        for(int i = 0; i < 3; i++){
            _soft_body.start_linear_pos[i] = startLinearPos[i];
            _soft_body.start_angular_pos[i] = startAngularPos[i];
            _soft_body.goal_linear_pos[i] = goalLinearPos[i];
            _soft_body.goal_angular_pos[i] = goalAngularPos[i];
            _soft_body.linearPosCost[i] = linearPosCosts[i];
            _soft_body.terminal_linear_pos_cost[i] = terminalLinearPosCosts[i];
            _soft_body.linear_vel_cost[i] = linearVelCosts[i];
            _soft_body.terminal_linear_vel_cost[i] = terminalLinearVelCosts[i];
            _soft_body.angular_pos_cost[i] = angularPosCosts[i];
            _soft_body.terminal_angular_pos_cost[i] = terminalAngularPosCosts[i];
            _soft_body.angular_vel_cost[i] = angularVelCosts[i];
            _soft_body.terminal_angular_vel_cost[i] = terminalAngularVelCosts[i];
        }

        _taskConfig.soft_bodies.push_back(_soft_body);
    }
}

void FileHandler::readSettingsFile(const std::string& settingsFilePath){

    YAML::Node node = YAML::LoadFile(projectParentPath + settingsFilePath);

    optimiser = node["optimiser"].as<std::string>();
    project_run_mode = node["runMode"].as<std::string>();
    taskName = node["task"].as<std::string>();
    taskInitMode = node["taskInitMode"].as<std::string>();
    csvRow = node["csvRow"].as<int>();
    filtering = node["filtering"].as<std::string>();
    costDerivsFD = node["costDerivsFD"].as<bool>();

    minIter = node["minIter"].as<int>();
    maxIter = node["maxIter"].as<int>();

    async_mpc = node["async_mpc"].as<bool>();
    record_trajectory = node["record"].as<bool>();
}

void FileHandler::saveTrajecInfomation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices, std::vector<MatrixXd> states, std::vector<MatrixXd> controls, std::string filePrefix, int trajecNumber, int horizonLength){
    std::string rootPath = projectParentPath + "/savedTrajecInfo" + filePrefix + "/" + std::to_string(trajecNumber);

    if (!filesystem::exists(rootPath)) {
        if (!filesystem::create_directories(rootPath)) {
            std::cerr << "Failed to create directory: " << rootPath << std::endl;
            exit(1);
        }
    }

    std::string filename = rootPath + "/A_matrices.csv";

    fileOutput.open(filename);
    int dof = A_matrices[0].rows() / 2;
    int num_ctrl = B_matrices[0].cols();

    // trajectory length
    for(int i = 0; i < horizonLength - 1; i++){
        // Row
        for(int j = 0; j < (2 * dof); j++){
            // Column
            for(int k = 0; k < (2 * dof); k++){
                fileOutput << A_matrices[i](j, k) << ",";
            }

        }
        fileOutput << endl;
    }
    fileOutput.close();

    filename = rootPath + "/B_matrices.csv";
    fileOutput.open(filename);

    for(int i = 0; i < horizonLength - 1; i++){
        for(int j = 0; j < (2 * dof); j++){
            for(int k = 0; k < num_ctrl; k++){
                fileOutput << B_matrices[i](j, k) << ",";
            }
        }
        fileOutput << endl;
    }

    fileOutput.close();

    filename = rootPath + "/states.csv";
    fileOutput.open(filename);
    for(int i = 0; i < horizonLength - 1; i++){
        for(int j = 0; j < (dof * 2); j++)
        {
            fileOutput << states[i](j) << ",";
        }
        fileOutput << endl;
    }

    fileOutput.close();

    filename = rootPath + "/controls.csv";
    fileOutput.open(filename);

    for(int i = 0; i < horizonLength - 1; i++){
        for(int j = 0; j < num_ctrl; j++) {
            fileOutput << controls[i](j) << ",";
        }
        fileOutput << endl;
    }

    fileOutput.close();
}

void FileHandler::saveTaskToFile(std::string taskPrefix, int fileNum, const stateVectorList &state_vector){

    std::string rootPath = projectParentPath + "/testTasks/" + taskPrefix;
    mkdir(rootPath.c_str(), 0777);

    std::string filename = rootPath + "/" + std::to_string(fileNum) + ".csv";
    fileOutput.open(filename);

    // file save data should be start state and then desired state
    // State should be robot pos, body pose, robot vel, body vel

    // -------------------- Start values --------------------------------
    // Robot positions
    for(auto & robot : state_vector.robots){
        for(int i = 0; i < robot.joint_names.size(); i++){
            fileOutput << robot.start_pos[i] << ",";
        }
    }

    // Body poses
    for( auto body : state_vector.rigid_bodies){
        for(int i = 0; i < 3; i++){
            fileOutput << body.start_linear_pos[i] << ",";
        }

        for(int i = 0; i < 3; i++){
            fileOutput << body.start_angular_pos[i] << ",";
        }
    }

    // Robot velocities
    for(auto & robot : state_vector.robots){
        for(int i = 0; i < robot.joint_names.size(); i++){
            fileOutput << 0 << ",";
        }
    }

    // Body velocities
    for( auto body : state_vector.rigid_bodies){
        for(int i = 0; i < 3; i++){
            fileOutput << 0 << ",";
        }

        for(int i = 0; i < 3; i++){
            fileOutput << 0 << ",";
        }
    }

    // ----------------- Goal values ---------------------------
    for(auto & robot : state_vector.robots){
        for(int i = 0; i < robot.joint_names.size(); i++){
            fileOutput << robot.goal_pos[i] << ",";
        }
    }

    // Body poses
    for( auto body : state_vector.rigid_bodies){
        for(int i = 0; i < 3; i++){
            fileOutput << body.goal_linear_pos[i] << ",";
        }

        for(int i = 0; i < 3; i++){
            fileOutput << body.goal_angular_pos[i] << ",";
        }
    }

    // Robot velocities
    for(auto & robot : state_vector.robots){
        for(int i = 0; i < robot.joint_names.size(); i++){
            fileOutput << robot.goal_vel[i] << ",";
        }
    }

    // Body velocities
    for( auto body : state_vector.rigid_bodies){
        for(int i = 0; i < 3; i++){
            fileOutput << 0 << ",";
        }

        for(int i = 0; i < 3; i++){
            fileOutput << 0 << ",";
        }
    }

    fileOutput << std::endl;

    fileOutput.close();

}

void FileHandler::loadTaskFromFile(std::string taskPrefix, int fileNum, stateVectorList &state_vector){

    std::string rootPath = projectParentPath + "/testTasks" + taskPrefix;
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + "/" + std::to_string(fileNum) + ".csv";

    fstream fin;

    fin.open(filename, ios::in);
    std::vector<string> row;
    std::string line, word, temp;

    // Sanity check to compute state vector size and compare it with file size
    // Loop through robots
    int state_vector_size = 0;
    for(auto & robot : state_vector.robots){
        state_vector_size += 2 * static_cast<int>(robot.joint_names.size());
    }

    // Loop through bodies
    for( auto body : state_vector.rigid_bodies){
        // x, y, z, r, p, y and *2 for velocities
        state_vector_size += 12;
    }

    // Only one row in this file so this while loop should only perform one iteration
    while(fin >> temp){
        row.clear();

        getline(fin, line);

        stringstream s(temp);

        while(getline(s, word, ',')){
            row.push_back(word);
        }

        // Start state and goal state
        if(row.size() != 2 * state_vector_size){
            std::cerr << "CSV file has " << row.size() << "elements, state vector size is: " << state_vector_size << "\n";
            exit(1);
        }

        int counter = 0;
        for(auto & robot : state_vector.robots){
            for(int i = 0; i < robot.joint_names.size(); i++){
                robot.start_pos[i] = stod(row[counter]);
                robot.goal_pos[i] = stod(row[counter + state_vector_size]);
                robot.goal_vel[i] = stod(row[counter + state_vector_size + (state_vector_size / 2)]);
                counter++;
            }
        }

        for(auto & body : state_vector.rigid_bodies){
            // Linear (x, y, z)
            for(int i = 0; i < 3; i++){
                body.start_linear_pos[i] = stod(row[counter]);
                body.goal_linear_pos[i] = stod(row[counter + state_vector_size]);
                counter++;
            }

            // Angular roll pitch yaw
            for(int i = 0; i < 3; i++){
                body.start_angular_pos[i] = stod(row[counter]);
                body.goal_angular_pos[i] = stod(row[counter + state_vector_size]);
                counter++;
            }
        }
    }
}

void FileHandler::saveCostHistory(std::vector<double> costHistory, std::string filePrefix, int trajecNumber){
    std::string rootPath = projectParentPath + "/testingData/" + filePrefix;
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + std::to_string(trajecNumber) + ".csv";
    fileOutput.open(filename);

    for(int i = 0; i < costHistory.size(); i++){
        fileOutput << costHistory[i] << ",";
    }

    fileOutput << std::endl;

    fileOutput.close();
}

void FileHandler::saveResultsDataForMethods(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> optTimes,
                                            std::vector<std::vector<double>> costReduction, std::vector<std::vector<double>> avgPercentageDerivs,
                                            std::vector<std::vector<double>> avgTimeGettingDerivs, std::vector<std::vector<int>> numIterations){
    std::string rootPath = projectParentPath;
    std::string filename = rootPath + taskPrefix + "_testingData.csv";

    fileOutput.open(filename);

    // Make header
    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
    }
    fileOutput << std::endl;

    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << "opt_time_ms" << ",";
        fileOutput << "Cost reduction" << ",";
        fileOutput << "avg_percent_derivs" << ",";
        fileOutput << "avgTimeDerivs" << ",";
        fileOutput << "numIterations" << ",";
    }
    fileOutput << std::endl;

    int numTrajecs = optTimes.size();

    for(int i = 0; i < numTrajecs; i++){
        for(int j = 0; j < methodNames.size(); j++){
            fileOutput << optTimes[i][j] << ",";
            fileOutput << costReduction[i][j] << ",";
            fileOutput << avgPercentageDerivs[i][j] << ",";
            fileOutput << avgTimeGettingDerivs[i][j] << ",";
            fileOutput << numIterations[i][j] << ",";

        }
        fileOutput << std::endl;
    }
    fileOutput.close();
}

void FileHandler::saveResultsData_MPC(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> finalCosts, std::vector<std::vector<double>> avgHZ,
                                      std::vector<std::vector<double>> avgTimeGettingDerivs, std::vector<std::vector<double>> avgTimeBP, std::vector<std::vector<double>> avgTimeFP, std::vector<std::vector<double>> avgPercentDerivs){
    std::string rootPath = projectParentPath;
    std::string filename = rootPath + taskPrefix + "_testingData.csv";

    fileOutput.open(filename);

    // Make header
    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
        fileOutput << methodNames[i] << ",";
    }
    fileOutput << std::endl;

    for(int i = 0; i < methodNames.size(); i++){
        fileOutput << "final costs" << ",";
        fileOutput << "avg Hz" << ",";
        fileOutput << "avgTimeDerivs" << ",";
        fileOutput << "avgTimeBP" << ",";
        fileOutput << "avgTimeFP" << ",";
        fileOutput << "avgpercent derivs" << ",";
    }
    fileOutput << std::endl;

    int numTrajecs = finalCosts.size();
    cout << "num trajecs: " << numTrajecs << endl;

    for(int i = 0; i < numTrajecs; i++){
        for(int j = 0; j < methodNames.size(); j++){
            fileOutput << finalCosts[i][j] << ",";
            fileOutput << avgHZ[i][j] << ",";
            fileOutput << avgTimeGettingDerivs[i][j] << ",";
            fileOutput << avgTimeBP[i][j] << ",";
            fileOutput << avgTimeFP[i][j] << ",";
            fileOutput << avgPercentDerivs[i][j] << ",";
        }
        fileOutput << std::endl;
    }
    fileOutput.close();
}