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

void FileHandler::ReadModelConfigFile(const std::string& yamlFilePath, task &_taskConfig){
    YAML::Node node = YAML::LoadFile(projectParentPath + yamlFilePath);

    // General task settings
    _taskConfig.model_filepath = projectParentPath + node["modelFile"].as<std::string>();
    _taskConfig.model_name = node["modelName"].as<std::string>();

    // model timestep
    if(node["timeStep"]){
        _taskConfig.model_time_step = node["timeStep"].as<double>();
    }
    else{
        _taskConfig.model_time_step = 0.004;
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
        string root_name;
        vector<string> jointNames;
        vector<string> actuatorNames;

        vector<double> startPos;

        vector<double> jointJerkThresholds;
        vector<double> magVelThresholds;

        robotName = robot_it->first.as<string>();

        if(robot_it->second["root_name"]){
            root_name = robot_it->second["root_name"].as<std::string>();

            // If the robot has a root, then we add 6 joint names
            // root_name_x, _y, _z, _roll, _pitch, _yaw]
//            std::string suffixes[6] = {"_x", "_y", "_z", "_roll", "_pitch", "_yaw"};
//            for(int i = 0; i < 6; i++){
//                std::string joint_name = robot_it->second["root_name"].as<std::string>() + suffixes[i];
//                jointNames.push_back(joint_name);
//            }
        }
        else{
            root_name = "-";
        }

        for(int i = 0; i < robot_it->second["jointNames"].size(); i++){
            jointNames.push_back(robot_it->second["jointNames"][i].as<std::string>());
        }

        for(int i = 0; i < robot_it->second["actuatorNames"].size(); i++){
            actuatorNames.push_back(robot_it->second["actuatorNames"][i].as<std::string>());
        }

        for(int i = 0; i < robot_it->second["startPos"].size(); i++){
            startPos.push_back(robot_it->second["startPos"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["jointJerkThresholds"].size(); i++){
            jointJerkThresholds.push_back(robot_it->second["jointJerkThresholds"][i].as<double>());
        }

        for(int i = 0; i < robot_it->second["magVelThresholds"].size(); i++){
            magVelThresholds.push_back(robot_it->second["magVelThresholds"][i].as<double>());
        }

        tempRobot.name = robotName;
        tempRobot.root_name = root_name;

        tempRobot.joint_names = jointNames;
        tempRobot.actuator_names = actuatorNames;

        tempRobot.start_pos = startPos;

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
            _rigid_body.linear_jerk_threshold[i] = linearJerkThreshold[i];
            _rigid_body.angular_jerk_threshold[i] = angularJerkThreshold[i];
            _rigid_body.linear_vel_change_threshold[i] = linearMagVelThreshold[i];
            _rigid_body.angular_vel_change_threshold[i] = angularMagVelThreshold[i];
            _rigid_body.base_color[0] = 0;
            _rigid_body.base_color[1] = 0;
            _rigid_body.base_color[2] = 0;
            _rigid_body.base_color[3] = 1;
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
        }

        _taskConfig.soft_bodies.push_back(_soft_body);
    }

    // Loop through residuals
    for(YAML::const_iterator residual_it=node["residuals"].begin(); residual_it!=node["residuals"].end(); ++residual_it) {
        residual _residual;

        _residual.name = residual_it->first.as<string>();
        for(int i = 0; i < residual_it->second["target"].size(); i++){
            _residual.target.push_back(residual_it->second["target"][i].as<double>());
        }

        if(residual_it->second["resid_dimension"]){
            _residual.resid_dimension = residual_it->second["resid_dimension"].as<int>();
        }
        else{
            _residual.resid_dimension = 1;
        }
        _residual.weight = residual_it->second["weight"].as<double>();
        _residual.weight_terminal = residual_it->second["weight_terminal"].as<double>();

        for(int i = 0; i < _residual.resid_dimension; i++){
            _taskConfig.residuals.push_back(_residual);
        }
    }
}

void FileHandler::ReadSettingsFile(const std::string& settingsFilePath){

    std::cout << "Reading settings file: " << projectParentPath + settingsFilePath << std::endl;
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

void FileHandler::SaveTrajecInformation(std::vector<MatrixXd> A_matrices, std::vector<MatrixXd> B_matrices,
                                        std::vector<MatrixXd> states, std::vector<MatrixXd> controls,
                                        std::string file_prefix){
    std::string rootPath = projectParentPath + "/savedTrajecInfo" + file_prefix;

    int horizon = A_matrices.size();

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
    for(int i = 0; i < horizon - 1; i++){
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

    for(int i = 0; i < horizon - 1; i++){
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
    for(int i = 0; i < horizon - 1; i++){
        for(int j = 0; j < (dof * 2); j++)
        {
            fileOutput << states[i](j) << ",";
        }
        fileOutput << endl;
    }

    fileOutput.close();

    filename = rootPath + "/controls.csv";
    fileOutput.open(filename);

    for(int i = 0; i < horizon - 1; i++){
        for(int j = 0; j < num_ctrl; j++) {
            fileOutput << controls[i](j) << ",";
        }
        fileOutput << endl;
    }

    fileOutput.close();
}

void FileHandler::SaveKeypointsToFile(std::string file_prefix, int file_num, const std::vector<std::vector<int>> &keypoints){
    std::string rootPath = projectParentPath + "/savedTrajecInfo" + file_prefix;

    std::string filename = rootPath + "/keypoints.csv";
    fileOutput.open(filename);

    if (!filesystem::exists(rootPath)) {
        if (!filesystem::create_directories(rootPath)) {
            std::cerr << "Failed to create directory: " << rootPath << std::endl;
            exit(1);
        }
    }

    int dof = keypoints[0].size();

    for(int i = 0; i < dof; i++){
        for(int j = 0; j < keypoints.size(); j++) {
            for(int k = 0; k < keypoints[j].size(); k++){
                if(keypoints[j][k] == i){
                    fileOutput << j << ",";
                    break;
                }
            }
        }
        fileOutput << endl;
    }

//    for(int i = 0; i < keypoints.size(); i++){
//        for(int j = 0; j < keypoints[i].size(); j++){
//            std::cout << "Keypoint: " << keypoints[i][j] << "\n";
//            fileOutput << keypoints[i][j] << ",";
//        }
//        fileOutput << endl;
//    }

    fileOutput.close();
}

void FileHandler::SaveTaskToFile(std::string file_prefix, int file_num, const stateVectorList &state_vector, const vector<residual> &residuals){

    std::string rootPath = projectParentPath + "/TestTasks/" + file_prefix;
    mkdir(rootPath.c_str(), 0777);

    std::string filename = rootPath + "/" + std::to_string(file_num) + ".csv";
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
    for( const auto& body : state_vector.rigid_bodies){
        for(double start_linear_po : body.start_linear_pos){
            fileOutput << start_linear_po << ",";
        }

        for(double start_angular_po : body.start_angular_pos){
            fileOutput << start_angular_po << ",";
        }
    }

    // Soft body poses
    for( const auto& soft_body : state_vector.soft_bodies){
        for(double start_linear_po : soft_body.start_linear_pos){
            fileOutput << start_linear_po << ",";
        }
    }

    // ----------------- Residual targets ----------------------
    for(auto & residual : residuals){
        for(double target : residual.target){
            fileOutput << target << ",";
        }
    }

    fileOutput << std::endl;
    fileOutput.close();
}

void FileHandler::LoadTaskFromFile(std::string task_prefix, int file_num, stateVectorList &state_vector, vector<residual> &residuals){

    std::string rootPath = projectParentPath + "/TestTasks" + task_prefix;
    mkdir(rootPath.c_str(), 0777);
    std::string filename = rootPath + "/" + std::to_string(file_num) + ".csv";

    fstream fin;

    // Check if file exists first
    if(!filesystem::exists(filename)){
        std::cerr << "File "<< filename << " does not exist\n";
        exit(1);
    }

    fin.open(filename, ios::in);
    std::vector<string> row;
    std::string line, word, temp;

    int num_dofs = 0;
    int residuals_targets_size = 0;
    // Compute size of the full state vector
    for(auto & robot : state_vector.robots){
        num_dofs += static_cast<int>(robot.joint_names.size());
    }

    // Loop through bodies
    for( auto body : state_vector.rigid_bodies){
        // x, y, z, r, p ,y
        num_dofs += 6;
    }

    for(auto soft_body : state_vector.soft_bodies){
        // x, y, z, r, p, y
        num_dofs += 6;
    }
    // ------------------------------------------------------------------------

    // ---------------- Compute residual sizes --------------------------------
    for(auto & residual : residuals){
        residuals_targets_size += residual.target.size();
    }
    // ------------------------------------------------------------------------

    // Only one row in this file so this while loop should only perform one iteration
    while(fin >> temp){
        row.clear();

        getline(fin, line);

        stringstream s(temp);

        while(getline(s, word, ',')){
            row.push_back(word);
        }

        // Start state and goal state
        if(row.size() != num_dofs + residuals_targets_size){
            std::cerr << "CSV file has " << row.size() << "elements, num dofs is: " <<
                num_dofs << "and resids target size is: " << residuals_targets_size << "\n";
            exit(1);
        }

        // ----------------- Start values --------------------------------
        int counter = 0;
        for(auto & robot : state_vector.robots){
            for(int i = 0; i < robot.joint_names.size(); i++){
                robot.start_pos[i] = stod(row[counter]);
                counter++;
            }
        }

        for(auto & body : state_vector.rigid_bodies){
            // Linear (x, y, z)
            for(int i = 0; i < 3; i++){
                body.start_linear_pos[i] = stod(row[counter]);
                counter++;
            }

            // Angular roll pitch yaw
            for(int i = 0; i < 3; i++){
                body.start_angular_pos[i] = stod(row[counter]);
                counter++;
            }
        }

        for(auto & soft_body : state_vector.soft_bodies){
            // General centroid things
            for(int i = 0; i < 3; i++) {
                soft_body.start_linear_pos[i] = stod(row[counter]);
                counter++;
            }

            for(int i = 0; i < 3; i++){
                soft_body.start_angular_pos[i] = stod(row[counter]);
                counter++;
            }
        }
        // -------------------------------------------------------------------

        // ----------------- Residual targets --------------------------------
        for(auto & residual : residuals){
            for(int i = 0; i < residual.target.size(); i++){
                residual.target[i] = stod(row[counter]);
                counter++;
            }
        }
    }
}

//void FileHandler::saveCostHistory(std::vector<double> costHistory, std::string filePrefix, int trajecNumber){
//    std::string rootPath = projectParentPath + "/testingData/" + filePrefix;
//    mkdir(rootPath.c_str(), 0777);
//    std::string filename = rootPath + std::to_string(trajecNumber) + ".csv";
//    fileOutput.open(filename);
//
//    for(int i = 0; i < costHistory.size(); i++){
//        fileOutput << costHistory[i] << ",";
//    }
//
//    fileOutput << std::endl;
//
//    fileOutput.close();
//}
//
//void FileHandler::saveResultsDataForMethods(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> optTimes,
//                                            std::vector<std::vector<double>> costReduction, std::vector<std::vector<double>> avgPercentageDerivs,
//                                            std::vector<std::vector<double>> avgTimeGettingDerivs, std::vector<std::vector<int>> numIterations){
//    std::string rootPath = projectParentPath;
//    std::string filename = rootPath + taskPrefix + "_testingData.csv";
//
//    fileOutput.open(filename);
//
//    // Make header
//    for(int i = 0; i < methodNames.size(); i++){
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//    }
//    fileOutput << std::endl;
//
//    for(int i = 0; i < methodNames.size(); i++){
//        fileOutput << "opt_time_ms" << ",";
//        fileOutput << "Cost reduction" << ",";
//        fileOutput << "avg_percent_derivs" << ",";
//        fileOutput << "avgTimeDerivs" << ",";
//        fileOutput << "numIterations" << ",";
//    }
//    fileOutput << std::endl;
//
//    int numTrajecs = optTimes.size();
//
//    for(int i = 0; i < numTrajecs; i++){
//        for(int j = 0; j < methodNames.size(); j++){
//            fileOutput << optTimes[i][j] << ",";
//            fileOutput << costReduction[i][j] << ",";
//            fileOutput << avgPercentageDerivs[i][j] << ",";
//            fileOutput << avgTimeGettingDerivs[i][j] << ",";
//            fileOutput << numIterations[i][j] << ",";
//
//        }
//        fileOutput << std::endl;
//    }
//    fileOutput.close();
//}
//
//void FileHandler::saveResultsData_MPC(std::string taskPrefix, std::vector<std::string> methodNames, std::vector<std::vector<double>> finalCosts, std::vector<std::vector<double>> avgHZ,
//                                      std::vector<std::vector<double>> avgTimeGettingDerivs, std::vector<std::vector<double>> avgTimeBP, std::vector<std::vector<double>> avgTimeFP, std::vector<std::vector<double>> avgPercentDerivs){
//    std::string rootPath = projectParentPath;
//    std::string filename = rootPath + taskPrefix + "_testingData.csv";
//
//    fileOutput.open(filename);
//
//    // Make header
//    for(int i = 0; i < methodNames.size(); i++){
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//        fileOutput << methodNames[i] << ",";
//    }
//    fileOutput << std::endl;
//
//    for(int i = 0; i < methodNames.size(); i++){
//        fileOutput << "final costs" << ",";
//        fileOutput << "avg Hz" << ",";
//        fileOutput << "avgTimeDerivs" << ",";
//        fileOutput << "avgTimeBP" << ",";
//        fileOutput << "avgTimeFP" << ",";
//        fileOutput << "avgpercent derivs" << ",";
//    }
//    fileOutput << std::endl;
//
//    int numTrajecs = finalCosts.size();
//    cout << "num trajecs: " << numTrajecs << endl;
//
//    for(int i = 0; i < numTrajecs; i++){
//        for(int j = 0; j < methodNames.size(); j++){
//            fileOutput << finalCosts[i][j] << ",";
//            fileOutput << avgHZ[i][j] << ",";
//            fileOutput << avgTimeGettingDerivs[i][j] << ",";
//            fileOutput << avgTimeBP[i][j] << ",";
//            fileOutput << avgTimeFP[i][j] << ",";
//            fileOutput << avgPercentDerivs[i][j] << ",";
//        }
//        fileOutput << std::endl;
//    }
//    fileOutput.close();
//}