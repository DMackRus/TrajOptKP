#include "GenTestingData.h"

GenTestingData::GenTestingData(std::shared_ptr<Optimiser> optimiser_,
                               std::shared_ptr<ModelTranslator> activeModelTranslator_,
                               std::shared_ptr<Differentiator> activeDifferentiator_,
                               std::shared_ptr<Visualiser> activeVisualiser_,
                               std::shared_ptr<FileHandler> yamlReader_) {

    optimiser = optimiser_;
    activeModelTranslator = activeModelTranslator_;
    activeDifferentiator = activeDifferentiator_;
    activeVisualiser = activeVisualiser_;
    yamlReader = yamlReader_;

    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

//    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);
//    optimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper,
//                                       activeDifferentiator, activeModelTranslator->openloop_horizon, activeVisualiser,
//                                       yamlReader);
}

int GenTestingData::gen_data_async_mpc(int task_horizon, int task_timeout){

    std::cout << "beginning testing asynchronus MPC for " << activeModelTranslator->model_name << std::endl;
    std::cout << "optimisation horizon is: " << task_horizon << " task timeout : " << task_timeout << "\n";

    keypoint_method keypoint_method;
    keypoint_method.name = "set_interval";
    keypoint_method.min_N = 1;
    keypoint_method.max_N = 1;
    keypoint_method.auto_adjust = false;

    testing_asynchronus_mpc(keypoint_method, 2, task_horizon, task_timeout);


//    std::vector<int> minN = {1};
//    std::vector<int> maxN_multiplier = {20};
//    std::vector<double> velocity_change_thresholds = {0.01, 0.1, 0.5, 1.0, 2.0};
//
//    for(int i = 0; i < minN.size(); i++){
//        for(int j = 0; j < maxN_multiplier.size(); j++){
//            for(int k = 0; k < velocity_change_thresholds.size(); k++){
//                keypoint_method keypoint_method;
//                keypoint_method.name = "velocity_change";
//                keypoint_method.min_N = minN[i];
//                keypoint_method.max_N = minN[i] * maxN_multiplier[j];
//                for(int l = 0; l < activeModelTranslator->dof; l++){
//                    keypoint_method.velocity_change_thresholds.push_back(velocity_change_thresholds[k]);
//                }
//                std::cout << "testing keypoint method: " << keypoint_method.name << " with minN: " << keypoint_method.min_N
//                          << " maxN: " << keypoint_method.max_N << " and velocity change thresholds: " << keypoint_method.velocity_change_thresholds[0] << std::endl;
//
//                testing_asynchronus_mpc(keypoint_method, 100);
//            }
//        }
//    }

//    keypoint_method keypoint_method;
//
//    int num_trials = 20;
//
//    keypoint_method.name = "adaptive_jerk";
//    keypoint_method.min_N = 1;
//    keypoint_method.max_N = 50;
//    keypoint_method.auto_adjust = true;
//    for(int i = 0; i < activeModelTranslator->dof; i++){
//        keypoint_method.jerk_thresholds.push_back(1e-15);
//    }
//    testing_asynchronus_mpc(keypoint_method, num_trials, task_horizon, task_timeout);
//
//    // Test set interval methods
//    keypoint_method.name = "set_interval";
//    keypoint_method.max_N = 1;
//    keypoint_method.auto_adjust = false;
//    std::vector<int> minNs = {1, 2, 5, 10, 20, 40, 60, 80, 150};
////    std::vector<int> minNs = {10, 110};
//    for(int minN : minNs){
//        // Only test for minN's < task_horizon
//        if(minN <= task_horizon){
//            keypoint_method.min_N = minN;
//            testing_asynchronus_mpc(keypoint_method, num_trials, task_horizon, task_timeout);
//        }
//    }



//    std::vector<int> minN = {1};
//    std::vector<int> maxN_multiplier = {50};
//    std::vector<double> jerk_thresholds = {0.0001, 0.001, 0.01, 0.1, 1.0};
//
//    for(int i = 0; i < minN.size(); i++){
//        for(int j = 0; j < maxN_multiplier.size(); j++){
//            for(int k = 0; k < jerk_thresholds.size(); k++){
//                keypoint_method.name = "adaptive_jerk";
//                keypoint_method.min_N = minN[i];
//                keypoint_method.max_N = minN[i] * maxN_multiplier[j];
//                for(int l = 0; l < activeModelTranslator->dof; l++){
//                    keypoint_method.jerk_thresholds.push_back(jerk_thresholds[k]);
//                }
//                std::cout << "testing keypoint method: " << keypoint_method.name << " with minN: " << keypoint_method.min_N
//                          << " maxN: " << keypoint_method.max_N << " and jerk thresholds: " << keypoint_method.jerk_thresholds[0] << std::endl;
//
//                testing_asynchronus_mpc(keypoint_method, 100);
//            }
//        }
//    }

    return EXIT_SUCCESS;
}

int GenTestingData::testing_asynchronus_mpc(keypoint_method keypoint_method, int num_trials, int task_horizon, int task_timeout){

    // ------------------ make method name ------------------
    std::string method_name;
    if(keypoint_method.auto_adjust){
        method_name = "AA_" + std::to_string(keypoint_method.min_N) + "_" + std::to_string(keypoint_method.max_N);
    }
    else{
        if(keypoint_method.name == "set_interval") {
            method_name = "SI_" + std::to_string(keypoint_method.min_N);
        }
        else if(keypoint_method.name == "velocity_change"){
            int substring_length = 3;
            if(keypoint_method.velocity_change_thresholds[0] < 0.1){
                substring_length = 4;
            }
            method_name = "VC_" +
                          std::to_string(keypoint_method.min_N) + "_" +
                          std::to_string(keypoint_method.max_N) + "_" +
                          std::to_string(keypoint_method.velocity_change_thresholds[0]).substr(0, substring_length);
        }
        else if(keypoint_method.name == "adaptive_jerk"){
            int substring_length = 4;
            if(keypoint_method.jerk_thresholds[0] <= 0.001){
                substring_length = 5;
            }
            if(keypoint_method.jerk_thresholds[0] <= 0.0001){
                substring_length = 6;
            }
            method_name = "AJ_" +
                          std::to_string(keypoint_method.min_N) + "_" +
                          std::to_string(keypoint_method.max_N) + "_" +
                          std::to_string(keypoint_method.jerk_thresholds[0]).substr(0, substring_length);
        }
    }

    std::string task_prefix = activeModelTranslator->model_name;

    // Make sure directories exist
    std::string projectParentPath = __FILE__;
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));

    std::string optimiser_name = optimiser->ReturnName();
    std::string rootPath = projectParentPath + "/testingData/" + optimiser_name + "/" + task_prefix +  "_" + std::to_string(task_horizon);

    // Check if task directory exists, if not create it
    if (!filesystem::exists(rootPath)) {
        if (!filesystem::create_directories(rootPath)) {
            std::cerr << "Failed to create directory: " << rootPath << std::endl;
        }
    }

    // Check if method directory exists, if not create it
    std::string method_directory = rootPath + "/" + method_name;
    if (!filesystem::exists(method_directory)) {
        if (!filesystem::create_directories(method_directory)) {
            std::cerr << "Failed to create directory: " << method_directory << std::endl;
            exit(1);
        }
    }

    // ------------------------- data storage -------------------------------------
    std::vector<double> finalCostsRow;

    std::vector<double> finalDistRow;

    std::vector<double> finalDofRow;

    std::vector<double> avgOptTimesRow;

    std::vector<double> avgPercentDerivsRow;

    std::vector<double> avgTimeForDerivsRow;

    std::vector<double> avgTimeBPRow;

    std::vector<double> avgTimeFPRow;

    std::vector<double> avgSurpriseRow;
    // -----------------------------------------------------------------------------

    auto startTimer = std::chrono::high_resolution_clock::now();
    optimiser->verbose_output = false;

    optimiser->SetCurrentKeypointMethod(keypoint_method);

    for (int i = 0; i < num_trials; i++) {
        std::cout << "trial: " << i << "\n";
        optimiser->keypoint_generator->ResetCache();
        // Load start and desired state from csv file

        yamlReader->loadTaskFromFile(task_prefix, i, activeModelTranslator->full_state_vector);
        activeModelTranslator->full_state_vector.Update();
        activeModelTranslator->current_state_vector = activeModelTranslator->full_state_vector;
        activeModelTranslator->UpdateSceneVisualisation();
        activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);

        std::cout << "current state vector: " << activeModelTranslator->current_state_vector.dof << " " << activeModelTranslator->current_state_vector.num_ctrl << "\n";
        std::cout << "full state vector: " << activeModelTranslator->full_state_vector.dof << " " << activeModelTranslator->full_state_vector.num_ctrl << "\n";

        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
        if(!activeModelTranslator->MuJoCo_helper->CheckIfDataIndexExists(0)){
            activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
        }

        std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

        // Perform the optimisation MPC test here asynchronously
        // Reset gravity back to normal
//        activeModelTranslator->MuJoCo_helper->model->opt.gravity[2] = -9.81;
        single_asynchronus_run(true, method_directory, i, task_horizon, task_timeout);
        stop_opt_thread = false;

        // ------------------------- data storage -------------------------------------
        finalCostsRow.push_back(final_cost);
        finalDistRow.push_back(final_dist);
        finalDofRow.push_back(average_dof);
        avgOptTimesRow.push_back(average_opt_time_ms);
        avgPercentDerivsRow.push_back(average_percent_derivs);
        avgTimeForDerivsRow.push_back(average_time_derivs_ms);
        avgTimeBPRow.push_back(average_time_bp_ms);
        avgTimeFPRow.push_back(average_time_fp_ms);
        avgSurpriseRow.push_back(average_surprise);
    }

    // ----------------------- Save data to file -------------------------------------
    std::string filename = method_directory + "/summary.csv";
    std::cout << "file_name: " << filename << std::endl;

    ofstream file_output;
    file_output.open(filename);

    // Make header
    file_output << "Final cost" << "," << "Final dist" << "," << "Average dofs" << "," << "Average optimisation time (ms)" << "," << "Average percent derivs" << ",";
    file_output << "Average time derivs (ms)" << "," << "Average time BP (ms)" << "," << "Average time FP (ms)" << "," << "Average surprise" << std::endl;

    // Loop through rows
    for(int i = 0; i < num_trials; i++){
        file_output << finalCostsRow[i] << "," << finalDistRow[i] << "," << avgOptTimesRow[i] << "," << avgPercentDerivsRow[i] << ",";
        file_output << avgTimeForDerivsRow[i] << "," << avgTimeBPRow[i] << "," << avgTimeFPRow[i] << "," <<  avgSurpriseRow[i] << std::endl;
    }

    file_output.close();

    return 1;
}

int GenTestingData::single_asynchronus_run(bool visualise,
                                           const std::string method_directory,
                                           int task_number,
                                           int task_horizon,
                                           const int TASK_TIMEOUT){

    activeVisualiser->trajectory_controls.clear();
    activeVisualiser->trajectory_states.clear();

    std::thread MPC_controls_thread;
    // Start the thread running
    MPC_controls_thread = std::thread(&GenTestingData::asynchronus_optimiser_worker, this, method_directory, task_number, task_horizon);

    int vis_counter = 0;
    MatrixXd next_control;

    // timer variables
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;

    int task_time = 0;

    while(task_time < TASK_TIMEOUT){
        begin = std::chrono::steady_clock::now();

        if(async_mpc || (!async_mpc && apply_next_control)){

            if(!async_mpc){
                apply_next_control = false;
            }

            if(activeVisualiser->current_control_index < activeVisualiser->controlBuffer.size()){

                next_control = activeVisualiser->controlBuffer[activeVisualiser->current_control_index];
                // Increment the current control index
                activeVisualiser->current_control_index++;
            }
            else{
                std::vector<double> grav_compensation;
                std::string robot_name = activeModelTranslator->current_state_vector.robots[0].name;
                activeModelTranslator->MuJoCo_helper->GetRobotJointsGravityCompensationControls(robot_name, grav_compensation,
                                                                                                activeModelTranslator->MuJoCo_helper->vis_data);
                MatrixXd empty_control(activeModelTranslator->current_state_vector.num_ctrl, 1);
//                empty_control.setZero();
                for(int i = 0; i < activeModelTranslator->current_state_vector.num_ctrl; i++){
                    empty_control(i) = grav_compensation[i];
                }
                next_control = empty_control;
            }

            if(APPLY_NOISE){
                for(int i = 0; i < activeModelTranslator->current_state_vector.num_ctrl; i++){
                    double gauss_noise = GaussNoise(0, 0.1);
                    next_control(i, 0) += gauss_noise;
                }
            }

//            std::cout << "next control is: " << next_control.transpose() << "\n";

            // Store latest control and state in a replay buffer
            activeVisualiser->trajectory_controls.push_back(next_control);
            MatrixXd next_state = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector);
            activeVisualiser->trajectory_states.push_back(next_state);

            // Set the latest control
            activeModelTranslator->SetControlVector(next_control, activeModelTranslator->MuJoCo_helper->vis_data,
                                                    activeModelTranslator->current_state_vector);

            // Update the simulation
            mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
            task_time++;

            // Check if task complete
            double dist;
            if(activeModelTranslator->TaskComplete(activeModelTranslator->MuJoCo_helper->vis_data, dist)){
                cout << "task complete - dist: " << dist  << endl;
                break;
            }
        }

        // Update the visualisation
        // Unsure why rendering every time causes it to lag so much more???
        vis_counter++;
        if(vis_counter > 5 && visualise){
            activeVisualiser->render("live-MPC");
            vis_counter = 0;
        }

        end = std::chrono::steady_clock::now();
        // time taken
        auto time_taken = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

        // compare how long we took versus the timestep of the model
        int difference_ms = (activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep() * 1000) - (time_taken / 1000.0f) + 1;

        if(difference_ms > 0) {
//                difference_ms = 20;
            std::this_thread::sleep_for(std::chrono::milliseconds(difference_ms));
        }
    }

    // Stop MPC thread
    {
        std::unique_lock<std::mutex> lock(mtx);
        stop_opt_thread = true;
    }

    MPC_controls_thread.join();

    final_cost = 0.0;
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){
        activeModelTranslator->SetControlVector(activeVisualiser->trajectory_controls[i], activeModelTranslator->MuJoCo_helper->vis_data,
                                                activeModelTranslator->current_state_vector);
        activeModelTranslator->SetStateVector(activeVisualiser->trajectory_states[i], activeModelTranslator->MuJoCo_helper->vis_data,
                                              activeModelTranslator->full_state_vector);
        activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
        final_cost += activeModelTranslator->CostFunction(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector, false);

    }

    std::cout << "final cost of entire MPC trajectory was: " << final_cost << "\n";
    std::cout << "avg opt time: " << average_opt_time_ms << " ms \n";
    std::cout << "avg num dofs: " << average_dof << "\n";
    std::cout << "avg percent derivs: " << average_percent_derivs << " % \n";
    std::cout << "avg time derivs: " << average_time_derivs_ms << " ms \n";
    std::cout << "avg time BP: " << average_time_bp_ms << " ms \n";
    std::cout << "avg time FP: " << average_time_fp_ms << " ms \n";


    // --------------------------------------------------------------------------------------------

//    activeVisualiser->trajectory_controls.clear();
//    activeVisualiser->trajectory_states.clear();
//
//    // Make a thread for the Optimiser
//    std::thread MPC_controls_thread;
//    // Start the thread running
//    MPC_controls_thread = std::thread(&GenTestingData::asynchronus_optimiser_worker, this, method_directory, task_number, task_horizon);
//    int vis_counter = 0;
//    MatrixXd next_control;
//    // timer variables
//    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
//    std::chrono::steady_clock::time_point end;
//
//    // elapsed task time
//    int task_time = 0;
//
//    activeModelTranslator->MuJoCo_helper->vis_data->time = 0.0f;
//
//    while(task_time++ < TASK_TIMEOUT){
//        begin = std::chrono::steady_clock::now();
//
//        if(activeVisualiser->current_control_index < activeVisualiser->controlBuffer.size()){
//
//            next_control = activeVisualiser->controlBuffer[activeVisualiser->current_control_index];
//            // Increment the current control index
//            activeVisualiser->current_control_index++;
//        }
//        else{
//            MatrixXd empty_control(activeModelTranslator->num_ctrl, 1);
//            empty_control.setZero();
//            next_control = empty_control;
//        }
//
//        if(APPLY_NOISE){
//            for(int i = 0; i < activeModelTranslator->num_ctrl; i++){
//                double gauss_noise = GaussNoise(0, 0.1);
//                next_control(i, 0) += gauss_noise;
//            }
//        }
//
//        // Store latest control and state in a replay buffer
//        activeVisualiser->trajectory_controls.push_back(next_control);
//        MatrixXd next_state = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data,
//                                                                       activeModelTranslator->full_state_vector);
//        activeVisualiser->trajectory_states.push_back(next_state);
//
//        // Set the latest control
//        activeModelTranslator->SetControlVector(next_control, activeModelTranslator->MuJoCo_helper->vis_data,
//                                                activeModelTranslator->current_state_vector);
//
//        // Update the simulation
//        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
//
////        std::cout << "time: " << activeModelTranslator->MuJoCo_helper->vis_data->time << std::endl;
//
//        double dist;
//        if(activeModelTranslator->TaskComplete(activeModelTranslator->MuJoCo_helper->vis_data, dist)){
//            std::cout << "Task complete" << std::endl;
//            break;
//        }
//
//        // Update the visualisation
//        // Unsure why rendering every time causes it to lag so much more???
//        vis_counter++;
//        if(vis_counter > 5 && visualise){
//            activeVisualiser->render("live-MPC");
//            vis_counter = 0;
//        }
//
//        end = std::chrono::steady_clock::now();
//        // time taken
//        auto time_taken = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
//
//        // compare how long we took versus the timestep of the model
//        int difference_ms = (activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep() * 1000) - (time_taken / 1000.0f) + 1;
//
//        if(difference_ms > 0) {
//            std::this_thread::sleep_for(std::chrono::milliseconds(difference_ms));
//        }
////        else
////            std::cout << "visualisation took " << (time_taken / 1000.0f) << " ms, longer than time-step, skipping sleep \n";
//
//
//        // Testing condition - change gravity halfway through task.
////        if(task_time == 1000){
////            activeModelTranslator->MuJoCo_helper->model->opt.gravity[2] = -13;
////        }
//    }
//
//    // Store final distance in final_dist
//    activeModelTranslator->TaskComplete(activeModelTranslator->MuJoCo_helper->vis_data, final_dist);
//
//    std::mutex mtx;
//    mtx.lock();
//    stop_opt_thread = true;
//    mtx.unlock();
//    MPC_controls_thread.join();
//
//    final_cost = 0.0;
//    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
//    for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){
//        activeModelTranslator->SetControlVector(activeVisualiser->trajectory_controls[i], activeModelTranslator->MuJoCo_helper->vis_data,
//                                                activeModelTranslator->current_state_vector);
//        activeModelTranslator->SetStateVector(activeVisualiser->trajectory_states[i], activeModelTranslator->MuJoCo_helper->vis_data,
//                                              activeModelTranslator->full_state_vector);
////        activeModelTranslator->MuJoCo_helper->forwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
////
////        activeVisualiser->render("playback");
//
//        final_cost += activeModelTranslator->CostFunction(activeModelTranslator->MuJoCo_helper->vis_data,
//                                                          activeModelTranslator->full_state_vector, false);
//    }
//
//    std::cout << "final cost of entire MPC trajectory was: " << final_cost << "\n";
//    std::cout << "avg opt time: " << average_opt_time_ms << " ms \n";
//    std::cout << "avg percent derivs: " << average_percent_derivs << " % \n";
//    std::cout << "avg time derivs: " << average_time_derivs_ms << " ms \n";
//    std::cout << "avg time BP: " << average_time_bp_ms << " ms \n";
//    std::cout << "avg time FP: " << average_time_fp_ms << " ms \n";

    return EXIT_SUCCESS;
}

void GenTestingData::asynchronus_optimiser_worker(const std::string& method_directory, int task_number, int task_horizon){
    std::vector<double> time_iteration;
    std::vector<int> num_dofs;
    std::vector<double> time_get_derivs;
    std::vector<double> time_bp;
    std::vector<double> time_fp;
    std::vector<double> percent_derivs_computed;
    std::vector<double> surprise;
    std::vector<double> expected;
    std::vector<double> new_cost;

    std::vector<MatrixXd> optimised_controls;

    // Instantiate init controls
    std::vector<MatrixXd> init_opt_controls;

    // Create init optimisation controls and reset system state
    init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(task_horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);

    optimised_controls = optimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1, task_horizon);

    while(!stop_opt_thread){

        // Copy current state of system (vis data) to starting data object for optimisation
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->vis_data);

        // Get the current control index
        int current_control_index = activeVisualiser->current_control_index;

        // Delete all controls before control index
        optimised_controls.erase(optimised_controls.begin(), optimised_controls.begin() + current_control_index);

        // Copy last control to keep control trajectory the same size
        MatrixXd last_control = optimised_controls.back();
        for (int i = 0; i < current_control_index; ++i) {
            optimised_controls.push_back(last_control);
        }

        optimised_controls = optimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                 optimised_controls, 1, 1, task_horizon);

        // Store last iteration timing results
        time_iteration.push_back(optimiser->opt_time_ms);
        num_dofs.push_back(activeModelTranslator->current_state_vector.dof);
        time_get_derivs.push_back(optimiser->avg_time_get_derivs_ms);
        time_bp.push_back(optimiser->avg_time_backwards_pass_ms);
        time_fp.push_back(optimiser->avg_time_forwards_pass_ms);
        percent_derivs_computed.push_back(optimiser->avg_percent_derivs);
        surprise.push_back(0.0);
        expected.push_back(0.0);
        new_cost.push_back(0.0);

        int opt_time_to_timestep = optimiser->opt_time_ms / (activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep() * 1000);

        // Compute the best starting state
        double smallestError = 1000.00;
        int bestMatchingStateIndex = opt_time_to_timestep;
        if(bestMatchingStateIndex >= task_horizon){
            bestMatchingStateIndex = task_horizon - 1;
        }

        // Mutex lock
        {
            std::unique_lock<std::mutex> lock(mtx);

            activeVisualiser->controlBuffer = optimised_controls;

            // Set the current control index to the best matching state index
            activeVisualiser->current_control_index = bestMatchingStateIndex;

            apply_next_control = true;
        }

//        std::cout << "best matching state index: " << bestMatchingStateIndex << std::endl;
    }

    // Save specific trajectory data
    std::string filename = method_directory + "/" + std::to_string(task_number) + ".csv";

    ofstream file_output;
    file_output.open(filename);

    // Make header
    file_output << "Optimisation time (ms)" << "," << "num dofs" << "," << "% derivs" << ",";
    file_output << "derivs time (ms)" << "," << "BP time (ms)" << "," "FP time (ms)" << ",";
    file_output << "surprise" << "," << "expected" << "," << "new cost" << std::endl;

    // Loop through rows
    for(int i = 0; i < time_get_derivs.size(); i++){
        file_output << time_iteration[i] << "," << num_dofs[i] << "," << percent_derivs_computed[i] << ",";
        file_output << time_get_derivs[i] << "," << time_bp[i] << "," << time_fp[i] << ",";
        file_output << surprise[i] << "," << expected[i] << "," << new_cost[i] << std::endl;
    }

    file_output.close();

    average_opt_time_ms = 0.0;
    average_dof = 0.0;
    average_time_derivs_ms = 0.0;
    average_time_bp_ms = 0.0;
    average_time_fp_ms = 0.0;
    average_percent_derivs = 0.0;
    average_surprise = 0.0;

    for(int i = 0; i < time_get_derivs.size(); i++){
        average_opt_time_ms += time_iteration[i];
        average_dof += num_dofs[i];
        average_time_derivs_ms += time_get_derivs[i];
        average_time_bp_ms += time_bp[i];
        average_time_fp_ms += time_fp[i];
        average_percent_derivs += percent_derivs_computed[i];
        average_surprise += surprise[i];
    }

    average_opt_time_ms /= time_iteration.size();
    average_dof /= num_dofs.size();
    average_time_derivs_ms /= time_get_derivs.size();
    average_time_bp_ms /= time_bp.size();
    average_time_fp_ms /= time_fp.size();
    average_percent_derivs /= percent_derivs_computed.size();
    average_surprise /= surprise.size();

}

int GenTestingData::GenerateDynamicsDerivsData(int num_trajecs, int num_iters_per_task){
    std::cout << "generate dynamics derivs in loop \n";

    int count = 0;
    int file_num = 0;
    std::string task_name = activeModelTranslator->model_name;

    int optimisation_horizon = activeModelTranslator->openloop_horizon;

    while(count < num_trajecs){

        // Initialise system for task
        yamlReader->loadTaskFromFile(task_name, file_num, activeModelTranslator->current_state_vector);
        activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);

        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

        if(!activeModelTranslator->MuJoCo_helper->CheckIfDataIndexExists(0)){
            activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
        }

        std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

        // Create initial optimisation controls
        std::vector<MatrixXd> initOptimisationControls;
        std::vector<MatrixXd> optimised_controls;

        initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(optimisation_horizon);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

        optimised_controls = optimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], initOptimisationControls, 1, 1, optimisation_horizon);

        // Loop n times
        for(int i = 0; i < num_iters_per_task; i++){

            // Save Data (A, B, X, U)
            yamlReader->saveTrajecInfomation(optimiser->A, optimiser->B,
                                             optimiser->X_old, optimiser->U_old,
                                             task_name, count, optimisation_horizon);

            count++;

            optimised_controls = optimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                     optimised_controls, 1, 1, optimisation_horizon);

            std::cout << "optim iteration done " << count << " done \n";

        }

        file_num++;
    }

    return EXIT_SUCCESS;
}

int GenTestingData::GenerateTestScenes(int num_scenes){
    for(int i = 0; i < 200; i++){
        activeModelTranslator->GenerateRandomGoalAndStartState();
        activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->vis_data);
        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
        activeVisualiser->render("Generating random test scenes");
        yamlReader->saveTaskToFile(activeModelTranslator->model_name, i, activeModelTranslator->current_state_vector);
    }

    return EXIT_SUCCESS;
}