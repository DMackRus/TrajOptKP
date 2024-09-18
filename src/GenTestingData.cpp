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

    activeDifferentiator = activeDifferentiator_;
}

int GenTestingData::GenDataOpenLoopMultipleMethods(int task_horizon){
    int tests_fine = EXIT_SUCCESS;
    int this_test_fine = EXIT_SUCCESS;

    keypoint_method keypoint_method = optimiser->ReturnCurrentKeypointMethod();

    // Testing loop to test when velocity change thresholds stop being effective
    keypoint_method.name = "velocity_change";
    keypoint_method.min_N = 1;
    keypoint_method.max_N = 1000;
//    vector<double> vel_change_thresholds = {0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0, 50.0, 100.0, 500.0, 1000.0};
    vector<double> vel_change_thresholds = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    for(int i = 0; i < vel_change_thresholds.size(); i++){
        keypoint_method.velocity_change_thresholds[0] = vel_change_thresholds[i];
        keypoint_method.velocity_change_thresholds[1] = vel_change_thresholds[i];

        optimiser->SetCurrentKeypointMethod(keypoint_method);

        this_test_fine = GenDataOpenloopOptimisation(task_horizon);
        if(this_test_fine != EXIT_SUCCESS){
            tests_fine = this_test_fine;
        }
        // Sleep for 60 seconds - enforces file name change for different tests
        std::this_thread::sleep_for(std::chrono::seconds(55));
    }

    // ----------------- Set interval 1 -------------------
//    keypoint_method.name = "set_interval";
//    keypoint_method.min_N = 1;
//    keypoint_method.max_N = 1;
//
//    // Set the keypoint method
//    optimiser->SetCurrentKeypointMethod(keypoint_method);
//
//    this_test_fine = GenDataOpenloopOptimisation(task_horizon);
//    if(this_test_fine != EXIT_SUCCESS){
//        tests_fine = this_test_fine;
//    }
//    // Sleep for 60 seconds - enforces file name change for different tests
//    std::this_thread::sleep_for(std::chrono::seconds(60));

    // ----------------- Set interval 5 ---------------------
//    keypoint_method.name = "set_interval";
//    keypoint_method.min_N = 5;
//    keypoint_method.max_N = 1;
//
//    // Set the keypoint method
//    optimiser->SetCurrentKeypointMethod(keypoint_method);
//
//    this_test_fine = GenDataOpenloopOptimisation(task_horizon);
//    if(this_test_fine != EXIT_SUCCESS){
//        tests_fine = this_test_fine;
//    }
//    // Sleep for 60 seconds - enforces file name change for different tests
//    std::this_thread::sleep_for(std::chrono::seconds(60));
//    // ----------------- Set interval 1000 ---------------------
//    keypoint_method.name = "set_interval";
//    keypoint_method.min_N = 1000;
//    keypoint_method.max_N = 1;
//
//    // Set the keypoint method
//    optimiser->SetCurrentKeypointMethod(keypoint_method);
//
//    this_test_fine = GenDataOpenloopOptimisation(task_horizon);
//    if(this_test_fine != EXIT_SUCCESS){
//        tests_fine = this_test_fine;
//    }
//    // Sleep for 60 seconds - enforces file name change for different tests
//    std::this_thread::sleep_for(std::chrono::seconds(30));
//
//    // ----------------- Adaptive jerk 1 50 ---------------------
//    keypoint_method.name = "adaptive_jerk";
//    keypoint_method.min_N = 1;
//    keypoint_method.max_N = 10;
//
//    // Set the keypoint method
//    optimiser->SetCurrentKeypointMethod(keypoint_method);
//
//    this_test_fine = GenDataOpenloopOptimisation(task_horizon);
//    if(this_test_fine != EXIT_SUCCESS){
//        tests_fine = this_test_fine;
//    }
    // Sleep for 60 seconds - enforces file name change for different tests
//    std::this_thread::sleep_for(std::chrono::seconds(60));

//     ----------------- Velocity change 1 50 -------------------
//    keypoint_method.name = "velocity_change";
//    keypoint_method.min_N = 1;
//    keypoint_method.max_N = 100;
//
//    // Set the keypoint method
//    optimiser->SetCurrentKeypointMethod(keypoint_method);
//
//    this_test_fine = GenDataOpenloopOptimisation(task_horizon);
//    if(this_test_fine != EXIT_SUCCESS){
//        tests_fine = this_test_fine;
//    }
//    // Sleep for 60 seconds - enforces file name change for different tests
//    std::this_thread::sleep_for(std::chrono::seconds(60));

//     ----------------- Iterative error 1 50 -------------------
//    keypoint_method.name = "iterative_error";
//    keypoint_method.min_N = 1;
//    keypoint_method.max_N = 50;
//
//    // Set the keypoint method
//    optimiser->SetCurrentKeypointMethod(keypoint_method);
//
//    this_test_fine = GenDataOpenloopOptimisation(task_horizon);
//    if(this_test_fine != EXIT_SUCCESS){
//        tests_fine = this_test_fine;
//    }
//
//    return tests_fine;
}

int GenTestingData::GenDataOpenloopOptimisation(int task_horizon){
    std::cout << "begining testing openloop optimisation for " << activeModelTranslator->model_name << std::endl;
    std::cout << "optimisation horizon is: " << task_horizon << std::endl;

    // --------------------------- Data we want to save ------------------------------
    // Individual trajectory information, including;
    // New cost, iteration time, dofs, % derivs, time derivs, time bp, time fp

    // Summary file over all N trajectories, with:
    // Cost reduction, optimisation time, num iterations, avg dofs, avg %derivs, avg time derivs, avg time bp, avg time fp,

    // Create the file directory root path dynamically
    std::string method_directory = CreateTestName("openloop");

    // ------------------------- data storage -------------------------------------
    std::vector<double> cost_reductions;
    std::vector<double> optimisation_times;
    std::vector<int>    num_iterations;
    std::vector<double> avg_num_dofs;
    std::vector<double> avg_percent_derivs;
    std::vector<double> total_time_derivs;
    std::vector<double> total_time_bp;
    std::vector<double> total_time_fp;
    // -----------------------------------------------------------------------------

    auto startTimer = std::chrono::high_resolution_clock::now();
    optimiser->verbose_output = true;

    for (int i = 0; i < 100; i++) {
        std::cout << "trial: " << i << "\n";

        // Reset internal optimisation data and clear key-points cache
        optimiser->Reset();
        optimiser->keypoint_generator->ResetCache();
        // Load start and desired state from csv file

        // Load the task from CSV file
        yamlReader->LoadTaskFromFile(activeModelTranslator->model_name, i, activeModelTranslator->full_state_vector, activeModelTranslator->residual_list);

        // Reset state vector (only really applicable for iLQR_SVR method)
        activeModelTranslator->ResetSVR();
        activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);

        // Setup mj data objects
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                              activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                              activeModelTranslator->MuJoCo_helper->master_reset_data);

//        MatrixXd test_state_start = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->master_reset_data,
//                                                                             activeModelTranslator->full_state_vector);
//        std::cout << "state vector after initialised: " << test_state_start.transpose() << "\n";

        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
//        test_state_start = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->master_reset_data,
//                                                                    activeModelTranslator->full_state_vector);
//        std::cout << "state vector after step: " << test_state_start.transpose() << "\n";

        if (!activeModelTranslator->MuJoCo_helper->CheckIfDataIndexExists(0)) {
            activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(
                    activeModelTranslator->MuJoCo_helper->master_reset_data);
        }

        // Perform any setup controls for this task
        std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data,
                                                              activeModelTranslator->MuJoCo_helper->main_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                              activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                              activeModelTranslator->MuJoCo_helper->master_reset_data);

        // Create init optimisation controls
        std::vector<MatrixXd> init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(task_horizon);
        activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                              activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->CopySystemState(
                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                activeModelTranslator->MuJoCo_helper->master_reset_data);

        // Do the optimisation!
        optimiser->lambda = 0.01;
        std::vector<MatrixXd> optimised_controls = optimiser->Optimise(
                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 10, 3,
                task_horizon);


        // ------------------------- Update the data storages -------------------------------------
        cost_reductions.push_back(optimiser->cost_reduction);
        optimisation_times.push_back(optimiser->opt_time_ms);
        num_iterations.push_back(optimiser->num_iterations);
        avg_num_dofs.push_back(optimiser->avg_dofs);
        avg_percent_derivs.push_back(optimiser->avg_percent_derivs);
        total_time_derivs.push_back(std::accumulate(optimiser->time_get_derivs_ms.begin(), optimiser->time_get_derivs_ms.end(), 0));
        total_time_bp.push_back(std::accumulate(optimiser->time_backwards_pass_ms.begin(), optimiser->time_backwards_pass_ms.end(), 0));
        total_time_fp.push_back(std::accumulate(optimiser->time_forwardsPass_ms.begin(), optimiser->time_forwardsPass_ms.end(), 0));
    }

    // ----------------------- Save data to file -------------------------------------
    std::string filename = method_directory + "/summary.csv";

    ofstream file_output;
    file_output.open(filename);

    // Make header
    file_output << "Cost reduction" << "," << "Optimisation time (ms)" << "," << "Number iterations" << ",";
    file_output << "Average num dofs" << "," << "Average percent derivs" << "," << "Average time derivs (ms)" << ",";
    file_output << "Average time BP (ms)" << "," << "Average time FP (ms)" << std::endl;

    // Loop through rows
    for(int i = 0; i < cost_reductions.size(); i++){
        file_output << cost_reductions[i] << "," << optimisation_times[i] << "," << num_iterations[i] << ",";
        file_output << avg_num_dofs[i] << "," << avg_percent_derivs[i] << "," << total_time_derivs[i] << ",";
        file_output << total_time_bp[i] << "," << total_time_fp[i] << std::endl;
    }

    file_output.close();

    SaveTestSummaryData(optimiser->activeKeyPointMethod, task_horizon,
                        controls_noise, optimiser->ReturnName(),
                        method_directory);

    return EXIT_SUCCESS;
}

int GenTestingData::GenDataAsyncMPC(int task_horizon, int task_timeout){

    std::cout << "beginning testing asynchronus MPC for " << activeModelTranslator->model_name << std::endl;
    std::cout << "optimisation horizon is: " << task_horizon << " task timeout : " << task_timeout << "\n";

    keypoint_method keypoint_method;
    keypoint_method.name = "set_interval";
    keypoint_method.min_N = 1;
    keypoint_method.max_N = 1;
    keypoint_method.auto_adjust = false;

    return TestingMPC(keypoint_method, true, 100, task_horizon, task_timeout);
}

int GenTestingData::GenDataMPCHorizons(int task_timeout){
    std::cout << "Beginning testing, synchronous MPC for " << activeModelTranslator->model_name << std::endl;

    std::vector<int> horizons = {20, 30, 40, 50, 60, 70, 80};

    // Load keypoint method
    keypoint_method keypoint_method = optimiser->ReturnCurrentKeypointMethod();
    int num_trials = 100;

    // -------------------- Set interval 1 ----------------------------
    keypoint_method.name = "set_interval";
    keypoint_method.min_N = 1;
    for(int horizon : horizons){
        TestingMPC(keypoint_method, false, num_trials, horizon, task_timeout);
    }
//
//    // ------------------- Set interval 5 -----------------------------
    keypoint_method.min_N = 5;
    for(int horizon : horizons){
        TestingMPC(keypoint_method, false, num_trials, horizon, task_timeout);
    }
//
//    // ------------------- Set interval 20 ----------------------------
    keypoint_method.min_N = 20;
    for(int horizon : horizons){
        TestingMPC(keypoint_method, false, num_trials, horizon, task_timeout);
    }
//
//    // ------------------- Adaptive Jerk ------------------------------
    keypoint_method.name = "adaptive_jerk";
    keypoint_method.min_N = 2;
    keypoint_method.max_N = 20;
    for(int horizon : horizons){
        TestingMPC(keypoint_method, false, num_trials, horizon, task_timeout);
    }
//
//    // ------------------- Velocity change ----------------------------
    keypoint_method.name = "velocity_change";
    keypoint_method.min_N = 2;
    keypoint_method.max_N = 20;
    for(int horizon : horizons){
        TestingMPC(keypoint_method, false, num_trials, horizon, task_timeout);
    }

    // ------------------- Iterative error ----------------------------
    keypoint_method.name = "iterative_error";
    keypoint_method.min_N = 2;
    for(int horizon : horizons){
        TestingMPC(keypoint_method, false, num_trials, horizon, task_timeout);
    }

}

int GenTestingData::TestingMPC(const keypoint_method& keypoint_method, bool asynchronus,
                                   int num_trials, int task_horzion, int task_timeout){

    std::string method_directory;
    if(asynchronus){
        method_directory = CreateTestName("asynchronus_mpc");
    }
    else{
        method_directory = CreateTestName("synchronus_mpc");
    }

    // ------------------------- data storage -------------------------------------
    std::vector<double> final_costs;
    std::vector<double> final_distances;
    std::vector<double> final_num_dofs;
    std::vector<double> avg_opt_time;
    std::vector<double> avg_percent_derivs;
    std::vector<double> avg_time_derivs;
    std::vector<double> avg_time_bp;
    std::vector<double> avg_time_fp;
    std::vector<double> avg_surprise;
    // -----------------------------------------------------------------------------

    auto startTimer = std::chrono::high_resolution_clock::now();
    optimiser->verbose_output = true;

    optimiser->SetCurrentKeypointMethod(keypoint_method);

    for (int i = 0; i < num_trials; i++) {
        std::cout << "trial: " << i << "\n";
        optimiser->keypoint_generator->ResetCache();
        // Load start and desired state from csv file

        yamlReader->LoadTaskFromFile(activeModelTranslator->model_name, i, activeModelTranslator->full_state_vector, activeModelTranslator->residual_list);
        activeModelTranslator->ResetSVR();
        std::cout << "current state vector sizes: \n";
        std::cout << activeModelTranslator->current_state_vector.dof << " " << activeModelTranslator->current_state_vector.num_ctrl << "\n";
        activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);


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
        SingleMPCRun(true, asynchronus, method_directory, i, task_horzion, task_timeout);

        // ------------------------- data storage -------------------------------------
        final_costs.push_back(final_cost);
        final_distances.push_back(final_dist);
        final_num_dofs.push_back(average_dof);
        avg_opt_time.push_back(average_opt_time_ms);
        avg_percent_derivs.push_back(average_percent_derivs);
        avg_time_derivs.push_back(average_time_derivs_ms);
        avg_time_bp.push_back(average_time_bp_ms);
        avg_time_fp.push_back(average_time_fp_ms);
        avg_surprise.push_back(average_surprise);
    }

    // ----------------------- Save data to file -------------------------------------
    std::string filename = method_directory + "/summary.csv";

    ofstream file_output;
    file_output.open(filename);

    // Make header
    file_output << "Final cost" << "," << "Final dist" << "," << "Average dofs" << "," << "Average optimisation time (ms)" << "," << "Average percent derivs" << ",";
    file_output << "Average time derivs (ms)" << "," << "Average time BP (ms)" << "," << "Average time FP (ms)" << "," << "Average surprise" << std::endl;

    // Loop through rows
    for(int i = 0; i < num_trials; i++){
        file_output << final_costs[i] << "," << final_distances[i] << "," << final_num_dofs[i] << "," << avg_opt_time[i] << "," << avg_percent_derivs[i] << ",";
        file_output << avg_time_derivs[i] << "," << avg_time_bp[i] << "," << avg_time_fp[i] << "," << avg_surprise[i] << std::endl;
    }

    file_output.close();

    SaveTestSummaryData(keypoint_method, task_horzion,
                        controls_noise, optimiser->ReturnName(),
                        method_directory);

    return EXIT_SUCCESS;
}

int GenTestingData::SingleMPCRun(bool visualise, bool asynchronus,
                                         const std::string& method_directory,
                                         int task_number,
                                         int task_horizon,
                                         const int TASK_TIMEOUT){

    activeVisualiser->trajectory_controls.clear();
    activeVisualiser->trajectory_states.clear();
    activeVisualiser->controlBuffer.clear();
    activeVisualiser->current_control_index = 0;
    stop_opt_thread = false;
    apply_next_control = false;
    reoptimise = true;

    std::thread MPC_controls_thread;
    // Start the thread running
    MPC_controls_thread = std::thread(&GenTestingData::AsyncronusMPCWorker, this, method_directory, task_number, task_horizon);

    int vis_counter = 0;
    MatrixXd next_control;

    // timer variables
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;

    int task_time = 0;

    while(task_time < TASK_TIMEOUT){
        begin = std::chrono::steady_clock::now();

        if(asynchronus || (!asynchronus && apply_next_control)){

            if(activeVisualiser->current_control_index < num_controls_apply && activeVisualiser->current_control_index < activeVisualiser->controlBuffer.size()){

                next_control = activeVisualiser->controlBuffer[activeVisualiser->current_control_index];
                // Increment the current control index
                activeVisualiser->current_control_index++;

                if(activeVisualiser->current_control_index >= num_steps_replan){
                    // Applied correct number of controls so lets replan
                    {
                        std::unique_lock<std::mutex> lock(mtx);
                        reoptimise = true;
                        if(!asynchronus){
                            apply_next_control = false;
                        }
                    }
                }

                MatrixXd control_lims = activeModelTranslator->ReturnControlLimits(activeModelTranslator->current_state_vector);
                for(int i = 0; i < activeModelTranslator->current_state_vector.num_ctrl; i++){
                    double control_noise = ((control_lims(i*2 + 1) - control_lims(i*2)) / 100) * controls_noise;

                    double gauss_noise = GaussNoise(0, control_noise);
                    next_control(i, 0) += gauss_noise;
                }
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

//            for(int i = 0; i < activeModelTranslator->current_state_vector.num_ctrl; i++){
//                double gauss_noise = GaussNoise(0, controls_noise);
//                next_control(i, 0) += gauss_noise;
//            }

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
            if(activeModelTranslator->TaskComplete(activeModelTranslator->MuJoCo_helper->vis_data, final_dist)){
                cout << "task complete - dist: " << final_dist  << endl;
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
//            difference_ms += 16;
            std::this_thread::sleep_for(std::chrono::milliseconds(difference_ms));
        }
    }

    // Stop MPC thread
    {
        std::unique_lock<std::mutex> lock(mtx);
        stop_opt_thread = true;
        std::cout << "stop opt thread called \n";
    }

    MPC_controls_thread.join();

    // NOTE - we change cost function of push soft to track how well we managed to push the soft body.
    // These cost function elements dont work in normal traj opt for some reason, so we counte this
    // by using a terminal position cost, however this then isnt trakced by our evaluation
//    if(1){
//        for(int i = 0; i < activeModelTranslator->full_state_vector.soft_bodies[0].num_vertices; i++){
//            activeModelTranslator->full_state_vector.soft_bodies[0].linearPosCost[0] = 1;
//            activeModelTranslator->full_state_vector.soft_bodies[0].linearPosCost[0] = 1;
//        }
//    }

//    if(1){
//        for(auto& rigid_body : activeModelTranslator->full_state_vector.rigid_bodies){
//            rigid_body.terminal_linear_pos_cost[0] = 1000;
//            rigid_body.terminal_linear_pos_cost[1] = 1000;
//        }
//    }

    final_cost = 0.0;
    bool terminal = false;
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){
        activeModelTranslator->SetControlVector(activeVisualiser->trajectory_controls[i], activeModelTranslator->MuJoCo_helper->vis_data,
                                                activeModelTranslator->full_state_vector);
        activeModelTranslator->SetStateVector(activeVisualiser->trajectory_states[i], activeModelTranslator->MuJoCo_helper->vis_data,
                                              activeModelTranslator->full_state_vector);
        activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);

        if(i == activeVisualiser->trajectory_states.size() - 1){
            terminal = true;
        }

        MatrixXd residuals(activeModelTranslator->residual_list.size(), 1);
        activeModelTranslator->Residuals(activeModelTranslator->MuJoCo_helper->vis_data, residuals);
        final_cost += activeModelTranslator->CostFunction(residuals,
                                          activeModelTranslator->full_state_vector, terminal);
    }

//    if(1){
//        for(auto& rigid_body : activeModelTranslator->full_state_vector.rigid_bodies){
//            rigid_body.terminal_linear_pos_cost[0] = 0;
//            rigid_body.terminal_linear_pos_cost[1] = 0;
//        }
//    }

    std::cout << "final cost of entire MPC trajectory was: " << final_cost << "\n";
    std::cout << "avg opt time: " << average_opt_time_ms << " ms \n";
    std::cout << "avg num dofs: " << average_dof << "\n";
    std::cout << "avg percent derivs: " << average_percent_derivs << " % \n";
    std::cout << "avg time derivs: " << average_time_derivs_ms << " ms \n";
    std::cout << "avg time BP: " << average_time_bp_ms << " ms \n";
    std::cout << "avg time FP: " << average_time_fp_ms << " ms \n";

    return EXIT_SUCCESS;
}

void GenTestingData::AsyncronusMPCWorker(const std::string& method_directory, int task_number, int task_horizon){
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

    MatrixXd current_state;

    // Create init optimisation controls and reset system state
    std::cout << "before create init opt controls \n";
    optimised_controls = activeModelTranslator->CreateInitOptimisationControls(task_horizon);
    std::cout << "after create init opt controls \n";
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);

//    optimised_controls = optimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1, task_horizon);

    int bestMatchingStateIndex = 0;

    while(!stop_opt_thread){
        if(reoptimise){
            std::cout << "reoptimise called \n";
            // Copy current state of system (vis data) to starting data object for optimisation
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->vis_data);

            // Delete all controls before control index
            optimised_controls.erase(optimised_controls.begin(), optimised_controls.begin() + bestMatchingStateIndex);

            // Copy last control to keep control trajectory the same size
            MatrixXd last_control = optimised_controls.back();
            for (int i = 0; i < bestMatchingStateIndex; ++i) {
                optimised_controls.push_back(last_control);
            }

            optimised_controls = optimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                     optimised_controls, 1, 1, task_horizon);

//        std::cout << "optimised controls: " << optimised_controls[0].transpose() << "\n";

            // Store last iteration timing results
            time_iteration.push_back(optimiser->opt_time_ms);
            num_dofs.push_back(optimiser->dof_used_last_optimisation);
            time_get_derivs.push_back(optimiser->avg_time_get_derivs_ms);
            time_bp.push_back(optimiser->avg_time_backwards_pass_ms);
            time_fp.push_back(optimiser->avg_time_forwards_pass_ms);
            percent_derivs_computed.push_back(optimiser->avg_percent_derivs);
            surprise.push_back(0.0);
            expected.push_back(0.0);
            new_cost.push_back(0.0);

            int opt_time_to_timestep = optimiser->opt_time_ms / (activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep() * 1000);

            current_state = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data,
                                                                     activeModelTranslator->current_state_vector);

            // Compute the best starting state
            double smallestError = 1000.00;

            bestMatchingStateIndex = opt_time_to_timestep;
            if(bestMatchingStateIndex >= task_horizon){
                bestMatchingStateIndex = task_horizon - 1;
            }
            for(int i = 0; i < task_horizon - 1; i++){
//                std::cout << "i: " << i << " state: " << activeOptimiser->X_old[i].transpose() << std::endl;
//                std::cout << "correct state: " << current_vis_state.transpose() << std::endl;
                double currError = 0.0f;
                for(int j = 0; j < activeModelTranslator->current_state_vector.dof*2; j++){
                    // TODO - im not sure about this, should we use full state?
                    currError += abs(optimiser->X_old[i](j) - current_state(j));
                }
                if(currError < smallestError){
                    smallestError = currError;
                    bestMatchingStateIndex = i;
                }
            }
//            bestMatchingStateIndex = 1;

            // Mutex lock
            {
                std::unique_lock<std::mutex> lock(mtx);

                activeVisualiser->controlBuffer = optimised_controls;

                // Set the current control index to the best matching state index
                activeVisualiser->current_control_index = bestMatchingStateIndex;

                apply_next_control = true;
                reoptimise = false;
            }
        }
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
    int count = 0;
    int file_num = 0;
    std::string task_name = activeModelTranslator->model_name;

    int optimisation_horizon = activeModelTranslator->openloop_horizon;

    while(count < num_trajecs){

        // Initialise system for task
        yamlReader->LoadTaskFromFile(task_name, file_num, activeModelTranslator->current_state_vector, activeModelTranslator->residual_list);
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
            // TODO - needs fixing
            yamlReader->SaveTrajecInformation(optimiser->A, optimiser->B,
                                              optimiser->X_old, optimiser->U_old,
                                              task_name);

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
    for(int i = 0; i < num_scenes; i++){
        activeModelTranslator->GenerateRandomGoalAndStartState();
        activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->vis_data);
        mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
        activeVisualiser->render("Generating random test scenes");
        yamlReader->SaveTaskToFile(activeModelTranslator->model_name, i, activeModelTranslator->full_state_vector, activeModelTranslator->residual_list);
        std::cout << "scene " << i << " generated \n";
    }

    return EXIT_SUCCESS;
}

std::string GenTestingData::CreateTestName(const std::string& testing_method) {

    // Go back two directories
    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    std::string task_prefix = activeModelTranslator->model_name;

    std::string time_stamp = GetCurrentTimestamp();

    std::string optimiser_name = optimiser->ReturnName();

    std::string root_path = project_parent_path + "/TestingData/" + optimiser_name;

    // Check if optimiser directory exists
    if (!filesystem::exists(root_path)) {
        if (!filesystem::create_directories(root_path)) {
            std::cerr << "Failed to create directory: " << root_path << std::endl;
        }
    }

    std::string method_directory = root_path + "/" + task_prefix + "_" + testing_method + "_" + time_stamp;

    // Check if method directory exists, if not create it
    if (!filesystem::exists(method_directory)) {
        if (!filesystem::create_directories(method_directory)) {
            std::cerr << "Failed to create directory: " << method_directory << std::endl;
            exit(1);
        }
    }

    return method_directory;
}

void GenTestingData::SaveTestSummaryData(keypoint_method keypoint_method,
                                         int opt_horizon,
                                         double control_noise,
                                         const std::string& optimiser_name,
                                         const std::string& testing_directory){

//  ------------------ make method name ------------------
    std::string keypoint_method_name;
    if(keypoint_method.auto_adjust){
        keypoint_method_name = "AA_" + std::to_string(keypoint_method.min_N) + "_" + std::to_string(keypoint_method.max_N);
    }
    else{
        if(keypoint_method.name == "set_interval") {
            keypoint_method_name = "SI_" + std::to_string(keypoint_method.min_N);
        }
        else if(keypoint_method.name == "velocity_change"){
            keypoint_method_name = "*VC_" +
                          std::to_string(keypoint_method.min_N) + "_" +
                          std::to_string(keypoint_method.max_N);
        }
        else if(keypoint_method.name == "adaptive_jerk"){
            keypoint_method_name = "AJ_" +
                          std::to_string(keypoint_method.min_N) + "_" +
                          std::to_string(keypoint_method.max_N);
        }
        else if(keypoint_method.name == "iterative_error"){
            keypoint_method_name = "*IE_" +
                          std::to_string(keypoint_method.min_N) + "_" +
                          std::to_string(keypoint_method.max_N);
        }
    }

    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "optimisation horizon";
    out << YAML::Value << opt_horizon;

    out << YAML::Key << "controls_noise";
    out << YAML::Value << control_noise;

    out << YAML::Key << "model timestep";
    out << YAML::Value << activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep();

    // -------------------- Keypoint names and other parameters -----------------------
    out << YAML::Key << "keypoint_name";
    out << YAML::Value << keypoint_method_name;

    out << YAML::Key << "keypoint_min_N";
    out << YAML::Value << keypoint_method.min_N;

    out << YAML::Key << "keypoint_max_N";
    out << YAML::Value << keypoint_method.max_N;

    if(keypoint_method.name == "velocity_change"){
        std::vector<double> thresholds;
        for(int i = 0; i < keypoint_method.velocity_change_thresholds.size(); i++){
            thresholds.push_back(keypoint_method.velocity_change_thresholds[i]);
        }
        out << YAML::Key << "velocity_change_thresholds";
        out << YAML::Value << thresholds;
    }

    if(keypoint_method.name == "adaptive_jerk"){
        std::vector<double> thresholds;
        for(int i = 0; i < keypoint_method.jerk_thresholds.size(); i++){
            thresholds.push_back(keypoint_method.jerk_thresholds[i]);
        }
        out << YAML::Key << "jerk_thresholds";
        out << YAML::Value << thresholds;
    }

    if(keypoint_method.name == "iterative_error"){
        out << YAML::Key << "error_threshold";
        out << YAML::Value << keypoint_method.iterative_error_threshold;
    }

    // -------------------------- State vector reduction ----------------------------

    if(optimiser_name == "iLQR_SVR"){
        out << YAML::Key << "num_dofs_readd";
        out << YAML::Value << optimiser->num_dofs_readd;

        out << YAML::Key << "K_matrix_threshold";
        out << YAML::Value << optimiser->K_matrix_threshold;

        out << YAML::Key << "Eigen vector method";
        out << YAML::Value << optimiser->eigen_vector_method;
    }
    out << YAML::EndMap;

    // Open a file for writing
    std::string file_name = testing_directory + "/summary.yaml";

    std::ofstream fout(file_name);
    fout << out.c_str();
    fout.close();
}

int GenTestingData::AnalyseToyContact(int horizon){

    // Perform special optimisation using iLQR. Do one iteration at a time, compare performance
    // when optimising with exact derivatives and approximated derivatives.
    // Save the exact derivatives, approximated derivatives, and the cost performance of each method.

    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    // Create the file directory root path dynamically
    std::string method_directory = CreateTestName("openloop");

    // ------------------------- data storage -------------------------------------
    std::vector<double> cost_reductions;
    std::vector<double> optimisation_times;
    std::vector<int>    num_iterations;
    std::vector<double> avg_num_dofs;
    std::vector<double> avg_percent_derivs;
    std::vector<double> total_time_derivs;
    std::vector<double> total_time_bp;
    std::vector<double> total_time_fp;
    // -----------------------------------------------------------------------------

    auto startTimer = std::chrono::high_resolution_clock::now();
    optimiser->verbose_output = true;

    // Arbitrary number of trials
    optimiser->keypoint_generator->ResetCache();
    // Load the task from CSV file
    yamlReader->LoadTaskFromFile(activeModelTranslator->model_name, 6, activeModelTranslator->full_state_vector, activeModelTranslator->residual_list);
    activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);
    // Setup mj data objects
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);

    // Simulation stabilise
    for(int i = 0; i < 100; i++){
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
    }

    if (!activeModelTranslator->MuJoCo_helper->CheckIfDataIndexExists(0)) {
        activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(
                activeModelTranslator->MuJoCo_helper->master_reset_data);
    }
    // Perform any setup controls for this task
    std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data,
                                                          activeModelTranslator->MuJoCo_helper->main_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);

    // Create init optimisation controls
    std::vector<MatrixXd> init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(
            activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
            activeModelTranslator->MuJoCo_helper->master_reset_data);

    keypoint_method saved_method = optimiser->activeKeyPointMethod;

    double old_cost;
    double new_cost_exact;
    std::vector<double> new_cost_approximated;
    std::vector<int> contact;
    std::string task_name = activeModelTranslator->model_name;

    double lambda_save = optimiser->lambda;

    // Different levels of contact smoothing
    for (int i = 1; i < 100; i++) {

        optimiser->lambda = lambda_save;

        // Do the optimisation with interpolated derivatives
        // ------------------ Manual optimisation iteration - detect the contact change and suggest key-points based on this ------------------------
        old_cost = optimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                       true, init_opt_controls);

        // Get the contact change list
        contact.clear();
        for(int t = 0; t < horizon; t++){
            bool contact_found = activeModelTranslator->MuJoCo_helper->CheckPairForCollisions("goal", "piston_rod",
                                                                                              activeModelTranslator->MuJoCo_helper->saved_systems_state_list[t]);
            contact.push_back(contact_found);
        }

        // Set the optimiser to smooth contact
        optimiser->smoothing_contact = true;
        optimiser->smoothing = i;

        // -----------------------------
        std::vector<MatrixXd> optimised_controls = optimiser->Optimise(
                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1,
                horizon);


        // Saving the data
        std::string file_prefix = task_name + "/iter_0" + "/smoothing_contact_" + std::to_string(i);
        yamlReader->SaveTrajecInformation(optimiser->A, optimiser->B,
                                          optimiser->X_old, optimiser->U_old,
                                          file_prefix);

        new_cost_approximated.push_back(optimiser->new_cost);

        // Set keypoint method to set interval 1 for exact derivatives
        optimiser->activeKeyPointMethod.name = "set_interval";
        optimiser->activeKeyPointMethod.min_N = 1;

        optimiser->SetCurrentKeypointMethod(optimiser->activeKeyPointMethod);

        // Reset the optimiser back to exact derivatives
        optimiser->smoothing_contact = false;
        optimiser->smoothing = 0;

        // Do the optimisation with exact derivatives!
        optimiser->lambda = lambda_save;
        optimised_controls = optimiser->Optimise(
                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1,
                horizon);

        // Saving the data
        file_prefix = task_name + "/iter_0" + "/exact_derivs";
        yamlReader->SaveTrajecInformation(optimiser->A, optimiser->B,
                                          optimiser->X_old, optimiser->U_old,
                                          file_prefix);

        new_cost_exact = optimiser->new_cost;

        // Reset keypoint method
        optimiser->activeKeyPointMethod = saved_method;
        optimiser->SetCurrentKeypointMethod(optimiser->activeKeyPointMethod);

        // Update the trajectory
//        init_opt_controls = optimised_controls;
    }

    // Save YAML file
    // old cost, new cost exact, new cost_1, new_cost_2 .....
    // contact list
    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "old_cost";
    out << YAML::Value << old_cost;

    out << YAML::Key << "new_cost_exact";
    out << YAML::Value << new_cost_exact;

    out << YAML::Key << "new_cost_approx";
    out << YAML::Value << new_cost_approximated;

    out << YAML::Key << "contact";
    out << YAML::Value << contact;

    out << YAML::EndMap;

    // Open a file for writing
    std::string file_name = project_parent_path + "/savedTrajecInfo" + task_name + "/iter_0" + "/performance.yaml";

    std::ofstream fout(file_name);
    fout << out.c_str();
    fout.close();

    return EXIT_SUCCESS;
}

int GenTestingData::AnalyseToyContactKeypoints(int horizon){

    // Perform special optimisation using iLQR. Do one iteration at a time, compare performance
    // when optimising with exact derivatives and approximated derivatives.
    // Save the exact derivatives, approximated derivatives, and the cost performance of each method.

    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    // Create the file directory root path dynamically
    std::string method_directory = CreateTestName("openloop");

    // ------------------------- data storage -------------------------------------
    std::vector<double> cost_reductions;
    std::vector<double> optimisation_times;
    std::vector<int>    num_iterations;
    std::vector<double> avg_num_dofs;
    std::vector<double> avg_percent_derivs;
    std::vector<double> total_time_derivs;
    std::vector<double> total_time_bp;
    std::vector<double> total_time_fp;
    // -----------------------------------------------------------------------------

    auto startTimer = std::chrono::high_resolution_clock::now();
    optimiser->verbose_output = true;

    // Arbitrary number of trials
    optimiser->keypoint_generator->ResetCache();
    // Load the task from CSV file
    yamlReader->LoadTaskFromFile(activeModelTranslator->model_name, 6, activeModelTranslator->full_state_vector, activeModelTranslator->residual_list);
    activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);
    // Setup mj data objects
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);

    for(int i = 0; i < 100; i++){
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
    }

    if (!activeModelTranslator->MuJoCo_helper->CheckIfDataIndexExists(0)) {
        activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(
                activeModelTranslator->MuJoCo_helper->master_reset_data);
    }
    // Perform any setup controls for this task
    std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data,
                                                          activeModelTranslator->MuJoCo_helper->main_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);

    // Create init optimisation controls
    std::vector<MatrixXd> init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(
            activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
            activeModelTranslator->MuJoCo_helper->master_reset_data);

    keypoint_method saved_method = optimiser->activeKeyPointMethod;

    double old_cost;
    double new_cost_exact;
    std::vector<double> new_cost_approximated;
    std::vector<int> contact;
    std::string task_name = activeModelTranslator->model_name;

    double lambda_save = optimiser->lambda;

    // Rollout the nominal trajectory
    old_cost = optimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                                true, init_opt_controls);

    // Compute dynamics derivatives fully.
    optimiser->GenerateDerivatives();
    yamlReader->SaveTrajecInformation(optimiser->A, optimiser->B,
                                      optimiser->X_old, optimiser->U_old,
                                      task_name + "/exact_derivs");

    // Save exact A and B matrices to file

    // Compute dynamics derivatives using key-points
    optimiser->activeKeyPointMethod.name = "adaptive_jerk";
    optimiser->activeKeyPointMethod.min_N = 2;
    optimiser->activeKeyPointMethod.max_N = 500;
    optimiser->SetCurrentKeypointMethod(optimiser->activeKeyPointMethod);

    optimiser->GenerateDerivatives();

    // Save approximated dynamics derivatives to file
    yamlReader->SaveTrajecInformation(optimiser->A, optimiser->B,
                                      optimiser->X_old, optimiser->U_old,
                                      task_name + "/approx_derivs_adaptive_jerk");

    yamlReader->SaveKeypointsToFile(task_name + "/keypoints_adaptive_jerk", 0, optimiser->keypoint_generator->keypoints);

//    for (int i = 1; i < 100; i++) {
//
//        optimiser->lambda = lambda_save;
//
//        // Do the optimisation with interpolated derivatives
//        // ------------------ Manual optimisation iteration - detect the contact change and suggest key-points based on this ------------------------
//        old_cost = optimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
//                                                true, init_opt_controls);
//
//        // Get the contact change list
//        contact.clear();
//        for(int t = 0; t < horizon; t++){
//            bool contact_found = activeModelTranslator->MuJoCo_helper->CheckPairForCollisions("goal", "piston_rod",
//                                                                                              activeModelTranslator->MuJoCo_helper->saved_systems_state_list[t]);
//            contact.push_back(contact_found);
//        }
//
//        // Set the optimiser to smooth contact
//        optimiser->smoothing_contact = true;
//        optimiser->smoothing = i;
//
//        // -----------------------------
//        std::vector<MatrixXd> optimised_controls = optimiser->Optimise(
//                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1,
//                horizon);
//
//
//        // Saving the data
//        std::string file_prefix = task_name + "/iter_0" + "/smoothing_contact_" + std::to_string(i);
//        yamlReader->SaveTrajecInformation(optimiser->A, optimiser->B,
//                                          optimiser->X_old, optimiser->U_old,
//                                          file_prefix);
//
//        new_cost_approximated.push_back(optimiser->new_cost);
//
//        // Set keypoint method to set interval 1 for exact derivatives
//        optimiser->activeKeyPointMethod.name = "set_interval";
//        optimiser->activeKeyPointMethod.min_N = 1;
//
//        optimiser->SetCurrentKeypointMethod(optimiser->activeKeyPointMethod);
//
//        // Reset the optimiser back to exact derivatives
//        optimiser->smoothing_contact = false;
//        optimiser->smoothing = 0;
//
//        // Do the optimisation with exact derivatives!
//        optimiser->lambda = lambda_save;
//        optimised_controls = optimiser->Optimise(
//                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 1, 1,
//                horizon);
//
//        // Saving the data
//        file_prefix = task_name + "/iter_0" + "/exact_derivs";
//        yamlReader->SaveTrajecInformation(optimiser->A, optimiser->B,
//                                          optimiser->X_old, optimiser->U_old,
//                                          file_prefix);
//
//        new_cost_exact = optimiser->new_cost;
//
//        // Reset keypoint method
//        optimiser->activeKeyPointMethod = saved_method;
//        optimiser->SetCurrentKeypointMethod(optimiser->activeKeyPointMethod);
//
//        // Update the trajectory
////        init_opt_controls = optimised_controls;
//    }

    // Save YAML file
    // old cost, new cost exact, new cost_1, new_cost_2 .....
    // contact list
//    YAML::Emitter out;
//
//    out << YAML::BeginMap;
//    out << YAML::Key << "old_cost";
//    out << YAML::Value << old_cost;
//
//    out << YAML::Key << "new_cost_exact";
//    out << YAML::Value << new_cost_exact;
//
//    out << YAML::Key << "new_cost_approx";
//    out << YAML::Value << new_cost_approximated;
//
//    out << YAML::Key << "contact";
//    out << YAML::Value << contact;
//
//    out << YAML::EndMap;
//
//    // Open a file for writing
//    std::string file_name = project_parent_path + "/savedTrajecInfo" + task_name + "/iter_0" + "/performance.yaml";
//
//    std::ofstream fout(file_name);
//    fout << out.c_str();
//    fout.close();

    return EXIT_SUCCESS;
}