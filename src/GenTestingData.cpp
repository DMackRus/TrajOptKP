#include "GenTestingData.h"

GenTestingData::GenTestingData(std::shared_ptr<iLQR> iLQROptimiser_,
                               std::shared_ptr<ModelTranslator> activeModelTranslator_,
                               std::shared_ptr<Differentiator> activeDifferentiator_,
                               std::shared_ptr<Visualiser> activeVisualiser_,
                               std::shared_ptr<FileHandler> yamlReader_) {

    iLQROptimiser = iLQROptimiser_;
    activeModelTranslator = activeModelTranslator_;
    activeDifferentiator = activeDifferentiator_;
    activeVisualiser = activeVisualiser_;
    yamlReader = yamlReader_;

    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);
    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator, activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator, yamlReader_->maxHorizon, activeVisualiser,
                                           yamlReader);
}

int GenTestingData::testing_different_minN_asynchronus_mpc(int lowest_minN, int higherst_minN, int step_size){

    for(int i = lowest_minN; i <= higherst_minN; i += step_size){
        keypoint_method keypoint_method;
        keypoint_method.name = "set_interval";
        keypoint_method.min_N = i;
        keypoint_method.max_N = i;

        testing_asynchronus_mpc(keypoint_method, 100, 100, 2000);
    }

    return EXIT_SUCCESS;
}

int GenTestingData::gen_data_async_mpc(int task_horizon, int task_timeout){

    std::cout << "beginning testing asynchronus MPC for " << activeModelTranslator->model_name << std::endl;
    std::cout << "optimisation horizon is: " << task_horizon << " task timeout : " << task_timeout << "\n";

//    std::vector<int> minN = {1};
//    std::vector<int> maxN_multiplier = {20};
//    std::vector<double> velocity_change_thresholds = {0.01, 0.1, 0.5, 1.0, 2.0};
//
//    for(int i = 0; i < minN.size(); i++){
//        for(int j = 0; j < maxN_multiplier.size(); j++){
//            for(int k = 0; k < velocity_change_thresholds.size(); k++){
//                keypoint_method keypoint_method;
//                keypoint_method.name = "magvel_change";
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

    keypoint_method keypoint_method;

    int num_trials = 100;

    keypoint_method.name = "adaptive_jerk";
    keypoint_method.min_N = 1;
    keypoint_method.max_N = 50;
    keypoint_method.auto_adjust = true;
    for(int i = 0; i < activeModelTranslator->dof; i++){
        keypoint_method.jerk_thresholds.push_back(1e-15);
    }
    testing_asynchronus_mpc(keypoint_method, num_trials, task_horizon, task_timeout);

    // Test set interval methods
    keypoint_method.name = "setInterval";
    keypoint_method.max_N = 1;
    keypoint_method.auto_adjust = false;
    std::vector<int> minNs = {1, 2, 5, 10, 20, 40, 60, 80, 150};
    for(int minN : minNs){
        // Only test for minN's < task_horizon
        if(minN <= task_horizon){
            keypoint_method.min_N = minN;
            testing_asynchronus_mpc(keypoint_method, num_trials, task_horizon, task_timeout);
        }
    }



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

    std::string rootPath = projectParentPath + "/testingData/" + task_prefix +  "_" + std::to_string(task_horizon);
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

    // start timer here
    auto startTime = std::chrono::high_resolution_clock::now();

    // ------------------------- data storage -------------------------------------
//    std::vector<std::vector<double>> finalCosts;
    std::vector<double> finalCostsRow;

    std::vector<std::vector<double>> finalDistances;
    std::vector<double> finalDistRow;

    std::vector<std::vector<double>> avgOptTimes;
    std::vector<double> avgOptTimesRow;

    std::vector<std::vector<double>> avgPercentDerivs;
    std::vector<double> avgPercentDerivsRow;

    std::vector<std::vector<double>> avgTimeForDerivs;
    std::vector<double> avgTimeForDerivsRow;

    std::vector<std::vector<double>> avgTimeBP;
    std::vector<double> avgTimeBPRow;

    std::vector<std::vector<double>> avgTimeFP;
    std::vector<double> avgTimeFPRow;

    std::vector<std::vector<double>> avgSurprise;
    std::vector<double> avgSurpriseRow;
    // -----------------------------------------------------------------------------

    auto startTimer = std::chrono::high_resolution_clock::now();
    iLQROptimiser->verbose_output = false;

    iLQROptimiser->SetCurrentKeypointMethod(keypoint_method);

//    finalCosts.clear();
    avgTimeForDerivs.clear();
    avgTimeBP.clear();
    avgTimeFP.clear();
    avgPercentDerivs.clear();

    for (int i = 0; i < num_trials; i++) {
        iLQROptimiser->keypoint_generator->ResetCache();
        // Load start and desired state from csv file
        MatrixXd X_start(activeModelTranslator->state_vector_size, 1);
        yamlReader->loadTaskFromFile(task_prefix, i, X_start, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = X_start;

        // Reset the time of simulation in all data?
        activeModelTranslator->MuJoCo_helper->master_reset_data->time = 0.0f;
        activeModelTranslator->MuJoCo_helper->vis_data->time = 0.0f;
        activeModelTranslator->MuJoCo_helper->main_data->time = 0.0f;
        for(auto & j : activeModelTranslator->MuJoCo_helper->fd_data){
            j->time = 0.0f;
        }
        for(auto & j : activeModelTranslator->MuJoCo_helper->savedSystemStatesList) {
            j->time = 0.0f;
        }

        activeModelTranslator->SetStateVector(X_start, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->master_reset_data);
        if(!activeModelTranslator->MuJoCo_helper->checkIfDataIndexExists(0)){
            activeModelTranslator->MuJoCo_helper->appendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
        }

        std::vector<MatrixXd> initSetupControls = activeModelTranslator->CreateInitSetupControls(1000);
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

        // Perform the optimisation MPC test here asynchronously
        // Reset gravity back to normal
//        activeModelTranslator->MuJoCo_helper->model->opt.gravity[2] = -9.81;
        single_asynchronus_run(true, method_directory, i, task_horizon, task_timeout);
        stop_opt_thread = false;

        // ------------------------- data storage -------------------------------------
        finalCostsRow.push_back(final_cost);
        finalDistRow.push_back(final_dist);
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
    file_output << "Final cost" << "," << "Final dist" << "," << "Average optimisation time" << "," << "Average percent derivs" << ",";
    file_output << "Average time derivs" << "," << "Average time BP" << "," << "Average time FP" << "," << "average_surprise" << std::endl;

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

    // Make a thread for the Optimiser
    std::thread MPC_controls_thread;
    // Start the thread running
    MPC_controls_thread = std::thread(&GenTestingData::asynchronus_optimiser_worker, this, method_directory, task_number, task_horizon);
    int vis_counter = 0;
    MatrixXd next_control;
    // timer variables
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;

    // elapsed task time
    int task_time = 0;

    activeModelTranslator->MuJoCo_helper->vis_data->time = 0.0f;

    while(task_time++ < TASK_TIMEOUT){
        begin = std::chrono::steady_clock::now();

        if(activeVisualiser->current_control_index < activeVisualiser->controlBuffer.size()){

            next_control = activeVisualiser->controlBuffer[activeVisualiser->current_control_index];
            // Increment the current control index
            activeVisualiser->current_control_index++;
        }
        else{
            MatrixXd empty_control(activeModelTranslator->num_ctrl, 1);
            empty_control.setZero();
            next_control = empty_control;
        }

        if(APPLY_NOISE){
            for(int i = 0; i < activeModelTranslator->num_ctrl; i++){
                double gauss_noise = GaussNoise(0, 0.1);
                next_control(i, 0) += gauss_noise;
            }
        }

        // Store latest control and state in a replay buffer
        activeVisualiser->trajectory_controls.push_back(next_control);
        activeVisualiser->trajectory_states.push_back(activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data));

        // Set the latest control
        activeModelTranslator->SetControlVector(next_control, activeModelTranslator->MuJoCo_helper->vis_data);

        // Update the simulation
        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);

//        std::cout << "time: " << activeModelTranslator->MuJoCo_helper->vis_data->time << std::endl;

        double dist;
        if(activeModelTranslator->TaskComplete(activeModelTranslator->MuJoCo_helper->vis_data, dist)){
            std::cout << "Task complete" << std::endl;
            break;
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
        int difference_ms = (activeModelTranslator->MuJoCo_helper->returnModelTimeStep() * 1000) - (time_taken / 1000.0f) + 1;

        if(difference_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(difference_ms));
        }
//        else
//            std::cout << "visualisation took " << (time_taken / 1000.0f) << " ms, longer than time-step, skipping sleep \n";


        // Testing condition - change gravity halfway through task.
//        if(task_time == 1000){
//            activeModelTranslator->MuJoCo_helper->model->opt.gravity[2] = -13;
//        }
    }

    // Store final distance in final_dist
    activeModelTranslator->TaskComplete(activeModelTranslator->MuJoCo_helper->vis_data, final_dist);

    std::mutex mtx;
    mtx.lock();
    stop_opt_thread = true;
    mtx.unlock();
    MPC_controls_thread.join();

    final_cost = 0.0f;
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){
        activeModelTranslator->SetControlVector(activeVisualiser->trajectory_controls[i], activeModelTranslator->MuJoCo_helper->vis_data);
        activeModelTranslator->SetStateVector(activeVisualiser->trajectory_states[i], activeModelTranslator->MuJoCo_helper->vis_data);
//        activeModelTranslator->MuJoCo_helper->forwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
//
//        activeVisualiser->render("playback");

        final_cost += activeModelTranslator->CostFunction(activeModelTranslator->MuJoCo_helper->vis_data, false);
    }

    std::cout << "final cost of entire MPC trajectory was: " << final_cost << "\n";
    std::cout << "avg opt time: " << average_opt_time_ms << " ms \n";
    std::cout << "avg percent derivs: " << average_percent_derivs << " % \n";
    std::cout << "avg time derivs: " << average_time_derivs_ms << " ms \n";
    std::cout << "avg time BP: " << average_time_bp_ms << " ms \n";
    std::cout << "avg time FP: " << average_time_fp_ms << " ms \n";

    return 1;
}

void GenTestingData::asynchronus_optimiser_worker(std::string method_directory, int task_number, int task_horizon){

    bool taskComplete = false;
    int visualCounter = 0;

    std::vector<double> timeOpt;
    std::vector<double> timeGettingDerivs;
    std::vector<double> timeBackwardsPass;
    std::vector<double> timeForwardsPass;
    std::vector<double> percentagesDerivsCalculated;
    std::vector<double> surprise;
    std::vector<double> expected;
    std::vector<double> new_cost;

    std::vector<MatrixXd> optimisedControls;

    // Instantiate init controls
    std::vector<MatrixXd> initOptimisationControls;

    int horizon = 150;

    initOptimisationControls = activeModelTranslator->CreateInitOptimisationControls(horizon);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    optimisedControls = iLQROptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], initOptimisationControls, 1, 1, horizon);

    MatrixXd currState;

    while(!taskComplete){

        if(stop_opt_thread){
            break;
        }

        visualCounter++;

        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->vis_data);
        activeModelTranslator->MuJoCo_helper->copySystemState(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], activeModelTranslator->MuJoCo_helper->main_data);

        int current_control_index = activeVisualiser->current_control_index;

        // Slice the optimised controls to get the current control
        for(int i = 0; i < current_control_index; i++){
            optimisedControls.erase(optimisedControls.begin());
            optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));
        }

        optimisedControls = iLQROptimiser->Optimise(activeModelTranslator->MuJoCo_helper->savedSystemStatesList[0], optimisedControls, 1, 1, horizon);

        // Saved statistical measures
        timeOpt.push_back(iLQROptimiser->opt_time_ms);
        timeGettingDerivs.push_back(iLQROptimiser->avg_time_get_derivs_ms);
        timeBackwardsPass.push_back(iLQROptimiser->avg_time_backwards_pass_ms);
        timeForwardsPass.push_back(iLQROptimiser->avg_time_forwards_pass_ms);
        percentagesDerivsCalculated.push_back(iLQROptimiser->avg_percent_derivs);
        surprise.push_back(iLQROptimiser->avg_surprise);
        expected.push_back(iLQROptimiser->avg_expected);
        new_cost.push_back(iLQROptimiser->new_cost);

        int optTimeToTimeSteps = iLQROptimiser->opt_time_ms / (activeModelTranslator->MuJoCo_helper->returnModelTimeStep() * 1000);

        int low_bound = optTimeToTimeSteps - 3;
        if (low_bound < 0) low_bound = 0;

        int high_bound = optTimeToTimeSteps + 3;

        // By the time we have computed optimal controls, main visualisation will be some number
        // of time-steps ahead. We need to find the correct control to apply.

        MatrixXd current_vis_state = activeModelTranslator->ReturnStateVector(activeModelTranslator->MuJoCo_helper->vis_data);

        double smallestError = 1000.00;
        int bestMatchingStateIndex = 0;
        // TODO - possible issue if optimisation time > horizon
        for(int i = low_bound; i < high_bound; i++){
            double currError = 0.0f;
            for(int j = 0; j < activeModelTranslator->state_vector_size; j++){
                currError += abs(iLQROptimiser->X_old[i](j) - current_vis_state(j));
            }

            if(currError < smallestError){
                smallestError = currError;
                bestMatchingStateIndex = i;
            }
        }

        // If we are use Async visualisation, need to copy our control vector to internal control vector for
        // visualisation class
        std::mutex mtx;
        mtx.lock();

        activeVisualiser->controlBuffer = optimisedControls;
        activeVisualiser->current_control_index = bestMatchingStateIndex;
        activeVisualiser->new_controls_flag = true;

        mtx.unlock();
    }

    // Save specific trajectory data
    std::string filename = method_directory + "/" + std::to_string(task_number) + ".csv";

    std::cout << "filename for task: " << filename << std::endl;

    ofstream file_output;
    file_output.open(filename);

    // Make header
    file_output << "Optimisation time (ms)" << "," << "derivs time (ms)" << "," << "BP time (ms)" << ",";
    file_output << "FP time (ms)" << "," << "% derivs" << "," << "surprise" << "," << "expected" << "," << "new cost" << std::endl;

    // Loop through rows
    for(int i = 0; i < timeGettingDerivs.size(); i++){
        file_output << timeOpt[i] << "," << timeGettingDerivs[i] << "," << timeBackwardsPass[i] << ",";
        file_output << timeForwardsPass[i] << "," << percentagesDerivsCalculated[i] << "," << surprise[i] << ",";
        file_output << expected[i] << "," << new_cost[i] << std::endl;
    }

    file_output.close();

    average_opt_time_ms = 0.0f;
    average_time_derivs_ms = 0.0f;
    average_time_bp_ms = 0.0f;
    average_time_fp_ms = 0.0f;
    average_percent_derivs = 0.0f;
    average_surprise = 0.0f;

    for(int i = 0; i < timeGettingDerivs.size(); i++){
        average_opt_time_ms += timeOpt[i];
        average_time_derivs_ms += timeGettingDerivs[i];
        average_time_bp_ms += timeBackwardsPass[i];
        average_time_fp_ms += timeForwardsPass[i];
        average_percent_derivs += percentagesDerivsCalculated[i];
        average_surprise += surprise[i];
    }

    average_opt_time_ms /= timeOpt.size();
    average_time_derivs_ms /= timeGettingDerivs.size();
    average_time_bp_ms /= timeBackwardsPass.size();
    average_time_fp_ms /= timeForwardsPass.size();
    average_percent_derivs /= percentagesDerivsCalculated.size();
    average_surprise /= surprise.size();

}