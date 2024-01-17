//
// Created by davidrussell on 1/17/24.
//

#include "Testing.h"

Testing::Testing(std::shared_ptr<interpolatediLQR> iLQROptimiser_,
                 std::shared_ptr<modelTranslator> activeModelTranslator_,
                 std::shared_ptr<differentiator> activeDifferentiator_,
                 std::shared_ptr<visualizer> activeVisualiser_,
                 std::shared_ptr<fileHandler> yamlReader_) {

    iLQROptimiser = iLQROptimiser_;
    activeModelTranslator = activeModelTranslator_;
    activeDifferentiator = activeDifferentiator_;
    activeVisualiser = activeVisualiser_;
    yamlReader = yamlReader_;

    activeDifferentiator = std::make_shared<differentiator>(activeModelTranslator, activeModelTranslator->myHelper);

    activeVisualiser = std::make_shared<visualizer>(activeModelTranslator);
    iLQROptimiser = std::make_shared<interpolatediLQR>(activeModelTranslator, activeModelTranslator->activePhysicsSimulator,
                                                       activeDifferentiator, yamlReader_->maxHorizon, activeVisualiser,
                                                       yamlReader);
}

int Testing::testing_asynchronus_mpc(derivative_interpolator keypoint_method){

    std::string task_prefix = activeModelTranslator->modelName;
    std::cout << "beginning testing asynchronus MPC for " << task_prefix << std::endl;

    // start timer here
    auto startTime = std::chrono::high_resolution_clock::now();

    // ------------------------- data storage -------------------------------------
    std::vector<std::vector<double>> finalCosts;
    std::vector<double> finalCostsRow;

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
    // -----------------------------------------------------------------------------

    std::string method_name;
    if(keypoint_method.keypoint_method == "setInterval") {
        method_name = keypoint_method.keypoint_method + "_" + std::to_string(keypoint_method.minN);
    }
    else{
        method_name = keypoint_method.keypoint_method + "_" + std::to_string(keypoint_method.minN) + "_" + std::to_string(keypoint_method.maxN);
    }

    std::vector<int> horizons = {20, 30, 40, 50, 60, 70, 80};
    std::vector<std::string> horizonNames;
    int numHorizons = horizons.size();

    for(int i = 0; i < horizons.size(); i++){
        horizonNames.push_back(std::to_string(horizons[i]));
    }

    std::vector<double> targetVelocities;
    double minTarget = 0.1;
    double maxTarget = 0.3;
    int numTests = 5;
    double currentVel = minTarget;

    for(int i = 0; i < numTests; i++){
        targetVelocities.push_back(currentVel);
        currentVel += (maxTarget - minTarget) / (numTests - 1);
    }

    auto startTimer = std::chrono::high_resolution_clock::now();
    iLQROptimiser->verboseOutput = false;

    // Setup the derivative interpolator object
    derivative_interpolator currentInterpolator = iLQROptimiser->returnDerivativeInterpolator();
    keypoint_method.jerkThresholds = currentInterpolator.jerkThresholds;
    keypoint_method.magVelChangeThresholds = currentInterpolator.magVelChangeThresholds;
    keypoint_method.iterativeErrorThreshold = currentInterpolator.iterativeErrorThreshold;

    finalCosts.clear();
    avgTimeForDerivs.clear();
    avgTimeBP.clear();
    avgTimeFP.clear();
    avgPercentDerivs.clear();

    for (int i = 0; i < targetVelocities.size(); i++) {
        // Load start and desired state from csv file
        MatrixXd X_start(activeModelTranslator->stateVectorSize, 1);
        yamlReader->loadTaskFromFile(task_prefix, i, X_start, activeModelTranslator->X_desired);
        activeModelTranslator->X_start = X_start;

        // TODO - make this better
        // Change walker desired velocity
        activeModelTranslator->X_desired(10) = targetVelocities[i];

        cout << "start state " << activeModelTranslator->X_start << endl;
        cout << "desired state " << activeModelTranslator->X_desired << endl;

        activeModelTranslator->setStateVector(X_start, MASTER_RESET_DATA);
        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, MASTER_RESET_DATA);
        activeModelTranslator->activePhysicsSimulator->appendSystemStateToEnd(MASTER_RESET_DATA);
        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
        activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MASTER_RESET_DATA);

        // Perform the optimisation MPC test here asynchronously
        single_asynchronus_run(true);
        stop_opt_thread = false;

        // ------------------------- data storage -------------------------------------
        finalCostsRow.push_back(final_cost);
        avgOptTimesRow.push_back(average_opt_time_ms);
        avgPercentDerivsRow.push_back(average_percent_derivs);
        avgTimeForDerivsRow.push_back(average_time_derivs_ms);
        avgTimeBPRow.push_back(average_time_bp_ms);
        avgTimeFPRow.push_back(average_time_fp_ms);

    }

    // ----------------------- Save data to file -------------------------------------
    std::string projectParentPath = __FILE__;
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));
    projectParentPath = projectParentPath.substr(0, projectParentPath.find_last_of("/\\"));

    std::string rootPath = projectParentPath + "/testingData/" + task_prefix + "/";
    std::string filename = rootPath + task_prefix + "_" + method_name + "_testingData.csv";

    ofstream file_output;
    file_output.open(filename);

    // Make header
    file_output << "Final cost" << "," << "Average optimisation time" << "," << "Average percent derivs" << ",";
    file_output << "Average time derivs" << "," << "Average time BP" << "," << "Average time FP" << std::endl;


    // Loop through rows
    for(int i = 0; i < numTests; i++){
        file_output << finalCostsRow[i] << "," << avgOptTimesRow[i] << "," << avgPercentDerivsRow[i] << ",";
        file_output << avgTimeForDerivsRow[i] << "," << avgTimeBPRow[i] << "," << avgTimeFPRow[i] << std::endl;
    }

    file_output.close();

    return 1;
}

int Testing::single_asynchronus_run(bool visualise){

    // Make a thread for the optimiser
    std::thread MPC_controls_thread;
    // Start the thread running
    MPC_controls_thread = std::thread(&Testing::asynchronus_optimiser_worker, this);
    int vis_counter = 0;
    MatrixXd next_control;
    // timer variables
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;

    // How long to perform the task for
    int MAX_TASK_TIME = 2500;
    int task_time = 0;

    while(task_time++ < MAX_TASK_TIME){
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

        // Store latest control and state in a replay buffer
        activeVisualiser->trajectory_controls.push_back(next_control);
        activeVisualiser->trajectory_states.push_back(activeModelTranslator->returnStateVector(VISUALISATION_DATA));

        // Set the latest control
        activeModelTranslator->setControlVector(next_control, VISUALISATION_DATA);

        // Update the simulation
        activeModelTranslator->activePhysicsSimulator->stepSimulator(1, VISUALISATION_DATA);

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
        int difference_ms = (activeModelTranslator->activePhysicsSimulator->returnModelTimeStep() * 1000) - (time_taken / 1000.0f) + 1;

        if(difference_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(difference_ms));
        }
//        else
//            std::cout << "visualisation took " << (time_taken / 1000.0f) << " ms, longer than time-step, skipping sleep \n";
    }

    std::mutex mtx;
    mtx.lock();
    stop_opt_thread = true;
    mtx.unlock();
    MPC_controls_thread.join();

    final_cost = 0.0f;
    activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MASTER_RESET_DATA);
    for(int i = 0; i < activeVisualiser->trajectory_states.size(); i++){
        activeModelTranslator->setControlVector(activeVisualiser->trajectory_controls[i], VISUALISATION_DATA);
        activeModelTranslator->setStateVector(activeVisualiser->trajectory_states[i], VISUALISATION_DATA);
        activeModelTranslator->activePhysicsSimulator->forwardSimulator(VISUALISATION_DATA);

        final_cost += (activeModelTranslator->costFunction(VISUALISATION_DATA, false) * activeModelTranslator->activePhysicsSimulator->returnModelTimeStep());

    }

    std::cout << "final cost of entire MPC trajectory was: " << final_cost << "\n";
    std::cout << "avg opt time: " << average_opt_time_ms << " ms \n";
    std::cout << "avg percent derivs: " << average_percent_derivs << " % \n";
    std::cout << "avg time derivs: " << average_time_derivs_ms << " ms \n";
    std::cout << "avg time BP: " << average_time_bp_ms << " ms \n";
    std::cout << "avg time FP: " << average_time_fp_ms << " ms \n";

    return 1;
}

void Testing::asynchronus_optimiser_worker(){
//    void MPCUntilComplete(double &trajecCost, double &avgHZ, double &avgTimeGettingDerivs, double &avgPercentDerivs, double &avgTimeBP, double &avgTimeFP,
//                          int MAX_TASK_TIME, int REPLAN_TIME, int OPT_HORIZON){
    bool taskComplete = false;
    int visualCounter = 0;

    std::vector<double> timeGettingDerivs;
    std::vector<double> timeBackwardsPass;
    std::vector<double> timeForwardsPass;
    std::vector<double> percentagesDerivsCalculated;

    std::vector<MatrixXd> optimisedControls;

    // Instantiate init controls
    std::vector<MatrixXd> initOptimisationControls;

    int horizon = 80;

    initOptimisationControls = activeModelTranslator->createInitOptimisationControls(horizon);
    activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->copySystemState(0, MASTER_RESET_DATA);
    activeModelTranslator->activePhysicsSimulator->copySystemState(VISUALISATION_DATA, MASTER_RESET_DATA);

    optimisedControls = iLQROptimiser->optimise(0, initOptimisationControls, 5, 4, horizon);

    MatrixXd currState;

    while(!taskComplete){

        if(stop_opt_thread){
            break;
        }

        visualCounter++;

        activeModelTranslator->activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, VISUALISATION_DATA);
        activeModelTranslator->activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);


        int current_control_index = activeVisualiser->current_control_index;

        // Slice the optimised controls to get the current control
        for(int i = 0; i < current_control_index; i++){
            optimisedControls.erase(optimisedControls.begin());
            optimisedControls.push_back(optimisedControls.at(optimisedControls.size() - 1));
        }

        optimisedControls = iLQROptimiser->optimise(0, optimisedControls, 1, 1, horizon);

        timeGettingDerivs.push_back(iLQROptimiser->avgTime_getDerivs_ms);
        timeBackwardsPass.push_back(iLQROptimiser->avgTime_backwardsPass_ms);
        timeForwardsPass.push_back(iLQROptimiser->avgTime_forwardsPass_ms);
        percentagesDerivsCalculated.push_back(iLQROptimiser->avgPercentDerivs);


        // If we are use Async visualisation, need to copy our control vector to internal control vector for
        // visualisation class
        std::mutex mtx;
        mtx.lock();

        int optTimeToTimeSteps = iLQROptimiser->optTime / (activeModelTranslator->activePhysicsSimulator->returnModelTimeStep() * 1000);

        int low_bound = optTimeToTimeSteps - 3;
        if (low_bound < 0) low_bound = 0;

        int high_bound = optTimeToTimeSteps + 3;

        // By the time we have computed optimal controls, main visualisation will be some number
        // of time-steps ahead. We need to find the correct control to apply.

        MatrixXd current_vis_state = activeModelTranslator->returnStateVector(VISUALISATION_DATA);

        double smallestError = 1000.00;
        int bestMatchingStateIndex = 0;
        // TODO - possible issue if optimisation time > horizon
        for(int i = low_bound; i < high_bound; i++){
            double currError = 0.0f;
            for(int j = 0; j < activeModelTranslator->stateVectorSize; j++){
                currError += abs(iLQROptimiser->X_old[i](j) - current_vis_state(j));
            }

            if(currError < smallestError){
                smallestError = currError;
                bestMatchingStateIndex = i;
            }
        }

        activeVisualiser->controlBuffer = optimisedControls;
        activeVisualiser->current_control_index = bestMatchingStateIndex;
        activeVisualiser->new_controls_flag = true;

        mtx.unlock();
    }


    average_time_derivs_ms = 0.0f;
    average_time_bp_ms = 0.0f;
    average_time_fp_ms = 0.0f;
    average_percent_derivs = 0.0f;

    for(int i = 0; i < timeGettingDerivs.size(); i++){
        average_time_derivs_ms += timeGettingDerivs[i];
        average_time_bp_ms += timeBackwardsPass[i];
        average_time_fp_ms += timeForwardsPass[i];
        average_percent_derivs += percentagesDerivsCalculated[i];
    }

    average_time_derivs_ms /= timeGettingDerivs.size();
    average_time_bp_ms /= timeBackwardsPass.size();
    average_time_fp_ms /= timeForwardsPass.size();
    average_percent_derivs /= percentagesDerivsCalculated.size();

    average_opt_time_ms = average_time_derivs_ms + average_time_bp_ms + average_time_fp_ms;

}