#include "StdInclude.h"
#include "FileHandler.h"
#include "Visualiser.h"
#include "MuJoCoHelper.h"

#include "ModelTranslator/BoxSweep.h"
#include "ModelTranslator/TwoDPushing.h"
#include "ModelTranslator/ImpactLargeBox.h"
#include "ModelTranslator/Walker.h"

#include "Optimiser/Optimiser.h"
#include "Optimiser/iLQR.h"

// --------------------- Global variables -----------------------
std::shared_ptr<ModelTranslator> activeModelTranslator;
std::shared_ptr<Differentiator> activeDifferentiator;
std::shared_ptr<iLQR> iLQROptimiser;
std::shared_ptr<Visualiser> activeVisualiser;
std::shared_ptr<FileHandler> yamlReader;

std::string CreateTestName(const std::string keypoint_name,
                           const std::string task_name) {

    // Go back two directories
    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    std::string root_path = project_parent_path + "/TestingData/figure_1/" + task_name;

    // Check if optimiser directory exists
    if (!filesystem::exists(root_path)) {
        if (!filesystem::create_directories(root_path)) {
            std::cerr << "Failed to create directory: " << root_path << std::endl;
        }
    }

    std::string method_directory = root_path + "/" + keypoint_name;

    // Check if method directory exists, if not create it
    if (!filesystem::exists(method_directory)) {
        if (!filesystem::create_directories(method_directory)) {
            std::cerr << "Failed to create directory: " << method_directory << std::endl;
            exit(1);
        }
    }

    return method_directory;
}

void SaveTestSummaryData(keypoint_method keypoint_method,
                         int opt_horizon,
                         const std::string& testing_directory){

    //  ------------------ make method name ------------------
    std::string keypoint_method_name;
    if(keypoint_method.name == "set_interval") {
        keypoint_method_name = "SI_" + std::to_string(keypoint_method.min_N);
    }
    else{
        keypoint_method_name = keypoint_method.name;
    }


    YAML::Emitter out;

    out << YAML::BeginMap;
    out << YAML::Key << "optimisation horizon";
    out << YAML::Value << opt_horizon;

    out << YAML::Key << "model timestep";
    out << YAML::Value << activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep();

    // -------------------- Keypoint names and other parameters -----------------------
    out << YAML::Key << "keypoint_name";
    out << YAML::Value << keypoint_method_name;

    out << YAML::Key << "keypoint_min_N";
    out << YAML::Value << keypoint_method.min_N;

    out << YAML::Key << "keypoint_max_N";
    out << YAML::Value << keypoint_method.max_N;

    // -------------------------- State vector reduction ----------------------------

    out << YAML::EndMap;

    // Open a file for writing
    std::string file_name = testing_directory + "/summary.yaml";

    std::ofstream fout(file_name);
    fout << out.c_str();
    fout.close();
}

void OpenLoopOptimisationTest(int task_horizon, int task_number,
                              std::string task_name, keypoint_method kp_method){
    std::cout << "Creating figure 1 for " << activeModelTranslator->model_name << " task - with keypoint method: " << std::endl;

    iLQROptimiser->SetCurrentKeypointMethod(kp_method);

    // --------------------------- Data we want to save ------------------------------
    // Individual trajectory information, including;
    // New cost, iteration time, dofs, % derivs, time derivs, time bp, time fp

    // Summary file over all N trajectories, with:
    // Cost reduction, optimisation time, num iterations, avg dofs, avg %derivs, avg time derivs, avg time bp, avg time fp,

    std::string keypoint_name;
    if(iLQROptimiser->activeKeyPointMethod.name == "set_interval") {
        keypoint_name = "SI_" + std::to_string(iLQROptimiser->activeKeyPointMethod.min_N);
    }
    else{
        keypoint_name = iLQROptimiser->activeKeyPointMethod.name;
    }

    // Create the file directory root path dynamically
    std::string method_directory = CreateTestName(keypoint_name, task_name);

    // ------------------------- data storage -------------------------------------
    std::vector<double> cost_reductions;
    std::vector<double> final_costs;
    std::vector<double> optimisation_times;
    std::vector<int>    num_iterations;
    std::vector<double> avg_num_dofs;
    std::vector<double> avg_percent_derivs;
    std::vector<double> total_time_derivs;
    std::vector<double> total_time_keypoint_generation;
    std::vector<double> total_time_FD;
    std::vector<double> total_time_interpolation;
    std::vector<double> total_time_residuals;
    std::vector<double> total_time_bp;
    std::vector<double> total_time_fp;
    // -----------------------------------------------------------------------------

    auto startTimer = std::chrono::high_resolution_clock::now();
    iLQROptimiser->verbose_output = true;

    // Reset internal optimisation data and clear key-points cache
    iLQROptimiser->Reset();
    iLQROptimiser->keypoint_generator->ResetCache();

    // Load the task from CSV file
    yamlReader->LoadTaskFromFile(activeModelTranslator->model_name, task_number, activeModelTranslator->full_state_vector, activeModelTranslator->residual_list);

    // Reset state vector (only really applicable for iLQR_SVR method)
    activeModelTranslator->ResetSVR();
    activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);

    // Setup mj data objects
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);

    for(int i = 0; i < 10; i++){
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
    std::vector<MatrixXd> init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(task_horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data,
                                                          activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(
            activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
            activeModelTranslator->MuJoCo_helper->master_reset_data);

    // Do the optimisation!
    iLQROptimiser->lambda = 0.01;
    std::vector<MatrixXd> optimised_controls = iLQROptimiser->Optimise(
            activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 10, 10,
            task_horizon);

    // --------- Save trial specific information to a folder labelled as trial number --------
    std::string trial_directory = method_directory + "/" + std::to_string(task_number);
    if (!std::filesystem::exists(trial_directory)) {
        std::filesystem::create_directories(trial_directory);
    }

    std::string filename = trial_directory + "/summary.csv";

    ofstream file_output;
    file_output.open(filename);


    // Make header
    file_output << "Iteration" << "," << "Cost" << "," << "Cost reduction" << "," << "time (ms)" << std::endl;

    // Loop through rows
    for(int j = 0; j < iLQROptimiser->num_iterations; j++){
        file_output << j << "," << iLQROptimiser->cost_after_iteration[j] << ",";
        file_output << iLQROptimiser->cost_reduction_after_iteration[j] << "," << iLQROptimiser->time_after_iteration_ms[j] << std::endl;
    }

    file_output.close();


    // ------------------------- Update the data storages -------------------------------------
    cost_reductions.push_back(iLQROptimiser->cost_reduction);
    final_costs.push_back(iLQROptimiser->new_cost);
    optimisation_times.push_back(iLQROptimiser->opt_time_ms);
    num_iterations.push_back(iLQROptimiser->num_iterations);
    avg_num_dofs.push_back(iLQROptimiser->avg_dofs);
    avg_percent_derivs.push_back(iLQROptimiser->avg_percent_derivs);

    total_time_keypoint_generation.push_back(std::accumulate(iLQROptimiser->time_keypoints_ms.begin(), iLQROptimiser->time_keypoints_ms.end(), 0));
    total_time_FD.push_back(std::accumulate(iLQROptimiser->time_FD_derivs_ms.begin(), iLQROptimiser->time_FD_derivs_ms.end(), 0));
    total_time_interpolation.push_back(std::accumulate(iLQROptimiser->time_interpolation_ms.begin(), iLQROptimiser->time_interpolation_ms.end(), 0));
    total_time_residuals.push_back(std::accumulate(iLQROptimiser->time_cost_derivs_ms.begin(), iLQROptimiser->time_cost_derivs_ms.end(), 0));
    total_time_derivs.push_back(std::accumulate(iLQROptimiser->time_get_derivs_ms.begin(), iLQROptimiser->time_get_derivs_ms.end(), 0));
    total_time_bp.push_back(std::accumulate(iLQROptimiser->time_backwards_pass_ms.begin(), iLQROptimiser->time_backwards_pass_ms.end(), 0));
    total_time_fp.push_back(std::accumulate(iLQROptimiser->time_forwardsPass_ms.begin(), iLQROptimiser->time_forwardsPass_ms.end(), 0));

    iLQROptimiser->GenerateDerivatives();

    // -------------------- Save Dynamics Derivatives Data of optimised trajectory -------------------
    // Save A matrices
    std::string A_filename = method_directory + "/A_" + keypoint_name + ".csv";
    ofstream A_file_output;
    A_file_output.open(A_filename);
    for(int t = 0; t < task_horizon; t++){
        for(int r = 0; r < iLQROptimiser->A[t].rows(); r++){
            for(int c = 0; c < iLQROptimiser->A[t].cols(); c++){
                A_file_output << iLQROptimiser->A[t](r,c);
                if(c < iLQROptimiser->A[t].cols() - 1){
                    A_file_output << ",";
                }
            }
            A_file_output << "\n";
        }
    }
    A_file_output.close();

    // Save B matrices
    std::string B_filename = method_directory + "/B_" + keypoint_name + ".csv";
    ofstream B_file_output;
    B_file_output.open(B_filename);
    for(int t = 0; t < task_horizon; t++){
        for(int r = 0; r < iLQROptimiser->B[t].rows(); r++){
            for(int c = 0; c < iLQROptimiser->B[t].cols(); c++){
                B_file_output << iLQROptimiser->B[t](r,c);
                if(c < iLQROptimiser->B[t].cols() - 1){
                    B_file_output << ",";
                }
            }
            B_file_output << "\n";
        }
    }
    B_file_output.close();

    // Save the keypoints used in final optimization as well
    std::string keypoints_filename = method_directory + "/keypoints_" + keypoint_name + ".csv";
    iLQROptimiser->keypoint_generator->keypoints;
    ofstream keypoints_file_output;
    keypoints_file_output.open(keypoints_filename);
//    keypoints_file_output << "time_index,is_keypoint\n";
    for(int t = 0; t < iLQROptimiser->keypoint_generator->keypoints.size(); t++){
//        keypoints_file_output << t << "," << iLQROptimiser->keypoint_generator->keypoints[t] << "\n";
        for(int i = 0; i < iLQROptimiser->keypoint_generator->keypoints[t].size(); i++){
            keypoints_file_output << iLQROptimiser->keypoint_generator->keypoints[t][i] << ",";
        }
        keypoints_file_output << "\n";
    }


    // Change key-point method to SI1 and recompute dynamics derivatives about the final nominal trajectory
    keypoint_method si1_method;
    si1_method.name = "set_interval";
    si1_method.min_N = 1;
    iLQROptimiser->SetCurrentKeypointMethod(si1_method);

    // Compute accurate dynamics derivatives about the final trajectory
    iLQROptimiser->GenerateDerivatives();

    A_filename = method_directory + "/A_SI1.csv";
    A_file_output.open(A_filename);
    for(int t = 0; t < task_horizon; t++){
        for(int r = 0; r < iLQROptimiser->A[t].rows(); r++){
            for(int c = 0; c < iLQROptimiser->A[t].cols(); c++){
                A_file_output << iLQROptimiser->A[t](r,c);
                if(c < iLQROptimiser->A[t].cols() - 1){
                    A_file_output << ",";
                }
            }
            A_file_output << "\n";
        }
    }
    A_file_output.close();

    // Save B matrices
    B_filename = method_directory + "/B_SI1.csv";
    B_file_output.open(B_filename);
    for(int t = 0; t < task_horizon; t++){
        for(int r = 0; r < iLQROptimiser->B[t].rows(); r++){
            for(int c = 0; c < iLQROptimiser->B[t].cols(); c++){
                B_file_output << iLQROptimiser->B[t](r,c);
                if(c < iLQROptimiser->B[t].cols() - 1){
                    B_file_output << ",";
                }
            }
            B_file_output << "\n";
        }
    }
    B_file_output.close();


    //-------------------------------------------------------------------------------

    // -------------------- Save the contact sequence of the optimised trajectory -------------------
    std::string contact_filename = method_directory + "/contact_sequence.csv";
    ofstream contact_file_output;
    contact_file_output.open(contact_filename);
    contact_file_output << "t,contact_id,body_a,body_b\n";
    for(int t = 0; t < iLQROptimiser->contact_list.size(); t++){
        const auto& contacts_t = iLQROptimiser->contact_list[t];
        for (size_t k = 0; k < contacts_t.size(); ++k) {
            contact_file_output << t << "," << k << "," << contacts_t[k].first << "," << contacts_t[k].second << "\n";
        }
    }

    // --------------------- Record Trajectory ------------------------------
    // activeVisualiser->StartRecording(task_name + "_" + keypoint_name);

    int visual_counter = 0;
    const char* label = "";
    auto time_start = std::chrono::steady_clock::now();
    auto time_end = std::chrono::steady_clock::now();

    for(int t = 0; t < optimised_controls.size(); t++){

        time_start = std::chrono::steady_clock::now();

        activeModelTranslator->SetControlVector(optimised_controls[t], activeModelTranslator->MuJoCo_helper->main_data,
                                                activeModelTranslator->current_state_vector);

        mj_step(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->main_data);

        visual_counter++;

        if(visual_counter >= 5){
            visual_counter = 0;
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->main_data);
            activeModelTranslator->MuJoCo_helper->ForwardSimulator(activeModelTranslator->MuJoCo_helper->vis_data);
            activeVisualiser->render(label);
        }

        // ------------------ Real-time synchronisation -----------------------------
        time_end = std::chrono::steady_clock::now();
        std::chrono::duration<double> real_elapsed = time_end - time_start;
        int delay = (activeModelTranslator->MuJoCo_helper->ReturnModelTimeStep() - real_elapsed.count()) * 1000;

        if (delay > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
        }
    }
    // activeVisualiser->StopRecording();


    // ----------------------- Save data to file -------------------------------------
    filename = method_directory + "/summary.csv";

    file_output.open(filename);

    // Make header
    file_output << "Cost reduction" << "," << "Final cost" << "," << "Optimisation time (ms)" << "," << "Number iterations" << ",";
    file_output << "Average num dofs" << "," << "Average percent derivs" << "," << "Total time derivs (ms)" << ",";
    file_output << "Total time keypoints (ms)" << "," << "Total time FD (ms)" << "," << "Total time interpolation (ms)" << ",";
    file_output << "Total time cost derivs (ms)" << "," << "Total time BP (ms)" << "," << "Total time FP (ms)" << std::endl;

    // Loop through rows
    for(int i = 0; i < cost_reductions.size(); i++){
        file_output << cost_reductions[i] << "," << final_costs[i] << "," << optimisation_times[i] << "," << num_iterations[i] << ",";
        file_output << avg_num_dofs[i] << "," << avg_percent_derivs[i] << "," << total_time_derivs[i] << ",";
        file_output << total_time_keypoint_generation[i] << "," << total_time_FD[i] << ",";
        file_output << total_time_interpolation[i] << "," << total_time_residuals[i] << ",";
        file_output << total_time_bp[i] << "," << total_time_fp[i] << std::endl;
    }

    file_output.close();

    SaveTestSummaryData(iLQROptimiser->activeKeyPointMethod, task_horizon,method_directory);
}

int main(int argc, char **argv) {
    std::string config_file_name = "-";
    yamlReader = std::make_shared<FileHandler>();

    if(argc < 2){
        std::cerr << "Usage: " << argv[0] << " <task_name>" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::string task_name = argv[1];
    std::cout << "task_name: " << task_name << std::endl;

    if(task_name == "box_sweep"){
        std::shared_ptr<BoxSweep> myBoxSweep = std::make_shared<BoxSweep>();
        activeModelTranslator = myBoxSweep;
    }
    else if(task_name == "walker" ){
        std::shared_ptr<walker> myWalker = std::make_shared<walker>(PLANE, RUN);
        activeModelTranslator = myWalker;
    }
    else if(task_name == "pushing_no_clutter") {
        std::shared_ptr<TwoDPushing> myTwoDPush = std::make_shared<TwoDPushing>(noClutter);
        activeModelTranslator = myTwoDPush;
    }
    else if(task_name == "pushing_low_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPush = std::make_shared<TwoDPushing>(lowClutter);
        activeModelTranslator = myTwoDPush;
    }
    else if(task_name == "impact_large_box"){
        std::shared_ptr<ImpactLargeBox> myImpactLargeBox = std::make_shared<ImpactLargeBox>();
        activeModelTranslator = myImpactLargeBox;
    }
    else{
        std::cerr << "invalid task name, exiting \n";
        exit(EXIT_FAILURE);
    }

    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Setup the initial horizon, based on open loop or mpc method
    const int opt_horizon = activeModelTranslator->openloop_horizon;

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    // Evaluate the parallelisation effectiveness of the dynamics derivatives computation
    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);

    int task_number = 7;
    // Keypoint method (contact_change)
    keypoint_method kp_method;
    iLQROptimiser->keypoint_generator->ReturnCurrentKeypointMethod();
    kp_method.name = "contact_change";
    OpenLoopOptimisationTest(opt_horizon, task_number, task_name, kp_method);

    // Keypoint method (SI1)
    kp_method.name = "set_interval";
    kp_method.min_N = 1;
    OpenLoopOptimisationTest(opt_horizon, task_number, task_name, kp_method);



    // Create a folder directory to save the results for this model
    // This code needs to set key-point method to contact change method
    // Load a task from csv which can be hardcoded.
    // Create a directory structure to save all the relevant data for later plotting
    // Optimize the trajectory to convergences
    // Save the resulting trajectory final cost, optimization time, iteration timing breakdown. SNapshots of the trajectory.
    // Also save contact sequence for the iteration we care about (can be hardcoded iteration number)?



    return EXIT_SUCCESS;
}