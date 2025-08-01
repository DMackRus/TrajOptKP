#include "StdInclude.h"
#include "FileHandler.h"
#include "Visualiser.h"
#include "MuJoCoHelper.h"

#include "ModelTranslator/BoxSweep.h"
#include "ModelTranslator/TwoDPushing.h"
#include "ModelTranslator/ImpactLargeBox.h"

#include "Optimiser/Optimiser.h"
#include "Optimiser/iLQR.h"

// --------------------- Global variables -----------------------
std::shared_ptr<ModelTranslator> activeModelTranslator;
std::shared_ptr<Differentiator> activeDifferentiator;
std::shared_ptr<iLQR> iLQROptimiser;
std::shared_ptr<Visualiser> activeVisualiser;
std::shared_ptr<FileHandler> yamlReader;

std::string CreateTestName(const std::string control_variable,
                           const double control_variable_value,
                           const std::string keypoint_name,
                           const std::string task_name) {

    // Go back two directories
    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    std::string root_path = project_parent_path + "/TestingData/stiffness_tests/" + task_name + "_" + control_variable;

    // Check if optimiser directory exists
    if (!filesystem::exists(root_path)) {
        if (!filesystem::create_directories(root_path)) {
            std::cerr << "Failed to create directory: " << root_path << std::endl;
        }
    }

    std::string method_directory = root_path + "/" + std::to_string(control_variable_value) + "/" + keypoint_name;

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

void OpenLoopOptimisationTest(int task_horizon, int num_tasks,
                              std::string control_variable, double control_variable_value,
                              std::string task_name){
    std::cout << "begining testing openloop optimisation for " << activeModelTranslator->model_name << std::endl;
    std::cout << "optimisation horizon is: " << task_horizon << std::endl;

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
    std::string method_directory = CreateTestName(control_variable, control_variable_value, keypoint_name, task_name);

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

    for (int i = 0; i < num_tasks; i++) {
        std::cout << "trial: " << i << "\n";

        // Reset internal optimisation data and clear key-points cache
        iLQROptimiser->Reset();
        iLQROptimiser->keypoint_generator->ResetCache();
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
        iLQROptimiser->lambda = 0.01;
        std::vector<MatrixXd> optimised_controls = iLQROptimiser->Optimise(
                activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], init_opt_controls, 6, 6,
                task_horizon);

        // --------- Save trial specific information to a folder labelled as trial number --------
        std::string trial_directory = method_directory + "/" + std::to_string(i);
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
    }

    // ----------------------- Save data to file -------------------------------------
    std::string filename = method_directory + "/summary.csv";

    ofstream file_output;
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

void get_solref_solimp(std::string body_name, double solref[2], double solimp[5]){
    int body_id = mj_name2id(activeModelTranslator->MuJoCo_helper->model, mjOBJ_BODY, body_name.c_str());

    for (int i = 0; i < activeModelTranslator->MuJoCo_helper->model->ngeom; ++i) {
        if (activeModelTranslator->MuJoCo_helper->model->geom_bodyid[i] == body_id) {
            solref[0] = activeModelTranslator->MuJoCo_helper->model->geom_solref[i*2 + 0];
            solref[1] = activeModelTranslator->MuJoCo_helper->model->geom_solref[i*2 + 1];

            solimp[0] = activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 0];
            solimp[1] = activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 1];
            solimp[2] = activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 2];
            solimp[3] = activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 3];
            solimp[4] = activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 4];
        }
    }
}

void set_solref_solimp(std::string body_name, double solref[2], double solimp[5]){
    int body_id = mj_name2id(activeModelTranslator->MuJoCo_helper->model, mjOBJ_BODY, body_name.c_str());

    for (int i = 0; i < activeModelTranslator->MuJoCo_helper->model->ngeom; ++i) {
        if (activeModelTranslator->MuJoCo_helper->model->geom_bodyid[i] == body_id) {
            activeModelTranslator->MuJoCo_helper->model->geom_solref[i*2 + 0] = solref[0];
            activeModelTranslator->MuJoCo_helper->model->geom_solref[i*2 + 1] = solref[1];

            activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 0] = solimp[0];
            activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 1] = solimp[1];
            activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 2] = solimp[2];
            activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 3] = solimp[3];
            activeModelTranslator->MuJoCo_helper->model->geom_solimp[i*5 + 4] = solimp[4];
        }
    }
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
    else if(task_name == "pushing_no_clutter") {
        std::shared_ptr<TwoDPushing> myTwoDPush = std::make_shared<TwoDPushing>(noClutter);
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

    // Create a folder directory to save the results for this model

    const int num_tasks = 100;

    //Default solref and solimp values
    double solref[2] = {0.0, 0.0};
    double solimp[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    get_solref_solimp("goal", solref, solimp);


    // Print solref and solimp
    std::cout << "Initial solref: [" << solref[0] << ", " << solref[1] << "]\n";
    std::cout << "Initial solimp: [" << solimp[0] << ", " << solimp[1] << ", " << solimp[2] << ", "
              << solimp[3] << ", " << solimp[4] << "]\n";

    if(1){
        // ----------------- solref tests --------------------------
        double solref_lower = 0.01;
        double solref_upper = 0.1;
        int solref_steps = 20;
        std::vector<double> solref_values;
        for(int i = 0; i < solref_steps; i++){
            double solref_value = solref_lower + (solref_upper - solref_lower) * (double)i / (double)(solref_steps - 1);
            solref_values.push_back(solref_value);
        }

        for(int i = 0; i < solref_values.size(); i++){
            // Set the solref value in the MuJoCo helper
            solref[0] = solref_values[i];
            set_solref_solimp("goal", solref, solimp);

            std::vector<std::string> method_names = {"set_interval", "set_interval", "set_interval", "contact_change", "contact_change_dyn"};
            std::vector<int> method_N_values = {1, 5, 1000, 1, 1}; // 0 for contact_change

            // For loop over keypoint methods
            for(int j = 0; j < method_names.size(); j++){
                keypoint_method method;
                method = iLQROptimiser->ReturnCurrentKeypointMethod();
                method.name = method_names[j];
                method.min_N = method_N_values[j];

                // Set the current keypoint method
                iLQROptimiser->SetCurrentKeypointMethod(method);

                // Perform optimisation over N tasks for this key-point method
                OpenLoopOptimisationTest(opt_horizon, num_tasks, "solref[0]", solref_values[i], task_name);
            }
        }
    }

    // Reset Solimp and solref
//    solref[0] = 0.06;
//
//    // Solimp min tests
//    double solimp0_lower = 0.0;
//    double solimp0_upper = 0.85;
//    int solimp0_steps = 20;
//    std::vector<double> solimp0_values;
//    for(int i = 0; i < solimp0_steps; i++){
//        double solref_value = solimp0_lower + (solimp0_upper - solimp0_lower) * (double)i / (double)(solimp0_steps - 1);
//        solimp0_values.push_back(solref_value);
//    }

//    for(int i = 0; i < solimp0_values.size(); i++){
//        // Set the solref value in the MuJoCo helper
//        solimp[0] = solimp0_values[i];
//        set_solref_solimp("goal", solref, solimp);
//
//        std::string folder_prefix = task_name + "_solimp0_" + std::to_string(solimp0_values[i]);
//
//
//        std::vector<std::string> method_names = {"set_interval", "set_interval", "set_interval", "contact_change"};
//        std::vector<int> method_N_values = {1, 5, 1000, 0}; // 0 for contact_change
//
//        // For loop over keypoint methods
//        for(int j = 0; j < method_names.size(); j++){
//            keypoint_method method;
//            method = iLQROptimiser->ReturnCurrentKeypointMethod();
//            method.name = method_names[j];
//            method.min_N = method_N_values[j];
//
//            // Set the current keypoint method
//            iLQROptimiser->SetCurrentKeypointMethod(method);
//
//            // Perform optimisation over N tasks for this key-point method
//            OpenLoopOptimisationTest(opt_horizon, num_tasks, "solimp[0]", solimp0_values[i]);
//        }
//    }

    return EXIT_SUCCESS;
}