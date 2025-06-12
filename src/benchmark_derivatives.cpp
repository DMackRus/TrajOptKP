#include "StdInclude.h"
#include "FileHandler.h"
#include "Visualiser.h"
#include "MuJoCoHelper.h"

// --------------------- different scenes -----------------------
#include "ModelTranslator/TwoDPushing.h"

#include "Optimiser/Optimiser.h"
#include "Optimiser/iLQR.h"

// --------------------- other -----------------------
#include <mutex>

// --------------------- Global variables -----------------------
std::shared_ptr<ModelTranslator> activeModelTranslator;
std::shared_ptr<Differentiator> activeDifferentiator;
std::shared_ptr<iLQR> iLQROptimiser;
std::shared_ptr<Visualiser> activeVisualiser;
std::shared_ptr<FileHandler> yamlReader;

int main(int argc, char **argv) {

    std::string config_file_name = "benchmark_derivatives";
    yamlReader = std::make_shared<FileHandler>();

    // Instantiate the model translator
    std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(heavyClutter);
    activeModelTranslator = myTwoDPushing;

    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Setup the initial horizon, based on open loop or mpc method
    int opt_horizon = 2000;

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    // Evaluate the parallelisation effectiveness of the dynamics derivatives computation
    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);

    // Generate initial controls for the task
    std::vector<MatrixXd> init_controls;
    std::vector<MatrixXd> optimised_controls;

    std::vector<MatrixXd> init_setup_controls = activeModelTranslator->CreateInitSetupControls(1000);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

    std::vector<MatrixXd> init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
    activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

    // Rollout the initial controls of the trajectory to give a sequence of states to compute dynamics derivatives from
    iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->master_reset_data, true, init_opt_controls);

    // Aliases
    int dof = activeModelTranslator->current_state_vector.dof;

    // -------------------- Time how long it takes in the baseline case -----------------------------------------------
    // Set SI1 keypoint method and time how long it takes to compute dynamics derivatives
    keypoint_method method;
    method = iLQROptimiser->ReturnCurrentKeypointMethod();
    method.min_N = 1;
    iLQROptimiser->SetCurrentKeypointMethod(method);
    iLQROptimiser->keypoint_generator->PrintKeypointMethod();

    // Compute keypoints
    iLQROptimiser->ComputeKeypoints();
    // Compute dynamics derivatives
    auto time_start = std::chrono::high_resolution_clock::now();
    iLQROptimiser->ComputeDynamicsDerivatives();
    auto time_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = time_end - time_start;

    double average_percent_derivs = 0.0;
    for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
        average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[i];
    }
    average_percent_derivs /= activeModelTranslator->current_state_vector.dof;

    std::cout << "------------------------------- Baseline Case (SI1) -------------------------------\n";
    std::cout << "Percentage of dynamics derivatives computed: " << average_percent_derivs << "%\n";
    std::cout << "Time taken to compute dynamics derivatives: " << elapsed.count() << " seconds." << std::endl;

    double baseline_time = elapsed.count();

    // -------------------- Time how long it takes when using SI5 keypoint method -------------------------------------
    method.min_N = 5;
    iLQROptimiser->SetCurrentKeypointMethod(method);
    iLQROptimiser->keypoint_generator->PrintKeypointMethod();

    // Compute keypoints
    iLQROptimiser->ComputeKeypoints();
    // Compute dynamics derivatives
    time_start = std::chrono::high_resolution_clock::now();
    iLQROptimiser->ComputeDynamicsDerivatives();
    time_end = std::chrono::high_resolution_clock::now();
    elapsed = time_end - time_start;

    average_percent_derivs = 0.0;
    for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
        average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[i];
    }
    average_percent_derivs /= activeModelTranslator->current_state_vector.dof;

    std::cout << "------------------------------- (SI5) -------------------------------\n";
    std::cout << "Percentage of dynamics derivatives computed: " << average_percent_derivs << "%\n";
    std::cout << "Time taken to compute dynamics derivatives: " << elapsed.count() << " seconds." << std::endl;
    std:: cout << "Expected time to compute dynamics derivatives: " << baseline_time * (average_percent_derivs / 100.0) << " seconds." << std::endl;


    // Time how long it takes when using a keypoint method where keypoints are staggered.
    iLQROptimiser->keypoint_generator->keypoints.clear();
    // Create a set of keypoints where were still computing about a 5th of the keypoints, but every time-step contains
    // some keypoints
    std::vector<std::vector<int>> partial_rows;
    int effective_dof_interval = 5;
    int dofs_per_row = dof / effective_dof_interval;
    int counter = 0;
    for(int i = 0; i < effective_dof_interval; i++){
        std::vector<int> partial_row;

        if(i == effective_dof_interval - 1) {
            dofs_per_row = dof - counter;
            for(int j = 0; j < dofs_per_row; j++){
                partial_row.push_back(counter);
                counter++;
            }
        }
        else{
            for(int j = 0; j < dofs_per_row; j++){
                partial_row.push_back(counter);
                counter++;
            }
        }
        partial_rows.push_back(partial_row);
    }

    for(int t = 0; t < opt_horizon; t++){
        iLQROptimiser->keypoint_generator->keypoints.push_back(partial_rows[t % effective_dof_interval]);
    }

//    for(int t = 0; t < opt_horizon; t++){
//        std::cout << "time " << t << " :";
//        for(int i = 0; i < iLQROptimiser->keypoint_generator->keypoints[t].size(); i++){
//            std::cout << iLQROptimiser->keypoint_generator->keypoints[t][i] << " ";
//        }
//        std::cout << "\n";
//    }

    iLQROptimiser->keypoint_generator->UpdateLastPercentageDerivatives(iLQROptimiser->keypoint_generator->keypoints);

    // Compute dynamics derivatives
    time_start = std::chrono::high_resolution_clock::now();
    iLQROptimiser->ComputeDynamicsDerivatives();
    time_end = std::chrono::high_resolution_clock::now();
    elapsed = time_end - time_start;

    average_percent_derivs = 0.0;
    for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
        average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[i];
    }
    average_percent_derivs /= activeModelTranslator->current_state_vector.dof;

    std::cout << "------------------------- Staggered derivatives -------------------------------\n";
    std::cout << "Percentage of dynamics derivatives computed: " << average_percent_derivs << "%\n";
    std::cout << "Time taken to compute dynamics derivatives: " << elapsed.count() << " seconds." << std::endl;
    std:: cout << "Expected time to compute dynamics derivatives: " << baseline_time * (average_percent_derivs / 100.0) << " seconds." << std::endl;

    // Time when staggered derivatives are uneven
    iLQROptimiser->keypoint_generator->keypoints.clear();
    std::vector<int> almost_full_row;
    std::vector<int> almost_empty_row;

    for(int i = 1; i < dof; i++){
        almost_full_row.push_back(i);
    }
    almost_empty_row.push_back(0);

    for(int i = 0; i < opt_horizon; i++){
        if(i % 5 == 0){
            iLQROptimiser->keypoint_generator->keypoints.push_back(almost_full_row);
        }
        else{
            iLQROptimiser->keypoint_generator->keypoints.push_back(almost_empty_row);
        }
    }

    iLQROptimiser->keypoint_generator->UpdateLastPercentageDerivatives(iLQROptimiser->keypoint_generator->keypoints);

    for(int t = 0; t < opt_horizon; t++){
        std::cout << "time " << t << " :";
        for(int i = 0; i < iLQROptimiser->keypoint_generator->keypoints[t].size(); i++){
            std::cout << iLQROptimiser->keypoint_generator->keypoints[t][i] << " ";
        }
        std::cout << "\n";
    }

    // Compute dynamics derivatives
    time_start = std::chrono::high_resolution_clock::now();
    iLQROptimiser->ComputeDynamicsDerivatives();
    time_end = std::chrono::high_resolution_clock::now();
    elapsed = time_end - time_start;

    average_percent_derivs = 0.0;
    for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
        average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[i];
    }
    average_percent_derivs /= activeModelTranslator->current_state_vector.dof;

    std::cout << "------------------------- Staggered derivatives -------------------------------\n";
    std::cout << "Percentage of dynamics derivatives computed: " << average_percent_derivs << "%\n";
    std::cout << "Time taken to compute dynamics derivatives: " << elapsed.count() << " seconds." << std::endl;
    std:: cout << "Expected time to compute dynamics derivatives: " << baseline_time * (average_percent_derivs / 100.0) << " seconds." << std::endl;

//    keypoint_generator->GenerateKeyPoints(X_old, A, B);


//    iLQROptimiser->ComputeDynamicsDerivatives();

    return EXIT_SUCCESS;
}