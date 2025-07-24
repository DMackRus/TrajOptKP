#include "StdInclude.h"
#include "FileHandler.h"
#include "Visualiser.h"
#include "MuJoCoHelper.h"

// --------------------- different scenes -----------------------
#include "ModelTranslator/TwoDPushing.h"
#include "ModelTranslator/Acrobot.h"
#include "ModelTranslator/PlaceObject.h"
#include "ModelTranslator/BoxSweep.h"
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

double ApproximationError(std::vector<MatrixXd> &A_SI1, std::vector<MatrixXd> &B_SI1,
                          std::vector<MatrixXd> &A_approximated, std::vector<MatrixXd> &B_approximated){
    double error = 0.0;
    size_t T = A_SI1.size();
    int nx = A_SI1[0].rows();
    int nu = B_SI1[0].cols();

    // Loop through the matrices over the time horizon and compute absolute error
    for(int t = 0; t < T; t++){
        double A_error, B_error = 0.0;

        // Compute the absolute error for A matrices
        for(int i = 0; i < A_SI1[t].rows(); i++){
            for(int j = 0; j < A_SI1[t].cols(); j++){
                A_error += std::abs(A_SI1[t](i, j) - A_approximated[t](i, j));
            }
        }

        // Compute the absolute error for B matrices
        for(int i = 0; i < B_SI1[t].rows(); i++){
            for(int j = 0; j < B_SI1[t].cols(); j++){
                B_error += std::abs(B_SI1[t](i, j) - B_approximated[t](i, j));
            }
        }

        error += (A_error / (nx * nx)) + (B_error / (nx * nu));
    }

    return error;
}

int assign_task(std::string task){
    if(task == "acrobot"){
        std::shared_ptr<Acrobot> myAcrobot = std::make_shared<Acrobot>();
        activeModelTranslator = myAcrobot;
    }
    else if(task == "pushing_no_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(noClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_low_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(lowClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "pushing_moderate_clutter"){
        std::shared_ptr<TwoDPushing> myTwoDPushing = std::make_shared<TwoDPushing>(heavyClutter);
        activeModelTranslator = myTwoDPushing;

    }
    else if(task == "place_low_clutter"){
        std::shared_ptr<PlaceObject> my_place_object = std::make_shared<PlaceObject>("end_effector", "goal", lowClutter);
        activeModelTranslator = my_place_object;
    }
    else if(task == "place_heavy_clutter"){
        std::shared_ptr<PlaceObject> my_place_object = std::make_shared<PlaceObject>("end_effector", "goal", heavyClutter);
        activeModelTranslator = my_place_object;
    }
    else if(task == "walker_run"){
        std::shared_ptr<walker> myLocomotion = std::make_shared<walker>(PLANE, RUN);
        activeModelTranslator = myLocomotion;
    }
    else if(task == "box_sweep"){
        std::shared_ptr<BoxSweep> myBoxSweep = std::make_shared<BoxSweep>();
        activeModelTranslator = myBoxSweep;
    }
    else if(task == "impact_large_box"){
        std::shared_ptr<ImpactLargeBox> my_impact_large_box = std::make_shared<ImpactLargeBox>();
        activeModelTranslator = my_impact_large_box;
    }
    else{
        std::cout << "invalid scene selected, " << task << " does not exist" << std::endl;
    }
    return EXIT_SUCCESS;
}

int main(int argc, char **argv) {

    if(argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <task_name>" << std::endl;
        return EXIT_FAILURE;
    }

    std::string config_file_name = "benchmark_derivatives";
    yamlReader = std::make_shared<FileHandler>();

    std::string task_name = argv[1];
    assign_task(task_name);


    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    // Setup the initial horizon, based on open loop or mpc method
    int opt_horizon = 1000;

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    // Evaluate the parallelisation effectiveness of the dynamics derivatives computation
    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);

    // Data storage - TODO: Make this a better data structure that isnt hardcoded to 4 methods.
    std::vector<MatrixXd> A_matrices_SI1, B_matrices_SI1;
    std::vector<MatrixXd> A_matrices_SI5, B_matrices_SI5;
    std::vector<MatrixXd> A_matrices_SI1000, B_matrices_SI1000;
    std::vector<MatrixXd> A_matrices_contact_change, B_matrices_contact_change;

    std::vector<std::vector<double>> approximation_errors, percentage_derivatives;
    // TODO - This is also hardcoded to 3 methods
    approximation_errors.resize(3);
    percentage_derivatives.resize(3);

    // Loop over 100 tasks
    int data_counter = 0;
    int task_counter = 0;
    int iteration_counter = 0;
    const int NUM_DATA_POINTS = 100;
    const int MAX_ITERATIONS_PER_TASK = 5;
    bool new_base_task = true;
    std::vector<MatrixXd> init_controls;
    std::vector<MatrixXd> optimised_controls;
    while(data_counter < NUM_DATA_POINTS){

        if(new_base_task){

            std::string task_prefix = activeModelTranslator->model_name;
            yamlReader->LoadTaskFromFile(task_prefix, task_counter, activeModelTranslator->full_state_vector,
                                         activeModelTranslator->residual_list);
            activeModelTranslator->full_state_vector.Update();
            activeModelTranslator->current_state_vector = activeModelTranslator->full_state_vector;
            activeModelTranslator->UpdateSceneVisualisation();

            activeModelTranslator->InitialiseSystemToStartState(activeModelTranslator->MuJoCo_helper->master_reset_data);

            std::vector<MatrixXd> init_setup_controls = activeModelTranslator->CreateInitSetupControls(1000);
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->master_reset_data, activeModelTranslator->MuJoCo_helper->main_data);

            std::vector<MatrixXd> init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

            optimised_controls = init_opt_controls;

            // Rollout the initial controls of the trajectory to give a sequence of states to compute dynamics derivatives from
            iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->master_reset_data, true, init_opt_controls);

            new_base_task = false;

            // Render
//            if(1){
//                for(int t = 0; t < opt_horizon; t++){
//                    // Set the state
//                    activeModelTranslator->SetStateVector(iLQROptimiser->X_old[t], activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector);
//
//                    mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
//
//                    activeVisualiser->render("Visualise");
//                }
//            }
        }

        // ----- Compute the accurate dynamics derivatives via SI1 method -----
        keypoint_method method;
        method = iLQROptimiser->ReturnCurrentKeypointMethod();
        method.min_N = 1;
        method.name = "set_interval";
        iLQROptimiser->SetCurrentKeypointMethod(method);
        iLQROptimiser->keypoint_generator->PrintKeypointMethod();
        iLQROptimiser->GenerateDerivatives();

        A_matrices_SI1 = iLQROptimiser->A;
        B_matrices_SI1 = iLQROptimiser->B;

        // ----- Compute the approximated dynamics derivatives via SI5 method -----
        method.min_N = 5;
        iLQROptimiser->SetCurrentKeypointMethod(method);
        iLQROptimiser->GenerateDerivatives();

        A_matrices_SI5 = iLQROptimiser->A;
        B_matrices_SI5 = iLQROptimiser->B;
        approximation_errors[0].push_back(ApproximationError(A_matrices_SI1, B_matrices_SI1, A_matrices_SI5, B_matrices_SI5));
        double average_percent_derivs = 0.0;
        for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
            average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[i];
        }
        average_percent_derivs /= activeModelTranslator->current_state_vector.dof;
        percentage_derivatives[0].push_back(average_percent_derivs);

        // ----- Compute the approximated dynamics derivatives via SI1000 method -----
        method.min_N = 1000;
        iLQROptimiser->SetCurrentKeypointMethod(method);
        iLQROptimiser->GenerateDerivatives();

        A_matrices_SI1000 = iLQROptimiser->A;
        B_matrices_SI1000 = iLQROptimiser->B;
        approximation_errors[1].push_back(ApproximationError(A_matrices_SI1, B_matrices_SI1, A_matrices_SI1000, B_matrices_SI1000));
        average_percent_derivs = 0.0;
        for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
            average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[i];
        }
        average_percent_derivs /= activeModelTranslator->current_state_vector.dof;
        percentage_derivatives[1].push_back(average_percent_derivs);

        // ----- Compute the approximated dynamics derivatives via contact_change method -----
        method.min_N = 1;
        method.name = "contact_change";
        iLQROptimiser->SetCurrentKeypointMethod(method);
        iLQROptimiser->GenerateDerivatives();

        A_matrices_contact_change = iLQROptimiser->A;
        B_matrices_contact_change = iLQROptimiser->B;
        approximation_errors[2].push_back(ApproximationError(A_matrices_SI1, B_matrices_SI1, A_matrices_contact_change, B_matrices_contact_change));
        average_percent_derivs = 0.0;
        for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
            average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[i];
        }
        average_percent_derivs /= activeModelTranslator->current_state_vector.dof;
        percentage_derivatives[2].push_back(average_percent_derivs);

        // Progress the task counter and task
        if(iteration_counter < MAX_ITERATIONS_PER_TASK){
            // Change keypoint method to SI1 - perform one iteration of optimisation
            method.min_N = 1;
            method.name = "set_interval";
            iLQROptimiser->SetCurrentKeypointMethod(method);
            optimised_controls = iLQROptimiser->Optimise(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0],
                                               optimised_controls, 1,
                                                1, opt_horizon);
            iteration_counter++;
        }
        else{
            iteration_counter = 0;
            task_counter++;
            new_base_task = true;
        }

        data_counter++;
    }

    // Compute average approximation errors and percentage derivatives for all methods
    std::vector<double> average_approximation_errors(3, 0.0);
    std::vector<double> average_percentage_derivatives(3, 0.0);

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < NUM_DATA_POINTS; j++){
            average_approximation_errors[i] += approximation_errors[i][j];
            average_percentage_derivatives[i] += percentage_derivatives[i][j];
        }
        average_approximation_errors[i] /= NUM_DATA_POINTS;
        average_percentage_derivatives[i] /= NUM_DATA_POINTS;
    }

    // Print the results
    std::cout << "Average Approximation Errors:\n";
    for(int i = 0; i < 3; i++){
        std::cout << "Method " << i + 1 << ": " << average_approximation_errors[i] << "\n";
    }
    std::cout << "Average Percentage Derivatives:\n";
    for(int i = 0; i < 3; i++){
        std::cout << "Method " << i + 1 << ": " << average_percentage_derivatives[i] << "\n";
    }
    // Save the results to a file


    return EXIT_SUCCESS;
}