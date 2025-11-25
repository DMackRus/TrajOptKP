#include "StdInclude.h"
#include "FileHandler.h"
#include "Visualiser.h"
#include "MuJoCoHelper.h"

#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <vector>
#include <cassert>

// --------------------- different scenes -----------------------
#include "ModelTranslator/TwoDPushing.h"
#include "ModelTranslator/Acrobot.h"
#include "ModelTranslator/PlaceObject.h"
#include "ModelTranslator/BoxSweep.h"
#include "ModelTranslator/ImpactLargeBox.h"
#include "ModelTranslator/Walker.h"
#include "ModelTranslator/PistonBlock.h"

#include "Optimiser/Optimiser.h"
#include "Optimiser/iLQR.h"

// --------------------- Global variables -----------------------
std::shared_ptr<ModelTranslator> activeModelTranslator;
std::shared_ptr<Differentiator> activeDifferentiator;
std::shared_ptr<iLQR> iLQROptimiser;
std::shared_ptr<Visualiser> activeVisualiser;
std::shared_ptr<FileHandler> yamlReader;

//void ApproximationError(std::vector<MatrixXd> &A_SI1, std::vector<MatrixXd> &B_SI1,
//                          std::vector<MatrixXd> &A_approximated, std::vector<MatrixXd> &B_approximated,
//                          double &max_error, double &rms_error, double &mean_squared_error){
//    max_error = 0.0, rms_error = 0.0, mean_squared_error = 0.0;
//
//    size_t T = A_SI1.size();
//    int nx = A_SI1[0].rows();
//    int nu = B_SI1[0].cols();
//
//    // Loop through the matrices over the time horizon and compute absolute error
//    for(int t = 0; t < T; t++){
//        double A_error= 0.0, B_error = 0.0;
//        double total_error = 0.0;
//        double total_error_scaled = 0.0;
//
////        std::cout << "A_SI1[" << t << "] = \n" << A_SI1[t] << "\n";
////        std::cout << "A_approximated[" << t << "] = \n" << A_SI1[t] << "\n";
//
//        // ----------------------------------------------------------------
//        double denominator = 0.0;
//        for(int i = 0; i < A_SI1[t].rows(); i++){
//            for(int j = 0; j < A_SI1[t].cols(); j++){
//                denominator += pow(A_SI1[t](i, j), 2);
//            }
//        }
//        for(int i = 0; i < B_SI1[t].rows(); i++){
//            for(int j = 0; j < B_SI1[t].cols(); j++){
//                denominator += pow(B_SI1[t](i, j), 2);
//            }
//        }
//        // ----------------------------------------------------------------
//
//        // Compute the squared error for A matrices
//        for(int i = 0; i < A_SI1[t].rows(); i++){
//            for(int j = 0; j < A_SI1[t].cols(); j++){
//                A_error += pow((A_SI1[t](i, j) - A_approximated[t](i, j)),2);
//            }
//        }
////        std::cout << "A_error = " << A_error << "\n";
//
//        // Compute the squared error for B matrices
//        for(int i = 0; i < B_SI1[t].rows(); i++){
//            for(int j = 0; j < B_SI1[t].cols(); j++){
//                B_error += pow((B_SI1[t](i, j) - B_approximated[t](i, j)), 2);
//            }
//        }
//
//        // Keep track of total error
//        total_error = (A_error / (nx * nx)) + (B_error / (nx * nu));
//        total_error_scaled = total_error / denominator;
//
//
//        if(total_error_scaled > max_error) max_error = total_error_scaled;
//
//        mean_squared_error += total_error;
//
//
////        error += (A_error / (nx * nx)) + (B_error / (nx * nu));
//    }
//
//    mean_squared_error /= T;
//
//    double denominator = 0.0;
//    for(int t = 0; t < T; t++){
//        for(int i = 0; i < A_SI1[t].rows(); i++){
//            for(int j = 0; j < A_SI1[t].cols(); j++){
//                denominator += pow(A_SI1[t](i, j), 2);
//            }
//        }
//        for(int i = 0; i < B_SI1[t].rows(); i++){
//            for(int j = 0; j < B_SI1[t].cols(); j++){
//                denominator += pow(B_SI1[t](i, j), 2);
//            }
//        }
//    }
//
//    denominator /= T;
//    rms_error = sqrt(mean_squared_error) / sqrt(denominator);
//}

//void ApproximationError(
//        const std::vector<Eigen::MatrixXd> &A_exact,
//        const std::vector<Eigen::MatrixXd> &B_exact,
//        const std::vector<Eigen::MatrixXd> &A_approx,
//        const std::vector<Eigen::MatrixXd> &B_approx,
//        double &mse,
//        double &frobenius_error,
//        double &elementnorm_mse_error,
//        double &elementrmsnorm_error,
//        double &max_abs_error,
//        double &max_rel_error)
//{
//    // Safety check assertions
//    assert(A_exact.size() == A_approx.size());
//    assert(B_exact.size() == B_approx.size());
//    size_t T = A_exact.size();
//
//    max_abs_error = 0.0;
//    max_rel_error = 0.0;
//    mse = 0.0;
//
//
//}

void ApproximationError(
        const std::vector<Eigen::MatrixXd> &A_exact,
        const std::vector<Eigen::MatrixXd> &B_exact,
        const std::vector<Eigen::MatrixXd> &A_approx,
        const std::vector<Eigen::MatrixXd> &B_approx,
        double &mse,
        double &frobenius_error,
        double &elementnorm_mse_error,
        double &max_abs_error,
        double &max_rel_error)
{
    assert(A_exact.size() == A_approx.size());
    assert(B_exact.size() == B_approx.size());
    size_t T = A_exact.size();

    const double eps = 1e-8;

    mse = 0.0;
    frobenius_error = 0.0;
    elementnorm_mse_error = 0.0;
    max_abs_error = 0.0;
    max_rel_error = 0.0;

    // 1. Compute global MSE and max absolute error
    size_t total_elements = 0;
    for (size_t t = 0; t < T; ++t) {
        Eigen::MatrixXd diffA = A_exact[t] - A_approx[t];
        Eigen::MatrixXd diffB = B_exact[t] - B_approx[t];
        double step_mse = diffA.squaredNorm() + diffB.squaredNorm();
        mse += step_mse;

        if(step_mse > max_abs_error){
            max_abs_error = step_mse;
        }
    }
    // Normalise MSE by total number of elements across all timesteps
    mse /= static_cast<double>((A_exact[0].size() + B_exact[0].size()) * T);
    max_abs_error /= static_cast<double>(A_exact[0].size() + B_exact[0].size());

    // 2. Compute per-timestep relative Frobenius error (for both A and B)
    for (size_t t = 0; t < T; ++t) {
        // Eigen .norm() returns sum squared of all values and square root of this sum
        double errA = (A_exact[t] - A_approx[t]).norm();
        double errB = (B_exact[t] - B_approx[t]).norm();
        double normA = A_exact[t].norm();
        double normB = B_exact[t].norm();

        double relA = errA / (normA + eps);
        double relB = errB / (normB + eps);
        double rel_t = relA + relB;

        frobenius_error += rel_t;
        if (rel_t > max_rel_error) max_rel_error = rel_t;
    }
    frobenius_error /= static_cast<double>((A_exact[0].size() + B_exact[0].size()) * T);
    max_rel_error /= static_cast<double>(A_exact[0].size() + B_exact[0].size());

    // 3. Compute elementwise norm matrices for A and B based on exact values
    Eigen::MatrixXd minA = Eigen::MatrixXd::Zero(A_exact[0].rows(), A_exact[0].cols());
    Eigen::MatrixXd maxA = Eigen::MatrixXd::Zero(A_exact[0].rows(), A_exact[0].cols());
    Eigen::MatrixXd minB = Eigen::MatrixXd::Zero(B_exact[0].rows(), B_exact[0].cols());
    Eigen::MatrixXd maxB = Eigen::MatrixXd::Zero(B_exact[0].rows(), B_exact[0].cols());

    // Find the min and max values for each element across all timesteps
    for(int t = 0; t < T; t++){
        for(int i = 0; i < A_exact[t].rows(); i++){
            for(int j = 0; j < A_exact[t].cols(); j++){
                if(t == 0){
                    minA(i, j) = A_exact[t](i, j);
                    maxA(i, j) = A_exact[t](i, j);
                }
                else{
                    if(A_exact[t](i, j) < minA(i, j)) minA(i, j) = A_exact[t](i, j);
                    if(A_exact[t](i, j) > maxA(i, j)) maxA(i, j) = A_exact[t](i, j);
                }
            }
        }
        for(int i = 0; i < B_exact[t].rows(); i++){
            for(int j = 0; j < B_exact[t].cols(); j++){
                if(t == 0){
                    minB(i, j) = B_exact[t](i, j);
                    maxB(i, j) = B_exact[t](i, j);
                }
                else{
                    if(B_exact[t](i, j) < minB(i, j)) minB(i, j) = B_exact[t](i, j);
                    if(B_exact[t](i, j) > maxB(i, j)) maxB(i, j) = B_exact[t](i, j);
                }
            }
        }
     }

    // Normalise the exact and approximate matrices using these min and max values
    for(int t = 0; t < T; t++){
        Eigen::MatrixXd normA_exact = Eigen::MatrixXd::Zero(A_exact[t].rows(), A_exact[t].cols());
        Eigen::MatrixXd normA_approx = Eigen::MatrixXd::Zero(A_approx[t].rows(), A_approx[t].cols());
        Eigen::MatrixXd normB_exact = Eigen::MatrixXd::Zero(B_exact[t].rows(), B_exact[t].cols());
        Eigen::MatrixXd normB_approx = Eigen::MatrixXd::Zero(B_approx[t].rows(), B_approx[t].cols());

        for(int i = 0; i < A_exact[t].rows(); i++){
            for(int j = 0; j < A_exact[t].cols(); j++){
                normA_exact(i, j) = (A_exact[t](i, j) - minA(i, j)) / (maxA(i, j) - minA(i, j) + eps);
                normA_approx(i, j) = (A_approx[t](i, j) - minA(i, j)) / (maxA(i, j) - minA(i, j) + eps);
            }
        }
        for(int i = 0; i < B_exact[t].rows(); i++){
            for(int j = 0; j < B_exact[t].cols(); j++){
                normB_exact(i, j) = (B_exact[t](i, j) - minB(i, j)) / (maxB(i, j) - minB(i, j) + eps);
                normB_approx(i, j) = (B_approx[t](i, j) - minB(i, j)) / (maxB(i, j) - minB(i, j) + eps);
            }
        }

        // Compute the MSE between the normalised exact and approximate matrices
        double step_mse = (normA_exact - normA_approx).squaredNorm() + (normB_exact - normB_approx).squaredNorm();
        elementnorm_mse_error += step_mse;
    }
    elementnorm_mse_error /= static_cast<double>((A_exact[0].size() + B_exact[0].size()) * T);
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
    else if(task == "piston_block"){
        std::shared_ptr<PistonBlock> my_piston_block = std::make_shared<PistonBlock>();
        activeModelTranslator = my_piston_block;
    }
    else{
        std::cout << "invalid scene selected, " << task << " does not exist" << std::endl;
    }
    return EXIT_SUCCESS;
}

int ApproximationAccuracyVersusOptimisationPerformance(int argc, char **argv){
    // Arguments {task_name}, {num_data_points}, {opt_horizon}, {num_opt_iterations}
    if(argc < 5) {
        std::cerr << "NOT ENOUGH ARGUMENTS PROVIDED (task_name, num_data_points,  opt_horizon, num_opt_iterations) \n";
        return EXIT_FAILURE;
    }

//    std::string config_file_name = "benchmark_derivatives";
    yamlReader = std::make_shared<FileHandler>();

    std::string task_name = argv[1];
    int num_data_points = std::stoi(argv[2]);
    int opt_horizon = std::stoi(argv[3]);
    int num_opt_iterations = std::stoi(argv[4]);
    assign_task(task_name);

    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    // Evaluate the parallelisation effectiveness of the dynamics derivatives computation
    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);

    std::vector<MatrixXd> A_matrices_SI1, B_matrices_SI1;

    std::vector<std::string> methods = {"SI2", "SI5", "SI10", "SI20", "SI100", "SI200", "SI500", "SI1000", "contact_change", "contact_change_dyn", "contact_change_maxN"};
    std::vector<std::string> keypoint_methods = {"set_interval", "set_interval", "set_interval", "set_interval", "set_interval",
                                                 "set_interval", "set_interval", "set_interval", "contact_change", "contact_change_dyn", "contact_change_maxN"};
    std::vector<int> min_n_values = {2, 5, 10, 20, 100, 200, 500, 1000, 1, 1, 1};
    std::vector<int> max_n_values = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 20};

    // Make sure all vectors are the same size
    assert(methods.size() == keypoint_methods.size());
    assert(methods.size() == min_n_values.size());
    assert(methods.size() == max_n_values.size());

//    std::vector<std::string> methods = {"contact_change_maxN"};
//    std::vector<std::string> keypoint_methods = {"contact_change_maxN"};
//    std::vector<int> min_n_values = {1};
//    std::vector<int> max_n_values = {20};

    // Create Vectors of MatrixXd to store A and B matrices for each method
    std::vector<std::vector<MatrixXd>> A_matrices(methods.size()), B_matrices(methods.size());

    // Size matrices appropriately
    for(int i = 0; i < methods.size(); i++){
        A_matrices[i].resize(opt_horizon, MatrixXd::Zero(activeModelTranslator->current_state_vector.dof, activeModelTranslator->current_state_vector.dof));
        B_matrices[i].resize(opt_horizon, MatrixXd::Zero(activeModelTranslator->current_state_vector.dof, activeModelTranslator->current_state_vector.num_ctrl));
    }

    std::vector<std::vector<double>> mean_squared_error,
            frobenius_errors,
            elementnorm_mse_errors,
            max_abs_error,
            max_rel_error,
            percentage_derivatives,
            cost_reductions;

    mean_squared_error.resize(methods.size());
    frobenius_errors.resize(methods.size());
    elementnorm_mse_errors.resize(methods.size());
    max_abs_error.resize(methods.size());
    max_rel_error.resize(methods.size());
    cost_reductions.resize(methods.size());

    percentage_derivatives.resize(methods.size());

    // Loop over 100 tasks
    int data_counter = 0;
    int task_counter = 0;
    int iteration_counter = 0;
    const int NUM_DATA_POINTS = num_data_points;
    const int MAX_ITERATIONS_PER_TASK = num_opt_iterations;

    const double LAMBDA = 0.001;
    bool new_base_task = true;
    std::vector<MatrixXd> init_controls;
    std::vector<MatrixXd> optimised_controls;
    while(data_counter < NUM_DATA_POINTS){

        std::vector<MatrixXd> init_opt_controls;
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

            init_opt_controls = activeModelTranslator->CreateInitOptimisationControls(opt_horizon);
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->main_data, activeModelTranslator->MuJoCo_helper->master_reset_data);
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->saved_systems_state_list[0], activeModelTranslator->MuJoCo_helper->master_reset_data);
            activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->master_reset_data);

            optimised_controls = init_opt_controls;

            // Rollout the initial controls of the trajectory to give a sequence of states to compute dynamics derivatives from
            iLQROptimiser->RolloutTrajectory(activeModelTranslator->MuJoCo_helper->master_reset_data, true, init_opt_controls);

            new_base_task = false;
        }


        // Render
//        if(1){
//            for(int t = 0; t < opt_horizon; t++){
//                // Set the state
//                activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[t]);
////                activeModelTranslator->SetStateVectorQuat(iLQROptimiser->X_old[t], activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector);
////                std::cout << "t = " << t << ", state = " << iLQROptimiser->X_old[t].transpose() << "\n";
//
//                mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
//
//                activeVisualiser->render("Visualise");
//            }
//        }


        // Perform some tests where we optimise the trajectory with SI1 and our contact method and compare opt performance
        // As well as logging error metrics to compare against

        // ----- Compute the accurate dynamics derivatives via SI1 method -----
        keypoint_method method;
        method = iLQROptimiser->ReturnCurrentKeypointMethod();
        method.min_N = 1;
        method.name = "set_interval";
        iLQROptimiser->SetCurrentKeypointMethod(method);
//        iLQROptimiser->GenerateDerivatives();

        init_opt_controls = optimised_controls;
//        std::cout << "init opt controls [0] = \n" << init_opt_controls[0] << "\n";


        for(int i = 0; i < methods.size(); i++){
            method.min_N = min_n_values[i];
            method.max_N = max_n_values[i];
            method.name = keypoint_methods[i];
            iLQROptimiser->SetCurrentKeypointMethod(method);

            // Always Set Lambda to the same constant
            iLQROptimiser->lambda = LAMBDA;
            std::vector<MatrixXd> curr_opt_controls = iLQROptimiser->Optimise(activeModelTranslator->MuJoCo_helper->master_reset_data,
                                                        init_opt_controls, 1,
                                                        1, opt_horizon);

            cost_reductions[i].push_back(iLQROptimiser->cost_reduction);

            A_matrices[i] = iLQROptimiser->A;
            B_matrices[i] = iLQROptimiser->B;

            double average_percent_derivs = 0.0;
            for(int j = 0; j < activeModelTranslator->current_state_vector.dof; j++){
                average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[j];
            }
            average_percent_derivs /= activeModelTranslator->current_state_vector.dof;
            percentage_derivatives[i].push_back(average_percent_derivs);
        }


        // Always Optimise with Set Interval 1
        method.min_N = 1;
        method.name = "set_interval";
        iLQROptimiser->SetCurrentKeypointMethod(method);
        // Always Set Lambda to the same constant
        iLQROptimiser->lambda = LAMBDA;
        optimised_controls = iLQROptimiser->Optimise(activeModelTranslator->MuJoCo_helper->master_reset_data,
                                                     init_opt_controls, 1,
                                                     1, opt_horizon);

        A_matrices_SI1 = iLQROptimiser->A;
        B_matrices_SI1 = iLQROptimiser->B;

        double cost_reduction_baseline = iLQROptimiser->cost_reduction;

        // Compute Error metrics for all methods
        for(int i = 0; i < methods.size(); i++){
            double mse, frobenius_error, elementnorm_mse_error, max_abs_err, max_rel_err;
            ApproximationError(A_matrices_SI1, B_matrices_SI1, A_matrices[i], B_matrices[i],
                               mse, frobenius_error, elementnorm_mse_error, max_abs_err, max_rel_err);

            mean_squared_error[i].push_back(mse);
            frobenius_errors[i].push_back(frobenius_error);
            elementnorm_mse_errors[i].push_back(elementnorm_mse_error);
            max_abs_error[i].push_back(max_abs_err);
            max_rel_error[i].push_back(max_rel_err);

//            cost_reductions[i].back() -= cost_reduction_baseline;
        }

        // Progress the task counter and task
        if(iteration_counter < MAX_ITERATIONS_PER_TASK){
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
    std::vector<double> averaged_mse_error(methods.size(), 0.0);
    std::vector<double> averaged_frobenius_error(methods.size(), 0.0);
    std::vector<double> averaged_elementnorm_mse_error(methods.size(), 0.0);
    std::vector<double> average_max_abs_error(methods.size(), 0.0);
    std::vector<double> average_max_rel_error(methods.size(), 0.0);
    std::vector<double> average_cost_reductions(methods.size(), 0.0);

    std::vector<double> average_percentage_derivatives(methods.size(), 0.0);

    for(int i = 0; i < methods.size(); i++){
        for(int j = 0; j < NUM_DATA_POINTS; j++){
            averaged_mse_error[i] += mean_squared_error[i][j];
            averaged_frobenius_error[i] += frobenius_errors[i][j];
            averaged_elementnorm_mse_error[i] += elementnorm_mse_errors[i][j];
            average_max_abs_error[i] += max_abs_error[i][j];
            average_max_rel_error[i] += max_rel_error[i][j];
            average_cost_reductions[i] += cost_reductions[i][j];

            average_percentage_derivatives[i] += percentage_derivatives[i][j];
        }
        averaged_mse_error[i] /= NUM_DATA_POINTS;
        averaged_frobenius_error[i] /= NUM_DATA_POINTS;
        averaged_elementnorm_mse_error[i] /= NUM_DATA_POINTS;
        average_max_abs_error[i] /= NUM_DATA_POINTS;
        average_max_rel_error[i] /= NUM_DATA_POINTS;
        average_cost_reductions[i] /= NUM_DATA_POINTS;

        average_percentage_derivatives[i] /= NUM_DATA_POINTS;
    }

    const int w_method = 20;
    const int w_num    = 18;

    std::cout << std::left
              << std::setw(w_method) << "Method"
              << std::right
              << std::setw(w_num) << "MSE"
              << std::setw(w_num) << "Frob Err"
              << std::setw(w_num) << "Elem Norm MSE"
              << std::setw(w_num) << "Max Error (abs)"
              << std::setw(w_num) << "Max Error (rel)"
              << std::setw(w_num) << "% Cost Reduct"
              << std::setw(w_num) << "% Derivatives"
              << "\n";

    for (int i = 0; i < methods.size(); i++) {
        std::cout << std::left << std::setw(w_method) << methods[i]
                  << std::right << std::setw(w_num) << std::fixed << std::setprecision(4) << averaged_mse_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(5) << averaged_frobenius_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(5) << averaged_elementnorm_mse_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(4) << average_max_abs_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(4) << average_max_rel_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(2) << average_cost_reductions[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(2) << average_percentage_derivatives[i]
                  << "\n";
    }

    // --------------- Save the results to a file --------------------------
    // directory "DerivativeErrorData / {{task_name}_{opt_horizon}_{num_iterations}} / {method_name}.csv

    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    // Check folder exists, if it does not create one
    std::string folder_name = project_parent_path + "/DerivativeErrorData/" + task_name + "_" + std::to_string(opt_horizon) + "_" + std::to_string(num_opt_iterations) + "/";
    if(!std::filesystem::exists(folder_name)){
        std::filesystem::create_directories(folder_name);
    }

    for(int i = 0; i < methods.size(); i++){
        std::string file_path = folder_name + methods[i] + ".csv";
        std::ofstream file(file_path);
        if(file.is_open()){
            file << "MSE,Frobenius Error,Elementnorm Error,Max Error (abs),Max Error (rel),Cost Reduction,% Derivatives\n";
            for(int j = 0; j < NUM_DATA_POINTS; j++){
                file << mean_squared_error[i][j] << ","
                     << frobenius_errors[i][j] << ","
                     << elementnorm_mse_errors[i][j] << ","
                     << max_abs_error[i][j] << ","
                     << max_rel_error[i][j] << ","
                     << cost_reductions[i][j] << ","
                     << percentage_derivatives[i][j] << "\n";
            }
            file.close();
        }
        else{
            std::cerr << "Could not open file: " << file_path << "\n";
        }
    }

    return EXIT_SUCCESS;
}

int main(int argc, char **argv) {

    if(1){
        ApproximationAccuracyVersusOptimisationPerformance(argc, argv);
        return EXIT_SUCCESS;
    }


    // Arguments {task_name}, {num_data_points}, {opt_horizon}, {num_opt_iterations}
    if(argc < 5) {
        std::cerr << "NOT ENOUGH ARGUMENTS PROVIDED (task_name, num_data_points,  opt_horizon, num_opt_iterations) \n";
        return EXIT_FAILURE;
    }

//    std::string config_file_name = "benchmark_derivatives";
    yamlReader = std::make_shared<FileHandler>();

    std::string task_name = argv[1];
    int num_data_points = std::stoi(argv[2]);
    int opt_horizon = std::stoi(argv[3]);
    int num_opt_iterations = std::stoi(argv[4]);
    assign_task(task_name);

    // Instantiate the differentiator
    activeDifferentiator = std::make_shared<Differentiator>(activeModelTranslator, activeModelTranslator->MuJoCo_helper);

    activeModelTranslator->MuJoCo_helper->AppendSystemStateToEnd(activeModelTranslator->MuJoCo_helper->master_reset_data);
    //Instantiate the visualiser
    activeVisualiser = std::make_shared<Visualiser>(activeModelTranslator);

    iLQROptimiser = std::make_shared<iLQR>(activeModelTranslator,
                                           activeModelTranslator->MuJoCo_helper,
                                           activeDifferentiator,
                                           opt_horizon, activeVisualiser, yamlReader);

    // Evaluate the parallelisation effectiveness of the dynamics derivatives computation
    iLQROptimiser->Resize(activeModelTranslator->current_state_vector.dof,
                          activeModelTranslator->current_state_vector.num_ctrl,
                          opt_horizon);

    std::vector<MatrixXd> A_matrices_SI1, B_matrices_SI1;
    std::vector<std::string> methods = {"SI2", "SI5", "SI20", "SI1000", "contact_change", "contact_change_dyn"};
    std::vector<std::string> keypoint_methods = {"set_interval", "set_interval", "set_interval", "set_interval", "contact_change", "contact_change_dyn"};
    std::vector<int> min_n_values = {2, 5, 20, 1000, 1, 1};
    // Create Vectors of MatrixXd to store A and B matrices for each method
    std::vector<std::vector<MatrixXd>> A_matrices(methods.size()), B_matrices(methods.size());

    // Size matrices appropriately
    for(int i = 0; i < methods.size(); i++){
        A_matrices[i].resize(opt_horizon, MatrixXd::Zero(activeModelTranslator->current_state_vector.dof, activeModelTranslator->current_state_vector.dof));
        B_matrices[i].resize(opt_horizon, MatrixXd::Zero(activeModelTranslator->current_state_vector.dof, activeModelTranslator->current_state_vector.num_ctrl));
    }

    std::vector<std::vector<double>> mean_squared_error,
                                    frobenius_errors,
                                    elementnorm_mse_errors,
                                    max_abs_error,
                                    max_rel_error,
                                    percentage_derivatives;

    mean_squared_error.resize(methods.size());
    frobenius_errors.resize(methods.size());
    elementnorm_mse_errors.resize(methods.size());
    max_abs_error.resize(methods.size());
    max_rel_error.resize(methods.size());

    percentage_derivatives.resize(methods.size());

    // Loop over 100 tasks
    int data_counter = 0;
    int task_counter = 0;
    int iteration_counter = 0;
    const int NUM_DATA_POINTS = num_data_points;
    const int MAX_ITERATIONS_PER_TASK = num_opt_iterations;
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

        }

        // Render
//        if(1){
//            for(int t = 0; t < opt_horizon; t++){
//                // Set the state
//                activeModelTranslator->MuJoCo_helper->CopySystemState(activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->MuJoCo_helper->saved_systems_state_list[t]);
////                activeModelTranslator->SetStateVectorQuat(iLQROptimiser->X_old[t], activeModelTranslator->MuJoCo_helper->vis_data, activeModelTranslator->full_state_vector);
////                std::cout << "t = " << t << ", state = " << iLQROptimiser->X_old[t].transpose() << "\n";
//
//                mj_forward(activeModelTranslator->MuJoCo_helper->model, activeModelTranslator->MuJoCo_helper->vis_data);
//
//                activeVisualiser->render("Visualise");
//            }
//        }

        // ----- Compute the accurate dynamics derivatives via SI1 method -----
        keypoint_method method;
        method = iLQROptimiser->ReturnCurrentKeypointMethod();
        method.min_N = 1;
        method.name = "set_interval";
        iLQROptimiser->SetCurrentKeypointMethod(method);
//        iLQROptimiser->keypoint_generator->PrintKeypointMethod();
        iLQROptimiser->GenerateDerivatives();

        A_matrices_SI1 = iLQROptimiser->A;
        B_matrices_SI1 = iLQROptimiser->B;

        for(int i = 0; i < methods.size(); i++){
            method.min_N = min_n_values[i];
            method.name = keypoint_methods[i];
            iLQROptimiser->SetCurrentKeypointMethod(method);
//            iLQROptimiser->keypoint_generator->PrintKeypointMethod();
            iLQROptimiser->GenerateDerivatives();

            A_matrices[i] = iLQROptimiser->A;
            B_matrices[i] = iLQROptimiser->B;

            double mse, frobenius_error, elementnorm_mse_error, max_abs_err, max_rel_err;
            ApproximationError(A_matrices_SI1, B_matrices_SI1, A_matrices[i], B_matrices[i],
                               mse, frobenius_error, elementnorm_mse_error, max_abs_err, max_rel_err);

            mean_squared_error[i].push_back(mse);
            frobenius_errors[i].push_back(frobenius_error);
            elementnorm_mse_errors[i].push_back(elementnorm_mse_error);
            max_abs_error[i].push_back(max_abs_err);
            max_rel_error[i].push_back(max_rel_err);

            double average_percent_derivs = 0.0;
            for(int j = 0; j < activeModelTranslator->current_state_vector.dof; j++){
                average_percent_derivs += iLQROptimiser->keypoint_generator->last_percentages[j];
            }
            average_percent_derivs /= activeModelTranslator->current_state_vector.dof;
            percentage_derivatives[i].push_back(average_percent_derivs);
        }

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
    std::vector<double> averaged_mse_error(methods.size(), 0.0);
    std::vector<double> averaged_frobenius_error(methods.size(), 0.0);
    std::vector<double> averaged_elementnorm_mse_error(methods.size(), 0.0);
    std::vector<double> average_max_abs_error(methods.size(), 0.0);
    std::vector<double> average_max_rel_error(methods.size(), 0.0);

    std::vector<double> average_percentage_derivatives(methods.size(), 0.0);

    for(int i = 0; i < methods.size(); i++){
        for(int j = 0; j < NUM_DATA_POINTS; j++){
            averaged_mse_error[i] += mean_squared_error[i][j];
            averaged_frobenius_error[i] += frobenius_errors[i][j];
            averaged_elementnorm_mse_error[i] += elementnorm_mse_errors[i][j];
            average_max_abs_error[i] += max_abs_error[i][j];
            average_max_rel_error[i] += max_rel_error[i][j];

            average_percentage_derivatives[i] += percentage_derivatives[i][j];
        }
        averaged_mse_error[i] /= NUM_DATA_POINTS;
        averaged_frobenius_error[i] /= NUM_DATA_POINTS;
        averaged_elementnorm_mse_error[i] /= NUM_DATA_POINTS;
        average_max_abs_error[i] /= NUM_DATA_POINTS;
        average_max_rel_error[i] /= NUM_DATA_POINTS;

        average_percentage_derivatives[i] /= NUM_DATA_POINTS;
    }

    const int w_method = 20;
    const int w_num    = 18;

    std::cout << std::left
              << std::setw(w_method) << "Method"
              << std::right
              << std::setw(w_num) << "MSE"
              << std::setw(w_num) << "Frob Err"
              << std::setw(w_num) << "Elem Norm MSE"
              << std::setw(w_num) << "Max Error (abs)"
              << std::setw(w_num) << "Max Error (rel)"
              << std::setw(w_num) << "% Derivatives"
              << "\n";

    for (int i = 0; i < methods.size(); i++) {
        std::cout << std::left << std::setw(w_method) << methods[i]
                  << std::right << std::setw(w_num) << std::fixed << std::setprecision(4) << averaged_mse_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(5) << averaged_frobenius_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(5) << averaged_elementnorm_mse_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(4) << average_max_abs_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(4) << average_max_rel_error[i]
                  << std::setw(w_num) << std::fixed << std::setprecision(2) << average_percentage_derivatives[i]
                  << "\n";
    }

    // --------------- Save the results to a file --------------------------
    // directory "DerivativeErrorData / {{task_name}_{opt_horizon}_{num_iterations}} / {method_name}.csv

    std::string project_parent_path = __FILE__;
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));
    project_parent_path = project_parent_path.substr(0, project_parent_path.find_last_of("/\\"));

    // Check folder exists, if it does not create one
    std::string folder_name = project_parent_path + "/DerivativeErrorData/" + task_name + "_" + std::to_string(opt_horizon) + "_" + std::to_string(num_opt_iterations) + "/";
    if(!std::filesystem::exists(folder_name)){
        std::filesystem::create_directories(folder_name);
    }

    for(int i = 0; i < methods.size(); i++){
        std::string file_path = folder_name + methods[i] + ".csv";
        std::ofstream file(file_path);
        if(file.is_open()){
            file << "MSE,Frobenius Error,Elementnorm Error,Max Error (abs),Max Error (rel),% Derivatives\n";
            for(int j = 0; j < NUM_DATA_POINTS; j++){
                file << mean_squared_error[i][j] << ","
                     << frobenius_errors[i][j] << ","
                     << elementnorm_mse_errors[i][j] << ","
                     << max_abs_error[i][j] << ","
                     << max_rel_error[i][j] << ","
                     << percentage_derivatives[i][j] << "\n";
            }
            file.close();
        }
        else{
            std::cerr << "Could not open file: " << file_path << "\n";
        }
    }

    return EXIT_SUCCESS;
}