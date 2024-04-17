#include <iomanip>
#include "iLQR.h"

iLQR::iLQR(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> MuJoCo_helper, std::shared_ptr<Differentiator> _differentiator, int horizon, std::shared_ptr<Visualiser> _visualizer, std::shared_ptr<FileHandler> _yamlReader) :
        Optimiser(_modelTranslator, MuJoCo_helper, _yamlReader, _differentiator){

    maxHorizon = horizon;
    active_visualiser = _visualizer;

    if(MuJoCo_helper->CheckIfDataIndexExists(0)){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[0], MuJoCo_helper->main_data);
    }
    else{
        MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
    }

    // initialise all vectors of matrices
    for(int i = 0; i < maxHorizon; i++){
        // Cost matrices
        l_x.emplace_back(MatrixXd(2*dof, 1));
        l_xx.emplace_back(MatrixXd(2*dof, 2*dof));
        l_u.emplace_back(MatrixXd(num_ctrl, 1));
        l_uu.emplace_back(MatrixXd(num_ctrl, num_ctrl));

        // Dynamics derivatives matrices
        // TODO - Move this to Optimiser constructor
        A.emplace_back(MatrixXd(2*dof, 2*dof));
        B.emplace_back(MatrixXd(2*dof, num_ctrl));

        A[i].block(0, 0, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof) *= MuJoCo_helper->ReturnModelTimeStep();
        B[i].setZero();

        K.emplace_back(MatrixXd(num_ctrl, 2*dof));
        k.emplace_back(MatrixXd(num_ctrl, 1));

        U_old.emplace_back(MatrixXd(num_ctrl, 1));
        U_new.emplace_back(MatrixXd(num_ctrl, 1));
        X_old.emplace_back(MatrixXd(2*dof, 1));
        X_new.emplace_back(MatrixXd(2*dof, 1));

        std::vector<MatrixXd> U_temp;
        for(int j = 0; j < 8; j++){
            U_temp.emplace_back(MatrixXd(num_ctrl, 1));
        }

        U_alpha.push_back(U_temp);

        if(MuJoCo_helper->CheckIfDataIndexExists(i + 1)){
            MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[i + 1], MuJoCo_helper->main_data);
        }
        else{
            MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
        }
    }

    // One more state than control
    X_old.push_back(MatrixXd(2*dof, 1));
    X_new.push_back(MatrixXd(2*dof, 1));
    l_x.push_back(MatrixXd(2*dof, 1));
    l_xx.push_back(MatrixXd(2*dof, 2*dof));


    // Whether to do some low pass filtering over A and B matrices
    filteringMethod = activeYamlReader->filtering;

}

double iLQR::RolloutTrajectory(mjData* d, bool save_states, std::vector<MatrixXd> initial_controls){
    double cost = 0.0f;
//    std::cout << "rollout trajectory" << std::endl;

//    if(d != MuJoCo_helper->main_data){
//        MuJoCo_helper->copySystemState(MuJoCo_helper->main_data, d);
//    }
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, d);

    MatrixXd Xt(activeModelTranslator->state_vector_size, 1);
    MatrixXd X_last(activeModelTranslator->state_vector_size, 1);
    MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
    MatrixXd U_last(activeModelTranslator->num_ctrl, 1);

    X_old[0] = activeModelTranslator->ReturnStateVector(MuJoCo_helper->main_data);

    if(MuJoCo_helper->CheckIfDataIndexExists(0)){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[0], MuJoCo_helper->main_data);
    }
    else{
        MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
    }

    for(int i = 0; i < horizonLength; i++){
        // set controls
        activeModelTranslator->SetControlVector(initial_controls[i], MuJoCo_helper->main_data);

        // Integrate simulator
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

        // return cost for this state
        Xt = activeModelTranslator->ReturnStateVector(MuJoCo_helper->main_data);
        Ut = activeModelTranslator->ReturnControlVector(MuJoCo_helper->main_data);
        double stateCost;
        
        if(i == horizonLength - 1){
            stateCost = activeModelTranslator->CostFunction(MuJoCo_helper->main_data, true);
        }
        else{
            stateCost = activeModelTranslator->CostFunction(MuJoCo_helper->main_data, false);
        }

        // If required to save states to trajectory tracking, then save state
        if(save_states){
            X_old[i + 1] = Xt.replicate(1, 1);
            U_old[i] = Ut.replicate(1, 1);
            if(MuJoCo_helper->CheckIfDataIndexExists(i + 1)){
                MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[i + 1], MuJoCo_helper->main_data);
            }
            else{
                MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
            }
        }

//        cost += (stateCost * active_physics_simulator->returnModelTimeStep());
        cost += stateCost;
    }

    initialCost = cost;
    costHistory.push_back(cost);

    return cost;
}

// ------------------------------------------------------------------------------------------------------
//
//  Optimise - Optimise a sequence of controls for a given problem
//  @Params:
//  d - The initial mujoco data to optimise from
//  initial_controls - The initial controls for the problem
//  maxIterations - The maximum iterations of the solver before it should return a new set of controls
//  horizonLength - How far into the future the Optimiser should look when optimising the controls
//
//  @Returns:
//  optimisedControls - New optimised controls that give a lower cost than the initial controls
//
// -------------------------------------------------------------------------------------------------------
std::vector<MatrixXd> iLQR::Optimise(mjData *d, std::vector<MatrixXd> initial_controls, int max_iterations, int min_iterations, int horizon_length){
    auto optStart = high_resolution_clock::now();
    
    // - Initialise variables
    std::vector<MatrixXd> optimisedControls(horizon_length);
    horizonLength = horizon_length;
    numberOfTotalDerivs = horizon_length * dof;

    // TODO - code to adjust max horizon if opt horizon > max_horizon
    std::cout << "horizon is " << horizon_length << "\n";

    if(keypoint_generator->horizon != horizonLength){
        std::cout << "horizon length changed" << std::endl;
        keypoint_generator->ResizeStateVector(dof, horizonLength);
    }

    bool costReducedLastIter = true;

    // ---------------------- Clear data saving variables ----------------------
    costHistory.clear();
    opt_time_ms = 0.0f;
    percentage_derivs_per_iteration.clear();
    timeDerivsPerIter.clear();
    avg_percent_derivs = 0;
    numIterationsForConvergence = 0;

    avg_time_get_derivs_ms = 0.0f;
    avg_time_forwards_pass_ms = 0.0f;
    avg_time_backwards_pass_ms = 0.0f;
    avg_surprise = 0.0f;
    avg_expected = 0.0f;

    percentage_derivs_per_iteration.clear();
    time_backwards_pass_ms.clear();
    time_forwardsPass_ms.clear();
    time_get_derivs_ms.clear();
    surprises.clear();
    expecteds.clear();
    // ------------------------------------------------------------------------

    auto time_start = high_resolution_clock::now();
    double oldCost = RolloutTrajectory(d, true, initial_controls);
    auto time_end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(time_end - time_start);
    if(verbose_output) {
        PrintBanner(duration.count() / 1000.0f);
    }
    initialCost = oldCost;
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    // Optimise for a set number of iterations
    for(int i = 0; i < max_iterations; i++){
        numIterationsForConvergence++;

        //STEP 1 - If forwards pass changed the trajectory, -
        auto derivsstart = high_resolution_clock::now();
        if(costReducedLastIter){
            GenerateDerivatives();
//            std::cout << "A[1]" << A[1] << "\n";
        }
        auto derivsstop = high_resolution_clock::now();
        auto linDuration = duration_cast<microseconds>(derivsstop - derivsstart);
        time_get_derivs_ms.push_back(linDuration.count() / 1000.0f);
        timeDerivsPerIter.push_back(linDuration.count() / 1000000.0f);

        // STEP 2 - BackwardsPass using the calculated derivatives to calculate an optimal feedback control law
        bool validBackwardsPass = false;
        bool lambdaExit = false;

        auto bp_start = high_resolution_clock::now();
        while(!validBackwardsPass){
            validBackwardsPass = BackwardsPassQuuRegularisation();

            if(!validBackwardsPass){
                if(lambda < max_lambda){
                    lambda *= lambda_factor;
                }
                else{
                    lambdaExit = true;
                    break;
                }
            }
            else{
                if(lambda > min_lambda){
                    lambda /= lambda_factor;
                }
            }
        }
        auto bp_stop = high_resolution_clock::now();
        auto bpDuration = duration_cast<microseconds>(bp_stop - bp_start);

        time_backwards_pass_ms.push_back(bpDuration.count() / 1000.0f);

        if(!lambdaExit){
            // STEP 3 - Forwards Pass - use the optimal control feedback law and rollout in simulation and calculate new cost of trajectory
            auto fp_start = high_resolution_clock::now();
            new_cost = ForwardsPass(oldCost);
            time_forwardsPass_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - fp_start).count() / 1000.0f);

            // Experimental
            auto time_start_k = high_resolution_clock::now();
//            std::vector<int> dofs_to_reduce = checkKMatrices();
//            std::cout << "time check k matrices: " << duration_cast<microseconds>(high_resolution_clock::now() - time_start_k).count() / 1000.0f << "ms" << std::endl;

            // Extra rollout with dimensionality
//            bool dimensionality_reduction_accepted = false;
//            if(dofs_to_reduce.size() > 0){
//                std::cout << "Reducing dimensionality of state vector" << std::endl;
//                dimensionality_reduction_accepted = RolloutWithKMatricesReduction(dofs_to_reduce, oldCost, newCost, last_alpha);
//            }

//            if(dimensionality_reduction_accepted){
//                // reduce the dimensionality of the state vector...
//                std::vector<std::string> state_vector_names = activeModelTranslator->GetStateVectorNames();
//                std::vector<std::string> dofs_to_reduce_str;
//                // pop the elements depending on dofs_to_reduce
//
//                // TESTING
////                dofs_to_reduce_str.push_back("blueTin_x");
////                dofs_to_reduce_str.push_back("blueTin_y");
//                for(int i = 0; i < dofs_to_reduce.size(); i++){
//                    dofs_to_reduce_str.push_back(state_vector_names[dofs_to_reduce[i]]);
//                }
//
//                // Temp print
//                if(verbose_output){
////                    std::cout << "Reducing dimensionality of state vector by removing: " << std::endl;
////                    for(int i = 0; i < dofs_to_reduce_str.size(); i++){
////                        std::cout << dofs_to_reduce_str[i] << std::endl;
////                    }
//                }
//                activeModelTranslator->UpdateStateVector(dofs_to_reduce_str, false);
//
//                ResizeStateVector(activeModelTranslator->dof);
//                for(int t = 0; t < horizonLength; t++){
//                    K[t].resize(num_ctrl, activeModelTranslator->dof * 2);
//                }
//                keypoint_generator->ResizeStateVector(activeModelTranslator->dof, horizon_length);
//            }

            // Update the X_old and U_old if cost was reduced
            if(new_cost < oldCost){
                for(int j = 0 ; j < horizonLength; j++){
                    X_old.at(j + 1) = activeModelTranslator->ReturnStateVector(MuJoCo_helper->saved_systems_state_list[j + 1]);
                    U_old[j] = U_new[j].replicate(1, 1);
                }
            }

            if(verbose_output){
                PrintBannerIteration(i, new_cost, oldCost,
                                     1 - (new_cost / oldCost), lambda, percentage_derivs_per_iteration[i],
                                     time_get_derivs_ms[i], time_backwards_pass_ms[i], time_forwardsPass_ms[i],
                                     last_iter_num_linesearches);
            }

            costHistory.push_back(new_cost);

            // Updates the keypoint parameters if auto_adjust is true.
            std::vector<double> dof_importances(activeModelTranslator->dof, 1.0);
            auto start_adjust = high_resolution_clock::now();
            keypoint_generator->AdjustKeyPointMethod(expected, oldCost - new_cost, X_old, dof_importances);
            auto stop_adjust = high_resolution_clock::now();
//            std::cout << "adjust took: " << duration_cast<microseconds>(stop_adjust - start_adjust).count() / 1000.0f << "ms" << std::endl;

            // STEP 4 - Check for convergence
            bool converged;
            converged = CheckForConvergence(oldCost, new_cost);

            if(new_cost < oldCost){
                oldCost = new_cost;
                costReducedLastIter = true;
            }
            else{
                costReducedLastIter = false;
                if(lambda < max_lambda){
                    lambda *= lambda_factor;
                    lambda *= lambda_factor;
                }
            }

            if(converged && (i >= min_iterations))
            {
                break;
            }
        }
        else{
            if(verbose_output){
                cout << "exiting optimisation due to lambda > lambdaMax \n";
            }
            break;
        }
    }

    // --------------------  Computing testing results ---------------------------

    costReduction = 1 - (new_cost / initialCost);
    auto optFinish = high_resolution_clock::now();
    auto optDuration = duration_cast<microseconds>(optFinish - optStart);
    opt_time_ms = optDuration.count() / 1000.0f;

    if(verbose_output){
        cout << setprecision(4);
        cout << " --------------------------------------------------- optimisation complete, took: " << opt_time_ms << " ms --------------------------------------------------" << endl;
    }

    // Time get derivs
    for(double time_get_derivs_m : time_get_derivs_ms){
        avg_time_get_derivs_ms += time_get_derivs_m;
    }

    // Percent derivs
    for(double i : percentage_derivs_per_iteration){
        avg_percent_derivs += i;
    }

    avg_time_get_derivs_ms /= static_cast<int>(time_get_derivs_ms.size());
    avg_percent_derivs /= static_cast<int>(percentage_derivs_per_iteration.size());

    // Time backwards pass
    for(double time_backwards_pass_m : time_backwards_pass_ms){
        avg_time_backwards_pass_ms += time_backwards_pass_m;
    }

    avg_time_backwards_pass_ms /= static_cast<int>(time_backwards_pass_ms.size());

    // Time forwards pass
    for(double time_forwardsPass_m : time_forwardsPass_ms){
        avg_time_forwards_pass_ms += time_forwardsPass_m;
    }

    if(!time_forwardsPass_ms.empty()){
        avg_time_forwards_pass_ms /= static_cast<int>(time_forwardsPass_ms.size());
    }

    // Surprise and expected
    for(int i = 0; i < surprises.size(); i++){
        avg_surprise += surprises[i];
        avg_expected += expecteds[i];
    }

    if(!surprises.empty()){
        avg_surprise /= static_cast<int>(surprises.size());
        avg_expected /= static_cast<int>(expecteds.size());
    }

    // Load the initial data back into main data
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    for(int i = 0; i < horizonLength; i++){
        optimisedControls[i] = U_old[i];
    }

    return optimisedControls;
}


// ------------------------------------------- STEP 2 FUNCTIONS (BACKWARDS PASS) ----------------------------------------------
bool iLQR::BackwardsPassQuuRegularisation(){
    MatrixXd V_x(2*dof, 2*dof);
    V_x = l_x[horizonLength - 1];
    MatrixXd V_xx(2*dof, 2*dof);
    V_xx = l_xx[horizonLength - 1];
    int Quu_pd_check_counter = 0;
    int number_steps_between_pd_checks = 100;

    MatrixXd Q_x(2*dof, 1);
    MatrixXd Q_u(num_ctrl, 1);
    MatrixXd Q_xx(2*dof, 2*dof);
    MatrixXd Q_uu(num_ctrl, num_ctrl);
    MatrixXd Q_ux(num_ctrl, 2*dof);

    // Reset delta J
    delta_J = 0.0f;

    // TODO check if this should start at -2 or -1 and end at 0 or 1?
    for(int t = horizonLength - 1; t >= 0; t--){

//        cout << "f_x[t] " << A[t] << endl;
//        cout << "f_u[t] " << B[t] << endl;

        Quu_pd_check_counter++;

        Q_x = l_x[t] + (A[t].transpose() * V_x);

        Q_u = l_u[t] + (B[t].transpose() * V_x);

        Q_xx = l_xx[t] + (A[t].transpose() * (V_xx * A[t]));

        Q_uu = l_uu[t] + (B[t].transpose() * (V_xx * B[t]));

        Q_ux = (B[t].transpose() * (V_xx * A[t]));

        MatrixXd Q_uu_reg = Q_uu.replicate(1, 1);

        for(int i = 0; i < Q_uu.rows(); i++){
            Q_uu_reg(i, i) += lambda;
        }

        if(Quu_pd_check_counter >= number_steps_between_pd_checks){
            if(!CheckMatrixPD(Q_uu_reg)){
                if(verbose_output){
                    cout << "non PD matrix encountered at t = " << t << endl;
                }
                return false;
            }
            Quu_pd_check_counter = 0;
        }

        auto temp = (Q_uu_reg).ldlt();
        MatrixXd I(num_ctrl, num_ctrl);
        I.setIdentity();
        MatrixXd Q_uu_inv = temp.solve(I);

        // control update law, open loop and feedback
        k[t] = -Q_uu_inv * Q_u;
        K[t] = -Q_uu_inv * Q_ux;

        V_x = Q_x + (K[t].transpose() * (Q_uu * k[t])) + (K[t].transpose() * Q_u) + (Q_ux.transpose() * k[t]);
        V_xx = Q_xx + (K[t].transpose() * (Q_uu * K[t])) + (K[t].transpose() * Q_ux) + (Q_ux.transpose() * K[t]);

        V_xx = (V_xx + V_xx.transpose()) / 2;

        delta_J += (k[t].transpose() * Q_u)(0);
        delta_J += (k[t].transpose() * Q_uu * k[t])(0);

        // if(t > horizonLength - 5){
        //     cout << "------------------ iteration " << t << " ------------------" << endl;
        //     cout << "l_x " << l_x[t] << endl;
        //     cout << "l_xx " << l_xx[t] << endl;
        //     cout << "l_u " << l_u[t] << endl;
        //     cout << "l_uu " << l_uu[t] << endl;
        //     cout << "Q_ux " << Q_ux << endl;
        //     cout << "f_u[t] " << f_u[t] << endl;
        //     cout << "Q_uu " << Q_uu << endl;
        //     cout << "Q_uu_inv " << Q_uu_inv << endl;
        //     cout << "Q_x " << Q_x << endl;
        //     cout << "Q_xx " << Q_xx << endl;
        //     cout << "V_xx " << V_xx << endl;
        //     cout << "V_x " << V_x << endl;
        //     cout << "K[t] " << K[t] << endl;
        // }
    }
    return true;
}

bool iLQR::CheckMatrixPD(Ref<MatrixXd> matrix){
    bool matrixPD = true;
    //TODO - implement cholesky decomp for PD check and maybe use result for inverse Q_uu

    Eigen::LLT<Eigen::MatrixXd> lltOfA(matrix); // compute the Cholesky decomposition of the matrix
    if(lltOfA.info() == Eigen::NumericalIssue)
    {
        matrixPD = false;
    }

    return matrixPD;
}

// ------------------------------------------- STEP 3 FUNCTIONS (FORWARDS PASS) ----------------------------------------------
double iLQR::ForwardsPass(double old_cost){
    double newCost = 0;
    bool costReduction = false;
    int alphaCount = 0;

    MatrixXd Xt(2 * dof, 1);
    MatrixXd Ut(num_ctrl, 1);

    std::vector<double> alphas = {1.0, 0.8, 0.5, 0.3, 0.1};

    while(!costReduction){

        // Copy initial data state into main data state for rollout
        MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

        newCost = 0;
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);

        for(int t = 0; t < horizonLength; t++) {
            // Step 1 - get old state and old control that were linearised around
            _X = X_old[t].replicate(1, 1);
            _U = U_old[t].replicate(1, 1);

            X_new = activeModelTranslator->ReturnStateVector(MuJoCo_helper->main_data);
            // Calculate difference from new state to old state
            stateFeedback = X_new - _X;

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
            U_new[t] = _U + (alphas[alphaCount] * k[t]) + feedBackGain;

            // Clamp torque within limits
            if(activeModelTranslator->active_state_vector.robots[0].torqueControlled){
                for(int i = 0; i < num_ctrl; i++){
                    if (U_new[t](i) > activeModelTranslator->active_state_vector.robots[0].torqueLimits[i]) U_new[t](i) = activeModelTranslator->active_state_vector.robots[0].torqueLimits[i];
                    if (U_new[t](i) < -activeModelTranslator->active_state_vector.robots[0].torqueLimits[i]) U_new[t](i) = -activeModelTranslator->active_state_vector.robots[0].torqueLimits[i];
                }
            }

            activeModelTranslator->SetControlVector(U_new[t], MuJoCo_helper->main_data);

            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->CostFunction(MuJoCo_helper->main_data, true);
            }
            else{
                newStateCost = activeModelTranslator->CostFunction(MuJoCo_helper->main_data, false);
            }

//            newCost += (newStateCost * active_physics_simulator->returnModelTimeStep());
            newCost += newStateCost;

            mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

            // Copy system state to fp_rollout_buffer to prevent a second rollout of computations using simulation integration
            MuJoCo_helper->SaveDataToRolloutBuffer(MuJoCo_helper->main_data, t + 1);

//             if(t % 5 == 0){
//                 const char* fplabel = "fp";
//                 active_visualiser->render(fplabel);
//             }

        }

        if(newCost < old_cost){
            costReduction = true;
        }
        else{
            if(alphaCount >= alphas.size() - 1){
                break;
            }
            alphaCount++;
        }
    }

    last_iter_num_linesearches = alphaCount + 1;
    last_alpha = alphas[alphaCount];

    // Compute expected costreduction
    expected = -(last_alpha * delta_J + (pow(last_alpha, 2) / 2) * delta_J);
    expecteds.push_back(expected);

    // If the cost was reduced
    if(newCost < old_cost){
        // Compute surprise

        surprise = (old_cost - newCost) / expected;
        surprises.push_back(surprise);

//        std::cout << "expected :" << expected << " actual: " << old_cost - newCost << std::endl;

        // Reset the system state to the initial state
        MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

        //Copy the rollout buffer to saved systems state list, prevents recomputation using optimal controls
        MuJoCo_helper->CopyRolloutBufferToSavedSystemStatesList();

        return newCost;
    }

    surprise = 0.0;
    surprises.push_back(0.0);

    return old_cost;
}

double iLQR::ForwardsPassParallel(double old_cost){
    auto start = std::chrono::high_resolution_clock::now();
    double newCost = 0.0;
    bool costReduction = false;

//    double alphas[8] = {0.125, 0.25, 0.375, 0.5, 0.675, 0.75, 0.875, 1.0};
//    double alphas[8] = {1.0, 0.875, 0.75, 0.675, 0.5, 0.375, 0.25, 0.125};

    std::vector<double> alphas = {1.0, 0.75, 0.5, 0.1};
    std::vector<double> newCosts;
    newCosts.resize(alphas.size());
//    double newCosts[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for(int i = 0; i < alphas.size(); i++){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[i + 1], MuJoCo_helper->saved_systems_state_list[0]);
    }

    MatrixXd initState = activeModelTranslator->ReturnStateVector(MuJoCo_helper->saved_systems_state_list[0]);
    auto end = std::chrono::high_resolution_clock::now();
    auto copy_duration = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
    cout << "copy duration: " << copy_duration.count() / 1000.0f << endl;

    start = std::chrono::high_resolution_clock::now();

    #pragma omp parallel for
    for(int i = 0; i < alphas.size(); i++){
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);
        MatrixXd Xt(2 * dof, 1);

        for(int t = 0; t < horizonLength; t++) {
            // Step 1 - get old state and old control that were linearised around
//            _X = X_old[t].replicate(1, 1);
            //_U = activeModelTranslator->ReturnControlVector(t);
//            _U = U_old[t].replicate(1, 1);

            X_new = activeModelTranslator->ReturnStateVector(MuJoCo_helper->saved_systems_state_list[i + 1]);

            // Calculate difference from new state to old state
//            stateFeedback = X_new - _X;
            stateFeedback = X_new - X_old[t];

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
//            U_alpha[t][i] = _U + (alphas[i] * k[t]) + feedBackGain;
            U_alpha[t][i] = U_old[t] + (alphas[i] * k[t]) + feedBackGain;

            // Clamp torque within limits
            if(activeModelTranslator->active_state_vector.robots[0].torqueControlled){
                for(int k = 0; k < num_ctrl; k++){
                    if (U_alpha[t][i](k) > activeModelTranslator->active_state_vector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = activeModelTranslator->active_state_vector.robots[0].torqueLimits[k];
                    if (U_alpha[t][i](k) < -activeModelTranslator->active_state_vector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = -activeModelTranslator->active_state_vector.robots[0].torqueLimits[k];
                }
            }

            activeModelTranslator->SetControlVector(U_alpha[t][i], MuJoCo_helper->saved_systems_state_list[i + 1]);
//            Xt = activeModelTranslator->ReturnStateVector(i+1);
//            //cout << "Xt: " << Xt << endl;
//
//
            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->CostFunction(MuJoCo_helper->saved_systems_state_list[i + 1], true);
            }
            else{
                newStateCost = activeModelTranslator->CostFunction(MuJoCo_helper->saved_systems_state_list[i + 1], false);
            }

            newCosts[i] += (newStateCost * MuJoCo_helper->ReturnModelTimeStep());

            mj_step(MuJoCo_helper->model, MuJoCo_helper->saved_systems_state_list[i + 1]);

        }
    }

    double bestAlphaCost = newCosts[0];
    int bestAlphaIndex = 0;
    for(int i = 0; i < alphas.size(); i++){
        if(newCosts[i] < bestAlphaCost){
            bestAlphaCost = newCosts[i];
            bestAlphaIndex = i;
        }
    }

    end = std::chrono::high_resolution_clock::now();
    auto rollout_duration = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
//    cout << "rollouts duration: " << rollout_duration.count() / 1000.0f << endl;

    newCost = bestAlphaCost;
//    cout << "best alpha cost = " << bestAlphaCost << " at alpha: " << alphas[bestAlphaIndex] << endl;
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    // If the cost was reduced - update all the data states
    if(newCost < old_cost){
        for(int i = 0; i < horizonLength; i++){

            activeModelTranslator->SetControlVector(U_alpha[i][bestAlphaIndex], MuJoCo_helper->main_data);
            mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

            // Log the old state
            X_old.at(i + 1) = activeModelTranslator->ReturnStateVector(MuJoCo_helper->main_data);

            MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[i + 1], MuJoCo_helper->main_data);

            U_old[i] = U_alpha[i][bestAlphaIndex].replicate(1, 1);

        }

        MatrixXd testState = activeModelTranslator->ReturnStateVector(MuJoCo_helper->saved_systems_state_list[horizonLength - 1]);
//        cout << "final state after FP: " << testState.transpose() << endl;

        return newCost;
    }

    return old_cost;
}

bool iLQR::RolloutWithKMatricesReduction(std::vector<int> dof_indices, double old_cost, double new_cost, double alpha){

    // Copy initial state into main data
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);
    double reduced_cost = 0.0f;

    MatrixXd stateFeedback(2*dof, 1);
    MatrixXd _X(2*dof, 1);
    MatrixXd X_new(2*dof, 1);
    MatrixXd _U(num_ctrl, 1);

    for(int t = 0; t < horizonLength; t++) {
        for( int dof_index : dof_indices) {
            K[t].block(0, dof_index, num_ctrl, 1) = MatrixXd::Zero(num_ctrl, 1);
            K[t].block(0, dof_index + dof, num_ctrl, 1) = MatrixXd::Zero(num_ctrl, 1);
        }
    }

    for(int t = 0; t < horizonLength; t++) {
        // Step 1 - get old state and old control that were linearised around
        _X = X_old[t].replicate(1, 1);
        _U = U_old[t].replicate(1, 1);

        X_new = activeModelTranslator->ReturnStateVector(MuJoCo_helper->main_data);
        // Calculate difference from new state to old state
        stateFeedback = X_new - _X;

        MatrixXd feedBackGain = K[t] * stateFeedback;
//            std::cout << "K[t] " << K[t] << std::endl;

        // Calculate new optimal controls
        U_new[t] = _U + (alpha * k[t]) + feedBackGain;

        // Clamp torque within limits
        if(activeModelTranslator->active_state_vector.robots[0].torqueControlled){
            for(int i = 0; i < num_ctrl; i++){
                if (U_new[t](i) > activeModelTranslator->active_state_vector.robots[0].torqueLimits[i]) U_new[t](i) = activeModelTranslator->active_state_vector.robots[0].torqueLimits[i];
                if (U_new[t](i) < -activeModelTranslator->active_state_vector.robots[0].torqueLimits[i]) U_new[t](i) = -activeModelTranslator->active_state_vector.robots[0].torqueLimits[i];
            }
        }

        activeModelTranslator->SetControlVector(U_new[t], MuJoCo_helper->main_data);

        double newStateCost;
        // Terminal state
        if(t == horizonLength - 1){
            newStateCost = activeModelTranslator->CostFunction(MuJoCo_helper->main_data, true);
        }
        else{
            newStateCost = activeModelTranslator->CostFunction(MuJoCo_helper->main_data, false);
        }

        reduced_cost += newStateCost;

        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

        // Copy system state to fp_rollout_buffer to prevent a second rollout of computations using simulation integration
//        MuJoCo_helper->saveDataToRolloutBuffer(MuJoCo_helper->main_data, t + 1);

    }

    double eps_before = 1.0f - (new_cost / old_cost);
    double eps_reduced = 1.0f - (reduced_cost / old_cost);

    std::cout << "eps_before: " << eps_before << " eps_reduced: " << eps_reduced << std::endl;
    std::cout << "reduced_cost: " << reduced_cost << " old_new_cost: " << new_cost << std::endl;

    if(eps_before - eps_reduced < eps_acceptable_diff){
        return true;
    }

    return false;
}

std::vector<int> iLQR::checkKMatrices(){

    double *K_dofs_sums = new double[dof];
    std::vector<int> dofs_to_remove;
    for(int i = 0; i < dof; i++){
        K_dofs_sums[i] = 0;
    }
//
//    for(int t = 0; t < horizonLength; t += sampling_k_interval){
//
//        for(int i = 0; i < dof; i++){
//
//            for(int j = 0; j < num_ctrl; j++){
//                K_dofs_sums[i] += abs(K[t](j, i));
//                K_dofs_sums[i] += abs(K[t](j, i + dof));
//            }
//        }
//    }
//

    for(int t = 0; t < horizonLength; t += sampling_k_interval){
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(K[t], Eigen::ComputeThinV);
        if (!svd.computeV()) {
            std::cerr << "SVD decomposition failed!" << std::endl;
            return dofs_to_remove;
        }

//        std::cout << "The singular values of K are:\n" << svd.singularValues() << std::endl;
//        std::cout << "The right singular vectors of K are:\n" << svd.matrixV() << std::endl;

        for(int i = 0; i < num_ctrl; i++){
            for(int j = 0; j < dof; j++){
                K_dofs_sums[j] += abs(svd.matrixV()(j, i));
                K_dofs_sums[j] += abs(svd.matrixV()(j + dof, i));
            }
        }
    }

//    std::cout << "K matrices importance weightings: \n";
    for (int i = 0; i < dof; i++){
//        std::cout << "DOF " << i << " : " << K_dofs_sums[i] << "\n";

        if(K_dofs_sums[i] < threshold_k_eignenvectors) {
            dofs_to_remove.push_back(i);
        }
    }

    return dofs_to_remove;
}

void iLQR::PrintBanner(double time_rollout){
    std::cout << "--------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "|                                                   iLQR begins, initial rollout took: " << std::setprecision(4) << time_rollout << "                                               |" << std::endl;

    std::cout << std::left << std::setw(12) << "| Iteration"
              << std::setw(12) << "| Old Cost"
              << std::setw(12) << "| New Cost"
              << std::setw(8)  << "| Eps"
              << std::setw(10) << "| Lambda"
              << std::setw(16) << "| % Derivatives"
              << std::setw(20) << "| Time Derivs (ms)"
              << std::setw(15) << "| Time BP (ms)"
              << std::setw(15) << "| Time FP (ms)"
              << std::setw(18) << "| Num Linesearches" << " |" << std::endl;
}

void iLQR::PrintBannerIteration(int iteration, double new_cost, double old_cost, double eps,
                                double lambda, double percent_derivatives, double time_derivs, double time_bp,
                                double time_fp, int num_linesearches){

    std::cout << std::left << "|" << std::setw(11) << iteration
              << "|" << std::setw(11) << old_cost
              << "|" << std::setw(11) << new_cost
              << "|" << std::setprecision(3) << std::setw(7)  << eps
              << "|" << std::setw(9) << lambda
              << "|" << std::setw(15) << percent_derivatives
              << "|" << std::setw(19) <<time_derivs
              << "|" << std::setw(14)  << time_bp
              << "|" << std::setw(14) << time_fp
              << "|" << std::setw(18) << num_linesearches << "|" << std::endl;

//    std::cout << std::setprecision(4);
//    std::cout << "|     " << iteration << "     |   " << std::setw(5) << old_cost << "    |    " << std::setw(5) << new_cost << "   |  ";
//    std::cout << std::setprecision(2) << eps << " |   ";
//    std::cout << std::setprecision(4) <<  lambda << "   |       " << std::setw(3) << percent_derivatives << "       |       ";
//    std::cout << std::setprecision(3) << time_derivs << "       |     " << time_bp << "     |     " << time_fp << "     |         "
//    << num_linesearches << "        |" << std::endl;

}

