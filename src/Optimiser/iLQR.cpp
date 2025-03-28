#include <iomanip>
#include "Optimiser/iLQR.h"

iLQR::iLQR(std::shared_ptr<ModelTranslator> _modelTranslator,
           std::shared_ptr<MuJoCoHelper> MuJoCo_helper,
           std::shared_ptr<Differentiator> _differentiator,
           int horizon,
           std::shared_ptr<Visualiser> _visualizer,
           std::shared_ptr<FileHandler> _yamlReader) :
        Optimiser(_modelTranslator, MuJoCo_helper, _yamlReader, _differentiator){

    active_visualiser = _visualizer;

    // Initialise saved systems state list
    if(MuJoCo_helper->CheckIfDataIndexExists(0)){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[0], MuJoCo_helper->main_data);
    }
    else{
        MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
    }

    // Initialise all vectors of matrices
    for(int i = 0; i < horizon; i++){

        if(MuJoCo_helper->CheckIfDataIndexExists(i + 1)){
            MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[i + 1], MuJoCo_helper->main_data);
        }
        else{
            MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
        }
    }

    rollout_data.resize(num_parallel_rollouts);

    // Whether to do some low pass filtering over A and B matrices
    filteringMethod = activeYamlReader->filtering;

    // Resize internal data variables
    Resize(activeModelTranslator->current_state_vector.dof,
           activeModelTranslator->current_state_vector.num_ctrl,
           horizon);

}

void iLQR::Resize(int new_num_dofs, int new_num_ctrl, int new_horizon){
    auto start = std::chrono::high_resolution_clock::now();
//    std::cout << "new dofs: " << new_num_dofs << " new ctrl: " << new_num_ctrl << " new horizon: " << new_horizon << std::endl;

    bool update_ctrl = false;
    bool update_dof = false;
    bool update_horizon = false;
    if(new_num_ctrl != this->num_ctrl){
        this->num_ctrl = new_num_ctrl;
        update_ctrl = true;
    }

    if(new_num_dofs != this->dof){
        this->dof = new_num_dofs;
        update_dof = true;
    }

    if(new_horizon != this->horizon_length){
        this->horizon_length = new_horizon;
        update_horizon = true;
    }

    // Clear old matrices
    if(update_ctrl){
        // Cost derivatives with respect to control
        l_u.clear();
        l_uu.clear();

        // Residual derivatives with respect to control
        r_u.clear();

        // Old control trajectory
        U_old.clear();

        // Open-loop feedback control law
        k.clear();
    }

    if(update_dof){
        // Cost derivatives with respect to state
        l_x.clear();
        l_xx.clear();

        // Residual derivatives with respect to state
        r_x.clear();

        // Dynamics derivatives with respect to state
        A.clear();

        // New and old state trajectories
        X_old.clear();
        X_new.clear();
    }

    if(update_horizon){
        residuals.clear();
    }

    // dependant on both dofs and num_ctrl
    B.clear();
    K.clear();

    int num_dof = activeModelTranslator->current_state_vector.dof;
    int num_dof_quat = activeModelTranslator->current_state_vector.dof_quat;

    for(int t = 0; t < this->horizon_length; t++){

        if(update_dof){
            l_x.emplace_back(MatrixXd(2*dof, 1));
            l_xx.emplace_back(MatrixXd(2*dof, 2*dof));

            A.emplace_back(MatrixXd(2*dof, 2*dof));

            X_old.emplace_back(MatrixXd(num_dof_quat + num_dof, 1));
            X_new.emplace_back(MatrixXd(num_dof_quat + num_dof, 1));

            vector<MatrixXd> r_x_;
            for(int i = 0; i < activeModelTranslator->residual_list.size(); i++) {
                r_x_.emplace_back(MatrixXd(2*dof, 1));
            }

            r_x.emplace_back(r_x_);
        }

        if(update_ctrl){
            l_u.emplace_back(MatrixXd(num_ctrl, 1));
            l_uu.emplace_back(MatrixXd(num_ctrl, num_ctrl));

            k.emplace_back(MatrixXd(num_ctrl, 1));

            U_old.emplace_back(MatrixXd(num_ctrl, 1));

            vector<MatrixXd> r_u_;
            for(int i = 0; i < activeModelTranslator->residual_list.size(); i++) {
                r_u_.emplace_back(MatrixXd(num_ctrl, 1));
            }

            r_u.emplace_back(r_u_);
        }

        B.emplace_back(MatrixXd(2*dof, num_ctrl));
        K.emplace_back(MatrixXd(num_ctrl, 2*dof));
    }

    // One more state than control
    if(update_dof){
        l_x.push_back(MatrixXd(2*dof, 1));
        l_xx.push_back(MatrixXd(2*dof, 2*dof));

        X_old.push_back(MatrixXd(num_dof_quat + num_dof, 1));
        X_new.push_back(MatrixXd(num_dof_quat + num_dof, 1));

        vector<MatrixXd> r_x_;
        vector<MatrixXd> r_u_;
        for(int i = 0; i < activeModelTranslator->residual_list.size(); i++) {
            r_x_.emplace_back(MatrixXd(2*dof, 1));
            r_u_.emplace_back(MatrixXd(num_ctrl, 1));

        }

        r_x.emplace_back(r_x_);
        r_u.emplace_back(r_u_);
    }

    if(update_horizon){
        // Clear old rollout datas
        for(int i = 0; i < num_parallel_rollouts; i++){
            rollout_data[i].clear();
        }

        std::vector<mujoco_data_min> data_horizon(horizon_length+1);

        mujoco_data_min data_timestep;
        data_timestep.time = 0.0;
        data_timestep.q_pos.resize(MuJoCo_helper->model->nq);
        data_timestep.q_vel.resize(MuJoCo_helper->model->nv);
        data_timestep.q_acc.resize(MuJoCo_helper->model->nv);
        data_timestep.q_acc_warmstart.resize(MuJoCo_helper->model->nv);
        data_timestep.qfrc_applied.resize(MuJoCo_helper->model->nv);
        data_timestep.xfrc_applied.resize(6*MuJoCo_helper->model->nbody);
        data_timestep.ctrl.resize(MuJoCo_helper->model->nu);

        for(int t = 0; t < horizon_length+1; t++){
            data_horizon[t] = data_timestep;
        }

        for(int i = 0; i < num_parallel_rollouts; i++){
            rollout_data[i].resize(horizon_length+1);
            rollout_data[i] = data_horizon;
        }

        for(int t = 0; t < horizon_length+1; t++){
            residuals.push_back(MatrixXd(activeModelTranslator->residual_list.size(), 1));
        }

    }

    // Resize Keypoint generator class
    keypoint_generator->Resize(dof, num_ctrl, horizon_length);

    std::cout << "iLQR time to allocate memory: " << duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0 << " ms \n";

//    std::cout << "length of A: " << A.size() << ", size of A is: " << A[0].cols() << "\n";
}

double iLQR::RolloutTrajectory(mjData* d, bool save_states, std::vector<MatrixXd> initial_controls){
    double cost = 0.0;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, d);

    X_old[0] = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);

    if(MuJoCo_helper->CheckIfDataIndexExists(0)){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[0], MuJoCo_helper->main_data);
    }
    else{
        MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
    }

    for(int i = 0; i < horizon_length; i++){

        // set controls
        activeModelTranslator->SetControlVector(initial_controls[i],
                                                          MuJoCo_helper->main_data,
                                                             activeModelTranslator->full_state_vector);

        // Integrate simulator
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

        // Return cost for this state
        double state_cost;
        activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[i]);
        if(i == horizon_length - 1){
            state_cost = activeModelTranslator->CostFunction(residuals[i], activeModelTranslator->full_state_vector, true);
        }
        else{
            state_cost = activeModelTranslator->CostFunction(residuals[i], activeModelTranslator->full_state_vector, false);
        }

        // If required to save states to trajectory tracking, then save state
        if(save_states){
            X_old[i + 1] = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);
            U_old[i] = activeModelTranslator->ReturnControlVector(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);
            if(MuJoCo_helper->CheckIfDataIndexExists(i + 1)){
                MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[i + 1], MuJoCo_helper->main_data);
            }
            else{
                MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
            }
        }

//        cost += (stateCost * active_physics_simulator->returnModelTimeStep());
        cost += state_cost;
    }

    cost_history.push_back(cost);

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

    // resize internal matrices if required
    Resize(activeModelTranslator->current_state_vector.dof,
           activeModelTranslator->current_state_vector.num_ctrl,
           horizon_length);


    // - Initialise variables
    std::vector<MatrixXd> optimisedControls(horizon_length);

    // TODO - code to adjust max horizon if opt horizon > max_horizon
//    std::cout << "horizon is " << horizon_length << "\n";

    if(keypoint_generator->horizon != horizon_length){
        std::cout << "horizon length changed" << std::endl;
        keypoint_generator->Resize(dof, num_ctrl, horizon_length);
    }

    // ---------------------- Clear data saving variables ----------------------
    cost_history.clear();

    opt_time_ms = 0.0;
    avg_time_get_derivs_ms = 0.0;
    avg_time_forwards_pass_ms = 0.0;
    avg_time_backwards_pass_ms = 0.0;
    avg_surprise = 0.0;
    avg_expected = 0.0;
    avg_percent_derivs = 0;
    num_iterations = 0;
    avg_dofs = 0.0;

    percentage_derivs_per_iteration.clear();
    num_dofs.clear();
    cost_history.clear();
    time_backwards_pass_ms.clear();
    time_forwardsPass_ms.clear();
    time_get_derivs_ms.clear();
    surprises.clear();
    expecteds.clear();
    // ------------------------------------------------------------------------

    auto time_start = high_resolution_clock::now();
    old_cost = RolloutTrajectory(d, true, initial_controls);
    std::cout << "cost from rollout: " << old_cost << "\n";
    auto time_end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(time_end - time_start);
    if(verbose_output) {
        PrintBanner(duration.count() / 1000.0f);
    }
    initial_cost = old_cost;
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    // ------------------- Main optimisation iteration loop ------------------------
    cost_reduced_last_iter = true;
    for(int i = 0; i < max_iterations; i++) {
        num_iterations++;

        bool lambda_exit, converged = false;
        Iteration(i, converged, lambda_exit);

        if (converged && (i >= min_iterations)) {
            break;
        }

        if (lambda_exit) {
            break;
        }
    }

    // --------------------  Computing testing results ---------------------------

    cost_reduction = 1 - (new_cost / initial_cost);
    auto optFinish = high_resolution_clock::now();
    auto optDuration = duration_cast<microseconds>(optFinish - optStart);
    opt_time_ms = optDuration.count() / 1000.0f;

    if(verbose_output){
        cout << setprecision(4);
        cout << " --------------------------------------------------- optimisation complete, took: " << opt_time_ms << " ms --------------------------------------------------" << endl;
    }

    // Average number of dofs
    for(int _num_dofs : num_dofs){
        avg_dofs += _num_dofs;
    }
    avg_dofs /= static_cast<int>(num_dofs.size());

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

    for(int i = 0; i < horizon_length; i++){
        optimisedControls[i] = U_old[i];
    }

    std::cout << "optimised control[0] " << optimisedControls[0].transpose() << "\n";

    return optimisedControls;
}

void iLQR::Iteration(int iteration_num, bool &converged, bool &lambda_exit){

    // This should always remain the same in baseline iLQR
    num_dofs.push_back(activeModelTranslator->current_state_vector.dof);

    // STEP 1 - Generate dynamics derivatives and cost derivatives
    auto timer_start = high_resolution_clock::now();
    if(cost_reduced_last_iter){
        GenerateDerivatives();
//        std::cout << "A[0] \n" << A[0] << "\n";
//        std::cout << "B[0] \n" << B[0] << "\n";
//        std::cout << "l_x[0] \n" << l_x[0] << "\n";
//        std::cout << "l_xx[0] \n" << l_xx[0] << "\n";
//        std::cout << "l_x[1] \n" << l_x[1] << "\n";
//        std::cout << "l_xx[1] \n" << l_xx[1] << "\n";
//        std::cout << "l_x[horizon - 1] \n" << l_x[horizon_length - 1] << "\n";
    }
    time_get_derivs_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);

    // STEP 2 - BackwardsPass using the calculated derivatives to calculate an optimal feedback control law
    bool valid_backwards_pass = false;

    timer_start = high_resolution_clock::now();
    while(!valid_backwards_pass){
        valid_backwards_pass = BackwardsPassQuuRegularisation();
        lambda_exit = UpdateLambda(valid_backwards_pass);

        if(lambda_exit){
            break;
        }
    }

    time_backwards_pass_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);

    // If we were unable to compute a new valid control law, exit
    if(lambda_exit){
        if(verbose_output){
            cout << "exiting optimisation due to lambda > lambdaMax \n";
        }
        return;
    }

    // STEP 3 - Forwards Pass - use the optimal control feedback law and rollout in simulation and calculate new cost of trajectory
    timer_start = high_resolution_clock::now();
    double best_alpha;
    if(0){
        new_cost = ForwardsPass(old_cost);
        best_alpha = last_iter_num_linesearches;
    }
    else{
//        auto temp_timer = high_resolution_clock::now();
        std::vector<std::future<double>> futures;
        std::vector<double> alphas;

        for(int i = 1; i < num_parallel_rollouts + 1; i++){
            double linearValue = static_cast<double>(i) / (num_parallel_rollouts); // Evenly spaced values between 0 and 1
            double compressed_vals = linearValue * linearValue;
            alphas.push_back(compressed_vals);
        }

//        for(int i = 0; i < num_parallel_rollouts; i++){
//            alphas.push_back(1.0 - (double)i/num_parallel_rollouts);
//        }
//        std::cout << "setup threads: " << duration_cast<microseconds>(high_resolution_clock::now() - temp_timer).count() / 1000.0f << "ms \n";

        // Create tasks and push them into the vector
        for (int i = 0; i < num_parallel_rollouts; ++i) {
            futures.push_back(std::async(std::launch::async, [this, i, &alphas]() {
                return this->ForwardsPassParallel(i, alphas[i]);
            }));
        }

        std::vector<double> results;
        for (auto& future : futures) {
            results.push_back(future.get()); // get the result from future
        }

        // Compute best alpha
        auto min_index = std::min_element(results.begin(), results.end()) - results.begin();
        best_alpha = alphas[min_index];

        // Check if best cost from rollouts < old_cost
        if(results[min_index] < old_cost){
            new_cost = results[min_index];

            // Update saved systems state list
            SaveBestRollout(static_cast<int>(min_index));
        }
        else{
            new_cost = old_cost;
        }
    }

    time_forwardsPass_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);

    if(verbose_output){
        PrintBannerIteration(iteration_num, new_cost, old_cost,
                             1 - (new_cost / old_cost), lambda, percentage_derivs_per_iteration[iteration_num],
                             time_get_derivs_ms[iteration_num], time_backwards_pass_ms[iteration_num], time_forwardsPass_ms[iteration_num],
                             best_alpha);
    }

    // STEP 4 - Check for convergence
    converged = CheckForConvergence(old_cost, new_cost);

    if(new_cost < old_cost){
        UpdateNominal();
        cost_reduced_last_iter = true;
    }
    else{
        cost_reduced_last_iter = false;

        // Undo lambda reduction after backwards pass - because cost didnt reduce as expected
        lambda *= lambda_factor;
        lambda *= lambda_factor;
        if(lambda > max_lambda) lambda = max_lambda;
    }

    cost_history.push_back(new_cost);
}


// ------------------------------------------- STEP 2 FUNCTIONS (BACKWARDS PASS) ----------------------------------------------
bool iLQR::BackwardsPassQuuRegularisation(){
    MatrixXd V_x(2*dof, 2*dof);
    V_x = l_x[horizon_length - 1];
    MatrixXd V_xx(2*dof, 2*dof);
    V_xx = l_xx[horizon_length - 1];
    int Quu_pd_check_counter = 0;
    int number_steps_between_pd_checks = 100;

    MatrixXd Q_x(2*dof, 1);
    MatrixXd Q_u(num_ctrl, 1);
    MatrixXd Q_xx(2*dof, 2*dof);
    MatrixXd Q_uu(num_ctrl, num_ctrl);
    MatrixXd Q_ux(num_ctrl, 2*dof);

    MatrixXd Q_uu_reg(num_ctrl, num_ctrl);

    MatrixXd A_t(2*dof, 2*dof);
    MatrixXd B_t(2*dof, num_ctrl);

    // Reset delta J
    delta_J = 0.0f;
    double temp_time = 0.0;
    auto time_start = high_resolution_clock::now();

    // TODO check if this should start at -2 or -1 and end at 0 or 1?
    for(int t = horizon_length - 1; t >= 0; t--){

//        cout << "f_x[t] " << A[t] << endl;
//        cout << "f_u[t] " << B[t] << endl;

        Quu_pd_check_counter++;

        A_t = A[t].transpose();
        B_t = B[t].transpose();

        Q_x = l_x[t] + (A_t * V_x);

        Q_u = l_u[t] + (B_t * V_x);

        // This is the line that takes the most time in the backwards pass.
        Q_xx = l_xx[t] + (A_t * V_xx * A[t]);

        Q_uu = l_uu[t] + (B_t * V_xx * B[t]);

        Q_ux = (B_t * V_xx * A[t]);

        Q_uu_reg = Q_uu.replicate(1, 1);

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

        // Ensure V_xx remains symmetrical
        V_xx = (V_xx + V_xx.transpose()) / 2;

        delta_J += (k[t].transpose() * Q_u)(0);
        delta_J += (k[t].transpose() * Q_uu * k[t])(0);

//         cout << "------------------ iteration " << t << " ------------------" << endl;
//         cout << "l_x " << l_x[t] << endl;
//         cout << "l_xx " << l_xx[t] << endl;
//         cout << "l_u " << l_u[t] << endl;
//         cout << "l_uu " << l_uu[t] << endl;
//         cout << "Q_ux " << Q_ux << endl;
//         cout << "f_u[t] " << f_u[t] << endl;
//         cout << "Q_uu " << Q_uu << endl;
//         cout << "Q_uu_inv " << Q_uu_inv << endl;
//         cout << "Q_x " << Q_x << endl;
//         cout << "Q_xx " << Q_xx << endl;
//         cout << "V_xx " << V_xx << endl;
//         cout << "V_x " << V_x << endl;
//         cout << "K[t] " << K[t] << endl;

//        cout << "----- K[t] ----- \n " << K[t] << endl;
    }
//    std::cout << "time spent on Q comps: " << temp_time << "ms \n";
    return true;
}

bool iLQR::UpdateLambda(bool valid_backwards_pass){
    bool lambda_exit = false;

    if(!valid_backwards_pass){
        lambda *= lambda_factor;
    }
    else{
        lambda /= lambda_factor;
    }

    // Clamp lambda and stop backwards pass
    if(lambda > max_lambda){
        lambda = max_lambda;
        lambda_exit = true;
    }

    if(lambda < min_lambda){
        lambda = min_lambda;
    }

    return lambda_exit;
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
double iLQR::ForwardsPass(double _old_cost){
    double _new_cost;
    bool cost_reduction = false;
    int alphaCount = 0;

    // Aliases
    int dof_quat = activeModelTranslator->current_state_vector.dof_quat;
    int nq = MuJoCo_helper->model->nq, nv = MuJoCo_helper->model->nv,
            na = MuJoCo_helper->model->na;

    MatrixXd state_feedback(2 * dof, 1);
    MatrixXd X_new(dof + dof_quat, 1);
    MatrixXd feedback_gain(num_ctrl, 1);
    MatrixXd U_new(num_ctrl, 1);

    auto *vel_diff  = new double[nv];
    auto *state_old = new double[nq + nv + na];
    auto *state_new = new double[nq + nv + na];

    std::vector<double> alphas = {1.0, 0.8, 0.5, 0.3, 0.1};
    MatrixXd control_limits = activeModelTranslator->ReturnControlLimits(activeModelTranslator->current_state_vector);

    while(!cost_reduction){

        // Copy initial data state into main data state for rollout
        MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

        _new_cost = 0;

        for(int t = 0; t < horizon_length; t++) {

            X_new = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->main_data,
                                                                        activeModelTranslator->current_state_vector);

            // Calculate difference from new state to old state
            // If there are no angular dofs, simply subtract the two
            if(dof == dof_quat){
                state_feedback = X_new - X_old[t];
            }
            else{
                // TODO - this is slightly inefficient, i should make custom functions for this
                mj_getState(MuJoCo_helper->model, MuJoCo_helper->saved_systems_state_list[t], state_old, mjSTATE_PHYSICS);
                mj_getState(MuJoCo_helper->model, MuJoCo_helper->main_data, state_new, mjSTATE_PHYSICS);

                // vel_diff = (state_new - state_old) / dt
                mj_differentiatePos(MuJoCo_helper->model, vel_diff, 1.0, state_old, state_new);

                // Compute state feedback
                // position differences
                for(int j = 0; j < dof; j++){
                    int q_index = activeModelTranslator->StateIndexToQposIndex(j, activeModelTranslator->current_state_vector);
                    state_feedback(j) = vel_diff[q_index];
                }

                // velocity differences
                for(int j = 0; j < dof; j++){
                    state_feedback(j + dof) = X_new(dof_quat + j) - X_old[t](dof_quat + j);
                }
            }

            feedback_gain = K[t] * state_feedback;

            // Calculate new optimal controls
            U_new = U_old[t] + (alphas[alphaCount] * k[t]) + feedback_gain;

            // Clamp torque within limits
            for(int i = 0; i < num_ctrl; i++){
                if(U_new(i) > control_limits(2*i+1, 0)) U_new(i) = control_limits(2*i+1, 0);
                if(U_new(i) < control_limits(2*i, 0)) U_new(i) = control_limits(2*i, 0);
            }

            activeModelTranslator->SetControlVector(U_new, MuJoCo_helper->main_data,
                                                    activeModelTranslator->current_state_vector);

            double newStateCost;
            activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[t]);
            if(t == horizon_length - 1){
                newStateCost = activeModelTranslator->CostFunction(residuals[t],
                                                                   activeModelTranslator->full_state_vector, true);
            }
            else{
                newStateCost = activeModelTranslator->CostFunction(residuals[t],
                                                                   activeModelTranslator->full_state_vector, false);
            }

//            newCost += (newStateCost * active_physics_simulator->returnModelTimeStep());
            _new_cost += newStateCost;

            mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

            // Copy system state to fp_rollout_buffer to prevent a second rollout of computations using simulation integration
            SaveSystemStateToRolloutData(MuJoCo_helper->main_data, 0, t);

//             if(t % 5 == 0){
//                 const char* fplabel = "fp";
//                 MuJoCo_helper->CopySystemState(MuJoCo_helper->vis_data, MuJoCo_helper->main_data);
//                 MuJoCo_helper->ForwardSimulator(MuJoCo_helper->vis_data);
//                 active_visualiser->render(fplabel);
//             }

        }

        std::cout << "cost from alpha: " << alphas[alphaCount] << " is " << _new_cost << std::endl;

        if(_new_cost < _old_cost){
            cost_reduction = true;
        }
        else{
            if(alphaCount >= alphas.size() - 1){
                break;
            }
            alphaCount++;
        }
    }

    last_alpha = alphas[alphaCount];

    // Compute expected costreduction
    expected = -(last_alpha * delta_J + (pow(last_alpha, 2) / 2) * delta_J);
    expecteds.push_back(expected);

//    std::cout << "expected :" << expected << " actual: " << old_cost - newCost << std::endl;

    // Free memory
    delete[] vel_diff;
    delete[] state_old;
    delete[] state_new;

    // If the cost was reduced
    if(_new_cost < _old_cost){
        // Compute surprise

        surprise = (_old_cost - _new_cost) / expected;
        surprises.push_back(surprise);

        // Reset the system state to the initial state
        MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

        //Copy the rollout buffer to saved systems state list, prevents recomputation using optimal controls
//        MuJoCo_helper->CopyRolloutBufferToSavedSystemStatesList();
        SaveBestRollout(0);

        return _new_cost;
    }

    surprise = 0.0;
    surprises.push_back(0.0);

    return _old_cost;
}

double iLQR::ForwardsPassParallel(int thread_id, double alpha){
    double _new_cost = 0.0;

    // Aliases
    int dof_quat = activeModelTranslator->current_state_vector.dof_quat;
    int nq = MuJoCo_helper->model->nq, nv = MuJoCo_helper->model->nv,
            na = MuJoCo_helper->model->na;

    MatrixXd state_feedback(2 * dof, 1);
    MatrixXd X_new(dof + dof_quat, 1);
    MatrixXd feedback_gain(num_ctrl, 1);
    MatrixXd U_new(num_ctrl, 1);

    auto *vel_diff  = new double[nv];
    auto *state_old = new double[nq + nv + na];
    auto *state_new = new double[nq + nv + na];

    // Copy initial data state into main data state for rollout
    MuJoCo_helper->CopySystemState(MuJoCo_helper->fd_data[thread_id], MuJoCo_helper->saved_systems_state_list[0]);
    MatrixXd control_limits = activeModelTranslator->ReturnControlLimits(activeModelTranslator->current_state_vector);

    for(int t = 0; t < horizon_length; t++) {

        X_new = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->fd_data[thread_id],
                                                                    activeModelTranslator->current_state_vector);

        // Calculate difference from new state to old state
        // If there are no angular dofs, simply subtract the two
        if(dof == dof_quat){
            state_feedback = X_new - X_old[t];
        }
        else{
            // TODO - this is slightly inefficient, i should make custom functions for this
            mj_getState(MuJoCo_helper->model, MuJoCo_helper->saved_systems_state_list[t], state_old, mjSTATE_PHYSICS);
            mj_getState(MuJoCo_helper->model, MuJoCo_helper->fd_data[thread_id], state_new, mjSTATE_PHYSICS);

            // vel_diff = (state_new - state_old) / dt
            mj_differentiatePos(MuJoCo_helper->model, vel_diff, 1.0, state_old, state_new);

            // Compute state feedback
            // position differences
            for(int j = 0; j < dof; j++){
                int q_index = activeModelTranslator->StateIndexToQposIndex(j, activeModelTranslator->current_state_vector);
                state_feedback(j) = vel_diff[q_index];
            }

            // velocity differences
            for(int j = 0; j < dof; j++){
                state_feedback(j + dof) = X_new(dof_quat + j) - X_old[t](dof_quat + j);
            }
        }

        feedback_gain = K[t] * state_feedback;

        // Calculate new optimal controls
        U_new = U_old[t] + (alpha * k[t]) + feedback_gain;

        // Clamp torque within limits
        for(int i = 0; i < num_ctrl; i++){
            if(U_new(i) > control_limits(2*i+1, 0)){
//                std::cout << "U_new(i) is " << U_new(i) << " and control limits are " << control_limits(2*i+1, 0) << "\n";
                U_new(i) = control_limits(2*i+1, 0);
//                std::cout << "U_new(i) is " << U_new(i) << "\n";

            }
            if(U_new(i) < control_limits(2*i, 0)) U_new(i) = control_limits(2*i, 0);
        }

//        if(t == 0){
//            std::cout << "U_new[0] " << U_new.transpose() << "\n";
//        }

        activeModelTranslator->SetControlVector(U_new, MuJoCo_helper->fd_data[thread_id],
                                                activeModelTranslator->current_state_vector);

        double new_state_cost;
        // Terminal state
        MatrixXd residuals_t(activeModelTranslator->residual_list.size(), 1);
        activeModelTranslator->Residuals(MuJoCo_helper->fd_data[thread_id], residuals_t);
        if(t == horizon_length - 1){
            new_state_cost = activeModelTranslator->CostFunction(residuals_t,
                                                                 activeModelTranslator->full_state_vector, true);
        }
        else{
            new_state_cost = activeModelTranslator->CostFunction(residuals_t,
                                                                 activeModelTranslator->full_state_vector, false);
        }

        _new_cost += new_state_cost;

        mj_step(MuJoCo_helper->model, MuJoCo_helper->fd_data[thread_id]);

        // Copy system state to fp_rollout_buffer to prevent a second rollout of computations using simulation integration

        SaveSystemStateToRolloutData(MuJoCo_helper->fd_data[thread_id], thread_id, t);
    }

    // Compute expected costreduction
    expected = -(alpha * delta_J + (pow(alpha, 2) / 2) * delta_J);
//    expecteds.push_back(expected);

//    std::cout << "expected :" << expected << " actual: " << old_cost - newCost << std::endl;

    // Free memory
    delete[] vel_diff;
    delete[] state_old;
    delete[] state_new;

    return _new_cost;
}

void iLQR::UpdateNominal(){
    // Update the nominal state and control vector
    for(int t = 0 ; t < horizon_length; t++){
        X_old.at(t + 1) = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->saved_systems_state_list[t + 1],
                                                                              activeModelTranslator->current_state_vector);
        U_old[t] = activeModelTranslator->ReturnControlVector(MuJoCo_helper->saved_systems_state_list[t],
                                                              activeModelTranslator->current_state_vector);
    }
//    std::cout << "X[0] " << X_old[0].transpose() << "\n";
//    std::cout << "nominal U[0] " << U_old[0].transpose() << "\n";

    old_cost = new_cost;
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
              << std::setw(18) << "|   Best Alpha  " << " |" << std::endl;
}

void iLQR::PrintBannerIteration(int iteration, double new_cost, double old_cost, double eps,
                                double lambda, double percent_derivatives, double time_derivs, double time_bp,
                                double time_fp, double best_alpha){

    std::cout << std::left << "|" << std::setw(11) << iteration
              << "|" << std::setw(11) << old_cost
              << "|" << std::setw(11) << new_cost
              << "|" << std::setprecision(3) << std::setw(7)  << eps
              << "|" << std::setw(9) << lambda
              << "|" << std::setw(15) << percent_derivatives
              << "|" << std::setw(19) <<time_derivs
              << "|" << std::setw(14)  << time_bp
              << "|" << std::setw(14) << time_fp
              << "|" << std::setw(18) << best_alpha << "|" << std::endl;
}

