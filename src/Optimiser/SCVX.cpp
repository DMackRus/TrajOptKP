#include "Optimiser/SCVX.h"

SCVX::SCVX(std::shared_ptr<ModelTranslator> _modelTranslator,
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

    // Resize internal data variables
    Resize(activeModelTranslator->current_state_vector.dof,
           activeModelTranslator->current_state_vector.num_ctrl,
           horizon);
}

void SCVX::Resize(int new_num_dofs, int new_num_ctrl, int new_horizon){
    auto start = std::chrono::high_resolution_clock::now();

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
        contact_list.clear();
    }

    // dependant on both dofs and num_ctrl
    B.clear();

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

            U_old.emplace_back(MatrixXd(num_ctrl, 1));

            vector<MatrixXd> r_u_;
            for(int i = 0; i < activeModelTranslator->residual_list.size(); i++) {
                r_u_.emplace_back(MatrixXd(num_ctrl, 1));
            }

            r_u.emplace_back(r_u_);
        }

        B.emplace_back(MatrixXd(2*dof, num_ctrl));
    }

    // One more state than control
    if(update_dof){
        l_x.push_back(MatrixXd(2*dof, 1));
        l_xx.push_back(MatrixXd(2*dof, 2*dof));

        X_old.push_back(MatrixXd(num_dof_quat + num_dof, 1));
        X_new.push_back(MatrixXd(num_dof_quat + num_dof, 1));

        l_u.emplace_back(MatrixXd(num_ctrl, 1));
        l_uu.emplace_back(MatrixXd(num_ctrl, num_ctrl));

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
        // Clear old rollout data
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
            contact_list.emplace_back();

            // Empty contact data lists for all parallel rollouts
            for(int i = 0; i < num_parallel_rollouts; i++){
                rollout_data[i][t].contacts.clear();
            }
        }
    }

    // QP related things
    int N = this->horizon_length;
    int n_u = this->num_ctrl;
    int n_x = 2 * this->dof;      // matches your l_x / l_xx sizing
    int n_z = N * n_u + N * n_x;
    int n_eq = N * n_x;

    qp_candidate_controls.clear();
    qp_candidate_states.clear();
    X_old_no_quat.clear();

    for(int t = 0; t < this->horizon_length; t++){
        qp_candidate_controls.push_back(MatrixXd::Zero(num_ctrl, 1));
        qp_candidate_states.push_back(MatrixXd::Zero(2 * num_dof, 1));
        X_old_no_quat.push_back(MatrixXd::Zero(2 * num_dof, 1));
    }

    // One more state than control
    qp_candidate_states.push_back(MatrixXd::Zero(2 * num_dof, 1));
    X_old_no_quat.push_back(MatrixXd::Zero(2 * num_dof, 1));


    // Resize Keypoint generator class
    keypoint_generator->Resize(dof, num_ctrl, horizon_length);

    if(verbose_output){
        std::cout << "iLQR time to allocate memory: " << duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0 << " ms \n";
    }
}

double SCVX::RolloutTrajectory(mjData* d, bool save_states, std::vector<MatrixXd> initial_controls){
    double cost = 0.0;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, d);

    if(MuJoCo_helper->CheckIfDataIndexExists(0)){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[0], MuJoCo_helper->main_data);
    }
    else{
        MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
    }

    // Get contact
    activeModelTranslator->GetContacts(MuJoCo_helper->main_data, contact_list[0]);

    for(int i = 0; i < horizon_length; i++){
        // set controls
        activeModelTranslator->SetControlVector(initial_controls[i],
                                                MuJoCo_helper->main_data,
                                                activeModelTranslator->full_state_vector);

        // If required to save states to trajectory tracking, then save state
        if(save_states){
            X_old[i] = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);
            X_old_no_quat[i] = activeModelTranslator->ReturnStateVector(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);
            U_old[i] = activeModelTranslator->ReturnControlVector(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);
            if(MuJoCo_helper->CheckIfDataIndexExists(i + 1)){
                MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[i + 1], MuJoCo_helper->main_data);
            }
            else{
                MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
            }
        }

        // Get contacts
        activeModelTranslator->GetContacts(MuJoCo_helper->main_data, contact_list[i]);

        // Return cost for this state
        double state_cost;
        activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[i]);
        state_cost = activeModelTranslator->CostFunction(residuals[i], activeModelTranslator->full_state_vector, false);

        // Integrate simulator
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

        cost += state_cost;
    }

    activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[horizon_length]);
    cost += activeModelTranslator->CostFunction(residuals[horizon_length], activeModelTranslator->full_state_vector, true);

    // Save the last state
    activeModelTranslator->GetContacts(MuJoCo_helper->main_data, contact_list[horizon_length]);
    X_old[horizon_length] = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);
    X_old_no_quat[horizon_length] = activeModelTranslator->ReturnStateVector(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);

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
std::vector<MatrixXd> SCVX::Optimise(mjData *d, std::vector<MatrixXd> initial_controls, int max_iterations, int min_iterations, int horizon_length){
    auto optStart = high_resolution_clock::now();
    start_time = high_resolution_clock::now();

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
    avg_time_keypoints_ms = 0.0;
    avg_time_FD_derivs_ms = 0.0;
    avg_time_interpolation_ms = 0.0;
    avg_time_cost_derivs_ms = 0.0;
    avg_time_forwards_pass_ms = 0.0;
    avg_time_backwards_pass_ms = 0.0;
    avg_percent_derivs = 0;
    num_iterations = 0;
    avg_dofs = 0.0;

    percentage_derivs_per_iteration.clear();
    num_dofs.clear();
    cost_history.clear();
    time_backwards_pass_ms.clear();
    time_forwardsPass_ms.clear();
    time_get_derivs_ms.clear();
    time_keypoints_ms.clear();
    time_FD_derivs_ms.clear();
    time_interpolation_ms.clear();
    time_cost_derivs_ms.clear();

    cost_after_iteration.clear();
    cost_reduction_after_iteration.clear();
    time_after_iteration_ms.clear();
    // ------------------------------------------------------------------------

    auto time_start = high_resolution_clock::now();
    old_cost = RolloutTrajectory(d, true, initial_controls);
    if(verbose_output){
        std::cout << "cost from rollout: " << old_cost << "\n";
    }
    auto time_end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(time_end - time_start);
    if(verbose_output) {
        PrintBanner(duration.count() / 1000.0f);
    }
    initial_cost = old_cost;
    new_cost = old_cost;
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    // ------------------- Main optimisation iteration loop ------------------------
    cost_reduced_last_iter = true;
    for(int i = 0; i < max_iterations; i++) {
        num_iterations++;

        bool converged = false;
        bool failed = false;
        Iteration(i, converged, failed);

        if (converged && (i >= min_iterations)) {
            break;
        }

        if(failed){
            break;
        }
    }

    // --------------------  Computing testing results ---------------------------
    cost_reduction = 1 - (new_cost / initial_cost);
    auto optFinish = high_resolution_clock::now();
    auto optDuration = duration_cast<microseconds>(optFinish - optStart);
    opt_time_ms = optDuration.count() / 1000.0;

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

    // Time keypoints
    for(double time_keypoints_m : time_keypoints_ms){
        avg_time_keypoints_ms += time_keypoints_m;
    }

    // Time FD derivs
    for(double time_FD_derivs_m : time_FD_derivs_ms){
        avg_time_FD_derivs_ms += time_FD_derivs_m;
    }

    // Time interpolation
    for(double time_interpolation_m : time_interpolation_ms){
        avg_time_interpolation_ms += time_interpolation_m;
    }

    // Time cost derivs
    for(double time_cost_derivs_m : time_cost_derivs_ms){
        avg_time_cost_derivs_ms += time_cost_derivs_m;
    }

    // Percent derivs
    for(double i : percentage_derivs_per_iteration){
        avg_percent_derivs += i;
    }

    avg_time_get_derivs_ms /= static_cast<int>(time_get_derivs_ms.size());
    avg_percent_derivs /= static_cast<int>(percentage_derivs_per_iteration.size());
    avg_time_keypoints_ms /= static_cast<int>(time_keypoints_ms.size());
    avg_time_FD_derivs_ms /= static_cast<int>(time_FD_derivs_ms.size());
    avg_time_interpolation_ms /= static_cast<int>(time_interpolation_ms.size());
    avg_time_cost_derivs_ms /= static_cast<int>(time_cost_derivs_ms.size());

    // Time QP solving
    for(double time_qp_solve_m : time_qp_ms){
        avg_time_qp_ms += time_qp_solve_m;
    }

    avg_time_qp_ms /= static_cast<int>(time_qp_ms.size());

    // Time forwards pass
    for(double time_forwardsPass_m : time_forwardsPass_ms){
        avg_time_forwards_pass_ms += time_forwardsPass_m;
    }

    if(!time_forwardsPass_ms.empty()){
        avg_time_forwards_pass_ms /= static_cast<int>(time_forwardsPass_ms.size());
    }

    // Load the initial data back into main data
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    for(int i = 0; i < horizon_length; i++){
        optimisedControls[i] = U_old[i];
    }

    return optimisedControls;
}

void SCVX::Iteration(int iteration_num, bool &converged, bool &failed){

    // This should always remain the same in baseline SCVX
    num_dofs.push_back(activeModelTranslator->current_state_vector.dof);

    // STEP 1 - Generate dynamics derivatives and cost derivatives
    auto timer_start = high_resolution_clock::now();
    if(cost_reduced_last_iter){
        GenerateDerivatives();
//        std::cout << "A[0] " << A[0] << "\n";
//        std::cout << "B[0] " << B[0] << "\n";
//        std::cout << "A[1] " << A[1] << "\n";
//        std::cout << "B[1] " << B[1] << "\n";
//        std::cout << "l_xx[0] " << l_xx[0] << "\n";
//        std::cout << "l_x[0] " << l_x[0] << "\n";
//        std::cout << "terminal residual derivs: " << residuals[horizon_length].transpose() << "\n";
//        std::cout << "r_x[horizon_length] " << r_x[horizon_length][0] << "\n";
//        std::cout << "r_u[horizon_length] " << r_u[horizon_length][0] << "\n";
//        std::cout << "l_xx[horizon] " << l_xx[horizon_length] << "\n";
//        std::cout << "l_x[horizon] " << l_x[horizon_length] << "\n";
//        std::cout << "A[horizon - 1]" << A[horizon_length - 1] << "\n";
//        std::cout << "B[horizon - 1] " << B[horizon_length - 1] << "\n";
    }
    else{
        percentage_derivs_per_iteration.push_back(0.0);
    }
    time_get_derivs_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);

    // STEP 2 - Formulate and solve the QP subproblem
    timer_start = high_resolution_clock::now();
    bool qp_success = SolveQP();
    time_qp_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);

    if(!qp_success){
        // Terminate optimization early
        failed = true;
        std::cout << "QP solve failed, terminating optimization early. \n";

        // Clean up data storage to prevent seg faults
        time_forwardsPass_ms.push_back(0.0f);
        cost_after_iteration.push_back(new_cost);
        cost_reduction_after_iteration.push_back(1 - (new_cost / initial_cost));
        time_after_iteration_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - start_time).count() / 1000.0f);

        return;
    }

    // STEP 2a - Evaluate cost of the candidate states and controls
    EvaluateLinSolutionCost();

    // STEP 3 - Rollout the new trajectory from QP sub-problem using original nonlinear dynamics
    timer_start = high_resolution_clock::now();
    non_linear_cost = ForwardsPass(old_cost);
    time_forwardsPass_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);

    std::cout << "non linear cost: " << non_linear_cost << "\n";


    if(verbose_output){
        PrintBannerIteration(iteration_num, non_linear_cost, old_cost,
                             1 - (non_linear_cost / old_cost), percentage_derivs_per_iteration[iteration_num],
                             time_get_derivs_ms[iteration_num], time_qp_ms[iteration_num], time_forwardsPass_ms[iteration_num]);
    }

    // STEP 4 - Handling deviations in non_linear_cost and linear_cost to scale trust region
    if(non_linear_cost < old_cost){
        // If cost reduced, then we can increase the trust region

        double actual_cost_improvement = old_cost - non_linear_cost;
        double predicted_cost_improvement = old_cost - linear_cost;

        double similarity_ratio = actual_cost_improvement / (predicted_cost_improvement + 0.00001);

        std::cout << "Similarity ratio: " << similarity_ratio << "\n";

        if(similarity_ratio < 0.2){
            // Shrink
            trust_region_radius *= 0.5;
        }
        else if(similarity_ratio >= 0.2 && similarity_ratio < 0.7){
            // Keep the same
            trust_region_radius *= 1.0;
        }
        else{
            // Expand trust region
            trust_region_radius *= 1.5;
        }

//        if(trust_region_validity < 0.25) {
//            trust_region_radius *= 0.5;
//        }
//        else if(trust_region_validity > 0.75) {
//            trust_region_radius *= 1.5;
//        }

        // TODO check non linear cost versus linear cost and decide whether to increase or decrease trust radius
//        trust_region_radius *= 1.2;
        SaveBestRollout(0);
        new_cost = non_linear_cost;
    }
    else{
        // If cost did not reduce, then we need to decrease the trust region
        trust_region_radius *= 0.5;

        // Dont update nominal
    }

    // STEP 5 - Check for convergence
    converged = CheckForConvergence(old_cost, new_cost);


    if(non_linear_cost < old_cost){
        UpdateNominal();
        cost_reduced_last_iter = true;
        old_cost = non_linear_cost;
    }
    else{
        cost_reduced_last_iter = false;
    }
    cost_after_iteration.push_back(new_cost);
    cost_reduction_after_iteration.push_back(1 - (new_cost / initial_cost));
    time_after_iteration_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - start_time).count() / 1000.0f);
}

void SCVX::EvaluateLinSolutionCost(){
    // Loop through horizon, set the states and controls from QP solution and calculate cost

//    std::cout << "EVALUATE LIN SOLUTION COST \n";

    // Copy the initial state to the main data
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    double lin_cost = 0.0;
    for(int t = 0; t < horizon_length; t++){

        activeModelTranslator->SetControlVector(qp_candidate_controls[t], MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);

        // Return cost for this state
        activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[t]);
        double state_cost = activeModelTranslator->CostFunction(residuals[t], activeModelTranslator->full_state_vector, false);

//        std::cout << "State at t == " << t << ": " << activeModelTranslator->ReturnStateVector(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector).transpose() << "\n";
//        std::cout << "state cost at t == " << t << ": " << state_cost << "\n";

        lin_cost += state_cost;

        // This is sort of like mj_step for this function, setting state directly from QP solution
        // Note: t+1 as this is effectively mj_step setting state for next time-step.
        activeModelTranslator->SetStateVector(qp_candidate_states[t+1], MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);

        // TODO - temp code to be removed later

//        MuJoCo_helper->CopySystemState(MuJoCo_helper->vis_data, MuJoCo_helper->main_data);
//        MuJoCo_helper->ForwardSimulator(MuJoCo_helper->vis_data);
//        active_visualiser->render("Lin solution");
    }

    activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[horizon_length]);
    lin_cost += activeModelTranslator->CostFunction(residuals[horizon_length], activeModelTranslator->full_state_vector, true);

    linear_cost = lin_cost;

    std::cout << "lin cost is " << lin_cost << "\n";
}

double SCVX::ForwardsPass(double _old_cost){
    double non_linear_cost = 0.0;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    for(int t = 0; t < horizon_length; t++){
        // Set the new control
        activeModelTranslator->SetControlVector(qp_candidate_controls[t], MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);

        // Calculate cost for this state
        activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[t]);
        double state_cost = activeModelTranslator->CostFunction(residuals[t], activeModelTranslator->full_state_vector, false);

        SaveSystemStateToRolloutData(MuJoCo_helper->main_data, 0, t, residuals[t]);

        // Integrate the simulator
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

        // Get contacts
        activeModelTranslator->GetContacts(MuJoCo_helper->main_data, contact_list[t+1]);

        non_linear_cost += state_cost;
    }

    // Terminal cost
    activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[horizon_length]);
    non_linear_cost += activeModelTranslator->CostFunction(residuals[horizon_length], activeModelTranslator->full_state_vector, true);

    // Save the last state
    SaveSystemStateToRolloutData(MuJoCo_helper->main_data, 0, horizon_length, residuals[horizon_length]);

    // Return new cost which in the case of SCVX is non-linear cost
    return non_linear_cost;
}

void SCVX::AddL1TrustRegionWithResize(Eigen::SparseMatrix<double>& A,
                                      Eigen::VectorXd& l,
                                      Eigen::VectorXd& u,
                                      Eigen::SparseMatrix<double>& hessian_matrix,
                                      Eigen::VectorXd& gradient_vector,
                                      double rho)
{
    int T  = horizon_length;
    int n_k = T / controls_per_knotpoint;
    int nx = 2 * dof;
    int nu = num_ctrl;

    // ---------------- Decision structure ----------------
    // z = [δx0 .. δxT, δu0 .. δu_{T-1}]
    int n_x_block = (n_k + 1) * nx;
    int n_u_block = n_k * nu;
    int n_z       = n_x_block + n_u_block;

    // slack variables, one per element of z
    int n_tr = n_z;

    // ---------------- Old sizes ----------------
    int old_rows  = A.rows();
    int old_cols  = A.cols();
    int old_nvars = old_cols;

    assert(old_cols == n_z && "A must currently have (T+1)*nx + T*nu columns.");
    assert(gradient_vector.size() == old_nvars && "gradient_vector size mismatch");
    assert(hessian_matrix.rows() == old_nvars &&
           hessian_matrix.cols() == old_nvars &&
           "hessian_matrix size mismatch");

    // ---------------- New sizes ----------------
    int add_rows = n_z + n_z + n_tr + 1;  // (1) |z| <= t, (2) -t <= 0, (3) sum(t) <= rho
    int new_rows = old_rows + add_rows;
    int new_cols = old_nvars + n_tr;      // append slack variables

    // ---------------- Build new A ----------------
    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(static_cast<size_t>(A.nonZeros()) + 3ULL * n_z + n_tr + 4);

    // copy old A
    A.makeCompressed();
    for (int k = 0; k < A.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(A, k); it; ++it) {
            trips.emplace_back(it.row(), it.col(), it.value());
        }
    }

    int row_off_1 = old_rows;             // z - t <= 0iLQR
    int row_off_2 = row_off_1 + n_z;      // -z - t <= 0
    int row_off_3 = row_off_2 + n_z;      // -t <= 0
    int row_off_4 = row_off_3 + n_tr;     // sum(t) <= rho
    int t_offset  = old_nvars;            // first index of slack block

    // (1) z - t <= 0
    for (int i = 0; i < n_z; ++i) {
        trips.emplace_back(row_off_1 + i, i,         1.0);
        trips.emplace_back(row_off_1 + i, t_offset + i, -1.0);
    }

    // (2) -z - t <= 0
    for (int i = 0; i < n_z; ++i) {
        trips.emplace_back(row_off_2 + i, i,        -1.0);
        trips.emplace_back(row_off_2 + i, t_offset + i, -1.0);
    }

    // (3) -t <= 0
    for (int i = 0; i < n_tr; ++i) {
        trips.emplace_back(row_off_3 + i, t_offset + i, -1.0);
    }

    // (4) sum(t) <= rho
    for (int i = 0; i < n_tr; ++i) {
        trips.emplace_back(row_off_4, t_offset + i, 1.0);
    }

    // ---------------- Bounds ----------------
    Eigen::VectorXd l_new(new_rows);
    Eigen::VectorXd u_new(new_rows);

    // copy old bounds
    l_new.head(old_rows) = l;
    u_new.head(old_rows) = u;

    // (1) z - t <= 0
    l_new.segment(row_off_1, n_z).setConstant(-OSQP_INFTY);
    u_new.segment(row_off_1, n_z).setZero();

    // (2) -z - t <= 0
    l_new.segment(row_off_2, n_z).setConstant(-OSQP_INFTY);
    u_new.segment(row_off_2, n_z).setZero();

    // (3) -t <= 0
    l_new.segment(row_off_3, n_tr).setConstant(-OSQP_INFTY);
    u_new.segment(row_off_3, n_tr).setZero();

    // (4) sum(t) <= rho
    l_new(row_off_4) = -OSQP_INFTY;
    u_new(row_off_4) = rho;

    // ---------------- Assemble A ----------------
    Eigen::SparseMatrix<double> A_new(new_rows, new_cols);
    A_new.setFromTriplets(trips.begin(), trips.end());
    A_new.makeCompressed();

    // ---------------- Expand Hessian & gradient ----------------
    int new_nvars = new_cols;

    Eigen::VectorXd q_new(new_nvars);
    q_new.head(old_nvars) = gradient_vector;
    q_new.tail(n_tr).setZero(); // no cost on slacks

    std::vector<Eigen::Triplet<double>> h_trips;
    h_trips.reserve(static_cast<size_t>(hessian_matrix.nonZeros()));

    hessian_matrix.makeCompressed();
    for (int k = 0; k < hessian_matrix.outerSize(); ++k) {
        for (Eigen::SparseMatrix<double>::InnerIterator it(hessian_matrix, k); it; ++it) {
            h_trips.emplace_back(it.row(), it.col(), it.value());
        }
    }

    Eigen::SparseMatrix<double> P_new(new_nvars, new_nvars);
    if (!h_trips.empty()) {
        P_new.setFromTriplets(h_trips.begin(), h_trips.end());
    } else {
        P_new.setZero();
    }
    P_new.makeCompressed();

    // ---------------- Swap back ----------------
    A.swap(A_new);
    l.swap(l_new);
    u.swap(u_new);
    hessian_matrix.swap(P_new);
    gradient_vector.swap(q_new);
}

void SCVX::ComputeCompressedDynamics(std::vector<Eigen::MatrixXd> &Phi,
                                     std::vector<Eigen::MatrixXd> &Gamma){

//    std::vector<Eigen::MatrixXd> Phi;
//    std::vector<Eigen::MatrixXd> Gamma;

    // Resize them appropriately
    if(horizon_length % controls_per_knotpoint != 0){
        throw std::runtime_error("ComputeKnotDynamics: horizon_length is not divisible by controls_per_knotpoint.");
    }
    const int num_knot_points = horizon_length / controls_per_knotpoint;

    const int K    = num_knot_points;
    const int n_x  = A[0].rows();
    const int n_u  = B[0].cols();

    Phi.resize(K);
    Gamma.resize(K);

        for (int k = 0; k < K; ++k)
        {
            // ----- Compute A_k ------
            Eigen::MatrixXd P = Eigen::MatrixXd::Identity(n_x, n_x);

            int t_start = k * controls_per_knotpoint;
            int t_end   = t_start + controls_per_knotpoint;

            // Backward product: A_{t_end-1} ... A_{t_start}
            for (int t = t_end - 1; t >= t_start; --t) {
                P = A[t] * P;
            }
            Phi[k] = P;

            // ----- Compute Γ_k -----
            //
            // Gamma = sum_{i=0}^{n_sub-1} (A_{t_end-1} ... A_{t_start+i+1}) * B_{t_start+i}
            //
            // We compute the products incrementally:
            // Let Q = I initially, representing product over an empty range.
            // At each iteration i, Q = A_{t_end-1} ... A_{t_start+i+1}.
            //
            // When i increments, Q is left-multiplied by A_{t_start+i+1}.
            Eigen::MatrixXd G = Eigen::MatrixXd::Zero(n_x, n_u);
            Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n_x, n_x);

            for (int i = controls_per_knotpoint - 1; i >= 0; --i)
            {
                // index of low-level step inside this knot
                int ti = t_start + i;

                // Add Q * B[ti] to G
                G += Q * B[ti];

                // If not at the last iteration, update Q ← Q * A_{ti}
                if (i > 0) {
                    Q = A[ti] * Q;
                }
            }

            Gamma[k] = G;
        }

//    std::cout << "Gamma[0]: \n" << Gamma[0] << "\n";
//
//    std::cout << "phi[0]: \n" << Phi[0] << "\n";

}

void SCVX::SetDynamicsConstraints(Eigen::SparseMatrix<double>& linear_matrix,
                                  Eigen::VectorXd& lower_bound,
                                  Eigen::VectorXd& upper_bound,
                                  const Eigen::VectorXd& x0){

    int T   = horizon_length; // number of control steps
    int n_k = horizon_length / controls_per_knotpoint; // number of knot points

    // Sanity check to make sure horizon_length is divisible by controls_per_knotpoint
    if(horizon_length % controls_per_knotpoint != 0){
        throw std::runtime_error("SetDynamicsConstraints: horizon_length is not divisible by controls_per_knotpoint.");
    }
    int n_x = (2 * dof);      // state dimension
    int n_u = num_ctrl;       // control dimension

    // Compute compressed linear dynamics matrices
    std::vector<MatrixXd> Phi, Gamma;
    ComputeCompressedDynamics(Phi, Gamma);

    // Decision vector length z = [delta x_0, delta x_1, ..., delta x_T, delta u_0, ..., delta u_{T-1}]
    int n_z = (n_u * n_k) + (n_x * (n_k+1));

    // Equality constraints: delta x_{t+1} = A_t delta x_t + B_t delta u_t, t=0..T-1
    int n_eq = n_k * n_x;
    linear_matrix.resize(n_eq, n_z);

    std::vector<Eigen::Triplet<double>> trips;
    trips.reserve(static_cast<size_t>(n_eq) * (n_x + n_u));

    // bounds vector for Ax = b form
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(n_eq);

    for (int k = 0; k < n_k; ++k) {
        int row_base = k * n_x;

        // Offsets in z:
        int u_offset  = (n_k+1) * n_x;
        int idx_x_tp1 = (k+1) * n_x;             // x_{t+1} in z (x1 is at 0)
        int idx_x_t   = (k) * n_x;       // x_t in z (only valid for t > 0)
        int idx_u_t   = u_offset + (k * n_u);

        // +1 * x_{t+1}
        for (int i = 0; i < n_x; ++i) {
            trips.emplace_back(row_base + i, idx_x_tp1 + i, 1.0);
        }

        // Handle -A_t * x_t term
        // \delta x_{t+1} - A_t * \delta x_t - B_t * \delta u_t = 0
        if(k > 0){
            for (int i = 0; i < n_x; ++i) {
                for (int j = 0; j < n_x; ++j) {
                    double val = -Phi[k](i, j);
                    if (val != 0.0) trips.emplace_back(row_base + i, idx_x_t + j, val);
                }
            }
        }

        // Handle -B_t * u_t term
        for (int i = 0; i < n_x; ++i) {
            for (int j = 0; j < n_u; ++j) {
                double val = -Gamma[k](i, j);
                if (val != 0.0) trips.emplace_back(row_base + i, idx_u_t + j, val);
            }
        }
    }

    linear_matrix.setFromTriplets(trips.begin(), trips.end());
    linear_matrix.makeCompressed();

    // Equality bounds = rhs
    lower_bound = rhs;
    upper_bound = rhs;
}

void SCVX::SetCostFunction(Eigen::SparseMatrix<double>& hessian_matrix,
                           Eigen::VectorXd& gradient_vector)
{
    int T  = horizon_length; // number of control stages
    int n_k = horizon_length / controls_per_knotpoint; // number of knot points

    // Sanity check to make sure horizon_length is divisible by controls_per_knotpoint
    if(horizon_length % controls_per_knotpoint != 0){
        throw std::runtime_error("SetDynamicsConstraints: horizon_length is not divisible by controls_per_knotpoint.");
    }
    int nx = dof * 2;
    int nu = num_ctrl;

    int total_vars = (nu * n_k) + (nx * (n_k+1)); // [x_0, x_1..x_T, u₀..u_{T-1}]

    gradient_vector = Eigen::VectorXd::Zero(total_vars);

    auto idx_x = [&](int t) { return (t) * nx; };                   // x_t, t >= 1
    auto idx_u = [&](int t) { return (nx * (n_k+1)) + t * nu; };    // u_t, t >= 0

    // Stage costs
    for (int k = 0; k < n_k; ++k) {
        // Controls u_t
        int iu = idx_u(k);
        gradient_vector.segment(iu, nu) += l_u[k*controls_per_knotpoint];

        // States x_{t+1}
        if(k != 0){
            int ix = idx_x(k);
            gradient_vector.segment(ix, nx) += l_x[k*controls_per_knotpoint];
        }
    }

    // Terminal gradient if provided (size = T+1 in l_x)
    int ixN = idx_x(n_k);
    gradient_vector.segment(ixN, nx) += l_x[T];


    // Hessian triplets
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(static_cast<size_t>(total_vars) * 4);

    for (int k = 0; k < n_k; ++k) {
        // Huu_t
        int iu = idx_u(k);
        const Eigen::MatrixXd &Huu = l_uu[k*controls_per_knotpoint];
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nu; ++j)
                if (Huu(i,j) != 0.0)
                    triplets.emplace_back(iu + i, iu + j, Huu(i,j));

        // Hxx_{t}
        int ix = idx_x(k);
        const Eigen::MatrixXd &Hxx = l_xx[k*controls_per_knotpoint];
        for (int i = 0; i < nx; ++i)
            for (int j = 0; j < nx; ++j)
                if (Hxx(i,j) != 0.0)
                    triplets.emplace_back(ix + i, ix + j, Hxx(i,j));


    }

    // Terminal Hessian if provided
    const Eigen::MatrixXd &HxxN = l_xx[T];
    for (int i = 0; i < nx; ++i)
        for (int j = 0; j < nx; ++j)
            if (HxxN(i,j) != 0.0)
                triplets.emplace_back(ixN + i, ixN + j, HxxN(i,j));

    // Assemble
    hessian_matrix.resize(total_vars, total_vars);
    hessian_matrix.setFromTriplets(triplets.begin(), triplets.end());
    hessian_matrix.makeCompressed();
}

bool SCVX::SolveQP() {

    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);
//    solver.settings()->setMaxIteration(1000);

    // Setup the QP problem
    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linear_matrix;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;

    //Formulate QP matrices
    auto start = std::chrono::high_resolution_clock::now();
//    ComputeCompressedDynamics();
    SetDynamicsConstraints(linear_matrix, lower_bound, upper_bound, X_old_no_quat[0]);
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Time to set dynamics constraints: " << duration_cast<microseconds>(end - start).count() / 1000.0 << " ms \n";

    start = std::chrono::high_resolution_clock::now();
    SetCostFunction(hessian, gradient);
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Time to set cost function: " << duration_cast<microseconds>(end - start).count() / 1000.0 << " ms \n";

    AddL1TrustRegionWithResize(linear_matrix,
                              lower_bound,
                              upper_bound,
                              hessian,
                              gradient,
                                 trust_region_radius);

    // ----------- Set QP matrices --------------

    int num_variables = static_cast<int>(hessian.cols());           // n
    int num_constraints = static_cast<int>(linear_matrix.rows());   // m

    solver.data()->setNumberOfVariables(num_variables);
    solver.data()->setNumberOfConstraints(num_constraints);

    // Print out sizes of matrices
//    std::cout << "Hessian size: " << hessian.rows() << " x " << hessian.cols() << std::endl;
//    std::cout << "Gradient size: " << gradient.size() << std::endl;
//    std::cout << "Linear constraints size: " << linear_matrix.rows() << " x " << linear_matrix.cols() << std::endl;
//    std::cout << "Lower bound size: " << lower_bound.size() << std::endl;
//    std::cout << "Upper bound size: " << upper_bound.size() << std::endl;


    if (!solver.data()->setHessianMatrix(hessian)) {
        std::cerr << "Failed to set Hessian matrix." << std::endl;
        return false;
    }
    if (!solver.data()->setGradient(gradient)) {
        std::cerr << "Failed to set gradient matrix." << std::endl;
        return false;
    }
    if (!solver.data()->setLinearConstraintsMatrix(linear_matrix)) {
        std::cerr << "Failed to set linear constraints matrix." << std::endl;
        return false;
    }
    if (!solver.data()->setLowerBound(lower_bound)) {
        std::cerr << "Failed to set lower bound." << std::endl;
        return false;
    }
    if (!solver.data()->setUpperBound(upper_bound)) {
        std::cerr << "Failed to set upper bound." << std::endl;
        return false;
    }

    if (!solver.initSolver()) {
        std::cerr << "Failed to initialise OSQP solver." << std::endl;
        return false;
    }

    // solve the QP problem
    start = std::chrono::high_resolution_clock::now();
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        std::cerr << "OSQP failed to solve the problem." << std::endl;
        return false;
    }
    end = std::chrono::high_resolution_clock::now();
    std::cout << "Time to get QP solution: " << duration_cast<microseconds>(end - start).count() / 1000.0 << " ms \n";

    // Get the controls from the solution
    auto qp_solution = solver.getSolution();
//    linear_cost = solver.getObjValue(); // TODO - Not used?

    // Compute the candidate delta x and delta u from the QP solution
    // Then compute delta u at all time-steps using knot points
    // Then compute states at all time-steps using linear dynamics \delta x_t+1 = A_t \delta x_t + B_t \delta u_t

    //Loop over knot points
    int n_k = horizon_length / controls_per_knotpoint;
    for(int k = 0; k < n_k; k++){
        // Extract the control vector for this knot point
        int idx_u = k * num_ctrl; // start of u_k

        // Solution to QP is perturbation about nominal trajectory
        Eigen::VectorXd delta_u_k = qp_solution.segment((n_k+1) * (2 * dof) + idx_u, num_ctrl);
        for(int c = 0; c < controls_per_knotpoint; c++){
            int t = k * controls_per_knotpoint + c;
            qp_candidate_controls[t] = U_old[t] + delta_u_k;
        }
    }

    // Loop over time-steps to get states
    for(int k = 0; k < n_k; k++){
        // Extract the state vector for this knot point
        int idx_x = k * (2 * dof); // start of x_k

        // Solution to QP is perturbation about nominal trajectory
        Eigen::VectorXd delta_x_k = qp_solution.segment(idx_x, 2 * dof);
        for(int c = 0; c < controls_per_knotpoint; c++){
            int t = k * controls_per_knotpoint + c;
            qp_candidate_states[t] = X_old_no_quat[t] + delta_x_k;
        }
    }

    // Final state at horizon_length
    qp_candidate_states[horizon_length] = X_old_no_quat[horizon_length] + qp_solution.segment(n_k * (2 * dof), 2 * dof);



//    int control_offset = (horizon_length+1) * (2 * dof);
//    for(int t = 0; t < horizon_length; t++){
//        // Extract the control vector for this time step
//        int idx_u = t * num_ctrl; // start of u_t
//
//        // Solution to QP is perturbation about nominal trajectory
//        qp_candidate_controls[t] = U_old[t] + qp_solution.segment(control_offset + idx_u, num_ctrl);
//        qp_candidate_states[t] = X_old_no_quat[t] + qp_solution.segment(t * (2 * dof), 2 * dof);
//    }
//
//    qp_candidate_states[horizon_length] = X_old_no_quat[horizon_length] + qp_solution.segment((horizon_length) * (2 * dof), 2 * dof);

    return true;
}

void SCVX::UpdateNominal() {
    for(int t = 0 ; t < horizon_length; t++){
        X_old.at(t + 1) = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->saved_systems_state_list[t + 1],
                                                                              activeModelTranslator->current_state_vector);
        U_old[t] = activeModelTranslator->ReturnControlVector(MuJoCo_helper->saved_systems_state_list[t],
                                                              activeModelTranslator->current_state_vector);
        X_old_no_quat[t + 1] = activeModelTranslator->ReturnStateVector(MuJoCo_helper->saved_systems_state_list[t + 1],
                                                                                  activeModelTranslator->current_state_vector);
    }

//    old_cost = new_cost;
}

void SCVX::PrintBanner(double time_rollout){
    std::cout << "--------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "|                                                   SCVX begins, initial rollout took: " << std::setprecision(4) << time_rollout << "                                               |" << std::endl;

    std::cout << std::left << std::setw(12) << "| Iteration"
              << std::setw(12) << "| Old Cost"
              << std::setw(12) << "| New Cost"
              << std::setw(8)  << "| Eps"
              << std::setw(15) << "| Trust Radius"
              << std::setw(16) << "| % Derivatives"
              << std::setw(20) << "| Time Derivs (ms) "
              << std::setw(15) << "| Time QP (ms)"
              << std::setw(15) << "| Time FP (ms)   |" << std::endl;
}

void SCVX::PrintBannerIteration(int iteration, double new_cost, double old_cost, double eps,
                                double percent_derivatives, double time_derivs, double time_qp,
                                double time_fp){

    std::cout << std::left << "|" << std::setw(11) << iteration
              << "|" << std::setw(11) << old_cost
              << "|" << std::setw(11) << new_cost
              << "|" << std::setprecision(3) << std::setw(7)  << eps
              << "|" << std::setw(14) << trust_region_radius
              << "|" << std::setw(15) << percent_derivatives
              << "|" << std::setw(19) <<time_derivs
              << "|" << std::setw(14)  << time_qp
              << "|" << std::setw(14) << time_fp << "|" << std::endl;
}