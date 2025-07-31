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

    // Resize Keypoint generator class
    keypoint_generator->Resize(dof, num_ctrl, horizon_length);

    if(verbose_output){
        std::cout << "iLQR time to allocate memory: " << duration_cast<microseconds>(std::chrono::high_resolution_clock::now() - start).count() / 1000.0 << " ms \n";
    }
}

double SCVX::RolloutTrajectory(mjData* d, bool save_states, std::vector<MatrixXd> initial_controls){
    double cost = 0.0;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, d);

    X_old[0] = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->main_data, activeModelTranslator->full_state_vector);

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
        // Integrate simulator
        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);

        // Get contacts
        activeModelTranslator->GetContacts(MuJoCo_helper->main_data, contact_list[i+1]);

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
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    // ------------------- Main optimisation iteration loop ------------------------
    cost_reduced_last_iter = true;
    for(int i = 0; i < max_iterations; i++) {
        num_iterations++;

        bool lambda_exit, converged = false;
        Iteration(i, converged);

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

void SCVX::Iteration(int iteration_num, bool &converged){

    // This should always remain the same in baseline SCVX
    num_dofs.push_back(activeModelTranslator->current_state_vector.dof);

    // STEP 1 - Generate dynamics derivatives and cost derivatives
    auto timer_start = high_resolution_clock::now();
    if(cost_reduced_last_iter){
        GenerateDerivatives();
    }
    else{
        percentage_derivs_per_iteration.push_back(0.0);
    }
    time_get_derivs_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);

    // STEP 2 - Formulate and solve the QP subproblem
    timer_start = high_resolution_clock::now();
//    SolveQP();
    time_qp_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);


    // STEP 3 - Rollout the new trajectory from QP subproblem using original nonlinear dynamics
    timer_start = high_resolution_clock::now();
    non_linear_cost = ForwardsPass(old_cost); // TODO - write forwards pass
    time_forwardsPass_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - timer_start).count() / 1000.0f);

    // STEP 4 - Update nominal trajectory if cost improvement - also update trust region based on difference between
    // nonlinear dynamics and linearised dynamics
    converged = CheckForConvergence(old_cost, new_cost);
    if(non_linear_cost < old_cost){
        UpdateNominal();
        cost_reduced_last_iter = true;
    }
    else{
        cost_reduced_last_iter = false;
    }
    cost_after_iteration.push_back(new_cost);
    cost_reduction_after_iteration.push_back(1 - (new_cost / initial_cost));
    time_after_iteration_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - start_time).count() / 1000.0f);


    if(verbose_output){
        PrintBannerIteration(iteration_num, new_cost, old_cost,
                             1 - (new_cost / old_cost), percentage_derivs_per_iteration[iteration_num],
                             time_get_derivs_ms[iteration_num], time_backwards_pass_ms[iteration_num], time_forwardsPass_ms[iteration_num]);
    }
}

void SolveQP(
        const SparseMatrix<double>& H,
        const VectorXd& h,
        const SparseMatrix<double>& A_eq,
        const VectorXd& b_eq,
        const SparseMatrix<double>& A_ineq,
        const VectorXd& l_ineq,
        const VectorXd& u_ineq)
{
    int nz = H.rows();  // number of decision variables
    int n_eq = A_eq.rows();
    int n_ineq = A_ineq.rows();
    int n_con = n_eq + n_ineq;

    // Combine A_eq and A_ineq into one matrix A
    SparseMatrix<double> A_combined(n_con, nz);
    typedef Triplet<double> T;
    vector<T> triplets;

    for (int k = 0; k < A_eq.outerSize(); ++k)
        for (SparseMatrix<double>::InnerIterator it(A_eq, k); it; ++it)
            triplets.emplace_back(it.row(), it.col(), it.value());

    for (int k = 0; k < A_ineq.outerSize(); ++k)
        for (SparseMatrix<double>::InnerIterator it(A_ineq, k); it; ++it)
            triplets.emplace_back(n_eq + it.row(), it.col(), it.value());

    A_combined.setFromTriplets(triplets.begin(), triplets.end());

    // Combine lower and upper bounds
    VectorXd l_combined(n_con), u_combined(n_con);
    l_combined << b_eq, l_ineq;
    u_combined << b_eq, u_ineq;

    // Convert Eigen sparse matrices to CSC format for OSQP
    A_combined.makeCompressed();
//    H.makeCompressed();
//
//    OSQPCscMatrix* P = OSQPCscMatrix_new(
//            nz, nz,
//            H.nonZeros(),
//            H.valuePtr(),
//            H.innerIndexPtr(),
//            H.outerIndexPtr()
//    );
//
//    OSQPCscMatrix* A = OSQPCscMatrix_new(
//            n_con, nz,
//            A_combined.nonZeros(),
//            A_combined.valuePtr(),
//            A_combined.innerIndexPtr(),
//            A_combined.outerIndexPtr()
//    );

    // Gradient vector q
//    double* q = const_cast<double*>(h.data());
//    double* l = const_cast<double*>(l_combined.data());
//    double* u = const_cast<double*>(u_combined.data());
//
//    // Settings
//    OSQPSettings* settings = OSQPSettings_new();
//    osqp_set_default_settings(settings);
//    settings->alpha = 1.0;  // relaxation parameter (controls step size of dual updates)
//
//    // Solver
//    OSQPSolver* solver = nullptr;
//    OSQPInt exitflag = osqp_setup(&solver, P, q, A, l, u, n_con, nz, settings);
//
//    if (exitflag != 0) {
//        std::cerr << "OSQP setup failed with exitflag " << exitflag << std::endl;
//        return;
//    }
//
//    // Solve QP
//    exitflag = osqp_solve(solver);
//    if (exitflag != 0) {
//        std::cerr << "OSQP solve failed with exitflag " << exitflag << std::endl;
//    } else {
//        // Access solution
//        VectorXd solution = Map<VectorXd>(solver->solution->x, nz);
//        std::cout << "QP solution:\n" << solution.transpose() << std::endl;
//    }
//
//    // Clean up
//    osqp_cleanup(solver);
//    OSQPCscMatrix_free(P);
//    OSQPCscMatrix_free(A);
//    OSQPSettings_free(settings);
}


//void SCVX::SolveQP(const vector<MatrixXd>& A_k,
//                   const vector<MatrixXd>& B_k,
//                   const vector<VectorXd>& d_k,
//                   const vector<MatrixXd>& cost_hess_xx,
//                   const vector<MatrixXd>& cost_hess_uu,
//                   const vector<MatrixXd>& cost_hess_xu,
//                   const vector<VectorXd>& cost_grad_x,
//                   const vector<VectorXd>& cost_grad_u,
//                   const VectorXd& terminal_grad,
//                   const MatrixXd& terminal_hess,
//                   int n, int m, int N,
//                   double trust_box){
//
//    int nz = N * (n + m) + n;
//
//    SparseMatrix<double> H, A_eq, A_ineq;
//    VectorXd h, b_eq, l_ineq, u_ineq;
//
//    buildEqualityConstraints(A_k, B_k, d_k, n, m, N, A_eq, b_eq);
//    buildCostFunction(cost_hess_xx, cost_hess_uu, cost_hess_xu,
//                      cost_grad_x, cost_grad_u, terminal_grad, terminal_hess,
//                      n, m, N, H, h);
//    buildTrustRegion(nz, trust_box, A_ineq, l_ineq, u_ineq);
//
//    // Combine A_eq and A_ineq
//    SparseMatrix<double> A_combined(A_eq.rows() + A_ineq.rows(), nz);
//    A_combined.topRows(A_eq.rows()) = A_eq;
//    A_combined.bottomRows(A_ineq.rows()) = A_ineq;
//
//    VectorXd l_combined(b_eq.size() + l_ineq.size());
//    VectorXd u_combined(b_eq.size() + u_ineq.size());
//    l_combined << b_eq, l_ineq;
//    u_combined << b_eq, u_ineq;
//
//    // OSQP setup
//    u_int n_var = nz;
//    u_int n_con = A_combined.rows();
//
//    /* Exitflag */
//    OSQPInt exitflag = 0;
//
//    /* Solver */
//    OSQPSolver *solver;
//
//    /* Create CSC matrices that are backed by the above data arrays. */
//    OSQPCscMatrix* P = OSQPCscMatrix_new(n, n, P_nnz, P_x, P_i, P_p);
//    OSQPCscMatrix* A = OSQPCscMatrix_new(m, n, A_nnz, A_x, A_i, A_p);
//
//    /* Setup settings */
//    OSQPSettings *settings = OSQPSettings_new();
//    settings->alpha = 1.0; /* Change alpha parameter */ //TODO - who knows what this does...
//
//    /* Setup solver */
//    exitflag = osqp_setup(&solver, P, q, A, l, u, m, n, settings);
//
//    /* Solve problem */
//    if (!exitflag) exitflag = osqp_solve(solver);
//
//    /* Cleanup */
//    osqp_cleanup(solver);
//    OSQPCscMatrix_free(A);
//    OSQPCscMatrix_free(P);
//    OSQPSettings_free(settings);
//
//
//
//
//
//}

void SCVX::BuildEqualityConstraints(){

}

void SCVX::BuildCostFunction(){

}

void SCVX::BuildTrustRegion(){

}

double SCVX::ForwardsPass(double _old_cost){

}

void SCVX::UpdateNominal(){

}

void SCVX::PrintBanner(double time_rollout){
    std::cout << "--------------------------------------------------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "|                                                   SCVX begins, initial rollout took: " << std::setprecision(4) << time_rollout << "                                               |" << std::endl;

    std::cout << std::left << std::setw(12) << "| Iteration"
              << std::setw(12) << "| Old Cost"
              << std::setw(12) << "| New Cost"
              << std::setw(8)  << "| Eps"
              << std::setw(16) << "| % Derivatives"
              << std::setw(20) << "| Time Derivs (ms)"
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
              << "|" << std::setw(9) << lambda
              << "|" << std::setw(15) << percent_derivatives
              << "|" << std::setw(19) <<time_derivs
              << "|" << std::setw(14)  << time_qp
              << "|" << std::setw(14) << time_fp << "|" << std::endl;
}