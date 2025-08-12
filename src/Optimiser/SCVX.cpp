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

    // QP related things
    int N = this->horizon_length;
    int n_u = this->num_ctrl;
    int n_x = 2 * this->dof;      // matches your l_x / l_xx sizing
    int n_z = N * n_u + N * n_x;
    int n_eq = N * n_x;

    qp_H.resize(n_z, n_z);
    qp_h = Eigen::VectorXd::Zero(n_z);

    qp_Aeq.resize(n_eq, n_z);
    qp_beq = Eigen::VectorXd::Zero(n_eq);

    qp_Aineq.resize(0, n_z); // empty by default
    qp_lineq.resize(0);
    qp_uineq.resize(0);

    qp_dz = Eigen::VectorXd::Zero(n_z);
    qp_candidate_controls.clear();
    qp_candidate_controls.resize(N);

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
    SolveQP();
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

// Helper to copy Eigen sparse -> CSC arrays (values + c_int indices)
static void EigenSparseToCSC(const Eigen::SparseMatrix<double> &M,
                             double *&x_out,
                             OSQPInt *&i_out,
                             OSQPInt *&p_out,
                             OSQPInt &nnz_out) {
    Eigen::SparseMatrix<double> Mc = M;
    Mc.makeCompressed();
    nnz_out = static_cast<OSQPInt>(Mc.nonZeros());
    p_out = (OSQPInt*)c_malloc(sizeof(OSQPInt) * (Mc.outerSize() + 1)); // outerSize()==cols for column-major
    i_out = (OSQPInt*)c_malloc(sizeof(OSQPInt) * nnz_out);
    x_out = (double*)c_malloc(sizeof(double) * nnz_out);

    // copy outer ptr (col pointers)
    for (int col = 0; col <= Mc.outerSize(); ++col) {
        p_out[col] = static_cast<OSQPInt>(Mc.outerIndexPtr()[col]);
    }
    // copy inner indices and values
    for (int k = 0; k < nnz_out; ++k) {
        i_out[k] = static_cast<OSQPInt>(Mc.innerIndexPtr()[k]); // row indices
        x_out[k] = Mc.valuePtr()[k];
    }
}

void SCVX::SetDynamicsConstraints(Eigen::SparseMatrix<double>& linear_matrix){
    // Use A and B matrices that have already been computed
    int N = horizon_length;
    int n_x = 2 * dof; // number of state variables
    int n_u = num_ctrl; // number of control variables
    int n_z = N * n_u + N * n_x; // total number of variables

    int n_eq = N * n_x; // number of equality constraints
    linear_matrix.resize(n_eq, n_z);

    for(int t = 0; t < horizon_length; t++){
        
    }
}

void SCVX::SetTrustRegionConstraints(Eigen::SparseMatrix<double>& linear_matrix){

}

void SCVX::SetCostFunction(Eigen::SparseMatrix<double>& hessian_matrix, Eigen::VectorXd& gradient_vector){

}

void SCVX::SolveQP() {

    OsqpEigen::Solver solver;

    // settings
    // solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // Setup the QP problem
    // allocate QP problem matrices and vectors
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> linear_matrix;
    Eigen::VectorXd lower_bound;
    Eigen::VectorXd upper_bound;

    //Formulate QP matrices
    SetDynamicsConstraints(linear_matrix);


    // ----------- Set QP matrices --------------
//    solver.data()->setNumberOfVariables();
//    solver.data()->setNumberOfConstraints();

    if(!solver.data()->setHessianMatrix(hessian)){
        std::cerr << "Failed to set Hessian matrix." << std::endl;
        return;
    }
    if (!solver.data()->setGradient(gradient)){
        std::cerr << "Failed to set gradient matrix." << std::endl;
        return;
    }
    if (!solver.data()->setLinearConstraintsMatrix(linear_matrix)){
        std::cerr << "Failed to set linear constraints matrix." << std::endl;
        return;
    }
    if (!solver.data()->setLowerBound(lower_bound)){
        std::cerr << "Failed to set lower bound." << std::endl;
        return;
    }
    if (!solver.data()->setUpperBound(upper_bound)){
        std::cerr << "Failed to set upper bound." << std::endl;
        return;
    }




    // solve the QP problem
    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        std::cerr << "OSQP failed to solve the problem." << std::endl;
        return;
    }


    // Build QP blocks (fill qp_H, qp_h, qp_Aeq, qp_beq)
//    BuildEqualityConstraints(qp_Aeq, qp_beq);
//    BuildCostFunction(qp_H, qp_h);
//
//    // Build trust region / inequalities if desired
//    // Example: use BuildTrustRegion to make qp_Aineq/l/u of size n_z
//    double trust_box = lambda;
//    if (trust_box <= 0) trust_box = 1.0;
//    BuildTrustRegion(qp_Aineq, qp_lineq, qp_uineq, trust_box);
//
//    // Combine equality and inequality matrices into A_combined
//    int n_z = static_cast<int>(qp_H.rows());
//    int n_eq = static_cast<int>(qp_Aeq.rows());
//    int n_ineq = static_cast<int>(qp_Aineq.rows());
//    int n_con = n_eq + n_ineq;
//
//    Eigen::SparseMatrix<double> A_combined(n_con, n_z);
//    std::vector<Eigen::Triplet<double>> trips;
//    trips.reserve(qp_Aeq.nonZeros() + qp_Aineq.nonZeros());
//
//    // copy Aeq triplets
//    for (int k = 0; k < qp_Aeq.outerSize(); ++k)
//        for (Eigen::SparseMatrix<double>::InnerIterator it(qp_Aeq, k); it; ++it)
//            trips.emplace_back(it.row(), it.col(), it.value());
//
//    // copy Aineq triplets, shifted by n_eq rows
//    for (int k = 0; k < qp_Aineq.outerSize(); ++k)
//        for (Eigen::SparseMatrix<double>::InnerIterator it(qp_Aineq, k); it; ++it)
//            trips.emplace_back(n_eq + it.row(), it.col(), it.value());
//
//    A_combined.setFromTriplets(trips.begin(), trips.end());
//    A_combined.makeCompressed();
//
//    // Build l and u
//    Eigen::VectorXd l_combined(n_con), u_combined(n_con);
//    if (n_eq > 0) {
//        l_combined.segment(0, n_eq) = qp_beq;
//        u_combined.segment(0, n_eq) = qp_beq;
//    }
//    if (n_ineq > 0) {
//        l_combined.segment(n_eq, n_ineq) = qp_lineq;
//        u_combined.segment(n_eq, n_ineq) = qp_uineq;
//    }
//
//    // Ensure qp_H is compressed
//    qp_H.makeCompressed();
//
//    // Convert Eigen sparse P (qp_H) -> CSC arrays
//    double *P_x = nullptr, *A_x = nullptr;
//    OSQPInt *P_i = nullptr, *P_p = nullptr;
//    OSQPInt *A_i = nullptr, *A_p = nullptr;
//    OSQPInt P_nnz = 0, A_nnz = 0;
//
//    EigenSparseToCSC(qp_H, P_x, P_i, P_p, P_nnz);
//    EigenSparseToCSC(A_combined, A_x, A_i, A_p, A_nnz);
//
//    // Copy q, l, u into C arrays (OSQP expects c_float* which is double in default builds)
//    OSQPFloat *q = (OSQPFloat *) c_malloc(sizeof(OSQPFloat) * n_z);
//    OSQPFloat *l = (OSQPFloat *) c_malloc(sizeof(OSQPFloat) * n_con);
//    OSQPFloat *u = (OSQPFloat *) c_malloc(sizeof(OSQPFloat) * n_con);
//    for (int i = 0; i < n_z; ++i) q[i] = static_cast<OSQPFloat>(qp_h(i));
//    for (int i = 0; i < n_con; ++i) {
//        l[i] = static_cast<c_float>(l_combined(i));
//        u[i] = static_cast<c_float>(u_combined(i));
//    }
//
//    // Build csc matrices with the OSQP-provided csc_matrix constructor
//    csc *P_csc = csc_matrix(n_z, n_z, P_nnz, P_x, P_i, P_p);
//    csc *A_csc = csc_matrix(n_con, n_z, A_nnz, A_x, A_i, A_p);
//
//    // Setup OSQP data
//    OSQPData *data = (OSQPData *) c_malloc(sizeof(OSQPData));
//    data->n = n_z;
//    data->m = n_con;
//    data->P = P_csc;
//    data->q = q;
//    data->A = A_csc;
//    data->l = l;
//    data->u = u;
//
//    // Settings
//    OSQPSettings *settings = (OSQPSettings *) c_malloc(sizeof(OSQPSettings));
//    osqp_set_default_settings(settings);
//    settings->alpha = 1.0;
//    settings->verbose = osqp_verbose;
//    settings->max_iter = osqp_max_iter;
//    settings->eps_abs = osqp_eps_abs;
//    settings->eps_rel = osqp_eps_rel;
//
//    // Workspace
//    OSQPWorkspace *work = nullptr;
//    c_int exitflag = osqp_setup(&work, data, settings);
//    if (exitflag != 0 || work == nullptr) {
//        std::cerr << "OSQP setup failed with exitflag " << exitflag << std::endl;
//        // cleanup partial allocations
//        if (work) osqp_cleanup(work);
//        c_free(q);
//        c_free(l);
//        c_free(u);
//        // free csc arrays (they point to P_x, etc.)
//        // free P_i/P_p/A_i/A_p and the x arrays allocated earlier
//        c_free(P_i);
//        c_free(P_p);
//        c_free(A_i);
//        c_free(A_p);
//        c_free(P_x);
//        c_free(A_x);
//        c_free(data);
//        c_free(settings);
//        return;
//    }
//
//    // Solve
//    osqp_solve(work);
//
//    // Read solution
//    if (work->info->status_val == OSQP_SOLVED || work->info->status_val == OSQP_SOLVED_INACCURATE) {
//        qp_dz = Eigen::VectorXd::Zero(n_z);
//        for (int i = 0; i < n_z; ++i) {
//            qp_dz(i) = static_cast<double>(work->solution->x[i]);
//        }
//
//        // Extract candidate controls u_k = U_old[k] + delta_u_k
//        int N = horizon_length;
//        int n_u_local = num_ctrl;
//        qp_candidate_controls.clear();
//        qp_candidate_controls.resize(N);
//        for (int k = 0; k < N; ++k) {
//            Eigen::VectorXd delta_u = qp_dz.segment(k * n_u_local, n_u_local);
//            qp_candidate_controls[k] = U_old[k] + delta_u;
//        }
//
//        if (verbose_output) {
//            std::cout << "[SolveQP] OSQP solved, dz norm = " << qp_dz.norm() << std::endl;
//        }
//    } else {
//        std::cerr << "OSQP did not find a solution. status_val = " << work->info->status_val << std::endl;
//    }
//
//    // Cleanup OSQP
//    osqp_cleanup(work);
//    // Free memory we allocated
//    c_free(q);
//    c_free(l);
//    c_free(u);
//    // csc struct will not free arrays; we allocated them so free now:
//    c_free(P_i);
//    c_free(P_p);
//    c_free(A_i);
//    c_free(A_p);
//    c_free(P_x);
//    c_free(A_x);
//    c_free(data);
//    c_free(settings);
}

//void SCVX::SolveQP(
//        const vector<MatrixXd>& A_k,
//        const vector<MatrixXd>& B_k,
//        const vector<VectorXd>& d_k,
//        const vector<MatrixXd>& cost_hess_xx,
//        const vector<MatrixXd>& cost_hess_uu,
//        const vector<MatrixXd>& cost_hess_xu,
//        const vector<VectorXd>& cost_grad_x,
//        const vector<VectorXd>& cost_grad_u,
//        const VectorXd& terminal_grad,
//        const MatrixXd& terminal_hess,
//        int n, int m, int N,
//        double trust_box)
//{
//    int nz = qp_H.rows();  // number of decision variables
//    int n_eq = qp_Aeq.rows();
//    int n_ineq = qp_Aineq.rows();
//    int n_con = n_eq + n_ineq;
//
//    // Combine A_eq and A_ineq into one matrix A
//    SparseMatrix<double> A_combined(n_con, nz);
//    typedef Triplet<double> T;
//    vector<T> triplets;
//
//    for (int k = 0; k < qp_Aeq.outerSize(); ++k)
//        for (SparseMatrix<double>::InnerIterator it(qp_Aeq, k); it; ++it)
//            triplets.emplace_back(it.row(), it.col(), it.value());
//
//    for (int k = 0; k < qp_Aineq.outerSize(); ++k)
//        for (SparseMatrix<double>::InnerIterator it(qp_Aineq, k); it; ++it)
//            triplets.emplace_back(n_eq + it.row(), it.col(), it.value());
//
//    A_combined.setFromTriplets(triplets.begin(), triplets.end());
//
//    // Combine lower and upper bounds
//    VectorXd l_combined(n_con), u_combined(n_con);
//    l_combined << b_eq, l_ineq;
//    u_combined << b_eq, u_ineq;
//
//    // Convert Eigen sparse matrices to CSC format for OSQP
//    A_combined.makeCompressed();
////    H.makeCompressed();
////
////    OSQPCscMatrix* P = OSQPCscMatrix_new(
////            nz, nz,
////            H.nonZeros(),
////            H.valuePtr(),
////            H.innerIndexPtr(),
////            H.outerIndexPtr()
////    );
////
////    OSQPCscMatrix* A = OSQPCscMatrix_new(
////            n_con, nz,
////            A_combined.nonZeros(),
////            A_combined.valuePtr(),
////            A_combined.innerIndexPtr(),
////            A_combined.outerIndexPtr()
////    );
//
//    // Gradient vector q
////    double* q = const_cast<double*>(h.data());
////    double* l = const_cast<double*>(l_combined.data());
////    double* u = const_cast<double*>(u_combined.data());
////
////    // Settings
////    OSQPSettings* settings = OSQPSettings_new();
////    osqp_set_default_settings(settings);
////    settings->alpha = 1.0;  // relaxation parameter (controls step size of dual updates)
////
////    // Solver
////    OSQPSolver* solver = nullptr;
////    OSQPInt exitflag = osqp_setup(&solver, P, q, A, l, u, n_con, nz, settings);
////
////    if (exitflag != 0) {
////        std::cerr << "OSQP setup failed with exitflag " << exitflag << std::endl;
////        return;
////    }
////
////    // Solve QP
////    exitflag = osqp_solve(solver);
////    if (exitflag != 0) {
////        std::cerr << "OSQP solve failed with exitflag " << exitflag << std::endl;
////    } else {
////        // Access solution
////        VectorXd solution = Map<VectorXd>(solver->solution->x, nz);
////        std::cout << "QP solution:\n" << solution.transpose() << std::endl;
////    }
////
////    // Clean up
////    osqp_cleanup(solver);
////    OSQPCscMatrix_free(P);
////    OSQPCscMatrix_free(A);
////    OSQPSettings_free(settings);
//}


void SCVX::BuildEqualityConstraints(SparseMatrix<double> &A_eq, VectorXd &b_eq) {
    // Build A_eq * dz = b_eq for linearised dynamics.
    // Decision ordering: [u0...u_{N-1}, x1...xN]
    const int N = horizon_length;
    const int n_u = num_ctrl;
    const int n_x = 2 * dof; // matches your l_x/l_xx sizing
    const int z_size = N * n_u + N * n_x;

    const int eq_rows = N * n_x;
    A_eq.resize(eq_rows, z_size);
    b_eq = VectorXd::Zero(eq_rows);

    std::vector<Triplet<double>> trips;
    trips.reserve(eq_rows * (n_u + 2*n_x)); // rough reserve

    auto u_idx = [&](int k){ return k * n_u; };               // k in [0..N-1]
    auto x_idx = [&](int k){ return N * n_u + (k-1) * n_x; }; // k in [1..N]

    // Build d_k as the affine offset: d_k = x_{k+1}_nom - A_k * x_k_nom - B_k * u_k_nom
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < n_x; ++i) {
            int row = k * n_x + i;

            // +1 * x_{k+1} (x_{k+1} is in decision vector, x1..xN)
            int col_xkp1 = x_idx(k+1) + i;
            trips.emplace_back(row, col_xkp1, 1.0);

            // -A_k * x_k -> if k == 0, x0 is known and moved to RHS
            if (k >= 1) {
                for (int j = 0; j < n_x; ++j) {
                    double val = -A[k].coeff(i, j);
                    if (val != 0.0) trips.emplace_back(row, x_idx(k) + j, val);
                }
            } else {
                // x0 known -> move -A0 * x0 to RHS (we will add to b_eq)
            }

            // -B_k * u_k (u_k is in decision vector)
            for (int j = 0; j < n_u; ++j) {
                double val = -B[k].coeff(i, j);
                if (val != 0.0) trips.emplace_back(row, u_idx(k) + j, val);
            }

            // Build RHS: d_k = x_{k+1}_nom - A_k * x_k_nom - B_k * u_k_nom
            // We use X_old and U_old as the nominal trajectories.
            VectorXd xkp1_nom = X_old[k+1].topRows(n_x); // note: X_old stores num_dof_quat + num_dof
            VectorXd xk_nom = X_old[k].topRows(n_x);
            VectorXd uk_nom = U_old[k];
            VectorXd Ax = A[k] * xk_nom;
            VectorXd Bu = B[k] * uk_nom;
            double rhs_i = xkp1_nom(i) - Ax(i) - Bu(i);
            // Add A0*x0 term implicitly included above via xkp1_nom - A*xk_nom - B*uk_nom
            b_eq(row) = rhs_i;
        }
    }

    A_eq.setFromTriplets(trips.begin(), trips.end());
    A_eq.makeCompressed();
}

void SCVX::BuildCostFunction(SparseMatrix<double> &H, VectorXd &h, double reg_diag = 1e-6) {
    // Build Hessian H and gradient h for QP in decision vector ordering:
    // z = [u0..u_{N-1}, x1..xN]
    const int N = horizon_length;
    const int n_u = num_ctrl;
    const int n_x = 2 * dof;
    const int z_size = N * n_u + N * n_x;

    H.resize(z_size, z_size);
    h = VectorXd::Zero(z_size);

    std::vector<Triplet<double>> H_trips;
    H_trips.reserve(N * (n_u*n_u + n_x*n_x) * 2);

    auto u_idx = [&](int k){ return k * n_u; };               // k in [0..N-1]
    auto x_idx = [&](int k){ return N * n_u + (k-1) * n_x; }; // k in [1..N]

    // Stage costs k = 0..N-1
    for (int k = 0; k < N; ++k) {
        // L_uu: add to H at u block
        const MatrixXd &Luu = l_uu[k];
        for (int i = 0; i < n_u; ++i) {
            for (int j = 0; j < n_u; ++j) {
                double v = Luu(i, j);
                if (v != 0.0) H_trips.emplace_back(u_idx(k) + i, u_idx(k) + j, v);
            }
        }

        // L_xx: stage cost w.r.t x_k. Note: x0 is not in decision vector => only k>=1 contribute to H
        const MatrixXd &Lxx = l_xx[k];
        if (k >= 1) {
            for (int i = 0; i < n_x; ++i) {
                for (int j = 0; j < n_x; ++j) {
                    double v = Lxx(i, j);
                    if (v != 0.0) H_trips.emplace_back(x_idx(k) + i, x_idx(k) + j, v);
                }
            }
        } else {
            // k == 0: cost w.r.t x0 (nominal) contributes to constants; skip for H
        }

        // gradients: we assume l_u and l_x are gradients in delta coordinates.
        h.segment(u_idx(k), n_u) += l_u[k];
        if (k >= 1) h.segment(x_idx(k), n_x) += l_x[k];
        else {
            // k == 0: l_x[0] is gradient wrt x0 which is not a decision variable; so ignore here.
        }
    }

    // Terminal cost at x_N (index N)
    const MatrixXd &LxxN = l_xx[N]; // l_xx sized N+1
    for (int i = 0; i < n_x; ++i)
        for (int j = 0; j < n_x; ++j) {
            double v = LxxN(i, j);
            if (v != 0.0) H_trips.emplace_back(x_idx(N) + i, x_idx(N) + j, v);
        }
    h.segment(x_idx(N), n_x) += l_x[N];

    // Assemble sparse H
    H.setFromTriplets(H_trips.begin(), H_trips.end());

    // Regularise H diagonal to ensure positive-definiteness (small),
    // and add any additional regularisation you want here.
    // We do this by adding reg_diag to the diagonal entries.
    // If a diagonal entry already exists it will be incremented; otherwise we create it.
    std::vector<Triplet<double>> diag_trips;
    diag_trips.reserve(z_size);
    for (int i = 0; i < z_size; ++i) {
        diag_trips.emplace_back(i, i, reg_diag);
    }
    SparseMatrix<double> reg(z_size, z_size);
    reg.setFromTriplets(diag_trips.begin(), diag_trips.end());
    H += reg;

    H.makeCompressed();
}

void SCVX::BuildTrustRegion(SparseMatrix<double> &A_ineq, VectorXd &l_ineq, VectorXd &u_ineq, double trust_box) {
    // Build simple box trust region: -trust_box <= delta_z_i <= trust_box
    // Implemented as 2* z_size rows in A_ineq (one per bound), but we can instead use A = I and l,u = [-trust, trust]
    const int N = horizon_length;
    const int n_u = num_ctrl;
    const int n_x = 2 * dof;
    const int z_size = N * n_u + N * n_x;

    // A_ineq will be identity selecting each z variable
    A_ineq.resize(z_size, z_size);
    std::vector<Triplet<double>> trips;
    trips.reserve(z_size);
    for (int i = 0; i < z_size; ++i) trips.emplace_back(i, i, 1.0);
    A_ineq.setFromTriplets(trips.begin(), trips.end());
    A_ineq.makeCompressed();

    l_ineq = VectorXd::Constant(z_size, -trust_box);
    u_ineq = VectorXd::Constant(z_size, trust_box);
}

//double SCVX::ForwardsPass(double _old_cost) {
//    // Build QP, solve it (unconstrained solve here), apply deltas to controls,
//    // evaluate the nonlinear rollout and if better, accept (save states).
//    const int N = horizon_length;
//    const int n_u = num_ctrl;
//    const int n_x = 2 * dof;
//    const int z_size = N * n_u + N * n_x;
//
//    // 1) Build equality constraints (not used by unconstrained solve, but kept for later)
//    SparseMatrix<double> A_eq;
//    VectorXd b_eq;
//    BuildEqualityConstraints(A_eq, b_eq);
//
//    // 2) Build cost
//    SparseMatrix<double> H;
//    VectorXd h;
//    const double regularisation = 1e-6;
//    BuildCostFunction(H, h, regularisation);
//
//    // 3) Build trust region
//    SparseMatrix<double> A_ineq;
//    VectorXd l_ineq, u_ineq;
//    double trust_box = lambda; // you already have lambda variable — use as trust radius, or set externally.
//    if (trust_box <= 0) trust_box = 1.0; // fallback
//    BuildTrustRegion(A_ineq, l_ineq, u_ineq, trust_box);
//
//    // 4) Solve QP
//    // For now: solve unconstrained quadratic subproblem: minimize 1/2 dz' H dz + h' dz
//    // => solve H dz = -h
//    VectorXd dz;
//    bool solved = false;
//
//    // Convert H to a dense/sparse factorisation and solve robustly
//    // Try sparse LDLT first
//    try {
//        Eigen::SimplicialLDLT<SparseMatrix<double>> ldlt;
//        ldlt.compute(H);
//        if (ldlt.info() == Eigen::Success) {
//            dz = ldlt.solve(-h);
//            if (ldlt.info() == Eigen::Success) solved = true;
//        }
//    } catch (...) {
//        solved = false;
//    }
//
//    if (!solved) {
//        // fall back to dense regularised solve
//        MatrixXd Hdense = MatrixXd(H);
//        for (int i = 0; i < Hdense.rows(); ++i) Hdense(i,i) += 1e-8;
//        Eigen::LDLT<MatrixXd> ld(dense_cast<MatrixXd>(Hdense));
//        dz = ld.solve(-h);
//    }
//
//    // 5) Map dz to delta_u and form candidate controls
//    vector<MatrixXd> candidate_controls(N);
//    for (int k = 0; k < N; ++k) {
//        VectorXd delta_u = dz.segment(k * n_u, n_u);
//        candidate_controls[k] = U_old[k] + delta_u;
//    }
//
//    // 6) Evaluate nonlinear cost (rollout) without saving states
//    double candidate_cost = RolloutTrajectory(MuJoCo_helper->main_data, false, candidate_controls);
//
//    // store new cost candidate, but only accept/commit later
//    non_linear_cost = candidate_cost;
//
//    if (candidate_cost < _old_cost) {
//        // Accept: do second rollout that saves states and writes U_old/X_old
//        double saved = RolloutTrajectory(MuJoCo_helper->main_data, true, candidate_controls);
//        // saved should equal candidate_cost unless simulator non-deterministic
//        new_cost = saved;
//        return new_cost;
//    } else {
//        // Reject: do not change nominals
//        new_cost = _old_cost; // keep previous nominal cost
//        return non_linear_cost;
//    }
//}

void SCVX::UpdateNominal() {
    // If a saved rollout already accepted (we called RolloutTrajectory(save_states=true) in ForwardsPass),
    // then X_old and U_old were updated already and we consider them as the new nominal.
    // Here we can perform any additional book-keeping if required (e.g. shrinking/expanding trust region).
    // For now this is effectively a no-op because ForwardsPass saved the new trajectory when accepted.
    // You could copy X_old -> X_new or similar if you maintain separate buffers.
    // Example (no-op):
    if (verbose_output) {
        std::cout << "Nominal updated. new_cost = " << new_cost << ", lambda/trust = " << lambda << std::endl;
    }
}

//double SCVX::ForwardsPass(double _old_cost){
//
//}
//
//void SCVX::UpdateNominal(){
//
//}

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