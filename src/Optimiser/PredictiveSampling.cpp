#include <iomanip>
#include "Optimiser/PredictiveSampling.h"

PredictiveSampling::PredictiveSampling(std::shared_ptr<ModelTranslator> _modelTranslator,
                                       std::shared_ptr<MuJoCoHelper> MuJoCo_helper,
                                       std::shared_ptr<FileHandler> _yamlReader,
                                       std::shared_ptr<Differentiator> _differentiator,
                                       int horizon,
                                       int _num_rollouts,
                                       int _knot_spacing,
                                       SplineType _spline_type,
                                       double _noise_scale) :
        Optimiser(_modelTranslator, MuJoCo_helper, _yamlReader, _differentiator){

    num_rollouts = std::max(1, _num_rollouts);
    spline_type = _spline_type;
    noise_scale = _noise_scale;
    SetKnotSpacing(_knot_spacing);

    // Initialise saved systems state list
    if(MuJoCo_helper->CheckIfDataIndexExists(0)){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[0], MuJoCo_helper->main_data);
    }
    else{
        MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
    }

    for(int i = 0; i < horizon; i++){
        if(MuJoCo_helper->CheckIfDataIndexExists(i + 1)){
            MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[i + 1], MuJoCo_helper->main_data);
        }
        else{
            MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
        }
    }

    Resize(activeModelTranslator->current_state_vector.dof,
           activeModelTranslator->current_state_vector.num_ctrl,
           horizon);
}

void PredictiveSampling::SetKnotSpacing(int _knot_spacing){
    if(_knot_spacing <= 0){
        knot_spacing = DEFAULT_KNOT_SPACING;
    }
    else{
        knot_spacing = _knot_spacing;
    }
}

SplineType PredictiveSampling::SplineTypeFromString(const std::string &name){
    if(name == "zero_order" || name == "zero"){
        return SplineType::ZERO_ORDER;
    }
    if(name == "linear" || name == "first_order"){
        return SplineType::LINEAR;
    }
    if(name != "cubic"){
        std::cerr << "Unrecognised spline type: " << name << ", defaulting to cubic \n";
    }
    return SplineType::CUBIC;
}

void PredictiveSampling::Resize(int new_num_dofs, int new_num_ctrl, int new_horizon){
    bool update_rollouts = (static_cast<int>(rollout_data.size()) != num_rollouts);
    bool update_horizon = update_rollouts || (new_horizon != horizon_length);
    bool update_ctrl = update_horizon || (new_num_ctrl != num_ctrl);
    bool update_dof = update_horizon || (new_num_dofs != dof);

    num_ctrl = new_num_ctrl;
    dof = new_num_dofs;
    horizon_length = new_horizon;

    int num_dof_quat = activeModelTranslator->current_state_vector.dof_quat;

    if(update_dof){
        X_old.assign(horizon_length + 1, MatrixXd(num_dof_quat + dof, 1));
    }

    if(update_ctrl){
        U_old.assign(horizon_length, MatrixXd(num_ctrl, 1));
        sampled_controls.assign(num_rollouts, std::vector<MatrixXd>(horizon_length, MatrixXd(num_ctrl, 1)));
    }

    // Residuals are resized on every call (as in iLQR), as the residual list can change between optimisations
    // (e.g. task stage changes) without the horizon changing.
    int num_residuals = static_cast<int>(activeModelTranslator->residual_list.size());
    residuals.assign(horizon_length + 1, MatrixXd(num_residuals, 1));

    if(update_horizon){
        contact_list.assign(horizon_length + 1, std::vector<std::pair<int, int>>());

        mujoco_data_min data_timestep;
        data_timestep.time = 0.0;
        data_timestep.q_pos.resize(MuJoCo_helper->model->nq);
        data_timestep.q_vel.resize(MuJoCo_helper->model->nv);
        data_timestep.q_acc.resize(MuJoCo_helper->model->nv);
        data_timestep.q_acc_warmstart.resize(MuJoCo_helper->model->nv);
        data_timestep.qfrc_applied.resize(MuJoCo_helper->model->nv);
        data_timestep.xfrc_applied.resize(6*MuJoCo_helper->model->nbody);
        data_timestep.ctrl.resize(MuJoCo_helper->model->nu);
        data_timestep.residuals.resize(num_residuals, 1);

        // One rollout buffer per sample, so the best sample can be copied without re-simulating
        num_parallel_rollouts = num_rollouts;
        rollout_data.assign(num_rollouts, std::vector<mujoco_data_min>(horizon_length + 1, data_timestep));
    }
    else{
        // Keep the saved rollout residuals in sync with the current residual list (no-op if the size is unchanged)
        for(auto &rollout : rollout_data){
            for(auto &data_timestep : rollout){
                data_timestep.residuals.resize(num_residuals, 1);
            }
        }
    }

    if(update_rollouts){
        sampled_costs.assign(num_rollouts, 0.0);
        std::random_device rd;
        generators.clear();
        for(int i = 0; i < num_rollouts; i++){
            generators.emplace_back(rd());
        }
    }
}

void PredictiveSampling::ComputeKnotTimes(){
    knot_times.clear();
    for(int t = 0; t < horizon_length; t += knot_spacing){
        knot_times.push_back(t);
    }

    // Always place a knot at the final control index
    if(knot_times.back() != horizon_length - 1){
        knot_times.push_back(horizon_length - 1);
    }

    int num_knots = static_cast<int>(knot_times.size());
    nominal_knots.assign(num_knots, MatrixXd(num_ctrl, 1));
    sampled_knots.assign(num_rollouts, std::vector<MatrixXd>(num_knots, MatrixXd(num_ctrl, 1)));
}

void PredictiveSampling::InterpolateControls(const std::vector<MatrixXd> &knots, std::vector<MatrixXd> &controls) const{
    int num_knots = static_cast<int>(knot_times.size());

    // Slope (per time-step) at a knot, via finite differences of neighbouring knots
    auto tangent = [&](int k) -> MatrixXd {
        if(k == 0){
            return (knots[1] - knots[0]) / (knot_times[1] - knot_times[0]);
        }
        if(k == num_knots - 1){
            return (knots[k] - knots[k - 1]) / (knot_times[k] - knot_times[k - 1]);
        }
        return 0.5 * ((knots[k + 1] - knots[k]) / (knot_times[k + 1] - knot_times[k]) +
                      (knots[k] - knots[k - 1]) / (knot_times[k] - knot_times[k - 1]));
    };

    // Loop over all the knot points and interpolate the controls between them
    for(int k = 0; k < num_knots - 1; k++){
        int t0 = knot_times[k];
        int t1 = knot_times[k + 1];
        double h = t1 - t0;

        MatrixXd m0, m1;
        if(spline_type == SplineType::CUBIC){
            m0 = tangent(k) * h;
            m1 = tangent(k + 1) * h;
        }

        // Loop over time-steps between knot points
        for(int t = t0; t < t1; t++){
            double s = (t - t0) / h;

            switch(spline_type){
                case SplineType::ZERO_ORDER:
                    controls[t] = knots[k];
                    break;
                case SplineType::LINEAR:
                    controls[t] = (1 - s) * knots[k] + s * knots[k + 1];
                    break;
                case SplineType::CUBIC: {
                    // Cubic Hermite basis functions
                    double s2 = s * s;
                    double s3 = s2 * s;
                    double h00 = 2*s3 - 3*s2 + 1;
                    double h10 = s3 - 2*s2 + s;
                    double h01 = -2*s3 + 3*s2;
                    double h11 = s3 - s2;
                    controls[t] = h00 * knots[k] + h10 * m0 + h01 * knots[k + 1] + h11 * m1;
                    break;
                }
            }
        }
    }

    controls[knot_times.back()] = knots.back();

    // Cubic splines can overshoot the knots, so clamp everything to the control limits
    for(int t = 0; t < horizon_length; t++){
        ClampControl(controls[t]);
    }
}

void PredictiveSampling::ClampControl(MatrixXd &control) const{
    for(int i = 0; i < num_ctrl; i++){
        double low = control_limits(2*i, 0);
        double high = control_limits(2*i + 1, 0);
        // Unlimited actuators have an empty control range
        if(low >= high) continue;

        if(control(i) > high) control(i) = high;
        if(control(i) < low) control(i) = low;
    }
}

void PredictiveSampling::SampleKnots(int rollout_index){
    std::normal_distribution<double> dist(0.0, 1.0);
    std::mt19937 &generator = generators[rollout_index];

    // The first rollout is always the nominal trajectory, so no noise is added
    if(rollout_index == 0){
        for(int k = 0; k < static_cast<int>(knot_times.size()); k++){
            sampled_knots[rollout_index][k] = nominal_knots[k];
        }
        return;
    }

    for(int k = 0; k < static_cast<int>(knot_times.size()); k++){
        MatrixXd &knot = sampled_knots[rollout_index][k];
        for(int i = 0; i < num_ctrl; i++){
            knot(i) = nominal_knots[k](i) + noise_std(i) * dist(generator);
        }
        ClampControl(knot);
    }
}

double PredictiveSampling::RolloutSample(int rollout_index, int thread_id){
    double cost = 0.0;
    mjData *d = MuJoCo_helper->fd_data[thread_id];
    MatrixXd residuals_t(activeModelTranslator->residual_list.size(), 1);

    MuJoCo_helper->CopySystemState(d, MuJoCo_helper->saved_systems_state_list[0]);

    for(int t = 0; t < horizon_length; t++){
        activeModelTranslator->SetControlVector(sampled_controls[rollout_index][t], d,
                                                activeModelTranslator->current_state_vector);

        activeModelTranslator->Residuals(d, residuals_t);
        cost += activeModelTranslator->CostFunction(residuals_t, activeModelTranslator->full_state_vector, false);

        // Save state at time t (with control t applied) so the best rollout can be restored without re-simulating
        SaveSystemStateToRolloutData(d, rollout_index, t, residuals_t);

        mj_step(MuJoCo_helper->model, d);
    }

    // Terminal cost
    activeModelTranslator->Residuals(d, residuals_t);
    cost += activeModelTranslator->CostFunction(residuals_t, activeModelTranslator->full_state_vector, true);
    SaveSystemStateToRolloutData(d, rollout_index, horizon_length, residuals_t);

    return cost;
}

void PredictiveSampling::WorkerRolloutSamples(int thread_id){
    while(true){
        int rollout_index = next_rollout.fetch_add(1);
        if(rollout_index >= num_rollouts){
            break;
        }

        SampleKnots(rollout_index);
        InterpolateControls(sampled_knots[rollout_index], sampled_controls[rollout_index]);
        sampled_costs[rollout_index] = RolloutSample(rollout_index, thread_id);
    }
}

double PredictiveSampling::RolloutTrajectory(mjData *d, bool save_states, std::vector<MatrixXd> initial_controls){
    double cost = 0.0;

    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, d);

    if(MuJoCo_helper->CheckIfDataIndexExists(0)){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[0], MuJoCo_helper->main_data);
    }
    else{
        MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
    }

    for(int t = 0; t < horizon_length; t++){
        activeModelTranslator->SetControlVector(initial_controls[t], MuJoCo_helper->main_data,
                                                activeModelTranslator->current_state_vector);

        if(save_states){
            X_old[t] = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->main_data,
                                                                           activeModelTranslator->current_state_vector);
            U_old[t] = activeModelTranslator->ReturnControlVector(MuJoCo_helper->main_data,
                                                                  activeModelTranslator->current_state_vector);
            if(MuJoCo_helper->CheckIfDataIndexExists(t)){
                MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[t], MuJoCo_helper->main_data);
            }
            else{
                MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
            }
            activeModelTranslator->GetContacts(MuJoCo_helper->main_data, contact_list[t]);
        }

        activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[t]);
        cost += activeModelTranslator->CostFunction(residuals[t], activeModelTranslator->full_state_vector, false);

        mj_step(MuJoCo_helper->model, MuJoCo_helper->main_data);
    }

    activeModelTranslator->Residuals(MuJoCo_helper->main_data, residuals[horizon_length]);
    cost += activeModelTranslator->CostFunction(residuals[horizon_length], activeModelTranslator->full_state_vector, true);

    if(save_states){
        X_old[horizon_length] = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->main_data,
                                                                                    activeModelTranslator->current_state_vector);
        if(MuJoCo_helper->CheckIfDataIndexExists(horizon_length)){
            MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[horizon_length], MuJoCo_helper->main_data);
        }
        else{
            MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
        }
        activeModelTranslator->GetContacts(MuJoCo_helper->main_data, contact_list[horizon_length]);
    }

    return cost;
}

std::vector<MatrixXd> PredictiveSampling::Optimise(mjData *d, std::vector<MatrixXd> initial_controls, int max_iterations, int min_iterations, int horizon_length){
    auto opt_start = high_resolution_clock::now();

    // Resize internal matrices if required
    Resize(activeModelTranslator->current_state_vector.dof,
           activeModelTranslator->current_state_vector.num_ctrl,
           horizon_length);

    ComputeKnotTimes();

    // Sampling noise is scaled by each actuator's control range
    control_limits = activeModelTranslator->ReturnControlLimits(activeModelTranslator->current_state_vector);
    noise_std.resize(num_ctrl, 1);
    for(int i = 0; i < num_ctrl; i++){
        double half_range = 0.5 * (control_limits(2*i + 1, 0) - control_limits(2*i, 0));
        noise_std(i) = (half_range > 0) ? noise_scale * half_range : noise_scale;
    }

    // ---------------------- Clear data saving variables ----------------------
    opt_time_ms = 0.0;
    avg_time_get_derivs_ms = 0.0;
    avg_time_backwards_pass_ms = 0.0;
    avg_time_forwards_pass_ms = 0.0;
    avg_percent_derivs = 0.0;
    num_iterations = 0;

    cost_history.clear();
    time_forwardsPass_ms.clear();
    cost_after_iteration.clear();
    cost_reduction_after_iteration.clear();
    time_after_iteration_ms.clear();
    // ------------------------------------------------------------------------

    auto time_start = high_resolution_clock::now();

    // Optimise from the provided start state d. All sampled rollouts start from saved_systems_state_list[0],
    // so it must be set to d here (otherwise it holds the start state of the previous optimisation).
    if(d != MuJoCo_helper->saved_systems_state_list[0]){
        MuJoCo_helper->CopySystemState(MuJoCo_helper->saved_systems_state_list[0], d);
    }
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    std::vector<MatrixXd> best_controls(initial_controls.begin(), initial_controls.begin() + horizon_length);
    for(int k = 0; k < static_cast<int>(knot_times.size()); k++){
        nominal_knots[k] = best_controls[knot_times[k]];
        ClampControl(nominal_knots[k]);
    }

    // ------------------- Main optimisation iteration loop ------------------------
    int num_threads = std::min(num_rollouts, static_cast<int>(MuJoCo_helper->fd_data.size()));
    for(int i = 0; i < max_iterations; i++){
        num_iterations++;
        auto iteration_start = high_resolution_clock::now();

        // Sample and rollout noisy knot sequences in parallel
        next_rollout = 0;
        std::vector<std::thread> thread_pool;
        for(int j = 0; j < num_threads; j++){
            thread_pool.emplace_back(&PredictiveSampling::WorkerRolloutSamples, this, j);
        }
        for(std::thread &thread : thread_pool){
            thread.join();
        }

        int best_index = static_cast<int>(std::min_element(sampled_costs.begin(), sampled_costs.end()) - sampled_costs.begin());
        int best_sample_index = static_cast<int>(std::min_element(sampled_costs.begin() + 1, sampled_costs.end()) - sampled_costs.begin());
        double best_cost = sampled_costs[best_index];
        double best_sample_cost = sampled_costs[best_sample_index];
        if(i == 0){
            initial_cost = sampled_costs[0];    // Index 0 is always unperturbed nominal rollout
            old_cost = initial_cost;
            new_cost = initial_cost;
            if(verbose_output){
                PrintBanner();
            }
        }


        // Only update the nominal if a sample improved on it. On the first iteration we always save the best
        // rollout (which may be index 0, the unperturbed nominal), so X_old / U_old / saved states always
        // describe this optimisation's trajectory rather than being left over from a previous call.
        if(best_cost < new_cost || i == 0){
            new_cost = best_cost;
            nominal_knots = sampled_knots[best_index];
            best_controls = sampled_controls[best_index];

            SaveBestRollout(best_index);
            for(int t = 0; t < horizon_length; t++){
                U_old[t] = best_controls[t];
            }
            for(int t = 0; t <= horizon_length; t++){
                X_old[t] = activeModelTranslator->ReturnStateVectorQuaternions(MuJoCo_helper->saved_systems_state_list[t],
                                                                               activeModelTranslator->current_state_vector);
            }
        }
//        else{
//            new_cost = old_cost;
//        }

        double time_iteration = duration_cast<microseconds>(high_resolution_clock::now() - iteration_start).count() / 1000.0;
        time_forwardsPass_ms.push_back(time_iteration);

        if(verbose_output){
            PrintBannerIteration(i, new_cost, best_sample_cost, time_iteration);
        }

        bool converged = CheckForConvergence(old_cost, new_cost);
        old_cost = new_cost;

        cost_history.push_back(new_cost);
        cost_after_iteration.push_back(new_cost);
        cost_reduction_after_iteration.push_back(1 - (new_cost / initial_cost));
        time_after_iteration_ms.push_back(duration_cast<microseconds>(high_resolution_clock::now() - opt_start).count() / 1000.0);

        if(converged && (i >= min_iterations)){
            break;
        }
    }

    // --------------------  Computing testing results ---------------------------
    cost_reduction = 1 - (new_cost / initial_cost);
    opt_time_ms = duration_cast<microseconds>(high_resolution_clock::now() - opt_start).count() / 1000.0;

    for(double time_fp : time_forwardsPass_ms){
        avg_time_forwards_pass_ms += time_fp;
    }
    if(!time_forwardsPass_ms.empty()){
        avg_time_forwards_pass_ms /= static_cast<int>(time_forwardsPass_ms.size());
    }

    if(verbose_output){
        std::cout << " --------------------------------------------------- optimisation complete, took: " << opt_time_ms << " ms --------------------------------------------------" << std::endl;
    }

    // Load the initial data back into main data
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, MuJoCo_helper->saved_systems_state_list[0]);

    return best_controls;
}

void PredictiveSampling::PrintBanner(){
    std::string spline_name = (spline_type == SplineType::ZERO_ORDER) ? "zero order" :
                              (spline_type == SplineType::LINEAR) ? "linear" : "cubic";

    std::cout << "------------------------------------------------------------------------------------" << std::endl;
    std::cout << "| Predictive sampling begins, initial rollout cost: " << std::setprecision(4) << initial_cost << " " << std::endl;
    std::cout << "| rollouts: " << num_rollouts << ", knots: " << knot_times.size() << " (spacing " << knot_spacing
              << "), spline: " << spline_name << ", noise scale: " << noise_scale << std::endl;

    std::cout << std::left << std::setw(12) << "| Iteration"
              << std::setw(16) << "| Best Cost"
              << std::setw(20) << "| Best Sample Cost"
              << std::setw(16) << "| Time (ms)" << " |" << std::endl;
}

void PredictiveSampling::PrintBannerIteration(int iteration, double best_cost, double best_sample_cost, double time_iteration){
    std::cout << std::left << "|" << std::setw(11) << iteration
              << "|" << std::setw(15) << best_cost
              << "|" << std::setw(19) << best_sample_cost
              << "|" << std::setw(15) << time_iteration << " |" << std::endl;
}
