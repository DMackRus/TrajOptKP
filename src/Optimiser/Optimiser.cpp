
#include "Optimiser/Optimiser.h"

Optimiser::Optimiser(std::shared_ptr<ModelTranslator> _modelTranslator,
                     std::shared_ptr<MuJoCoHelper> _MuJoCo_helper,
                     std::shared_ptr<FileHandler> _yamlReader,
                     std::shared_ptr<Differentiator> _differentiator){
    activeModelTranslator = _modelTranslator;
    MuJoCo_helper = _MuJoCo_helper;
    activeYamlReader = _yamlReader;
    activeDifferentiator = _differentiator;

    // Set up the derivative interpolator from YAML settings
    activeKeyPointMethod.name = activeModelTranslator->keypoint_method;
    activeKeyPointMethod.auto_adjust = activeModelTranslator->auto_adjust;
    activeKeyPointMethod.min_N = activeModelTranslator->min_N;
    activeKeyPointMethod.max_N = activeModelTranslator->max_N;
    activeKeyPointMethod.jerk_thresholds = activeModelTranslator->jerk_thresholds;
    // TODO - fix this - add acell thresholds to yaml
    activeKeyPointMethod.accell_thresholds = activeModelTranslator->jerk_thresholds;
    activeKeyPointMethod.iterative_error_threshold = activeModelTranslator->iterative_error_threshold;
    activeKeyPointMethod.velocity_change_thresholds = activeModelTranslator->velocity_change_thresholds;

    keypoint_generator = std::make_shared<KeypointGenerator>(activeDifferentiator,
                                                             MuJoCo_helper,
                                                             activeModelTranslator->current_state_vector.dof,
                                                             0);

    keypoint_generator->SetKeypointMethod(activeKeyPointMethod);
    keypoint_generator->PrintKeypointMethod();
}

bool Optimiser::CheckForConvergence(double old_cost, double new_cost){
    double costGrad = (old_cost - new_cost) / new_cost;

    if(costGrad < epsConverge){
        return true;
    }
    return false;
}

keypoint_method Optimiser::ReturnCurrentKeypointMethod(){
    return keypoint_generator->ReturnCurrentKeypointMethod();
}

void Optimiser::SetCurrentKeypointMethod(keypoint_method _keypoint_method){
    activeKeyPointMethod = _keypoint_method;
    keypoint_generator->SetKeypointMethod(_keypoint_method);
}

void Optimiser::SmoothDerivativesAtContact(int smoothing){
    // Get the contact list (this is hard coded for toy piston contact example)

    std::vector<bool> contact_list;
    for(int t = 0; t < horizon_length; t++){
        bool contact = MuJoCo_helper->CheckPairForCollisions("piston_rod", "goal",
                                                             MuJoCo_helper->saved_systems_state_list[t]);
        contact_list.push_back(contact);
    }

    // Find the contact making point
    int contact_time_step = 0;
    for(int i = 0; i < contact_list.size(); i++){
        if(contact_list[i]){
            contact_time_step = i;
            break;
        }
    }

    // Remove key-points about the smoothing site = int smoothing
    // if contact is at 100 and smoothing is 2, we remove 98, 99, 100, 101 and 102
    for(int i = contact_time_step - smoothing; i < contact_time_step + smoothing; i++){
        if(i >= 0 && i < horizon_length){
            keypoint_generator->keypoints[i].clear();
        }
    }
}

void Optimiser::GenerateDerivatives(){

    // Compute key-points at which we compute expensive dynamics derivatives
    ComputeKeypoints();

    if(smoothing_contact){
        SmoothDerivativesAtContact(smoothing);
    }

    // Compute dynamics derivatives at key-points and interpolate the remainder
    ComputeDynamicsDerivatives();

    // Compute cost derivatives
    ComputeCostDerivatives();

    // Compute the average percentage derivatives for each dof
    double average_percent_derivs = 0.0;
    for(int i = 0; i < activeModelTranslator->current_state_vector.dof; i++){
        average_percent_derivs += keypoint_generator->last_percentages[i];
    }
    average_percent_derivs /= activeModelTranslator->current_state_vector.dof;

    percentage_derivs_per_iteration.push_back(average_percent_derivs);

    // Filter dynamics derivatives if required
    if(filteringMethod != "none"){
        FilterDynamicsMatrices();
    }

    //    std::cout <<" interpolate derivs took: " << duration_cast<microseconds>(high_resolution_clock::now() - start_interp_time).count() / 1000.0f << " ms\n";

//    std::cout << "------------- residual derivatives wrt State -------------------" << std::endl;
//    for(int i = 0; i < activeModelTranslator->residual_list.size(); i++){
//        std::cout << "r_x[0][" << i << "]: " << std::endl;
//        std:: cout << r_x[0][i] << std::endl;
//        std::cout << "r_x[1][" << i << "]: " << std::endl;
//        std:: cout << r_x[1][i] << std::endl;
//    }
//
//    for(int i = 0; i <= 10; i++){
//        std:: cout << "r_x[ " << i << "] " << r_x[i][1] << "\n";
//    }

//    std::cout << "-------------- residual derivatives wrt State ------------------" << std::endl;
//    for(int i = 0; i < activeModelTranslator->num_residual_terms; i++){
//        std::cout << "r_u[" << i << "]: " << std::endl;
//        std:: cout << r_u[0][i] << std::endl;
//    }

//    std::cout << "------------------ cost derivatives from residuals ------------------------- \n";
//    std::cout << "residuals[0]: " << residuals[0] << "\n";
//    std::cout << "l_x[0]: " << std::endl;
//    std:: cout << l_x[0] << std::endl;
//    std::cout << "l_xx[0]: " << std::endl;
//    std:: cout << l_xx[0] << std::endl;
//    std::cout << "l_u[0]: " << std::endl;
//    std:: cout << l_u[0] << std::endl;
//    std::cout << "l_uu[0]: " << std::endl;
//    std:: cout << l_uu[0] << std::endl;
//
//    std::cout << "residuals[horizon_length - 1]: " << residuals[horizon_length - 1] << "\n";
//    std::cout << "l_x[horizon-1]: " << std::endl;
//    std:: cout << l_x[horizon_length-1] << std::endl;
//    std::cout << "l_xx[horizon-1]: " << std::endl;
//    std:: cout << l_xx[horizon_length-1] << std::endl;
//    std::cout << "l_u[horizon-1]: " << std::endl;
//    std:: cout << l_u[horizon_length-1] << std::endl;
//    std::cout << "l_uu[horizon-1]: " << std::endl;
//    std:: cout << l_uu[horizon_length-1] << std::endl;

//    ComputeCostDerivatives();
//
//    std::cout << "l_x[0]: " << std::endl;
//    std:: cout << l_x[1] << std::endl;
//    std::cout << "l_xx[0]: " << std::endl;
//    std:: cout << l_xx[1] << std::endl;
//    std::cout << "l_u[0]: " << std::endl;
//    std:: cout << l_u[0] << std::endl;
//    std::cout << "l_uu[0]: " << std::endl;
//    std:: cout << l_uu[0] << std::endl;
//
//    std::cout << "l_x[horizon-1]: " << std::endl;
//    std:: cout << l_x[horizon_length-1] << std::endl;
//    std::cout << "l_xx[horizon-1]: " << std::endl;
//    std:: cout << l_xx[horizon_length-1] << std::endl;
//    std::cout << "l_u[horizon-1]: " << std::endl;
//    std:: cout << l_u[horizon_length-1] << std::endl;
//    std::cout << "l_uu[horizon-1]: " << std::endl;
//    std:: cout << l_uu[horizon_length-1] << std::endl;
}

void Optimiser::ComputeKeypoints(){
    //auto start_keypoint_time = high_resolution_clock::now();
    keypoint_generator->GenerateKeyPoints(X_old, A, B);
    keypoint_generator->ResetCache();
    //std::cout << "gen keypoints time: " << duration_cast<microseconds>(high_resolution_clock::now() - start_keypoint_time).count() / 1000.0f << " ms\n";
}

void Optimiser::ComputeDynamicsDerivatives(){
    // Compute dynamics derivatives at keypoints - note if keypoint method = iterative error, we do not need to compute derivatives
    // as they have already been computed
    if(activeKeyPointMethod.name != "iterative_error") {
        auto start_fd_time = high_resolution_clock::now();
        ComputeDynamicsDerivativesAtKeypoints(keypoint_generator->keypoints);
        auto stop_fd_time = high_resolution_clock::now();
        auto duration_fd_time = duration_cast<microseconds>(stop_fd_time - start_fd_time);
    }



    // Interpolate the dynamics derivatives
//    auto start_interp_time = high_resolution_clock::now();
    keypoint_generator->InterpolateDerivatives(keypoint_generator->keypoints, horizon_length,
                                               A, B, r_x, r_u, activeYamlReader->costDerivsFD,
                                               activeModelTranslator->current_state_vector.num_ctrl);
}

void Optimiser::ComputeCostDerivatives(){
    // Compute residual derivatives over the entire trajectory
    auto time_start_residual_derivs = high_resolution_clock::now();
    ComputeResidualDerivatives();
    // If finite differencing is used for cost derivatives, compute cost derivs from residual derivatives
    for(int t = 0; t < horizon_length; t++){
        activeModelTranslator->CostDerivativesFromResiduals(activeModelTranslator->current_state_vector,
                                                            l_x[t], l_xx[t], l_u[t], l_uu[t],
                                                            residuals[t], r_x[t], r_u[t], false);
    }

    activeModelTranslator->CostDerivativesFromResiduals(activeModelTranslator->current_state_vector,
                                                        l_x[horizon_length - 1], l_xx[horizon_length - 1],
                                                        l_u[horizon_length - 1], l_uu[horizon_length - 1],
                                                        residuals[horizon_length - 1], r_x[horizon_length - 1], r_u[horizon_length - 1], true);

    auto time_stop_residual_derivs = high_resolution_clock::now();
    std::cout << "time resid derivs: " << duration_cast<microseconds>(time_stop_residual_derivs - time_start_residual_derivs).count() / 1000.0f << " ms\n";
}

void Optimiser::ComputeResidualDerivatives(){

    current_iteration = 0;
    num_threads_iterations = horizon_length + 1;
    tasks_residual_derivs.clear();

    for (int i = 0; i < horizon_length + 1; ++i) {
        tasks_residual_derivs.push_back(&Differentiator::ResidualDerivatives);
    }

    const int num_threads = std::thread::hardware_concurrency() - 1;  // Get the number of available CPU cores
    std::vector<std::thread> thread_pool;
    for (int i = 0; i < num_threads; ++i) {
        thread_pool.push_back(std::thread(&Optimiser::WorkerComputeResidualDerivatives, this, i));
    }

    for (std::thread& thread : thread_pool) {
        thread.join();
    }
}


void Optimiser::ComputeDynamicsDerivativesAtKeypoints(std::vector<std::vector<int>> keyPoints){

    MuJoCo_helper->InitModelForFiniteDifferencing();

    std::vector<int> timeIndices;
    for(int i = 0; i < keyPoints.size(); i++){
        if(!keyPoints[i].empty()){
            timeIndices.push_back(i);
        }
    }

    // Loop through keypoints and delete any entries that have no keypoints
    for(int i = 0; i < keyPoints.size(); i++){
        if(keyPoints[i].empty()){
            keyPoints.erase(keyPoints.begin() + i);
            i--;
        }
    }

    current_iteration = 0;
    num_threads_iterations = static_cast<int>(keyPoints.size());
    timeIndicesGlobal = timeIndices;

    // TODO - remove this? It is used for WorkerComputeDerivatives function to compute derivatives in parallel
    keypointsGlobal = keyPoints;

    // compute derivs serially
//    for(int i = 0; i < horizon_length; i++){
//        if(!keyPoints[i].empty()){
//            activeDifferentiator->DynamicsDerivatives(A[i], B[i], keyPoints[i], l_x[i], l_u[i], l_xx[i], l_uu[i],
//                                                     i, 0, false, activeYamlReader->costDerivsFD, true, 1e-6);
//        }
//    }

    // Setup all the required tasks
    tasks_dynamics_derivs.clear();
    for (int i = 0; i < keyPoints.size(); ++i) {
        tasks_dynamics_derivs.push_back(&Differentiator::DynamicsDerivatives);
    }

    // Get the number of threads available
    const int num_threads = std::thread::hardware_concurrency() - 1;  // Get the number of available CPU cores
    std::vector<std::thread> thread_pool;
    for (int i = 0; i < num_threads; ++i) {
        thread_pool.push_back(std::thread(&Optimiser::WorkerComputeDerivatives, this, i));
    }

    for (std::thread& thread : thread_pool) {
        thread.join();
    }
      
    MuJoCo_helper->ResetModelAfterFiniteDifferencing();

    auto time_cost_start = std::chrono::high_resolution_clock::now();
}

void Optimiser::WorkerComputeDerivatives(int threadId) {
    while (true) {
        int iteration = current_iteration.fetch_add(1);
        if (iteration >= num_threads_iterations) {
            break;  // All iterations done
        }

        int timeIndex = timeIndicesGlobal[iteration];

        std::vector<int> keyPoints;
        (activeDifferentiator.get()->*(tasks_dynamics_derivs[iteration]))(A[timeIndex], B[timeIndex],
                                        keypointsGlobal[iteration],
                                        timeIndex, threadId, true, 1e-6);
    }
}

void Optimiser::WorkerComputeResidualDerivatives(int threadId){
    while (true) {
        int iteration = current_iteration.fetch_add(1);
        if (iteration >= num_threads_iterations) {
            break;  // All iterations done
        }

//        int timeIndex = timeIndicesGlobal[iteration];

        std::vector<int> keyPoints;
        (activeDifferentiator.get()->*(tasks_residual_derivs[iteration]))(r_x[iteration], r_u[iteration],
                                                                          iteration, threadId, true, 1e-6);
    }
}

void Optimiser::FilterDynamicsMatrices() {

    // Aliases
    int dof = activeModelTranslator->current_state_vector.dof;

    for(int i = dof; i < 2 * dof; i++){
        for(int j = 0; j < 2 * dof; j++){
            std::vector<double> unfiltered;
            std::vector<double> filtered;

            for(int k = 0; k < horizon_length; k++){
                unfiltered.push_back(A[k](i, j));
            }

            if(filteringMethod == "low_pass"){
                filtered = FilterIndValLowPass(unfiltered);
            }
            else if(filteringMethod == "FIR"){
                filtered = FilterIndValFIRFilter(unfiltered, FIRCoefficients);
            }
            else{
                std::cerr << "Filtering method not recognised" << std::endl;
            }


            for(int k = 0; k < horizon_length; k++){
                A[k](i, j) = filtered[k];
            }
        }
    }
}

std::vector<double> Optimiser::FilterIndValLowPass(std::vector<double> unfiltered){
    double yn1 = unfiltered[0];
    double xn1 = unfiltered[0];

    std::vector<double> filtered;
    for(int i = 0; i < unfiltered.size(); i++){
        double xn = unfiltered[i];

        double yn = ((1-lowPassACoefficient)*yn1) + lowPassACoefficient*((xn + xn1)/2);

        xn1 = xn;
        yn1 = yn;

        filtered.push_back(yn);
    }
    return filtered;
}

std::vector<double> Optimiser::FilterIndValFIRFilter(std::vector<double> unfiltered, std::vector<double> filterCoefficients){
    std::vector<double> filtered;

    for(int i = 0; i < unfiltered.size(); i++){
        filtered.push_back(0);
    }

    for(int i = 0; i < unfiltered.size(); i++){
        for(int j = 0; j < filterCoefficients.size(); j++){
            if(i - j >= 0){
                filtered[i] += unfiltered[i - j] * filterCoefficients[j];
            }
        }

    }
    return filtered;
}

void Optimiser::setFIRFilter(std::vector<double> _FIRCoefficients){
    FIRCoefficients.clear();

    for(int i = 0; i < _FIRCoefficients.size(); i++){
        FIRCoefficients.push_back(_FIRCoefficients[i]);
    }
}

void Optimiser::SaveSystemStateToRolloutData(mjData *d, int thread_id, int data_index){

    rollout_data[thread_id][data_index].time = d->time;

    for(int i = 0; i < MuJoCo_helper->model->nq; i++){
        rollout_data[thread_id][data_index].q_pos[i] = d->qpos[i];
    }

    for(int i = 0; i < MuJoCo_helper->model->nv; i++){
        rollout_data[thread_id][data_index].q_vel[i] = d->qvel[i];
        rollout_data[thread_id][data_index].q_acc[i] = d->qacc[i];
        rollout_data[thread_id][data_index].q_acc_warmstart[i] = d->qacc_warmstart[i];
        rollout_data[thread_id][data_index].qfrc_applied[i] = d->qfrc_applied[i];
    }

    for(int i = 0; i < MuJoCo_helper->model->nu; i++){
        rollout_data[thread_id][data_index].ctrl[i] = d->ctrl[i];
    }

    for(int i = 0; i < 6*MuJoCo_helper->model->nbody; i++){
        rollout_data[thread_id][data_index].xfrc_applied[i] = d->xfrc_applied[i];
    }
}

void Optimiser::SaveBestRollout(int thread_id){
    for(int t = 0; t < horizon_length; t++){

        MuJoCo_helper->saved_systems_state_list[t]->time = rollout_data[thread_id][t].time;

        for(int i = 0; i < MuJoCo_helper->model->nq; i++){
            MuJoCo_helper->saved_systems_state_list[t]->qpos[i] = rollout_data[thread_id][t].q_pos[i];
        }

        for(int i = 0; i < MuJoCo_helper->model->nv; i++){
            MuJoCo_helper->saved_systems_state_list[t]->qvel[i]           = rollout_data[thread_id][t].q_vel[i];
            MuJoCo_helper->saved_systems_state_list[t]->qacc[i]           = rollout_data[thread_id][t].q_acc[i];
            MuJoCo_helper->saved_systems_state_list[t]->qacc_warmstart[i] = rollout_data[thread_id][t].q_acc_warmstart[i];
            MuJoCo_helper->saved_systems_state_list[t]->qfrc_applied[i]   = rollout_data[thread_id][t].qfrc_applied[i];
        }

        for(int i = 0; i < MuJoCo_helper->model->nu; i++){
            MuJoCo_helper->saved_systems_state_list[t]->ctrl[i] = rollout_data[thread_id][t].ctrl[i];
        }

        for(int i = 0; i < 6*MuJoCo_helper->model->nbody; i++){
            MuJoCo_helper->saved_systems_state_list[t]->xfrc_applied[i] = rollout_data[thread_id][t].xfrc_applied[i];
        }

        // Update the residuals of the nominal trajectory
        activeModelTranslator->Residuals(MuJoCo_helper->saved_systems_state_list[t], residuals[t]);
    }
}