
#include "Optimiser.h"

Optimiser::Optimiser(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> _MuJoCo_helper, std::shared_ptr<FileHandler> _yamlReader, std::shared_ptr<Differentiator> _differentiator){
    activeModelTranslator = _modelTranslator;
    MuJoCo_helper = _MuJoCo_helper;
    activeYamlReader = _yamlReader;
    activeDifferentiator = _differentiator;

    dof = activeModelTranslator->dof;
    num_ctrl = activeModelTranslator->num_ctrl;

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
                                                             activeModelTranslator->dof,
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

void Optimiser::ResizeStateVector(int new_num_dofs){

    dof = new_num_dofs;
    int state_vector_size = new_num_dofs * 2;

    for(int t = 0; t < horizonLength; t++){

        // State vectors
        X_new[t].resize(state_vector_size, 1);
        X_old[t].resize(state_vector_size, 1);

        // Cost derivatives
        l_x[t].resize(state_vector_size, 1);
        l_xx[t].resize(state_vector_size, state_vector_size);

        // Dynamics derivatives
        A[t].resize(state_vector_size, state_vector_size);
        A[t].block(0, 0, dof, dof).setIdentity();
        A[t].block(0, dof, dof, dof).setIdentity();
        A[t].block(0, dof, dof, dof) *= MuJoCo_helper->ReturnModelTimeStep();
        B[t].resize(state_vector_size, num_ctrl);

    }
}

void Optimiser::SetTrajecNumber(int trajec_number) {
    currentTrajecNumber = trajec_number;
}

void Optimiser::ReturnOptimisationData(double &_optTime, double &_costReduction, double &_avgPercentageDerivs, double &_avgTimeGettingDerivs, int &_numIterations){

    _optTime = opt_time_ms;
    _costReduction = costReduction;
    _avgPercentageDerivs = avg_percent_derivs;
    _avgTimeGettingDerivs = avg_time_get_derivs_ms;
    _numIterations = numIterationsForConvergence;
}

keypoint_method Optimiser::ReturnCurrentKeypointMethod(){
    return keypoint_generator->ReturnCurrentKeypointMethod();
}

void Optimiser::SetCurrentKeypointMethod(keypoint_method _keypoint_method){
    keypoint_generator->SetKeypointMethod(_keypoint_method);
}

void Optimiser::GenerateDerivatives(){
    // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory
    // generate the dynamics evaluation waypoints
    auto start_keypoint_time = high_resolution_clock::now();
    keypoint_generator->GenerateKeyPoints(X_old, A, B);

    keypoint_generator->ResetCache();
//    std::cout << "gen keypoints time: " << duration_cast<microseconds>(high_resolution_clock::now() - start_keypoint_time).count() / 1000.0f << " ms\n";

    // Calculate derivatives via finite differencing / analytically for cost functions if available
    if(activeKeyPointMethod.name != "iterative_error"){
        auto start_fd_time = high_resolution_clock::now();
        ComputeDerivativesAtSpecifiedIndices(keypoint_generator->keypoints);
        auto stop_fd_time = high_resolution_clock::now();
        auto duration_fd_time = duration_cast<microseconds>(stop_fd_time - start_fd_time);

//        std::cout << "fd time: " << duration_fd_time.count() / 1000.0f << " ms\n";
    }
    else{
        ComputeCostDerivatives();
    }

    auto start_interp_time = high_resolution_clock::now();
//    InterpolateDerivatives(keypoint_generator->keypoints, activeYamlReader->costDerivsFD);
    keypoint_generator->InterpolateDerivatives(keypoint_generator->keypoints,  horizonLength,
                                               A, B, l_x, l_u, l_xx, l_uu, activeYamlReader->costDerivsFD,
                                               num_ctrl);
//    std::cout <<" interpolate derivs took: " << duration_cast<microseconds>(high_resolution_clock::now() - start_interp_time).count() / 1000.0f << " ms\n";

    double average_percent_derivs = 0.0f;
    for(int i = 0; i < dof; i++){
        average_percent_derivs += keypoint_generator->last_percentages[i];
    }
    average_percent_derivs /= dof;

    percentage_derivs_per_iteration.push_back(average_percent_derivs);

    if(filteringMethod != "none"){
        FilterDynamicsMatrices();
    }
}

void Optimiser::ComputeCostDerivatives(){
    #pragma omp parallel for
    for(int i = 0; i < horizonLength; i++){
        activeModelTranslator->CostDerivatives(MuJoCo_helper->saved_systems_state_list[i], l_x[i], l_xx[i], l_u[i], l_uu[i], false);
    }

    activeModelTranslator->CostDerivatives(MuJoCo_helper->saved_systems_state_list[horizonLength - 1],
                                           l_x[horizonLength - 1], l_xx[horizonLength - 1], l_u[horizonLength - 1], l_uu[horizonLength - 1], true);
}

void Optimiser::ComputeDerivativesAtSpecifiedIndices(std::vector<std::vector<int>> keyPoints){

    MuJoCo_helper->InitModelForFiniteDifferencing();

    std::vector<int> timeIndices;
    for(int i = 0; i < keyPoints.size(); i++){
        if(keyPoints[i].size() != 0){
            timeIndices.push_back(i);
        }
    }

    // Loop through keypoints and delete any entries that have no keypoints
    for(int i = 0; i < keyPoints.size(); i++){
        if(keyPoints[i].size() == 0){
            keyPoints.erase(keyPoints.begin() + i);
            i--;
        }
    }

    current_iteration = 0;
    num_threads_iterations = keyPoints.size();
    timeIndicesGlobal = timeIndices;

    // TODO - remove this? It is used for WorkerComputeDerivatives function to compute derivatives in parallel
    keypointsGlobal = keyPoints;

    // Setup all the required tasks
    for (int i = 0; i < keyPoints.size(); ++i) {
        tasks.push_back(&Differentiator::ComputeDerivatives);
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

    // compute derivs serially
//    for(int i = 0; i < horizonLength; i++){
//        if(keyPoints[i].size() != 0){
//            activeDifferentiator->ComputeDerivatives(A[i], B[i], keyPoints[i], l_x[i], l_xx[i], l_u[i], l_uu[i], false, i, false, 0);
//        }
//    }
      
    MuJoCo_helper->ResetModelAfterFiniteDifferencing();

    auto time_cost_start = std::chrono::high_resolution_clock::now();

    if(!activeYamlReader->costDerivsFD){
        for(int i = 0; i < horizonLength; i++){
            if(i == 0){
                activeModelTranslator->CostDerivatives(MuJoCo_helper->saved_systems_state_list[i],
                                                       l_x[i], l_xx[i], l_u[i], l_uu[i], false);
            }
            else{
                activeModelTranslator->CostDerivatives(MuJoCo_helper->saved_systems_state_list[i],
                                                       l_x[i], l_xx[i], l_u[i], l_uu[i], false);
            }
        }
        activeModelTranslator->CostDerivatives(MuJoCo_helper->saved_systems_state_list[horizonLength - 1],
                                               l_x[horizonLength - 1], l_xx[horizonLength - 1], l_u[horizonLength - 1], l_uu[horizonLength - 1], true);
    }

//    std::cout << "time cost derivs: " << duration_cast<microseconds>(high_resolution_clock::now() - time_cost_start).count() / 1000.0f << " ms\n";
}

void Optimiser::WorkerComputeDerivatives(int threadId) {
    while (true) {
        int iteration = current_iteration.fetch_add(1);
        if (iteration >= num_threads_iterations) {
            break;  // All iterations done
        }

        int timeIndex = timeIndicesGlobal[iteration];
        bool terminal = false;
        if(timeIndex == horizonLength - 1){
            terminal = true;
        }

        std::vector<int> keyPoints;
        (activeDifferentiator.get()->*(tasks[iteration]))(A[timeIndex], B[timeIndex],
                                        keypointsGlobal[iteration], l_x[timeIndex], l_u[timeIndex], l_xx[timeIndex], l_uu[timeIndex],
                                        timeIndex, threadId, terminal, activeYamlReader->costDerivsFD, true, 1e-6);
    }
}

//void Optimiser::InterpolateDerivatives(const std::vector<std::vector<int>> &keyPoints, bool costDerivs){
//    MatrixXd startB;
//    MatrixXd endB;
//    MatrixXd addB;
//
//    MatrixXd startACol1;
//    MatrixXd endACol1;
//    MatrixXd addACol1;
//
//    MatrixXd startACol2;
//    MatrixXd endACol2;
//    MatrixXd addACol2;
//
//    double start_l_x_col1;
//    double end_l_x_col1;
//    double add_l_x_col1;
//    double start_l_x_col2;
//    double end_l_x_col2;
//    double add_l_x_col2;
//
//    MatrixXd start_l_xx_col1;
//    MatrixXd end_l_xx_col1;
//    MatrixXd add_l_xx_col1;
//    MatrixXd start_l_xx_col2;
//    MatrixXd end_l_xx_col2;
//    MatrixXd add_l_xx_col2;
//
//    // Create an array to track startIndices of next interpolation for each dof
//    int startIndices[dof];
//    for(int i = 0; i < dof; i++){
//        startIndices[i] = 0;
//    }
//
//    // Loop through all the time indices - can skip the first
//    // index as we preload the first index as the start index for all dofs.
//    for(int t = 1; t < horizonLength; t++){
//        // Loop through all the dofs
//        for(int i = 0; i < dof; i++){
//            // Check the current vector at that time segment for the current dof
//            std::vector<int> columns = keyPoints[t];
//
//            // If there are no keypoints, continue onto second run of the loop
//            if(columns.empty()){
//                continue;
//            }
//
//            for(int j = 0; j < columns.size(); j++){
//
//                // If there is a match, interpolate between the start index and the current index
//                // For the given columns
//                if(i == columns[j]){
////                    cout << "dof: " << i << " end index: " << t << " start index: " << startIndices[i] << "\n";
//                    startACol1 = A[startIndices[i]].block(0, i, 2*dof, 1);
//                    endACol1 = A[t].block(0, i, 2*dof, 1);
//                    addACol1 = (endACol1 - startACol1) / (t - startIndices[i]);
//
//                    // Same again for column 2 which is dof + i
//                    startACol2 = A[startIndices[i]].block(0, i + dof, 2*dof, 1);
//                    endACol2 = A[t].block(0, i + dof, 2*dof, 1);
//                    addACol2 = (endACol2 - startACol2) / (t - startIndices[i]);
//
//                    if(costDerivs){
//                        start_l_x_col1 = l_x[startIndices[i]](i, 0);
//                        end_l_x_col1 = l_x[t](i, 0);
//                        add_l_x_col1 = (end_l_x_col1 - start_l_x_col1) / (t - startIndices[i]);
//
//                        start_l_x_col2 = l_x[startIndices[i]](i + dof, 0);
//                        end_l_x_col2 = l_x[t](i + dof, 0);
//                        add_l_x_col2 = (end_l_x_col2 - start_l_x_col2) / (t - startIndices[i]);
//
//                        start_l_xx_col1 = l_xx[startIndices[i]].block(i, 0, 1, dof);
//                        end_l_xx_col1 = l_xx[t].block(i, 0, 1, dof);
//                        add_l_xx_col1 = (end_l_xx_col1 - start_l_xx_col1) / (t - startIndices[i]);
//
//                        start_l_xx_col2 = l_xx[startIndices[i]].block(i + dof, 0, 1, dof);
//                        end_l_xx_col2 = l_xx[t].block(i + dof, 0, 1, dof);
//                        add_l_xx_col2 = (end_l_xx_col2 - start_l_xx_col2) / (t - startIndices[i]);
//                    }
//
//                    if(i < num_ctrl){
//                        startB = B[startIndices[i]].block(0, i, 2*dof, 1);
//                        endB = B[t].block(0, i, 2*dof, 1);
//                        addB = (endB - startB) / (t - startIndices[i]);
//                    }
//
//                    for(int k = startIndices[i]; k < t; k++){
//                        A[k].block(0, i, 2*dof, 1) = startACol1 + ((k - startIndices[i]) * addACol1);
//
//                        A[k].block(0, i + dof, 2*dof, 1) = startACol2 + ((k - startIndices[i]) * addACol2);
//
//                        if(costDerivs){
//                            l_x[k](i) = start_l_x_col1 + ((k - startIndices[i]) * add_l_x_col1);
//                            l_x[k](i + dof) = start_l_x_col2 + ((k - startIndices[i]) * add_l_x_col2);
//
//                            l_xx[k].block(i, 0, 1, dof) = start_l_xx_col1 + ((k - startIndices[i]) * add_l_xx_col1);
//                            l_xx[k].block(i + dof, 0, 1, dof) = start_l_xx_col2 + ((k - startIndices[i]) * add_l_xx_col2);
//                        }
//
//                        if(i < num_ctrl){
//                            B[k].block(0, i, 2*dof, 1) = startB + ((k - startIndices[i]) * addB);
//                        }
//                    }
//                    startIndices[i] = t;
//                }
//            }
//        }
//    }
//}

void Optimiser::FilterDynamicsMatrices() {

    for(int i = dof; i < 2 * dof; i++){
        for(int j = 0; j < 2 * dof; j++){
            std::vector<double> unfiltered;
            std::vector<double> filtered;

            for(int k = 0; k < horizonLength; k++){
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


            for(int k = 0; k < horizonLength; k++){
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

