#include "interpolated_iLQR.h"

interpolatediLQR::interpolatediLQR(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<physicsSimulator> _physicsSimulator, std::shared_ptr<differentiator> _differentiator, int _maxHorizon, std::shared_ptr<visualizer> _visualizer, std::shared_ptr<fileHandler> _yamlReader) :
        Optimiser(_modelTranslator, _physicsSimulator, _yamlReader, _differentiator){

    maxHorizon = _maxHorizon;
    activeVisualizer = _visualizer;

    // initialise all vectors of matrices
    for(int i = 0; i < maxHorizon; i++){
        // Cost matrices
        l_x.push_back(MatrixXd(2*dof, 1));
        l_xx.push_back(MatrixXd(2*dof, 2*dof));
        l_u.push_back(MatrixXd(num_ctrl, 1));
        l_uu.push_back(MatrixXd(num_ctrl, num_ctrl));

        // Dynamics derivatives matrices
        // TODO - Move this to Optimiser constructor
        A.push_back(MatrixXd(2*dof, 2*dof));
        B.push_back(MatrixXd(2*dof, num_ctrl));

        A[i].block(0, 0, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof) *= activePhysicsSimulator->returnModelTimeStep();
        B[i].setZero();

        K.push_back(MatrixXd(num_ctrl, 2*dof));
        k.push_back(MatrixXd(num_ctrl, 1));

        U_old.push_back(MatrixXd(num_ctrl, 1));
        U_new.push_back(MatrixXd(num_ctrl, 1));
        X_old.push_back(MatrixXd(2*dof, 1));
        X_new.push_back(MatrixXd(2*dof, 1));

        std::vector<MatrixXd> U_temp;
        for(int j = 0; j < 8; j++){
            U_temp.push_back(MatrixXd(num_ctrl, 1));
        }

        U_alpha.push_back(U_temp);
    }

    // One more state than control
    X_old.push_back(MatrixXd(2*dof, 1));
    X_new.push_back(MatrixXd(2*dof, 1));
    l_x.push_back(MatrixXd(2*dof, 1));
    l_xx.push_back(MatrixXd(2*dof, 2*dof));


    // Whether to do some low pass filtering over A and B matrices
    filteringMethod = activeYamlReader->filtering;

}

double interpolatediLQR::rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls){
    double cost = 0.0f;

    if(initialDataIndex != MAIN_DATA_STATE){
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, initialDataIndex);
    }

    MatrixXd Xt(activeModelTranslator->state_vector_size, 1);
    MatrixXd X_last(activeModelTranslator->state_vector_size, 1);
    MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
    MatrixXd U_last(activeModelTranslator->num_ctrl, 1);

    X_old[0] = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);
//    std::cout << "X_old[0]: " << X_old[0].transpose() << std::endl;

    if(activePhysicsSimulator->checkIfDataIndexExists(0)){
        activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    }
    else{
        activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
    }

    for(int i = 0; i < horizonLength; i++){
        // set controls
        activeModelTranslator->SetControlVector(initControls[i], MAIN_DATA_STATE);

        // Integrate simulator
        activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        // return cost for this state
        Xt = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);
        Ut = activeModelTranslator->ReturnControlVector(MAIN_DATA_STATE);
        double stateCost;
        
        if(i == horizonLength - 1){
            stateCost = activeModelTranslator->CostFunction(MAIN_DATA_STATE, true);
        }
        else{
            stateCost = activeModelTranslator->CostFunction(MAIN_DATA_STATE, false);
        }

        // If required to save states to trajectory tracking, then save state
        if(saveStates){
            X_old[i + 1] = Xt.replicate(1, 1);
            U_old[i] = Ut.replicate(1, 1);
            if(activePhysicsSimulator->checkIfDataIndexExists(i + 1)){
                activePhysicsSimulator->copySystemState(i + 1, MAIN_DATA_STATE);
            }
            else{
                activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
            }
        }

//        cost += (stateCost * active_physics_simulator->returnModelTimeStep());
        cost += stateCost;
    }

//    cout << "cost of initial trajectory was: " << cost << endl;
    initialCost = cost;
    costHistory.push_back(cost);

    return cost;
}

// ------------------------------------------------------------------------------------------------------
//
//  optimise - Optimise a sequence of controls for a given problem
//  @Params:
//  initialDataIndex - the data index of the system state that the optimisation problem should start from
//  initControls - The initial controls for the problem
//  maxIterations - The maximum iterations of the solver before it should return a new set of controls
//  horizonLength - How far into the future the Optimiser should look when optimising the controls
//
//  @Returns:
//  optimisedControls - New optimised controls that give a lower cost than the initial controls
//
// -------------------------------------------------------------------------------------------------------
std::vector<MatrixXd> interpolatediLQR::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength){
    if(verboseOutput) {
        cout << " ---------------- optimisation begins -------------------" << endl;
        cout << " ------ " << activeModelTranslator->model_name << " ------ " << endl;
        cout << "min_N " << activeDerivativeInterpolator.minN << "  keypointsMethod: " << activeDerivativeInterpolator.keypoint_method;
        cout << " filtering: " << filteringMethod << endl;
    }

    auto optStart = high_resolution_clock::now();
    
    // - Initialise variables
    std::vector<MatrixXd> optimisedControls;
    horizonLength = _horizonLength;
    numberOfTotalDerivs = _horizonLength * dof;
    // TODO - decide whether to use this or not, it seems to break when i remove it.
//    lambda = 0.1;
    double oldCost = 0.0f;
    double newCost = 0.0f;
    bool costReducedLastIter = true;

    // ---------------------- Clear data saving variables ----------------------
    costHistory.clear();
    optTime = 0.0f;
    percentDerivsPerIter.clear();
    timeDerivsPerIter.clear();
    avgPercentDerivs = 0;
    numIterationsForConvergence = 0;

    avgTime_getDerivs_ms = 0.0f;
    avgTime_forwardsPass_ms = 0.0f;
    avgTime_backwardsPass_ms = 0.0f;
    percentDerivsPerIter.clear();
    time_backwardsPass_ms.clear();
    time_forwardsPass_ms.clear();
    time_getDerivs_ms.clear();
    // ------------------------------------------------------------------------

    auto time_start = high_resolution_clock::now();
    oldCost = rolloutTrajectory(initialDataIndex, true, initControls);
    auto time_end = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(time_end - time_start);
//    std::cout << "time for rollout: " << duration.count() / 1000.0f << endl;
    initialCost = oldCost;
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    // Optimise for a set number of iterations
    for(int i = 0; i < maxIter; i++){
        numIterationsForConvergence++;

        //STEP 1 - If forwards pass changed the trajectory, -
        auto derivsstart = high_resolution_clock::now();
        if(costReducedLastIter){
            generateDerivatives();
        }
        auto derivsstop = high_resolution_clock::now();
        auto linDuration = duration_cast<microseconds>(derivsstop - derivsstart);
        time_getDerivs_ms.push_back(linDuration.count() / 1000.0f);
        timeDerivsPerIter.push_back(linDuration.count() / 1000000.0f);

        if(saveTrajecInfomation){

            activeYamlReader->saveTrajecInfomation(A, B, X_old, U_old, activeModelTranslator->model_name, activeYamlReader->csvRow, horizonLength);
        }

        // STEP 2 - BackwardsPass using the calculated derivatives to calculate an optimal feedback control law
        bool validBackwardsPass = false;
        bool lambdaExit = false;

        auto bp_start = high_resolution_clock::now();
        while(!validBackwardsPass){
            validBackwardsPass = backwardsPass_Quu_reg();

            if(!validBackwardsPass){
                if(lambda < maxLambda){
                    lambda *= lambdaFactor;
                }
                else{
                    lambdaExit = true;
                    break;
                }
            }
            else{
                if(lambda > minLambda){
                    lambda /= lambdaFactor;
                }
            }
        }
        auto bp_stop = high_resolution_clock::now();
        auto bpDuration = duration_cast<microseconds>(bp_stop - bp_start);

        time_backwardsPass_ms.push_back(bpDuration.count() / 1000.0f);

        if(!lambdaExit){
            // STEP 3 - Forwards Pass - use the optimal control feedback law and rollout in simulation and calculate new cost of trajectory
            auto fp_start = high_resolution_clock::now();
            newCost = forwardsPass(oldCost);
//            newCost = forwardsPassParallel(oldCost);
            auto fp_stop = high_resolution_clock::now();
            auto fpDuration = duration_cast<microseconds>(fp_stop - fp_start);
            time_forwardsPass_ms.push_back(fpDuration.count() / 1000.0f);

            if(verboseOutput){
                cout << "| derivs: " << time_getDerivs_ms[i] << " | backwardsPass: " << time_backwardsPass_ms[i] << " | forwardsPass: " << time_forwardsPass_ms[i] << " ms |\n";
                cout << "| Cost went from " << oldCost << " ---> " << newCost << " | \n";
            }

            costHistory.push_back(newCost);

            // STEP 4 - Check for convergence
            bool converged;
            converged = checkForConvergence(oldCost, newCost);

            if(newCost < oldCost){
                oldCost = newCost;
                costReducedLastIter = true;
            }
            else{
                costReducedLastIter = false;
                if(lambda < maxLambda){
                    lambda *= lambdaFactor;
                    lambda *= lambdaFactor;
                }
            }

            if(converged && (i >= minIter))
            {
                break;
            }
        }
        else{
            cout << "exiting optimisation due to lambda > lambdaMax \n";
            break;
        }
    }

    costReduction = 1 - (newCost / initialCost);
    auto optFinish = high_resolution_clock::now();
    auto optDuration = duration_cast<microseconds>(optFinish - optStart);
    optTime = optDuration.count() / 1000.0f;
    if(verboseOutput){
        cout << "optimisation took: " << optTime<< " ms\n";
        cout << " ---------------- optimisation complete -------------------" << endl;
    }

    for(int i = 0; i < time_getDerivs_ms.size(); i++){
        avgTime_getDerivs_ms += time_getDerivs_ms[i];
    }

    for(int i = 0; i < percentDerivsPerIter.size(); i++){
        avgPercentDerivs += percentDerivsPerIter[i];
    }

    avgTime_getDerivs_ms /= time_getDerivs_ms.size();
    avgPercentDerivs /= percentDerivsPerIter.size();

    for(int i = 0; i < time_backwardsPass_ms.size(); i++){
        avgTime_backwardsPass_ms += time_backwardsPass_ms[i];
    }

    avgTime_backwardsPass_ms /= time_backwardsPass_ms.size();

    for(int i = 0; i < time_forwardsPass_ms.size(); i++){
        avgTime_forwardsPass_ms += time_forwardsPass_ms[i];
    }

    if(time_forwardsPass_ms.size() != 0){
        avgTime_forwardsPass_ms /= time_forwardsPass_ms.size();
    }

    // Load the initial data back into main data
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    for(int i = 0; i < horizonLength; i++){
        optimisedControls.push_back(U_old[i]);
    }

    if(saveTrajecInfomation){
        activeYamlReader->saveTrajecInfomation(A, B, X_old, U_old, activeModelTranslator->model_name, activeYamlReader->csvRow, horizonLength);
    }


    return optimisedControls;
}


// ------------------------------------------- STEP 2 FUNCTIONS (BACKWARDS PASS) ----------------------------------------------
bool interpolatediLQR::backwardsPass_Quu_reg(){
    MatrixXd V_x(2*dof, 2*dof);
    V_x = l_x[horizonLength - 1];
    MatrixXd V_xx(2*dof, 2*dof);
    V_xx = l_xx[horizonLength - 1];
    int Quu_pd_check_counter = 0;
    int number_steps_between_pd_checks = 100;

    double timeInverse = 0.0f;

    MatrixXd Q_x(2*dof, 1);
    MatrixXd Q_u(num_ctrl, 1);
    MatrixXd Q_xx(2*dof, 2*dof);
    MatrixXd Q_uu(num_ctrl, num_ctrl);
    MatrixXd Q_ux(num_ctrl, 2*dof);

//    cout << "V_x \n" << V_x << endl;
//    cout << "V_xx \n " << V_xx << endl;

    // TODO check if this should start at -2 or -1 and end at 0 or 1?
    for(int t = horizonLength - 1; t > -1; t--){

//        cout << "t: " << t << endl;
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
            if(!isMatrixPD(Q_uu_reg)){
                cout << "non PD matrix encountered at t = " << t << endl;
                return false;
            }
            Quu_pd_check_counter = 0;
        }

        //  time this using chrono
//        auto timeStart = std::chrono::high_resolution_clock::now();

        auto temp = (Q_uu_reg).ldlt();
        MatrixXd I(num_ctrl, num_ctrl);
        I.setIdentity();
        MatrixXd Q_uu_inv = temp.solve(I);

//        auto timeEnd = std::chrono::high_resolution_clock::now();
//        auto timeTaken = std::chrono::duration_cast<std::chrono::microseconds>(timeEnd - timeStart);
//        timeInverse += timeTaken.count()/ 1000000.0f;

        k[t] = -Q_uu_inv * Q_u;
        K[t] = -Q_uu_inv * Q_ux;

//        auto temp1 = Q_u.transpose() * Q_uu_inv * Q_ux;
//        cout << "temp1: " << temp1 << endl;
//        cout << "Q_x: " << Q_x << endl;

//        V_x = Q_x - (Q_u.transpose() * Q_uu_inv * Q_ux).transpose();
//        V_xx = Q_xx - (Q_ux.transpose() * Q_uu_inv * Q_ux);

        V_x = Q_x + (K[t].transpose() * (Q_uu * k[t])) + (K[t].transpose() * Q_u) + (Q_ux.transpose() * k[t]);
        V_xx = Q_xx + (K[t].transpose() * (Q_uu * K[t])) + (K[t].transpose() * Q_ux) + (Q_ux.transpose() * K[t]);

        V_xx = (V_xx + V_xx.transpose()) / 2;

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

bool interpolatediLQR::isMatrixPD(Ref<MatrixXd> matrix){
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
double interpolatediLQR::forwardsPass(double oldCost){
    double newCost;
    bool costReduction = false;
    int alphaCount = 0;

    MatrixXd Xt(2 * dof, 1);
    MatrixXd Ut(num_ctrl, 1);

    std::vector<double> alphas = {1.0, 0.8, 0.5, 0.3, 0.1};

    while(!costReduction){

        // Copy intial data state into main data state for rollout
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        newCost = 0;
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);

        for(int t = 0; t < horizonLength; t++) {
            // Step 1 - get old state and old control that were linearised around
            _X = X_old[t].replicate(1, 1);
            _U = U_old[t].replicate(1, 1);

            X_new = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);
            // Calculate difference from new state to old state
            stateFeedback = X_new - _X;

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
            U_new[t] = _U + (alphas[alphaCount] * k[t]) + feedBackGain;

            // Clamp torque within limits
            if(activeModelTranslator->state_vector.robots[0].torqueControlled){
                for(int i = 0; i < num_ctrl; i++){
                    if (U_new[t](i) > activeModelTranslator->state_vector.robots[0].torqueLimits[i]) U_new[t](i) = activeModelTranslator->state_vector.robots[0].torqueLimits[i];
                    if (U_new[t](i) < -activeModelTranslator->state_vector.robots[0].torqueLimits[i]) U_new[t](i) = -activeModelTranslator->state_vector.robots[0].torqueLimits[i];
                }
            }

            activeModelTranslator->SetControlVector(U_new[t], MAIN_DATA_STATE);

            Ut = U_new[t].replicate(1, 1);

            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->CostFunction(MAIN_DATA_STATE, true);
            }
            else{
                newStateCost = activeModelTranslator->CostFunction(MAIN_DATA_STATE, false);
            }

//            newCost += (newStateCost * active_physics_simulator->returnModelTimeStep());
            newCost += newStateCost;

            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            // Copy system state to fp_rollout_buffer to prevent a second rollout of computations using simulation integration
            activePhysicsSimulator->saveDataToRolloutBuffer(MAIN_DATA_STATE, t + 1);

//             if(t % 5 == 0){
//                 const char* fplabel = "fp";
//                 activeVisualizer->render(fplabel);
//             }

        }

//        cout << "cost from alpha: " << alphaCount << ": " << newCost << endl;

        if(newCost < oldCost){
            costReduction = true;
        }
        else{
            alphaCount++;
            if(alphaCount >= alphas.size()){
                break;
            }
        }
    }

    // If the cost was reduced
    if(newCost < oldCost){
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        //Copy the rollout buffer to saved systems state list, prevents recomputation using optimal controls
        activePhysicsSimulator->copyRolloutBufferToSavedSystemStatesList();

        for(int i = 0 ; i < horizonLength; i++){
            activeModelTranslator->active_physics_simulator->forwardSimulator(i + 1);
            X_old.at(i + 1) = activeModelTranslator->ReturnStateVector(i + 1);
            U_old[i] = U_new[i].replicate(1, 1);
        }

        return newCost;
    }

    return oldCost;
}

double interpolatediLQR::forwardsPassParallel(double oldCost){
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
        activePhysicsSimulator->copySystemState(i+1, 0);
    }

    MatrixXd initState = activeModelTranslator->ReturnStateVector(1);
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

            X_new = activeModelTranslator->ReturnStateVector(i + 1);

            // Calculate difference from new state to old state
//            stateFeedback = X_new - _X;
            stateFeedback = X_new - X_old[t];

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
//            U_alpha[t][i] = _U + (alphas[i] * k[t]) + feedBackGain;
            U_alpha[t][i] = U_old[t] + (alphas[i] * k[t]) + feedBackGain;

            // Clamp torque within limits
            if(activeModelTranslator->state_vector.robots[0].torqueControlled){
                for(int k = 0; k < num_ctrl; k++){
                    if (U_alpha[t][i](k) > activeModelTranslator->state_vector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = activeModelTranslator->state_vector.robots[0].torqueLimits[k];
                    if (U_alpha[t][i](k) < -activeModelTranslator->state_vector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = -activeModelTranslator->state_vector.robots[0].torqueLimits[k];
                }
            }

            activeModelTranslator->SetControlVector(U_alpha[t][i], i + 1);
//            Xt = activeModelTranslator->ReturnStateVector(i+1);
//            //cout << "Xt: " << Xt << endl;
//
//
            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->CostFunction(i + 1, true);
            }
            else{
                newStateCost = activeModelTranslator->CostFunction(i + 1, false);
            }

            newCosts[i] += (newStateCost * activePhysicsSimulator->returnModelTimeStep());

            activePhysicsSimulator->stepSimulator(1, i+1);

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
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    // If the cost was reduced - update all the data states
    if(newCost < oldCost){
        for(int i = 0; i < horizonLength; i++){

            activeModelTranslator->SetControlVector(U_alpha[i][bestAlphaIndex], MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            // Log the old state
            X_old.at(i + 1) = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);

            activePhysicsSimulator->copySystemState(i+1, MAIN_DATA_STATE);

            U_old[i] = U_alpha[i][bestAlphaIndex].replicate(1, 1);

        }

        MatrixXd testState = activeModelTranslator->ReturnStateVector(horizonLength - 1);
//        cout << "final state after FP: " << testState.transpose() << endl;

        return newCost;
    }

    return oldCost;
}

