#include "interpolated_iLQR.h"

interpolatediLQR::interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, differentiator *_differentiator, int _maxHorizon, visualizer *_visualizer, fileHandler *_yamlReader) : optimiser(_modelTranslator, _physicsSimulator){

    activeDifferentiator = _differentiator;
    maxHorizon = _maxHorizon;
    activeVisualizer = _visualizer;
    activeYamlReader = _yamlReader;

    // initialise all vectors of matrices
    for(int i = 0; i < maxHorizon; i++){
        // Cost matrices
        l_x.push_back(MatrixXd(2*dof, 1));
        l_xx.push_back(MatrixXd(2*dof, 2*dof));
        l_u.push_back(MatrixXd(num_ctrl, 1));
        l_uu.push_back(MatrixXd(num_ctrl, num_ctrl));

        // Dynamics derivatives matrices
        A.push_back(MatrixXd(2*dof, 2*dof));
        B.push_back(MatrixXd(2*dof, num_ctrl));

        A[i].block(0, 0, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof) *= MUJOCO_DT;
        B[i].setZero();

        f_x.push_back(MatrixXd(2*dof, 2*dof));
        f_u.push_back(MatrixXd(2*dof, num_ctrl));

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

    intervalSize = _yamlReader->intervalSize;
    setIntervalMethod = _yamlReader->setIntervalMethod;
}

double interpolatediLQR::rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls){
    double cost = 0.0f;

    if(initialDataIndex != MAIN_DATA_STATE){
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, initialDataIndex);
    }

    MatrixXd Xt(activeModelTranslator->stateVectorSize, 1);
    MatrixXd X_last(activeModelTranslator->stateVectorSize, 1);
    MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
    MatrixXd U_last(activeModelTranslator->num_ctrl, 1);

    X_old[0] = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
    if(activePhysicsSimulator->checkIfDataIndexExists(0)){
        activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    }
    else{
        activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
    }

    for(int i = 0; i < initControls.size(); i++){
        // set controls
        activeModelTranslator->setControlVector(initControls[i], MAIN_DATA_STATE);

        // Integrate simulator
        activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        // return cost for this state
        Xt = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
        Ut = activeModelTranslator->returnControlVector(MAIN_DATA_STATE);
        double stateCost;
        
        if(i == initControls.size() - 1){
            stateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, true);
        }
        else{
            stateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, false);
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

        cost += (stateCost * MUJOCO_DT);
    }

    cout << "cost of trajectory was: " << cost << endl;

    return cost;
}

// ------------------------------------------------------------------------------------------------------
//
//  optimise - Optimise a sequence of controls for a given problem
//  @Params:
//  initialDataIndex - the data index of the system state that the optimisation problem should start from
//  initControls - The initial controls for the problem
//  maxIterations - The maximum iterations of the solver before it should return a new set of controls
//  horizonLength - How far into the future the optimiser should look when optimising the controls
//
//  @Returns:
//  optimisedControls - New optimised controls that give a lower cost than the initial controls
//
// -------------------------------------------------------------------------------------------------------
std::vector<MatrixXd> interpolatediLQR::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength){
    cout << " ---------------- optimisation begins -------------------" << endl;
    auto optStart = high_resolution_clock::now();
    
    // - Initialise variables
    std::vector<MatrixXd> optimisedControls;
    horizonLength = _horizonLength;
    lambda = 0.1;
    double oldCost = 0.0f;
    double newCost = 0.0f;


    oldCost = rolloutTrajectory(MAIN_DATA_STATE, true, initControls);
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);
    //MatrixXd initState = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
    //cout << "init state at at start of optimisation: " << initState << endl;

    // Optimise for a set number of iterations
    for(int i = 0; i < maxIter; i++){

        auto start = high_resolution_clock::now();
        // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory
        // generate the dynamics evaluation waypoints
        std::vector<int> evaluationPoints = generateEvalWaypoints(X_old, U_old);
        // for(int j = 0; j < evaluationPoints.size(); j++){
        //     cout << "eval: " << j << ": " << evaluationPoints[j] << endl;
        // }
        
        // Calculate derivatives via finite differnecing / analytically for cost functions if available
        getDerivativesAtSpecifiedIndices(evaluationPoints);

        // Interpolate derivatvies as required for a full set of derivatives
        interpolateDerivatives(evaluationPoints);

        auto stop = high_resolution_clock::now();
        auto linDuration = duration_cast<microseconds>(stop - start);
        cout << "number of derivatives calculated via fd: " << evaluationPoints.size() << endl;
        cout << "calc derivatives took: " << linDuration.count() / 1000000.0f << " s\n";

        //cout << "f_x[1900] \n" << f_x[1900] << endl;
        // cout << "f_u[1000] \n" << f_u[1000] << endl;
        // cout << "l_x[1000] \n" << l_x[1000] << endl;
        // cout << "l_xx[1000] \n" << l_xx[1000] << endl;
        // cout << "l_u[1000] \n" << l_u[1000] << endl;
        // cout << "l_uu[1000] \n" << l_uu[1000] << endl;

        // STEP 2 - BackwardsPass using the calculated derivatives to calculate an optimal feedback control law
        bool validBackwardsPass = false;
        bool lambdaExit = false;

        auto bp_start = high_resolution_clock::now();
        while(!validBackwardsPass){
            cout << "lamda is: " << lambda << "\n";
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
        cout << "bp took: " << bpDuration.count() / 1000000.0f << " s\n";

        if(!lambdaExit){
            bool costReduced;
            // STEP 3 - Forwards Pass - use the optimal control feedback law and rollout in simulation and calculate new cost of trajectory
            auto fp_start = high_resolution_clock::now();
//            newCost = forwardsPass(oldCost, costReduced);
            newCost = forwardsPassParallel(oldCost, costReduced);
            auto fp_stop = high_resolution_clock::now();
            auto fpDuration = duration_cast<microseconds>(fp_stop - fp_start);
            cout << "forward pass took: " << fpDuration.count() / 1000000.0f << " s\n";
            cout << " ---------------- new cost is: " << newCost << " -------------------" << endl;

            // STEP 4 - Check for convergence
            bool converged = checkForConvergence(oldCost, newCost);

            if(newCost < oldCost){
                oldCost = newCost;

            }
            else{
                if(lambda < maxLambda){
                    lambda *= lambdaFactor;
                    lambda *= lambdaFactor;
                }
            }


            if(converged && (i >= minIter)){
                std::cout << "converged after " << i << " iterations" << std::endl;
                break;
            }
        }
        else{
            cout << "exiting optimisation due to lambda > lambdaMax \n";
            break;
        }
    }

    // Load the initial data back into main data
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    for(int i = 0; i < horizonLength; i++){
        optimisedControls.push_back(U_old[i]);
    }

    activeYamlReader->saveTrajecInfomation(f_x, f_u, X_old, U_old, activeModelTranslator->modelName, 0);

    auto optFinish = high_resolution_clock::now();
    auto optDuration = duration_cast<microseconds>(optFinish - optStart);
    cout << "optimisation took: " << optDuration.count() / 1000000.0f << " s\n";
    cout << " ---------------- optimisation complete -------------------" << endl;

    return optimisedControls;
}
// ------------------------------------------- STEP 1 FUNCTIONS (GET DERIVATIVES) ----------------------------------------------

std::vector<int> interpolatediLQR::generateEvalWaypoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls){
    // Loop through the trajectory and decide what indices should be evaluated via finite differencing
    std::vector<int> evaluationWaypoints;
    int counter = 0;
    int numEvals = horizonLength / intervalSize;

    evaluationWaypoints.push_back(counter);

    // set-interval method
    if(setIntervalMethod){
        for(int i = 0; i < numEvals; i++){
            evaluationWaypoints.push_back(i * intervalSize);
        }
    }
    // adaptive-interval method
    else{

    }
    evaluationWaypoints.push_back(horizonLength);

    return evaluationWaypoints;
}

void interpolatediLQR::getDerivativesAtSpecifiedIndices(std::vector<int> indices){

    activeDifferentiator->initModelForFiniteDifferencing();

    #pragma omp parallel for
    for(int i = 0; i < indices.size(); i++){

        int index = indices[i];
        activeDifferentiator->getDerivatives(A[i], B[i], false, index);

    }

    activeDifferentiator->resetModelAfterFiniteDifferencing();

    #pragma omp parallel for
    for(int i = 0; i < horizonLength; i++){
        if(i == 0){
            activeModelTranslator->costDerivatives(X_old[0], U_old[0], X_old[0], U_old[0], l_x[i], l_xx[i], l_u[i], l_uu[i], false);
        }
        else{
            activeModelTranslator->costDerivatives(X_old[i], U_old[i], X_old[i-1], U_old[i-1], l_x[i], l_xx[i], l_u[i], l_uu[i], false);
        }

    }

    activeModelTranslator->costDerivatives(X_old[horizonLength], U_old[horizonLength - 1], X_old[horizonLength - 1], U_old[horizonLength - 1],
                                             l_x[horizonLength], l_xx[horizonLength], l_u[horizonLength], l_uu[horizonLength], true);

}

void interpolatediLQR::interpolateDerivatives(std::vector<int> calculatedIndices){

    // Interpolate all the derivatvies that were not calculated via finite differencing
    for(int t = 0; t < calculatedIndices.size()-1; t++){

        MatrixXd addA(2*dof, 2*dof);
        MatrixXd addB(2*dof, num_ctrl);
        int nextInterpolationSize = calculatedIndices[t+1] - calculatedIndices[t];
        int startIndex = calculatedIndices[t];

        MatrixXd startA = A[t].replicate(1, 1);
        MatrixXd endA = A[t + 1].replicate(1, 1);
        MatrixXd diffA = endA - startA;
        addA = diffA / nextInterpolationSize;

        MatrixXd startB = B[t].replicate(1, 1);
        MatrixXd endB = B[t + 1].replicate(1, 1);
        MatrixXd diffB = endB - startB;
        addB = diffB / nextInterpolationSize;


        // Interpolate A and B matrices
        for(int i = 0; i < nextInterpolationSize; i++){
            f_x[startIndex + i] = A[t].replicate(1,1) + (addA * i);
            f_u[startIndex + i] = B[t].replicate(1,1) + (addB * i);
        }
    }

    f_x[horizonLength - 1] = f_x[horizonLength - 2].replicate(1,1);
    f_u[horizonLength - 1] = f_u[horizonLength - 2].replicate(1,1);

    f_x[horizonLength] = f_x[horizonLength - 1].replicate(1,1);
    f_u[horizonLength] = f_u[horizonLength - 1].replicate(1,1);
}


// ------------------------------------------- STEP 2 FUNCTIONS (BACKWARDS PASS) ----------------------------------------------
bool interpolatediLQR::backwardsPass_Quu_reg(){
    MatrixXd V_x(2*dof, 2*dof);
    V_x = l_x[horizonLength];
    MatrixXd V_xx(2*dof, 2*dof);
    V_xx = l_xx[horizonLength];
    int Quu_pd_check_counter = 0;
    int number_steps_between_pd_checks = 100;

    for(int t = horizonLength - 1; t > -1; t--){
        MatrixXd Q_x(2*dof, 1);
        MatrixXd Q_u(num_ctrl, 1);
        MatrixXd Q_xx(2*dof, 2*dof);
        MatrixXd Q_uu(num_ctrl, num_ctrl);
        MatrixXd Q_ux(num_ctrl, 2*dof);

        Quu_pd_check_counter++;

        Q_u = l_u[t] + (f_u[t].transpose() * V_x);

        Q_x = l_x[t] + (f_x[t].transpose() * V_x);

        Q_ux = (f_u[t].transpose() * (V_xx * f_x[t]));

        Q_uu = l_uu[t] + (f_u[t].transpose() * (V_xx * f_u[t]));

        Q_xx = l_xx[t] + (f_x[t].transpose() * (V_xx * f_x[t]));

        MatrixXd Q_uu_reg = Q_uu.replicate(1, 1);

        for(int i = 0; i < Q_uu.rows(); i++){
            Q_uu_reg(i, i) += lambda;
        }

        if(Quu_pd_check_counter >= number_steps_between_pd_checks){
            if(!isMatrixPD(Q_uu_reg)){
                cout << "iteration " << t << endl;
                cout << "f_x[t - 3] " << f_x[t - 3] << endl;
                cout << "f_x[t - 2] " << f_x[t - 2] << endl;
                cout << "f_x[t - 1] " << f_x[t - 1] << endl;
                cout << "f_x[t] " << f_x[t] << endl;
                cout << "Q_uu_reg " << Q_uu_reg << endl;
                return false;
            }
            Quu_pd_check_counter = 0;
        }

        auto temp = (Q_uu_reg).ldlt();
        MatrixXd I(num_ctrl, num_ctrl);
        I.setIdentity();
        MatrixXd Q_uu_inv = temp.solve(I);

        k[t] = -Q_uu_inv * Q_u;
        K[t] = -Q_uu_inv * Q_ux;

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
double interpolatediLQR::forwardsPass(double oldCost, bool &costReduced){
    float alpha = 1.0;
    double newCost = 0.0;
    bool costReduction = false;
    int alphaCount = 0;
    float alphaReduc = 0.1;
    int alphaMax = (1 / alphaReduc) - 1;

    MatrixXd Xt(2 * dof, 1);
    MatrixXd X_last(2 * dof, 1);
    MatrixXd Ut(num_ctrl, 1);
    MatrixXd U_last(num_ctrl, 1);

    while(!costReduction){

        // Copy intial data state into main data state for rollout
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        newCost = 0;
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);

//        #pragma omp parallel for
        for(int t = 0; t < horizonLength; t++) {
            // Step 1 - get old state and old control that were linearised around
            _X = activeModelTranslator->returnStateVector(t);
            //_U = activeModelTranslator->returnControlVector(t);
            _U = U_old[t].replicate(1, 1);

            X_new = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
            // Calculate difference from new state to old state
            stateFeedback = X_new - _X;

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
            U_new[t] = _U + (alpha * k[t]) + feedBackGain;

            // Clamp torque within limits
            if(activeModelTranslator->myStateVector.robots[0].torqueControlled){
                for(int i = 0; i < num_ctrl; i++){
                    if (U_new[t](i) > activeModelTranslator->myStateVector.robots[0].torqueLimits[i]) U_new[t](i) = activeModelTranslator->myStateVector.robots[0].torqueLimits[i];
                    if (U_new[t](i) < -activeModelTranslator->myStateVector.robots[0].torqueLimits[i]) U_new[t](i) = -activeModelTranslator->myStateVector.robots[0].torqueLimits[i];
                }

            }

//            cout << "old control: " << endl << U_old[t] << endl;
//            cout << "state feedback" << endl << stateFeedback << endl;
//            cout << "new control: " << endl << U_new[t] << endl;

            activeModelTranslator->setControlVector(U_new[t], MAIN_DATA_STATE);
            Xt = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

            Ut = U_new[t].replicate(1, 1);

            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, true);
            }
            else{
                newStateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last, false);
            }

            newCost += (newStateCost * MUJOCO_DT);

            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

             if(t % 20 == 0){
                 const char* fplabel = "fp";
                 activeVisualizer->render(fplabel);
             }

            X_last = Xt.replicate(1, 1);
            U_last = Ut.replicate(1, 1);

        }

        cout << "cost from alpha: " << alphaCount << ": " << newCost << endl;

        if(newCost < oldCost){
            costReduction = true;
            costReduced = true;
        }
        else{
            alpha = alpha - 0.1;
            alphaCount++;
            if(alphaCount >= alphaMax){
                break;
            }
        }
    }

    // If the cost was reduced
    if(newCost < oldCost){
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

        for(int i = 0; i < horizonLength; i++){

            activeModelTranslator->setControlVector(U_new[i], MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            // Log the old state
             X_old.at(i + 1) = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

             activePhysicsSimulator->copySystemState(i+1, MAIN_DATA_STATE);

             U_old[i] = U_new[i].replicate(1, 1);

        }

        return newCost;
    }

    return oldCost;
}

double interpolatediLQR::forwardsPassParallel(double oldCost, bool &costReduced){
    float alpha = 1.0;
    double newCost = 0.0;
    bool costReduction = false;
    int alphaCount = 0;
    float alphaReduc = 0.1;
    int alphaMax = (1 / alphaReduc) - 1;

    MatrixXd Xt(2 * dof, 1);
    MatrixXd X_last(2 * dof, 1);
    MatrixXd Ut(num_ctrl, 1);
    MatrixXd U_last(num_ctrl, 1);
//    double alphas[8] = {0.125, 0.25, 0.375, 0.5, 0.675, 0.75, 0.875, 1.0};
    double alphas[8] = {1.0, 0.875, 0.75, 0.675, 0.5, 0.375, 0.25, 0.125};
    double newCosts[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for(int i = 0; i < 8; i++){
        activePhysicsSimulator->copySystemState(i+1, 0);
    }

    MatrixXd initState = activeModelTranslator->returnStateVector(1);

    #pragma omp parallel for
    for(int i = 0; i < 8; i++){
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);

        for(int t = 0; t < horizonLength; t++) {
            // Step 1 - get old state and old control that were linearised around
            _X = X_old[t].replicate(1, 1);
            //_U = activeModelTranslator->returnControlVector(t);
            _U = U_old[t].replicate(1, 1);

            X_new = activeModelTranslator->returnStateVector(i+1);

            // Calculate difference from new state to old state
            stateFeedback = X_new - _X;

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
            U_alpha[t][i] = _U + (alphas[i] * k[t]) + feedBackGain;

            // Clamp torque within limits
            if(activeModelTranslator->myStateVector.robots[0].torqueControlled){
                for(int k = 0; k < num_ctrl; k++){
                    if (U_alpha[t][i](k) > activeModelTranslator->myStateVector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = activeModelTranslator->myStateVector.robots[0].torqueLimits[k];
                    if (U_alpha[t][i](k) < -activeModelTranslator->myStateVector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = -activeModelTranslator->myStateVector.robots[0].torqueLimits[k];
                }
            }

            activeModelTranslator->setControlVector(U_alpha[t][i], i+1);
//            Xt = activeModelTranslator->returnStateVector(i+1);
//            //cout << "Xt: " << Xt << endl;
//
            Ut = U_alpha[t][i].replicate(1, 1);
//
            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->costFunction(X_new, Ut, X_last, U_last, true);
            }
            else{
                newStateCost = activeModelTranslator->costFunction(X_new, Ut, X_last, U_last, false);
            }

            newCosts[i] += (newStateCost * MUJOCO_DT);

            activePhysicsSimulator->stepSimulator(1, i+1);

//             if(t % 20 == 0){
//                 const char* fplabel = "fp";
//                 activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, i+1);
//
//                 activeVisualizer->render(fplabel);
//             }

            X_last = Xt.replicate(1, 1);
            U_last = Ut.replicate(1, 1);
        }
    }

    double bestAlphaCost = newCosts[0];
    int bestAlphaIndex = 0;
    for(int i = 0; i < 8; i++){
        if(newCosts[i] < bestAlphaCost){
            bestAlphaCost = newCosts[i];
            bestAlphaIndex = i;
        }
    }

    newCost = bestAlphaCost;
    cout << "best alpha cost = " << bestAlphaCost << " at alpha: " << alphas[bestAlphaIndex] << endl;
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    // If the cost was reduced
    if(newCost < oldCost){
        initState = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

        for(int i = 0; i < horizonLength; i++){

            activeModelTranslator->setControlVector(U_new[i], MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            // Log the old state
            X_old.at(i + 1) = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

            activePhysicsSimulator->copySystemState(i+1, MAIN_DATA_STATE);

            U_old[i] = U_alpha[i][bestAlphaIndex].replicate(1, 1);

        }

        return newCost;
    }

    return oldCost;
}

