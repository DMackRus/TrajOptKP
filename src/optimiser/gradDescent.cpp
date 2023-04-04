//
// Created by davidrussell on 4/4/23.
//

#include "gradDescent.h"

gradDescent::gradDescent(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, differentiator *_differentiator, int _maxHorizon, fileHandler _yamlReader) : optimiser(_modelTranslator, _physicsSimulator) {
    activeDifferentiator = _differentiator;

    maxHorizon = _maxHorizon;

    // initialise all vectors of matrices
    for(int i = 0; i < maxHorizon; i++){
        // Cost matrices
        l_x.push_back(MatrixXd(2*dof, 1));
        l_u.push_back(MatrixXd(num_ctrl, 1));

        // Dynamics derivatives matrices
        A.push_back(MatrixXd(2*dof, 2*dof));
        B.push_back(MatrixXd(2*dof, num_ctrl));

        A[i].block(0, 0, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof).setIdentity();
        A[i].block(0, dof, dof, dof) *= MUJOCO_DT;
        B[i].setZero();

        f_x.push_back(MatrixXd(2*dof, 2*dof));
        f_u.push_back(MatrixXd(2*dof, num_ctrl));

        J_u.push_back(MatrixXd(num_ctrl, 1));

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

    setIntervalMethod = _yamlReader.setIntervalMethod;
    intervalSize = _yamlReader.intervalSize;
}

double gradDescent::rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls){
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

std::vector<MatrixXd> gradDescent::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength){
    cout << " ---------------- optimisation begins -------------------" << endl;
    auto optStart = high_resolution_clock::now();

    // - Initialise variables
    std::vector<MatrixXd> optimisedControls;
    horizonLength = _horizonLength;
    double oldCost = 0.0f;
    double newCost = 0.0f;

    oldCost = rolloutTrajectory(MAIN_DATA_STATE, true, initControls);
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    for(int i = 0; i < maxIter; i++){

        auto start = high_resolution_clock::now();
        // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory
        // generate the dynamics evaluation waypoints
        std::vector<int> evaluationPoints = generateEvalWaypoints(X_old, U_old);
//         for(int j = 0; j < evaluationPoints.size(); j++){
//             cout << "eval: " << j << ": " << evaluationPoints[j] << endl;
//         }

        // Calculate derivatives via finite differnecing / analytically for cost functions if available
        getDerivativesAtSpecifiedIndices(evaluationPoints);

        // Interpolate derivatvies as required for a full set of derivatives
        interpolateDerivatives(evaluationPoints);

        auto stop = high_resolution_clock::now();
        auto linDuration = duration_cast<microseconds>(stop - start);
        cout << "number of derivatives calculated via fd: " << evaluationPoints.size() << endl;
        cout << "calc derivatives took: " << linDuration.count() / 1000000.0f << " s\n";

        // STEP 2 - Backwards Pass to calculate optimal control gradient update
        auto bp_start = high_resolution_clock ::now();
        backwardsPass();
        auto bp_stop = high_resolution_clock::now();
        auto bpDuration = duration_cast<microseconds>(bp_stop - bp_start);
        cout << "bp took: " << bpDuration.count() / 1000000.0f << " s\n";

        // STEP 3 - rollout new trajectory with line searching parameter for optimal sequence of controls
        bool costReduced;
        auto fp_start = high_resolution_clock::now();
        //newCost = forwardsPass(oldCost, costReduced);
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

        if(converged && (i >= minIter)){
            std::cout << "converged after " << i << " iterations" << std::endl;
            break;
        }
    }

    // Load the initial data back into main data
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    for(int i = 0; i < horizonLength; i++){
        optimisedControls.push_back(U_old[i]);
    }

    auto optFinish = high_resolution_clock::now();
    auto optDuration = duration_cast<microseconds>(optFinish - optStart);
    cout << "optimisation took: " << optDuration.count() / 1000000.0f << " s\n";
    cout << " ---------------- optimisation complete -------------------" << endl;

    return optimisedControls;
}

std::vector<int> gradDescent::generateEvalWaypoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls){
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

void gradDescent::getDerivativesAtSpecifiedIndices(std::vector<int> indices){
    activeDifferentiator->initModelForFiniteDifferencing();

    #pragma omp parallel for
    for(int i = 0; i < indices.size(); i++){

        int index = indices[i];
        activeDifferentiator->getDerivatives(A[i], B[i], false, index);

    }

    activeDifferentiator->resetModelAfterFiniteDifferencing();

    #pragma omp parallel for
    for(int i = 0; i < horizonLength; i++){
        MatrixXd l_uu(num_ctrl, num_ctrl);
        MatrixXd l_xx(2*dof, 2*dof);
        if(i == 0){
            activeModelTranslator->costDerivatives(X_old[0], U_old[0], X_old[0], U_old[0], l_x[i], l_xx, l_u[i], l_uu, false);
        }
        else{
            activeModelTranslator->costDerivatives(X_old[i], U_old[i], X_old[i-1], U_old[i-1], l_x[i], l_xx, l_u[i], l_uu, false);
        }

    }

    MatrixXd l_uu(num_ctrl, num_ctrl);
    MatrixXd l_xx(2*dof, 2*dof);
    activeModelTranslator->costDerivatives(X_old[horizonLength], U_old[horizonLength - 1], X_old[horizonLength - 1], U_old[horizonLength - 1],
                                           l_x[horizonLength], l_xx, l_u[horizonLength], l_uu, true);
}

void gradDescent::interpolateDerivatives(std::vector<int> calculatedIndices){

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

void gradDescent::backwardsPass(){
    MatrixXd V_x(2*dof, 2*dof);
    V_x = l_x[horizonLength];

    for(int t = horizonLength - 1; t > -1; t--){

        J_u[t] = l_u[t] + (f_u[t].transpose() * V_x);

        V_x = l_x[t] + (f_x[t].transpose() * V_x);

    }
}

double gradDescent::forwardsPass(double oldCost, bool &costReduced){

}

double gradDescent::forwardsPassParallel(double oldCost, bool &costReduced){
    double newCost = 0.0;
    bool costReduction = false;

    MatrixXd Xt(2 * dof, 1);
    MatrixXd X_last(2 * dof, 1);
    MatrixXd Ut(num_ctrl, 1);
    MatrixXd U_last(num_ctrl, 1);
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
            _U = U_old[t].replicate(1, 1);

            X_new = X_old[t].replicate(1,1);

            // Calculate new optimal controls
            U_alpha[t][i] = _U - (alphas[i] * J_u[t]);

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
