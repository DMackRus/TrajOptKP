//
// Created by davidrussell on 4/4/23.
//

#include "GradDescent.h"

GradDescent::GradDescent(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<PhysicsSimulator> _physicsSimulator, std::shared_ptr<Differentiator> _differentiator, std::shared_ptr<Visualiser> _visualizer, int _maxHorizon, std::shared_ptr<FileHandler> _yamlReader) : Optimiser(_modelTranslator, _physicsSimulator, _yamlReader, _differentiator){
//    activeDifferentiator = _differentiator;
    activeVisualizer = _visualizer;

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
        A[i].block(0, dof, dof, dof) *= activePhysicsSimulator->returnModelTimeStep();
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

    //setIntervalMethod = _yamlReader->keyPointMethod;
    intervalSize = _yamlReader->minInterval;
}

double GradDescent::RolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls){
    double cost = 0.0f;

    if(initialDataIndex != MAIN_DATA_STATE){
        activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, initialDataIndex);
    }

    MatrixXd Xt(activeModelTranslator->state_vector_size, 1);
    MatrixXd X_last(activeModelTranslator->state_vector_size, 1);
    MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
    MatrixXd U_last(activeModelTranslator->num_ctrl, 1);

    X_old[0] = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);
    if(activePhysicsSimulator->checkIfDataIndexExists(0)){
        activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    }
    else{
        activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
    }

    for(int i = 0; i < initControls.size(); i++){
        // set controls
        activeModelTranslator->SetControlVector(initControls[i], MAIN_DATA_STATE);

        // Integrate simulator
        activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        // return cost for this state
        Xt = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);
        Ut = activeModelTranslator->ReturnControlVector(MAIN_DATA_STATE);
        double stateCost;

        if(i == initControls.size() - 1){
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

        cost += (stateCost * activePhysicsSimulator->returnModelTimeStep());
    }

    cout << "cost of trajectory was: " << cost << endl;

    return cost;
}

std::vector<MatrixXd> GradDescent::Optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength){
    cout << " ---------------- optimisation begins -------------------" << endl;
    auto optStart = high_resolution_clock::now();

    // - Initialise variables
    std::vector<MatrixXd> optimisedControls;
    horizonLength = _horizonLength;
    double oldCost = 0.0f;
    double newCost = 0.0f;

    oldCost = RolloutTrajectory(MAIN_DATA_STATE, true, initControls);
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);

    for(int i = 0; i < maxIter; i++){

        auto start = high_resolution_clock::now();
        // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory
        // generate the dynamics evaluation waypoints

        // TODO - use Optimiser class get derivatives function
        GenerateDerivatives();

        auto stop = high_resolution_clock::now();
        auto linDuration = duration_cast<microseconds>(stop - start);
//        cout << "number of derivatives calculated via fd: " << evaluationPoints.size() << endl;
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
//        newCost = ForwardsPass(oldCost, costReduced);
        newCost = forwardsPassParallel(oldCost, costReduced);
        auto fp_stop = high_resolution_clock::now();
        auto fpDuration = duration_cast<microseconds>(fp_stop - fp_start);
        cout << "forward pass took: " << fpDuration.count() / 1000000.0f << " s\n";
        cout << " ---------------- new cost is: " << newCost << " -------------------" << endl;

        // STEP 4 - Check for convergence
        bool converged = CheckForConvergence(oldCost, newCost);

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

void GradDescent::backwardsPass(){
    MatrixXd V_x(2*dof, 1);
    V_x = l_x[horizonLength];

    MatrixXd Q_x(2*dof, 1);
    MatrixXd Q_u(num_ctrl, 1);

    for(int t = horizonLength - 1; t > -1; t--){

//        // Q_x = l_x + A*V_x
//        Q_x = l_x[t] + (f_x[t].transpose() * V_x);
//
//        Q_u = l_u[t] + (f_u[t].transpose() * V_x);
//
//        // k = -Q_u;
//        J_u[t] = -Q_u;
//
//        // Update cost to go
//        V_x = Q_x.replicate(1,1);

        J_u[t] = l_u[t] + (f_u[t].transpose() * V_x.replicate(1, 1));

        V_x = l_x[t] + (f_x[t].transpose() * V_x);

    }
}

double GradDescent::forwardsPass(double oldCost, bool &costReduced){
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
            _X = activeModelTranslator->ReturnStateVector(t);
            //_U = activeModelTranslator->ReturnControlVector(t);
            _U = U_old[t].replicate(1, 1);

            X_new = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);

            // Calculate new optimal controls
            U_new[t] = _U + (alpha * J_u[t]);

            // Clamp torque within limits
            if(activeModelTranslator->state_vector.robots[0].torqueControlled){
                for(int i = 0; i < num_ctrl; i++){
                    if (U_new[t](i) > activeModelTranslator->state_vector.robots[0].torqueLimits[i]) U_new[t](i) = activeModelTranslator->state_vector.robots[0].torqueLimits[i];
                    if (U_new[t](i) < -activeModelTranslator->state_vector.robots[0].torqueLimits[i]) U_new[t](i) = -activeModelTranslator->state_vector.robots[0].torqueLimits[i];
                }

            }

//            cout << "old control: " << endl << U_old[t] << endl;
//            cout << "state feedback" << endl << stateFeedback << endl;
//            cout << "new control: " << endl << U_new[t] << endl;

            activeModelTranslator->SetControlVector(U_new[t], MAIN_DATA_STATE);
            Xt = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);

            Ut = U_new[t].replicate(1, 1);

            double newStateCost;
            // Terminal state
            if(t == horizonLength - 1){
                newStateCost = activeModelTranslator->CostFunction(MAIN_DATA_STATE, true);
            }
            else{
                newStateCost = activeModelTranslator->CostFunction(MAIN_DATA_STATE, false);
            }

            newCost += (newStateCost * activePhysicsSimulator->returnModelTimeStep());

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

            activeModelTranslator->SetControlVector(U_new[i], MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            // Log the old state
            X_old.at(i + 1) = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);

            activePhysicsSimulator->copySystemState(i+1, MAIN_DATA_STATE);

            U_old[i] = U_new[i].replicate(1, 1);

        }

        return newCost;
    }

    return oldCost;
}

double GradDescent::forwardsPassParallel(double oldCost, bool &costReduced){
    double newCost = 0.0;
    bool costReduction = false;

//    double alphas[8] = {1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 32e-6, 64e-6, 128e-6};
    double alphas[8] = {1e-4, 5e-4, 1e-3, 5e-3, 1e-2, 5e-2, 1e-1, 1};
//    double alphas[8] = {1e-8, 5e-8, 1e-7, 5e-7, 1e-6, 5e-6, 1e-5, 5e-4};
//    double alphas[8] = {1e-20, 5e-16, 1e-9, 5e-9, 1e-8, 5e-8, 1e-5, 0};
    double newCosts[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for(int i = 0; i < 8; i++){
        activePhysicsSimulator->copySystemState(i+1, 0);
    }

    #pragma omp parallel for
    for(int i = 0; i < 8; i++){
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);
        MatrixXd Xt(2 * dof, 1);
        MatrixXd X_last(2 * dof, 1);
        MatrixXd Ut(num_ctrl, 1);
        MatrixXd U_last(num_ctrl, 1);

        for(int t = 0; t < horizonLength; t++) {
            _U = U_old[t].replicate(1, 1);

            // Calculate new optimal controls
            U_alpha[t][i] = _U - (alphas[i] * J_u[t]);

            // Clamp torque within limits
            if(activeModelTranslator->state_vector.robots[0].torqueControlled){
                for(int k = 0; k < num_ctrl; k++){
                    if (U_alpha[t][i](k) > activeModelTranslator->state_vector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = activeModelTranslator->state_vector.robots[0].torqueLimits[k];
                    if (U_alpha[t][i](k) < -activeModelTranslator->state_vector.robots[0].torqueLimits[k]) U_alpha[t][i](k) = -activeModelTranslator->state_vector.robots[0].torqueLimits[k];
                }
            }

            activeModelTranslator->SetControlVector(U_alpha[t][i], i + 1);
            Xt = activeModelTranslator->ReturnStateVector(i + 1);
//            //cout << "Xt: " << Xt << endl;
//
            Ut = U_alpha[t][i].replicate(1, 1);
//            cout << "old U: " << _U << endl;
//            cout << "New U: " << Ut << endl;
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

        for(int i = 0; i < horizonLength; i++){

            activeModelTranslator->SetControlVector(U_alpha[i][bestAlphaIndex], MAIN_DATA_STATE);
            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

            // Log the old state
            X_old.at(i + 1) = activeModelTranslator->ReturnStateVector(MAIN_DATA_STATE);

            activePhysicsSimulator->copySystemState(i+1, MAIN_DATA_STATE);

            U_old[i] = U_alpha[i][bestAlphaIndex].replicate(1, 1);

        }

        return newCost;
    }

    return oldCost;
}
