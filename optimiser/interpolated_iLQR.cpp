#include "interpolated_iLQR.h"

interpolatediLQR::interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator) : optimiser(_modelTranslator, _physicsSimulator){

}

double interpolatediLQR::rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls){
    double cost = 0.0f;

    if(initialDataIndex != MAIN_DATA_STATE){
        activePhysicsSimulator->loadSystemStateFromIndex(MAIN_DATA_STATE, initialDataIndex);
    }

    MatrixXd Xt(activeModelTranslator->stateVectorSize, 1);
    MatrixXd X_last(activeModelTranslator->stateVectorSize, 1);
    MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
    MatrixXd U_last(activeModelTranslator->num_ctrl, 1);

    for(int i = 0; i < initControls.size(); i++){
        // set controls
        activeModelTranslator->setControlVector(initControls[i], MAIN_DATA_STATE);

        // Integrate simulator
        activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);

        // return cost for this state
        Xt = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);
        Ut = activeModelTranslator->returnControlVector(MAIN_DATA_STATE);
        double stateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last);

        // If required to save states to trajectoy tracking, then save state
        if(saveStates){
            if(activePhysicsSimulator->checkIfDataIndexExists(i)){
                cout << "data " << i << "exists \n";
                activePhysicsSimulator->saveSystemStateToIndex(MAIN_DATA_STATE, i);
            }
            else{
                activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
            }
            
        }

        cout << "xt: " << Xt << endl;

        cost += (stateCost * 0.004);


    }


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
std::vector<MatrixXd> interpolatediLQR::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int _horizonLength){
    
    // - Initialise variables
    std::vector<MatrixXd> optimisedControls;
    horizonLength = _horizonLength;

    // Optimise for a set number of iterations
    for(int i = 0; i < maxIterations; i++){

        // STEP 1 - Linearise dynamics and calculate first + second order cost derivatives for current trajectory

        // generate the dynamics evaluation waypoints
        
        // Calculate derivatives via finite differnecing / analytically for cost functions if available

        // Interpolate derivatvies as required for a full set of derivatives

        // STEP 2 - BackwardsPass using the calculated derivatives to calculate an optimal feedback control law
        bool validBackwardsPass = false;
        bool lambdaExit = false;

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

        // STEP 3 - Forwards Pass - use the optimal control feedback law and rollout in simulation and calculate new cost of trajectory

        // STEP 4 - Check for convergence
    }



    return optimisedControls;
}
// ------------------------------------------- STEP 1 FUNCTIONS (GET DERIVATIVES) ----------------------------------------------

std::vector<int> interpolatediLQR::generateEvalWaypoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls){
    // Loop through the trajectory and decide what indices should be evaluated via finite differencing

    // set-interval method

    // adaptive-interval method
}

void interpolatediLQR::getDerivativesAtSpecifiedIndices(){

    // Calculate dynamics derivatives at specified indices

    // calculate all cost derivatives if analytical or only specified ones if by finite differencing

}

void interpolatediLQR::interpolateDerivatives(std::vector<int> calculatedIndices){

    // Interpolate all the derivatvies that were not calculated via finite differencing

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

//        cout << "------------------ iteration " << t << " ------------------" << endl;
//        cout << "l_x " << l_x[t] << endl;
//        cout << "l_xx " << l_xx[t] << endl;
//        cout << "l_u " << l_u[t] << endl;
//        cout << "l_uu " << l_uu[t] << endl;
//        cout << "Q_ux " << Q_ux << endl;
//        cout << "f_u[t] " << f_u[t] << endl;
//        cout << "Q_uu " << Q_uu << endl;
//        cout << "Q_uu_inv " << Q_uu_inv << endl;
//        cout << "Q_x " << Q_x << endl;
//        cout << "Q_xx " << Q_xx << endl;
//        cout << "V_xx " << V_xx << endl;
//        cout << "V_x " << V_x << endl;
//        cout << "K[t] " << K[t] << endl;
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
        activePhysicsSimulator->loadSystemStateFromIndex(MAIN_DATA_STATE, 0);
        
        newCost = 0;
        MatrixXd stateFeedback(2*dof, 1);
        MatrixXd _X(2*dof, 1);
        MatrixXd X_new(2*dof, 1);
        MatrixXd _U(num_ctrl, 1);

        for(int t = 0; t < horizonLength; t++) {
            // Step 1 - get old state and old control that were linearised around
            _X = activeModelTranslator->returnStateVector(t);
            _U = activeModelTranslator->returnControlVector(t);

            X_new = activeModelTranslator->returnStateVector(MAIN_DATA_STATE);

            // Calculate difference from new state to old state
            stateFeedback = X_new - _X;

            MatrixXd feedBackGain = K[t] * stateFeedback;

            // Calculate new optimal controls
            U_new[t] = _U + (alpha * k[t]) + feedBackGain;

            // Clamp torques within torque limits
            // if (TORQUE_CONTROL) {
            //     for (int k = 0; k < NUM_CTRL; k++) {
            //         if (U_new[t](k) > modelTranslator->torqueLims[k]) U_new[t](k) = modelTranslator->torqueLims[k];
            //         if (U_new[t](k) < -modelTranslator->torqueLims[k]) U_new[t](k) = -modelTranslator->torqueLims[k];
            //     }
            // }

//            cout << "old control: " << endl << U_old[t] << endl;
////            cout << "state feedback" << endl << stateFeedback << endl;
//            cout << "new control: " << endl << U_new[t] << endl;

            activeModelTranslator->setControlVector(U_new[t], MAIN_DATA_STATE);

            double newStateCost;
            newStateCost = activeModelTranslator->costFunction(Xt, Ut, X_last, U_last);

            newCost += (newStateCost * MUJOCO_DT);

            activePhysicsSimulator->stepSimulator(1, MAIN_DATA_STATE);
        }

        //cout << "cost from alpha: " << alphaCount << ": " << newCost << endl;

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
//     if(newCost < oldCost){

//         // Copy initial data to main data
//         cpMjData(model, mdata, d_init);

// //        for(int k = 0; k < NUM_CTRL; k++){
// //            mdata->ctrl[k] = U_new[0](k);
// //        }
//         modelTranslator->setControls(mdata, U_new.at(0), grippersOpen_iLQR[0]);

//         cpMjData(model, dArray[0], mdata);

//         for(int i = 0; i < ilqr_horizon_length; i++){

//             X_final[i] = modelTranslator->returnState(mdata);
//             modelTranslator->setControls(mdata, U_new.at(i), grippersOpen_iLQR[i]);
//             X_old.at(i) = modelTranslator->returnState(mdata);

//             modelTranslator->stepModel(mdata, 1);
//             cpMjData(model, dArray[i + 1], mdata);
//         }

//         //cout << "best alpha was " << alpha << endl;
//         //cout << "cost improved in F.P - new cost: " << newCost << endl;
//         m_state termStateBest = modelTranslator->returnState(dArray[ilqr_horizon_length]);
//         return newCost;
//     }

    return oldCost;
}