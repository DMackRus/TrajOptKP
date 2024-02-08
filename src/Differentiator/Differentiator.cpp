#include "Differentiator.h"

Differentiator::Differentiator(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> _physicsSimulator){
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;

}

// Hack to prevent strange segmentation faults I was epxeriecning when running my code with O3 optimisation flag.
// If anyone can find a way to remove these pragma commands without it breaking the code that would be fantastic  ... :).
#pragma GCC push_options
#pragma GCC optimize ("O0")
void Differentiator::getDerivatives(MatrixXd &A, MatrixXd &B, std::vector<int> cols,
                                    MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu,
                                    bool costDerivs, int dataIndex, bool terminal, int threadId){
    int dof = activeModelTranslator->dof;
    int numCtrl = activeModelTranslator->num_ctrl;
    int tid = threadId;

    // This seems random, in Optimiser we define -1 = "mainData", -2 = "masterData",
    // -3 = "visualisation data", 0 -> horizon Length = "stored trajectory data"
    // So we need values below -3 for finite-differencing data
    int physicsHelperId = -4 - tid;

    // Initialise sub matrices of A and B matrix
    // ------------ dof x dof ----------------
    MatrixXd dqveldq(dof, dof);
    MatrixXd dqaccdq(dof, dof);
    MatrixXd dqveldqvel(dof, dof);
    MatrixXd dqaccdqvel(dof, dof);

    // ------------ dof x ctrl --------------
    MatrixXd dqveldctrl(dof, numCtrl);
    MatrixXd dqaccdctrl(dof, numCtrl);

    // ---------- 1 x dof -------------------
    MatrixXd dcostdctrl(numCtrl, 1);
    MatrixXd dcostdpos(dof, 1);
    MatrixXd dcostdvel(dof, 1);

    double costInc = 0.0f;
    double costDec = 0.0f;

    // Allocate memory for variables
    activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
    activePhysicsSimulator->forwardSimulator(physicsHelperId);

    if(USE_DQACC){
        dqaccdctrl = calc_dqaccdctrl(cols, dataIndex, physicsHelperId, dcostdctrl, costDerivs, terminal);

        dqaccdqvel = calc_dqaccdqvel(cols, dataIndex, physicsHelperId, dcostdvel, costDerivs, terminal);

        dqaccdq = calc_dqaccdqpos(cols, dataIndex, physicsHelperId, dcostdpos, costDerivs, terminal);
    }
    else{
        dqveldctrl = calc_dqveldctrl(cols, dataIndex, physicsHelperId, dcostdctrl, costDerivs, terminal);

        dqveldq = calc_dqveldqpos(cols, dataIndex, physicsHelperId, dcostdpos, costDerivs, terminal);

        dqveldqvel = calc_dqveldqvel(cols, dataIndex, physicsHelperId, dcostdvel, costDerivs, terminal);
    }


    if(costDerivs) {
        l_x.block(0, 0, dof, 1) = dcostdpos;
        l_x.block(dof, 0, dof, 1) = dcostdvel;

        // ------------- l_u / l_uu -------------------
        //                dqcostdctrl
        // --------------------------------------------
        l_u.block(0, 0, numCtrl, 1) = dcostdctrl;
    }

    MatrixXd l_x_inc(dof*2, 1);
    MatrixXd l_x_dec(dof*2, 1);
    MatrixXd l_u_inc(numCtrl, 1);
    MatrixXd l_u_dec(numCtrl, 1);

    MatrixXd currentState = activeModelTranslator->ReturnStateVector(physicsHelperId);
    MatrixXd currentControl = activeModelTranslator->ReturnControlVector(physicsHelperId);

    if(costDerivs && !HESSIAN_APPROXIMATION) {
        double epsCost = 1e-6;
        double epsTest = 1e-6;

        MatrixXd test1;
        MatrixXd test2;

        for(int i = 0; i < 2*dof; i++){
            MatrixXd perturbedState = currentState.replicate(1, 1);
            perturbedState(i) += epsCost;

            for(int j = 0; j < 2*dof; j++){
//                cout << "perturb state index " << i << " with state index " << j << endl;
                MatrixXd stateInc = perturbedState.replicate(1,1);
                stateInc(j) += epsTest;
//                cout << "state inc pos: " << endl << stateInc << endl;

                activeModelTranslator->SetStateVector(stateInc, physicsHelperId);
//                active_physics_simulator->forwardSimulator(physicsHelperId);
//                active_physics_simulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_NONE, 0);
                activePhysicsSimulator->stepSimulator(1, physicsHelperId);
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                test1 = activeModelTranslator->ReturnStateVector(physicsHelperId);
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                MatrixXd stateDec = perturbedState.replicate(1,1);
                stateDec(j) -= epsTest;
//                cout << "state dec: " << endl << stateDec << endl;

                activeModelTranslator->SetStateVector(stateDec, physicsHelperId);
//                active_physics_simulator->forwardSimulator(physicsHelperId);
//                active_physics_simulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_NONE, 0);
                activePhysicsSimulator->stepSimulator(1, physicsHelperId);
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                test2 = activeModelTranslator->ReturnStateVector(physicsHelperId);
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
//                cout << "difference in returned states: " << endl << test1 - test2 << endl;
//                cout << "cost inc: " << costInc << endl;
//                cout << "cost dec: " << costDec << endl;

                l_x_inc(j, 0) = (costInc - costDec)/(2*epsTest);
            }

//            cout << "perturbed state pos: " << endl << perturbedState << endl;
//            cout << "l_x_inc: " << endl << l_x_inc << endl;

            perturbedState = currentState.replicate(1, 1);
            perturbedState(i) -= epsCost;

            for(int j = 0; j < 2*dof; j++){
                MatrixXd stateInc = perturbedState.replicate(1,1);
                stateInc(j) += epsTest;

                activeModelTranslator->SetStateVector(stateInc, physicsHelperId);
//                active_physics_simulator->forwardSimulator(physicsHelperId);
                activePhysicsSimulator->stepSimulator(1, physicsHelperId);
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                MatrixXd stateDec = perturbedState.replicate(1,1);
                stateDec(j) -= epsTest;

                activeModelTranslator->SetStateVector(stateDec, physicsHelperId);
//                active_physics_simulator->forwardSimulator(physicsHelperId);
                activePhysicsSimulator->stepSimulator(1, physicsHelperId);
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                l_x_dec(j, 0) = (costInc - costDec)/(2*epsTest);
            }

//            cout << "perturbed state neg: " << endl << perturbedState << endl;
//            cout << "l_x_dec: " << endl << l_x_dec << endl;

            // New l_x at perturbed position i
            for(int j = 0; j < 2*dof; j++){
                l_xx(j, i) = (l_x_inc(j, 0) - l_x_dec(j, 0))/(2*epsCost);
            }
        }

        for(int i = 0; i < numCtrl; i++){
            MatrixXd perturbedControl = currentControl.replicate(1, 1);
            perturbedControl(i) += epsCost;

            for(int j = 0; j < numCtrl; j++){
                MatrixXd controlInc = perturbedControl.replicate(1,1);
                controlInc(j) += epsCost;

                activeModelTranslator->SetControlVector(controlInc, physicsHelperId);

                activePhysicsSimulator->stepSimulator(1, physicsHelperId);
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                MatrixXd controlDec = perturbedControl.replicate(1,1);
                controlDec(j) -= epsCost;

                activeModelTranslator->SetControlVector(controlDec, physicsHelperId);

                activePhysicsSimulator->stepSimulator(1, physicsHelperId);
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                l_u_inc(j, 0) = (costInc - costDec)/(2*epsCost);
            }

            perturbedControl = currentControl.replicate(1, 1);
            perturbedControl(i) -= epsCost;

            for(int j = 0; j < numCtrl; j++){
                MatrixXd controlInc = perturbedControl.replicate(1,1);
                controlInc(j) += epsCost;

                activeModelTranslator->SetControlVector(controlInc, physicsHelperId);

                activePhysicsSimulator->stepSimulator(1, physicsHelperId);
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                MatrixXd controlDec = perturbedControl.replicate(1,1);
                controlDec(j) -= epsCost;

                activeModelTranslator->SetControlVector(controlDec, physicsHelperId);

                activePhysicsSimulator->stepSimulator(1, physicsHelperId);
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                l_u_dec(j, 0) = (costInc - costDec)/(2*epsCost);
            }

            for(int j = 0; j < numCtrl; j++){
                l_uu(j, i) = (l_u_inc(j, 0) - l_u_dec(j, 0))/(2*epsCost);
            }
        }
    }
//    cout << "l_x: " << endl << l_x << endl;
//    cout << "l_xx: " << endl << l_xx << endl;

    if(costDerivs && HESSIAN_APPROXIMATION){
        l_xx = l_x * l_x.transpose();
        l_uu = l_u * l_u.transpose();
    }
    // need to set this as zero when reaching but not when pendulum????
    //dqaccdq.setZero();

    // ------------ A -----------------
    // dqposdqpos       dqposdqvel
    //
    // dqveldqpos       dqveldqvel
    // --------------------------------

    for(int i = 0; i < cols.size(); i++){
        if(USE_DQACC){
            A.block(dof, cols[i], dof, 1) = dqaccdq.block(0, cols[i], dof, 1) * activePhysicsSimulator->returnModelTimeStep();
            for(int j = 0; j < dof; j ++){
                if(j == cols[i]){
                    A(dof + j, cols[i] + dof) = 1 + (dqaccdqvel(j, cols[i]) * activePhysicsSimulator->returnModelTimeStep());
                }
                else{
                    A(dof + j, cols[i] + dof) = dqaccdqvel(j, cols[i]) * activePhysicsSimulator->returnModelTimeStep();
                }

            }
        }
        else{
            A.block(dof, cols[i], dof, 1) = dqveldq.block(0, cols[i], dof, 1);
            A.block(dof, cols[i] + dof, dof, 1) = dqveldqvel.block(0, cols[i], dof, 1);
        }

    }

    // ------------- B -------------------
    //          dqposdctrl
    //          dqveldctrl
    // ----------------------------------
    for(int i = 0; i < cols.size(); i++){
        if(cols[i] < numCtrl){
            if(USE_DQACC){
                B.block(dof, cols[i], dof, 1) = dqaccdctrl.block(0, cols[i], dof, 1) * activePhysicsSimulator->returnModelTimeStep();
            }
            else{
                B.block(dof, cols[i], dof, 1) = dqveldctrl.block(0, cols[i], dof, 1);
            }
        }
    }
}


MatrixXd Differentiator::calc_dqveldctrl(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal){
    int numCtrl = activeModelTranslator->num_ctrl;
    int dof = activeModelTranslator->dof;
    MatrixXd velocityInc(dof, 1);
    MatrixXd velocityDec(dof, 1);
    MatrixXd dqveldctrl(dof, numCtrl);
    double costInc;
    double costDec;

    MatrixXd unperturbedControls = activeModelTranslator->ReturnControlVector(physicsHelperId);
    for(int i = 0; i < numCtrl; i++){
        bool computeColumn = false;
        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                computeColumn = true;
            }
        }

        if(computeColumn){
            // perturb control vector positively
            MatrixXd perturbedControls = unperturbedControls.replicate(1,1);
            perturbedControls(i) += epsControls;
            activeModelTranslator->SetControlVector(perturbedControls, physicsHelperId);

            // Integrate the simulator
//            mju_copy(active_physics_simulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);
//            cout << "after first step simulator - " << i << endl;

            // return the  new velcoity vector
            velocityInc = activeModelTranslator->returnVelocityVector(physicsHelperId);

            // If calculating cost derivatives
            if(fd_costDerivs){
                // Calculate cost
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // return data state back to initial data state
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            // perturb control vector in opposite direction
            perturbedControls = unperturbedControls.replicate(1, 1);
            perturbedControls(i) -= epsControls;

            // integrate simulator
//            mju_copy(active_physics_simulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);
//            cout << "after second step simulator - " << i << endl;

            // return the new velocity vector
            velocityDec = activeModelTranslator->returnVelocityVector(physicsHelperId);

            // If calculating cost derivatives via finite-differencing
            if(fd_costDerivs){
                // Calculate cost
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // Calculate one column of the dqveldctrl matrix
            for(int j = 0; j < dof; j++){
                dqveldctrl(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsControls);

            }

//            cout << "vel inc " << velocityInc << endl;
//            cout << "vel dec " << velocityDec << endl;

            if(fd_costDerivs){
                dcostdctrl(i, 0) = (costInc - costDec)/(2*epsControls);
            }

            // Undo pertubation
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
//            cout << "reset simulator state - " << i << endl;
        }
    }

    return dqveldctrl;
}

MatrixXd Differentiator::calc_dqaccdctrl(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdctrl, bool fd_costDerivs, bool terminal){
    int numCtrl = activeModelTranslator->num_ctrl;
    int dof = activeModelTranslator->dof;
    MatrixXd acellInc(dof, 1);
    MatrixXd acellDec(dof, 1);
    MatrixXd dqaccdctrl(dof, numCtrl);
    double costInc;
    double costDec;

    int tid = -4 - physicsHelperId;
    mjtNum* warmstart = mj_stackAlloc(activePhysicsSimulator->fd_data[tid].get(), dof);
    mju_copy(warmstart, activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, dof);

    MatrixXd unperturbedControls = activeModelTranslator->ReturnControlVector(physicsHelperId);
    for(int i = 0; i < numCtrl; i++){
        bool computeColumn = false;
        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                computeColumn = true;
            }
        }

        if(computeColumn){
            // perturb control vector positively
            MatrixXd perturbedControls = unperturbedControls.replicate(1,1);
            perturbedControls(i) += epsControls;
            activeModelTranslator->SetControlVector(perturbedControls, physicsHelperId);

            // Integrate the simulator
            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, dof);
//            active_physics_simulator->forwardSimulator(physicsHelperId);
            activePhysicsSimulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_VEL, 0);
            acellInc = activeModelTranslator->returnAccelerationVector(physicsHelperId);

            // If calculating cost derivatives
            if(fd_costDerivs){
                // Calculate cost
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // return data state back to initial data state
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            // perturb control vector in opposite direction
            perturbedControls = unperturbedControls.replicate(1, 1);
            perturbedControls(i) -= epsControls;

            // integrate simulator
//            mju_copy(active_physics_simulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, dof);
//            active_physics_simulator->forwardSimulator(physicsHelperId);
            activePhysicsSimulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_VEL, 0);
            acellDec = activeModelTranslator->returnAccelerationVector(physicsHelperId);

            // If calculating cost derivatives via finite-differencing
            if(fd_costDerivs){
                // Calculate cost
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // Calculate one column of the dqveldctrl matrix
            for(int j = 0; j < dof; j++){
                dqaccdctrl(j, i) = (acellInc(j) - acellDec(j))/(2*epsControls);

            }

            if(fd_costDerivs){
                dcostdctrl(i, 0) = (costInc - costDec)/(2*epsControls);
            }

            // Undo pertubation
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
        }
    }

    return dqaccdctrl;
}

MatrixXd Differentiator::calc_dqveldqvel(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal){
    int dof = activeModelTranslator->dof;
    MatrixXd velocityInc(dof, 1);
    MatrixXd velocityDec(dof, 1);
    double costInc;
    double costDec;
    MatrixXd dqveldqvel(dof, dof);
    MatrixXd unperturbedVelocities = activeModelTranslator->returnVelocityVector(physicsHelperId);
//    cout << "unperturbed velocities - " << unperturbedVelocities << endl;

    for(int i = 0; i < dof; i++){
        bool computeColumn = false;

        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                computeColumn = true;
            }
        }

        if(computeColumn){
            // Perturb velocity vector positively
            MatrixXd perturbedVelocities = unperturbedVelocities.replicate(1, 1);
            perturbedVelocities(i) += epsVelocities;
            activeModelTranslator->setVelocityVector(perturbedVelocities, physicsHelperId);

            // Integrate the simulator
//            mju_copy(active_physics_simulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);
//            cout << "after first step simulator velocity - " << i << endl;


            // return the new velocity vector
            velocityInc = activeModelTranslator->returnVelocityVector(physicsHelperId);
//            cout << "velocityInc - " << velocityInc << endl;

            // If calculating cost derivs via finite-differencing
            if(fd_costDerivs){
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // reset the data state back to initial data state
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            // perturb velocity vector negatively
            perturbedVelocities = unperturbedVelocities.replicate(1, 1);
            perturbedVelocities(i) -= epsVelocities;
            activeModelTranslator->setVelocityVector(perturbedVelocities, physicsHelperId);

            // Integrate the simulatormodel
//            mju_copy(active_physics_simulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);
//            cout << "after 2nd step simulator - " << i << endl;

            // Return the new velocity vector
            velocityDec = activeModelTranslator->returnVelocityVector(physicsHelperId);
//            cout << "velocityDec - " << velocityDec << endl;


            // If calculating cost derivs via finite-differencing
            if(fd_costDerivs){
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // Calculate one column of the dqveldqvel matrix
            for(int j = 0; j < dof; j++){
                dqveldqvel(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsVelocities);
            }

            if(fd_costDerivs){
                dcostdvel(i, 0) = (costInc - costDec)/(2*epsVelocities);
            }

            // Undo perturbation
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
        }
    }
    return dqveldqvel;
}

MatrixXd Differentiator::calc_dqaccdqvel(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdvel, bool fd_costDerivs, bool terminal){
    int dof = activeModelTranslator->dof;
    MatrixXd acellInc(dof, 1);
    MatrixXd acellDec(dof, 1);
    double costInc;
    double costDec;
    MatrixXd dqaccdvel(dof, dof);
    MatrixXd unperturbedVelocities = activeModelTranslator->returnVelocityVector(physicsHelperId);

    int tid = -4 - physicsHelperId;
    mjtNum* warmstart = mj_stackAlloc(activePhysicsSimulator->fd_data[tid].get(), dof);
    mju_copy(warmstart, activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, dof);

    for(int i = 0; i < dof; i++){
        bool computeColumn = false;

        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                computeColumn = true;
            }
        }

        if(computeColumn){
            // Perturb velocity vector positively
            MatrixXd perturbedVelocities = unperturbedVelocities.replicate(1, 1);
            perturbedVelocities(i) += epsVelocities;
            activeModelTranslator->setVelocityVector(perturbedVelocities, physicsHelperId);

            //Integrate the simulator
            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, dof);
//            active_physics_simulator->forwardSimulator(physicsHelperId);
            activePhysicsSimulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_POS, 0);
            acellInc = activeModelTranslator->returnAccelerationVector(physicsHelperId);

            // If calculating cost derivs via finite-differencing
            if(fd_costDerivs){
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // reset the data state back to initial data state
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            // perturb velocity vector negatively
            perturbedVelocities = unperturbedVelocities.replicate(1, 1);
            perturbedVelocities(i) -= epsVelocities;
            activeModelTranslator->setVelocityVector(perturbedVelocities, physicsHelperId);

            // Integrate the simulator
            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, dof);
//            active_physics_simulator->forwardSimulator(physicsHelperId);
            activePhysicsSimulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_POS, 0);
            acellDec = activeModelTranslator->returnAccelerationVector(physicsHelperId);

            // If calculating cost derivs via finite-differencing
            if(fd_costDerivs){
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // Calculate one column of the dqveldqvel matrix
            for(int j = 0; j < dof; j++){
                dqaccdvel(j, i) = (acellInc(j) - acellDec(j))/(2*epsVelocities);
            }

            if(fd_costDerivs){
                dcostdvel(i, 0) = (costInc - costDec)/(2*epsVelocities);
            }

            // Undo perturbation
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
        }
    }
    return dqaccdvel;
}

MatrixXd Differentiator::calc_dqveldqpos(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal){
    MatrixXd unperturbedPositions = activeModelTranslator->returnPositionVector(physicsHelperId);
    int dof = activeModelTranslator->dof;
    MatrixXd velocityInc(dof, 1);
    MatrixXd velocityDec(dof, 1);
    double costInc;
    double costDec;
    MatrixXd dqveldq(dof, dof);

    for(int i = 0; i < dof; i++){
        bool computeColumn = false;
        for(int j = 0; j < cols.size(); j++) {
            if (i == cols[j]) {
                computeColumn = true;
            }
        }

        if(computeColumn){
            // Perturb position vector positively
            MatrixXd perturbedPositions = unperturbedPositions.replicate(1, 1);
            perturbedPositions(i) += epsPositions;
            activeModelTranslator->setPositionVector(perturbedPositions, physicsHelperId);
//                cout << "Perturbed positions positive " << perturbedPositions << endl;cout << "Perturbed positions positive " << perturbedPositions << endl;

            // Integrate the simulator
//                mju_copy(active_physics_simulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);
//                cout << "after step simulator positon - " << i << endl;

            // return the new velocity vector
            velocityInc = activeModelTranslator->returnVelocityVector(physicsHelperId);
//                cout << "Velocity positive " << velocityInc << endl;

            if(fd_costDerivs){
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // reset the data state back to initial data statedataIndex
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            // perturb position vector negatively
            perturbedPositions = unperturbedPositions.replicate(1, 1);
            perturbedPositions(i) -= epsPositions;
            activeModelTranslator->setPositionVector(perturbedPositions, physicsHelperId);

            // Integrate the simulator
//                mju_copy(active_physics_simulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);

            // Return the new velocity vector
            velocityDec = activeModelTranslator->returnVelocityVector(physicsHelperId);
//                cout << "velocity dec " << velocityDec << endl;

            if(fd_costDerivs){
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // Calculate one column of the dqaccdq matrix
            for(int j = 0; j < dof; j++){
                dqveldq(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsPositions);
            }

            if(fd_costDerivs){
                dcostdpos(i, 0) = (costInc - costDec)/(2*epsPositions);
            }

            // Undo perturbation
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
        }
    }
    return dqveldq;
}

MatrixXd Differentiator::calc_dqaccdqpos(std::vector<int> cols, int dataIndex, int physicsHelperId, MatrixXd &dcostdpos, bool fd_costDerivs, bool terminal){
    int dof = activeModelTranslator->dof;
    MatrixXd unperturbedPositions = activeModelTranslator->returnPositionVector(physicsHelperId);
    MatrixXd accellInc(dof, 1);
    MatrixXd accellDec(dof, 1);
    double costInc;
    double costDec;
    MatrixXd dqaccdq(dof, dof);

    int tid = -4 - physicsHelperId;
    mjtNum* warmstart = mj_stackAlloc(activePhysicsSimulator->fd_data[tid].get(), dof);
    mju_copy(warmstart, activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, dof);

    for(int i = 0; i < dof; i++){

        bool computeColumn = false;

        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                computeColumn = true;
            }
        }

        if(computeColumn){
            // Perturb position vector positively
            MatrixXd perturbedPositions = unperturbedPositions.replicate(1, 1);
            perturbedPositions(i) += epsPositions;
            activeModelTranslator->setPositionVector(perturbedPositions, physicsHelperId);

            // Integrate the simulator
            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, dof);
//            active_physics_simulator->forwardSimulator(physicsHelperId);
            activePhysicsSimulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_NONE, 0);
            accellInc = activeModelTranslator->returnAccelerationVector(physicsHelperId);

            if(fd_costDerivs){
                costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            //Reset data state
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            //Perturb position vector negatively
            perturbedPositions = unperturbedPositions.replicate(1, 1);
            perturbedPositions(i) -= epsPositions;
            activeModelTranslator->setPositionVector(perturbedPositions, physicsHelperId);

            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, dof);
//            active_physics_simulator->forwardSimulator(physicsHelperId);
            activePhysicsSimulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_NONE, 0);
            accellDec = activeModelTranslator->returnAccelerationVector(physicsHelperId);

            if(fd_costDerivs){
                costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
            }

            // Calculate one column of the dqaccdq matrix
            for(int j = 0; j < dof; j++){
                dqaccdq(j, i) = (accellInc(j) - accellDec(j))/(2*epsPositions);
            }

            if(fd_costDerivs){
                dcostdpos(i, 0) = (costInc - costDec)/(2*epsPositions);
            }

            // Undo perturbation
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

        }
    }
    return dqaccdq;
}

#pragma GCC pop_options