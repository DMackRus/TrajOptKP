#include "Differentiator.h"

Differentiator::Differentiator(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> _MuJoCo_helper){
    activeModelTranslator = _modelTranslator;
    MuJoCo_helper = _MuJoCo_helper;

    dof = activeModelTranslator->dof;
    num_ctrl = activeModelTranslator->num_ctrl;
    dim_state = 2 * dof;
}

void Differentiator::ComputeDerivatives(MatrixXd &A, MatrixXd &B, const std::vector<int> &cols,
                                        MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu,
                                        int data_index, int thread_id, bool terminal, bool cost_derivs,
                                        bool central_diff, double eps){

    // Reset some debugging timing variables
    time_mj_forwards = 0.0f;
    count_integrations = 0;
    auto start = std::chrono::high_resolution_clock::now();

    auto diff_start = std::chrono::high_resolution_clock::now();

    // Get the thread Id
    int tid = thread_id;

    // Memory allocation for next states, whether perturbed or not
    MatrixXd next_state(dim_state, 1);
    MatrixXd next_state_plus(dim_state, 1);
    MatrixXd next_state_minus(dim_state, 1);

    // Memory allocation for current state and controls
    MatrixXd unperturbed_controls(num_ctrl, 1);
    MatrixXd unperturbed_positions(dof, 1);
    MatrixXd unperturbed_velocities(dof, 1);

    // Memory allocation for perturbed state and controls
    MatrixXd perturbed_controls(num_ctrl, 1);
    MatrixXd perturbed_positions(dof, 1);
    MatrixXd perturbed_velocities(dof, 1);

    // Initialise sub matrices of A and B matrix
    // ------------ dof x dof ----------------
    MatrixXd dstatedqpos(dim_state, dof);
    MatrixXd dstatedqvel(dim_state, dof);

//    MatrixXd dqveldqpos(dim_state, dof);
//    MatrixXd dqveldqvel(dim_state, dof);

    // ------------ dof x ctrl --------------
    MatrixXd dstatedctrl(dim_state, num_ctrl);
//    MatrixXd dqveldctrl(dof, num_ctrl);

    // How the cost changes
    MatrixXd dcostdctrl(num_ctrl, 1);
    MatrixXd dcostdpos(dof, 1);
    MatrixXd dcostdvel(dof, 1);

    double costInc = 0.0f;
    double costDec = 0.0f;

    // Copy data we wish to finite-difference into finite differencing data (for multi threading)
    MuJoCo_helper->cpMjData(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[data_index]);

    // TODO (DMackRus) We could in theory use the information from the rollout here instead, except for t = T - 1
    // Compute next state with no perturbations
    mj_step(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]);
    next_state = activeModelTranslator->ReturnStateVector(MuJoCo_helper->fd_data[tid]);

    // Reset the simulator to the initial state
    MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[data_index]);

    unperturbed_controls = activeModelTranslator->ReturnControlVector(MuJoCo_helper->fd_data[tid]);
    unperturbed_positions = activeModelTranslator->returnPositionVector(MuJoCo_helper->fd_data[tid]);
    unperturbed_velocities = activeModelTranslator->returnVelocityVector(MuJoCo_helper->fd_data[tid]);

    // --------------------------------------------- FD for controls ---------------------------------------------
    MatrixXd control_limits = activeModelTranslator->ReturnControlLimits();
    for(int i = 0; i < num_ctrl; i++){
        bool compute_column = false;
        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                compute_column = true;
            }
        }

        // Skip this column if it is not in the list of columns to compute
        if(!compute_column){
            continue;
        }

        // perturb control vector positively
        perturbed_controls = unperturbed_controls.replicate(1,1);
        perturbed_controls(i) += eps;

        // Check if the perturbed control is within the control limits
        int nudge_forward = 1;
        if(perturbed_controls(i) > control_limits(2*i + 1)){
            nudge_forward = 0;
        }

        if(nudge_forward){
            // count how many calls to step_skip we make
            count_integrations++;

            // Set perturbed control vector
            activeModelTranslator->SetControlVector(perturbed_controls, MuJoCo_helper->fd_data[tid]);

            // Integrate the simulator
            start = std::chrono::high_resolution_clock::now();
            mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_VEL, 1);
            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

            // return the new state vector
            next_state_plus = activeModelTranslator->ReturnStateVector(MuJoCo_helper->fd_data[tid]);

            // If computing cost derivatives
            if(cost_derivs){
                costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
            }

            // Undo the perturbation
            MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[data_index]);
        }

        int nudge_back;

        // perturb control vector in opposite direction
        perturbed_controls = unperturbed_controls.replicate(1, 1);
        perturbed_controls(i) -= eps;

        // If we use central difference or we didnt nudge forward, due to control limits
        if(central_diff || !nudge_forward){
            nudge_back = 1;
            if(perturbed_controls(i) < control_limits(2*i)){
                nudge_back = 0;
            }
        } else {
            nudge_back = 0;
        }

        if(nudge_back){
            activeModelTranslator->SetControlVector(perturbed_controls, MuJoCo_helper->fd_data[tid]);

            // integrate simulator
            start = std::chrono::high_resolution_clock::now();
            mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_VEL, 1);
            time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

            // return the new state vector
            next_state_minus = activeModelTranslator->ReturnStateVector(MuJoCo_helper->fd_data[tid]);

            // If calculating cost derivatives via finite-differencing
            if(cost_derivs){
                costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
            }

            // Undo perturbation
            MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[data_index]);
        }

        // Compute finite differences, depending on what perturbations were made
        if(nudge_forward && nudge_back){
            for(int j = 0; j < dim_state; j++){
                dstatedctrl(j, i) = (next_state_plus(j) - next_state_minus(j))/(2*eps);
            }

            if(cost_derivs){
                dcostdctrl(i, 0) = (costInc - costDec)/(2*eps);
            }
        }
        else if(nudge_forward){
            for(int j = 0; j < dim_state; j++){
                dstatedctrl(j, i) = (next_state_plus(j) - next_state(j))/(eps);
            }

            // TODO(DMackRus) this is wrong
            if(cost_derivs){
                dcostdctrl(i, 0) = (costInc - costDec)/(2*eps);
            }
        }
        else if(nudge_back){
            for(int j = 0; j < dim_state; j++){
                dstatedctrl(j, i) = (next_state(j) - next_state_minus(j))/(eps);
            }

            // TODO(DMackRus) this is wrong
            if(cost_derivs){
                dcostdctrl(i, 0) = (costInc - costDec)/(2*eps);
            }
        }
    }

    // ----------------------------------------------- FD for velocities ---------------------------------------------
    for(int i = 0; i < dof; i++){
        bool compute_column = false;

        for(int j = 0; j < cols.size(); j++){
            if(i == cols[j]){
                compute_column = true;
            }
        }

        // Skip this column if it is not in the list of columns to compute
        if(!compute_column){
            continue;
        }

        count_integrations++;
        // Perturb velocity vector positively
        perturbed_velocities = unperturbed_velocities.replicate(1, 1);
        perturbed_velocities(i) += eps;
        activeModelTranslator->setVelocityVector(perturbed_velocities, MuJoCo_helper->fd_data[tid]);

        // Integrate the simulator
        start = std::chrono::high_resolution_clock::now();
        mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_POS, 1);
        time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

        // return the new velocity vector
        next_state_plus = activeModelTranslator->ReturnStateVector(MuJoCo_helper->fd_data[tid]);

        // If calculating cost derivs via finite-differencing
        if(cost_derivs){
            costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
        }

        // reset the data state back to initial data state
        MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[data_index]);

        // perturb velocity vector negatively
        perturbed_velocities = unperturbed_velocities.replicate(1, 1);
        perturbed_velocities(i) -= eps;
        activeModelTranslator->setVelocityVector(perturbed_velocities, MuJoCo_helper->fd_data[tid]);

        // Integrate the simulator
        start = std::chrono::high_resolution_clock::now();
        mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_POS, 1);
        time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

        // Return the new velocity vector
        next_state_minus = activeModelTranslator->ReturnStateVector(MuJoCo_helper->fd_data[tid]);

        // If calculating cost derivs via finite-differencing
        if(cost_derivs){
            costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
        }

        // Calculate one column of the dqveldqvel matrix
        for(int j = 0; j < dim_state; j++){
            dstatedqvel(j, i) = (next_state_plus(j) - next_state_minus(j))/(2*eps);
        }

        if(cost_derivs){
            dcostdvel(i, 0) = (costInc - costDec)/(2*eps);
        }

        // Undo perturbation
        MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[data_index]);
    }

    // ----------------------------------------------- FD for positions ---------------------------------------------
    mj_markStack(MuJoCo_helper->fd_data[tid]);
    mjtNum *dpos  = mj_stackAllocNum(MuJoCo_helper->fd_data[tid], MuJoCo_helper->model->nv);
    for(int i = 0; i < dof; i++){
        bool compute_column = false;
        for(int j = 0; j < cols.size(); j++) {
            if (i == cols[j]) {
                compute_column = true;
            }
        }

        // Skip this column if it is not in the list of columns to compute
        if(!compute_column){
            continue;
        }

        // Compute the index of the position vector in MuJoCo that corresponds to the index of the state vector
        int dpos_index = activeModelTranslator->StateIndexToQposIndex(i);

        count_integrations++;
        // Perturb position vector positively

        mju_zero(dpos, MuJoCo_helper->model->nv);
        dpos[dpos_index] = 1;
        mj_integratePos(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]->qpos, dpos, eps);

        // Integrate the simulator
        start = std::chrono::high_resolution_clock::now();
        mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_NONE, 1);
        time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

        // return the new velocity vector
        next_state_plus = activeModelTranslator->ReturnStateVector(MuJoCo_helper->fd_data[tid]);

        if(cost_derivs){
            costInc = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
        }

        // reset the data state back to initial data statedataIndex
        MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[data_index]);

        // perturb position vector negatively
//        perturbed_positions = unperturbed_positions.replicate(1, 1);
//        perturbed_positions(i) -= eps;
//        activeModelTranslator->setPositionVector(perturbed_positions, MuJoCo_helper->fd_data[tid]);
        mju_zero(dpos, MuJoCo_helper->model->nv);
        dpos[dpos_index] = 1;
        mj_integratePos(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid]->qpos, dpos, -eps);

        // Integrate the simulator
        start = std::chrono::high_resolution_clock::now();
        mj_stepSkip(MuJoCo_helper->model, MuJoCo_helper->fd_data[tid], mjSTAGE_NONE, 1);
        time_mj_forwards += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();

        // Return the decremented vectors
        next_state_minus = activeModelTranslator->ReturnStateVector(MuJoCo_helper->fd_data[tid]);

        if(cost_derivs){
            costDec = activeModelTranslator->CostFunction(MuJoCo_helper->fd_data[tid], terminal);
        }

        // Calculate one column of the dqaccdq matrix
        for(int j = 0; j < dim_state; j++){
            dstatedqpos(j, i) = (next_state_plus(j) - next_state_minus(j))/(2*eps);
        }

        if(cost_derivs){
            dcostdpos(i, 0) = (costInc - costDec)/(2*eps);
        }

        // Undo perturbation
        MuJoCo_helper->copySystemState(MuJoCo_helper->fd_data[tid], MuJoCo_helper->savedSystemStatesList[data_index]);

    }

    mj_freeStack(MuJoCo_helper->fd_data[tid]);

    // ------------ A -----------------
    // dqposdqpos       dqposdqvel
    //
    // dqveldqpos       dqveldqvel
    // --------------------------------
    for(int i = 0; i < cols.size(); i++){
//        if(USE_DQACC){
//            A.block(dof, cols[i], dof, 1) = dqaccdqpos.block(0, cols[i], dof, 1) * MuJoCo_helper->returnModelTimeStep();
//            for(int j = 0; j < dof; j ++){
//                if(j == cols[i]){
//                    A(dof + j, cols[i] + dof) = 1 + (dqaccdqvel(j, cols[i]) * MuJoCo_helper->returnModelTimeStep());
//                }
//                else{
//                    A(dof + j, cols[i] + dof) = dqaccdqvel(j, cols[i]) * MuJoCo_helper->returnModelTimeStep();
//                }
//            }
//        }
//        else{
        A.block(0, cols[i], dim_state, 1) = dstatedqpos.block(0, cols[i], dim_state, 1);
//        A.block(0, cols[i] + dof, dof, 1) = dqposdqvel.block(0, cols[i], dof, 1);

        A.block(0, cols[i] + dof, dim_state, 1) = dstatedqvel.block(0, cols[i], dim_state, 1);
//        A.block(dof, cols[i] + dof, dof, 1) = dqveldqvel.block(0, cols[i], dof, 1);
//        }

    }

    // ------------- B -------------------
    //          dqposdctrl
    //          dqveldctrl
    // ----------------------------------
    for(int i = 0; i < cols.size(); i++){
        if(cols[i] < num_ctrl){
//            if(USE_DQACC){
//                B.block(dof, cols[i], dof, 1) = dqaccdctrl.block(0, cols[i], dof, 1) * MuJoCo_helper->returnModelTimeStep();
//            }
//            else{
            B.block(0, cols[i], dim_state, 1) = dstatedctrl.block(0, cols[i], dim_state, 1);

//            B.block(dof, cols[i], dof, 1) = dqveldctrl.block(0, cols[i], dof, 1);
//            }
        }
    }

//    std::cout << "time of sim integration: " << time_mj_forwards / 1000.0f << "\n";
//    std::cout << "num of sim integration: " << count_integrations << "\n";
//    std::cout << "diff time: "  << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - diff_start).count() / 1000.0 << std::endl;
}

// finite difference cost derivatives graveyard
//if(costDerivs) {
//l_x.block(0, 0, dof, 1) = dcostdpos;
//l_x.block(dof, 0, dof, 1) = dcostdvel;
//
//// ------------- l_u / l_uu -------------------
////                dqcostdctrl
//// --------------------------------------------
//l_u.block(0, 0, numCtrl, 1) = dcostdctrl;
//}
//
//MatrixXd l_x_inc(dof*2, 1);
//MatrixXd l_x_dec(dof*2, 1);
//MatrixXd l_u_inc(numCtrl, 1);
//MatrixXd l_u_dec(numCtrl, 1);
//
//MatrixXd currentState = activeModelTranslator->ReturnStateVector(physicsHelperId);
//MatrixXd currentControl = activeModelTranslator->ReturnControlVector(physicsHelperId);
//
//if(costDerivs && !HESSIAN_APPROXIMATION) {
//double epsCost = 1e-6;
//double epsTest = 1e-6;
//
//MatrixXd test1;
//MatrixXd test2;
//
//for(int i = 0; i < 2*dof; i++){
//MatrixXd perturbedState = currentState.replicate(1, 1);
//perturbedState(i) += epsCost;
//
//for(int j = 0; j < 2*dof; j++){
////                cout << "perturb state index " << i << " with state index " << j << endl;
//MatrixXd stateInc = perturbedState.replicate(1,1);
//stateInc(j) += epsTest;
////                cout << "state inc pos: " << endl << stateInc << endl;
//
//activeModelTranslator->SetStateVector(stateInc, physicsHelperId);
////                active_physics_simulator->forwardSimulator(physicsHelperId);
////                active_physics_simulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_NONE, 0);
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//test1 = activeModelTranslator->ReturnStateVector(physicsHelperId);
//costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//MatrixXd stateDec = perturbedState.replicate(1,1);
//stateDec(j) -= epsTest;
////                cout << "state dec: " << endl << stateDec << endl;
//
//activeModelTranslator->SetStateVector(stateDec, physicsHelperId);
////                active_physics_simulator->forwardSimulator(physicsHelperId);
////                active_physics_simulator->forwardSimulatorWithSkip(physicsHelperId, mjSTAGE_NONE, 0);
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//test2 = activeModelTranslator->ReturnStateVector(physicsHelperId);
//costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
////                cout << "difference in returned states: " << endl << test1 - test2 << endl;
////                cout << "cost inc: " << costInc << endl;
////                cout << "cost dec: " << costDec << endl;
//
//l_x_inc(j, 0) = (costInc - costDec)/(2*epsTest);
//}
//
////            cout << "perturbed state pos: " << endl << perturbedState << endl;
////            cout << "l_x_inc: " << endl << l_x_inc << endl;
//
//perturbedState = currentState.replicate(1, 1);
//perturbedState(i) -= epsCost;
//
//for(int j = 0; j < 2*dof; j++){
//MatrixXd stateInc = perturbedState.replicate(1,1);
//stateInc(j) += epsTest;
//
//activeModelTranslator->SetStateVector(stateInc, physicsHelperId);
////                active_physics_simulator->forwardSimulator(physicsHelperId);
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//MatrixXd stateDec = perturbedState.replicate(1,1);
//stateDec(j) -= epsTest;
//
//activeModelTranslator->SetStateVector(stateDec, physicsHelperId);
////                active_physics_simulator->forwardSimulator(physicsHelperId);
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//l_x_dec(j, 0) = (costInc - costDec)/(2*epsTest);
//}
//
////            cout << "perturbed state neg: " << endl << perturbedState << endl;
////            cout << "l_x_dec: " << endl << l_x_dec << endl;
//
//// New l_x at perturbed position i
//for(int j = 0; j < 2*dof; j++){
//l_xx(j, i) = (l_x_inc(j, 0) - l_x_dec(j, 0))/(2*epsCost);
//}
//}
//
//for(int i = 0; i < numCtrl; i++){
//MatrixXd perturbedControl = currentControl.replicate(1, 1);
//perturbedControl(i) += epsCost;
//
//for(int j = 0; j < numCtrl; j++){
//MatrixXd controlInc = perturbedControl.replicate(1,1);
//controlInc(j) += epsCost;
//
//activeModelTranslator->SetControlVector(controlInc, physicsHelperId);
//
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//MatrixXd controlDec = perturbedControl.replicate(1,1);
//controlDec(j) -= epsCost;
//
//activeModelTranslator->SetControlVector(controlDec, physicsHelperId);
//
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//l_u_inc(j, 0) = (costInc - costDec)/(2*epsCost);
//}
//
//perturbedControl = currentControl.replicate(1, 1);
//perturbedControl(i) -= epsCost;
//
//for(int j = 0; j < numCtrl; j++){
//MatrixXd controlInc = perturbedControl.replicate(1,1);
//controlInc(j) += epsCost;
//
//activeModelTranslator->SetControlVector(controlInc, physicsHelperId);
//
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costInc = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//MatrixXd controlDec = perturbedControl.replicate(1,1);
//controlDec(j) -= epsCost;
//
//activeModelTranslator->SetControlVector(controlDec, physicsHelperId);
//
//MuJoCo_helper->stepSimulator(1, physicsHelperId);
//MuJoCo_helper->forwardSimulator(physicsHelperId);
//costDec = activeModelTranslator->CostFunction(physicsHelperId, terminal);
//MuJoCo_helper->copySystemState(physicsHelperId, dataIndex);
//
//l_u_dec(j, 0) = (costInc - costDec)/(2*epsCost);
//}
//
//for(int j = 0; j < numCtrl; j++){
//l_uu(j, i) = (l_u_inc(j, 0) - l_u_dec(j, 0))/(2*epsCost);
//}
//}
//}
////    cout << "l_x: " << endl << l_x << endl;
////    cout << "l_xx: " << endl << l_xx << endl;
//
//if(costDerivs && HESSIAN_APPROXIMATION){
//l_xx = l_x * l_x.transpose();
//l_uu = l_u * l_u.transpose();
//}