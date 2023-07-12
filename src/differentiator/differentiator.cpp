#include "differentiator.h"

#define USE_DQACC_DQ 0

differentiator::differentiator(modelTranslator *_modelTranslator, MuJoCoHelper *_physicsSimulator){
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;

    char error[1000];
    const char* fileName = activeModelTranslator->modelFilePath.c_str();

    m = mj_loadXML(fileName, NULL, NULL, 1000);

    if( !m ) {

        printf("%s\n", error);
    }
}

void differentiator::initModelForFiniteDifferencing(){
    save_iterations = m->opt.iterations;
    save_tolerance = m->opt.tolerance;
    m->opt.iterations = 30;
    m->opt.tolerance = 0;
}

void differentiator::resetModelAfterFiniteDifferencing(){
    m->opt.iterations = save_iterations;
    m->opt.tolerance = save_tolerance;

}

void differentiator::getDerivatives(MatrixXd &A, MatrixXd &B, std::vector<int> cols, MatrixXd &l_x, MatrixXd &l_u, MatrixXd &l_xx, MatrixXd &l_uu, bool costDerivs, int dataIndex, bool terminal){
    double epsControls = 1e-5;
    double epsVelocities = 1e-5;
    double epsPositions = 1e-5;

    int dof = activeModelTranslator->dof;
    int numCtrl = activeModelTranslator->num_ctrl;

    int tid = omp_get_thread_num();
    // This seems random, in optimiser we define -1 = "mainData", -2 = "masterData", 0 -> horizon Length = "stored trajectory data"
    // So we need values below -2 for finite-differencing data
    int physicsHelperId = -3 - tid;
//    cout << "Thread " << tid << " is doing this task" << endl;

    activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

    MatrixXd currentState = activeModelTranslator->returnStateVector(physicsHelperId);
    MatrixXd currentControls = activeModelTranslator->returnControlVector(physicsHelperId);
//    cout << "dataIndex: " << dataIndex << endl;
//    cout << "current state: " << endl << currentState << endl;

    // Initialise sub matrices of A and B matrix
    // ------------ dof x dof ----------------
    MatrixXd dqveldq(dof, dof);
    MatrixXd dqveldqvel(dof, dof);
    MatrixXd dqposdqvel(dof, dof);
    MatrixXd dqaccdqvel(dof, dof);
    MatrixXd dqaccdq(dof, dof);

    // ------------ dof x ctrl --------------
    MatrixXd dqveldctrl(dof, numCtrl);
    MatrixXd dqaccdctrl(dof, numCtrl);

    // ---------- dof x 1 -------------------
    MatrixXd velocityInc(dof, 1);
    MatrixXd velocityDec(dof, 1);
    MatrixXd positionInc(dof, 1);
    MatrixXd positionDec(dof, 1);
    MatrixXd accellInc(dof, 1);
    MatrixXd accellDec(dof, 1);

    // ---------- 1 x dof -------------------
    MatrixXd dqcostdctrl(numCtrl, 1);
    MatrixXd dqcostdq(dof, 1);
    MatrixXd dqcostdqvel(dof, 1);

    MatrixXd l_x_inc(dof*2, 1);
    MatrixXd l_x_dec(dof*2, 1);

    double costInc = 0.0f;
    double costDec = 0.0f;


    // Allocate memory for variables
//    mjtNum* warmstart = mj_stackAlloc(activePhysicsSimulator->savedSystemStatesList[dataIndex].get(), dof);
//    mju_copy(warmstart, activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, dof);



//    // Compute mj_forward once with no skips
//    mj_forward(m, activePhysicsSimulator->savedSystemStatesList[dataIndex].get());

    // Compute mj_forward a few times to allow optimiser to get a more accurate value for qacc
    // skips position and velocity stages (TODO LOOK INTO IF THIS IS NEEDED FOR MY METHOD)

//    for( int rep=1; rep<5; rep++ )
//        mj_forwardSkip(m, activePhysicsSimulator->savedSystemStatesList[dataIndex].get(), mjSTAGE_VEL, 1);

    // save output for center point and warmstart (needed in forward only)


    MatrixXd unperturbedControls = activeModelTranslator->returnControlVector(physicsHelperId);

    // Calculate dqveldctrl
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
            activeModelTranslator->setControlVector(perturbedControls, physicsHelperId);

            // Integrate the simulator
//            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);

            // return the  new velcoity vector
            velocityInc = activeModelTranslator->returnVelocityVector(physicsHelperId);

            // If calculating cost derivatives
            if(costDerivs){
                // Calculate cost
                costInc = activeModelTranslator->costFunction(currentState, perturbedControls, currentState, perturbedControls, terminal);
            }

            // return data state back to initial data state
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            // perturb control vector in opposite direction
            perturbedControls = unperturbedControls.replicate(1, 1);
            perturbedControls(i) -= epsControls;

            // integrate simulator
//            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);

            // return the new velocity vector
            velocityDec = activeModelTranslator->returnVelocityVector(physicsHelperId);

            // If calculating cost derivatives via finite-differencing
            if(costDerivs){
                // Calculate cost
                costDec = activeModelTranslator->costFunction(currentState, perturbedControls, currentState, perturbedControls, terminal);
            }

            // Calculate one column of the dqveldctrl matrix
            for(int j = 0; j < dof; j++){
                dqveldctrl(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsControls);

            }

            if(costDerivs){
                dqcostdctrl(i, 0) = (costInc - costDec)/(2*epsControls);
            }

            // Undo pertubation
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
        }
    }

    MatrixXd unperturbedVelocities = activeModelTranslator->returnVelocityVector(physicsHelperId);

    // Calculate dqveldqvel
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
//            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);

            // return the new velocity vector
            velocityInc = activeModelTranslator->returnVelocityVector(physicsHelperId);

            // If calculating cost derivs via finite-differencing
            if(costDerivs){
                // Calculate cost
                MatrixXd perturbedState = currentState.replicate(1, 1);
                perturbedState(i + dof) = perturbedVelocities(i);
                costInc = activeModelTranslator->costFunction(perturbedState, currentControls, perturbedState, currentControls, terminal);
            }

            // reset the data state back to initial data state
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            // perturb velocity vector negatively
            perturbedVelocities = unperturbedVelocities.replicate(1, 1);
            perturbedVelocities(i) -= epsVelocities;
            activeModelTranslator->setVelocityVector(perturbedVelocities, physicsHelperId);

            // Integrate the simulatormodel
//            mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
            activePhysicsSimulator->stepSimulator(1, physicsHelperId);

            // Return the new velocity vector
            velocityDec = activeModelTranslator->returnVelocityVector(physicsHelperId);

            // If calculating cost derivs via finite-differencing
            if(costDerivs){
                // Calculate cost
                MatrixXd perturbedState = currentState.replicate(1, 1);
                perturbedState(i + dof) = perturbedVelocities(i);
                costDec = activeModelTranslator->costFunction(perturbedState, currentControls, perturbedState, currentControls, terminal);
            }

            // Calculate one column of the dqveldqvel matrix
            for(int j = 0; j < dof; j++){
                dqveldqvel(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsVelocities);
            }

            if(costDerivs){
                dqcostdqvel(i, 0) = (costInc - costDec)/(2*epsVelocities);
            }

            // Undo perturbation
            activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
        }
    }

    MatrixXd unperturbedPositions = activeModelTranslator->returnPositionVector(physicsHelperId);

    // Calculate dqaccdq
    if(USE_DQACC_DQ){
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
                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                accellInc = activeModelTranslator->returnAccelerationVector(physicsHelperId);

                //Reset data state
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                //Perturb position vector negatively
                perturbedPositions = unperturbedPositions.replicate(1, 1);
                perturbedPositions(i) -= epsPositions;
                activeModelTranslator->setPositionVector(perturbedPositions, physicsHelperId);

                activePhysicsSimulator->forwardSimulator(physicsHelperId);
                accellDec = activeModelTranslator->returnAccelerationVector(physicsHelperId);

                // Calculate one column of the dqaccdq matrix
                for(int j = 0; j < dof; j++){
                    dqaccdq(j, i) = (accellInc(j) - accellDec(j))/(2*epsPositions);
                }

                // Undo perturbation
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

            }
        }
    }
    else{
        // Calculate dqveldq
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
//                mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
                activePhysicsSimulator->stepSimulator(1, physicsHelperId);

                // return the new velocity vector
                velocityInc = activeModelTranslator->returnVelocityVector(physicsHelperId);
//                cout << "Velocity positive " << velocityInc << endl;

                if(costDerivs){
                    // Calculate cost
                    MatrixXd perturbedState = currentState.replicate(1, 1);
                    perturbedState(i) = perturbedPositions(i);
                    costInc = activeModelTranslator->costFunction(perturbedState, currentControls, perturbedState, currentControls, terminal);
                }

                // reset the data state back to initial data statedataIndex
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);

                // perturb position vector negatively
                perturbedPositions = unperturbedPositions.replicate(1, 1);
                perturbedPositions(i) -= epsPositions;
                activeModelTranslator->setPositionVector(perturbedPositions, physicsHelperId);

                // Integrate the simulator
//                mju_copy(activePhysicsSimulator->fd_data[tid]->qacc_warmstart, warmstart, m->nv);
                activePhysicsSimulator->stepSimulator(1, physicsHelperId);

                // Return the new velocity vector
                velocityDec = activeModelTranslator->returnVelocityVector(physicsHelperId);
//                cout << "velocity dec " << velocityDec << endl;

                if(costDerivs){
                    // Calculate cost
                    MatrixXd perturbedState = currentState.replicate(1, 1);
                    perturbedState(i) = perturbedPositions(i);
                    costDec = activeModelTranslator->costFunction(perturbedState, currentControls, perturbedState, currentControls, terminal);
                }

                // Calculate one column of the dqaccdq matrix
                for(int j = 0; j < dof; j++){
                    dqveldq(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsPositions);
                }

                if(costDerivs){
                    dqcostdq(i, 0) = (costInc - costDec)/(2*epsPositions);
                }

                // Undo perturbation
                activePhysicsSimulator->copySystemState(physicsHelperId, dataIndex);
            }
        }
    }

    if(costDerivs) {
        l_x.block(0, 0, dof, 1) = dqcostdq;
        l_x.block(dof, 0, dof, 1) = dqcostdqvel;


//    l_xx.block(0, 0, dof, dof) = dqcostdq;

        // ------------- l_u / l_uu -------------------
        //                dqcostdctrl
        // --------------------------------------------

        l_u.block(0, 0, numCtrl, 1) = dqcostdctrl;

    }

    if(costDerivs) {
        double epsCost = 1e-1;

        for(int i = 0; i < 2*dof; i++){
            MatrixXd perturbedState = currentState.replicate(1, 1);
            perturbedState(i) += epsCost;

            for(int j = 0; j < 2*dof; j++){
                MatrixXd stateInc = perturbedState.replicate(1,1);
                stateInc(j) += epsPositions;

                costInc = activeModelTranslator->costFunction(stateInc, currentControls, stateInc, currentControls, terminal);

                MatrixXd stateDec = perturbedState.replicate(1,1);
                stateDec(j) -= epsPositions;

                costDec = activeModelTranslator->costFunction(stateDec, currentControls, stateDec, currentControls, terminal);

                l_x_inc(j, 0) = (costInc - costDec)/(2*epsPositions);
            }

            perturbedState = currentState.replicate(1, 1);
            perturbedState(i) -= epsCost;

            for(int j = 0; j < 2*dof; j++){
                MatrixXd stateInc = perturbedState.replicate(1,1);
                stateInc(j) += epsPositions;

                costInc = activeModelTranslator->costFunction(stateInc, currentControls, stateInc, currentControls, terminal);

                MatrixXd stateDec = perturbedState.replicate(1,1);
                stateDec(j) -= epsPositions;

                costDec = activeModelTranslator->costFunction(stateDec, currentControls, stateDec, currentControls, terminal);

                l_x_dec(j, 0) = (costInc - costDec)/(2*epsPositions);
            }

            // New l_x at perturbed position i

            for(int j = 0; j < 2*dof; j++){
                l_xx(j, i) = (l_x_inc(j, 0) - l_x_dec(j, 0))/(2*epsCost);
            }
        }
    }

    // Delete temporary data object to prevent memory leak
//    mj_deleteData(saveData);

    // need to set this as zero when reaching but not when pendulum????
    //dqaccdq.setZero();
    if(USE_DQACC_DQ){
        for(int i = 0; i < dof; i++){
            for(int j = 0; j < dof; j++){
                if(dqaccdq(i, j) > DQACCDQ_MAX){
                    dqaccdq(i, j) = DQACCDQ_MAX;
                }
                if(dqaccdq(i, j)< -DQACCDQ_MAX){
                    dqaccdq(i, j) = -DQACCDQ_MAX;
                }
            }
        }
    }
    else {
        for (int i = 0; i < dof; i++) {
            for (int j = 0; j < dof; j++) {
                if (dqveldq(i, j) > 2) {
                    dqveldq(i, j) = 2;
                }
                if (dqveldq(i, j) < -2) {
                    dqveldq(i, j) = -2;
                }
            }
        }
    }

    // ------------ A -----------------
    // dqposdqpos       dqposdqvel
    //
    // dqveldqpos       dqveldqvel
    // --------------------------------
//    A.block(0, 0, dof, dof).setIdentity();
//    // A.block(0, dof, dof, dof) = dqposdqvel;
//    A.block(0, dof, dof, dof).setIdentity();
//    A.block(0, dof, dof, dof) *= MUJOCO_DT;

    for(int i = 0; i < cols.size(); i++){
        A.block(dof, cols[i], dof, 1) = dqveldq.block(0, cols[i], dof, 1);
        if(USE_DQACC_DQ){
            A.block(dof, cols[i] + dof, dof, 1) = dqaccdq.block(0, cols[i], dof, 1) * MUJOCO_DT;
        }
        else{
            A.block(dof, cols[i] + dof, dof, 1) = dqveldqvel.block(0, cols[i], dof, 1);
        }
    }

//    cout << "A: " << endl << A << endl;

//    A.block(dof, dof, dof, dof) = dqveldqvel;
//    A.block(dof, 0, dof, dof) = (dqaccdq * MUJOCO_DT);


    // ------------- B -------------------
    //          dqposdctrl
    //          dqveldctrl
    // ----------------------------------

//    B.block(0, 0, dof, numCtrl).setZero();

    for(int i = 0; i < cols.size(); i++){
        if(cols[i] < numCtrl){
            B.block(dof, cols[i], dof, 1) = dqveldctrl.block(0, cols[i], dof, 1);
        }
    }
//    B.block(dof, 0, dof, numCtrl) = dqveldctrl;

    l_u.setZero();
    l_uu.setZero();

//    cout << "tid: " << tid  << " end" << endl;

}