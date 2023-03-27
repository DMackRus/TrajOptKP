#include "differentiator.h"

differentiator::differentiator(modelTranslator *_modelTranslator, MuJoCoHelper *_physicsSimulator){
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;

    char error[1000];

    const char* fileName = activeModelTranslator->modelFilePath.c_str();
    cout << "filename diff " << fileName << endl;

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

void differentiator::getDerivatives(MatrixXd &A, MatrixXd &B, bool costDerivs, int dataIndex){
    double epsControls = 1e-5;
    double epsVelocities = 1e-5;
    double epsPositions = 1e-5;


    int dof = activeModelTranslator->dof;
    int numCtrl = activeModelTranslator->num_ctrl;

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

    // A.resize(2*dof, 2*dof);
    // B.resize(2*dof, numCtrl);

    // create a copy of the data ------  TO DO - FIX MUJOCO SPECIFNESS
    mjData *saveData;
    saveData = mj_makeData(m);
    activePhysicsSimulator->cpMjData(m, saveData, activePhysicsSimulator->savedSystemStatesList[dataIndex]);

    // Allocate memory for variables
    mjtNum* warmstart = mj_stackAlloc(saveData, dof);

//    cout << "accel before: " << saveData->qacc[0] << endl;
//    // Compute mj_forward once with no skips
    mj_forward(m, saveData);
//    cout << "accel before: " << saveData->qacc[0] << endl;

    // Compute mj_forward a few times to allow optimiser to get a more accurate value for qacc
    // skips position and velocity stages (TODO LOOK INTO IF THIS IS NEEDED FOR MY METHOD)
    for( int rep=1; rep<5; rep++ )
        mj_forwardSkip(m, saveData, mjSTAGE_VEL, 1);

    // save output for center point and warmstart (needed in forward only)
    mju_copy(warmstart, saveData->qacc_warmstart, dof);

    

    // cout << "address of data to be linearised:" << (void *)activePhysicsSimulator->savedSystemStatesList[dataIndex] << endl;
    // cout << "address of save data object:" << (void *)saveData << endl;
    // cout << "address of main data:" << (void *)activePhysicsSimulator->mdata << endl;

    MatrixXd unperturbedControls = activeModelTranslator->returnControlVector(dataIndex);

    // Calculate dqveldctrl
    for(int i = 0; i < numCtrl; i++){
        // perturb control vector positively
        MatrixXd perturbedControls = unperturbedControls.replicate(1,1);
        perturbedControls(i) += epsControls;
        activeModelTranslator->setControlVector(perturbedControls, dataIndex);

        // Integrate the simulator
        mju_copy(activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, warmstart, m->nv);
        activePhysicsSimulator->stepSimulator(1, dataIndex);

        // return the  new velcoity vector
        velocityInc = activeModelTranslator->returnVelocityVector(dataIndex);

        // return data state back to initial data state
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

        // perturb control vector in opposite direction
        perturbedControls = unperturbedControls.replicate(1, 1);
        perturbedControls(i) -= epsControls;

        // integrate simulator
        mju_copy(activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, warmstart, m->nv);
        activePhysicsSimulator->stepSimulator(1, dataIndex);

        // return the new velocity vector
        velocityDec = activeModelTranslator->returnVelocityVector(dataIndex);

        // Calculate one column of the dqveldctrl matrix
        for(int j = 0; j < dof; j++){
            dqveldctrl(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsControls);
        }

        // Undo pertubation
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

    }

    MatrixXd unperturbedVelocities = activeModelTranslator->returnVelocityVector(dataIndex);

    // Calculate dqveldqvel
    for(int i = 0; i < dof; i++){
        // Perturb velocity vector positively
        MatrixXd perturbedVelocities = unperturbedVelocities.replicate(1, 1);
        perturbedVelocities(i) += epsVelocities;
        activeModelTranslator->setVelocityVector(perturbedVelocities, dataIndex);

        // Integrate the simulator
        mju_copy(activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, warmstart, m->nv);
        activePhysicsSimulator->stepSimulator(1, dataIndex);

        // return the new velocity vector
        velocityInc = activeModelTranslator->returnVelocityVector(dataIndex);

        // reset the data state back to initial data state
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

        // perturb velocity vector negatively
        perturbedVelocities = unperturbedVelocities.replicate(1, 1);
        perturbedVelocities(i) -= epsVelocities;
        activeModelTranslator->setVelocityVector(perturbedVelocities, dataIndex);

        // Integrate the simulatormodel
        mju_copy(activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, warmstart, m->nv);
        activePhysicsSimulator->stepSimulator(1, dataIndex);

        // Return the new velocity vector
        velocityDec = activeModelTranslator->returnVelocityVector(dataIndex);

        // Calculate one column of the dqveldqvel matrix
        for(int j = 0; j < dof; j++){
            dqveldqvel(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsVelocities);
        }

        // Undo perturbation
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

    }

    MatrixXd unperturbedPositions = activeModelTranslator->returnPositionVector(dataIndex);

    // Calculate dqaccdq
    for(int i = 0; i < dof; i++){
        // Perturb position vector positively
        MatrixXd perturbedPositions = unperturbedPositions.replicate(1, 1);
        perturbedPositions(i) += epsPositions;
        activeModelTranslator->setPositionVector(perturbedPositions, dataIndex);

        // Integrate the simulator
        mju_copy(activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, warmstart, m->nv);
        mj_forwardSkip(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], mjSTAGE_NONE, 1);

        // return the new velocity vector
        accellInc = activeModelTranslator->returnAccelerationVector(dataIndex);

        // reset the data state back to initial data state
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

        // perturb position vector negatively
        perturbedPositions = unperturbedPositions.replicate(1, 1);
        perturbedPositions(i) -= epsPositions;
        activeModelTranslator->setPositionVector(perturbedPositions, dataIndex);

        // Integrate the simulator
        mju_copy(activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, warmstart, m->nv);
        mj_forwardSkip(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], mjSTAGE_NONE, 1);

        // Return the new velocity vector
        accellDec = activeModelTranslator->returnAccelerationVector(dataIndex);

        // Calculate one column of the dqaccdq matrix
        for(int j = 0; j < dof; j++){
            dqaccdq(j, i) = (accellInc(j) - accellDec(j))/(2*epsPositions);
        }

        // Undo perturbation
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);
    }

    // Calculate dqveldq
    // for(int i = 0; i < dof; i++){
    //     // Perturb position vector positively
    //     MatrixXd perturbedPositions = unperturbedPositions.replicate(1, 1);
    //     perturbedPositions(i) += epsPositions;
    //     activeModelTranslator->setPositionVector(perturbedPositions, dataIndex);

    //     // Integrate the simulator
    //     mju_copy(activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, warmstart, m->nv);
    //     activePhysicsSimulator->stepSimulator(1, dataIndex);

    //     // return the new velocity vector
    //     velocityInc = activeModelTranslator->returnVelocityVector(dataIndex);

    //     // reset the data state back to initial data state
    //     activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

    //     // perturb position vector negatively
    //     perturbedPositions = unperturbedPositions.replicate(1, 1);
    //     perturbedPositions(i) -= epsPositions;
    //     activeModelTranslator->setPositionVector(perturbedPositions, dataIndex);

    //     // Integrate the simulator
    //     mju_copy(activePhysicsSimulator->savedSystemStatesList[dataIndex]->qacc_warmstart, warmstart, m->nv);
    //     activePhysicsSimulator->stepSimulator(1, dataIndex);

    //     // Return the new velocity vector
    //     velocityDec = activeModelTranslator->returnVelocityVector(dataIndex);

    //     // Calculate one column of the dqaccdq matrix
    //     for(int j = 0; j < dof; j++){
    //         dqveldq(j, i) = (velocityInc(j) - velocityDec(j))/(2*epsPositions);
    //     }

    //     // Undo perturbation
    //     activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);
    // }

    // Delete temporary data object to prevent memory leak
    mj_deleteData(saveData);

    // need to set this as zero when reaching but not when pendulum????
    //dqaccdq.setZero();



    // ------------ A -----------------
    // dqposdqpos       dqposdqvel
    //
    // dqveldqpos       dqveldqvel
    // --------------------------------
    A.block(0, 0, dof, dof).setIdentity();
    // A.block(0, dof, dof, dof) = dqposdqvel;
    A.block(0, dof, dof, dof).setIdentity();
    A.block(0, dof, dof, dof) *= MUJOCO_DT;

    A.block(dof, dof, dof, dof) = dqveldqvel;

    //A.block(dof, 0, dof, dof) = dqveldq;
    A.block(dof, 0, dof, dof) = (dqaccdq * MUJOCO_DT);


    // ------------- B -------------------
    //          dqposdctrl
    //          dqveldctrl
    // ----------------------------------

    B.block(0, 0, dof, numCtrl).setZero();
    B.block(dof, 0, dof, numCtrl) = dqveldctrl;
}