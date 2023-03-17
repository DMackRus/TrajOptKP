#include "differentiator.h"

differentiator::differentiator(modelTranslator *_modelTranslator, MuJoCoHelper *_physicsSimulator){
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;

    // m = _physicsSimulator->model;
    char error[1000];

    
    const char* fileName = "/home/davidrussell/catkin_ws/src/physicsSimSwitching/Franka-emika-panda-arm/Acrobot.xml";
    m = mj_loadXML(fileName, NULL, NULL, 1000);

    if( !m ) {

        printf("%s\n", error);
    }
}

void differentiator::getDerivatives(MatrixXd &A, MatrixXd &B, bool costDerivs, int dataIndex){
    double epsControls = 1e-6;
    double epsVelocities = 1e-6;
    double epsPositions = 1e-6;


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

    A.resize(2*dof, 2*dof);
    B.resize(2*dof, numCtrl);

    // create a copy of the data ------  TO DO - FIX MUJOCO SPECIFNESS
    mjData *saveData;
    saveData = mj_makeData(m);

    // mjData *data_to_be_linearised;
    // if(dataIndex == MAIN_DATA_STATE){
    //     data_to_be_linearised = activePhysicsSimulator->mdata;
    // }
    // else{
    //     if(dataIndex >= activePhysicsSimulator->savedSystemStatesList.size()){
    //         cout << "data index outside of saved systems state idnex get derivs \n";
    //     }
        
    //     cout << "saved system state size: " << activePhysicsSimulator->savedSystemStatesList.size() << endl;
    //     data_to_be_linearised = activePhysicsSimulator->savedSystemStatesList[dataIndex];
    // }
    
    cout << "after assigning data to be linearised \n";

    activePhysicsSimulator->cpMjData(m, saveData, activePhysicsSimulator->savedSystemStatesList[dataIndex]);

    cout << "address of data to be linearised:" << (void *)activePhysicsSimulator->savedSystemStatesList[dataIndex] << endl;
    cout << "address of save data object:" << (void *)saveData << endl;
    cout << "address of main data:" << (void *)activePhysicsSimulator->mdata << endl;

    MatrixXd unperturbedControls = activeModelTranslator->returnControlVector(dataIndex);

    // Calculate dqveldctrl
    for(int i = 0; i < numCtrl; i++){
        // perturb control vector positively
        MatrixXd perturbedControls = unperturbedControls.replicate(1,1);
        perturbedControls(i) += epsControls;
        activeModelTranslator->setControlVector(perturbedControls, dataIndex);

        // Integrate the simulator
        activePhysicsSimulator->stepSimulator(1, dataIndex);

        // return the  new velcoity vector
        velocityInc = activeModelTranslator->returnVelocityVector(dataIndex);

        // return data state back to initial data state
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

        // perturb control vector in opposite direction
        perturbedControls = unperturbedControls.replicate(1, 1);
        perturbedControls(i) -= epsControls;

        // integrate simulator
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

    cout << "after dqveldctrl \n";
    MatrixXd unperturbedVelocities = activeModelTranslator->returnVelocityVector(dataIndex);

    // Calculate dqveldqvel
    for(int i = 0; i < dof; i++){
        // Perturb velocity vector positively
        MatrixXd perturbedVelocities = unperturbedVelocities.replicate(1, 1);
        perturbedVelocities(i) += epsVelocities;
        activeModelTranslator->setVelocityVector(perturbedVelocities, dataIndex);

        // Integrate the simulator
        activePhysicsSimulator->stepSimulator(1, dataIndex);

        // return the new velocity vector
        velocityInc = activeModelTranslator->returnVelocityVector(dataIndex);

        // reset the data state back to initial data state
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

        // perturb velocity vector negatively
        perturbedVelocities = unperturbedVelocities.replicate(1, 1);
        perturbedVelocities(i) -= epsVelocities;
        activeModelTranslator->setVelocityVector(perturbedVelocities, dataIndex);

        // Integrate the simulator
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
        //activePhysicsSimulator->stepSimulator(1, dataIndex);
        mj_forwardSkip(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], mjSTAGE_NONE, 1);

        // return the new velocity vector
        accellInc = activeModelTranslator->returnAccelerationVector(dataIndex);
        cout << "accellInc: " << accellInc << endl;

        // reset the data state back to initial data state
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

        // perturb position vector negatively
        perturbedPositions = unperturbedPositions.replicate(1, 1);
        perturbedPositions(i) -= epsPositions;
        activeModelTranslator->setPositionVector(perturbedPositions, dataIndex);

        // Integrate the simulator
        //activePhysicsSimulator->stepSimulator(1, dataIndex);
        mj_forwardSkip(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], mjSTAGE_NONE, 1);

        // Return the new velocity vector
        accellDec = activeModelTranslator->returnAccelerationVector(dataIndex);
        cout << "accelldec: " << accellDec << endl;

        // Calculate one column of the dqaccdq matrix
        for(int j = 0; j < dof; j++){
            dqaccdq(j, i) = (accellInc(j) - accellDec(j))/(2*epsPositions);
        }

        // Undo perturbation
        activePhysicsSimulator->cpMjData(m, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);
    }

    cout << "accell: " << dqaccdq << endl;

    // Delete temporary data object to prevent memory leak
    mj_deleteData(saveData);



    // ------------ A -----------------
    // dqposdqpos       dqposdqvel
    //
    // dqveldqpos       dqveldqvel
    // --------------------------------
    A.block(0, 0, dof, dof).setIdentity();
    // A.block(0, dof, dof, dof) = dqposdqvel;
    A.block(0, dof, dof, dof).setIdentity();
    A.block(0, dof, dof, dof) *= 0.004;

    A.block(dof, dof, dof, dof) = dqveldqvel;
    A.block(dof, 0, dof, dof) = (dqaccdq * 0.004);


    // ------------- B -------------------
    //          dqposdctrl
    //          dqveldctrl
    // ----------------------------------

    B.block(0, 0, dof, numCtrl).setZero();
    B.block(dof, 0, dof, numCtrl) = dqveldctrl;
}