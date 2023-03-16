#include "differentiator.h"

differentiator::differentiator(modelTranslator *_modelTranslator, MuJoCoHelper *_physicsSimulator){
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;
}

differentiator::getDerivatives(MatrixXd &A, MatrixXd &B, bool costDerivs, mjModel *m, int dataIndex){
    double epsControls = 1e-6;
    double epsVelocities = 1e-6;
    double epsPositions = 1e-6;

    dof = activeModelTranslator->dof;
    numCtrl = activeModelTranslator->num_ctrl;

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


    // create a copy of the data ------  TO DO - FIX MUJOCO SPECIFNESS
    mjData *d saveData;
    saveData = mj_makeModel(m);

    mjData *data_to_be_linearised;
    data_to_be_linearised = activePhysicsSimulator->savedSystemStatesList[dataIndex];

    activePhysicsSimulator->cpMjData(model, saveData, data_to_be_linearised);

    vector<double> unperturbedControls = activeModelTranslator->returnControlVector(dataIndex);

    // Calculate dqveldctrl
    for(int i = 0; i < numCtrl; i++){
        // perturb control vector positively

        vector<double> perturbedControls = unperturbedControls;
        perturbedControls += epsControls;
        activeModelTranslator->setControlVector(perturbedControls, dataIndex);
        
        // Integrate the simulator
        activePhysicsSimulator->stepSimulator(1, dataIndex);
        
        // return the  new velcoity vector
        velocityInc = activeModelTranslator->returnVelocityVector(dataIndex);

        // return data state back to initial data state
        activePhysicsSimulator->cpMjData(model, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

        // perturb control vector in opposite direction
        perturbedControls = unperturbedControls;
        perturbedControls -= epsControls;

        // integrate simulator
        activePhysicsSimulator->stepSimulator(1, dataIndex);

        // return the new velocity vector
        velocityDec = activeModelTranslator->returnVelocityVector(dataIndex);

        // Calculate one column of the dqveldctrl matrix
        for(int j = 0; j < dof, j++){
            dqveldctrl(j, i) = (velInc(j) - velDec(j))/(2*epsControls);
        }

        // Undo pertubation
        activePhysicsSimulator->cpMjData(model, activePhysicsSimulator->savedSystemStatesList[dataIndex], saveData);

    }
    



    // ------------ A -----------------
    // dqposdqpos       dqposdqvel
    //
    // dqveldqpos       dqveldqvel
    // --------------------------------
    A.block(0, dof, dof, dof) = dqposdqvel;




    // ------------- B -------------------       
    //          dqposdctrl
    //          dqveldctrl  
    // ----------------------------------

    B.block(0, 0, dof, numCtrl).setZero();
    B.block(dof, 0, dof, numCtrl) = dqveldctrl;



}