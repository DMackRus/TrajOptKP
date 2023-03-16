#include "differentiator.h"

differentiator::differentiator(modelTranslator *_modelTranslator, MuJoCoHelper *_physicsSimulator){
    activeModelTranslator = _modelTranslator;
    activePhysicsSimulator = _physicsSimulator;
}

differentiator::getDerivatives(MatrixXd &A, MatrixXd &B, bool costDerivs, mjData *d, mjModel *m){
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


    // create a copy of the data
    mjData *d saveData;
    saveData = mj_makeModel(m);
    activePhysicsSimulator->cpMjData(model, saveData, d);

    // Calculate dqveldctrl
    for(int i = 0; i < numCtrl; i++){
        // perturb control vector positively
        saveData->ctrl[i] = d->ctrl[i] + epsControls;
        
        // Integrate the simulator
        activePhysicsSimulator->stepSimulator(1);

        // return the  new velcoity vector

        // return data state back to initial data state

        // perturb control vector in opposite direction

        // integrate simulator

        // return the new velocity vector

        // Calculate one column of the dqveldctrl matrix
        for(int j = 0; j < dof, j++){

        }

        // Undo pertubation

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

    B.block(dof, 0, dof, numCtrl) = dqveldctrl;



}