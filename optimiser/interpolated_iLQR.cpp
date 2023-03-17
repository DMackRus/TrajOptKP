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

std::vector<MatrixXd> interpolatediLQR::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIterations, int horizonLength){
    std::vector<MatrixXd> optimisedControls;



    return optimisedControls;
}