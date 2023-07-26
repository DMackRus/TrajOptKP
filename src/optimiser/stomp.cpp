#include "stomp.h"

stomp::stomp(std::shared_ptr<modelTranslator> _modelTranslator, std::shared_ptr<physicsSimulator> _physicsSimulator, std::shared_ptr<fileHandler> _yamlReader, std::shared_ptr<differentiator> _differentiator, int _maxHorizon, int _rolloutsPerIter): optimiser(_modelTranslator, _physicsSimulator, _yamlReader, _differentiator){
    maxHorizon = _maxHorizon;
    rolloutsPerIter = _rolloutsPerIter;

    noiseProfile.resize(num_ctrl, 1);
    for(int i = 0; i < num_ctrl; i++){
        noiseProfile(i) = 0.5f;
    }

    for(int i = 0; i < maxHorizon; i++){
        std::vector<MatrixXd> U_temp;
        for(int j = 0; j < rolloutsPerIter; j++){
            U_temp.push_back(MatrixXd(num_ctrl, 1));
        }

        U_noisy.push_back(U_temp);
        
        U_best.push_back(MatrixXd(num_ctrl, 1));
    }

    for(int i = 0; i < rolloutsPerIter; i++){
        activePhysicsSimulator->appendSystemStateToEnd(MAIN_DATA_STATE);
    }
}

double stomp::rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls){
    double cost = 0.0f;
//
//    if(initialDataIndex != MAIN_DATA_STATE){
//        activePhysicsSimulator->copySystemState(initialDataIndex, 0);
//    }
    activePhysicsSimulator->copySystemState(initialDataIndex, MAIN_DATA_STATE);

//    MatrixXd testStart = activeModelTranslator->returnStateVector(initialDataIndex);
//    cout << "init state: " << testStart << endl;

    MatrixXd Xt(activeModelTranslator->stateVectorSize, 1);
    MatrixXd X_last(activeModelTranslator->stateVectorSize, 1);
    MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
    MatrixXd U_last(activeModelTranslator->num_ctrl, 1);

    Xt = activeModelTranslator->returnStateVector(initialDataIndex);
//    cout << "X_start:" << Xt << endl;

    for(int i = 0; i < horizonLength; i++){
        // set controls
        activeModelTranslator->setControlVector(initControls[i], initialDataIndex);

        // Integrate simulator
        activePhysicsSimulator->stepSimulator(1, initialDataIndex);

        // return cost for this state
        Xt = activeModelTranslator->returnStateVector(initialDataIndex);
        Ut = activeModelTranslator->returnControlVector(initialDataIndex);
        double stateCost;
        
        if(i == initControls.size() - 1){
            stateCost = activeModelTranslator->costFunction(initialDataIndex, true);
        }
        else{
            stateCost = activeModelTranslator->costFunction(initialDataIndex, false);
        }

        cost += (stateCost * MUJOCO_DT);

    }

    return cost;
}

std::vector<MatrixXd> stomp::optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength){

    std::vector<MatrixXd> optimisedControls;
    double bestCost;
    horizonLength = _horizonLength;

    for(int i = 0; i < initControls.size(); i++){
        U_best[i] = initControls[i];
    }
    activePhysicsSimulator->copySystemState(MAIN_DATA_STATE, 0);
    MatrixXd testStart = activeModelTranslator->returnStateVector(0);
    bestCost = rolloutTrajectory(0, true, initControls);
//    activePhysicsSimulator->copySystemState(0, MAIN_DATA_STATE);
    cout << "cost of initial trajectory: " << bestCost << endl;

    for(int i = 0; i < maxIter; i++) {
        double costs[rolloutsPerIter];

        #pragma omp parallel for
        for (int j = 0; j < rolloutsPerIter; j++) {
            U_noisy[j] = createNoisyTrajec(U_best);
            costs[j] = rolloutTrajectory(j, false, U_noisy[j]);
        }

        double bestCostThisIter = costs[0];
        int bestCostIndex = 0;

        for(int j = 0; j < rolloutsPerIter; j++){
            if(costs[j] < bestCostThisIter){
                bestCostThisIter = costs[j];
                bestCostIndex = j;
            }
        }

        bool converged = checkForConvergence(bestCost, bestCostThisIter);

//        cout << "best cost this iteration: " << bestCostThisIter << endl;

        // reupdate U_best with new best trajectory if a better trajectory found
        if(bestCostThisIter < bestCost){
            cout << "new trajectory cost: " << bestCostThisIter << " at iteration: " << i << endl;
            bestCost = bestCostThisIter;
            for(int j = 0; j < horizonLength; j++) {
                U_best[j] = U_noisy[bestCostIndex][j];
            }
        }

        if(converged && (i >= minIter)){
            cout << "converged early at iteration: " << i << endl;
            break;
        }
    }

    for(int i = 0; i < horizonLength; i++){
        optimisedControls.push_back(U_best[i]);
    }
    return optimisedControls;
}

MatrixXd stomp::returnNoisyControl(MatrixXd Ut, MatrixXd noise){
    MatrixXd noisyControl(num_ctrl, 1);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    for(int i = 0; i < Ut.size(); i++){
        std::normal_distribution<double> dist(0.0, noise(i));
        double noiseVal = dist(generator);
        noisyControl(i) = Ut(i) + noiseVal;
    }

    return noisyControl;
}

std::vector<MatrixXd> stomp::createNoisyTrajec(std::vector<MatrixXd> nominalTrajectory){
    std::vector<MatrixXd> noisyTrajec;

    for(int i = 0; i < horizonLength; i++){
        MatrixXd newControl = returnNoisyControl(nominalTrajectory[i], noiseProfile);
        noisyTrajec.push_back(newControl);
    }

    return noisyTrajec;
}