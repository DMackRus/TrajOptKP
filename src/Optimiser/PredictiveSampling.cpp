#include "Optimiser/PredictiveSampling.h"

PredictiveSampling::PredictiveSampling(std::shared_ptr<ModelTranslator> _modelTranslator,
                                       std::shared_ptr<MuJoCoHelper> MuJoCo_helper,
                                       std::shared_ptr<FileHandler> _yamlReader,
                                       std::shared_ptr<Differentiator> _differentiator,
                                       int _maxHorizon, int _rolloutsPerIter):
                                       Optimiser(_modelTranslator, MuJoCo_helper,
                                                 _yamlReader, _differentiator){
    maxHorizon = _maxHorizon;
    rolloutsPerIter = _rolloutsPerIter;

//    double tempNoiseProfile[6] = {2.0, 1.0, 0.1, 2.0, 1.0, 0.1};
    double tempNoiseProfile[2] = {1.0, 1.0};


    noiseProfile.resize(num_ctrl, 1);
    for(int i = 0; i < num_ctrl; i++){
        noiseProfile(i) = tempNoiseProfile[i] / 2;
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
        MuJoCo_helper->AppendSystemStateToEnd(MuJoCo_helper->main_data);
    }
}

double PredictiveSampling::RolloutTrajectory(mjData *d, bool saveStates, std::vector<MatrixXd> initControls){
    double cost = 0.0f;
//
//    if(initialDataIndex != MAIN_DATA_STATE){
//        active_physics_simulator->copySystemState(initialDataIndex, 0);
//    }
    MuJoCo_helper->CopySystemState(d, MuJoCo_helper->main_data);

//    MatrixXd testStart = activeModelTranslator->ReturnStateVector(initialDataIndex);
//    cout << "init state: " << testStart << endl;

    MatrixXd Xt(activeModelTranslator->state_vector_size, 1);
    MatrixXd X_last(activeModelTranslator->state_vector_size, 1);
    MatrixXd Ut(activeModelTranslator->num_ctrl, 1);
    MatrixXd U_last(activeModelTranslator->num_ctrl, 1);

    Xt = activeModelTranslator->ReturnStateVector(d);
//    cout << "X_start:" << Xt << endl;

    for(int i = 0; i < horizon_length; i++){
        // set controls
        activeModelTranslator->SetControlVector(initControls[i], d);

        // Integrate simulator
        mj_step(MuJoCo_helper->model, d);

        // return cost for this state
        Xt = activeModelTranslator->ReturnStateVector(d);
        Ut = activeModelTranslator->ReturnControlVector(d);
        double stateCost;
        
        if(i == initControls.size() - 1){
            stateCost = activeModelTranslator->CostFunction(d, true);
        }
        else{
            stateCost = activeModelTranslator->CostFunction(d, false);
        }

        cost += (stateCost * MuJoCo_helper->ReturnModelTimeStep());

    }

    return cost;
}

std::vector<MatrixXd> PredictiveSampling::Optimise(mjData *d, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength){

    std::vector<MatrixXd> optimisedControls;
    double bestCost;
    horizon_length = _horizonLength;

    for(int i = 0; i < initControls.size(); i++){
        U_best[i] = initControls[i];
    }
    MuJoCo_helper->CopySystemState(MuJoCo_helper->main_data, 0);
    MatrixXd testStart = activeModelTranslator->ReturnStateVector(0);
    bestCost = RolloutTrajectory(MuJoCo_helper->saved_systems_state_list[0], true, initControls);
//    active_physics_simulator->copySystemState(0, MAIN_DATA_STATE);
    cout << "cost of initial trajectory: " << bestCost << endl;
    cout << "min iter: " << minIter << endl;

    for(int i = 0; i < maxIter; i++) {
        double costs[rolloutsPerIter];

        #pragma omp parallel for
        for (int j = 0; j < rolloutsPerIter; j++) {
            U_noisy[j] = createNoisyTrajec(U_best);
            costs[j] = RolloutTrajectory(MuJoCo_helper->saved_systems_state_list[j], false, U_noisy[j]);
        }

        double bestCostThisIter = costs[0];
        int bestCostIndex = 0;

        for(int j = 0; j < rolloutsPerIter; j++){
            if(costs[j] < bestCostThisIter){
                bestCostThisIter = costs[j];
                bestCostIndex = j;
            }
        }

        bool converged = CheckForConvergence(bestCost, bestCostThisIter);

//        cout << "best cost this iteration: " << bestCostThisIter << endl;

        // reupdate U_best with new best trajectory if a better trajectory found
        if(bestCostThisIter < bestCost){
            cout << "new trajectory cost: " << bestCostThisIter << " at iteration: " << i << endl;
            bestCost = bestCostThisIter;
            for(int j = 0; j < horizon_length; j++) {
                U_best[j] = U_noisy[bestCostIndex][j];
            }
        }

        if(converged && (i >= minIter)){
            cout << "converged early at iteration: " << i << endl;
            break;
        }
    }

    for(int i = 0; i < horizon_length; i++){
        optimisedControls.push_back(U_best[i]);
    }
    return optimisedControls;
}

MatrixXd PredictiveSampling::returnNoisyControl(MatrixXd Ut, MatrixXd noise){
    MatrixXd noisyControl(num_ctrl, 1);
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);

    for(int i = 0; i < Ut.size(); i++){
        std::normal_distribution<double> dist(0.0, noise(i));
        double noiseVal = dist(generator);
        noisyControl(i) = Ut(i) + noiseVal;

        if(noisyControl(i) > 1.0f){
            noisyControl(i) = 1.0f;
        }
        else if(noisyControl(i) < -1.0f){
            noisyControl(i) = -1.0f;
        }
    }

    return noisyControl;
}

std::vector<MatrixXd> PredictiveSampling::createNoisyTrajec(std::vector<MatrixXd> nominalTrajectory){
    std::vector<MatrixXd> noisyTrajec;

    for(int i = 0; i < horizon_length; i++){
        MatrixXd newControl = returnNoisyControl(nominalTrajectory[i], noiseProfile);
        noisyTrajec.push_back(newControl);
    }

    return noisyTrajec;
}