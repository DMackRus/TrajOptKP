//
// Created by davidrussell on 4/4/23.
//

#ifndef AUTOTOTASK_GRADDESCENT_H
#define AUTOTOTASK_GRADDESCENT_H

#include "Optimiser.h"
#include "ModelTranslator/ModelTranslator.h"
#include "MuJoCoHelper.h"
#include "Visualiser.h"
#include "Differentiator.h"

class GradDescent: public Optimiser{
public:
    GradDescent(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<MuJoCoHelper> MuJoCo_helper, std::shared_ptr<Differentiator> _differentiator, std::shared_ptr<Visualiser> _visualizer, int _maxHorizon, std::shared_ptr<FileHandler> _yamlReader);

    double RolloutTrajectory(mjData *d, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> Optimise(mjData *d, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) override;

    void backwardsPass();

    double forwardsPass(double oldCost, bool &costReduced);
    double forwardsPassParallel(double oldCost, bool &costReduced);


private:
    int intervalSize = 1;
    bool setIntervalMethod = true;
    int maxHorizon;

    // -------------- Vectors of matrices for gradient information about the trajectory -------------
    // First order dynamics
    vector<MatrixXd> f_x;
    vector<MatrixXd> f_u;
    vector<MatrixXd> A;
    vector<MatrixXd> B;

    // First order cost derivatives
    vector<MatrixXd> l_x;
    vector<MatrixXd> l_u;

    // Optimal control gradient information
    vector<MatrixXd> J_u;

    // Saved states and controls
    vector<MatrixXd> U_new;
    vector<MatrixXd> U_old;
    vector<MatrixXd> X_new;
    vector<MatrixXd> X_old;

//    Differentiator *activeDifferentiator;
    std::shared_ptr<Visualiser> activeVisualizer;

    vector<vector<MatrixXd>> U_alpha;

};

#endif //AUTOTOTASK_GRADDESCENT_H
