//
// Created by davidrussell on 4/4/23.
//

#ifndef AUTOTOTASK_GRADDESCENT_H
#define AUTOTOTASK_GRADDESCENT_H

#include "optimiser.h"
#include "modelTranslator.h"
#include "physicsSimulator.h"
#include "visualizer.h"
#include "differentiator.h"

class gradDescent: public optimiser{
public:
    gradDescent(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, differentiator *_differentiator, visualizer *_visualizer, int _maxHorizon, fileHandler _yamlReader);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) override;

    std::vector<int> generateEvalWaypoints(std::vector<MatrixXd> trajecStates, std::vector<MatrixXd> trajecControls);
    void getDerivativesAtSpecifiedIndices(std::vector<int> indices);
    void interpolateDerivatives(std::vector<int> calculatedIndices);

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

    differentiator *activeDifferentiator;
    visualizer *activeVisualizer;

    vector<vector<MatrixXd>> U_alpha;

};

#endif //AUTOTOTASK_GRADDESCENT_H
