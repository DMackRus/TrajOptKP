#ifndef INTERPOLATED_ILQR_H
#define INTERPOLATED_ILQR_H

#include "optimiser.h"
#include "differentiator.h"
#include "visualizer.h"
#include "fileHandler.h"
#include <algorithm>

class interpolatediLQR: public optimiser{
public:
    interpolatediLQR(modelTranslator *_modelTranslator, physicsSimulator *_physicsSimulator, differentiator *_differentiator, int _maxHorizon, visualizer *_visualizer, fileHandler *_yamlReader);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) override;

    bool backwardsPass_Quu_reg();
    bool backwardsPass_Quu_reg_parallel();
    bool backwardsPass_Quu_skips();
    bool isMatrixPD(Ref<MatrixXd> matrix);

    double forwardsPass(double oldCost, bool &costReduced);
    double forwardsPassParallel(double oldCost, bool &costReduced);


    std::vector<double> costHistory;
    int numIters = 0;

    bool saveTrajecInfomation = false;
    bool saveCostHistory = false;

private:
    double lambda = 0.1;
    double maxLambda = 10000;
    double minLambda = 0.00001;
    double lambdaFactor = 10;
    int maxHorizon = 0;

    // Feedback gains matrices
    vector<MatrixXd> k;
    vector<MatrixXd> K;

    visualizer *activeVisualizer;

    vector<vector<MatrixXd>> U_alpha;

};


#endif