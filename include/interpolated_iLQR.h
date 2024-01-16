#ifndef INTERPOLATED_ILQR_H
#define INTERPOLATED_ILQR_H

#include "optimiser.h"
#include "differentiator.h"
#include "visualizer.h"
#include "fileHandler.h"
#include <algorithm>

class interpolatediLQR: public optimiser{
public:
    interpolatediLQR(std::shared_ptr<modelTranslator> _modelTranslator, std::shared_ptr<physicsSimulator> _physicsSimulator, std::shared_ptr<differentiator> _differentiator, int _maxHorizon, std::shared_ptr<visualizer> _visualizer, std::shared_ptr<fileHandler> _yamlReader);

    double rolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) override;

    bool backwardsPass_Quu_reg();
    bool isMatrixPD(Ref<MatrixXd> matrix);

    double forwardsPass(double oldCost);
    double forwardsPassParallel(double oldCost);

    bool saveTrajecInfomation = false;

private:
    double lambda = 0.1;
    double maxLambda = 10.0;
    double minLambda = 0.01;
    double lambdaFactor = 10;
    int maxHorizon = 0;

    // Feedback gains matrices
    vector<MatrixXd> k;
    vector<MatrixXd> K;

    std::shared_ptr<visualizer> activeVisualizer;

    vector<vector<MatrixXd>> U_alpha;

};


#endif