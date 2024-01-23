#ifndef INTERPOLATED_ILQR_H
#define INTERPOLATED_ILQR_H

#include "Optimiser.h"
#include "Differentiator.h"
#include "Visualiser.h"
#include "FileHandler.h"
#include <algorithm>

class interpolatediLQR: public Optimiser{
public:
    interpolatediLQR(std::shared_ptr<ModelTranslator> _modelTranslator, std::shared_ptr<PhysicsSimulator> _physicsSimulator, std::shared_ptr<Differentiator> _differentiator, int _maxHorizon, std::shared_ptr<Visualiser> _visualizer, std::shared_ptr<FileHandler> _yamlReader);

    double RolloutTrajectory(int initialDataIndex, bool saveStates, std::vector<MatrixXd> initControls) override;
    std::vector<MatrixXd> Optimise(int initialDataIndex, std::vector<MatrixXd> initControls, int maxIter, int minIter, int _horizonLength) override;

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

    std::shared_ptr<Visualiser> activeVisualizer;

    vector<vector<MatrixXd>> U_alpha;

};


#endif