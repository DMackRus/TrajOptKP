//
// Created by dave on 09/03/23.
//

#ifndef PHYSICSSIMSWITCHING_REACHING_H
#define PHYSICSSIMSWITCHING_REACHING_H

#include "ModelTranslator.h"

class pandaReaching : public ModelTranslator {
public:
    pandaReaching();

    bool TaskComplete(mjData *d, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;

    double jointLimsMax[7] = {2.2, 0.6, 2.97, 0, 2.97, 1.5, 2.5};
    double jointLimsMin[7] = {-2.2, -0.6, -2.97, -1.5, -2.97, -1.1, -2.5};

};

#endif //PHYSICSSIMSWITCHING_REACHING_H
