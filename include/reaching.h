//
// Created by dave on 09/03/23.
//

#ifndef PHYSICSSIMSWITCHING_REACHING_H
#define PHYSICSSIMSWITCHING_REACHING_H

#include "modelTranslator.h"

class pandaReaching : public modelTranslator {
public:
    pandaReaching();

    bool taskComplete(int dataIndex) override;
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState(MatrixXd X0) override;
    std::vector<MatrixXd> createInitOptimisationControls(int horizonLength) override;

    double jointLimsMax[7] = {2.2, 0.6, 2.97, 0, 2.97, 1.5, 2.5};
    double jointLimsMin[7] = {-2.2, -0.6, -2.97, -1.5, -2.97, -1.1, -2.5};

};

#endif //PHYSICSSIMSWITCHING_REACHING_H
