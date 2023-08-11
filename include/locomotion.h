//
// Created by davidrussell on 7/21/23.
//

#ifndef AUTOMATICTOTASKSPECIFICATION_LOCOMOTION_H
#define AUTOMATICTOTASKSPECIFICATION_LOCOMOTION_H

#include "modelTranslator.h"

class locomotion_anymal : public modelTranslator {
public:
    locomotion_anymal();

//    double costFunction(int dataIndex, bool terminal) override;
//    void costDerivatives(int dataIndex, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

    bool taskComplete(int dataIndex, double &dist) override;
    void generateRandomGoalAndStartState() override;
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState(MatrixXd X0) override;

    std::vector<MatrixXd> createInitOptimisationControls(int horizonLength) override;

};

#endif //AUTOMATICTOTASKSPECIFICATION_LOCOMOTION_H
