//
// Created by dave on 09/03/23.
//

#ifndef PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
#define PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H

#include "ModelTranslator.h"

class DoublePendulum : public ModelTranslator {
public:
    DoublePendulum();

    bool TaskComplete(mjData *d, double &dist) override;
    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;

};

#endif //PHYSICSSIMSWITCHING_DOUBLEPENDULUM_H
