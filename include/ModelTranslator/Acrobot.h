#pragma once
#include "ModelTranslator.h"

class Acrobot : public ModelTranslator {
public:
    Acrobot();

    bool TaskComplete(mjData *d, double &dist) override;

    MatrixXd Residuals(mjData *d) override;

//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;

};