#pragma once
#include "ModelTranslator.h"

class Acrobot : public ModelTranslator {
public:
    Acrobot();

    bool TaskComplete(mjData *d, double &dist) override;

    void Residuals(mjData *d, MatrixXd &residuals) override;

//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;

};