#pragma once
#include "ModelTranslator.h"

class Acrobot : public ModelTranslator {
public:
    Acrobot();

    bool TaskComplete(mjData *d, double &dist) override;

    MatrixXd Residuals(mjData *d, const struct stateVectorList &state_vector) override;
//    double CostFunction(mjData* d, const struct stateVectorList &state_vector, bool terminal) override;

//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;

};