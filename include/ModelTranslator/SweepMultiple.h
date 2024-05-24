#pragma once

#include "ModelTranslator/ModelTranslator.h"
#include "ModelTranslator/PushBaseClass.h"

class SweepMultiple: virtual public ModelTranslator, public PushBaseClass{
public:
    SweepMultiple();

    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;

    bool TaskComplete(mjData *d, double &dist) override;

private:
    int clutterLevel = noClutter;
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;
};
