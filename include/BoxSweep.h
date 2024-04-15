#pragma once

#include "ModelTranslator.h"
#include "PushBaseClass.h"

class BoxSweep: virtual public ModelTranslator, public PushBaseClass{
public:
    BoxSweep();

//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;

    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;

    bool TaskComplete(mjData *d, double &dist) override;

private:
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;

};