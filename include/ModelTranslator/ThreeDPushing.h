#pragma once

#include "ModelTranslator/ModelTranslator.h"
#include "ModelTranslator/PushBaseClass.h"

class ThreeDPushing: virtual public ModelTranslator, public PushBaseClass{
public:
    ThreeDPushing();

//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;
//    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
//    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;
//
//    bool TaskComplete(mjData *d, double &dist) override;

private:
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;

};
