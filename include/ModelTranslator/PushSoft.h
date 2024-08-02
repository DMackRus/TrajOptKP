#pragma once

#include "ModelTranslator/ModelTranslator.h"
#include "ModelTranslator/PushBaseClass.h"

enum task_mode{
    PUSH_SOFT,
    PUSH_SOFT_RIGID
};

class PushSoft: virtual public ModelTranslator, public PushBaseClass{
public:
    PushSoft(int _task_mode);
//
//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;
//    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
////    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;
//
//    bool TaskComplete(mjData *d, double &dist) override;

private:
    int task_mode = PUSH_SOFT;
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;

};