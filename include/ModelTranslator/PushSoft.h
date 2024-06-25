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

    void ReturnRandomStartState() override;
    void ReturnRandomGoalState() override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
//    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;

//    double CostFunction(mjData *d, bool terminal) override;

//    void CostDerivatives(mjData *d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

    bool TaskComplete(mjData *d, double &dist) override;

private:
    int task_mode = PUSH_SOFT;
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;

};