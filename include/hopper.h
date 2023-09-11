#pragma once

#include "modelTranslator.h"

class hopper: public modelTranslator {
    public:
    hopper();

    bool taskComplete(int dataIndex, double &dist) override;
    void generateRandomGoalAndStartState() override;
    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState(MatrixXd X0) override;

};
