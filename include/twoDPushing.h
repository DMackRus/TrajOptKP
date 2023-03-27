#ifndef TWOD_PUSHING_H
#define TWOD_PUSHING_H

#include "modelTranslator.h"

class twoDPushing: public modelTranslator{
public:
    twoDPushing();

    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState() override;
    std::vector<MatrixXd> createInitControls(int horizonLength) override;

};


#endif