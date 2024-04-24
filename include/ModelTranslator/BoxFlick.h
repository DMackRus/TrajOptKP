#ifndef BOX_FLICK_H
#define BOX_FLICK_H

#include "ModelTranslator.h"
#include "PushBaseClass.h"

class BoxFlick: virtual public ModelTranslator, public PushBaseClass{
public:
    BoxFlick(int _clutterLevel);

    double CostFunction(mjData *d, bool terminal) override;
    void CostDerivatives(mjData *d, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

//    void ReturnRandomStartState() override;
//    void ReturnRandomGoalState() override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;

    bool TaskComplete(mjData *d, double &dist) override;

private:
    int clutterLevel;

    double boxStartX = 0.5;
    double boxStartY = 0.1;

    double A = 1;
    double sigma = 0.005;

};


#endif