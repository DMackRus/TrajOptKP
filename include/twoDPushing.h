#ifndef TWOD_PUSHING_H
#define TWOD_PUSHING_H

#include "modelTranslator.h"

class twoDPushing: public modelTranslator{
public:
    twoDPushing();

    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState() override;
    std::vector<MatrixXd> createInitOptimisationControls(int horizonLength) override;
    void initControls_mainWayPoints_optimisation(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);
    std::vector<MatrixXd> createInitSetupControls(int horizonLength) override;
    void initControls_mainWayPoints_setup(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);


    std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming);
    std::vector<MatrixXd> generate_initControls_fromWayPoints(std::vector<m_point> initPath);

};


#endif