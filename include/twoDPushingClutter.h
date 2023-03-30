//
// Created by davidrussell on 3/28/23.
//

#ifndef PHYSICSSIMSWITCHING_TWODPUSHINGCLUTTER_H
#define PHYSICSSIMSWITCHING_TWODPUSHINGCLUTTER_H

#include "modelTranslator.h"

class twoDPushingClutter: public modelTranslator{
public:
    twoDPushingClutter();

    MatrixXd returnRandomStartState() override;
    MatrixXd returnRandomGoalState() override;
    std::vector<MatrixXd> createInitOptimisationControls(int horizonLength) override;

    void initControls_mainWayPoints(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);
    std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming);
    std::vector<MatrixXd> generate_initControls_fromWayPoints(std::vector<m_point> initPath);

};

#endif //PHYSICSSIMSWITCHING_TWODPUSHINGCLUTTER_H
