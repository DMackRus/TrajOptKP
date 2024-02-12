#ifndef TWOD_PUSHING_H
#define TWOD_PUSHING_H

#include "ModelTranslator.h"

class TwoDPushing: public ModelTranslator{
public:
    TwoDPushing(int clutterLevel);

    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
    void initControls_mainWayPoints_optimisation(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);
    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;
    void initControls_mainWayPoints_setup(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);


    std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming);
    std::vector<MatrixXd> generate_initControls_fromWayPoints(std::vector<m_point> initPath);


    double CostFunction(int data_index, bool terminal) override;

    void CostDerivatives(int data_index, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

    bool TaskComplete(int dataIndex, double &dist) override;

private:
    int clutterLevel = noClutter;
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;

    Matrix<double, 6, 6> cost_reach;

};


#endif