#pragma once

#include "ModelTranslator.h"

class BoxSweep: public ModelTranslator{
public:
    BoxSweep();

    void GenerateRandomGoalAndStartState() override;
    MatrixXd ReturnRandomStartState() override;
    MatrixXd ReturnRandomGoalState(MatrixXd X0) override;
    std::vector<MatrixXd> CreateInitOptimisationControls(int horizonLength) override;
    void initControls_mainWayPoints_optimisation(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);
    std::vector<MatrixXd> CreateInitSetupControls(int horizonLength) override;
    void initControls_mainWayPoints_setup(m_point desiredObjectEnd, std::vector<m_point>& mainWayPoints, std::vector<int>& wayPointsTiming, int horizon);


    std::vector<m_point> initControls_createAllWayPoints(std::vector<m_point> mainWayPoints, std::vector<int> wayPointsTiming);
    std::vector<MatrixXd> generate_initControls_fromWayPoints(std::vector<m_point> initPath);

    bool TaskComplete(int dataIndex, double &dist) override;

private:
    double randomGoalX = 0.0;
    double randomGoalY = 0.0;

};