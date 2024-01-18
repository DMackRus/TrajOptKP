#ifndef BOX_FLICK_H
#define BOX_FLICK_H

#include "ModelTranslator.h"

class BoxFlick: public ModelTranslator{
public:
    BoxFlick(int _clutterLevel);

    double CostFunction(int dataIndex, bool terminal) override;
    void CostDerivatives(int dataIndex, MatrixXd &l_x, MatrixXd &l_xx, MatrixXd &l_u, MatrixXd &l_uu, bool terminal) override;

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
    int clutterLevel;

    double boxStartX = 0.5;
    double boxStartY = 0.1;

    double A = 1;
    double sigma = 0.005;

};


#endif